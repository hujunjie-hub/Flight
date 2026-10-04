/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAG (BMM350) 数据环形缓冲区实现 (接口说明见 mag_data.h)
 *
 * 采集线程按 ODR 周期 (100Hz -> 10ms) 轮询 mag_bmm350: 每次 rt_device_read
 * 触发一次 I2C 读取。T_event 在读取触发时刻打戳 (timebase T_MCU 时基);
 * BMM350 INT 引脚 (PB5) 硬件已预留, 迁移到 EXTI 事件源后时戳改为
 * EXTI ISR 捕获 (缓冲区与线程结构不变)。
 *
 * 入环前处理链 (顺序不可调): mag_calib_apply 椭球硬/软磁校正 (传感器系)
 * -> 轴映射到体系前右下 (软磁矩阵与轴重排不可交换) -> 干扰检查 (模值应落
 * 在地磁 20~100µT 量级, 超限置 quality 干扰位; 位于低通之前 —— 低通会抹平
 * 瞬时尖峰, 被平滑的污染样本须先行检出) -> 一阶 EMA 低通。
 * 校准采集 (magcal) 从同一缓冲区取 raw 字段拟合, 因此环内同时携带
 * raw + processed 两份数据。
 *
 * 环形缓冲区用 RT-Thread 的 rt_ringbuffer (字节流), 本模块按定长记录读写:
 * 缓冲区大小为采样元素的整数倍, 满时先弹出一个最旧元素再写入。put/get 均
 * 在关中断临界区内完成。
 */

#include "mag_data.h"
#include "record_ring.h"
#include "sensor_bmm350.h"
#include "mag_calib.h"                      /* middleware/Sensor_Preprocessing/filter_calib 椭球校正 */
#include "gins_config.h"                    /* GINS_MAG_LPF_TAU_S */
#include "param_nav.h"                      /* 轴映射镜像 (安装参数, nav 分区) */
#include "timebase.h"                       /* middleware/Sensor_Drivers T_MCU 时基 */
#include <rtdevice.h>
#include <drivers/sensor.h>
#include <ipc/ringbuffer.h>
#include <math.h>

#define LOG_TAG "data.mag"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

/* 是否启用该链路 (0=跳过初始化, 磁力计未接线时置 0) */
#define MAG_DATA_ENABLE         1

/* 缓冲区容量 (样本数): 128 @100Hz ≈ 1.3s 历史数据, 占 4KB (raw+processed) */
#define MAG_DATA_BUF_COUNT      128

/* 数据源设备 (框架单位 mGauss) */
#define MAG_DATA_DEV_NAME       "mag_bmm350"

/* 轮询周期 ms = 1000 / ODR (Excel 清单 100Hz) */
#define MAG_DATA_PERIOD_MS      10

/* 采集线程: 磁力计观测无硬实时要求, 低于 gins 解算(9) */
#define MAG_DATA_THREAD_PRIO    11
#define MAG_DATA_THREAD_STACK   1024
#define MAG_DATA_THREAD_TICK    10

/* mGauss -> µT */
#define MAG_MGAUSS_TO_UT        0.1f

/* 干扰检查窗口: 地磁模值量级 20~100 µT (校准+轴映射后, 低通之前) */
#define MAG_FIELD_MIN_UT        20.0f
#define MAG_FIELD_MAX_UT        100.0f
#define MAG_QUALITY_INTERFERENCE 0x01u      /* quality bit0: 模值超限干扰 */

/* ------------------------- 入环前处理链 ------------------------- */

/* 磁力计轴映射到体系前右下: 安装参数取 param_nav 镜像 (W25Q64 nav 分区,
 * 缺省 = gins_config.h 编译期宏); `nav set maxis` 现场改向即时生效.
 * 软磁矩阵与轴重排不可交换, 因此本函数只在 mag_calib_apply 之后调用 */
static void mag_axis_map(const double src[3], double dst[3])
{
    const struct nav_params *nav = param_nav();

    for (int i = 0; i < 3; i++)
        dst[i] = (double)nav->mag_axis_sign[i] *
                 src[nav->mag_axis_src[i]];
}

/* 一阶 EMA 低通状态 (体系内逐轴), 首样本直通避免零起瞬态 */
static struct
{
    rt_bool_t init;
    float     y[3];
} mag_lpf;

/* raw(传感器系 µT) -> cal(体系前右下 µT): 校准 -> 轴映射 -> 干扰检查
 * -> 低通; quality 置干扰位 (校准后体系模值超出地磁量级, 低通之前) */
static void mag_process(const float raw[3], float cal[3], rt_uint8_t *quality)
{
    double cal_ut[3], body[3];
    float norm;

    /* float -> double 显式转换后传入: 直接 (const double*)强转会按 double
     * 读 24 字节, 实际取的是 12 字节 float 缓冲 + 相邻栈垃圾 —— 两个
     * float 位模式拼成的 double 恰为 ~1e10 量级垃圾 (实测 -4.85e10),
     * 磁链路自编译起全程被污染 (cal 全废, 磁航向观测从未生效) */
    {
        double raw_ut[3] = { (double)raw[0], (double)raw[1], (double)raw[2] };

        mag_calib_apply(raw_ut, cal_ut);               /* 无参数时直通 */
    }
    mag_axis_map(cal_ut, body);

    /* 量程守卫 (float 域): 驱动补偿链的除法 (温漂 tcs / 交叉轴分母) 在
     * 系数退化时产出 double 域巨大有限值 (转 float 溢出为 inf) 或直接
     * inf/NaN; EMA 低通会永久闩锁 (inf-inf=NaN, 一个坏样本毒死全部
     * 后续输出, 磁航向观测断供)。地磁 <100µT, 超出 1e6 判垃圾;
     * fabs(x) < 1e6 同时拒绝 NaN/±inf/huge */
    if (!(fabs(body[0]) < 1e6 && fabs(body[1]) < 1e6 && fabs(body[2]) < 1e6))
    {
        mag_lpf.init = RT_FALSE;
        *quality = MAG_QUALITY_INTERFERENCE;
        cal[0] = cal[1] = cal[2] = 0.0f;
        return;
    }

    /* 干扰检查 (位于低通之前): 模值应落在地磁 20~100µT 量级 */
    norm = sqrtf((float)(body[0] * body[0] + body[1] * body[1] +
                         body[2] * body[2]));
    *quality = (norm < MAG_FIELD_MIN_UT || norm > MAG_FIELD_MAX_UT)
               ? MAG_QUALITY_INTERFERENCE : 0u;

    /* 一阶 EMA 低通 (体系内逐轴), 首样本直通避免零起瞬态;
     * τ=0 时 α=1, 自然退化为直通旁路 */
    {
        const float alpha = (float)MAG_DATA_PERIOD_MS * 1e-3f /
                            ((float)GINS_MAG_LPF_TAU_S + (float)MAG_DATA_PERIOD_MS * 1e-3f);

        if (!mag_lpf.init)
        {
            for (int i = 0; i < 3; i++)
                mag_lpf.y[i] = (float)body[i];
            mag_lpf.init = RT_TRUE;
        }
        for (int i = 0; i < 3; i++)
            mag_lpf.y[i] += alpha * ((float)body[i] - mag_lpf.y[i]);
        for (int i = 0; i < 3; i++)
            cal[i] = mag_lpf.y[i];
    }
}

/* ------------------------- 运行状态 ------------------------- */

static struct
{
    rt_device_t          dev;
    struct record_ring   ring;      /* 定长记录环 (record_ring 公共层) */
        rt_thread_t          thread;

    struct mag_data_status st;
    struct mag_sample    last;           /* 最近推送样本 (FinSH 展示) */
} ctx;

static rt_uint8_t mag_pool[MAG_DATA_BUF_COUNT * sizeof(struct mag_sample)];

/* ------------------------- 缓冲区操作 ------------------------- */

/* 推入样本; 满时挤掉最旧并计数 (公共层 record_ring) */
static void mag_push(const struct mag_sample *s)
{
    record_ring_push(&ctx.ring, s);
}

/* ------------------------- 采集线程 ------------------------- */

/* 设备惰性打开: BMM350 init 失败由驱动后台线程无界重试, 迟到的注册
 * 依赖这里重新 find+open (init 期找不到就永久禁用的话, 驱动重试白做) */
static rt_bool_t mag_open_device(void)
{
    if (ctx.dev != RT_NULL)
        return RT_TRUE;

    ctx.dev = rt_device_find(MAG_DATA_DEV_NAME);
    if (ctx.dev == RT_NULL)
        return RT_FALSE;
    if (rt_device_open(ctx.dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        ctx.dev = RT_NULL;
        return RT_FALSE;
    }
    return RT_TRUE;
}

static void mag_thread_entry(void *parameter)
{
    struct rt_sensor_data sd;
    struct mag_sample s;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_thread_mdelay(MAG_DATA_PERIOD_MS);

        if (!mag_open_device())
        {
            if ((ctx.st.errors++ & 0x3FFu) == 0u)   /* ~100s 一次提示 */
                LOG_W("mag: device \"%s\" not ready, waiting for driver retry",
                      MAG_DATA_DEV_NAME);
            continue;
        }

        /* 时戳锚定本轮采样触发时刻 (I2C 读取之前), 与读取耗时解耦 */
        s.T_event = timebase_now_us();

        if (rt_device_read(ctx.dev, 0, &sd, sizeof(sd)) != 1 ||
            sd.type != RT_SENSOR_CLASS_MAG)
        {
            ctx.st.errors++;
            continue;
        }

        s.mag[0] = (float)sd.data.mag.x * MAG_MGAUSS_TO_UT;
        s.mag[1] = (float)sd.data.mag.y * MAG_MGAUSS_TO_UT;
        s.mag[2] = (float)sd.data.mag.z * MAG_MGAUSS_TO_UT;

        /* 入环前处理: 椭球校准 (传感器系) -> 轴映射 (前右下) ->
         * 干扰检查 (低通前) -> 低通; quality 随样本传递 */
        mag_process(s.mag, s.cal, &s.quality);

        mag_push(&s);
        ctx.st.pushed++;
        ctx.last = s;
    }
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t mag_data_peek_latest(struct mag_sample *out, rt_uint32_t *seq)
{
    /* 最新样本镜像 (非消费): magout 等调试读者专用, 不与 ginsaux 抢环 */
    return record_ring_peek_latest(&ctx.ring, out, seq);
}

rt_err_t mag_data_pop(struct mag_sample *out)
{
    rt_err_t ret = record_ring_pop(&ctx.ring, out);

    if (ret == RT_EOK)
        ctx.st.popped++;
    return ret;
}

rt_err_t mag_data_wait(rt_int32_t timeout_ms)
{
    return record_ring_wait(&ctx.ring, timeout_ms);
}

rt_uint32_t mag_data_count(void)
{
    return record_ring_count(&ctx.ring);
}

void mag_data_flush(void)
{
    record_ring_flush(&ctx.ring);
}

void mag_data_get_status(struct mag_data_status *st)
{
    if (st == RT_NULL)
        return;

    ctx.st.lost = ctx.ring.lost;      /* 挤掉计数在公共层维护 */
    *st = ctx.st;
}

/* ------------------------- 初始化 ------------------------- */

int mag_data_init(void)
{
#if MAG_DATA_ENABLE

    /* IPC/环形缓冲区无条件初始化: 设备由采集线程惰性打开 (驱动重试场景),
     * 缺失期间 wait/pop 仍安全 (信号量存在但永不释放 -> 消费方超时返回),
     * 否则 rt_sem_take 断言挂死 */
    record_ring_init(&ctx.ring, mag_pool, sizeof(mag_pool),
                     sizeof(struct mag_sample), "magdat");
    
    (void)mag_open_device();               /* 常规路径立即打开; 失败不阻止线程启动 */

    ctx.thread = rt_thread_create("magdata", mag_thread_entry, RT_NULL,
                                  MAG_DATA_THREAD_STACK, MAG_DATA_THREAD_PRIO,
                                  MAG_DATA_THREAD_TICK);
    if (ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.thread);
    ctx.st.running = RT_TRUE;

    if (ctx.dev != RT_NULL)
        LOG_I("ready: %d samples x %d bytes, %d ms period, dev=%s",
              MAG_DATA_BUF_COUNT, (int)sizeof(struct mag_sample),
              MAG_DATA_PERIOD_MS, MAG_DATA_DEV_NAME);
    else
        LOG_W("device \"%s\" not found yet, link starts when driver retry succeeds",
              MAG_DATA_DEV_NAME);
#endif /* MAG_DATA_ENABLE */

    return 0;
}
INIT_APP_EXPORT(mag_data_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void magdata(void)
{
    struct mag_data_status st;

    mag_data_get_status(&st);

    LOG_I("=== MAG data buffer (raw + calib/axis/lpf processed) ===");
    LOG_I("running : %s, count=%u/%d",
          st.running ? "yes" : "no", mag_data_count(), MAG_DATA_BUF_COUNT);
    LOG_I("stats   : pushed=%u popped=%u lost=%u errors=%u",
          st.pushed, st.popped, st.lost, st.errors);
    if (st.pushed > 0)
    {
        LOG_I("last raw: %.2f %.2f %.2f uT (sensor frame, magcal 拟合用)",
              (double)ctx.last.mag[0], (double)ctx.last.mag[1],
              (double)ctx.last.mag[2]);
        LOG_I("last cal: %.2f %.2f %.2f uT (FRD body, 喂引擎)",
              (double)ctx.last.cal[0], (double)ctx.last.cal[1],
              (double)ctx.last.cal[2]);
        LOG_I("stamp   : %u.%06u s, quality=0x%02x%s",
              (rt_uint32_t)(ctx.last.T_event / 1000000u),
              (rt_uint32_t)(ctx.last.T_event % 1000000u),
              ctx.last.quality,
              (ctx.last.quality & 0x01u) ? " (干扰, ginsaux 降权)" : "");
    }
    LOG_I("hint    : cal 字段 = mag_calib 椭球校正 -> 轴映射 -> 低通 (入环前完成)");
}
MSH_CMD_EXPORT(magdata, MAG raw data ring buffer status);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
