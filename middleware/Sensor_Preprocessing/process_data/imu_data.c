/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU (ADIS16505) 环形缓冲区实现 (接口说明见 imu_data.h)
 *
 * 数据链路 (与 mag_data 同款: 入环前完成单位换算 + 系统级校准):
 *   驱动 DMA 链路 (DR 中断捕获 TIM2 时戳 + 直启 burst, 校验/解析后
 *     1kHz 快照) --rx_indicate--> 本模块采集线程
 *       -> adis16505_get_snapshot() 原子拷贝 (陀螺/加计/温度/时戳同源同时刻)
 *       -> 单位换算 (rad/s, m/s^2, °C)
 *       -> 系统级校准: 轴映射到体坐标系前右下 (gins_config.h 安装宏,
 *          与 GINS 解算同源; 灵敏度标度在单位换算中完成, 零偏由 EKF 在线估计)
 *       -> T_event 时戳 (驱动 EXTI ISR 捕获的 T_MCU) + data_cnt (16bit
 *          芯片计数扩展为 32bit 单调值) -> rt_ringbuffer 环形缓冲区
 *   消费方 imu_data_pop() 读出的即为可解算样本 (体坐标系物理单位)。
 *
 * 去重/时间: 重复快照用 DATA_CNTR 去重 (同一快照被重复读取时命中);
 * dt 不随样本携带, 由 gins 消费侧按 data_cnt 差分计算 (名义 ODR 换算,
 * 丢拍期 EKF 按实际间隔积分, 不误当作均匀 1ms)。
 *
 * 环形缓冲区用 RT-Thread 的 rt_ringbuffer (字节流), 本模块按定长记录读写:
 * 缓冲区大小为采样元素的整数倍, 满时先弹出一个最旧元素再写入 (put 语义,
 * 同时计数 lost)。rt_ringbuffer 自身无锁, put/get 均在关中断临界区内完成
 * (单条 ~40 字节 memcpy, 1kHz 下对中断延迟影响可忽略)。
 */

#include "imu_data.h"
#include "record_ring.h"
#include "sensor_adis16505.h"
#include "param_nav.h"                  /* 轴映射镜像 (安装参数, 与 GINS 同源) */
#include <rtdevice.h>
#include <drivers/sensor.h>
#include <ipc/ringbuffer.h>

#define LOG_TAG "data.imu"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

/* 是否启用该链路 (0=跳过初始化, IMU 未接线时置 0) */
#define IMU_DATA_ENABLE         1

/* 缓冲区容量 (样本数): 512 @1kHz ≈ 0.5s 历史数据, 占 20KB */
#define IMU_DATA_BUF_COUNT      512

/* 事件源设备: acce/gyro/temp 共用驱动快照, 挂任一设备即可 */
#define IMU_DATA_DEV_NAME       "acce_adis16505"

/* 名义输出数据率 Hz (data_cnt 差分换算 dt 用; DEC_RATE 写失败时驱动跑
 * 2000Hz, 消费侧以差分为准不受影响) */
#define IMU_DATA_NOMINAL_ODR    1000

/* 事件等待超时 ms: 超时认为 DR 事件链路失效, 主动读设备触发快照刷新 */
#define IMU_DATA_WAIT_MS        5

/* 采集线程: 优先级高于 gins 解算(9)/消费线程, 低于驱动处理线程(6) */
#define IMU_DATA_THREAD_PRIO    7
#define IMU_DATA_THREAD_STACK   1536
#define IMU_DATA_THREAD_TICK    10

/* 单位换算常数 (驱动文件头说明): 加速度 1 LSB = 78/32000 m/s², 温度 0.1°C/LSB */
#define IMU_ACCEL_LSB_MSS       (78.0f / 32000.0f)
#define IMU_TEMP_LSB_DC         0.1f
#define IMU_MDPS_TO_RADS        (0.001f * 0.017453292519943295f)

/* ------------------------- 运行状态 ------------------------- */

static struct
{
    rt_device_t          dev;
    struct record_ring   ring;      /* 定长记录环 (pop/wait/count/flush 公共层) */
    struct rt_semaphore  trig_sem;  /* 采集触发: rx_indicate 每次采样释放 */
    rt_thread_t          thread;
    rt_bool_t            running;

    rt_uint16_t          last_data_cntr;   /* 去重基准 (驱动快照 DATA_CNTR) */
    struct imu_data_status st;
    struct imu_sample    last;           /* 最近推送样本 (FinSH 展示) */
} ctx;

static rt_uint8_t imu_pool[IMU_DATA_BUF_COUNT * sizeof(struct imu_sample)];

/*
 * DATA_CNTR 16bit -> 32bit 单调扩展: 芯片计数 1kHz 下 65.5s 回绕,
 * 正常差分/回绕表现为小正增量; 大幅倒退视为芯片复位重新基线。
 * 扩展后 data_cnt 在 2^32/1kHz ≈ 49.7 天内单调, 消费侧差分即得丢拍。
 * 复位检出拍仍推进 +1: 保证 data_cnt 严格单调, 消费侧 dt 差分永不为 0
 * (复位间隔按 1ms 计虽失真, 但好于 dt=0 的除零/零间隔积分)。
 */
static rt_uint32_t s_cntr_resets;

static rt_uint32_t imu_extend_cntr(rt_uint16_t raw)
{
    static rt_bool_t init = RT_FALSE;
    static rt_uint16_t last_raw = 0;
    static rt_uint32_t acc = 0;

    if (init)
    {
        rt_uint16_t d = (rt_uint16_t)(raw - last_raw);

        if (d <= 0x8000u)
            acc += d;                     /* 正常/回绕/丢拍增量 */
        else
        {
            acc += 1;                     /* 芯片复位: +1 保持严格单调 */
            s_cntr_resets++;
        }
    }
    init = RT_TRUE;
    last_raw = raw;
    return acc;
}

/* ------------------------- 入环前处理链 ------------------------- */

/* 体坐标系轴映射 (前右下): 安装参数取 param_nav 镜像 (W25Q64 nav 分区,
 * 缺省 = gins_config.h 编译期宏), 单位换算后调用, 环形缓冲区内即为
 * 可解算的体系样本; `nav set iaxis` 现场改向即时生效 */
static void imu_axis_map(const float src[3], float dst[3])
{
    const struct nav_params *nav = param_nav();

    for (int i = 0; i < 3; i++)
        dst[i] = nav->imu_axis_sign[i] * src[nav->imu_axis_src[i]];
}

/* ------------------------- 缓冲区操作 ------------------------- */

/* 推入一个样本; 满时挤掉最旧样本并计数 (公共层 record_ring) */
static void imu_push(const struct imu_sample *s)
{
    record_ring_push(&ctx.ring, s);
}

/* ------------------------- 采集线程 ------------------------- */

static rt_err_t imu_rx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    rt_sem_release(&ctx.trig_sem);
    return RT_EOK;
}

static rt_bool_t imu_open_device(void);

static void imu_thread_entry(void *parameter)
{
    struct adis16505_snapshot snap;
    struct imu_sample s;
    rt_uint8_t i;

    RT_UNUSED(parameter);

    while (1)
    {
        /* 设备惰性打开: ADIS 迟到上线 (adisretry 重试成功) 后自动接入;
         * 未上线期间 1s 节流重试, ~17min 一次提示 */
        if (!imu_open_device())
        {
            if ((ctx.st.errors++ & 0x3FFu) == 0u)
                LOG_W("imu: device \"%s\" not ready, waiting for driver retry",
                      IMU_DATA_DEV_NAME);
            rt_sem_take(&ctx.trig_sem, rt_tick_from_millisecond(1000));
            continue;
        }

        if (rt_sem_take(&ctx.trig_sem, rt_tick_from_millisecond(IMU_DATA_WAIT_MS)) != RT_EOK)
        {
            /* DR 事件超时 (驱动看门狗退化轮询/断线): 主动读一次设备触发快照刷新 */
            struct rt_sensor_data sd;

            if (rt_device_read(ctx.dev, 0, &sd, sizeof(sd)) != 1)
                ctx.st.errors++;
        }

        /* 丢弃积压触发只取最新 (快照语义, 重复快照由 DATA_CNTR 去重) */
        while (rt_sem_take(&ctx.trig_sem, 0) == RT_EOK)
            ;

        if (adis16505_get_snapshot(&snap) != RT_EOK)
            continue;
        if (snap.data_cntr == ctx.last_data_cntr)
            continue;                       /* 无新样本 */
        ctx.last_data_cntr = snap.data_cntr;

        /* 量程系数随取 (设备迟到上线时 gyro_idx 可能在线程启动后才就绪) */
        {
            float gyro_lsb_mdps = adis16505_gyro_lsb_mdps();

            s.T_event = snap.t_event_us;      /* DR 沿时刻 (EXTI ISR 捕获) */
            s.data_cnt = imu_extend_cntr(snap.data_cntr);
            for (i = 0; i < 3; i++)
            {
                s.gyro[i]  = (float)snap.gyro[i] * gyro_lsb_mdps * IMU_MDPS_TO_RADS;
                s.accel[i] = (float)snap.acce[i] * IMU_ACCEL_LSB_MSS;
            }
        }
        s.temperature = (float)snap.temp * IMU_TEMP_LSB_DC;

        /* 系统级校准: 轴映射到体系前右下 (灵敏度标度已含在单位换算内) */
        imu_axis_map(s.gyro, s.gyro);
        imu_axis_map(s.accel, s.accel);

        imu_push(&s);
        ctx.st.pushed++;
        ctx.last = s;
    }
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t imu_data_peek_latest(struct imu_sample *out, rt_uint32_t *seq)
{
    /* 最新样本镜像 (非消费): out_* 调试读者专用, 不与融合消费方抢环 */
    return record_ring_peek_latest(&ctx.ring, out, seq);
}

rt_err_t imu_data_pop(struct imu_sample *out)
{
    rt_err_t ret = record_ring_pop(&ctx.ring, out);

    if (ret == RT_EOK)
        ctx.st.popped++;
    return ret;
}

rt_err_t imu_data_wait(rt_int32_t timeout_ms)
{
    return record_ring_wait(&ctx.ring, timeout_ms);
}

rt_uint32_t imu_data_count(void)
{
    return record_ring_count(&ctx.ring);
}

void imu_data_flush(void)
{
    record_ring_flush(&ctx.ring);
}

void imu_data_get_status(struct imu_data_status *st)
{
    if (st == RT_NULL)
        return;

    ctx.st.lost = ctx.ring.lost;      /* 挤掉计数在公共层维护 */
    ctx.st.resets = s_cntr_resets;
    *st = ctx.st;
}

/* ------------------------- 初始化 ------------------------- */

/* 设备惰性打开 (2026-10-01, 对齐 mag_data 模式): ADIS 驱动 init 失败后
 * 由 adisretry 线程无界重试, 迟到的注册依赖这里重新 find+open —— 原先
 * init 期找不到设备就永久禁用整条 IMU 数据链, 驱动重试等于白做 */
static rt_bool_t imu_open_device(void)
{
    if (ctx.dev != RT_NULL)
        return RT_TRUE;

    ctx.dev = rt_device_find(IMU_DATA_DEV_NAME);
    if (ctx.dev == RT_NULL)
        return RT_FALSE;
    if (rt_device_set_rx_indicate(ctx.dev, imu_rx_indicate) != RT_EOK)
    {
        ctx.dev = RT_NULL;
        return RT_FALSE;
    }
    if (rt_device_open(ctx.dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        ctx.dev = RT_NULL;
        return RT_FALSE;
    }
    return RT_TRUE;
}

int imu_data_init(void)
{
#if IMU_DATA_ENABLE
    /* IPC/环形缓冲区无条件初始化: 设备缺失时 wait/pop 仍安全
     * (信号量存在但永不释放 -> 消费方超时返回), 否则 rt_sem_take 断言挂死 */
    record_ring_init(&ctx.ring, imu_pool, sizeof(imu_pool),
                     sizeof(struct imu_sample), "imudat");
    rt_sem_init(&ctx.trig_sem, "imutrig", 0, RT_IPC_FLAG_FIFO);

    /* 设备打开移入线程惰性路径 (见 imu_open_device) */
    ctx.thread = rt_thread_create("imudata", imu_thread_entry, RT_NULL,
                                  IMU_DATA_THREAD_STACK, IMU_DATA_THREAD_PRIO,
                                  IMU_DATA_THREAD_TICK);
    if (ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.thread);
    ctx.running = RT_TRUE;
    ctx.st.running = RT_TRUE;

    LOG_I("ready: %d samples x %d bytes, %d Hz, trig=%s (lazy open)",
          IMU_DATA_BUF_COUNT, (int)sizeof(struct imu_sample),
          IMU_DATA_NOMINAL_ODR, IMU_DATA_DEV_NAME);
#endif /* IMU_DATA_ENABLE */

    return 0;
}
INIT_APP_EXPORT(imu_data_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void imudata(void)
{
    struct imu_data_status st;
    struct adis16505_dr_stats dr;
    struct imu_sample s;
    rt_bool_t has_last;

    imu_data_get_status(&st);
    adis16505_get_dr_stats(&dr);
    has_last = (st.pushed > 0);

    LOG_I("=== IMU data buffer (calibrated body-frame, 1kHz) ===");
    LOG_I("running : %s, count=%u/%d",
          st.running ? "yes" : "no", imu_data_count(), IMU_DATA_BUF_COUNT);
    LOG_I("stats   : pushed=%u popped=%u lost=%u errors=%u resets=%u",
          st.pushed, st.popped, st.lost, st.errors, st.resets);
    LOG_I("dr chain: miss=%u drop=%u chk_err=%u diag_err=%u dma_err=%u "
          "overrun=%u recover=%u",
          dr.miss, dr.drop, dr.chk_err, dr.diag_err, dr.dma_err,
          dr.overrun, dr.recover);
    if (has_last)
    {
        s = ctx.last;
        LOG_I("last    : T_event=%u.%06u s data_cnt=%u temp=%.1fC",
              (rt_uint32_t)(s.T_event / 1000000u),
              (rt_uint32_t)(s.T_event % 1000000u),
              s.data_cnt, (double)s.temperature);
        LOG_I("  gyro  : %.6f %.6f %.6f rad/s (FRD)",
              (double)s.gyro[0], (double)s.gyro[1], (double)s.gyro[2]);
        LOG_I("  accel : %.3f %.3f %.3f m/s^2 (FRD)",
              (double)s.accel[0], (double)s.accel[1], (double)s.accel[2]);
    }
    LOG_I("hint    : 样本已完成 单位换算+轴映射 (体坐标系); dt 由 gins 按 "
          "data_cnt 差分计算 (名义 %d Hz); 零偏由 EKF 在线估计",
          IMU_DATA_NOMINAL_ODR);
}
MSH_CMD_EXPORT(imudata, IMU ring buffer status (calibrated samples));

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
