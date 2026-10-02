/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BARO (BMP585) 原始数据环形缓冲区实现 (接口说明见 baro_data.h)
 *
 * 采集线程按 ODR 周期 (100Hz -> 10ms) 轮询: 先读 baro_bmp585, 再读
 * temp_bmp585 取同芯片温度。T_event 在读取触发时刻打戳 (timebase T_MCU
 * 时基; BMP585 INT 引脚 PE13 硬件已预留, 迁移到 EXTI 事件源后时戳改为
 * EXTI ISR 捕获)。
 *
 * 环形缓冲区用 RT-Thread 的 rt_ringbuffer (字节流), 本模块按定长记录读写:
 * 缓冲区大小为采样元素的整数倍, 满时先弹出一个最旧元素再写入。put/get 均
 * 在关中断临界区内完成。
 */

#include "baro_data.h"
#include "record_ring.h"
#include "sensor_bmp585.h"
#include "timebase.h"               /* middleware/timebase T_MCU 时基 */
#include <rtdevice.h>
#include <drivers/sensor.h>
#include <ipc/ringbuffer.h>

#define LOG_TAG "data.baro"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

/* 是否启用该链路 (0=跳过初始化, 气压计未接线时置 0) */
#define BARO_DATA_ENABLE        1

/* 缓冲区容量 (样本数): 128 @100Hz ≈ 1.3s 历史数据, 占 2KB */
#define BARO_DATA_BUF_COUNT     128

/* 数据源设备 (框架单位: 气压 Pa, 温度 0.1°C) */
#define BARO_DATA_DEV_NAME      "baro_bmp585"
#define BARO_DATA_TEMP_DEV_NAME "temp_bmp585"

/* 轮询周期 ms = 1000 / ODR (Excel 清单 100Hz) */
#define BARO_DATA_PERIOD_MS     10

/* 采集线程: 气压观测无硬实时要求, 低于 gins 解算(9) */
#define BARO_DATA_THREAD_PRIO   11
#define BARO_DATA_THREAD_STACK  4096    /* ulog 格式化尖峰 + 迟到重试路径 (2026-09-29: 1024 死于设备上线 LOG_I) */
#define BARO_DATA_THREAD_TICK   10

/* 0.1°C -> °C */
#define BARO_DCELSIUS_TO_C      0.1f

/* 设备迟到补偿: init 期设备未就绪时, 采集线程重试 find+open 的参数
 * (窗口须覆盖 BMP585 复位后 NVM 停摆自愈时间, 见 sensor_bmp585.c) */
#define BARO_DATA_FIND_RETRY_MS     5000

/* ------------------------- 运行状态 ------------------------- */

static struct
{
    rt_device_t          dev;
    rt_device_t          temp_dev;
    struct record_ring   ring;      /* 定长记录环 (record_ring 公共层) */
        rt_thread_t          thread;

    struct baro_data_status st;
    struct baro_sample   last;           /* 最近推送样本 (FinSH 展示) */
} ctx;

static rt_uint8_t baro_pool[BARO_DATA_BUF_COUNT * sizeof(struct baro_sample)];

/* ------------------------- 缓冲区操作 ------------------------- */

/* 推入样本; 满时挤掉最旧并计数 (公共层 record_ring) */
static void baro_push(const struct baro_sample *s)
{
    record_ring_push(&ctx.ring, s);
}

/* ------------------------- 采集线程 ------------------------- */

/* find + open 两设备 (部分失败时回退); init 与线程内迟到重试共用 */
static rt_err_t baro_open_devices(void)
{
    rt_device_t dev, temp_dev;

    dev       = rt_device_find(BARO_DATA_DEV_NAME);
    temp_dev  = rt_device_find(BARO_DATA_TEMP_DEV_NAME);
    if (dev == RT_NULL || temp_dev == RT_NULL)
        return -RT_ENOSYS;

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
        return -RT_EIO;
    if (rt_device_open(temp_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_device_close(dev);
        return -RT_EIO;
    }

    ctx.dev      = dev;
    ctx.temp_dev = temp_dev;
    return RT_EOK;
}

static void baro_thread_entry(void *parameter)
{
    struct rt_sensor_data sd_baro, sd_temp;
    struct baro_sample s;
    rt_uint8_t find_retries = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        /* 设备迟到补偿 (BMP585 上电初始化失败重试等场景): init 期未找到
         * 设备时线程仍启动, 在此周期性重试 find+open, 成功后进入正常采集 */
        if (ctx.dev == RT_NULL || ctx.temp_dev == RT_NULL)
        {
            if (baro_open_devices() == RT_EOK)
            {
                ctx.st.running = RT_TRUE;
                LOG_I("baro devices online (deferred, retry %u)", find_retries);
            }
            else
            {
                /* 无界重试: BMP585 复位后 NVM 停摆自愈可达十几分钟
                 * (见 sensor_bmp585.c), 提前放弃会让链路迟到即死 */
                find_retries++;
            }
            rt_thread_mdelay(BARO_DATA_FIND_RETRY_MS);
            continue;
        }

        rt_thread_mdelay(BARO_DATA_PERIOD_MS);

        /* 时戳锚定本轮采样触发时刻 (I2C 读取之前), 与读取耗时解耦 */
        s.T_event = timebase_now_us();

        /* 先读气压, 再读温度 (同芯片同源) */
        if (rt_device_read(ctx.dev, 0, &sd_baro, sizeof(sd_baro)) != 1 ||
            sd_baro.type != RT_SENSOR_CLASS_BARO)
        {
            ctx.st.errors++;
            continue;
        }
        if (rt_device_read(ctx.temp_dev, 0, &sd_temp, sizeof(sd_temp)) != 1 ||
            sd_temp.type != RT_SENSOR_CLASS_TEMP)
        {
            ctx.st.errors++;
            continue;
        }

        s.pressure_pa   = (float)sd_baro.data.baro;
        s.temperature_c = (float)sd_temp.data.temp * BARO_DCELSIUS_TO_C;

        baro_push(&s);
        ctx.st.pushed++;
        ctx.last = s;
    }
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t baro_data_pop(struct baro_sample *out)
{
    rt_err_t ret = record_ring_pop(&ctx.ring, out);

    if (ret == RT_EOK)
        ctx.st.popped++;
    return ret;
}

rt_err_t baro_data_wait(rt_int32_t timeout_ms)
{
    return record_ring_wait(&ctx.ring, timeout_ms);
}

rt_uint32_t baro_data_count(void)
{
    return record_ring_count(&ctx.ring);
}

void baro_data_flush(void)
{
    record_ring_flush(&ctx.ring);
}

void baro_data_get_status(struct baro_data_status *st)
{
    if (st == RT_NULL)
        return;

    ctx.st.lost = ctx.ring.lost;      /* 挤掉计数在公共层维护 */
    *st = ctx.st;
}

/* ------------------------- 初始化 ------------------------- */

int baro_data_init(void)
{
#if BARO_DATA_ENABLE

    /* IPC/环形缓冲区无条件初始化: 设备缺失提前 return 后 wait/pop 仍安全
     * (信号量存在但永不释放 -> 消费方超时返回), 否则 rt_sem_take 断言挂死 */
    record_ring_init(&ctx.ring, baro_pool, sizeof(baro_pool),
                     sizeof(struct baro_sample), "bardat");
    
    /* 设备未就绪不再直接放弃: 线程照常启动, 由其周期重试 find+open
     * (见 baro_thread_entry), 覆盖传感器上电初始化慢/失败重试的场景 */
    if (baro_open_devices() != RT_EOK)
    {
        LOG_W("device \"%s\"/\"%s\" not found yet, deferred retry in thread",
              BARO_DATA_DEV_NAME, BARO_DATA_TEMP_DEV_NAME);
    }
    else
    {
        ctx.st.running = RT_TRUE;
    }

    ctx.thread = rt_thread_create("barodata", baro_thread_entry, RT_NULL,
                                  BARO_DATA_THREAD_STACK, BARO_DATA_THREAD_PRIO,
                                  BARO_DATA_THREAD_TICK);
    if (ctx.thread == RT_NULL)
    {
        if (ctx.temp_dev) rt_device_close(ctx.temp_dev);
        if (ctx.dev) rt_device_close(ctx.dev);
        return -RT_ENOMEM;
    }
    rt_thread_startup(ctx.thread);

    if (ctx.st.running)
    {
        LOG_I("ready: %d samples x %d bytes, %d ms period, dev=%s+%s",
              BARO_DATA_BUF_COUNT, (int)sizeof(struct baro_sample),
              BARO_DATA_PERIOD_MS, BARO_DATA_DEV_NAME, BARO_DATA_TEMP_DEV_NAME);
    }
#endif /* BARO_DATA_ENABLE */

    return 0;
}
INIT_APP_EXPORT(baro_data_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void barodata(void)
{
    struct baro_data_status st;

    baro_data_get_status(&st);

    LOG_I("=== BARO data buffer (raw, uncalibrated) ===");
    LOG_I("running : %s, count=%u/%d",
          st.running ? "yes" : "no", baro_data_count(), BARO_DATA_BUF_COUNT);
    LOG_I("stats   : pushed=%u popped=%u lost=%u errors=%u",
          st.pushed, st.popped, st.lost, st.errors);
    if (st.pushed > 0)
    {
        LOG_I("last    : %.1f Pa, %.1f C, T_event %u.%06u s",
              (double)ctx.last.pressure_pa, (double)ctx.last.temperature_c,
              (rt_uint32_t)(ctx.last.T_event / 1000000u),
              (rt_uint32_t)(ctx.last.T_event % 1000000u));
    }
    LOG_I("hint    : pop 出的样本由消费方做基准偏移校准 (barocal)");
}
MSH_CMD_EXPORT(barodata, BARO raw data ring buffer status);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
