/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GNSS (UM982) 结构化数据环形缓冲区实现 (接口说明见 gnss_data.h)
 *
 * 接收线程 (gnssrx, 只搬字节不解析): USART2 rx_indicate 唤醒 ->
 *   rt_device_read 搬出 -> gnss_raw_data_push() 原始字节环。
 *   串口开 DMA_RX: RX 空闲线批量指示 (每语句至多唤醒一次, 与统一流程
 *   "ISR 最短路径"同构; BSP_UART2_RX_USING_DMA 已在 .config 使能,
 *   DMA1_Stream2 见 board.c NVIC 表)。
 *
 * 解析线程 (gnssdata, 字节环唯一常驻消费者):
 *   gnss_raw_data_wait() 阻塞等新字节 -> pop 批量取出 -> 组句
 *   ($/\r/\n 状态机, 超长丢整句; um982_nmea 内部自带 "$..*HH" 校验和
 *   验证, 垃圾句被拒绝) -> 检测 update_cnt 变化 (10Hz 定位解)
 *   -> UTC 双分流:
 *        整秒语句且定位有效 -> timebase_pps_pair() 刷新 PPS 滑窗
 *        (门槛1 定位有效在此判定: pos_valid 且 fix_type 达单点以上;
 *         门槛2~4 与滑窗拟合由时间基座执行)
 *        语句时标 -> timebase_utc_to_mcu() 换算 T_event
 *        (映射未就绪/作废时置 0 并计数, utc_sec/usec 仍保真入环)
 *   -> 组装 struct gnss_sample 推入本环形缓冲区。
 *
 * 环形缓冲区用 RT-Thread 的 rt_ringbuffer (字节流), 按定长记录读写:
 * 缓冲区大小为采样元素的整数倍, 满时先弹出最旧元素再写入。
 */

#include "gnss_data.h"
#include "record_ring.h"
#include "gnss_raw_data.h"              /* middleware/data 字节镜像环 */
#include "um982_nmea.h"                 /* middleware/protocol 解析 */
#include "timebase.h"                   /* middleware/timebase 时间基座 */
#include <rtdevice.h>
#include <ipc/ringbuffer.h>

#define LOG_TAG "data.gnss"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 配置 ------------------------- */

/* 是否启用该链路 (0=跳过线程, 协议层数据仍可经 um982_nmea_get_data 直接读) */
#define GNSS_DATA_ENABLE         1

/* 缓冲区容量 (样本数): 32 @10Hz ≈ 3.2s 历史数据 */
#define GNSS_DATA_BUF_COUNT      32

/* 字节环等待超时 ms (无数据时休眠; 生产者随语句到达推送) */
#define GNSS_DATA_WAIT_MS        100

/* 组句缓冲: 与协议层 UM982_NMEA_LINE_MAX 一致 (GGA 最长约 103 字节) */
#define GNSS_DATA_LINE_MAX       UM982_NMEA_LINE_MAX

/* 采集线程: GNSS 观测无硬实时要求, 与 magdata/barodata 同级
 * 栈预算: feed_line buf[256]+f[24] (~360B) + 组句/样本局部 (~300B) +
 * um982_nmea_to_gpst/timebase_pps_pair 内 ulog LOG_I 格式化尖峰;
 * 1536 时 PPS 配对日志曾击穿栈底 (# 魔数被踩 -> 调度器溢出死循环) */
#define GNSS_DATA_THREAD_PRIO    11
#define GNSS_DATA_THREAD_STACK   3072
#define GNSS_DATA_THREAD_TICK    10

/* 接收线程单次搬运上限 (字节) */
#define GNSS_RX_CHUNK            128

/* 串口配置: 460800 8N1 (Flight.ioc USART2), 消费环 (rx_bufsz) 4KB
 * (serial v2 默认 rx 64B/ping 32B 在 460800 下仅 1.4ms 缓冲, 不足一句
 * GGA; 512B 在 ~4KB/s NMEA 流下仅 128ms 余量 —— rx 线程偶发调度延迟导致
 * 消费环满后按 OVERWRITE 策略丢最旧数据, 读出"句子缝合+重复"流, 校验
 * 失败率 ~50%; 4KB 把余量提高到 ~1s, 覆盖最坏调度抖动, 并与 0.5s 时效
 * 门限解耦: 环容量 > 门限时间窗的 2 倍, 积压不再直接决定样本时效。
 * v2 语义: DMA 直写 ping 环 (256B, 事件即搬走不积压), 丢数据点只在
 * 消费环满 —— 与 v1 "DMA 直接套圈覆写未读区" 的失效位置不同, 但对
 * 时延敏感的解析结果等价) */
#define GNSS_RX_BAUD             BAUD_RATE_460800
#define GNSS_RX_BUF_SZ           4096

/* ------------------------- 运行状态 ------------------------- */

static struct
{
    /* 接收线程 (USART2 -> gnss_raw_data) */
    rt_device_t          rx_dev;
    struct rt_semaphore  rx_sem;      /* 接收唤醒: rx_indicate 释放 */
    rt_thread_t          rx_thread;

    /* 解析线程 (gnss_raw_data -> um982_nmea -> 样本环) */
    struct record_ring   ring;      /* 定长记录环 (record_ring 公共层) */
        rt_thread_t          thread;
    rt_uint32_t          last_upd;    /* 上次已推的协议层 update_cnt */

    struct gnss_data_status st;
    struct gnss_sample     last;      /* 最近推送样本 (FinSH 展示) */
} ctx;

static rt_uint8_t gnss_pool[GNSS_DATA_BUF_COUNT * sizeof(struct gnss_sample)];

/* RX 链路诊断计数 (SWD 直读, 区分 DMA 停摆/线程饿死/上游截流):
 *   isr_evt     rx_indicate 触发次数 (IDLE/TC/HT 事件, DMA 活性)
 *   wake        gnssrx 实际唤醒次数
 *   wake_gap_max_ms  两次唤醒最大间隔 (饿死判据: >>100ms)
 *   drain_us_max     单次唤醒排空 serial FIFO 的最大耗时
 *   moved       累计搬运字节 (= raw 环 pushed)
 *   chunk_max   单次 rt_device_read 返回的最大字节数 */
static struct
{
    volatile rt_uint32_t isr_evt;
    volatile rt_uint32_t wake;
    volatile rt_uint32_t wake_gap_max_ms;
    volatile rt_uint32_t drain_us_max;
    volatile rt_uint32_t moved;
    volatile rt_uint32_t chunk_max;
} g_rxdiag;

/* ------------------------- 缓冲区操作 ------------------------- */

/* 推入样本; 满时挤掉最旧并计数 (公共层 record_ring) */
static void gnss_push(const struct gnss_sample *s)
{
    record_ring_push(&ctx.ring, s);
}

/* ------------------------- 接收线程 (只搬字节, 不解析) ------------------------- */

static rt_err_t gnssrx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    g_rxdiag.isr_evt++;
    rt_sem_release(&ctx.rx_sem);
    return RT_EOK;
}

static void gnss_rx_thread_entry(void *parameter)
{
    rt_uint8_t chunk[GNSS_RX_CHUNK];
    rt_tick_t last_wake = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_size_t n;

        /* 事件驱动: 等 USART2 新字节 (UM982 断连时静默超时空转) */
        if (rt_sem_take(&ctx.rx_sem,
                        rt_tick_from_millisecond(GNSS_RX_WAIT_MS)) != RT_EOK)
            continue;

        {
            rt_tick_t now = rt_tick_get();
            rt_tick_t gap = (rt_tick_t)(now - last_wake);

            if (last_wake != 0 && gap > g_rxdiag.wake_gap_max_ms)
                g_rxdiag.wake_gap_max_ms = gap;
            last_wake = now;
            g_rxdiag.wake++;
        }

        {
            rt_uint64_t t0 = timebase_now_us();
            rt_size_t moved = 0;

            while ((n = rt_device_read(ctx.rx_dev, 0, chunk, sizeof(chunk))) > 0)
            {
                if (n > g_rxdiag.chunk_max)
                    g_rxdiag.chunk_max = n;
                moved += n;
                gnss_raw_data_push(chunk, n);
            }
            g_rxdiag.moved += moved;

            if (moved > 0)
            {
                rt_uint32_t us = (rt_uint32_t)(timebase_now_us() - t0);

                if (us > g_rxdiag.drain_us_max)
                    g_rxdiag.drain_us_max = us;
            }
        }
    }
}

/* ------------------------- 解析线程 ------------------------- */

/* 字节流 -> 完整语句: 遇 '\n' 结句喂协议层 (其内部验校验和), 超长丢弃 */
static void gnss_line_feed(rt_uint8_t c)
{
    static char line[GNSS_DATA_LINE_MAX];
    static rt_uint16_t len = 0;
    static rt_bool_t discard = RT_FALSE;   /* 超长句剩余字节丢弃到 '\n' */

    if (c == '\n')
    {
        if (len > 0 && !discard)
        {
            line[len] = '\0';
            um982_nmea_feed_line(line);
        }
        len = 0;
        discard = RT_FALSE;
        return;
    }
    if (c == '\r')
        return;

    if (discard)
        return;                             /* 超长句的残尾: 直接丢弃 */

    if (len < GNSS_DATA_LINE_MAX - 1u)
        line[len++] = (char)c;
    else
        discard = RT_TRUE;                  /* 超长: 丢整句 (含残尾) */
}

static void gnss_thread_entry(void *parameter)
{
    rt_uint8_t chunk[128];

    RT_UNUSED(parameter);

    while (1)
    {
        /* 事件驱动: 等 gnss_raw_data 新字节, 排空并逐句喂协议层 */
        if (gnss_raw_data_wait(GNSS_DATA_WAIT_MS) != RT_EOK)
            continue;

        {
            rt_size_t n;

            while ((n = gnss_raw_data_pop(chunk, sizeof(chunk))) > 0)
                for (rt_size_t i = 0; i < n; i++)
                    gnss_line_feed(chunk[i]);
        }

        /* 协议层有新定位解 (update_cnt 变化) 则组装入环 */
        {
            struct gnss_data d;

            um982_nmea_get_data(&d);
            if (d.update_cnt == 0u || d.update_cnt == ctx.last_upd)
                continue;
            ctx.last_upd = d.update_cnt;

            /*
             * UTC 双分流 (整秒语句先配对再换算: 配对语句的整秒恰为新
             * 基准点 PPS2, 待换算时标属 <=1 个 PPS 周期的外推)。
             * 门槛1 (定位有效): pos_valid 且 fix_type 达单点定位以上 ——
             * GNSS 未定位时接收机 PPS/NMEA 时标照常输出但走内部自由
             * 时钟, 配对会"成功"却锁错 UTC。
             */
            if (d.time_valid && d.time.utc_usec == 0u &&
                d.pos_valid && d.status.fix_type >= GNSS_FIX_GPS)
            {
                timebase_pps_pair((rt_uint64_t)d.time.utc_sec * 1000000u,
                                  timebase_now_us());
            }

            {
                struct gnss_sample s;
                rt_uint64_t utc_us;

                s.utc_sec  = d.time_valid ? d.time.utc_sec : 0u;
                s.utc_usec = d.time_valid ? d.time.utc_usec : 0u;

                utc_us = (rt_uint64_t)s.utc_sec * 1000000u + s.utc_usec;
                if (!d.time_valid ||
                    !timebase_utc_to_mcu(utc_us, &s.T_event))
                {
                    s.T_event = 0;        /* 映射未就绪/作废: gins 跳过 */
                    ctx.st.ts_zero++;
                }

                s.T_arrival     = timebase_now_us();
                s.latitude_deg  = d.position.latitude;
                s.longitude_deg = d.position.longitude;
                s.altitude_m    = d.position.altitude;
                s.vn            = d.velocity.vn;
                s.ve            = d.velocity.ve;
                s.vu            = d.velocity.vu;
                s.vel_valid     = (d.vel_valid == RT_TRUE) ? 1u : 0u;
                s.fix_type      = d.status.fix_type;
                s.satellites    = d.status.satellites;
                s.rtk_status    = d.status.rtk_status;
                s.hdop          = d.status.hdop;

                gnss_push(&s);
                ctx.st.pushed++;
                ctx.last = s;
            }
        }
    }
}

/* ------------------------- 对外接口 ------------------------- */

rt_err_t gnss_data_peek_latest(struct gnss_sample *out, rt_uint32_t *seq)
{
    /* 最新样本镜像 (非消费): out_* 调试读者专用, 不与融合消费方抢环 */
    return record_ring_peek_latest(&ctx.ring, out, seq);
}

rt_err_t gnss_data_pop(struct gnss_sample *out)
{
    rt_err_t ret = record_ring_pop(&ctx.ring, out);

    if (ret == RT_EOK)
        ctx.st.popped++;
    return ret;
}

rt_err_t gnss_data_wait(rt_int32_t timeout_ms)
{
    return record_ring_wait(&ctx.ring, timeout_ms);
}

rt_uint32_t gnss_data_count(void)
{
    return record_ring_count(&ctx.ring);
}

void gnss_data_flush(void)
{
    record_ring_flush(&ctx.ring);
}

void gnss_data_get_status(struct gnss_data_status *st)
{
    if (st == RT_NULL)
        return;

    ctx.st.lost = ctx.ring.lost;      /* 挤掉计数在公共层维护 */
    *st = ctx.st;
}

/* ------------------------- 初始化 ------------------------- */

static rt_err_t gnss_rx_start(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    ctx.rx_dev = rt_device_find(GNSS_RX_DEV_NAME);
    if (ctx.rx_dev == RT_NULL)
    {
        LOG_E("uart \"%s\" not found, GNSS receive link disabled", GNSS_RX_DEV_NAME);
        return -RT_ERROR;
    }

    /* 460800 8N1 + 加大 RX 环 (serial v2 默认 64B/115200, ping 环 32B)。
     * v2 结构字段: rx_bufsz=消费环 (框架 rb), dma_ping_bufsz=DMA 直写环
     * (ping 环长度必须 32 倍数: B5 对齐布局按行补齐, 见 dev_serial_v2.c)。
     * ping 256B: 460800 下 TC 事件 ~180次/s (32B 时 ~1.4k次/s), 语句
     * 边界仍由 IDLE 事件兜底。打开前配置 (ref_count==0) 允许改尺寸。 */
    cfg.baud_rate       = GNSS_RX_BAUD;
    cfg.rx_bufsz        = GNSS_RX_BUF_SZ;
    cfg.dma_ping_bufsz  = 256;
    rt_device_control(ctx.rx_dev, RT_DEVICE_CTRL_CONFIG, &cfg);

    /* 信号量须在挂回调/开设备前就绪: open 即武装 RX DMA, 460800 下 UM982
     * 持续发送, 事件早于 sem_init 到达时 rx_indicate 会对未初始化信号量
     * rt_sem_release (零值挂起链表被 rt_list_isempty 判非空 -> NULL 链表
     * 操作, RT_ASSERT 配置下则直接断言挂死) */
    rt_sem_init(&ctx.rx_sem, "gnssrxs", 0, RT_IPC_FLAG_FIFO);

    rt_device_set_rx_indicate(ctx.rx_dev, gnssrx_indicate);
    if (rt_device_open(ctx.rx_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_RX_NON_BLOCKING) != RT_EOK)
    {
        LOG_E("open \"%s\" failed", GNSS_RX_DEV_NAME);
        ctx.rx_dev = RT_NULL;
        return -RT_ERROR;
    }

    ctx.rx_thread = rt_thread_create("gnssrx", gnss_rx_thread_entry, RT_NULL,
                                     GNSS_RX_THREAD_STACK, GNSS_RX_THREAD_PRIO,
                                     GNSS_RX_THREAD_TICK);
    if (ctx.rx_thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.rx_thread);
    ctx.st.rx_running = RT_TRUE;

    LOG_I("rx link: %s @460800 8N1 DMA_RX (bufsz %d), mirror -> gnss_raw_data",
          GNSS_RX_DEV_NAME, GNSS_RX_BUF_SZ);
    return RT_EOK;
}

int gnss_data_init(void)
{
#if GNSS_DATA_ENABLE
    record_ring_init(&ctx.ring, gnss_pool, sizeof(gnss_pool),
                     sizeof(struct gnss_sample), "gnssdat");
    
    /* 接收线程 (生产者): 失败时解析线程照常等待 (字节环无数据, 静默) */
    gnss_rx_start();

    ctx.thread = rt_thread_create("gnssdata", gnss_thread_entry, RT_NULL,
                                  GNSS_DATA_THREAD_STACK, GNSS_DATA_THREAD_PRIO,
                                  GNSS_DATA_THREAD_TICK);
    if (ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.thread);
    ctx.st.running = RT_TRUE;

    LOG_I("ready: %d samples x %d bytes, parse from gnss_raw_data -> um982_nmea",
          GNSS_DATA_BUF_COUNT, (int)sizeof(struct gnss_sample));
#endif /* GNSS_DATA_ENABLE */

    return 0;
}
INIT_APP_EXPORT(gnss_data_init);

/* ------------------------- FinSH 命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gnssdata(void)
{
    struct gnss_data_status st;

    gnss_data_get_status(&st);

    LOG_I("=== GNSS data buffer (parsed) ===");
    LOG_I("running : rx=%s parse=%s, count=%u/%d",
          st.rx_running ? "yes" : "no", st.running ? "yes" : "no",
          gnss_data_count(), GNSS_DATA_BUF_COUNT);
    LOG_I("stats   : pushed=%u popped=%u lost=%u ts_zero=%u (T_event=0, 映射未就绪)",
          st.pushed, st.popped, st.lost, st.ts_zero);
    if (st.pushed > 0)
    {
        rt_uint64_t delay_us = (ctx.last.T_arrival > ctx.last.T_event)
                               ? ctx.last.T_arrival - ctx.last.T_event : 0;

        LOG_I("last    : fix=%u sat=%u, lat %.6f lon %.6f alt %.1f m, "
              "v=(%.2f %.2f %.2f) m/s",
              ctx.last.fix_type, ctx.last.satellites,
              (double)ctx.last.latitude_deg, (double)ctx.last.longitude_deg,
              (double)ctx.last.altitude_m,
              (double)ctx.last.vn, (double)ctx.last.ve, (double)ctx.last.vu);
        LOG_I("         utc=%u.%06u, T_event=%u.%06u s, 链路延迟=%u us "
              "(T_arrival-T_event)",
              ctx.last.utc_sec, ctx.last.utc_usec,
              (rt_uint32_t)(ctx.last.T_event / 1000000u),
              (rt_uint32_t)(ctx.last.T_event % 1000000u),
              (rt_uint32_t)delay_us);
    }
    LOG_I("hint    : 原始字节镜像看 `gnssraw`; 映射/PPS 看 `timebase`; "
          "解析细节看 `um982`");
}
MSH_CMD_EXPORT(gnssdata, GNSS parsed data ring buffer status);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */
