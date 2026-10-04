/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 遥控输入数据层 — 实现 (轴约定/硬件占位说明见 rc_data.h 头注)
 */
#include <string.h>

#include <rtthread.h>
#include <rtdevice.h>

#include "rc_data.h"
#include "crsf.h"                   /* CRSF 解析 (纯协议层) */

/* ulog 日志: LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "rc"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define RCRX_THREAD_PRIO       11   /* 与采集线程同层, 高于 FinSH(20) */
#define RCRX_THREAD_STACK      1536
#define RCRX_THREAD_TICK       10
#define RCRX_POLL_MS           50   /* rx_indicate 之外的兜底轮询 */

/* ---------------------------- 快照 ---------------------------- */

static struct
{
    struct rc_data snap;
    rt_sem_t       rx_sem;
    rt_thread_t    thread;
    rt_device_t    dev;
    struct crsf_stats crsf_st;
} s;

void rc_data_publish(enum rc_data_src src, const float axis[4],
                     const float aux[4])
{
    rt_base_t lv = rt_hw_interrupt_disable();

    s.snap.src = (rt_uint8_t)src;
    memcpy(s.snap.axis, axis, sizeof(s.snap.axis));
    memcpy(s.snap.aux, aux, sizeof(s.snap.aux));
    s.snap.stamp_ms = rt_tick_get_millisecond();
    s.snap.seq++;
    rt_hw_interrupt_enable(lv);
}

static void snap_get(struct rc_data *out)
{
    rt_base_t lv = rt_hw_interrupt_disable();

    *out = s.snap;
    rt_hw_interrupt_enable(lv);
}

void rc_data_get_latest(struct rc_data *out)
{
    rt_uint32_t now = rt_tick_get_millisecond();

    snap_get(out);
    out->valid = (out->seq != 0 &&
                  now - out->stamp_ms < RC_DATA_TIMEOUT_MS) ? 1 : 0;
}

rt_uint32_t rc_data_age_ms(void)
{
    struct rc_data rc;

    snap_get(&rc);
    if (rc.seq == 0)
        return 0xFFFFFFFFu;
    return rt_tick_get_millisecond() - rc.stamp_ms;
}

/* ---------------------------- CRSF 接收线程 ----------------------------
 * 占位 UART (见 rc_data.h): 设备不存在时 rc_data_crsf_start 返回错误,
 * 本线程不创建 —— 虚拟摇杆源照常工作。 */

static rt_err_t rcrx_indicate(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);
    if (s.rx_sem != RT_NULL)
        rt_sem_release(s.rx_sem);
    return RT_EOK;
}

static void rcrx_entry(void *parameter)
{
    static rt_uint8_t buf[32];
    struct crsf_channels ch;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_size_t n;

        rt_sem_take(s.rx_sem, rt_tick_from_millisecond(RCRX_POLL_MS));

        while ((n = rt_device_read(s.dev, 0, buf, sizeof(buf))) > 0)
        {
            for (rt_size_t i = 0; i < n; i++)
            {
                if (crsf_feed(buf[i], &ch) == 1)
                {
                    /* AETR -> 统一轴约定 (rc_data.h):
                     * pitch = -E (推杆为 +), roll = A, yaw = R, thr = T;
                     * aux = ch5-8 */
                    float axis[4] = { -ch.ch[1], ch.ch[0], ch.ch[3],
                                      ch.ch[2] };
                    float aux[4]  = { ch.ch[4], ch.ch[5], ch.ch[6],
                                      ch.ch[7] };

                    rc_data_publish(RC_SRC_CRSF, axis, aux);
                }
            }
        }
    }
}

int rc_data_crsf_start(void)
{
    if (s.thread != RT_NULL)
        return RT_EOK;

    s.dev = rt_device_find(RC_CRSF_DEV);
    if (s.dev == RT_NULL)
    {
        LOG_W("crsf uart \"%s\" not found (占位未启用, 仅虚拟摇杆源)",
              RC_CRSF_DEV);
        return -RT_ERROR;
    }
    if (!(s.dev->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        /* 打开前配置 (ref_count==0 才允许改缓冲尺寸): 420000 (ELRS 标准)
         * + 加大 RX 环 —— serial v2 默认 64B/32B, 42kB/s 下消费环仅 ~1.5ms
         * 余量, 调度抖动即环满丢字节 -> CRC 错帧 (gnss_data 460800 链已踩过,
         * 扩 4KB/256B 后消除)。ping 环长度须 32 倍数 (B5 对齐布局,
         * 见 dev_serial_v2.c)。 */
        struct serial_configure cfg;

        if (rt_device_control(s.dev, RT_SERIAL_CTRL_GET_CONFIG, &cfg) == RT_EOK)
        {
            cfg.baud_rate      = RC_CRSF_BAUD;
            cfg.rx_bufsz       = RC_CRSF_RX_BUF_SZ;
            cfg.dma_ping_bufsz = RC_CRSF_DMA_PING_BUF_SZ;
            if (rt_device_control(s.dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
                LOG_W("crsf config %s failed (check wiring/ELRS cfg)",
                      RC_CRSF_BAUD_TEXT);
        }

        if (rt_device_open(s.dev, RT_DEVICE_OFLAG_RDWR |
                                 RT_DEVICE_FLAG_RX_NON_BLOCKING) != RT_EOK)
        {
            LOG_E("crsf open \"%s\" failed", RC_CRSF_DEV);
            s.dev = RT_NULL;
            return -RT_ERROR;
        }
    }
    rt_device_set_rx_indicate(s.dev, rcrx_indicate);

    s.rx_sem = rt_sem_create("rcrx", 0, RT_IPC_FLAG_FIFO);
    if (s.rx_sem == RT_NULL)
        return -RT_ENOMEM;

    s.thread = rt_thread_create("rcrx", rcrx_entry, RT_NULL,
                                RCRX_THREAD_STACK, RCRX_THREAD_PRIO,
                                RCRX_THREAD_TICK);
    if (s.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(s.thread);

    LOG_I("rcrx: CRSF on %s @%s 8N1 (ELRS)", RC_CRSF_DEV, RC_CRSF_BAUD_TEXT);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static const char *rc_src_name(rt_uint8_t src)
{
    switch (src)
    {
    case RC_SRC_CRSF:    return "CRSF";
    case RC_SRC_VIRTUAL: return "virtual";
    default:             return "none";
    }
}

static void rc(int argc, char **argv)
{
    struct rc_data r;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rc_data_get_latest(&r);
    crsf_get_stats(&s.crsf_st);

    LOG_I("=== rc input (CRSF 占位 %s / QGC 虚拟摇杆) ===",
          s.thread ? "ON" : "off");
    LOG_I("src    : %s, %s, age=%u ms, seq=%u",
          rc_src_name(r.src),
          r.valid ? "VALID" : "LOST/none",
          rc_data_age_ms(), r.seq);
    LOG_I("axis   : pitch=%d roll=%d yaw=%d thr=%d (x1000)",
          (int)(r.axis[0] * 1000.0f), (int)(r.axis[1] * 1000.0f),
          (int)(r.axis[2] * 1000.0f), (int)(r.axis[3] * 1000.0f));
    LOG_I("aux    : ch5=%d ch6=%d ch7=%d ch8=%d (x1000, ch5<-0.5=kill)",
          (int)(r.aux[0] * 1000.0f), (int)(r.aux[1] * 1000.0f),
          (int)(r.aux[2] * 1000.0f), (int)(r.aux[3] * 1000.0f));
    if (s.thread)
        LOG_I("crsf   : frames=%u ch=%u crc_err=%u", s.crsf_st.frames,
              s.crsf_st.ch_frames, s.crsf_st.crc_err);
}
MSH_CMD_EXPORT(rc, rc input snapshot);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
