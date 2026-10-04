/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * KF-GINS 融合解算 (ADIS16505 + UM982) -> VOFA+ JustFloat (USART1)
 *
 * 数据来源: middleware/Navigation/gins 组合导航桥接 (ADIS16505 1kHz DR + UM982 10Hz ->
 * GIEngine EKF), 该模块由 INIT_ENV_EXPORT 自启解算线程并发布解算快照, 本链路
 * 只按固定节拍取最新快照转发, 不再做姿态解算。
 *
 * 输出 (USART1 = PA9/PA10, 与 console 同口, 460800 8N1, 波特率见 board.c
 * 与 Flight.ioc 的 USART1.BaudRate):
 *   JustFloat: 10 x float32(小端) + 帧尾 {00 00 80 7F}, 共 44 字节/帧
 *     ch0 roll(°)   ch1 pitch(°)   ch2 yaw(°, 引擎输出已连续, 无 ±180° 跳变)
 *     ch3 vn(m/s)   ch4 ve(m/s)    ch5 vd(m/s)
 *     ch6 lat(°)    ch7 lon(°)     ch8 alt(m)
 *     ch9 ready     (0 = 对准/等定位中, 1 = 解算中)
 *   50Hz 时约 2200 B/s, 占 460800 带宽的 4.8%。二进制帧尾不会出现在 ASCII 日志
 *   里, 同口混排时 VOFA 解析器不会失步; 跑数据时建议执行 `vofa log off`。
 *   注意 float32 只有约 7 位有效数字, ch6/ch7 经纬度在 VOFA 上有 ~0.4 m 级
 *   显示抖动, 看精确位置用 FinSH `gins` 命令。
 *
 * 引擎就绪条件 (见 gins_bridge.h): UM982 定位有效 + PPS 同步 + 静止对准窗口
 * 满。就绪前姿态/速度/位置通道全 0, ch9 = 0; 刚就绪瞬间解算跳到真实姿态
 * (roll/pitch 由加计对准播种) 属正常现象。
 *
 * FinSH 命令: vofa [log on|off]
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>

#include "gins_bridge.h"            /* gins_bridge_get_solution(): KF-GINS 融合解算快照 */
#include "app_out.h"                /* app_out_write(): USART1 共享写 */

/* ulog 日志: LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "vofa"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define VOFA_UART_DEV           "uart1"         /* 与 console 同口 (PA9/PA10) */
#define VOFA_RATE_HZ            50              /* JustFloat 输出频率 */
#define VOFA_CH_NUM             10
#define VOFA_FRAME_SIZE         (VOFA_CH_NUM * 4 + 4)

#define VOFA_THREAD_PRIO        12              /* 低于 GINS 解算线程(9), 高于 FinSH(20) */
#define VOFA_THREAD_STACK       4096
#define VOFA_THREAD_TICK        10
#define LOG_PERIOD_MS           1000            /* 状态日志周期, 0 = 关闭 */

/* ---------------------------- 运行状态 ---------------------------- */

static struct
{
    rt_device_t uart;
    rt_thread_t thread;

    rt_uint32_t frames;
    rt_bool_t on;               /* 数据帧输出开关 (默认关, uart1 单写者) */
    rt_bool_t log_on;
    rt_tick_t log_tick;
} ctx;

/* ---------------------------- 数据输出 ---------------------------- */

/*
 * JustFloat 帧: n 个小端 float32 + 帧尾 {00 00 80 7F}。
 * 一次 app_out_write 写完并用共享互斥保护, 帧不会被其他数据链插断。
 */
static void vofa_send_justfloat(const float *ch, rt_uint8_t n)
{
    static rt_uint8_t frame[VOFA_FRAME_SIZE];

    if (ctx.uart == RT_NULL || n == 0 || n > VOFA_CH_NUM)
        return;

    memcpy(frame, ch, (rt_size_t)n * 4u);
    frame[n * 4u + 0] = 0x00;
    frame[n * 4u + 1] = 0x00;
    frame[n * 4u + 2] = 0x80;
    frame[n * 4u + 3] = 0x7F;

    app_out_write(ctx.uart, frame, (rt_size_t)n * 4u + 4u);
}

/* ---------------------------- 输出线程 ---------------------------- */

static void vofa_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        struct gins_solution sol;
        float out[VOFA_CH_NUM];

        rt_thread_mdelay(1000 / VOFA_RATE_HZ);

        if (!ctx.on)
            continue;

        gins_bridge_get_solution(&sol);

        out[0] = (float)sol.roll;
        out[1] = (float)sol.pitch;
        out[2] = (float)sol.yaw;
        out[3] = (float)sol.vn;
        out[4] = (float)sol.ve;
        out[5] = (float)sol.vd;
        out[6] = (float)sol.latitude;
        out[7] = (float)sol.longitude;
        out[8] = (float)sol.altitude;
        out[9] = sol.ready ? 1.0f : 0.0f;

        vofa_send_justfloat(out, VOFA_CH_NUM);
        ctx.frames++;

#if LOG_PERIOD_MS > 0
        if (ctx.log_on &&
            rt_tick_get() - ctx.log_tick >= rt_tick_from_millisecond(LOG_PERIOD_MS))
        {
            ctx.log_tick = rt_tick_get();
            LOG_I("natt r=%d p=%d y=%d deg | v=(%d,%d,%d) cm/s | "
                  "imu=%u gnss=%u | frames=%d | %s",
                  (int)sol.roll, (int)sol.pitch, (int)sol.yaw,
                  (int)(sol.vn * 100.0), (int)(sol.ve * 100.0),
                  (int)(sol.vd * 100.0),
                  sol.imu_cnt, sol.gnss_cnt, (int)ctx.frames,
                  sol.ready ? "RUN" : "ALIGN/WAIT");
        }
#endif
    }
}

/* ---------------------------- 初始化 ---------------------------- */

int vofa_link_init(void)
{
    /* 输出串口: console 已打开则直接复用, 不改变其配置 */
    ctx.uart = rt_device_find(VOFA_UART_DEV);
    if (ctx.uart == RT_NULL)
    {
        LOG_E("vofa: uart \"%s\" not found", VOFA_UART_DEV);
        return -RT_ERROR;
    }
    if (!(ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(ctx.uart, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            LOG_E("vofa: open uart \"%s\" failed", VOFA_UART_DEV);
            ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
    }

    ctx.log_on = RT_FALSE;      /* 默认关: natt 日志与数据帧同口, 跑数据要干净 */

    ctx.thread = rt_thread_create("vofa", vofa_thread_entry, RT_NULL,
                                   VOFA_THREAD_STACK, VOFA_THREAD_PRIO, VOFA_THREAD_TICK);
    if (ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.thread);

    LOG_I("vofa: JustFloat %d ch @ %d Hz on %s (ch0-2 r/p/y deg, "
          "ch3-5 vn/ve/vd m/s, ch6-8 lat/lon/alt, ch9 ready)",
          VOFA_CH_NUM, VOFA_RATE_HZ, VOFA_UART_DEV);
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void vofa(int argc, char **argv)
{
    struct gins_solution sol;

    if (argc >= 2)
    {
        if (!rt_strcmp(argv[1], "on") || !rt_strcmp(argv[1], "off"))
        {
            /* 数据帧输出开关 (默认 off: uart1 单写者约定, 见 README) */
            ctx.on = !rt_strcmp(argv[1], "on") ? RT_TRUE : RT_FALSE;
            LOG_I("vofa %s (默认 off, on 后与 console 并写同口存在残余竞态)",
                  ctx.on ? "on" : "off");
            return;
        }
        if (!rt_strcmp(argv[1], "log"))
        {
            if (argc >= 3 && !rt_strcmp(argv[2], "off"))
                ctx.log_on = RT_FALSE;
            else if (argc >= 3 && !rt_strcmp(argv[2], "on"))
                ctx.log_on = RT_TRUE;
            LOG_I("vofa log %s", ctx.log_on ? "on" : "off");
            return;
        }
    }

    gins_bridge_get_solution(&sol);

    LOG_I("=== VOFA GINS link ===");
    LOG_I("run     : %s, uart %s @ %s 8N1",
          ctx.thread ? "yes" : "no", VOFA_UART_DEV, APP_OUT_BAUD_TEXT);
    LOG_I("frames  : %d, ch=%d, %d Hz",
          (int)ctx.frames, VOFA_CH_NUM, VOFA_RATE_HZ);
    LOG_I("gins    : %s, r=%d p=%d y=%d deg, imu=%u gnss=%u",
          sol.ready ? "RUNNING" : "ALIGN/WAIT",
          (int)sol.roll, (int)sol.pitch, (int)sol.yaw,
          sol.imu_cnt, sol.gnss_cnt);
    LOG_I("hint    : 姿态/位置细节看 `gins` 命令; 跑数据时 `vofa log off`");
}
MSH_CMD_EXPORT(vofa, VOFA GINS link: vofa [log on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
