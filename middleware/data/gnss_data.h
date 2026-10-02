/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GNSS (UM982) 结构化数据环形缓冲区
 *
 * 数据链路 (两级环形缓冲区, 见根 README "UM982 UART 变体"):
 *   UM982 --460800 8N1--> USART2 (INT_RX)
 *     -> 本模块接收线程 (只搬字节, 不解析)
 *       -> gnss_raw_data_push() 原始字节环形缓冲区 (middleware/data)
 *   本模块解析线程 (字节环唯一常驻消费者):
 *     gnss_raw_data_wait/pop() 取字节 -> 组句 ($/\r/\n 状态机)
 *     -> um982_nmea_feed_line() 协议解析 (middleware/protocol,
 *        内部验校验和) -> 检测 update_cnt 变化 (10Hz 定位解)
 *     -> UTC 双分流:
 *          整秒语句 (定位有效) -> timebase_pps_pair() 刷新 PPS 滑窗
 *          语句时标 -> timebase_utc_to_mcu() 换算 T_event
 *     -> 组装结构化样本推入本环形缓冲区
 *       -> gnss_data_pop() 消费方 (KF-GINS / 数据记录)
 *
 * 采样元素 (结构见下, 与根 README "传感器数据结构"一致):
 *   T_event  T_MCU 时戳 (us), 由语句 UTC 经 clock_map 映射换算而来
 *            (注意: 不是 GNSS 数据到达 MCU 的时间); 映射未就绪时置 0,
 *            gins 线程跳过该样本, utc_sec/usec 仍保真入环
 *   utc_sec/usec  UTC 纪元秒 + 秒内微秒 (语句原始时标)
 *   T_arrival 语句到达 MCU 的 T_MCU 时刻 (监控 T_event - T_arrival
 *            链路延迟, 异常增大即接收/解析链拥塞)
 *
 * 溢出策略: 缓冲区满时挤掉最旧样本 (新数据永远进得来), 计入 status.lost。
 *
 * FinSH: gnssdata  查看缓冲区状态与最新样本
 */

#ifndef __GNSS_DATA_H__
#define __GNSS_DATA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 配置 ------------------------- */

/* 接收线程 (只搬字节): USART2 事件源, 优先级高于解析线程 */
#define GNSS_RX_DEV_NAME            "uart2"     /* UM982, 460800 8N1 */
#define GNSS_RX_THREAD_PRIO         8           /* 高于解析(11), 低于 imudata(7) */
#define GNSS_RX_THREAD_STACK        1024
#define GNSS_RX_THREAD_TICK         10
#define GNSS_RX_WAIT_MS             100         /* 无数据等待超时 */

/* ------------------------- 采样结构 (根 README gnss_data_t) ------------------------- */

struct gnss_sample
{
    rt_uint64_t T_event;        /* 时间戳, us; T_MCU 时基, 由语句 UTC 经
                                 * clock_map 映射换算; 映射未就绪时置 0,
                                 * gins 线程跳过该样本 */

    rt_uint32_t utc_sec;        /* UTC 纪元秒 (1970-01-01 起, 含日期):
                                 * 语句原始时标保真入环, 映射未就绪期仍有效,
                                 * 可离线重算映射 */
    rt_uint32_t utc_usec;       /* 亚秒部分, us (10Hz 语句为 0.1s 步进) */

    rt_uint64_t T_arrival;      /* 语句到达 MCU 的 T_MCU 时刻: 监控
                                 * T_event - T_arrival 链路延迟 */

    double      latitude_deg;   /* 纬度, deg (N 为正) */
    double      longitude_deg;  /* 经度, deg (E 为正) */
    double      altitude_m;     /* 椭球高, m */

    float       vn;             /* 北向速度, m/s */
    float       ve;             /* 东向速度, m/s */
    float       vu;             /* 天向速度, m/s (NMEA 无此量, 恒 0) */

    rt_uint8_t  fix_type;       /* enum gnss_fix_type (um982_nmea.h, 0=无定位) */
    rt_uint8_t  satellites;     /* 使用或参与解算的卫星数 */
    float       hdop;           /* 水平精度因子 (KF-GINS 观测方差: std=hdop*UERE) */
    rt_uint8_t  rtk_status;     /* enum gnss_rtk_status (差分/RTK 状态) */
    rt_uint8_t  vel_valid;      /* RMC 水平速度有效 (KF-GINS 速度观测) */
};

/* ------------------------- 状态 ------------------------- */

struct gnss_data_status
{
    rt_bool_t   running;        /* 解析线程已启动 */
    rt_bool_t   rx_running;     /* 接收线程已启动 */
    rt_uint32_t pushed;         /* 累计推送样本数 */
    rt_uint32_t popped;         /* 累计读出样本数 */
    rt_uint32_t lost;           /* 缓冲区满被挤掉的样本数 */
    rt_uint32_t ts_zero;        /* T_event=0 入环样本数 (映射未就绪/作废) */
};

/* ------------------------- 接口 ------------------------- */

/* 初始化并启动接收/解析线程 (INIT_APP_EXPORT 自动执行) */
int gnss_data_init(void);

/* 读出一个样本 (FIFO); 空时返回 -RT_EEMPTY */
rt_err_t gnss_data_pop(struct gnss_sample *out);

/* 阻塞等待新样本 (每个推送释放一次), 超时返回 -RT_ETIMEOUT */
rt_err_t gnss_data_wait(rt_int32_t timeout_ms);

/* 当前缓冲样本数 */
rt_uint32_t gnss_data_count(void);

/* 清空缓冲区 */
void gnss_data_flush(void);

void gnss_data_get_status(struct gnss_data_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_DATA_H__ */
