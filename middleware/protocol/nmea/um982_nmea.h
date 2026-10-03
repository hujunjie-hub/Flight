/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UM982 NMEA PVT 解析 (组合导航 GNSS 观测输入)
 *
 * 硬件 (doc/Flight.xlsx 接口配置页):
 *   UM982 RTK GNSS: UART4 @460800, 10Hz, TXD->PA1(RX)
 *
 * ---------------------------------------------------------------------------
 * 定位
 * ---------------------------------------------------------------------------
 * 本模块是纯协议解析: 不占串口、不开线程。UART4 接收链路随时间框架
 * 重建接入, 届时原始字节进 middleware/data 的 gnss_raw_data
 * 字节环形缓冲区; gnss_data 的解析线程从字节环取流组句, 经
 * um982_nmea_feed_line() 喂到这里, 解析结果汇入 struct gnss_data,
 * 并被组装成 gnss_sample 推入 gnss_data 结构环形缓冲区供 KF-GINS 消费。
 *
 * 解析语句 (UM982 上电配置需使能, 10Hz):
 *   RMC: UTC 日期+时间 / 定位状态(A/V) / 地面速度(kn) / 航迹角(deg true)
 *        -> 换算 N/E 天向速度: vn = v*cos(cog), ve = v*sin(cog)
 *   GGA: 纬度/经度/高程 / 定位质量(=fix_type, 同时给出 rtk_status)
 *        / 参与定位卫星数 / HDOP / UTC 时间
 *   ZDA: UTC 日期+时间 (可选, 补充/校准日期基准)
 *
 * GGA 只有时间没有日期, 日期取最近一条 RMC/ZDA 的日期 (跨 UTC 零点自动修正)。
 *
 * ---------------------------------------------------------------------------
 * 与组合导航 (KF-GINS) 的关系
 * ---------------------------------------------------------------------------
 * struct gnss_data 是本工程的 GNSS 观测标准格式; 对接 KF-GINS 时:
 *   GNSS.time = GPST 周内秒  <- um982_nmea_to_gpst() (UTC + 闰秒)
 *   GNSS.blh  = [纬度,经度,高程] (纬/经转 rad)
 *   GNSS.std  <- um982_nmea_pos_std() (HDOP 粗估 NED 方差, 精确值在
 *               kf_gins.yaml 的 gnss std 里标定)
 *
 * 注意: NMEA 标准语句不含天向速度, velocity.vu 恒为 0, vel_valid 只代表
 * 水平速度有效。KF-GINS v1.0 的 EKF 只用 GNSS 位置观测, 不受影响。
 *
 * FinSH 命令: um982  查看解析结果与统计。
 */

#ifndef __UM982_NMEA_H__
#define __UM982_NMEA_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif


/* ------------------------- 配置 ------------------------- */

/* 是否启用 (组合导航不用 GNSS 时置 0) */
#define UM982_NMEA_ENABLE           1

/* NMEA 单句缓冲: GGA 约 103 字节; #UNIHEADINGA (双天线航向) 约 190 字节,
 * 160 会截断长句 -> 残句进解析器误记 csum_err 且干扰 RMC/GGA 统计 */
#define UM982_NMEA_LINE_MAX         256
#define UM982_NMEA_FIELD_MAX        24

/* 位置标准差粗估 (KF-GINS 观测方差初值用):
 * std_h = HDOP * UERE, std_v = 2 * HDOP * UERE */
#define UM982_NMEA_UERE_M           1.0     /* 用户等效测距误差 (多星座) */
#define UM982_NMEA_HDOP_FALLBACK    2.0     /* GGA 未给 HDOP 时的保守值 */

/* ------------------------- 定位状态 ------------------------- */

/* fix_type: 取值与 GGA 第 6 字段 (定位质量) 对齐 */
enum gnss_fix_type
{
    GNSS_FIX_INVALID  = 0,      /* 无定位 */
    GNSS_FIX_GPS      = 1,      /* 单点定位 (GPS SPS) */
    GNSS_FIX_DGPS     = 2,      /* 差分定位 */
    GNSS_FIX_PPS      = 3,      /* PPS 精密定位 */
    GNSS_FIX_RTK_FIX  = 4,      /* RTK 固定解 */
    GNSS_FIX_RTK_FLOAT= 5,      /* RTK 浮点解 */
    GNSS_FIX_EST      = 6,      /* 估算 (航位推算) */
};

enum gnss_rtk_status
{
    GNSS_RTK_NONE   = 0,        /* 无差分/RTK */
    GNSS_RTK_DGPS   = 1,        /* 码差分 */
    GNSS_RTK_FLOAT  = 2,        /* RTK 浮点解 */
    GNSS_RTK_FIX    = 3,        /* RTK 固定解 */
};

/* ------------------------- 数据结构 -------------------------
 * 与 doc 中的 GNSS 信息树一一对应:
 *   time      -> UTC
 *   position  -> latitude / longitude / altitude
 *   velocity  -> vn / ve / vu (NED, m/s)
 *   status    -> fix_type / satellites / hdop / rtk_status
 */

struct gnss_time
{
    rt_uint32_t utc_sec;        /* UTC 纪元秒 (1970-01-01 起, 含日期) */
    rt_uint32_t utc_usec;       /* 秒内微秒, 报文自带 (10Hz 报文通常为 0) */
};

struct gnss_position
{
    double latitude;            /* 纬度, deg (N 为正) */
    double longitude;           /* 经度, deg (E 为正) */
    float  altitude;            /* 椭球高 (GGA 天线高 + 海面分离), m.
                                 * 组合导航 (KF-GINS BLH) 用 WGS84 椭球高;
                                 * GGA 未给海面分离时退化为海拔高 */
    float  geoid_sep;           /* 海面分离 (GGA field 11), m; 海拔高 = altitude - geoid_sep */
};

struct gnss_velocity
{
    float vn;                   /* 北向速度, m/s */
    float ve;                   /* 东向速度, m/s */
    float vu;                   /* 天向速度, m/s (NMEA 无此量, 恒为 0) */
};

struct gnss_status
{
    rt_uint8_t fix_type;        /* enum gnss_fix_type (GGA 定位质量) */
    rt_uint8_t satellites;      /* 参与定位的卫星数 (GGA) */
    float      hdop;            /* 水平精度因子 (GGA) */
    rt_uint8_t rtk_status;      /* enum gnss_rtk_status */
};

struct gnss_data
{
    struct gnss_time     time;
    struct gnss_position position;
    struct gnss_velocity velocity;
    struct gnss_status   status;

    rt_bool_t   time_valid;     /* time 为有效 UTC (已有日期基准) */
    rt_bool_t   pos_valid;      /* 最近一条 GGA 定位有效 (RMC 收到 V 时清零) */
    rt_bool_t   vel_valid;      /* 水平速度有效 (RMC A 且速度/航迹角可解析) */

    rt_uint32_t update_cnt;     /* 有效位置更新计数, 变化即有新观测 */
};

/* ------------------------- 接口 ------------------------- */

/* 喂入一条完整 NMEA 语句 "$..*HH" (只读不改写 s)。
 * 由 middleware/data/gnss_data.c 的解析线程从 gnss_raw_data 字节环取流
 * 组句后喂入; 也可自行喂入 (内部会再验校验和, 垃圾句被拒绝)。 */
void um982_nmea_feed_line(const char *s);

/* 取一致性快照 (短临界区拷贝, 可在任意线程调用) */
void um982_nmea_get_data(struct gnss_data *out);

/* UTC -> GPST: 由 time 换算 GPS 周 + 周内秒 (KF-GINS 的 GNSS.time 格式)。
 * 未建立时间基准时返回 -RT_ERROR。闰秒按 18s (2017-01-01 起, 注意闰秒公告)。 */
rt_err_t um982_nmea_to_gpst(const struct gnss_data *d,
                            rt_uint16_t *week, double *sow);

/* HDOP 粗估 NED 位置标准差 [n, e, u] (KF-GINS 的 GNSS.std 格式, 单位 m) */
void um982_nmea_pos_std(const struct gnss_data *d, double std[3]);

/* 复位解析状态与统计 */
void um982_nmea_reset(void);


#ifdef __cplusplus
}
#endif

#endif /* __UM982_NMEA_H__ */
