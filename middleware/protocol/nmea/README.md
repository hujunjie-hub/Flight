# um982_nmea — UM982 NMEA PVT 解析

（原 middleware/protocol 顶层 README 的 um982 章节，目录重组后随模块移入。）

## 定位

**纯协议解析模块**：不开线程、不占串口。UART4 (PA1, 460800, 10Hz) 的
接收链路在 `middleware/data`：**gnssrx 接收线程**只搬字节入
`gnss_raw_data` 字节环，**gnssdata 解析线程**按 `'\n'` 组句后喂
`um982_nmea_feed_line()`（协议层内部验校验和, 见 middleware/README.md
"UM982" 节的完整链路说明）。

```
UM982 ──460800──> UART4 (DMA_RX) ──> [gnssrx 线程] ──> gnss_raw_data 字节环
                                                            │ '\n' 组句
                                                            ▼
                                     [gnssdata 解析线程] um982_nmea.c ──解析──> struct gnss_data
                                                            │  (定位有效整秒句 → timebase PPS 配对;
                                                            │   语句 UTC → T_MCU 映射打 T_event)
                          组合导航应用 <──um982_nmea_get_data()─┘
                          (对接 middleware/KF-GINS)
```

## 语句要求（UM982 上电配置）

| 语句 | 提供内容 | 必需性 |
|------|----------|--------|
| GGA  | 纬度/经度/高程/定位质量(fix_type, rtk_status)/卫星数/HDOP/UTC 时间 | **必需** |
| RMC  | UTC 日期+时间/定位状态(A/V)/地面速度(kn)+航迹角 → N/E 速度 | **必需**（日期与速度来源） |
| ZDA  | UTC 日期+时间 | 可选（日期备份，RMC 未使能时必需） |

UM982 未使能相应语句时对应字段保持旧值；`vel_valid` 只代表水平速度
（**NMEA 标准语句无天向速度，vu 恒为 0**；KF-GINS v1.0 的 EKF 只用
GNSS 位置观测，不受影响）。

**单句缓冲 `UM982_NMEA_LINE_MAX` = 256**：双天线 `#UNIHEADINGA` 约
190 字符、多星 GSV 可达 200+，此前 160 会截断长句（丢头残句进解析器
误记 csum_err 并干扰统计，2026-09-29 实测修复）；组句层
`GNSS_DATA_LINE_MAX` 与其同源。解析器只认 RMC/GGA/ZDA，其余语句校验
通过后静默忽略（仍占 460800 带宽，UM982 端建议只输出所需语句）。

## 数据结构（与信息树一一对应）

```
struct gnss_data
├── time       struct gnss_time       utc_sec (UTC 纪元秒) / utc_usec
├── position   struct gnss_position   latitude / longitude (deg, N/E 为正)
│                                     altitude (椭球高 m) / geoid_sep (m)
├── velocity   struct gnss_velocity   vn / ve / vu (NED, m/s)
├── status     struct gnss_status     fix_type / satellites / hdop / rtk_status
└── time_valid / pos_valid / vel_valid / update_cnt
```

`altitude` 说明：GGA 天线高是海拔高（MSL），解析时加上海面分离（field 11）
得到组合导航需要的 **WGS84 椭球高**；GGA 未给海面分离时退化为海拔高原值，
`geoid_sep` 可用于还原。

fix_type 取值与 GGA 定位质量对齐（4=RTK 固定解，5=RTK 浮点解，2/3=差分，
1=单点，0=无效），rtk_status 由其归纳。

## KF-GINS 对接（middleware/KF-GINS）

```c
#include "um982_nmea.h"

struct gnss_data d;
um982_nmea_get_data(&d);

if (d.pos_valid && d.update_cnt != last_cnt)
{
    kf_gins_t g;                    /* KF-GINS 的 GNSS 结构 (types.h) */
    rt_uint16_t week;
    double sow, std[3];

    um982_nmea_to_gpst(&d, &week, &sow);    /* UTC -> GPST 周内秒 (闰秒 18s) */
    g.time = sow;
    g.blh  = {d.position.latitude  * DEG2RAD,
              d.position.longitude * DEG2RAD,
              d.position.altitude};
    um982_nmea_pos_std(&d, std);            /* HDOP 粗估 N/E/U 方差 (UERE=1m) */
    g.std   = {std[0], std[1], std[2]};
    g.isvalid = true;
    engine->addGnssData(g);
}
```

时序上 GNSS 观测的精确时刻取报文 UTC（`um982_nmea_to_gpst`）；HDOP 粗估
只做初值，精确观测方差在 `middleware/gins/gins_config.h` /
`gins_bridge.cpp`（固件侧参数）里标定。

## 构建与调试

- `nmea/SConscript` 把本目录编入固件（经 `middleware/protocol/SConscript`
  汇入），头文件路径全局可见，`middleware/data/gnss_data.c` 解析线程直接
  `#include "um982_nmea.h"`（CMake 侧在根 CMakeLists 的 RT_USING_SENSOR
  源列表与 include 路径里）。
- FinSH：`um982` 查看解析结果（lat/lon/alt/vn/ve/fix/rtk/sats/hdop）与统计
  （rmc/gga/zda 计数、校验错、字段错、更新次数）；链路问题先查 `gnss` 命令。
- 回归：链路数据质量走 build_host SWD 流水线（`swd_10rounds.py` +
  `eval_10rounds.py`, zda/rmc 速率与 csum/ts_zero 计数属 17 判据）。
  （utest 框架已于 2026-09-30 移除, 原 `middleware.protocol.um982_nmea`
  用例随之下线。）
