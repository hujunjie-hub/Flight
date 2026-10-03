# crsf/ — CRSF 遥控链路纯协议层

ExpressLRS/Crossfire 接收机的 CRSF 帧解析 (2026-10-03, FMT_README §13.5
任务 #4)。与 nmea/mavlink 同约定: **不占串口、不开线程** —— 串口接收
线程在 `middleware/data/rc_data.c` ("rcrx"), 本层只做帧同步/CRC8
(poly 0xD5)/通道解包。

- 帧格式: `[sync 0xC8][len][type][payload][crc8]`, len = type+payload+crc;
- RC_CHANNELS_PACKED (type 0x16, 22B): 16 通道 × 11bit LSB 优先连续位流,
  原始值域 172..1811 (中点 992), 归一化 [-1,1];
- 通道序 AETR (ELRS 缺省), 轴约定换算在 rc_data 层 (见 rc_data.h);
- 只消费接收方向; CRSF telemetry (飞控→接收机, MSP/电池回传) 为后续项。

CRC8 查表由脚本生成并与逐位实现互证一致 (对拍样帧 0xE3)。
