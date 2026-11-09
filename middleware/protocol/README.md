# 线协议

| 目录 | 职责 |
|---|---|
| [mavlink](mavlink/) | MAVLink 2 封帧与 CRC、HIL 消息、FCU 方言生成布局与来源哈希 |
| [nmea](nmea/) | 有界的 NMEA 句子校验与正文提取 |
| [unicore_ascii](unicore_ascii/) | UM982 原生 BESTNAVA/RECTIMEA 报文解码 |

[通信模块](../communication/) 管理串口和发送队列。[M2](../sensor/) 负责原生 GNSS 的时间映射和测量发布，HIL 任务负责仿真测量适配。NMEA 入口供需要标准句子的接收链路使用，当前 UM982 配置使用原生 ASCII 报文。
