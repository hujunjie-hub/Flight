# M2 传感器处理

| 目录 | 职责 |
|---|---|
| [sample_acquisition](sample_acquisition/) | 原生设备读取、采样时间映射、测量交接和 HIL 输入切换 |
| [sample_normalization](sample_normalization/) | 组装标准 IMU、气压与 GNSS 样本 |
| [measurement_filter](measurement_filter/) | 气压和磁场有效性、GNSS 时效与位置跳变筛选 |
| [calibration](calibration/) | 偏置、尺度、非正交修正和安装旋转 |
| [source_health](source_health/) | 源级健康、隔离、降级和恢复 |

器件的字节顺序、校验和、物理量换算及专用命令归 [sensor_device](../../sensor_device/) 的型号实现。M2 采集按板上实际型号引用驱动头文件，通过 RT-Thread 取得读数，使用 [time_base](../../sensor_device/time_base/) 映射计数和捕获时间，再向应用交付统一样本。GNSS 报文由 [protocol](../protocol/) 解码。

工程参数见 [config](../../config/)，标定资料见 [doc/calibration](../../doc/calibration/)。
