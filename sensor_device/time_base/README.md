# 时间基准

[timer_capture.c](timer_capture.c) 注册 RT-Thread Timebase 设备，独占 TIM2 寄存器和中断。CH4 捕获 IMU 就绪，CH3 捕获 PPS，CH1/CH2 提供执行器比较事件。

[systime.c](systime.c) 提供单调时间、计数转换、PPS 校时和时钟域映射。[systime_rtthread.c](systime_rtthread.c) 在板级启动标识就绪后打开 Timebase，并启动 PPS 消费线程。

TIM2 上电后独立计数。连续 8 个合格的 1 Hz PPS 间隔用于估计频偏，软件逐步修正 tick 到微秒的换算比例。已锁定后超过 2.5 秒未收到有效 PPS 时，沿用最后已施加的修正比例，TIM2 和 IMU 时间戳继续运行。UTC 对时另用 GNSS 时间报文与 PPS 配对，失去参考后 UTC 不确定度逐步增加。

板级绑定见 [board](../../board/README.md)，时钟与硬件预算见 [spec/07](../../spec/07_实时调度与板级硬件资源设计.md)。
