# 板级外设与设备接入

RT-Thread 负责设备注册和生命周期。板级总线、DMA 和引脚绑定见 [board_device.h](board_device.h)。

| 当前板载器件 | RT-Thread 对象 | 源码归属 |
|---|---|---|
| ADIS16505-2 / SPI1 | `imu` | [IMU](../sensor_device/imu/) |
| BMM350 / I2C1 | `mag_sensor` | [磁强计](../sensor_device/mag/) |
| BMP585 / I2C4 | `barometer` | [气压计](../sensor_device/barometer/) |
| PMU / I2C2，型号待定 | `current`，支持有界寄存器读取 | [PMU](../sensor_device/pmu/) |
| UM982 / UART4 + PPS | 原生 `uart4`，460800 baud，2048 B RX DMA | [GNSS 接收](../sensor_device/gnss/)；日志解析在 M2 |
| GCS / USART2、RC / USART3 | 原生 `uart2`、`uart3` | [BSP 串口 DMA](../libraries/HAL_Drivers/drivers/drv_usart_dma.c) 与 [M1 串口配置](../middleware/communication/telemetry_link/telemetry_link.c) |
| DShot / TIM1 与 DMA2 Stream2 | `dshot` | [DShot 设备驱动](../sensor_device/actuator/dshot.c) |
| 捕获与比较 / TIM2 | `timebase` | [捕获设备](../sensor_device/time_base/timer_capture.c) 和 [时间映射](../sensor_device/time_base/systime.c) |
| W25Q64JV / OCTOSPI1 | 原生 MTD `nor0` | [NOR 驱动](../sensor_device/storage/W25Q64/w25q64jv.c) |
| STM32 RNG | RT-Thread HWCRYPTO 的 `hwcryto` | [BSP 硬件 RNG](../libraries/HAL_Drivers/drivers/drv_crypto.c) 与 [启动标识](board_identity.c) |
| IWDG | 原生 `wdt` | [BSP 看门狗](../libraries/HAL_Drivers/drivers/drv_wdt.c) |
| microTF / SDMMC1 | 原生 MMC/SD 块设备 | [SDMMC 主机驱动](../sensor_device/storage/SDMMC/drv_sdmmc.c) 与 RT-Thread MMC/SD |
| USB0 / USB1 OTG_HS 内置 FS PHY | CDC 虚拟串口 `vcom` | [BSP USB 驱动](../libraries/HAL_Drivers/drivers/drv_usbd.c)、[板级时钟和引脚](board.c)、[CubeMX 配置](CubeMX_Config/Flight.ioc) |

现有片上外设走 BSP 驱动。DShot 和存储设备的引脚和外设时钟由板级 MSP 配置。`.device_dma` 中的缓冲位于 D2 SRAM，按 32 字节对齐。资源预算见 [spec/07](../spec/07_实时调度与板级硬件资源设计.md)。

USB0_N 接 PA11（D−），USB0_P 接 PA12（D+），两脚均为 AF10。USB 使用 48 MHz PLL3Q 和 OTG_HS IRQ。RT-Thread 提供 USB Device 和 CDC，调用 [板级身份接口](board_identity.c) 取得 STM32 UID 序列号。`flight_console` 位于 [板级维护命令](diagnostics/device_diagnostics.c)。当前接法按板端供电、VDD33USB 已供电和关闭专用 VBUS 检测配置。
