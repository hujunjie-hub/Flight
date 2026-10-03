#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

#define SOC_STM32H723ZG
#define BOARD_STM32H723_NUCLEO

/* RT-Thread Kernel */

/* klibc options */

/* rt_vsnprintf options */

/* 标准版 rt_vsnprintf (rt_vsnprintf_std.c): 支持 %f 浮点 (ulog/日志用),
 * 替换默认 tiny 版; 栈开销高于 tiny 版, 见 ulog 文档 "注意事项" */
#define RT_KLIBC_USING_VSNPRINTF_LONGLONG
#define RT_KLIBC_USING_VSNPRINTF_STANDARD
#define RT_KLIBC_USING_VSNPRINTF_DECIMAL_SPECIFIERS
#define RT_KLIBC_USING_VSNPRINTF_EXPONENTIAL_SPECIFIERS
#define RT_KLIBC_USING_VSNPRINTF_WRITEBACK_SPECIFIER
#define RT_KLIBC_USING_VSNPRINTF_CHECK_NUL_IN_FORMAT_SPECIFIER
/* end of rt_vsnprintf options */

/* rt_vsscanf options */

/* end of rt_vsscanf options */

/* rt_memset options */

/* end of rt_memset options */

/* rt_memcpy options */

/* end of rt_memcpy options */

/* rt_memmove options */

/* end of rt_memmove options */

/* rt_memcmp options */

/* end of rt_memcmp options */

/* rt_strstr options */

/* end of rt_strstr options */

/* rt_strcasecmp options */

/* end of rt_strcasecmp options */

/* rt_strncpy options */

/* end of rt_strncpy options */

/* rt_strcpy options */

/* end of rt_strcpy options */

/* rt_strncmp options */

/* end of rt_strncmp options */

/* rt_strcmp options */

/* end of rt_strcmp options */

/* rt_strlen options */

/* end of rt_strlen options */

/* rt_strnlen options */

/* end of rt_strnlen options */
/* end of klibc options */
#define RT_NAME_MAX 16
#define RT_CPUS_NR 1
#define RT_ALIGN_SIZE 8
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_HOOK_USING_FUNC_PTR
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256

/* kservice options */

/* end of kservice options */
#define RT_USING_DEBUG
#define RT_DEBUGING_ASSERT
/* RT_DEBUGING_COLOR/CONTEXT 关闭 (2026-10-02 裁剪): COLOR 给 rt_kprintf
 * 每行输出加 ANSI 转义 (轮询 TX 下纯开销), CONTEXT 在 IPC/调度路径加
 * 上下文校验; ASSERT 保留作固件安全网 */

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
/* EVENT/MAILBOX/MESSAGEQUEUE 全仓零引用 (2026-10-02 核验, serial V2 用
 * completion+ringbuffer), 裁掉减内核编译面; 有需要再开 */

/* end of Inter-Thread communication */

/* Memory Management */

/* MEMPOOL 全仓零引用, SMALL_MEM 为实际堆实现 */
#define RT_USING_SMALL_MEM
#define RT_USING_SMALL_MEM_AS_HEAP
#define RT_USING_HEAP
/* end of Memory Management */
#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 256
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x50300
#define RT_BACKTRACE_LEVEL_MAX_NR 32
/* end of RT-Thread Kernel */
#define RT_USING_CACHE
/* 使能 Cortex-M7 I/D-Cache (rt_hw_board_init 中按宏开启; DMA 需要 cache 维护) */
#define BSP_SCB_ENABLE_I_CACHE
#define BSP_SCB_ENABLE_D_CACHE
#define RT_USING_HW_ATOMIC
#define RT_USING_CPU_FFS
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10
#define RT_USING_MSH
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_CMD_SIZE 80
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_USING_DESCRIPTION
#define FINSH_ARG_MAX 10
#define FINSH_USING_OPTION_COMPLETION

/* DFS: device virtual file system */

/* end of DFS: device virtual file system */

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_UNAMED_PIPE_NUMBER 64
#define RT_USING_SERIAL
/* 串口驱动框架 V2 (dev_serial_v2 + drv_usart_v2): 消费环 + DMA ping 环,
 * 打开语义改为 阻塞/非阻塞 位段 (旧 INT_RX/DMA_RX 旗标自动落入非阻塞默认) */
#define RT_USING_SERIAL_V2
#define RT_SERIAL_USING_DMA
/* 环满策略: 覆盖旧数据 —— 与 v1 DMA 直写环的物理回绕行为一致 */
#define RT_SERIAL_BUF_STRATEGY_OVERWRITE
#define RT_USING_I2C
#define RT_USING_SPI
/* QSPI 总线框架 (dev_qspi_core + drv_qspi @ OCTOSPI1): W25Q64 存储 */
#define RT_USING_QSPI
#define RT_USING_PIN
/* TIM2 空闲 (时间基座已移除), 未用作 hwtimer */

/* Sensor framework (RT-Thread components/drivers/sensor v1) */
#define RT_USING_SENSOR
#define RT_USING_SENSOR_CMD
/* end of Device Drivers */

/* C/C++ and POSIX layer */

/* ISO-ANSI C layer */

/* Timezone and Daylight Saving Time */

#define RT_USING_CPLUSPLUS   /* C++ 全局构造与 operator new -> RT-Thread 堆 (KF-GINS 需要) */
#define RT_LIBC_USING_LIGHT_TZ_DST
#define RT_LIBC_TZ_DEFAULT_HOUR 8
#define RT_LIBC_TZ_DEFAULT_MIN 0
#define RT_LIBC_TZ_DEFAULT_SEC 0
/* end of Timezone and Daylight Saving Time */
/* end of ISO-ANSI C layer */

/* POSIX (Portable Operating System Interface) layer */


/* Interprocess Communication (IPC) */


/* Socket is in the 'Network' category */

/* end of Interprocess Communication (IPC) */
/* end of POSIX (Portable Operating System Interface) layer */
/* end of C/C++ and POSIX layer */

/* Network */

/* end of Network */

/* Memory protection */

/* end of Memory protection */

/* Utilities */

/* ulog 日志组件 (取代业务代码里的 rt_kprintf): 各模块用 DBG_TAG/LOG_X (rtdbg.h) */
#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL_D
#define ULOG_OUTPUT_LVL 7
/* 中断上下文日志 (按键 IRQ / 串口 RX 回调等) */
#define ULOG_USING_ISR_LOG
#define ULOG_ASSERT_ENABLE
/* 行缓冲 256: 业务最长日志行 ~127 字符 + ulog 头 (时间/级别/标签) */
#define ULOG_LINE_BUF_SIZE 256

/* log format */

#define ULOG_OUTPUT_FLOAT
#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG
/* end of log format */
#define ULOG_BACKEND_USING_CONSOLE

/* end of Utilities */

/* Using USB legacy version */

/* end of Using USB legacy version */
/* end of RT-Thread Components */

/* RT-Thread Utestcases */

/* end of RT-Thread Utestcases */

/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */

/* end of Marvell WiFi */

/* Wiced WiFi */

/* end of Wiced WiFi */

/* CYW43012 WiFi */

/* end of CYW43012 WiFi */

/* BL808 WiFi */

/* end of BL808 WiFi */

/* CYW43439 WiFi */

/* end of CYW43439 WiFi */
/* end of Wi-Fi */

/* IoT Cloud */

/* end of IoT Cloud */
/* end of IoT - internet of things */

/* security packages */

/* end of security packages */

/* language packages */

/* JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* end of JSON: JavaScript Object Notation, a lightweight data-interchange format */

/* XML: Extensible Markup Language */

/* end of XML: Extensible Markup Language */
/* end of language packages */

/* multimedia packages */

/* LVGL: powerful and easy-to-use embedded GUI library */

/* end of LVGL: powerful and easy-to-use embedded GUI library */

/* u8g2: a monochrome graphic library */

/* end of u8g2: a monochrome graphic library */
/* end of multimedia packages */

/* tools packages */

/* end of tools packages */

/* system packages */

/* enhanced kernel services */

/* end of enhanced kernel services */

/* acceleration: Assembly language or algorithmic acceleration packages */

/* end of acceleration: Assembly language or algorithmic acceleration packages */

/* CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

#define PKG_USING_CMSIS_CORE
#define PKG_USING_CMSIS_CORE_LATEST_VERSION
/* end of CMSIS: ARM Cortex-M Microcontroller Software Interface Standard */

/* Micrium: Micrium software products porting for RT-Thread */

/* end of Micrium: Micrium software products porting for RT-Thread */
/* end of system packages */

/* peripheral libraries and drivers */

/* HAL & SDK Drivers */

/* STM32 HAL & SDK Drivers */

#define PKG_USING_STM32H7_HAL_DRIVER
#define PKG_USING_STM32H7_HAL_DRIVER_LATEST_VERSION
#define PKG_USING_STM32H7_CMSIS_DRIVER
#define PKG_USING_STM32H7_CMSIS_DRIVER_LATEST_VERSION
/* end of STM32 HAL & SDK Drivers */

/* Infineon HAL Packages */

/* end of Infineon HAL Packages */

/* Kendryte SDK */

/* end of Kendryte SDK */

/* WCH HAL & SDK Drivers */

/* end of WCH HAL & SDK Drivers */

/* AT32 HAL & SDK Drivers */

/* end of AT32 HAL & SDK Drivers */

/* HC32 DDL Drivers */

/* end of HC32 DDL Drivers */

/* NXP HAL & SDK Drivers */

/* end of NXP HAL & SDK Drivers */

/* NUVOTON Drivers */

/* end of NUVOTON Drivers */

/* GD32 Drivers */

/* end of GD32 Drivers */
/* end of HAL & SDK Drivers */

/* sensors drivers */

/* end of sensors drivers */

/* touch drivers */

/* end of touch drivers */
/* end of peripheral libraries and drivers */

/* AI packages */

/* end of AI packages */

/* Signal Processing and Control Algorithm Packages */

/* end of Signal Processing and Control Algorithm Packages */

/* miscellaneous packages */

/* project laboratory */

/* end of project laboratory */

/* samples: kernel and components samples */

/* end of samples: kernel and components samples */

/* entertainment: terminal games and other interesting software packages */

/* end of entertainment: terminal games and other interesting software packages */
/* end of miscellaneous packages */

/* Arduino libraries */


/* Projects and Demos */

/* end of Projects and Demos */

/* Sensors */

/* end of Sensors */

/* Display */

/* end of Display */

/* Timing */

/* end of Timing */

/* Data Processing */

/* end of Data Processing */

/* Data Storage */

/* Communication */

/* end of Communication */

/* Device Control */

/* end of Device Control */

/* Other */

/* end of Other */

/* Signal IO */

/* end of Signal IO */

/* Uncategorized */

/* end of Arduino libraries */
/* end of RT-Thread online packages */
#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32H7
#define BOARD_SERIES_STM32_NUCLEO_144

/* Hardware Drivers Config */

/* Onboard Peripheral Drivers */

/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_USING_UART4
/* 2026-10-04 硬件定案: USART2 = 数传电台 (PD5 TX/PD6 RX, 中断收发),
 * USART3 = ELRS 遥控接收机 (PD8 TX/PD9 RX, 420000, RX DMA ping 环
 * DMA1_Stream6) —— 引脚见 Flight.ioc, rc_data/mavgcs 设备名对应。 */
#define BSP_USING_UART2
#define BSP_USING_UART3
#define BSP_UART4_RX_USING_DMA
#define BSP_UART1_TX_USING_DMA
#define BSP_UART3_RX_USING_DMA
/* serial v2 每串口缓冲 (board/Kconfig 菜单, V2 才导出):
 * UART1 控制台 rx 256; UART4 (UM982, PA0/PA1) 与 gnss_data.c 的
 * GNSS_RX_BUF_SZ/ping 尺寸保持一致 (打开前 CTRL_CONFIG 会再显式设置)。
 * UART1 TX DMA (DMA1_Stream4): tx_bufsz 非 0 即选 NO_BUFFER 模式, 发送
 * 直接从调用方缓冲起 DMA (零拷贝), bufsz 仅是准入门槛 (>=64), 不占 RAM。
 * console/FinSH 线程上下文输出同享 DMA; ISR/调度器未起/临界区上下文由
 * dev_serial_v2.c 自动回退轮询发送。 */
#define BSP_UART1_RX_BUFSIZE 256
#define BSP_UART1_TX_BUFSIZE 64
#define BSP_UART4_RX_BUFSIZE 4096
#define BSP_UART4_TX_BUFSIZE 0
#define BSP_UART4_DMA_PING_BUFSIZE 256
/* UART2 数传电台: rx 512 (MAVLink 上行低频小帧), tx 256 (环形 + 中断断续发)。
 * UART3 ELRS CRSF: rx 1024 + ping 256 (与 UART4 同款 DMA ping 环,
 * DMA1_Stream6), tx 0 (CRSF 下行遥测暂未实现, 打开即阻塞直写)。 */
#define BSP_UART2_RX_BUFSIZE 512
#define BSP_UART2_TX_BUFSIZE 256
#define BSP_UART3_RX_BUFSIZE 1024
#define BSP_UART3_TX_BUFSIZE 0
#define BSP_UART3_DMA_PING_BUFSIZE 256
#define BSP_USING_SPI
#define BSP_USING_SPI1
#define BSP_SPI1_TX_USING_DMA
#define BSP_SPI1_RX_USING_DMA
/* OCTOSPI1 QSPI 总线 (硬件 NCS=PG6, 间接模式轮询) */
#define BSP_USING_QSPI
#define BSP_USING_HARD_I2C
#define BSP_USING_HARD_I2C1
#define BSP_USING_HARD_I2C4
#define BSP_USING_HARD_I2C2
#define BSP_I2C1_TX_USING_INT 1
#define BSP_I2C1_RX_USING_INT 1
#define BSP_I2C4_TX_USING_INT 1
#define BSP_I2C4_RX_USING_INT 1
#define BSP_I2C2_TX_USING_INT 1
#define BSP_I2C2_RX_USING_INT 1
/* B5: I2C2 DMA 直写调用方栈缓冲, DCache 下不可维护 -> 回 IT/PIO
 * (100kHz 总线 12B 传输耗时不变, 语义同为阻塞 completion) */
/* 2026-10-01 修复两总线 "无响应": 5.3.0 drv_hard_i2c 的 TX/RX 后端全部由
 * BSP_I2C{x}_{TX,RX}_USING_{INT,DMA,POLL} 宏条件编译, 一个模式宏都没有
 * 时 init/IRQ 回调照常跑但 xfer 三个后端全不命中 -> 一切传输 (含写寄存器
 * 地址+读数据的两消息序列, seq 检查要求 TX+RX 异步后端) 直接失败, BMM350/
 * BMP585 探测恒超时。传感器读时序天然 repeated-start, POLL 不支持 seq,
 * 故按 B5 结论统一开 INT (TX+RX), 三总线同配。 */
/* 2026-10-02 BMM350 由 I2C1 重映射到 I2C4 (SCL=PF14/SDA=PF15)。
 * 2026-10-04 硬件定案再迁移: BMM350 -> I2C1 (SCL=PB8/SDA=PB7, INT PB5),
 * BMP585 -> I2C4 (SCL=PB6/SDA=PB9, INT PE1), I2C2 (PB10/PB11) 留给
 * 电流计; 三总线内核时钟同为 137.5MHz (D2PCLK1/D3PCLK1), TIMINGR 通用。 */
/* TIM2 空闲 (时间基座已移除) */
/* end of On-chip Peripheral Drivers */

/* Board extended module Drivers */

/* end of Hardware Drivers Config */

#endif
