CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m7)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_C_COMPILER "C:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_ASM_COMPILER "C:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_C_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O2 -gdwarf-2 -g -DNDEBUG -std=c99 -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type")
SET(CMAKE_ASM_FLAGS " -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_COMPILER "C:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-g++.exe")
# 注意: 工程定义了 -D_POSIX_C_SOURCE=1, newlib 会隐藏 M_PI 等杂项宏
# (KF-GINS 依赖 M_PI); 这里直接命令行补上, 不要用 _DEFAULT_SOURCE
# (会连带放开 POSIX 头, 与 RT-Thread 的 sys/time.h 垫片冲突)
SET(CMAKE_CXX_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O2 -gdwarf-2 -g -std=c++17 -fno-exceptions -fno-rtti -DNDEBUG -DKF_GINS_EMBEDDED -DM_PI=3.14159265358979323846 -ffp-contract=fast -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_OBJCOPY "C:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-objcopy.exe")
SET(CMAKE_SIZE "C:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-size.exe")

SET(CMAKE_EXE_LINKER_FLAGS " -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=Flight.map,-cref,-u,Reset_Handler -T ${CMAKE_SOURCE_DIR}/board/linker_scripts/link.lds -Wl,--gc-sections,--print-memory-usage")

SET(CMAKE_C_STANDARD 99)
SET(CMAKE_CXX_STANDARD 17)

PROJECT(rtthread C CXX ASM)

# 最终固件目标名: Flight.elf (调试配置/烧录脚本引用此名; bin 为 Flight.bin)
SET(FIRMWARE_TARGET Flight.elf)

INCLUDE_DIRECTORIES(
	.
	applications
	board
	board/CubeMX_Config/Inc
	middleware/sensor
	middleware/protocol/nmea
	middleware/protocol/mavlink
	middleware/data
	middleware/gins
	middleware/control/so3
	middleware/calibration
	middleware/control/position_mpc
	middleware/control/attitude_so3
	middleware/control/control_allocation
	middleware/control/dshot_output
	middleware/param_calib
	middleware/timebase
	middleware/KF-GINS/src
	middleware/KF-GINS/ThirdParty/eigen-3.3.9
	packages/CMSIS-Core-latest/Include
	packages/stm32h7_cmsis_driver-latest/Include
	packages/stm32h7_hal_driver-latest/Inc
	packages/stm32h7_hal_driver-latest/Inc/Legacy
	rt-thread/include
	rt-thread/libcpu/arm/common
	rt-thread/libcpu/arm/cortex-m7
	rt-thread/components/drivers/include
	rt-thread/components/drivers/phy
	rt-thread/components/drivers/smp_call
	rt-thread/components/finsh
	rt-thread/components/utilities/ulog
	rt-thread/components/libc/compilers/common/include
	rt-thread/components/libc/compilers/newlib
	rt-thread/components/libc/posix/io/epoll
	rt-thread/components/libc/posix/io/eventfd
	rt-thread/components/libc/posix/io/poll
	rt-thread/components/libc/posix/ipc
	libraries/HAL_Drivers/drivers
	libraries/HAL_Drivers/drivers/config
	libraries/HAL_Drivers
)

ADD_DEFINITIONS(
	-DSTM32H723xx
	-DUSE_HAL_DRIVER
	-D__RTTHREAD__
	-DRT_USING_NEWLIBC
	-DRT_USING_LIBC
	-D_POSIX_C_SOURCE=1
	-DARM_MATH_CM7
)

# Library source files
# 应用层: main 入口 + USART1 调试输出层 (app_out 公共设施 + 六条 out_* 链路
# + cmd_* 辅助命令); 与 applications/SConscript 的 Glob('*.c') 保持一致
SET(RT_APPLICATIONS_SOURCES
	applications/main.c
	applications/app_out.c
	applications/out_vofa.c
	applications/out_imu.c
	applications/out_gnss.c
	applications/out_gins.c
	applications/out_mag.c
	applications/out_baro.c
	applications/cmd_cpuload.c
)

SET(RT_CMSIS-CORE_SOURCES
)

SET(RT_COMPILER_SOURCES
	rt-thread/components/libc/compilers/common/cctype.c
	rt-thread/components/libc/compilers/common/cstdlib.c
	rt-thread/components/libc/compilers/common/cstring.c
	rt-thread/components/libc/compilers/common/ctime.c
	rt-thread/components/libc/compilers/common/cunistd.c
	rt-thread/components/libc/compilers/common/cwchar.c
	rt-thread/components/libc/compilers/newlib/syscalls.c
)

SET(RT_DEVICEDRIVERS_SOURCES
	rt-thread/components/drivers/core/device.c
	rt-thread/components/drivers/ipc/completion_comm.c
	rt-thread/components/drivers/ipc/completion_up.c
	rt-thread/components/drivers/ipc/condvar.c
	rt-thread/components/drivers/ipc/dataqueue.c
	rt-thread/components/drivers/ipc/pipe.c
	rt-thread/components/drivers/ipc/ringblk_buf.c
	rt-thread/components/drivers/ipc/ringbuffer.c
	rt-thread/components/drivers/ipc/waitqueue.c
	rt-thread/components/drivers/ipc/workqueue.c
	rt-thread/components/drivers/pin/dev_pin.c
	# 串口框架 V2 (rtconfig.h: RT_USING_SERIAL_V2), 与 drivers/SConscript 的
	# GetDepend(RT_USING_SERIAL_V2) 选择保持同步
	rt-thread/components/drivers/serial/dev_serial_v2.c
)

#
# Optional sources driven by rtconfig.h: every driver registered in
# board/Kconfig (menuconfig) is compiled only when it has been selected.
# Keep the conditions in sync with libraries/HAL_Drivers/drivers/SConscript.
#
FILE(STRINGS ${CMAKE_SOURCE_DIR}/rtconfig.h RTT_RTCONFIG_DEFINES
	REGEX "^#define[ \t]+[A-Za-z0-9_]+")

MACRO(RTT_IS_ENABLED SYMBOL RESULT)
	IF(RTT_RTCONFIG_DEFINES MATCHES "#define[ \t]+${SYMBOL}([ \t;]|$)")
		SET(${RESULT} TRUE)
	ELSE()
		SET(${RESULT} FALSE)
	ENDIF()
ENDMACRO()

RTT_IS_ENABLED(RT_USING_CPLUSPLUS RTT_USING_CPLUSPLUS)
RTT_IS_ENABLED(RT_USING_SPI RTT_USING_SPI)
RTT_IS_ENABLED(RT_USING_QSPI RTT_USING_QSPI)
RTT_IS_ENABLED(RT_USING_I2C RTT_USING_I2C)
RTT_IS_ENABLED(RT_USING_SENSOR RTT_USING_SENSOR)
RTT_IS_ENABLED(RT_USING_SENSOR_CMD RTT_USING_SENSOR_CMD)
RTT_IS_ENABLED(RT_USING_PIN RTT_USING_PIN)
RTT_IS_ENABLED(RT_USING_SERIAL RTT_USING_SERIAL)
RTT_IS_ENABLED(RT_USING_ULOG RTT_USING_ULOG)
RTT_IS_ENABLED(ULOG_BACKEND_USING_CONSOLE RTT_ULOG_BACKEND_CONSOLE)
RTT_IS_ENABLED(RT_KLIBC_USING_VSNPRINTF_STANDARD RTT_KLIBC_VSNPRINTF_STANDARD)
RTT_IS_ENABLED(BSP_USING_UART BSP_USING_UART)
RTT_IS_ENABLED(BSP_USING_TIM BSP_USING_TIM)

IF(RTT_USING_SPI)
	LIST(APPEND RT_DEVICEDRIVERS_SOURCES
		rt-thread/components/drivers/spi/dev_spi_core.c
		rt-thread/components/drivers/spi/dev_spi.c
	)
ENDIF()

# QSPI 总线框架核心 (drv_qspi.c 在下方 RT_DRIVERS_SOURCES 侧)
IF(RTT_USING_QSPI)
	LIST(APPEND RT_DEVICEDRIVERS_SOURCES
		rt-thread/components/drivers/spi/dev_qspi_core.c
	)
ENDIF()

IF(RTT_USING_I2C)
	LIST(APPEND RT_DEVICEDRIVERS_SOURCES
		rt-thread/components/drivers/i2c/dev_i2c_core.c
		rt-thread/components/drivers/i2c/dev_i2c_dev.c
	)
ENDIF()

	IF(RTT_USING_SENSOR)
		LIST(APPEND RT_DEVICEDRIVERS_SOURCES
			rt-thread/components/drivers/sensor/v1/sensor.c
			middleware/sensor/sensor_adis16505.c
		middleware/sensor/sensor_bmm350.c
		middleware/sensor/sensor_bmp585.c
			middleware/sensor/sensor_w25q64.c
			middleware/sensor/sensor_sdmmc.c
			middleware/protocol/nmea/um982_nmea.c
		middleware/protocol/mavlink/mavlink_link.c
		middleware/protocol/mavlink/mavlink_helpers.c
		middleware/data/record_ring.c
		middleware/data/imu_data.c
		middleware/data/mag_data.c
		middleware/data/baro_data.c
		middleware/data/gnss_raw_data.c
		middleware/data/gnss_data.c
	)
	IF(RTT_USING_SENSOR_CMD)
		LIST(APPEND RT_DEVICEDRIVERS_SOURCES
			rt-thread/components/drivers/sensor/v1/sensor_cmd.c
		)
	ENDIF()
ENDIF()

# 5.3.0 已移除 components/drivers/hwtimer (并入 clock_time 框架);
# 本项目时基由 middleware/timebase 直用 HAL TIM, 不受影响。

SET(RT_DRIVERS_SOURCES
	board/CubeMX_Config/Src/stm32h7xx_hal_msp.c
	board/board.c
	middleware/timebase/timebase.c
	libraries/HAL_Drivers/drv_common.c
	# 5.3.0 BSP: 串口/SPI/I2C 驱动的 DMA 配置统一在 drv_dma.c (stm32_dma_setup/deinit)
	libraries/HAL_Drivers/drivers/drv_dma.c
)

IF(RTT_USING_PIN)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_gpio.c
	)
ENDIF()

IF(RTT_USING_SERIAL AND BSP_USING_UART)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_usart_v2.c
	)
ENDIF()

IF(RTT_USING_SPI)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_spi.c
	)
ENDIF()

# STM32H7 OCTOSPI QSPI 总线驱动 (间接模式轮询, 硬件 NCS)
IF(RTT_USING_QSPI)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_qspi.c
	)
ENDIF()

IF(RTT_USING_I2C)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_hard_i2c.c
	)
ENDIF()

IF(BSP_USING_TIM)
	LIST(APPEND RT_DRIVERS_SOURCES
		libraries/HAL_Drivers/drivers/drv_tim.c
	)
ENDIF()

SET(RT_FINSH_SOURCES
	rt-thread/components/finsh/msh_parse.c
	rt-thread/components/finsh/shell.c
	rt-thread/components/finsh/cmd.c
	rt-thread/components/finsh/msh.c
)

SET(RT_KERNEL_SOURCES
	rt-thread/src/clock.c
	rt-thread/src/components.c
	rt-thread/src/cpu_up.c
	rt-thread/src/defunct.c
	rt-thread/src/idle.c
	rt-thread/src/ipc.c
	rt-thread/src/irq.c
	rt-thread/src/kservice.c
	rt-thread/src/mem.c
	rt-thread/src/mempool.c
	rt-thread/src/object.c
	rt-thread/src/scheduler_comm.c
	rt-thread/src/scheduler_up.c
	rt-thread/src/thread.c
	rt-thread/src/timer.c
)

SET(RT_KLIBC_SOURCES
	rt-thread/src/klibc/rt_vsscanf.c
	rt-thread/src/klibc/kstdio.c
	rt-thread/src/klibc/kerrno.c
	rt-thread/src/klibc/kstring.c
)

#
# ulog 日志组件 (rtconfig.h: RT_USING_ULOG); 开浮点时用标准版 rt_vsnprintf
# (支持 %f), 与 rt-thread/src/klibc/SConscript 的选择逻辑保持一致。
#
SET(RT_ULOG_SOURCES
)

IF(RTT_KLIBC_VSNPRINTF_STANDARD)
	LIST(APPEND RT_KLIBC_SOURCES rt-thread/src/klibc/rt_vsnprintf_std.c)
ELSE()
	LIST(APPEND RT_KLIBC_SOURCES rt-thread/src/klibc/rt_vsnprintf_tiny.c)
ENDIF()

IF(RTT_USING_ULOG)
	LIST(APPEND RT_ULOG_SOURCES rt-thread/components/utilities/ulog/ulog.c)
	IF(RTT_ULOG_BACKEND_CONSOLE)
		LIST(APPEND RT_ULOG_SOURCES
			rt-thread/components/utilities/ulog/backend/console_be.c
		)
	ENDIF()
ENDIF()

SET(RT_LIBCPU_SOURCES
	rt-thread/libcpu/arm/common/atomic_arm.c
	rt-thread/libcpu/arm/common/div0.c
	rt-thread/libcpu/arm/common/showmem.c
	rt-thread/libcpu/arm/cortex-m7/context_gcc.S
	rt-thread/libcpu/arm/cortex-m7/cpu_cache.c
	rt-thread/libcpu/arm/cortex-m7/cpuport.c
)

SET(RT_POSIX_SOURCES
)

SET(RT_STM32H7-CMSIS_SOURCES
	packages/stm32h7_cmsis_driver-latest/Source/Templates/gcc/startup_stm32h723xx.s
	packages/stm32h7_cmsis_driver-latest/Source/Templates/system_stm32h7xx.c
)

SET(RT_STM32H7-HAL_SOURCES
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_cryp_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_usart.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_pwr.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_cec.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_rcc_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_uart.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_uart_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_comp.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_dma_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_mdma.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_rng.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_sram.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_gpio.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_cryp.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_dma.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_pwr_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_rcc.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_crc.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_cortex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_crc_ex.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_ospi.c
	# middleware/timebase (TIM2 时基 + PPS 输入捕获) 独占 HAL TIM
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_tim.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_tim_ex.c
	# middleware/sensor SD 驱动 (miniTF, SDMMC1 轮询) 直用 HAL SD,
	# 不经 RT SDIO 框架; delayblock 仅高速时钟调谐需要, 不编
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_sd.c
	packages/stm32h7_hal_driver-latest/Src/stm32h7xx_ll_sdmmc.c
)

# HAL modules required by the drivers registered in board/Kconfig
IF(RTT_USING_I2C)
	LIST(APPEND RT_STM32H7-HAL_SOURCES
		packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_i2c.c
		packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_i2c_ex.c
	)
ENDIF()

IF(RTT_USING_SPI)
	LIST(APPEND RT_STM32H7-HAL_SOURCES
		packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_spi.c
		packages/stm32h7_hal_driver-latest/Src/stm32h7xx_hal_spi_ex.c
	)
ENDIF()

SET(RT_GINS_SOURCES
	middleware/KF-GINS/src/kf-gins/gi_engine.cpp
	middleware/KF-GINS/src/kf-gins/insmech.cpp
	middleware/gins/gins_bridge.cpp
	middleware/gins/aligned_new.cpp
	middleware/gins/kf_math.cpp
	middleware/gins/gins_wdt.c
)

SET(RT_SO3_SOURCES
	middleware/control/so3/so3.c
	middleware/control/so3/so3_gins.c
)

# 飞控控制层 (级联: 位置 MPC 外环 + SO3/PID 姿态内环 + 控制分配 + 电调输出),
# 与各子目录 SConscript 的 Glob('*.c') 保持一致
SET(RT_CONTROL_SOURCES
	middleware/control/position_mpc/mpc_pos.c
	middleware/control/position_mpc/mpc_pos_gins.c
	middleware/control/attitude_so3/att_pid.c
	middleware/control/attitude_so3/att_pid_gins.c
	middleware/control/control_allocation/mixer.c
	middleware/control/control_allocation/mixer_cmd.c
	middleware/control/dshot_output/dshot_enc.c
	middleware/control/dshot_output/dshot.c
)

SET(RT_CALIBRATION_SOURCES
	middleware/calibration/ellipsoid_fit.c
	middleware/calibration/mag_calib.c
	middleware/calibration/baro_calib.c
)

# 飞控参数分区存储 (W25Q64): 分区引擎 + calib/nav/sys 三个参数域,
# 与 middleware/param_calib/SConscript 的 Glob('*.c') 保持一致
SET(RT_PARAMCALIB_SOURCES
	middleware/param_calib/param_part.c
	middleware/param_calib/param_calib.c
	middleware/param_calib/param_nav.c
	middleware/param_calib/param_sys.c
)

# MAVLink 官方生成代码: packed 结构 encode 函数触发已知告警, 按惯例
# 仅对包含 mavlink_link.h 的编译单元抑制
SET_SOURCE_FILES_PROPERTIES(
	middleware/protocol/mavlink/mavlink_link.c
	middleware/protocol/mavlink/mavlink_helpers.c
	PROPERTIES COMPILE_OPTIONS "-Wno-address-of-packed-member")

# Library search paths
# Library local macro definitions
SET(RT_DEVICEDRIVERS_DEFINES
	__RT_IPC_SOURCE__
)

SET(RT_KERNEL_DEFINES
	__RT_KERNEL_SOURCE__
)

# C++ 运行时: 全局构造 (__ctors_start/end 遍历) + operator new -> rt_malloc
IF(RTT_USING_CPLUSPLUS)
	LIST(APPEND RT_COMPILER_SOURCES
		rt-thread/components/libc/cplusplus/cxx_crt_init.c
		rt-thread/components/libc/cplusplus/cxx_crt.cpp
	)
ENDIF()

# Library dependencies
SET(RT_COMPILER_LIBS
	c
	m
)

# Libraries
ADD_LIBRARY(rtt_Compiler OBJECT ${RT_COMPILER_SOURCES})
ADD_LIBRARY(rtt_DeviceDrivers OBJECT ${RT_DEVICEDRIVERS_SOURCES})
ADD_LIBRARY(rtt_Drivers OBJECT ${RT_DRIVERS_SOURCES})
ADD_LIBRARY(rtt_Finsh OBJECT ${RT_FINSH_SOURCES})
ADD_LIBRARY(rtt_Kernel OBJECT ${RT_KERNEL_SOURCES})
ADD_LIBRARY(rtt_klibc OBJECT ${RT_KLIBC_SOURCES})
IF(RTT_USING_ULOG)
	ADD_LIBRARY(rtt_Ulog OBJECT ${RT_ULOG_SOURCES})
ENDIF()

ADD_LIBRARY(rtt_libcpu OBJECT ${RT_LIBCPU_SOURCES})
ADD_LIBRARY(rtt_STM32H7-CMSIS OBJECT ${RT_STM32H7-CMSIS_SOURCES})
ADD_LIBRARY(rtt_STM32H7-HAL OBJECT ${RT_STM32H7-HAL_SOURCES})
ADD_LIBRARY(rtt_GINS OBJECT ${RT_GINS_SOURCES})
ADD_LIBRARY(rtt_SO3 OBJECT ${RT_SO3_SOURCES})
ADD_LIBRARY(rtt_Control OBJECT ${RT_CONTROL_SOURCES})
ADD_LIBRARY(rtt_Calibration OBJECT ${RT_CALIBRATION_SOURCES})
ADD_LIBRARY(rtt_ParamCalib OBJECT ${RT_PARAMCALIB_SOURCES})

# Interface libraries
ADD_LIBRARY(rtt_CMSIS-Core INTERFACE)
ADD_LIBRARY(rtt_POSIX INTERFACE)

# Private macros
TARGET_COMPILE_DEFINITIONS(rtt_DeviceDrivers PRIVATE ${RT_DEVICEDRIVERS_DEFINES})
TARGET_COMPILE_DEFINITIONS(rtt_Kernel PRIVATE ${RT_KERNEL_DEFINES})

# GINS/Eigen 热路径: -O3 + FMA 融合 (覆盖全局 -O2, GCC 后者生效)
TARGET_COMPILE_OPTIONS(rtt_GINS PRIVATE -O3 -ffp-contract=fast)

# Interface library search paths
TARGET_LINK_LIBRARIES(rtt_Compiler INTERFACE ${RT_COMPILER_LIBS})

ADD_EXECUTABLE(${FIRMWARE_TARGET} ${RT_APPLICATIONS_SOURCES})
# 含 C++ 源 (KF-GINS/Eigen): 用 C++ 链接器以自动带上 libstdc++ 运行时
SET_TARGET_PROPERTIES(${FIRMWARE_TARGET} PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(${FIRMWARE_TARGET}
	rtt_Compiler
	rtt_DeviceDrivers
	rtt_Drivers
	rtt_Finsh
	rtt_GINS
	rtt_Kernel
	rtt_klibc
	rtt_libcpu
	$<$<BOOL:${RTT_USING_ULOG}>:rtt_Ulog>
	rtt_STM32H7-CMSIS
	rtt_STM32H7-HAL
	rtt_CMSIS-Core
	rtt_Calibration
	rtt_ParamCalib
	rtt_POSIX
	rtt_SO3
	rtt_Control
)

ADD_CUSTOM_COMMAND(TARGET ${FIRMWARE_TARGET} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${FIRMWARE_TARGET} Flight.bin
	COMMAND ${CMAKE_SIZE} ${FIRMWARE_TARGET}
)

# if custom.cmake is exist, add it
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
endif()
