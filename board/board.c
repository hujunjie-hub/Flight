/*
 * Copyright (c) 2006-2024 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include "board.h"

#include "board.h"
#include <rtdevice.h>          /* serial_configure / RT_DEVICE_CTRL_CONFIG */

/*
 * DMA handles referenced by the CubeMX generated MSP
 * (board/CubeMX_Config/Src/stm32h7xx_hal_msp.c).
 *
 * CubeMX normally defines these in Core/Src/main.c, which is not part of this
 * BSP: the RT-Thread drivers (drv_spi.c / drv_hard_i2c.c / drv_usart.c) create
 * and link their own DMA handles, but HAL_xxx_MspInit() still points the
 * peripheral to these globals, so they have to exist to link.
 */
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart3_rx;   /* USART3 RX (ELRS CRSF), 同上占位 */

/*
 * NVIC preemption priorities (priority group 4) of every interrupt this board
 * uses.  The values come from board/CubeMX_Config/Flight.ioc, which is the
 * single source of truth for the hardware checklist.
 *
 * 2026-10-04 抢占优先级全表重排 (与 doc/Flight.xlsx 接口配置页一致):
 *   1  UM982 PPS (TIM2 输入捕获)          —— 时间同步最高
 *   2  ADIS DR EXTI + ADIS SPI1 DMA       —— 1kHz 惯导主观测链
 *   3  UM982 UART4 RX DMA / ELRS USART3 RX DMA —— 遥控链与 IMU 同级兜底
 *   5  BMM350/BMP585 INT+I2C EV / 数传 USART2 RX
 *   6  电流计 I2C2 DMA/EV / USART1 调试链
 *   7  其他非实时 (I2C 错误中断等)
 *
 * The RT-Thread STM32 drivers set hard-coded priorities when they initialize
 * (DMA RX=0, DMA TX=1, peripheral IRQ=1/2/3) and again when a device is opened
 * or a pin interrupt is enabled, so without this table the CubeMX priorities
 * would never take effect.  The drivers call board_nvic_set_priority() instead
 * of HAL_NVIC_SetPriority() (see libraries/HAL_Drivers/drv_common.h); the weak
 * default in drv_common.c keeps the driver's own value for interrupts that are
 * not listed here.
 */
void board_nvic_set_priority(IRQn_Type irq, uint32_t default_preempt)
{
    uint32_t preempt = default_preempt;

    switch (irq)
    {
    /* P1: UM982 1PPS 时间同步 (最高实时) */
    case TIM2_IRQn:         preempt = 1; break;   /* 1PPS input capture      */

    /* P2: ADIS16505 IMU 1kHz 观测链 (DR EXTI 由驱动宏
     * ADIS16505_DR_IRQ_PRIO=2 直设, 同级) */
    case EXTI4_IRQn:        preempt = 2; break;   /* DR  - data ready        */
    case SPI1_IRQn:         preempt = 2; break;   /* SPI1 global (原 0 越过 PPS, 已归链) */
    case DMA1_Stream0_IRQn: preempt = 2; break;   /* SPI1 RX (DOUT)          */
    case DMA1_Stream1_IRQn: preempt = 2; break;   /* SPI1 TX (DIN)           */

    /* P3: UM982 GNSS 观测链 (UART4 TX/RX 与 RX DMA 同向量族) */
    case UART4_IRQn:        preempt = 3; break;   /* UART4 TX/RX (UM982)     */
    case DMA1_Stream2_IRQn: preempt = 3; break;   /* UART4 RX (circular)     */

    /* P3: ELRS CRSF 遥控接收链 (USART3 与 RX DMA 同向量族) */
    case USART3_IRQn:       preempt = 3; break;   /* USART3 (ELRS)           */
    case DMA1_Stream6_IRQn: preempt = 3; break;   /* USART3 RX (circular)    */

    /* P5: BMM350 磁链路 (I2C1, INT PB5) / BMP585 气压链路 (I2C4, INT PE1)
     * 2026-10-04 迁移: 传感器 INT 自 EXTI15_10 (PF12/PE13) 拆到
     * EXTI9_5 (BMM350 PB5) 与 EXTI1 (BMP585 PE1), EXTI15_10 释放
     * (PE13 让位 TIM1_CH3 电调输出)。 */
    case EXTI9_5_IRQn:      preempt = 5; break;   /* BMM350 INT (PB5)         */
    case EXTI1_IRQn:        preempt = 5; break;   /* BMP585 INT (PE1)         */
    case I2C1_EV_IRQn:      preempt = 5; break;   /* I2C1 event (BMM350)      */
    case I2C4_EV_IRQn:      preempt = 5; break;   /* I2C4 event (BMP585)      */

    /* P5: 数传电台 (MAVLink GCS) */
    case USART2_IRQn:       preempt = 5; break;   /* USART2 (数传)            */

    /* P6: 电流计 I2C2 链 (DMA2 Stream0/1 保留) */
    case I2C2_EV_IRQn:      preempt = 6; break;   /* I2C2 event (电流计)      */
    case DMA2_Stream0_IRQn: preempt = 6; break;   /* I2C2 RX                  */
    case DMA2_Stream1_IRQn: preempt = 6; break;   /* I2C2 TX                  */

    /* P6: USART1 debug console / 数据链路 */
    case USART1_IRQn:       preempt = 6; break;   /* USART1                  */
    case DMA1_Stream3_IRQn: preempt = 6; break;   /* USART1 RX               */
    case DMA1_Stream4_IRQn: preempt = 6; break;   /* USART1 TX               */

    /* P7: 其他非实时 (I2C 总线错误中断, drv_hard_i2c 经本表设置) */
    case I2C1_ER_IRQn:      preempt = 7; break;
    case I2C2_ER_IRQn:      preempt = 7; break;
    case I2C4_ER_IRQn:      preempt = 7; break;

    default:
        break;
    }

    HAL_NVIC_SetPriority(irq, preempt, 0);
}

/*
 * USART1 (console / VOFA / imuout 数据口, PA9/PA10) 波特率 460800 8N1,
 * 与 Flight.ioc 的 USART1.BaudRate 保持一致。
 *
 * 串口驱动注册设备时统一用 RT_SERIAL_CONFIG_DEFAULT (115200), console
 * 在 rt_hw_board_init() 内即被打开; 本 BOARD 级钩子随后 (RT-Thread 版本横幅
 * 打印前) 通过 RT_DEVICE_CTRL_CONFIG 重写 BRR, 之后所有输出均为该波特率。
 * 打开状态下重配是串口框架支持的操作 (serial v2 框架: 缓冲尺寸不变时
 * 仅应用波特率, 见 dev_serial_v2.c rt_serial_control 的 ref_count 分支),
 * HAL_UART_Init 只 MODIFY 特定 CR1 位, 不影响已使能的 RX 中断。
 * 配置先经 RT_SERIAL_CTRL_GET_CONFIG 取设备当前值再改波特率, 不依赖
 * RT_SERIAL_CONFIG_DEFAULT 的具体字段 (v1/v2 结构布局不同)。
 *
 * 实测 (2026-09, DAPLink CDC 虚拟串口 COM9): 115200/460800 收发干净,
 * 921600 只收到乱码 (桥接固件上限, 与负载无关; MCU 侧 BRR=149, +0.13%
 * 误差本身正常)。换 FTDI/CP210x 等适配器想跑 921600 时, 改回
 * BAUD_RATE_921600 并同步改 Flight.ioc 与 main.c 的 IMUOUT_BAUD_TEXT。
 */
static int board_uart1_baud_init(void)
{
    rt_device_t dev = rt_device_find("uart1");
    struct serial_configure cfg;

    if (dev == RT_NULL)
        return -RT_ERROR;

    if (rt_device_control(dev, RT_SERIAL_CTRL_GET_CONFIG, &cfg) != RT_EOK)
        return -RT_ERROR;

    cfg.baud_rate = BAUD_RATE_460800;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    return 0;
}
INIT_BOARD_EXPORT(board_uart1_baud_init);

/**
  * @brief System Clock Configuration
  *
  * 按 Flight.ioc (硬件设计单一事实来源) 配置完整时钟树:
  *   HSE 25MHz -> PLL1 (M=2, N=44) -> VCO 550MHz
  *     P=1 -> SYSCLK/CPU = 550MHz (VOS0)
  *     Q=9 -> SPI1/2/3 内核时钟 = 61.111MHz (/64 = 954.861kHz SCLK)
  *   HPRE=/2  -> AHB  275MHz
  *   APB1..4=/2 -> 137.5MHz (USART/I2C/TIM PCLK)
  *   HSE CSS 使能
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage (550MHz 需 VOS0)
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {}

  /** Initializes the RCC Oscillators (HSE 25MHz + PLL1)
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;   /* fREF = 12.5MHz */
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;   /* VCO = 550MHz   */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Clock Security System on HSE
  */
  HAL_RCC_EnableCSS();
}
