/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include "board.h"

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 550 MHz is legal only with the programmed CPU frequency boost option.
   * Do not silently overclock or program nonvolatile option bytes at boot. */
  if ((FLASH->OPTSR2_CUR & FLASH_OPTSR2_CPUFREQ_BOOST) == 0u)
  {
    _Error_Handler("CPU_FREQ_BOOST required for Flight.ioc 550MHz", __LINE__);
  }

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
}

#ifdef BSP_USING_USBD
/* Board clock and pin setup for the USB device driver. */
rt_err_t board_usb_prepare(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};
    GPIO_InitTypeDef gpio = {0};

    /* 25 MHz HSE / 5 * 96 / 10 = 48 MHz on PLL3Q. */
    clock.PeriphClockSelection = RCC_PERIPHCLK_USB;
    clock.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    clock.PLL3.PLL3M = 5;
    clock.PLL3.PLL3N = 96;
    clock.PLL3.PLL3P = 2;
    clock.PLL3.PLL3Q = 10;
    clock.PLL3.PLL3R = 2;
    clock.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    clock.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
        return -RT_EIO;

    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &gpio);
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
    return RT_EOK;
}
#endif

#if defined(BSP_USING_FLIGHT_DRIVERS)
/* Board-wide MPU, cache and DMA clock setup. */

static int board_mpu_cache_init(void)
{
    ARM_MPU_Disable();
    ARM_MPU_SetRegionEx(0u, 0x24000000u,
        ARM_MPU_RASR(1u, ARM_MPU_AP_FULL, 1u, 0u, 1u, 1u, 0u, ARM_MPU_REGION_SIZE_512KB));
    /* H723 D2 SRAM: normal non-cacheable, shareable, execute-never. */
    ARM_MPU_SetRegionEx(1u, 0x30000000u,
        ARM_MPU_RASR(1u, ARM_MPU_AP_FULL, 1u, 1u, 0u, 0u, 0u, ARM_MPU_REGION_SIZE_32KB));
    ARM_MPU_SetRegionEx(2u, 0x40000000u,
        ARM_MPU_RASR(1u, ARM_MPU_AP_FULL, 0u, 1u, 0u, 1u, 0u, ARM_MPU_REGION_SIZE_512MB));
    ARM_MPU_SetRegionEx(3u, 0x20000000u,
        ARM_MPU_RASR(1u, ARM_MPU_AP_FULL, 1u, 0u, 0u, 0u, 0u, ARM_MPU_REGION_SIZE_128KB));
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
    SCB_EnableICache();
    SCB_EnableDCache();

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    return RT_EOK;
}
INIT_BOARD_EXPORT(board_mpu_cache_init);
#endif
