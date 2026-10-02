/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     zylx         first version
 */

#ifndef __DRV_QSPI_H__
#define __DRV_QSPI_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t rt_hw_qspi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin, rt_uint8_t data_line_width, void (*enter_qspi_mode)(), void (*exit_qspi_mode)());

#ifdef __cplusplus
}
#endif

#endif /* __DRV_QSPI_H__ */

/* HAL 直连诊断探针 (w25q64 dbg 调用; 仅诊断, 发一次 JEDEC 传输) */
void stm32_qspi_bus_diag(void);
