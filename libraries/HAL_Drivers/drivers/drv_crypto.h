/*
 * Copyright (c) 2019 Winner Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-10     Ernest       1st version
 */

#ifndef __DRV_CRYPTO_H__
#define __DRV_CRYPTO_H__

int l4_hw_crypto_device_init(void);

#ifdef BSP_USING_RNG
#include <hwcrypto.h>

/* Read one word with a separate status so zero remains a valid value. */
rt_err_t stm32_hwcrypto_rng_read(struct rt_hwcrypto_ctx *ctx, rt_uint32_t *value);
#endif

#endif /* __DRV_CRYPTO_H__ */
