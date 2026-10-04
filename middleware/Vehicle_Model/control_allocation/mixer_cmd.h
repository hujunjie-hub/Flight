/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 控制分配桥接接口 (实现见 mixer_cmd.c; 原声明只在 .c 内, 控制模型层
 * (middleware/Vehicle_Model/model) 接入后提出到头文件)
 */
#ifndef __MIXER_CMD_H__
#define __MIXER_CMD_H__

#include <rtthread.h>
#include "mixer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 一拍分配: 内环最新 (thrust_n, alpha_b) -> 电机推力/归一化量。
 * 内环无有效输出时 out 清零 valid=0 (电机层应停转)。
 */
rt_bool_t mixer_cmd_step_att(struct mixer_out *out);

/* 模块混控上下文 (FinSH `mix set` 整定作用于此; 勿与 step 并发改配置) */
struct mixer_ctx *mixer_cmd_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* __MIXER_CMD_H__ */
