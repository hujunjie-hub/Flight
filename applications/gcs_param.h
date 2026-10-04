/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * QGC 参数协议 (PARAM_REQUEST_LIST/READ, PARAM_SET -> PARAM_VALUE)
 *
 * 把 W25Q64 参数域 (param_calib: sys/nav/calib) 映射成 MAVLink 参数表,
 * 供 QGC 参数页查看/修改 —— 阶段 2, FMT_README §13.4-13.5。实现在
 * gcs_param.c; mavgcs.c 负责拉起 (init) 与节拍驱动 (poll, 落盘防抖)。
 */

#ifndef __GCS_PARAM_H__
#define __GCS_PARAM_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

struct gcs_param_status
{
    rt_uint16_t count;          /* 参数表条目数 */
    rt_bool_t   save_pending;   /* 有已修改未落盘的参数 (防抖计时中) */
};

/* 注册 PARAM_* 报文处理器 (mavlink_link_attach); 幂等 */
rt_err_t gcs_param_init(void);

/* 节拍驱动: PARAM_SET 的落盘防抖到点后执行 W25Q64 保存 (mavgcs 线程
 * 每拍调用; 最后一次修改 3s 后落盘一次, 批量修改合并为一次写) */
void gcs_param_poll(void);

/* 状态快照 (FinSH `gcs` 显示用) */
void gcs_param_status(struct gcs_param_status *out);

#ifdef __cplusplus
}
#endif

#endif /* __GCS_PARAM_H__ */
