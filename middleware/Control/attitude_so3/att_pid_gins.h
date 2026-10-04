/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 内环姿态控制器 <-> KF-GINS/IMU/SO3 桥接接口 (实现见 att_pid_gins.c 头注)
 */
#ifndef __ATT_PID_GINS_H__
#define __ATT_PID_GINS_H__

#include <rtthread.h>
#include "att_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 期望航向设定 (deg); 供制导/遥控层调用 */
void att_pid_gins_set_yaw_deg(double yaw_deg);

double att_pid_gins_get_yaw_deg(void);

/* 把期望航向设为当前航向 (航向保持) */
rt_bool_t att_pid_gins_hold_yaw(void);

/*
 * 一拍内环:
 *   当前姿态   <- so3_current_quat (KF-GINS 融合)
 *   角速度     <- imu_data_peek_latest (ADIS 陀螺, FRD rad/s)
 *   a_des      <- mpc_pos_gins_last_accel; 无效时取 0 (悬停姿态目标)
 *   dt         <- 相邻 IMU 样本 T_event 差, 异常时退回标称 2ms
 * 期望姿态同拍发布到 so3_target (so3 命令可直接观察误差)。
 * gins 未就绪返回 RT_FALSE (输出全零, valid=0 —— 电机层必须停转)。
 */
rt_bool_t att_pid_gins_step(struct att_pid_out *out);

/* 最近一拍输出快照 (关中断拷贝, 任意线程) */
void att_pid_gins_get_last(struct att_pid_out *out);

/* 复位积分器/微分历史 (重新使能时) */
void att_pid_gins_reset(void);

/* 模块控制器上下文 (调参/日志用; 勿与 step 并发改配置) */
struct att_pid_ctx *att_pid_gins_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* __ATT_PID_GINS_H__ */
