/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 位置 MPC <-> KF-GINS 桥接接口 (实现见 mpc_pos_gins.c 头注)
 */
#ifndef __MPC_POS_GINS_H__
#define __MPC_POS_GINS_H__

#include <rtthread.h>
#include "mpc_pos.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 把当前 KF-GINS 位置捕为本地 NED 参考点 (原点); 设定点随之失效 */
void mpc_pos_gins_set_ref_here(void);

rt_bool_t mpc_pos_gins_ref_valid(void);

/*
 * 设定 NED 设定点 (相对参考点, m / m/s)。v_ned 为 NULL 时速度设定清零。
 * 首次调用且参考点未设时自动捕当前位姿为参考。内部会清 MPC 热启动。
 */
void mpc_pos_gins_set_sp(const double p_ned[3], const double v_ned[3]);

void mpc_pos_gins_get_sp(double p_ned[3], double v_ned[3]);

/*
 * 经纬高定点设定 (QGC DO_REPOSITION 通路): LLA -> 相对参考点 NED 后走
 * set_sp; 相对当前位置水平 >50m / 垂直 >10m 按界钳位 (拒绝台架跳点)。
 * 高度口径 = gins 椭球高 (与 GLOBAL_POSITION_INT 上报一致, QGC 往返自洽)。
 * gins 未就绪返回 -RT_ERROR。
 */
rt_err_t mpc_pos_gins_set_sp_lla(double lat_deg, double lon_deg, double alt_m);

/* 最近一拍 step 时的当前位置 NED m (摇杆叠加层用; 未 step 过为 0) */
void mpc_pos_gins_get_p_ned(double p_ned[3]);

/* 一拍外环 (读 KF-GINS 快照 -> 本地 NED -> mpc_pos_step)。
 * gins 未就绪/参考点或设定点未设/解降级时返回 RT_FALSE。 */
rt_bool_t mpc_pos_gins_step(struct mpc_pos_out *out);

/* 最近一拍输出快照 (关中断拷贝, 任意线程) */
void mpc_pos_gins_get_last(struct mpc_pos_out *out);

/* 内环取最近一拍期望加速度 (NED m/s^2); 无有效输出返回 RT_FALSE */
rt_bool_t mpc_pos_gins_last_accel(double a_des[3]);

/* 模块 MPC 上下文 (调参/日志用; 勿在 step 并发时改配置) */
struct mpc_pos_ctx *mpc_pos_gins_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPC_POS_GINS_H__ */
