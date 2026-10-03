/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 内环姿态控制器: SO(3) 姿态误差 + 串级 PID (纯 C, 仅依赖 middleware/so3,
 * 无 RT-Thread 依赖, 可主机测试/复用)
 *
 * ---------------------------------------------------------------------------
 * 控制结构 (见根 README "飞行控制" 章)
 * ---------------------------------------------------------------------------
 *   期望加速度 a_des (NED, 来自 position_mpc 外环)
 *     -> 总推力方向  z_b_des = normalize(g_vec - a_des),  g_vec = [0,0,+g0]
 *        (FRD 体轴 z 朝下, 电机推力沿 -z_b; NED 悬停时 z_b_des = [0,0,1])
 *     -> 总推力大小  f = m * ||g_vec - a_des||
 *     -> 期望姿态 R_des: 由 z_b_des 与期望航向 yaw_des 正交化构造
 *          y_b = unit(z_b_des x x_c),  x_c = [cos(yaw), sin(yaw), 0]
 *          x_b = y_b x z_b_des,  C_bd_n = [x_b | y_b | z_b_des]
 *     -> SO(3) 姿态误差 e_b (middleware/so3, 期望体轴系表达)
 *     -> 角度环 P:  omega_des = Kp_att * e_b  (e_b 单位 rad, P 增益 1/s)
 *     -> 角速度环 PID: alpha_b = Kp*e_w + Ki*int(e_w) - Kd*d(omega)/dt|filt
 *        (D 项作用于测量微分, 一阶低通; 条件积分抗饱和)
 *
 * 输出 alpha_b 语义为"角加速度指令"(rad/s^2, 期望体轴系), 与转动惯量解耦;
 * 混控层按机体转动惯量/力臂换算为电机差动 (N*m), 见模块 README。
 *
 * ---------------------------------------------------------------------------
 * 安全限幅 (本层保证的输出性质)
 * ---------------------------------------------------------------------------
 * - 倾斜限幅: z_b_des 与铅垂 [0,0,1] 夹角超过 tilt_max 时投影回锥内
 *   (推力大小按未限幅方向计算, 姿态按限幅方向构造);
 * - omega_des 逐轴限幅; alpha_b 逐轴限幅; 推力 clamp [thrust_min, thrust_max];
 * - 输入非有限 / dt 非法: valid=0, 输出全部清零且控制器状态复位 —— 电机层
 *   见 valid=0 必须停转, 不得把 0 当悬停推力。
 */
#ifndef __ATT_PID_H__
#define __ATT_PID_H__

#include "so3.h"

#ifdef __cplusplus
extern "C" {
#endif

struct att_pid_cfg
{
    double  mass;               /* kg, 推力换算用 (>0) */
    double  g0;                 /* 重力加速度 m/s^2, 默认 9.80665 */
    double  kp_att[3];          /* 角度环 P (1/s), omega_des = Kp*e_b */
    double  kp_rate[3];         /* 角速度环 P (1/s) */
    double  ki_rate[3];         /* I (1/s^2) */
    double  kd_rate[3];         /* D (无量纲, 作用于 d(omega)/dt) */
    double  d_cutoff_hz;        /* D 项一阶低通截止 (Hz, >0) */
    double  tilt_max_rad;       /* 最大倾斜角 (rad, (0, pi/2)) */
    double  omega_max;          /* omega_des 逐轴限幅 (rad/s, >0) */
    double  alpha_max[3];       /* alpha_b 逐轴限幅 (rad/s^2, >0) */
    double  thrust_min;         /* N, >=0 (默认 0 = 允许动力掉到 0) */
    double  thrust_max;         /* N, > thrust_min */
    double  int_hold_e;         /* |e_omega| 超过此值冻结积分 (rad/s, >0) */
};

struct att_pid_state
{
    double  integ[3];           /* 角速度环积分器 (rad) */
    double  dwf[3];             /* D 项低通后的 d(omega)/dt (rad/s^2) */
    double  wprev[3];           /* 上拍角速度 (D 项用) */
    int     has_prev;
};

struct att_pid_in
{
    double  a_des[3];           /* 期望加速度, NED m/s^2 (外环输出) */
    double  yaw_des;            /* 期望航向 rad */
    struct so3_quat q_cur;      /* 当前姿态 (体->导航, KF-GINS 融合) */
    double  omega_b[3];         /* 机体角速度 rad/s (FRD, 陀螺) */
    double  dt;                 /* 本拍步长 s */
};

struct att_pid_out
{
    int             valid;      /* 0 = 输入非法/未就绪, 其余字段全零 */
    double          thrust_n;   /* 总推力 N (已 clamp) */
    double          thrust_norm;/* 归一化推力 f/(m*g0), 悬停=1 */
    double          alpha_b[3]; /* 角加速度指令 rad/s^2 (期望体轴系) */
    double          omega_des[3];/* 角度环输出 rad/s */
    struct so3_quat q_des;      /* 期望姿态 (体->导航) */
    double          rpy_des[3]; /* 期望欧拉角 rad (日志用) */
    double          e_b[3];     /* SO(3) 体轴误差 rad */
    double          att_err_deg;/* 总姿态误差角 deg */
    int             tilt_limited; /* 1 = a_des 方向超出倾斜锥被投影 */
};

struct att_pid_ctx
{
    struct att_pid_cfg  cfg;
    struct att_pid_state st;
};

/* 使用约定: ctx 须先清零 (静态定义 / memset), 再赋 cfg —— 增益无预计算,
 * 但积分器与 D 项历史零初始化是正确性前提 (att_pid_reset 等效)。 */

/* a_des/yaw -> 期望姿态 (含倾斜限幅)。thrust_dir_n 为限幅后的推力方向
 * (即 z_b_des, NED 单位矢量)。输入非有限返回 -1, 否则 0。 */
int att_thrust_to_attitude(const double a_des[3], double yaw_des,
                           double tilt_max, double g0,
                           struct so3_quat *q_des, double rpy_des[3],
                           double thrust_dir_n[3], int *tilt_limited);

/* 总推力 (未 clamp): f = m*||g_vec - a_des||; 输入非有限返回 -1 */
int att_thrust_newton(double mass, double g0, const double a_des[3],
                      double *f_out);

/* 复位积分器/微分历史 (重新使能 / 目标大跳变后调用) */
void att_pid_reset(struct att_pid_ctx *ctx);

/* 一拍内环。返回 0 且 out->valid=1 为正常; 输入非法返回 -1 (out 清零,
 * 内部状态复位)。 */
int att_pid_step(struct att_pid_ctx *ctx, const struct att_pid_in *in,
                 struct att_pid_out *out);

/* 缺省配置 (安全保守初值, 增益需按机体调参) */
extern const struct att_pid_cfg att_pid_cfg_default;

#ifdef __cplusplus
}
#endif

#endif /* __ATT_PID_H__ */
