/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 位置 MPC (外环控制器, 纯 C, 无 RT-Thread 依赖, 可主机测试/复用)
 *
 * ---------------------------------------------------------------------------
 * 控制结构 (见根 README "飞行控制" 章):
 *   本模块 (外环, ~20Hz):  NED 位置/速度 -> 期望加速度 a_cmd (NED, m/s^2)
 *   attitude_so3 (内环):   a_cmd -> (总推力 + 期望姿态 R_des), SO(3) 误差串级 PID
 *
 * ---------------------------------------------------------------------------
 * 模型与坐标约定
 * ---------------------------------------------------------------------------
 * - 导航系 NED (x=北, y=东, z=地, D 轴向下为正), 与 gins_solution 一致。
 * - 每轴独立双积分器:  p' = v,  v' = u  (u = 该轴期望加速度);
 *   离散化 (ZOH): p_{k+1} = p_k + v_k dt + (1/2) u_k dt^2
 *                  v_{k+1} = v_k + u_k dt
 * - a_cmd 语义是"比力之外的总加速度指令", 悬停 = 0; 向上加速 = a_z < 0
 *   (D 轴向下)。内环按  z_b_des = normalize(g_vec - a_des)  映射姿态,
 *   f = m*||g_vec - a_des||  映射总推力, 因此本模块不需要任何质量/姿态信息。
 *
 * ---------------------------------------------------------------------------
 * QP 形式 (凝结/condensed)
 * ---------------------------------------------------------------------------
 * 预测: X = Phi*x0 + Gamma*U  (N 步, U = [u_0..u_{N-1}] 逐轴独立)
 * 代价: J = sum_{k=1..N-1} (p_k-p_ref)' Qp (.) + (v_k-v_ref)' Qv (.)
 *        + 终端 (x_N - x_ref)' P (.)  +  sum_{k=0..N-1} u_k' Ru_k
 *   P 由每轴 2 维 Riccati 迭代收敛得到 (LQR 终端权, 与 (Qp,Qv,Ru) 自洽)。
 * 凝结:  min (1/2) U' H U + g' U,  g = M (x0 - x_ref)
 *   H = Gamma'QbarGamma + Rbar (N x N, 正定), M = Gamma'QbarPhi (N x 2)。
 * 约束:  u_min <= u_k <= u_max  (逐轴, 每步相同)。
 *
 * 求解: 序贯坐标法 SCA (投影 Gauss-Seidel, 框约束 QP 的标准解法):
 *   u_i <- clamp( u_i - (g_i + (Hu)_i)/H_ii ), 逐分量一轮为一次 sweep;
 *   投影梯度 inf 范数 < tol 提前收敛。迭代点恒可行 (每次更新都 clamp),
 *   到达 max_sweeps 未收敛也输出当前可行解 (状态字段标注)。
 *   热启动: 上一拍解左移一步 (末位重复)。
 *
 * ---------------------------------------------------------------------------
 * 数值与确定性
 * ---------------------------------------------------------------------------
 * - 全程 double (Cortex-M7 FPUv5 双精度); 无动态内存; 单次 step 耗时确定
 *   (sweeps 上限固定), 板上预算见模块 README。
 * - 输入非有限值 (NaN/Inf, 导航解坏值) 时返回负值, 输出清零并保持上拍
 *   热启动缓冲区不动; 调用方须检查返回值。
 */
#ifndef __MPC_POS_H__
#define __MPC_POS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_POS_N_MAX       16      /* 预测步数上限 (内存按此静态分配) */
#define MPC_POS_AXES        3       /* NED 三轴 */

/* 返回码 */
#define MPC_POS_OK          0
#define MPC_POS_ECFG       -1       /* 配置非法 (setup 校验失败) */
#define MPC_POS_EINPUT     -2       /* step 输入非有限值 */
#define MPC_POS_ENOTSETUP  -3       /* 未 setup 即 step */

struct mpc_pos_cfg
{
    int     N;                  /* 预测步数, [2, MPC_POS_N_MAX] */
    double  dt;                 /* 离散步长 = 外环周期, s (>0) */
    double  q_pos[3];           /* 位置权 (逐 NED 轴), >0 */
    double  q_vel[3];           /* 速度权, >=0 */
    double  r_acc[3];           /* 输入 (加速度) 权, >0 */
    double  u_min[3];           /* 逐轴加速度下界, m/s^2 (z 轴向下为正) */
    double  u_max[3];           /* 上界, 须 u_min < 0 < u_max 否则无效 */
    int     max_sweeps;         /* SCA 每轴 sweep 上限, >=1 */
    double  tol_grad;           /* 投影梯度收敛容差 (>0, 建议同量级 1e-4) */
};

struct mpc_pos_status
{
    int       sweeps;           /* 最近一拍实际 sweep 数 (三轴最大) */
    double    res_grad;         /* 收敛时残差 / 未收敛时剩余投影梯度 */
    double    cost;             /* 最近一拍最优代价 J */
    uint32_t  step_cnt;         /* 累计 step 次数 */
    uint32_t  stall_cnt;        /* 达到 max_sweeps 未收敛的次数 */
    uint32_t  bad_cnt;          /* 输入非有限被拒绝次数 */
};

struct mpc_pos_out
{
    double  a_cmd[3];           /* u_0 + a_ff (再 clamp 到界内), NED m/s^2 */
    double  u_pred[3 * MPC_POS_N_MAX];   /* 计划加速度序列 (逐拍 [x y z]),
                                          * 调试/日志用, 有效长度 3*N */
    struct mpc_pos_status st;
};

struct mpc_pos_ctx
{
    struct mpc_pos_cfg cfg;
    /* 预计算 (逐轴独立): H = N x N, M = N x 2 (行主序) */
    double  H[MPC_POS_AXES][MPC_POS_N_MAX * MPC_POS_N_MAX];
    double  M[MPC_POS_AXES][MPC_POS_N_MAX * 2];
    double  lb[MPC_POS_AXES][MPC_POS_N_MAX];      /* = u_min[axis] 重复 */
    double  ub[MPC_POS_AXES][MPC_POS_N_MAX];
    /* 求解器工作区 / 热启动缓冲 */
    double  U[MPC_POS_AXES][MPC_POS_N_MAX];
    int     has_sol;
    struct mpc_pos_status st;
};

/*
 * 校验配置并预计算 H/M/界 (Riccati 终端权在内部求解)。
 * 修改 cfg 后必须重新调用; step 热路径不做重算。返回 MPC_POS_OK 或 MPC_POS_ECFG。
 * 注意: 非线程安全 (与 step 并发改配置属调用方责任)。
 */
int mpc_pos_setup(struct mpc_pos_ctx *ctx, const struct mpc_pos_cfg *cfg);

/* 清空热启动解 (设定值大跳变 / 重新使能时调用) */
void mpc_pos_reset(struct mpc_pos_ctx *ctx);

/*
 * 一拍 MPC:
 *   p/v      当前 NED 位置 (m) / 速度 (m/s)
 *   p_ref/v_ref  位置/速度设定 (恒值预测; v_ref 常取 0 或制导值)
 *   a_ff     前馈加速度 (直接加到 u_0 输出并整体 clamp, 不进预测模型;
 *            置 NULL 视为 0)
 * 输入非有限返回 MPC_POS_EINPUT (out 清零, 内部状态不动); 否则 MPC_POS_OK。
 */
int mpc_pos_step(struct mpc_pos_ctx *ctx,
                 const double p[3], const double v[3],
                 const double p_ref[3], const double v_ref[3],
                 const double a_ff[3],
                 struct mpc_pos_out *out);

/* 缺省配置 (N=10, dt=0.05 -> 0.5s 视界; 权重为安全保守初值, 需现场调参) */
extern const struct mpc_pos_cfg mpc_pos_cfg_default;

#ifdef __cplusplus
}
#endif

#endif /* __MPC_POS_H__ */
