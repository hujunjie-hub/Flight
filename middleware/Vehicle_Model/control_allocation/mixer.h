/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 控制分配 (混控) —— 纯 C 核心, 无 RT-Thread 依赖, 可主机测试/复用
 *
 * ---------------------------------------------------------------------------
 * 职责 (控制链末两级之一, 见根 README "飞行控制" 章)
 * ---------------------------------------------------------------------------
 *   输入: T_d      总推力 N (attitude_so3 输出 thrust_n)
 *         alpha_b  角加速度指令 rad/s² (期望体轴系, attitude_so3 输出)
 *   输出: 电机推力 f[4] (N) 与归一化 u[4] ∈ [0,1] (喂 dshot_output/PWM)
 *
 *   alpha_b --J(对角)--> 力矩 τ --> 四电机推力 (quad-X 解析逆) --> 饱和
 *
 * ---------------------------------------------------------------------------
 * 电机布局与符号约定 (quad-X, 体轴 FRD, x 前 y 右, 四臂 45°)
 * ---------------------------------------------------------------------------
 *   m0 前右 FR (x+,y+) 顺时针 CW (反扭矩 -z)
 *   m1 前左 FL (x+,y-) 逆时针 CCW (+z)
 *   m2 后右 RR (x-,y+) 逆时针 CCW (+z)
 *   m3 后左 RL (x-,y-) 顺时针 CW (反扭矩 -z)
 *
 * 正向映射 (推导见 mixer.c 头注):
 *   T   = Σ f_i
 *   τ_x = l/√2 · ( -f0 +f1 -f2 +f3 )
 *   τ_y = l/√2 · ( +f0 +f1 -f2 -f3 )
 *   τ_z = c_q  · ( -f0 +f1 +f2 -f3 )
 * 解析逆 (Mx = √2τx/l, My = √2τy/l, Mz = τz/c_q):
 *   f0 = (T - Mx + My - Mz)/4      f1 = (T + Mx + My + Mz)/4
 *   f2 = (T - Mx - My + Mz)/4      f3 = (T + Mx - My - Mz)/4
 *
 * 饱和: 逐电机 clamp [f_min, f_max] (简单裁剪, 记录饱和计数; 推力/姿态
 * 优先级协调留待需要时升级 —— 上游 PID 已有抗饱和, 短暂裁剪可接受)。
 *
 * ---------------------------------------------------------------------------
 * 参数现状 (占位)
 * ---------------------------------------------------------------------------
 * arm_l / tau_coeff / inertia / f_min / f_max 均为 500g 级四旋翼占位值,
 * **未按实际机架标定** —— 硬件方案落地后经 FinSH `mix set` 现场整定
 * (见本目录 README.md)。
 */
#ifndef __MIXER_H__
#define __MIXER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define MIXER_MOTORS    4       /* quad-X 固定四电机 */

struct mixer_cfg
{
    double  arm_l_m;        /* 力臂 m (电机轴线到 CG, 轴距 = l·√2) */
    double  tau_coeff;      /* 反扭矩系数 c_q, N·m/N (τ_z 每单位推力) */
    double  inertia[3];     /* 机体转动惯量对角 Jx/Jy/Jz, kg·m² (alpha→τ) */
    double  f_min_n;        /* 单电机推力下界 N (含起转推力; 0=允许停转) */
    double  f_max_n;        /* 单电机推力上界 N (按电机+桨静推力实测) */
};

struct mixer_out
{
    int     valid;          /* 0 = 输入非有限, 输出全零 (电机层应停转) */
    double  f_n[MIXER_MOTORS];  /* 电机推力 N (已 clamp) */
    double  u_norm[MIXER_MOTORS]; /* 归一化 [0,1], 输出层直接消费 */
    int     sat[MIXER_MOTORS]; /* 1 = 该电机触界 */
    int     n_sat;          /* 本拍触界电机数 */
};

struct mixer_ctx
{
    struct mixer_cfg cfg;
    struct
    {
        unsigned step_cnt;
        unsigned sat_cnt;   /* 累计出现触界的拍数 */
        unsigned bad_cnt;   /* 输入非有限被拒绝次数 */
    } st;
};

/* 校验配置 (>0 界); OK 返回 0 */
int mixer_setup(struct mixer_ctx *ctx, const struct mixer_cfg *cfg);

/* 一拍混控。thrust_n/alpha 非有限返回 -1 (out 清零); 否则 0。
 * thrust_n < 0 按推力 0 处理 (不出反向推力)。 */
int mixer_step(struct mixer_ctx *ctx, double thrust_n, const double alpha_b[3],
               struct mixer_out *out);

/* 缺省配置 (500g 级占位, 见头注) */
extern const struct mixer_cfg mixer_cfg_default;

#ifdef __cplusplus
}
#endif

#endif /* __MIXER_H__ */
