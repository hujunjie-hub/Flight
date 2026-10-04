/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SO(3) 姿态数学与姿态误差 (纯 C, 无 RT-Thread 依赖, 可主机测试/复用)
 *
 * ---------------------------------------------------------------------------
 * 坐标系与姿态约定 (与 middleware/Navigation/gins / KF-GINS 完全一致, 改前必读)
 * ---------------------------------------------------------------------------
 * - 导航系 n: NED (北东地); 体坐标系 b: FRD (前右下)。
 * - 欧拉角 rpy = [roll, pitch, yaw] (rad), ZYX 顺序 (3-2-1):
 *       C_bn = Rz(yaw) * Ry(pitch) * Rx(roll)
 *   即 KF-GINS Rotation::euler2matrix 的 C_b^n (体->导航)。
 * - 四元数: Hamilton 约定, (w, x, y, z); R(q1⊗q2) = R(q1)*R(q2),
 *   与 Eigen::Quaterniond 一致 (KF-GINS 上游即 Eigen)。
 *
 * ---------------------------------------------------------------------------
 * 姿态误差定义与坐标变换 (本模块核心)
 * ---------------------------------------------------------------------------
 * 给定当前姿态 R = C_bn 与期望姿态 R_d = C_bd_n (均为体->导航):
 *
 *   导航系误差 (失准角):    e_n = Log(R * R_d^T)          [NED 轴系表达]
 *   体轴系误差 (供控制):    e_b = Log(R_d^T * R)          [期望体轴 FRD 表达]
 *   精确坐标变换:           e_b = R_d^T * e_n  =  C_bd_n^T * e_n
 *
 * 注意两点 (常见错误):
 * 1. 欧拉角逐分量相减 != 姿态误差。欧拉角是三次旋转的参数, 其差不属于任何
 *    坐标系; 必须先回到 SO(3) 组合 (上式) 或用 so3_euler_err_to_body() 的
 *    T 阵变换。例: 航向 100°、水平姿态下 1° 的纯偏航差, 体轴误差是
 *    [0°, 0.98°, -0.17°] -- 误差几乎全落在俯仰轴上, 直接拿 yaw 差喂滚转/
 *    偏航通道会彻底耦合错。
 * 2. e_n 与 e_b 差一个姿态阵旋转变换 (精确关系式), 不是同一个向量在两套
 *    记号下的写法; so3_err_nav_to_body() / so3_err_body_to_nav() 给出精确
 *    互算。控制律 (几何控制 / 串级 PID) 消费 e_b; 日志/回传/与 GINS 误差
 *    状态 (失准角) 对表用 e_n。
 *
 * e_b 取期望体系表达 (Lee 几何控制 e_R 与 PX4 q_error 同款); 小误差时与
 * 当前体系二阶一致。|e| ∈ [0, π], Log 取最短旋转, 对 yaw 缠绕 (±180° 跳变、
 * KF-GINS 连续 yaw / [0,2π) yaw) 均不敏感。
 */
#ifndef __SO3_H__
#define __SO3_H__

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* newlib 在 -D_POSIX_C_SOURCE=1 下会隐藏 M_PI (见 middleware/Navigation/gins/README.md) */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SO3_DEG2RAD(x) ((x) * (M_PI / 180.0))
#define SO3_RAD2DEG(x) ((x) * (180.0 / M_PI))

typedef double so3_vec[3];      /* 3 维向量 / 旋转矢量 (rad, 轴角: 方向=转轴, 模=转角) */
typedef double so3_dcm[3][3];   /* 方向余弦阵, R[row][col], C_bn 为体->导航 */

struct so3_quat
{
    double w, x, y, z;
};

/* 姿态误差结果 */
struct so3_att_err
{
    so3_vec e_b;    /* 体轴系 (期望体系 FRD) 误差旋转矢量, rad, 供控制律 */
    so3_vec e_n;    /* 导航系 (NED) 误差旋转矢量 (失准角), rad */
    double  angle;  /* 姿态误差总角 |e_b| = |e_n|, rad, 范围 [0, π] */
};

/* ------------------------- 欧拉角 / DCM / 四元数互转 ------------------------- */

/* rpy[3] (rad, ZYX) -> C_bn (体->导航) */
void so3_euler_to_dcm(const double rpy[3], so3_dcm R);

/* C_bn -> rpy[3] (rad); roll/yaw ∈ (-π,π], pitch ∈ (-π/2,π/2), 含俯仰奇点保护 */
void so3_dcm_to_euler(const so3_dcm R, double rpy[3]);

void so3_euler_to_quat(const double rpy[3], struct so3_quat *q);
void so3_quat_to_euler(const struct so3_quat *q, double rpy[3]);
void so3_quat_to_dcm(const struct so3_quat *q, so3_dcm R);
void so3_dcm_to_quat(const so3_dcm R, struct so3_quat *q);   /* Shepperd 法, 数值稳健 */

/* ------------------------- 四元数基本运算 ------------------------- */

void so3_quat_normalize(struct so3_quat *q);
/* out = a ⊗ b (Hamilton); out 与 a/b 可重叠时请传不同变量 */
void so3_quat_mul(const struct so3_quat *a, const struct so3_quat *b, struct so3_quat *out);

/* 旋转矢量 -> 四元数 (指数映射 exp([v]×)); 对任意 |v| 数值稳健 */
void so3_exp(const so3_vec v, struct so3_quat *q);

/* 四元数 -> 旋转矢量 (对数映射 Log), 最短旋转, |v| ∈ [0, π]; 零姿态返回 0 */
void so3_log(const struct so3_quat *q, so3_vec v);

/* 角度归一化到 [-π, π) */
double so3_wrap_angle(double a);

/* 欧拉角逐分量差并归一化 (仅用于显示/记录; 做误差必须走下面的变换函数) */
void so3_euler_diff(const double rpy_a[3], const double rpy_b[3], double d_rpy[3]);

/* ------------------------- 姿态误差 ------------------------- */

/*
 * 姿态误差 (核心函数):
 *   e_n = Log(R_cur * R_des^T)   导航系 (NED) 失准角
 *   e_b = Log(R_des^T * R_cur)   期望体轴系 (FRD) 误差, = R_des^T * e_n (精确)
 * q_cur / q_des 需为单位四元数 (内部先归一化容错)。
 */
void so3_att_error_quat(const struct so3_quat *q_cur, const struct so3_quat *q_des,
                        struct so3_att_err *out);
void so3_att_error_dcm(const so3_dcm R_cur, const so3_dcm R_des,
                       struct so3_att_err *out);

/* 误差旋转矢量精确互算 (R = C_bn, 误差定义对应的参考姿态, 小误差时取当前/期望均可):
 *   nav -> body: e_b = R^T * e_n;  body -> nav: e_n = R * e_b
 */
void so3_err_nav_to_body(const so3_vec e_n, const so3_dcm R, so3_vec e_b);
void so3_err_body_to_nav(const so3_vec e_b, const so3_dcm R, so3_vec e_n);

/*
 * 欧拉角差 -> 误差旋转矢量 (小角近似: e_b = T(φ,θ)·Δrpy + O(Δrpy²)):
 *   T = [ 1     0      -sinθ    ]
 *       [ 0    cosφ   cosθ·sinφ ]     (ZYX 欧拉角速率 -> 体角速率阵)
 *       [ 0   -sinφ   cosθ·cosφ ]
 * 各单轴分量与 SO(3) 精确解完全一致 (滚转/俯仰/偏航单独误差时无近似),
 * 组合误差为二阶小量; 输入 d_rpy 建议先经 so3_euler_diff 归一化。
 * rpy_ref 为参考姿态 (期望姿态), nav 版本 = R_ref * e_b。
 */
void so3_euler_err_to_body(const double d_rpy[3], const double rpy_ref[3], so3_vec e_b);
void so3_euler_err_to_nav(const double d_rpy[3], const double rpy_ref[3], so3_vec e_n);

#ifdef __cplusplus
}
#endif

#endif /* __SO3_H__ */
