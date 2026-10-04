/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 椭球代数拟合 (纯 C99 数学, 不依赖 RT-Thread, 可主机端测试)
 *
 * 用途: 磁力计硬磁/软磁误差标定的数学核心。
 *   采集一批磁场样本 (器件绕各姿态旋转, 理想情况下样本落在以地磁场模长
 *   为半径的球面上), 硬磁偏置把球心挪离原点, 软磁畸变把球拉成椭球。
 *   最小二乘拟合该椭球得球心 c (硬磁偏置) 与形状矩阵, 再取矩阵平方根
 *   把椭球"搓圆"即得软磁校正矩阵 S:
 *
 *     校正输出 = S * (raw - c)
 *
 * 数学模型 (x 为单样本, 未知量 M 对称正定, v 为三维向量):
 *     x^T M x + 2 v^T x = 1
 * 展开后是 9 参数线性最小二乘:
 *     m11 x² + m22 y² + m33 z² + 2m12 xy + 2m13 xz + 2m23 yz
 *       + 2v1 x + 2v2 y + 2v3 z = 1
 * 对每样本构造设计行
 *     phi = [x², y², z², 2xy, 2xz, 2yz, 2x, 2y, 2z]
 * 只需增量累加法方程 (Σphi·phi^T, Σphi), 不必保存样本, 内存固定 9x9。
 * 求解后:
 *     球心   c = -M^-1 v
 *     半径² k = 1 - v^T c        (椭球 (x-c)^T (M/k) (x-c) = 1)
 *     A = M/k,  特征值 l_i = 1/r_i² (r_i 为主轴半径)
 *     S = r_gm * A^(1/2),  r_gm = (r1 r2 r3)^(1/3) 几何平均半径
 * 尺度取 r_gm 是因为真值 |B| 未知, 保持输出模长与椭球几何平均半径一致
 * (软/硬磁校正只恢复球形与方向, 模长精度对航向解算不敏感)。
 */

#ifndef __ELLIPSOID_FIT_H__
#define __ELLIPSOID_FIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ELL_FIT_N       9

/* 法方程累加器: ell_fit_add() 逐样本喂入, ell_fit_solve() 一次性求解 */
struct ell_fit_acc
{
    double          n[ELL_FIT_N][ELL_FIT_N];    /* Σ phi·phi^T (只填上三角) */
    double          b[ELL_FIT_N];               /* Σ phi */
    double          mn[3];                      /* 各轴样本最小值 */
    double          mx[3];                      /* 各轴样本最大值 */
    unsigned long   cnt;                        /* 样本数 */
    int             have;                       /* mn/mx 已有效 */
};

/* 拟合结果 (与输入同单位) */
struct ell_fit_result
{
    double bias[3];         /* 椭球心 = 硬磁偏置 */
    double softiron[3][3];  /* 软磁校正矩阵 S, 校正 = S (raw - bias) */
    double radius;          /* 等效球半径 (主轴半径几何平均) */
    double radii[3];        /* 椭球三个主轴半径 */
    double resid_rms;       /* 代数残差 rms (方程右边为 1, 即相对量) */
    unsigned long samples;
};

/* 返回码 */
enum
{
    ELL_FIT_OK              = 0,
    ELL_FIT_ERR_SAMPLES     = -1,   /* 样本数不足或某轴无覆盖 */
    ELL_FIT_ERR_SINGULAR    = -2,   /* 法方程病态/奇异 */
    ELL_FIT_ERR_SHAPE       = -3,   /* 拟合结果不是正定椭球 (特征值非正/数据退化) */
};

void ell_fit_reset(struct ell_fit_acc *acc);
void ell_fit_add(struct ell_fit_acc *acc, double x, double y, double z);
int  ell_fit_solve(const struct ell_fit_acc *acc, struct ell_fit_result *res);

#ifdef __cplusplus
}
#endif

#endif /* __ELLIPSOID_FIT_H__ */
