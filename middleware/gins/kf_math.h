/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * kf_math: KF-GINS 嵌入式 EKF 内核 (仅 KF_GINS_EMBEDDED 使用)
 *
 * v2 设计 (double 精度 + 传播节流):
 *   - double: 21 状态协方差全程双精度 (与上游 KF-GINS 一致)。v1 的 float32
 *     内核实测数值不可行 —— P 对角跨度 1e-19 ~ 1e19 超出 f32 的 7 位有效
 *     数字, 舍入使对角变负 (协方差告警 150 万次, 滤波发散)。
 *   - 节流: 机械编排仍 1kHz, 协方差传播 (P = Phi·P·Phiᵀ + Qd) 按
 *     KF_PRED_MIN_DT (~10ms => 100Hz) 节流, dt 逐历元累计。上游数据集
 *     即 100Hz IMU 量级; v1 在 1kHz 全速传播占 86% CPU (858µs/步),
 *     饿死 FinSH/输出线程, 是系统级故障根源。
 *   - update() 先冲刷未传播的 P (用最近一次 F/G + 累计 dt), 保证观测
 *     更新作用在与观测同历元的 P 上 (滞后 < 10ms, 对 10Hz GNSS 无影响)。
 *   - 列主序直连: 内部矩阵即 Eigen 列主序 double, 边界零转换; 矩阵乘
 *     对右矩阵零元素跳过 (F/G/Phi/H 结构性稀疏, 实测收益显著)。
 *   - DTCM/计时仅固件侧 (ARM_MATH_CM7); 主机 (gins_host_test) 同一份
 *     源码编译, 用于数值验证 (v1 曾因绑定 stm32h7xx.h 使主机测试失效)。
 */

#ifndef __KF_MATH_H__
#define __KF_MATH_H__

namespace kf_math
{

/* 初始化: 清零滤波矩阵, 固件侧使能 DWT 周期计数器 (耗时统计) */
void init(void);

/* 装载初始协方差 / 连续系统噪声阵 (Eigen 列主序 double)
 * P: 21×21, Qc: 18×18 (与 gi_engine 的 RANK/NOISERANK 一致) */
void set_P(const double *P_cm);
void set_Qc(const double *Qc_cm);

/* EKF 预测 (每个 IMU 历元调用, 1kHz):
 *   每次调用: dx += dt·F·dx (F 为本次传入的雅可比)
 *   累计 dt >= KF_PRED_MIN_DT 时冲刷一次完整传播:
 *     Phi = I + F·dt_acc
 *     Qd  = (Phi·(G·Qc·Gᵀ·dt_acc)·Phiᵀ + G·Qc·Gᵀ·dt_acc) / 2
 *     P   = Phi·P·Phiᵀ + Qd (对称化)
 * @param F_cm     21×21 状态转移雅可比 (Eigen 列主序 double)
 * @param G_cm     21×18 噪声驱动阵   (Eigen 列主序 double)
 * @param dt       IMU 采样间隔, s
 * @param dx_inout 21×1 误差状态 (double, 原地更新) */
void predict(const double *F_cm, const double *G_cm, double dt, double *dx_inout);

/* F/G 内核缓冲地址 (21×21 / 21×18, 列主序 double, DTCM): 引擎以
 * Eigen::Map 直写内核缓冲, 免逐历元 "Eigen 侧清零重建 → memcpy 进内核"
 * 的双份搬运。写毕调用 predict_inplace (勿再走 predict, 会自我拷贝) */
double *F_buf(void);
double *G_buf(void);

/* 原地预测: F/G 已在内核缓冲 (F_buf/G_buf 写毕后调用), 语义同 predict。
 * 返回 1 = 本次冲刷了完整 P 传播 (P 已变, 调用方此时才需要回写 Eigen
 * 副本); 0 = 仅 dx 逐历元推进, P 未动 */
int predict_inplace(double dt, double *dx_inout);

/* EKF 量测更新 (观测历元, ~10Hz), Joseph 形式:
 *   K   = P·Hᵀ·(H·P·Hᵀ+R)⁻¹        <- (·)⁻¹ 用 Cholesky
 *   dx  = dx + K·(dz - H·dx)
 *   P   = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ (对称化)
 * 调用时先冲刷未传播的 P (若累计 dt > 0)。
 * @param m        观测维数 (1..6)
 * @param dz       m×1 新息 (double)
 * @param H_cm     m×21 观测阵 (Eigen 列主序 double)
 * @param R_cm     m×m 观测噪声阵 (Eigen 列主序 double)
 * @param dx_inout 21×1 误差状态 (double, 原地更新)
 * (H·P·Hᵀ+R) 非正定时丢弃本次观测 (P/dx 保持不变), 返回 0 */
int update(int m, const double *dz, const double *H_cm, const double *R_cm,
           double *dx_inout);

/* 内部 P 回写为 21×21 Eigen 列主序 double (供 checkCov / getCovariance) */
void cov_to_double(double *P_cm);

/* 耗时统计 (固件 DWT 周期计数, 主机 clock), us
 * predict 只统计冲刷的重传播 (每 ~10 历元), update 每观测历元 */
void stats(float *predict_avg_us, float *predict_max_us,
           float *update_avg_us,  float *update_max_us);

} // namespace kf_math

#endif /* __KF_MATH_H__ */
