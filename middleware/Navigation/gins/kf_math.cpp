/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * kf_math 实现: 设计说明见 kf_math.h 头注释
 *
 * 布局约定: 全部矩阵为 Eigen 列主序 double, 与 gi_engine 侧零转换;
 * 矩阵乘对右矩阵结构性零元素跳过 (F/G/Phi/H 稀疏)。
 */

#include <string.h>
#include <math.h>
#include <stdint.h>

#include "kf_math.h"

#if defined(__arm__) || defined(__thumb__) || defined(_M_ARM)
#define KF_ON_TARGET 1
#include "stm32h7xx.h"
#define KF_DTCM __attribute__((section(".dtcm_bss"), aligned(8)))
#else
#define KF_ON_TARGET 0
#define KF_DTCM
#endif

#define KF_N    21      /* 状态维数  = gi_engine::RANK */
#define KF_M    18      /* 噪声维数  = gi_engine::NOISERANK */
#define KF_OBS  6       /* 观测维数上限 (GNSS 位置 3) */

/* 协方差传播最小间隔, s: 1kHz IMU 下 ~100Hz 冲刷一次 P 传播 */
#define KF_PRED_MIN_DT   0.0095

static KF_DTCM struct
{
    /* 权威滤波状态 */
    double P     [KF_N * KF_N];
    double Qc    [KF_M * KF_M];
    double dx    [KF_N];
    double dt_acc;                          /* 自上次 P 冲刷累计的 dt */

    /* 最近一次 predict 传入的 F/G (冲刷用) */
    double F     [KF_N * KF_N];
    double G     [KF_N * KF_M];

    /* 传播/更新临时阵 */
    double Phi   [KF_N * KF_N];
    double Phit  [KF_N * KF_N];
    double Qd    [KF_N * KF_N];
    double Qd2   [KF_N * KF_N];
    double X     [KF_N * KF_N];
    double X2    [KF_N * KF_N];
    double T     [KF_N * KF_N];
    double T2    [KF_N * KF_N];
    double Gt    [KF_M * KF_N];
    double A     [KF_N * KF_M];

    /* update 缓冲 */
    double H     [KF_OBS * KF_N];
    double R     [KF_OBS * KF_OBS];
    double S     [KF_OBS * KF_OBS];
    double L     [KF_OBS * KF_OBS];
    double Linv  [KF_OBS * KF_OBS];
    double Sinv  [KF_OBS * KF_OBS];
    double PHt   [KF_N * KF_OBS];
    double K     [KF_N * KF_OBS];
    double dzv   [KF_OBS];
    double w     [KF_OBS];
    double dxt   [KF_N];                    /* predict 的 dx 推进中转 */
} s_dtc;

/* 耗时统计: 0.05 EMA 平滑均值 + 最大值 */
static float s_pd_avg, s_pd_max, s_up_avg, s_up_max;

/* ---------------- 基础矩阵内核 (列主序) ---------------- */

/* C(n×p) = A(n×k)·B(k×p); B 元素为精确 0 时跳过 (结构性稀疏) */
static void mmul(double *C, const double *A, const double *B, int n, int k, int p)
{
    for (int j = 0; j < p; j++)
    {
        double *c = C + j * n;

        for (int i = 0; i < n; i++)
            c[i] = 0.0;
        for (int t = 0; t < k; t++)
        {
            const double b = B[t + j * k];

            if (b == 0.0)
                continue;
            const double *a = A + t * n;

            for (int i = 0; i < n; i++)
                c[i] += a[i] * b;
        }
    }
}

/* Dst(r×c) = Src(c×r)ᵀ 转置 */
static void mtrans(double *Dst, const double *Src, int r, int c)
{
    for (int j = 0; j < c; j++)
        for (int i = 0; i < r; i++)
            Dst[j + i * c] = Src[i + j * r];
}

/* A = (A + Aᵀ)/2 对称化 */
static void msym(double *A, int n)
{
    for (int j = 0; j < n; j++)
        for (int i = j + 1; i < n; i++)
        {
            const double v = 0.5 * (A[i + j * n] + A[j + i * n]);

            A[i + j * n] = v;
            A[j + i * n] = v;
        }
}

/* ---------------- 耗时统计 ---------------- */

#if KF_ON_TARGET
static inline uint32_t kf_tmark(void)
{
    return DWT->CYCCNT;
}
static inline float kf_elapsed_us(uint32_t t0)
{
    return (float)(DWT->CYCCNT - t0) / ((float)SystemCoreClock / 1000000.0f);
}
#else
#include <time.h>
static inline uint32_t kf_tmark(void)
{
    return (uint32_t)clock();              /* host: ms 计数, 仅量级参考 */
}
static inline float kf_elapsed_us(uint32_t t0)
{
    return (float)(clock() - (clock_t)t0) * 1000.0f;
}
#endif

static inline void stat_step(uint32_t t0, float *avg, float *max)
{
    float us = kf_elapsed_us(t0);

    if (*avg == 0.0f)
        *avg = us;
    else
        *avg += (us - *avg) * 0.05f;

    if (us > *max)
        *max = us;
}

namespace kf_math
{

void init(void)
{
    memset(&s_dtc, 0, sizeof(s_dtc));

#if KF_ON_TARGET
    /* startup 不清零 .dtcm_bss, 已由上面 memset 完成; 使能 DWT 计数器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR    = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

void set_P(const double *P_cm)
{
    memcpy(s_dtc.P, P_cm, sizeof(s_dtc.P));
}

void set_Qc(const double *Qc_cm)
{
    memcpy(s_dtc.Qc, Qc_cm, sizeof(s_dtc.Qc));
}

void cov_to_double(double *P_cm)
{
    memcpy(P_cm, s_dtc.P, sizeof(s_dtc.P));
}

/* 完整协方差传播 (dt = s_dtc.dt_acc, 用后清零) */
static void pred_flush(void)
{
    const double dt = s_dtc.dt_acc;

    /* Phi = I + F·dt */
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.Phi[i] = s_dtc.F[i] * dt;
    for (int i = 0; i < KF_N; i++)
        s_dtc.Phi[i + i * KF_N] += 1.0;
    mtrans(s_dtc.Phit, s_dtc.Phi, KF_N, KF_N);

    /* Qd = G·Qc·Gᵀ·dt */
    mmul(s_dtc.A,  s_dtc.G,  s_dtc.Qc, KF_N, KF_M, KF_M);
    mtrans(s_dtc.Gt, s_dtc.G, KF_N, KF_M);
    mmul(s_dtc.Qd, s_dtc.A,  s_dtc.Gt, KF_N, KF_M, KF_N);
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.Qd[i] *= dt;

    /* Qd = (Phi·Qd·Phiᵀ + Qd)/2 */
    mmul(s_dtc.X,   s_dtc.Qd, s_dtc.Phit, KF_N, KF_N, KF_N);
    mtrans(s_dtc.T, s_dtc.X,  KF_N, KF_N);          /* Qd 对称: Xᵀ = Phi·Qd */
    mmul(s_dtc.Qd2, s_dtc.T,  s_dtc.Phit, KF_N, KF_N, KF_N);
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.Qd[i] = 0.5 * (s_dtc.Qd[i] + s_dtc.Qd2[i]);

    /* P = Phi·P·Phiᵀ + Qd (P 对称: 先算 P·Phiᵀ 再转置) */
    mmul(s_dtc.X,   s_dtc.P,  s_dtc.Phit, KF_N, KF_N, KF_N);
    mtrans(s_dtc.T, s_dtc.X,  KF_N, KF_N);
    mmul(s_dtc.X2,  s_dtc.T,  s_dtc.Phit, KF_N, KF_N, KF_N);
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.P[i] = s_dtc.X2[i] + s_dtc.Qd[i];
    msym(s_dtc.P, KF_N);

    s_dtc.dt_acc = 0.0;
}

void predict(const double *F_cm, const double *G_cm, double dt, double *dx_inout)
{
    memcpy(s_dtc.F, F_cm, sizeof(s_dtc.F));
    memcpy(s_dtc.G, G_cm, sizeof(s_dtc.G));
    predict_inplace(dt, dx_inout);
}

double *F_buf(void)
{
    return s_dtc.F;
}

double *G_buf(void)
{
    return s_dtc.G;
}

int predict_inplace(double dt, double *dx_inout)
{
    memcpy(s_dtc.dx, dx_inout, sizeof(s_dtc.dx));

    /* dx += dt·F·dx (逐历元一阶推进, 与冲刷的 Phi=I+F·dt 一致) */
    memcpy(s_dtc.dxt, s_dtc.dx, sizeof(s_dtc.dxt));
    memset(s_dtc.dx, 0, sizeof(s_dtc.dx));
    for (int j = 0; j < KF_N; j++)
    {
        const double dj = s_dtc.dxt[j];

        if (dj == 0.0)
            continue;
        for (int i = 0; i < KF_N; i++)
        {
            const double f = s_dtc.F[i + j * KF_N];

            if (f != 0.0)
                s_dtc.dx[i] += dt * f * dj;
        }
    }
    for (int i = 0; i < KF_N; i++)
        s_dtc.dx[i] += s_dtc.dxt[i];

    s_dtc.dt_acc += dt;
    int flushed = 0;
    if (s_dtc.dt_acc >= KF_PRED_MIN_DT)
    {
        uint32_t t0 = kf_tmark();

        pred_flush();
        stat_step(t0, &s_pd_avg, &s_pd_max);
        flushed = 1;
    }

    memcpy(dx_inout, s_dtc.dx, sizeof(s_dtc.dx));
    return flushed;
}

/* S(m×m) = L·Lᵀ 的 Cholesky 分解, 并求 S⁻¹ = (L⁻¹)ᵀ·(L⁻¹).
 * 返回 0: 非正定 (滤波异常, 上层丢弃本次观测) */
static int chol_inv(int m)
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            double sum = s_dtc.S[i + j * m];

            for (int k = 0; k < j; k++)
                sum -= s_dtc.L[i + k * m] * s_dtc.L[j + k * m];

            if (i == j)
            {
                /* 正定判据相对化: 绝对阈值 1e-12 在未来高精度观测 (RTK 级
                 * R ~1e-4) 下会误杀合法小对角; 以原对角 S(i,i) 为尺度。
                 * S(i,i)<=0 时 sum 只会更小, 一并拒绝 (相对比较对负基准
                 * 会误放行, 须先判原对角) */
                double sii = s_dtc.S[i + i * m];

                if (!(sii > 0.0) || !(sum > 1e-12 * sii))
                    return 0;
                s_dtc.L[i + i * m] = sqrt(sum);
            }
            else
            {
                s_dtc.L[i + j * m] = sum / s_dtc.L[j + j * m];
            }
        }
        for (int j = i + 1; j < m; j++)
            s_dtc.L[i + j * m] = 0.0;
    }

    /* Linv = L⁻¹: 逐列前代解 L·X = I */
    for (int c = 0; c < m; c++)
    {
        for (int i = 0; i < m; i++)
        {
            double sum = (i == c) ? 1.0 : 0.0;

            for (int k = 0; k < i; k++)
                sum -= s_dtc.L[i + k * m] * s_dtc.Linv[k + c * m];
            s_dtc.Linv[i + c * m] = sum / s_dtc.L[i + i * m];
        }
    }

    /* Sinv = Linvᵀ·Linv */
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++)
        {
            double sum = 0.0;

            for (int k = 0; k < m; k++)
                sum += s_dtc.Linv[k + i * m] * s_dtc.Linv[k + j * m];
            s_dtc.Sinv[i + j * m] = sum;
        }

    return 1;
}

int update(int m, const double *dz, const double *H_cm, const double *R_cm,
           double *dx_inout)
{
    uint32_t t0 = kf_tmark();

    /* 观测更新作用在与观测同历元的 P 上: 先冲刷未传播部分 */
    if (s_dtc.dt_acc > 0.0)
        pred_flush();

    memcpy(s_dtc.H,  H_cm,  (size_t)m * KF_N * sizeof(double));
    memcpy(s_dtc.R,  R_cm,  (size_t)m * m * sizeof(double));
    memcpy(s_dtc.dzv, dz,   (size_t)m * sizeof(double));

    /* PHt = P·Hᵀ: PHt(i,j) = Σ_k P(i,k)·H(j,k), H 稀疏按元素跳零 */
    for (int j = 0; j < m; j++)
    {
        double *c = s_dtc.PHt + j * KF_N;

        for (int i = 0; i < KF_N; i++)
            c[i] = 0.0;
        for (int k = 0; k < KF_N; k++)
        {
            const double b = s_dtc.H[j + k * m];

            if (b == 0.0)
                continue;
            const double *p = s_dtc.P + k * KF_N;

            for (int i = 0; i < KF_N; i++)
                c[i] += p[i] * b;
        }
    }

    /* S = H·PHt + R */
    for (int j = 0; j < m; j++)
        for (int i = 0; i < m; i++)
        {
            double sum = s_dtc.R[i + j * m];

            for (int k = 0; k < KF_N; k++)
                sum += s_dtc.H[i + k * m] * s_dtc.PHt[k + j * KF_N];
            s_dtc.S[i + j * m] = sum;
        }

    if (!chol_inv(m))
    {
        stat_step(t0, &s_up_avg, &s_up_max);
        return 0;                           /* 非正定: 丢弃本次观测 */
    }

    /* K = PHt·S⁻¹ */
    mmul(s_dtc.K, s_dtc.PHt, s_dtc.Sinv, KF_N, m, m);

    /* dx += K·(dz - H·dx) */
    for (int i = 0; i < m; i++)
    {
        double sum = s_dtc.dzv[i];

        for (int k = 0; k < KF_N; k++)
            sum -= s_dtc.H[i + k * m] * s_dtc.dx[k];
        s_dtc.w[i] = sum;
    }
    for (int i = 0; i < KF_N; i++)
    {
        double sum = s_dtc.dx[i];

        for (int j = 0; j < m; j++)
            sum += s_dtc.K[i + j * KF_N] * s_dtc.w[j];
        s_dtc.dx[i] = sum;
    }

    /* Joseph 形式: P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ */
    mmul(s_dtc.T, s_dtc.K, s_dtc.H, KF_N, m, KF_N);
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.T[i] = -s_dtc.T[i];
    for (int i = 0; i < KF_N; i++)
        s_dtc.T[i + i * KF_N] += 1.0;
    mmul(s_dtc.T2, s_dtc.T, s_dtc.P, KF_N, KF_N, KF_N);
    mtrans(s_dtc.X, s_dtc.T, KF_N, KF_N);
    mmul(s_dtc.X2, s_dtc.T2, s_dtc.X, KF_N, KF_N, KF_N);
    mmul(s_dtc.A,  s_dtc.K, s_dtc.R, KF_N, m, m);
    mtrans(s_dtc.Gt, s_dtc.K, KF_N, m);
    mmul(s_dtc.Qd2, s_dtc.A, s_dtc.Gt, KF_N, m, KF_N);
    for (int i = 0; i < KF_N * KF_N; i++)
        s_dtc.P[i] = s_dtc.X2[i] + s_dtc.Qd2[i];
    msym(s_dtc.P, KF_N);

    stat_step(t0, &s_up_avg, &s_up_max);

    memcpy(dx_inout, s_dtc.dx, sizeof(s_dtc.dx));
    return 1;
}

void stats(float *predict_avg_us, float *predict_max_us,
           float *update_avg_us,  float *update_max_us)
{
    if (predict_avg_us) *predict_avg_us = s_pd_avg;
    if (predict_max_us) *predict_max_us = s_pd_max;
    if (update_avg_us)  *update_avg_us  = s_up_avg;
    if (update_max_us)  *update_max_us  = s_up_max;
}

} // namespace kf_math
