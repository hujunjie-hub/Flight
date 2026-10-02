/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 椭球代数拟合适算实现, 模型与推导见 ellipsoid_fit.h 头注释。
 *
 * 数值措施:
 *   - solve 内先按各轴半量程归一 (x' = x/s), 等价于对法方程左右乘对角
 *     缩放 D = diag(1/s²..1/s), 把 µT 量级的平方项压回 1 附近再求解,
 *     避免双重精度下条件数无谓增大;
 *   - 9x9 线性方程用列主元高斯消元;
 *   - 3x3 对称矩阵特征分解用循环 Jacobi 旋转 (50 轮内必收敛)。
 */

#include <math.h>
#include <string.h>
#include <rtthread.h>                         /* rt_bool_t (结果 isfinite 扫描) */

#include "ellipsoid_fit.h"

/* ------------------------- 基础小工具 ------------------------- */

/* 列主元高斯消元解 A x = b (A 会被破坏), 成功返回 0 */
static int ell_solve_lin(double a[ELL_FIT_N][ELL_FIT_N], double b[ELL_FIT_N],
                         double x[ELL_FIT_N])
{
    for (int col = 0; col < ELL_FIT_N; col++)
    {
        int piv = col;
        double best = fabs(a[col][col]);

        for (int row = col + 1; row < ELL_FIT_N; row++)
        {
            if (fabs(a[row][col]) > best)
            {
                best = fabs(a[row][col]);
                piv = row;
            }
        }
        if (best < 1e-12)
            return -1;                      /* 奇异 */

        if (piv != col)
        {
            double tmp;

            for (int j = col; j < ELL_FIT_N; j++)
            {
                tmp = a[col][j];
                a[col][j] = a[piv][j];
                a[piv][j] = tmp;
            }
            tmp = b[col];
            b[col] = b[piv];
            b[piv] = tmp;
        }

        for (int row = col + 1; row < ELL_FIT_N; row++)
        {
            double f = a[row][col] / a[col][col];

            if (f == 0.0)
                continue;
            for (int j = col; j < ELL_FIT_N; j++)
                a[row][j] -= f * a[col][j];
            b[row] -= f * b[col];
        }
    }

    for (int row = ELL_FIT_N - 1; row >= 0; row--)
    {
        double sum = b[row];

        for (int j = row + 1; j < ELL_FIT_N; j++)
            sum -= a[row][j] * x[j];
        x[row] = sum / a[row][row];
    }
    return 0;
}

/* 3x3 对称矩阵 Jacobi 特征分解: a 输入对称阵 (破坏),
 * eval[3] 特征值无序, evec[3][3] 列向量为对应特征向量 */
static void ell_eig3_sym(double a[3][3], double eval[3], double evec[3][3])
{
    double q[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };

    for (int sweep = 0; sweep < 50; sweep++)
    {
        double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
        double dia = fabs(a[0][0]) + fabs(a[1][1]) + fabs(a[2][2]);

        if (off <= 1e-14 * dia)
            break;

        for (int p = 0; p < 2; p++)
        {
            for (int qq = p + 1; qq < 3; qq++)
            {
                double apq = a[p][qq];
                double theta, t, c, s;

                if (fabs(apq) < 1e-300)
                    continue;

                theta = (a[qq][qq] - a[p][p]) / (2.0 * apq);
                t = ((theta >= 0.0) ? 1.0 : -1.0) /
                    (fabs(theta) + sqrt(theta * theta + 1.0));
                c = 1.0 / sqrt(t * t + 1.0);
                s = t * c;

                /* 对行列 p/q 做旋转 A <- J^T A J */
                for (int k = 0; k < 3; k++)
                {
                    double akp = a[k][p], akq = a[k][qq];

                    a[k][p]  = c * akp - s * akq;
                    a[k][qq] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; k++)
                {
                    double apk = a[p][k], aqk = a[qq][k];

                    a[p][k]  = c * apk - s * aqk;
                    a[qq][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; k++)
                {
                    double qkp = q[k][p], kq = q[k][qq];

                    q[k][p]  = c * qkp - s * kq;
                    q[k][qq] = s * qkp + c * kq;
                }
            }
        }
    }

    for (int i = 0; i < 3; i++)
    {
        eval[i] = a[i][i];
        for (int j = 0; j < 3; j++)
            evec[j][i] = q[j][i];            /* 第 i 列为特征向量 */
    }
}

/* ------------------------- 对外接口 ------------------------- */

void ell_fit_reset(struct ell_fit_acc *acc)
{
    memset(acc, 0, sizeof(*acc));
}

void ell_fit_add(struct ell_fit_acc *acc, double x, double y, double z)
{
    double phi[ELL_FIT_N];
    double v[3] = { x, y, z };

    phi[0] = x * x;
    phi[1] = y * y;
    phi[2] = z * z;
    phi[3] = 2.0 * x * y;
    phi[4] = 2.0 * x * z;
    phi[5] = 2.0 * y * z;
    phi[6] = 2.0 * x;
    phi[7] = 2.0 * y;
    phi[8] = 2.0 * z;

    for (int i = 0; i < ELL_FIT_N; i++)
    {
        for (int j = i; j < ELL_FIT_N; j++)
            acc->n[i][j] += phi[i] * phi[j];
        acc->b[i] += phi[i];
    }

    if (!acc->have)
    {
        acc->mn[0] = acc->mx[0] = x;
        acc->mn[1] = acc->mx[1] = y;
        acc->mn[2] = acc->mx[2] = z;
        acc->have = 1;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            if (v[i] < acc->mn[i])
                acc->mn[i] = v[i];
            if (v[i] > acc->mx[i])
                acc->mx[i] = v[i];
        }
    }
    acc->cnt++;
}

int ell_fit_solve(const struct ell_fit_acc *acc, struct ell_fit_result *res)
{
    double a[ELL_FIT_N][ELL_FIT_N], rhs[ELL_FIT_N], u[ELL_FIT_N];
    double d[ELL_FIT_N];                     /* 归一对角缩放 */
    double m[3][3], v[3], minv[3][3], det;
    double c[3], k, amat[3][3];
    double eval[3], evec[3][3];
    double s = 0.0;
    int i, j;

    memset(res, 0, sizeof(*res));
    res->samples = acc->cnt;

    if (acc->cnt < 100 || !acc->have)
        return ELL_FIT_ERR_SAMPLES;

    /* 各轴必须有足够覆盖, 否则椭球形状不可辨识 */
    for (i = 0; i < 3; i++)
    {
        double range = acc->mx[i] - acc->mn[i];

        if (range < 1e-3)
            return ELL_FIT_ERR_SAMPLES;
        if (0.5 * range > s)
            s = 0.5 * range;
    }
    if (s <= 0.0)
        return ELL_FIT_ERR_SAMPLES;

    /* 归一法方程: x' = x/s 等价 phi' = D phi, D = diag(d),
     * N' = D N D, b' = D b; 解出 u' 后 u = D u' */
    for (i = 0; i < 6; i++)
        d[i] = 1.0 / (s * s);
    for (i = 6; i < ELL_FIT_N; i++)
        d[i] = 1.0 / s;

    for (i = 0; i < ELL_FIT_N; i++)
    {
        for (j = 0; j < ELL_FIT_N; j++)
            a[i][j] = acc->n[(i < j) ? i : j][(i < j) ? j : i] * d[i] * d[j];
        rhs[i] = acc->b[i] * d[i];
    }

    if (ell_solve_lin(a, rhs, u) != 0)
        return ELL_FIT_ERR_SINGULAR;

    for (i = 0; i < ELL_FIT_N; i++)
        u[i] *= d[i];

    /* M, v 还原到原始量纲 */
    m[0][0] = u[0];  m[1][1] = u[1];  m[2][2] = u[2];
    m[0][1] = m[1][0] = u[3];
    m[0][2] = m[2][0] = u[4];
    m[1][2] = m[2][1] = u[5];
    v[0] = u[6];  v[1] = u[7];  v[2] = u[8];

    /* c = -M^-1 v, k = 1 - v^T c */
    {
        double a00 = m[0][0], a01 = m[0][1], a02 = m[0][2];
        double a11 = m[1][1], a12 = m[1][2], a22 = m[2][2];
        double mscale;

        det = a00 * (a11 * a22 - a12 * a12)
            - a01 * (a01 * a22 - a12 * a02)
            + a02 * (a01 * a12 - a11 * a02);
        /* 奇异判据必须无量纲: det(M) ~ (1/r²)³, 磁半径 r ~50-110µT 时
         * 绝对阈值 1e-12 会把大半径的正常拟合误判成病态 (r=110µT ->
         * det ~5.6e-13)。以对角均值³为尺度做相对判据 */
        mscale = (fabs(a00) + fabs(a11) + fabs(a22)) / 3.0;
        if (!isfinite(det) || mscale <= 0.0 ||
            fabs(det) < 1e-12 * mscale * mscale * mscale)
            return ELL_FIT_ERR_SINGULAR;

        minv[0][0] =  (a11 * a22 - a12 * a12);
        minv[0][1] = -(a01 * a22 - a12 * a02);
        minv[0][2] =  (a01 * a12 - a11 * a02);
        minv[1][0] =  minv[0][1];
        minv[1][1] =  (a00 * a22 - a02 * a02);
        minv[1][2] = -(a00 * a12 - a01 * a02);
        minv[2][0] =  minv[0][2];
        minv[2][1] =  minv[1][2];
        minv[2][2] =  (a00 * a11 - a01 * a01);
        for (i = 0; i < 3; i++)
            for (j = 0; j < 3; j++)
                minv[i][j] /= det;
    }

    for (i = 0; i < 3; i++)
        c[i] = -(minv[i][0] * v[0] + minv[i][1] * v[1] + minv[i][2] * v[2]);

    k = 1.0 - (v[0] * c[0] + v[1] * c[1] + v[2] * c[2]);
    if (!isfinite(k) || k <= 0.0)
        return ELL_FIT_ERR_SHAPE;

    /* A = M / k, 特征值即 1/r² */
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            amat[i][j] = m[i][j] / k;

    ell_eig3_sym(amat, eval, evec);

    for (i = 0; i < 3; i++)
    {
        if (!isfinite(eval[i]) || eval[i] <= 0.0)
            return ELL_FIT_ERR_SHAPE;
        res->radii[i] = 1.0 / sqrt(eval[i]);
    }

    {
        double r_gm = pow(res->radii[0] * res->radii[1] * res->radii[2], 1.0 / 3.0);

        /* S = r_gm * Q diag(sqrt(l)) Q^T */
        for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 3; j++)
            {
                double sum = 0.0;

                for (int l = 0; l < 3; l++)
                    sum += evec[i][l] * sqrt(eval[l]) * evec[j][l];
                res->softiron[i][j] = r_gm * sum;
            }
        }
        res->radius = r_gm;
    }

    for (i = 0; i < 3; i++)
        res->bias[i] = c[i];

    /* 代数残差 rms = sqrt(u^T N u - 2 u^T b + cnt) / sqrt(cnt), rhs 为 1 故是相对量 */
    {
        double q = (double)acc->cnt;

        for (i = 0; i < ELL_FIT_N; i++)
        {
            double row = 0.0;

            for (j = 0; j < ELL_FIT_N; j++)
                row += acc->n[(i < j) ? i : j][(i < j) ? j : i] * u[j];
            q += u[i] * row - 2.0 * u[i] * acc->b[i];
        }
        if (q < 0.0)
            q = 0.0;
        res->resid_rms = sqrt(q / (double)acc->cnt);
    }

    /* 结果全量 isfinite 扫描: 极端解的 NaN 可能只落在个别分量上,
     * 抽查代表值会漏 (漏网的会随标定参数写入 flash) */
    {
        rt_bool_t ok = isfinite(res->radius) && isfinite(res->resid_rms);

        for (i = 0; i < 3 && ok; i++)
            ok = isfinite(res->bias[i]);
        for (i = 0; i < 3 && ok; i++)
            for (j = 0; j < 3 && ok; j++)
                ok = isfinite(res->softiron[i][j]);
        for (i = 0; i < 3 && ok; i++)
            ok = isfinite(res->radii[i]);
        if (!ok)
            return ELL_FIT_ERR_SHAPE;
    }

    return ELL_FIT_OK;
}
