/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SO(3) 姿态数学实现 (纯 C99, double 精度, 与 KF-GINS/Eigen 语义一致)
 * 数学约定见 so3.h 头注释。
 */
#include "so3.h"

#include <float.h>

/* ------------------------- 欧拉角 / DCM ------------------------- */

void so3_euler_to_dcm(const double rpy[3], so3_dcm R)
{
    double cr = cos(rpy[0]), sr = sin(rpy[0]);
    double cp = cos(rpy[1]), sp = sin(rpy[1]);
    double cy = cos(rpy[2]), sy = sin(rpy[2]);

    R[0][0] = cy * cp; R[0][1] = cy * sp * sr - sy * cr; R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp; R[1][1] = sy * sp * sr + cy * cr; R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;     R[2][1] = cp * sr;                R[2][2] = cp * cr;
}

void so3_dcm_to_euler(const so3_dcm R, double rpy[3])
{
    /* sin(pitch) = -R[2][0] (ZYX 第三行第一列) */
    double sp = -R[2][0];

    if (sp > 1.0)
        sp = 1.0;
    else if (sp < -1.0)
        sp = -1.0;

    rpy[1] = asin(sp);

    if (sp > 1.0 - DBL_EPSILON || sp < -1.0 + DBL_EPSILON)
    {
        /* 俯仰 ±90° 奇点: roll 置 0 (与 KF-GINS Rotation::matrix2euler 同策略),
         * 由 R01 = ∓sin(r±ψ), R11 = cos(r±ψ) 反解 ψ。 */
        rpy[0] = 0.0;
        rpy[2] = atan2(-R[0][1], R[1][1]);
    }
    else
    {
        rpy[0] = atan2(R[2][1], R[2][2]);
        rpy[2] = atan2(R[1][0], R[0][0]);
    }
}

/* ------------------------- 四元数基本运算 ------------------------- */

void so3_quat_normalize(struct so3_quat *q)
{
    double n2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    double n = sqrt(n2);

    if (n < DBL_MIN)            /* 非法输入: 兜底为无旋转 */
    {
        q->w = 1.0;
        q->x = q->y = q->z = 0.0;
        return;
    }
    q->w /= n;
    q->x /= n;
    q->y /= n;
    q->z /= n;
}

void so3_quat_mul(const struct so3_quat *a, const struct so3_quat *b, struct so3_quat *out)
{
    struct so3_quat r;

    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;

    *out = r;
}

void so3_exp(const so3_vec v, struct so3_quat *q)
{
    double n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if (n2 < 1e-16)             /* |v| < 1e-8 rad: 泰勒展开避免 0/0 */
    {
        double k = 0.5 - n2 / 48.0;         /* sin(θ/2)/θ */

        q->w = 1.0 - 0.125 * n2;            /* cos(θ/2) */
        q->x = v[0] * k;
        q->y = v[1] * k;
        q->z = v[2] * k;
    }
    else
    {
        double n = sqrt(n2);
        double h = 0.5 * n;
        double k = sin(h) / n;

        q->w = cos(h);
        q->x = v[0] * k;
        q->y = v[1] * k;
        q->z = v[2] * k;
    }
}

void so3_log(const struct so3_quat *q_in, so3_vec v)
{
    struct so3_quat q = *q_in;
    double n2, n, scale;

    so3_quat_normalize(&q);

    /* 双覆盖: q 与 -q 同一旋转, 取 w>=0 保证 |v| ∈ [0, π] (最短旋转) */
    if (q.w < 0.0)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    n2 = q.x * q.x + q.y * q.y + q.z * q.z;

    if (n2 < 1e-24)             /* |v| < 1e-12 rad: q ≈ [1, v/2] */
    {
        v[0] = 2.0 * q.x;
        v[1] = 2.0 * q.y;
        v[2] = 2.0 * q.z;
        return;
    }

    n = sqrt(n2);
    scale = 2.0 * atan2(n, q.w) / n;

    v[0] = scale * q.x;
    v[1] = scale * q.y;
    v[2] = scale * q.z;
}

double so3_wrap_angle(double a)
{
    a = fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0)
        a += 2.0 * M_PI;
    return a - M_PI;
}

void so3_euler_diff(const double rpy_a[3], const double rpy_b[3], double d_rpy[3])
{
    d_rpy[0] = so3_wrap_angle(rpy_a[0] - rpy_b[0]);
    d_rpy[1] = so3_wrap_angle(rpy_a[1] - rpy_b[1]);
    d_rpy[2] = so3_wrap_angle(rpy_a[2] - rpy_b[2]);
}

/* ------------------------- 欧拉角 / DCM / 四元数互转 ------------------------- */

void so3_euler_to_quat(const double rpy[3], struct so3_quat *q)
{
    /* q = qz(yaw) ⊗ qy(pitch) ⊗ qx(roll), 与 C_bn = Rz*Ry*Rx 一致 */
    double cr = cos(0.5 * rpy[0]), sr = sin(0.5 * rpy[0]);
    double cp = cos(0.5 * rpy[1]), sp = sin(0.5 * rpy[1]);
    double cy = cos(0.5 * rpy[2]), sy = sin(0.5 * rpy[2]);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

void so3_quat_to_euler(const struct so3_quat *q, double rpy[3])
{
    so3_dcm R;

    so3_quat_to_dcm(q, R);
    so3_dcm_to_euler(R, rpy);
}

void so3_quat_to_dcm(const struct so3_quat *q_in, so3_dcm R)
{
    struct so3_quat q = *q_in;
    double xx, yy, zz, xy, xz, yz, wx, wy, wz;

    so3_quat_normalize(&q);

    xx = q.x * q.x; yy = q.y * q.y; zz = q.z * q.z;
    xy = q.x * q.y; xz = q.x * q.z; yz = q.y * q.z;
    wx = q.w * q.x; wy = q.w * q.y; wz = q.w * q.z;

    R[0][0] = 1.0 - 2.0 * (yy + zz);
    R[0][1] = 2.0 * (xy - wz);
    R[0][2] = 2.0 * (xz + wy);
    R[1][0] = 2.0 * (xy + wz);
    R[1][1] = 1.0 - 2.0 * (xx + zz);
    R[1][2] = 2.0 * (yz - wx);
    R[2][0] = 2.0 * (xz - wy);
    R[2][1] = 2.0 * (yz + wx);
    R[2][2] = 1.0 - 2.0 * (xx + yy);
}

void so3_dcm_to_quat(const so3_dcm R, struct so3_quat *q)
{
    /* Shepperd 法: 按 trace / 主对角最大者选分支, 数值稳健 */
    double tr = R[0][0] + R[1][1] + R[2][2];
    double s;

    if (tr > 0.0)
    {
        s = 2.0 * sqrt(tr + 1.0);
        q->w = 0.25 * s;
        q->x = (R[2][1] - R[1][2]) / s;
        q->y = (R[0][2] - R[2][0]) / s;
        q->z = (R[1][0] - R[0][1]) / s;
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        s = 2.0 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
        q->w = (R[2][1] - R[1][2]) / s;
        q->x = 0.25 * s;
        q->y = (R[0][1] + R[1][0]) / s;
        q->z = (R[0][2] + R[2][0]) / s;
    }
    else if (R[1][1] > R[2][2])
    {
        s = 2.0 * sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
        q->w = (R[0][2] - R[2][0]) / s;
        q->x = (R[0][1] + R[1][0]) / s;
        q->y = 0.25 * s;
        q->z = (R[1][2] + R[2][1]) / s;
    }
    else
    {
        s = 2.0 * sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
        q->w = (R[1][0] - R[0][1]) / s;
        q->x = (R[0][2] + R[2][0]) / s;
        q->y = (R[1][2] + R[2][1]) / s;
        q->z = 0.25 * s;
    }

    so3_quat_normalize(q);
}

/* ------------------------- 姿态误差 ------------------------- */

void so3_att_error_quat(const struct so3_quat *q_cur, const struct so3_quat *q_des,
                        struct so3_att_err *out)
{
    struct so3_quat qd_inv = { q_des->w, -q_des->x, -q_des->y, -q_des->z };
    struct so3_quat e_b_q, e_n_q;

    /* e_b = Log(R_d^T * R):  R(q_d^-1 ⊗ q_c) = R_d^T * R_c */
    so3_quat_mul(&qd_inv, q_cur, &e_b_q);
    /* e_n = Log(R * R_d^T):  R(q_c ⊗ q_d^-1) = R_c * R_d^T */
    so3_quat_mul(q_cur, &qd_inv, &e_n_q);

    so3_log(&e_b_q, out->e_b);
    so3_log(&e_n_q, out->e_n);

    out->angle = sqrt(out->e_b[0] * out->e_b[0] +
                      out->e_b[1] * out->e_b[1] +
                      out->e_b[2] * out->e_b[2]);
}

void so3_att_error_dcm(const so3_dcm R_cur, const so3_dcm R_des, struct so3_att_err *out)
{
    struct so3_quat qc, qd;

    so3_dcm_to_quat(R_cur, &qc);
    so3_dcm_to_quat(R_des, &qd);
    so3_att_error_quat(&qc, &qd, out);
}

void so3_err_nav_to_body(const so3_vec e_n, const so3_dcm R, so3_vec e_b)
{
    int i, j;

    for (i = 0; i < 3; i++)
    {
        e_b[i] = 0.0;
        for (j = 0; j < 3; j++)
            e_b[i] += R[j][i] * e_n[j];     /* R^T * e_n */
    }
}

void so3_err_body_to_nav(const so3_vec e_b, const so3_dcm R, so3_vec e_n)
{
    int i, j;

    for (i = 0; i < 3; i++)
    {
        e_n[i] = 0.0;
        for (j = 0; j < 3; j++)
            e_n[i] += R[i][j] * e_b[j];     /* R * e_b */
    }
}

void so3_euler_err_to_body(const double d_rpy[3], const double rpy_ref[3], so3_vec e_b)
{
    double cr = cos(rpy_ref[0]), sr = sin(rpy_ref[0]);      /* roll */
    double ct = cos(rpy_ref[1]), st = sin(rpy_ref[1]);      /* pitch (θ) */

    /* e_b = T(φ,θ)·Δrpy, T 为 ZYX 欧拉角速率->体角速率阵 */
    e_b[0] = d_rpy[0] - st * d_rpy[2];
    e_b[1] = cr * d_rpy[1] + sr * ct * d_rpy[2];
    e_b[2] = -sr * d_rpy[1] + cr * ct * d_rpy[2];
}

void so3_euler_err_to_nav(const double d_rpy[3], const double rpy_ref[3], so3_vec e_n)
{
    so3_dcm R;
    so3_vec e_b;

    so3_euler_err_to_body(d_rpy, rpy_ref, e_b);
    so3_euler_to_dcm(rpy_ref, R);
    so3_err_body_to_nav(e_b, R, e_n);
}
