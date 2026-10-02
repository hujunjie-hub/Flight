/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GINSOptions 构造 (平台无关, 固件桥与主机仿真测试共用)
 *
 * 职责: 把 gins_config.h 的参数 + 首次定位/静止对准的实测值,
 * 按 KF-GINS loadConfig 相同的单位约定装配成 GINSOptions。
 * 单位换算只有这一份实现, 避免 KF-GINS yaml 路径与本路径出现分叉。
 *
 * 依赖: Eigen + KF-GINS 核心头 (kf_gins_types.h), 无 RT-Thread 依赖。
 */

#ifndef __GINS_OPTIONS_HPP__
#define __GINS_OPTIONS_HPP__

#include <cmath>
#include <Eigen/Dense>

#include "kf-gins/kf_gins_types.h"

#include "gins_config.h"

/* 体坐标系轴映射: 按配置把 ADIS 三轴重排/变号为 前右下 */
inline void gins_apply_axis(const double src[3], double dst[3])
{
    const int axis[3]   = GINS_AXIS_SRC;
    const double sign[3] = GINS_AXIS_SIGN;

    for (int i = 0; i < 3; i++)
        dst[i] = sign[i] * src[axis[i]];
}

/* 磁力计轴映射不在此处: BMM350 的校准->轴映射->低通在 data 层采集线程
 * 入环前完成 (middleware/data/mag_data.c, 用同一组 GINS_MAG_AXIS_* 宏) */

/* 静止加计对准: 比力(体系 FRD, m/s^2) -> roll/pitch (rad), yaw 不可观置 0.
 * 注意: 加速度计测的是比力, 静止时 z 轴(朝下)读数约为 -g
 * (敏感轴朝上才读 +g), 因此 roll/pitch 用负号提取 */
inline void gins_level_from_accel(const double f_b[3], double euler[3])
{
    const double g = 9.80665;
    double pitch_sin = f_b[0] / g;

    if (pitch_sin > 1.0)
        pitch_sin = 1.0;
    else if (pitch_sin < -1.0)
        pitch_sin = -1.0;

    euler[0] = atan2(-f_b[1], -f_b[2]);     /* roll: fy=-g cθ sφ, fz=-g cθ cφ */
    euler[1] = asin(pitch_sin);             /* pitch: fx=+g sθ */
    euler[2] = 0.0;                         /* yaw: GNSS 位置观测下缓慢收敛 */
}

/*
 * 构造 GINSOptions (内部单位: rad / rad/s / m / m/s)
 * @param lat_deg, lon_deg, alt_m   首次定位位置 (椭球高)
 * @param vn, ve, vd                首次定位速度 (NED)
 * @param f_b_mean                  对准窗口加计均值 (体系 m/s^2, 已轴映射)
 * @param acc_bias_mgal             C8: 持久化的加计零偏初值 (mGal, FRD),
 *                                  空指针 = 无记录用缺省 0
 * @param acc_bias_prior_mgal       C8: 有记录时的零偏先验 std (mGal),
 *                                  <=0 用缺省 GINS_IMU_AB_STD
 */
inline GINSOptions gins_build_options(double lat_deg, double lon_deg, double alt_m,
                                      double vn, double ve, double vd,
                                      const double f_b_mean[3],
                                      const double *acc_bias_mgal = 0,
                                      double acc_bias_prior_mgal = 0.0)
{
    GINSOptions opt;

    /* 初始状态 */
    opt.initstate.pos << lat_deg * D2R, lon_deg * D2R, alt_m;
    opt.initstate.vel << vn, ve, vd;

    double euler[3];
    gins_level_from_accel(f_b_mean, euler);
    opt.initstate.euler << euler[0], euler[1], euler[2];

    double gb[3] = GINS_INIT_GYR_BIAS;
    double ab[3] = GINS_INIT_ACC_BIAS;
    double ab_prior = GINS_IMU_AB_STD;

    if (acc_bias_mgal != 0)
    {
        ab[0] = acc_bias_mgal[0];              /* mGal, 收敛估值快照 */
        ab[1] = acc_bias_mgal[1];
        ab[2] = acc_bias_mgal[2];
        if (acc_bias_prior_mgal > 0.0)
            ab_prior = acc_bias_prior_mgal;
    }
    opt.initstate.imuerror.gyrbias << gb[0] * D2R / 3600.0, gb[1] * D2R / 3600.0, gb[2] * D2R / 3600.0;
    opt.initstate.imuerror.accbias << ab[0] * 1e-5, ab[1] * 1e-5, ab[2] * 1e-5;
    opt.initstate.imuerror.gyrscale.setZero();
    opt.initstate.imuerror.accscale.setZero();

    /* 初始状态标准差 */
    double pstd[3] = GINS_INIT_POS_STD;
    double vstd[3] = GINS_INIT_VEL_STD;
    double astd[3] = GINS_INIT_ATT_STD;
    opt.initstate_std.pos << pstd[0], pstd[1], pstd[2];
    opt.initstate_std.vel << vstd[0], vstd[1], vstd[2];
    opt.initstate_std.euler << astd[0] * D2R, astd[1] * D2R, astd[2] * D2R;
    /* 初始 IMU 误差标准差取噪声参数同值 (上游 loadConfig 的缺省回退路径);
     * C8: 有持久化零偏时先验收紧 (估值残余不确定度, 远小于冷启动先验) */
    opt.initstate_std.imuerror.gyrbias  << GINS_IMU_GB_STD, GINS_IMU_GB_STD, GINS_IMU_GB_STD;
    opt.initstate_std.imuerror.gyrbias *= D2R / 3600.0;
    opt.initstate_std.imuerror.accbias  << ab_prior, ab_prior, ab_prior;
    opt.initstate_std.imuerror.accbias *= 1e-5;
    opt.initstate_std.imuerror.gyrscale << GINS_IMU_GS_STD, GINS_IMU_GS_STD, GINS_IMU_GS_STD;
    opt.initstate_std.imuerror.gyrscale *= 1e-6;
    opt.initstate_std.imuerror.accscale << GINS_IMU_AS_STD, GINS_IMU_AS_STD, GINS_IMU_AS_STD;
    opt.initstate_std.imuerror.accscale *= 1e-6;

    /* IMU 噪声参数 (单位换算与上游 loadConfig 一致) */
    opt.imunoise.gyr_arw << GINS_IMU_ARW, GINS_IMU_ARW, GINS_IMU_ARW;
    opt.imunoise.gyr_arw *= D2R / 60.0;
    opt.imunoise.acc_vrw << GINS_IMU_VRW, GINS_IMU_VRW, GINS_IMU_VRW;
    opt.imunoise.acc_vrw /= 60.0;
    opt.imunoise.gyrbias_std << GINS_IMU_GB_STD, GINS_IMU_GB_STD, GINS_IMU_GB_STD;
    opt.imunoise.gyrbias_std *= D2R / 3600.0;
    opt.imunoise.accbias_std << GINS_IMU_AB_STD, GINS_IMU_AB_STD, GINS_IMU_AB_STD;
    opt.imunoise.accbias_std *= 1e-5;
    opt.imunoise.gyrscale_std << GINS_IMU_GS_STD, GINS_IMU_GS_STD, GINS_IMU_GS_STD;
    opt.imunoise.gyrscale_std *= 1e-6;
    opt.imunoise.accscale_std << GINS_IMU_AS_STD, GINS_IMU_AS_STD, GINS_IMU_AS_STD;
    opt.imunoise.accscale_std *= 1e-6;
    opt.imunoise.corr_time = GINS_IMU_CORR_TIME * 3600.0;

    /* GNSS 天线杆臂 (天线相位中心在体系中的位置) */
    double lever[3] = GINS_ANT_LEVER;
    opt.antlever << lever[0], lever[1], lever[2];

    /* 磁力计航向观测 (BMM350), 角度量换算到 rad */
    opt.magenable    = GINS_MAG_ENABLE ? true : false;
    opt.magdecl      = GINS_MAG_DECL_DEG * D2R;
    opt.magstd       = GINS_MAG_STD_DEG * D2R;
    opt.maggaterad   = GINS_MAG_GATE_DEG * D2R;

    /* 气压高度观测 (BMP585) */
    /* GNSS 水平速度观测 (RMC, 方案A) */
    opt.gnssvelstd   = GINS_GNSS_VEL_STD_M;

    opt.baroenable   = GINS_BARO_ENABLE ? true : false;
    opt.barostd      = GINS_BARO_STD_M;
    opt.barogatem    = GINS_BARO_GATE_M;
    opt.barotau      = GINS_BARO_ANCHOR_TAU_S;
    opt.barofusedt   = GINS_BARO_FUSE_DT_S;

    return opt;
}

#endif /* __GINS_OPTIONS_HPP__ */
