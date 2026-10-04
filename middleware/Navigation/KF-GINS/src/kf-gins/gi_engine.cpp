/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "common/earth.h"
#include "common/rotation.h"

#include "gi_engine.h"
#include "insmech.h"

#ifdef KF_GINS_EMBEDDED
#include "kf_math.h"                       /* float32 EKF 内核 (CMSIS-DSP + DTCM) */
#endif

GIEngine::GIEngine(GINSOptions &options) {

    this->options_ = options;
#ifndef KF_GINS_EMBEDDED
    options_.print_options();
#endif
    timestamp_ = 0;

    // 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
    // resize covariance matrix, system noise matrix, and system error state matrix
    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);
#ifdef KF_GINS_EMBEDDED
    // float32 滤波内核初始化 (清零 DTCM 矩阵 + 使能 DWT), 绑定 F/G 内核
    // 缓冲地址 (传播期经 Eigen::Map 直写, 见 insPropagation)
    kf_math::init();
    F_kernel_ = kf_math::F_buf();
    G_kernel_ = kf_math::G_buf();
#endif
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵
    // initialize noise matrix
    auto imunoise                   = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
    Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

#ifdef KF_GINS_EMBEDDED
    // 连续系统噪声阵装载到 float32 内核 (之后 Qc_ 不再参与逐历元运算)
    kf_math::set_Qc(Qc_.data());
#endif

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);

    // 幽灵观测防御 (双保险, 见 types.h isvalid 注释): 显式清观测槽
    gnssdata_.isvalid = false;
    magdata_.isvalid  = false;
    barodata_.isvalid = false;
    zuptdata_.isvalid = false;
}

void GIEngine::initialize(const NavState &initstate, const NavState &initstate_std) {

    // 初始化位置、速度、姿态
    // initialize position, velocity and attitude
    pvacur_.pos       = initstate.pos;
    pvacur_.vel       = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn   = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn   = Rotation::euler2quaternion(pvacur_.att.euler);
    // 初始化IMU误差
    // initialize imu error
    imuerror_ = initstate.imuerror;

    // 给上一时刻状态赋同样的初值
    // set the same value to the previous state
    pvapre_ = pvacur_;

    // 初始化协方差
    // initialize covariance
    ImuError imuerror_std            = initstate_std.imuerror;
    Cov_.block(P_ID, P_ID, 3, 3)     = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3)     = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3)   = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3)   = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
    Cov_.block(SG_ID, SG_ID, 3, 3)   = imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
    Cov_.block(SA_ID, SA_ID, 3, 3)   = imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();

    // 初始对角留档: checkCov 自愈 (A4) 的复位基准
    Pdiag0_ = Cov_.diagonal();

#ifdef KF_GINS_EMBEDDED
    // 初始协方差装载到 float32 内核, 之后 P 以 f32 为权威副本
    kf_math::set_P(Cov_.data());
#endif
}

void GIEngine::newImuProcess() {

    // 当前IMU时间作为系统当前状态时间,
    // set current IMU time as the current state time
    timestamp_ = imucur_.time;

    // 依时间先后逐个处理本IMU区间内的观测 (GNSS / 磁力计 / 气压计):
    // 每轮取最早的有效观测, 复用 isToUpdate 的四种对齐情形, 保证单一观测源时
    // 的处理顺序与原实现逐分支等价。propagated 标记状态是否已传播到当前历元,
    // res==3 (观测位于两IMU之间) 只传播前半段并推进 imupre_ 到内插时刻。
    // process pending observations (GNSS / mag / baro) in chronological order:
    // pick the earliest valid observation each round and reuse the four
    // alignment cases of isToUpdate. With a single source this is branch-by-branch
    // equivalent to the original implementation.
    bool propagated = false;

    while (true) {
        double updatetime = -1;
        int src            = OBS_NONE;

        if (gnssdata_.isvalid) {
            updatetime = gnssdata_.time;
            src        = OBS_GNSS;
        }
        if (magdata_.isvalid && (src == OBS_NONE || magdata_.time < updatetime)) {
            updatetime = magdata_.time;
            src        = OBS_MAG;
        }
        if (barodata_.isvalid && (src == OBS_NONE || barodata_.time < updatetime)) {
            updatetime = barodata_.time;
            src        = OBS_BARO;
        }
        if (zuptdata_.isvalid && (src == OBS_NONE || zuptdata_.time < updatetime)) {
            updatetime = zuptdata_.time;
            src        = OBS_ZUPT;
        }

        if (src == OBS_NONE) {
            break;
        }

        int res = isToUpdate(imupre_.time, imucur_.time, updatetime);

        if (res == 1) {
            // 观测数据靠近上一历元，先对上一历元进行观测更新
            // observation is near to the previous imudata, update the last navstate
            obsUpdate(src);
            stateFeedback();

            pvapre_ = pvacur_;
        } else if (res == 2) {
            // 观测数据靠近当前历元，先对当前IMU进行状态传播
            // observation is near current imudata, firstly propagate navigation state
            if (!propagated) {
                insPropagation(imupre_, imucur_);
                propagated = true;
            }
            obsUpdate(src);
            stateFeedback();
        } else if (res == 3) {
            // 观测数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到观测时刻
            // observation is between the two imudata, interpolate imudata to obs time
            IMU midimu;
            imuInterpolate(imupre_, imucur_, updatetime, midimu);
            // NOTE：内插之后采样间隔会变化，严格上不满足INSMech的假设，但影响较小暂时忽略
            // Interpolation changes sampling interval, slightly violating INSMech's assumption

            // 对前一半IMU进行状态传播, 并在观测时刻进行观测更新
            // propagate for the first half imudata, then update at obs time
            insPropagation(imupre_, midimu);
            obsUpdate(src);
            stateFeedback();

            // 后半段传播推迟到循环结束统一进行 (imupre_ 推进到内插时刻)
            // second-half propagation is deferred, imupre_ advances to the middle
            pvapre_ = pvacur_;
            imupre_ = midimu;
        } else {
            // res == 0: 观测晚于当前历元 (留给后续历元处理), 或早于上一历元
            // (过旧样本, 丢弃避免阻塞后续观测)
            // res == 0: obs is later than current epoch (kept for later), or older
            // than the previous epoch (stale, drop to avoid blocking)
            if (updatetime < imupre_.time - TIME_ALIGN_ERR) {
                if (src == OBS_MAG) {
                    magdata_.isvalid = false;
                    updstat_.magdrop++;
                } else if (src == OBS_BARO) {
                    barodata_.isvalid = false;
                    updstat_.barodrop++;
                } else if (src == OBS_ZUPT) {
                    // ZUPT 时标 = 注入拍 IMU 时刻, 正常调度不会过旧; 防御性丢弃
                    zuptdata_.isvalid = false;
                } else {
                    gnssdata_.isvalid = false;
                    updstat_.gnssdrop++;   // D13: 引擎侧静默丢弃可观测 (与桥接侧 stale 分口径)
                }
                continue;
            }
            break;
        }
    }

    if (!propagated) {
        // 只传播导航状态
        // only propagate navigation state
        insPropagation(imupre_, imucur_);
    }

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 更新上一时刻的状态和IMU数据
    // update system state and imudata at the previous epoch
    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

/* 分发观测更新 (更新完成后该观测置为不可用) */
/* dispatch observation update (mark consumed) */
void GIEngine::obsUpdate(int src) {

    lastsrc_ = src;

    switch (src) {
    case OBS_GNSS:
        gnssUpdate(gnssdata_);
        break;
    case OBS_MAG:
        magUpdate(magdata_);
        break;
    case OBS_BARO:
        baroUpdate(barodata_);
        break;
    case OBS_ZUPT:
        zuptUpdate(zuptdata_);
        break;
    default:
        break;
    }
}

void GIEngine::imuCompensate(IMU &imu) {

    // 补偿IMU零偏
    // compensate the imu bias
    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;

    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale   = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel   = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}

void GIEngine::insPropagation(IMU &imupre, IMU &imucur) {

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    // compensate imu error to 'imucur', 'imupre' has been compensated
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
#ifdef KF_GINS_EMBEDDED
    // 嵌入式: F/G 经 Eigen::Map 直写 kf_math 内核 DTCM 缓冲 (构造时绑定
    // 地址), Phi/Qd 的矩阵乘在内核 (kf_math::predict_inplace) 完成;
    // 结构性零块仍由 setZero 清出, 数值语义与原 Eigen 侧重建一致
    Eigen::Map<Eigen::MatrixXd> F(F_kernel_, RANK, RANK);
    Eigen::Map<Eigen::MatrixXd> G(G_kernel_, RANK, NOISERANK);
    F.setZero();
    G.setZero();
#else
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();
#endif

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state
    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    double gravity;
#ifdef KF_GINS_EMBEDDED
    // 复用 velUpdate 首遍 (pvapre 输入, 本函数 290 行 insMech 内) 的地理
    // 参数: 输入与公式完全相同, 免每毫秒 ~6 次冗余双精三角
    {
        const INSMech::PreGeo &g = INSMech::preGeo();
        rmrn    = g.rmrn;
        gravity = g.gravity;
        wie_n   = g.wie_n;
        wen_n   = g.wen_n;
    }
#else
    rmrn    = Earth::meridianPrimeVerticalRadius(pvapre_.pos[0]);
    gravity = Earth::gravity(pvapre_.pos);
    wie_n << WGS84_WIE * cos(pvapre_.pos[0]), 0, -WGS84_WIE * sin(pvapre_.pos[0]);
    wen_n << pvapre_.vel[1] / (rmrn[1] + pvapre_.pos[2]), -pvapre_.vel[0] / (rmrn[0] + pvapre_.pos[2]),
        -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rmrn[1] + pvapre_.pos[2]);
#endif

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh   = rmrn[0] + pvapre_.pos[2];
    rnh   = rmrn[1] + pvapre_.pos[2];
    accel = imucur.dvel / imucur.dt;
    omega = imucur.dtheta / imucur.dt;

    // 位置误差
    // position error
    temp.setZero();
    temp(0, 0)                = -pvapre_.vel[2] / rmh;
    temp(0, 2)                = pvapre_.vel[0] / rmh;
    temp(1, 0)                = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                = pvapre_.vel[1] / rnh;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    temp.setZero();
    temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh - pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) / rmh +
                 pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
    temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 0)                  = pvapre_.vel[2] / rmh;
    temp(0, 1)                  = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
    temp(0, 2)                  = pvapre_.vel[0] / rmh;
    temp(1, 0)                  = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1)                  = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2)                  = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
    temp(2, 0)                  = -2 * pvapre_.vel[0] / rmh;
    temp(2, 1)                  = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
    F.block(V_ID, V_ID, 3, 3)   = temp;
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
    F.block(V_ID, SA_ID, 3, 3)  = pvapre_.att.cbn * (accel.asDiagonal());

    // 姿态误差
    // attitude error
    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
    temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
    temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
    temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh - pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
    F.block(PHI_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 1)                    = 1 / rnh;
    temp(1, 0)                    = -1 / rmh;
    temp(2, 1)                    = -tan(pvapre_.pos[0]) / rnh;
    F.block(PHI_ID, V_ID, 3, 3)   = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
    F.block(PHI_ID, BG_ID, 3, 3)  = -pvapre_.att.cbn;
    F.block(PHI_ID, SG_ID, 3, 3)  = -pvapre_.att.cbn * (omega.asDiagonal());

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block(V_ID, VRW_ID, 3, 3)    = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

#ifdef KF_GINS_EMBEDDED
    // F/G 已直写内核: 原地预测。仅在本调用冲刷了 P 传播 (P 已变) 时才
    // 回写 Eigen 副本 —— 逐历元 cov_to_double 是 3.5KB/ms 的纯搬运;
    // 回写空窗 (≤一个冲刷周期 ~10ms) 内 checkCov/磁新息门读到的是略
    // 陈旧的 P, 对自愈检测与门限自适应影响可忽略
    if (kf_math::predict_inplace(imucur.dt, dx_.data()))
        kf_math::cov_to_double(Cov_.data());
#else
    // 状态转移矩阵
    // compute the state transition matrix
    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    // 计算系统传播噪声
    // compute system propagation noise
    Qd = G * Qc_ * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
#endif
}

/* 垂直通道健康监测: GNSS 垂直新息持续超限 = 垂直滤波异常 (观测过度定权
 * 导致方差塌缩等), 膨胀垂直位置/速度/加计零偏方差恢复 GNSS 修正权 */
/* vertical health monitor thresholds */
static const double VHEALTH_INNOV_M = 30.0;          // 垂直新息超限门限, m
static const double VHEALTH_TRIG_S  = 10.0;          // 持续超限触发膨胀, s

/* GNSS 水平速度观测野值门限 (m/s, 2D 新息模长): RMC 地速/航迹角野值保护;
 * 远大于启动暂态 (|v| 峰 ~4m/s), 不阻碍收敛期的强修正 */
static const double GNSS_VEL_INNOV_MAX_MPS = 30.0;

void GIEngine::gnssUpdate(GNSS &gnssdata) {

    // IMU位置转到GNSS天线相位中心位置
    // convert IMU position to GNSS antenna phase center position
    Eigen::Vector3d antenna_pos;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv      = Earth::DRi(pvacur_.pos);
    Dr          = Earth::DR(pvacur_.pos);
    antenna_pos = pvacur_.pos + Dr_inv * pvacur_.att.cbn * options_.antlever;

    // GNSS位置测量新息
    // compute GNSS position innovation
    // 固定尺寸 (栈上) 而非 MatrixXd: 观测路径每样本 3~6 次 rt_malloc/free
    // 会落入 1kHz 解算线程的时间预算并放大最坏步抖动
    Eigen::Vector3d dz;
    dz = Dr * (antenna_pos - gnssdata.blh);

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    Eigen::Matrix<double, 3, RANK> H_gnsspos = Eigen::Matrix<double, 3, RANK>::Zero();
    H_gnsspos.block(0, P_ID, 3, 3)   = Eigen::Matrix3d::Identity();
    H_gnsspos.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever);

    // 位置观测噪声阵
    // construct measurement noise matrix
    Eigen::Matrix3d R_gnsspos;
    R_gnsspos = gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();

    // 垂直健康监测: 垂直新息持续超限说明垂直通道滤波异常 (典型: 辅助
    // 观测过度定权导致方差塌缩, GNSS 失去修正权)。膨胀垂直位置/速度/
    // 加计零偏方差, 让本次及后续 GNSS 更新重新获得增益
    // vertical health monitor: persistently large vertical innovation means
    // the vertical filter is overconfident; inflate vertical variances so
    // the GNSS update below regains authority
    if (fabs(dz(2)) > VHEALTH_INNOV_M) {
        if (vinnov_t_ < 0.0) {
            vinnov_t_ = gnssdata.time;
        } else if (gnssdata.time - vinnov_t_ > VHEALTH_TRIG_S) {
            double su = fabs(dz(2));
            if (su < 10.0) {
                su = 10.0;
            }
#ifdef KF_GINS_EMBEDDED
            // P 权威副本在内核且 Cov_ 仅冲刷/更新出口回写: 膨胀前先刷新,
            // 否则 fmax 基于陈旧对角 + set_P 回写会把内核 P 回退一个冲刷周期
            kf_math::cov_to_double(Cov_.data());
#endif
            for (int i = 0; i < 3; i++) {
                Cov_(P_ID + i, P_ID + i) = fmax(Cov_(P_ID + i, P_ID + i), su * su);
                Cov_(V_ID + i, V_ID + i) = fmax(Cov_(V_ID + i, V_ID + i), 1.0);
                Cov_(BA_ID + i, BA_ID + i) = fmax(Cov_(BA_ID + i, BA_ID + i), 0.02 * 0.02);
            }
#ifdef KF_GINS_EMBEDDED
            // 权威 P 副本在 float32 内核 (DTCM), 修改 Eigen 侧后必须回灌
            kf_math::set_P(Cov_.data());
#endif
            updstat_.vreset++;
            vinnov_t_ = gnssdata.time;   // 重新计时, 持续发散则周期性再膨胀
        }
    } else {
        vinnov_t_ = -1.0;
    }

    // EKF更新协方差和误差状态
    // do EKF update to update covariance and error state
    EKFUpdate(dz, H_gnsspos, R_gnsspos);

    // 记录 GNSS 更新时刻 (气压计再锚定的新鲜度判据) 与观测等效高度
    // (气压基准再锚定的目标: 用观测值而非融合状态, 切断正反馈)
    // record gnss update time (freshness criterion for baro re-anchoring)
    // and the lever-compensated observed height (baro re-anchor target)
    lastgnssupdate_ = gnssdata.time;
    // 观测等效高度 (杆臂补偿) 仅在合理范围内采集: 启动竞态窗口内姿态
    // 可能是垃圾-但-有限值, 直接采集会毒死气压锚定基准
    {
        double hgnss = gnssdata.blh(2) - (pvacur_.att.cbn * options_.antlever)(2);
        if (fabs(hgnss) < 20000.0) {
            lastgnssalt_ = hgnss;
        }
    }

    // GNSS 水平速度观测 (RMC 地速/航迹角 -> N/E; NMEA 无垂向速度只做 2 维):
    // 静止/低速时 "速度≈0" 是强先验, 直接钉住速度暂态, 加速启动期
    // 姿态/加计零偏/yaw 的可观测性分离 (收敛 ~3min -> <1min)。
    // 新息 = 预测-测量, 与位置观测约定一致 (状态反馈 vel -= dx)
    // GNSS horizontal velocity update (RMC-derived N/E components)
    if (gnssdata.velvalid && options_.gnssvelstd > 0.0) {
        double dvn = pvacur_.vel(0) - gnssdata.velne(0);
        double dve = pvacur_.vel(1) - gnssdata.velne(1);
        if (dvn * dvn + dve * dve < GNSS_VEL_INNOV_MAX_MPS * GNSS_VEL_INNOV_MAX_MPS) {
            Eigen::Matrix<double, 2, 1> dz_gnssvel;
            dz_gnssvel(0, 0)  = dvn;
            dz_gnssvel(1, 0)  = dve;
            Eigen::Matrix<double, 2, RANK> H_gnssvel = Eigen::Matrix<double, 2, RANK>::Zero();
            H_gnssvel(0, V_ID)     = 1.0;
            H_gnssvel(1, V_ID + 1) = 1.0;
            Eigen::Matrix2d R_gnssvel;
            R_gnssvel = options_.gnssvelstd * options_.gnssvelstd *
                        Eigen::Matrix2d::Identity();

            EKFUpdate(dz_gnssvel, H_gnssvel, R_gnssvel);
        }
    }

    // GNSS更新之后设置为不可用
    // Set GNSS invalid after update
    gnssdata.isvalid = false;
}

/* 角度差缠绕到 (-pi, pi] */
/* wrap angle difference to (-pi, pi] */
static double wrapAngle(double angle) {

    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

/* 磁航向观测常数 (BMM350) */
/* magnetic heading observation constants (BMM350) */
static const double MAG_TILT_LIMIT   = 60.0 * D2R;  // 倾角保护门限, rad
static const double MAG_FIELD_MIN_UT = 10.0;        // 地磁场模值下限, uT (干扰/失联检查)
static const double MAG_FIELD_MAX_UT = 100.0;       // 地磁场模值上限, uT (硬磁干扰检查)

/*
 * 倾角补偿磁航向 (公共实现, magUpdate 与桥接层播种共用):
 * 去航向的水平旋转 Rz(-psi)*Cbn = Ry(pitch)*Rx(roll) (Z-Y-X 欧拉序) 把
 * 体系磁场转水平, 水平分量方位即机体磁航向, 加磁偏角。
 * 不能用完整 Cbn: 那 = Rz(psi)*R_level, 水平方位被 yaw 抵消, 得到的
 * 是磁场在导航系的方位 (恒等于磁偏角), 与 yaw 误差无关, 观测失效。
 * tilt-compensated heading via de-yaw'd level rotation (yaw-independent)
 */
double GIEngine::levelMagHeading(const Eigen::Vector3d &mag_ut, double roll,
                                 double pitch, double decl_rad) {

    const double cr = cos(roll), sr = sin(roll);
    const double cp = cos(pitch), sp = sin(pitch);

    // Rx(roll) * mag
    Eigen::Vector3d r(mag_ut[0],
                      cr * mag_ut[1] - sr * mag_ut[2],
                      sr * mag_ut[1] + cr * mag_ut[2]);
    // Ry(pitch) * r
    Eigen::Vector3d m(cp * r[0] + sp * r[2],
                      r[1],
                      -sp * r[0] + cp * r[2]);

    return atan2(-m[1], m[0]) + decl_rad;
}

void GIEngine::magUpdate(MAG &magdata) {

    magdata.isvalid = false;

    if (!options_.magenable) {
        return;
    }

    // 倾角保护: 航向分解只在 pitch 接近 ±90° (万向节锁) 时退化。
    // 仅保护 pitch —— 倒装安装 (roll≈±180°) 水平放置时磁倾角补偿数学上
    // 完全有效, 按 |roll| 拒绝会把倒装机体的磁观测全部误杀 (yaw 失去
    // 观测源, 仅靠陀螺零偏估计缓慢漂移)
    // tilt guard: heading decomposition degenerates only near pitch ±90°
    // (gimbal lock). Inverted-level mounts (roll ≈ ±180°) remain valid;
    // gating on |roll| wrongly rejects all their mag observations.
    if (fabs(pvacur_.att.euler[1]) > MAG_TILT_LIMIT) {
        updstat_.magrej++;
        return;
    }

    // 磁场模值检查: 排除明显磁干扰 (地磁场典型 25~65 uT)
    // NaN 安全: NaN 与任何比较均为 false, 用 !(min<=x<=max) 形式拒绝 NaN
    // field magnitude check: reject obvious magnetic disturbance (NaN-safe)
    double fieldnorm = magdata.mag.norm();
    if (!(fieldnorm >= MAG_FIELD_MIN_UT && fieldnorm <= MAG_FIELD_MAX_UT)) {
        updstat_.magrej++;
        return;
    }

    // 倾角补偿 (公共实现, 与桥接层播种 yaw 初始化同一份数学)
    // tilt compensation (shared with bridge-side seeding)
    double heading_meas = levelMagHeading(magdata.mag, pvacur_.att.euler[0],
                                          pvacur_.att.euler[1], options_.magdecl);

    // 新息 = INS航向 - 磁航向测量, 缠绕处理
    // innovation = INS yaw - measured heading, wrapped
    double dz = wrapAngle(pvacur_.att.euler[2] - heading_meas);

    // 自适应新息门限: max(配置门限, 3倍新息标准差)。初始收敛期 yaw 方差大
    // (上电未知), 门限随方差放宽让磁航向把 yaw 拉入; 收敛后退化为配置门限,
    // 拒绝磁干扰/硬铁异常
    // adaptive gate: max(configured gate, 3-sigma of innovation). Widened while
    // yaw variance is large (initial convergence), tightened to the configured
    // gate afterwards to reject magnetic disturbance / hard-iron anomaly
    double magstd         = options_.magstd * magdata.std_scale;   // 数据链质量降权 (嵌入扩展)
    double innovation_var = Cov_(PHI_ID + 2, PHI_ID + 2) + magstd * magstd;
    double gate           = options_.maggaterad;
    double gate_3sigma    = 3.0 * sqrt(innovation_var);
    if (gate_3sigma > gate) {
        gate = gate_3sigma;
    }

    if (fabs(dz) > gate) {
        updstat_.magrej++;
        return;
    }

    // 入滤限速 (对齐 baro 先例 barofusedt): BMM350 ~100Hz 生产全量入滤时,
    // 样本经 τ=20ms EMA 低通相邻相关 ~0.6, 按独立样本定权使 yaw 信息量
    // 虚增 ~10 倍 (baro 2026-09-29 垂直方差塌缩事故的同款失效模式)。
    // 航向计算/门限每样本照常, 仅 EKF 融合限速到 1/magfusedt; std 按信息量
    // 守恒配平 (gins_config.h GINS_MAG_FUSED_STD_DEG), 保持稳态 yaw 刚度
    // rate-limit the EKF fusion (baro precedent): heading computation and
    // gating run on every sample, only the filter update is throttled
    if (options_.magfusedt > 0.0 && magdata.time - magfuset_ < options_.magfusedt) {
        updstat_.magskip++;
        return;
    }
    magfuset_ = magdata.time;

    // 构造磁航向观测矩阵
    // phi角误差模型: C_true = exp(phi x) * C_ins, 对纯航向误差有
    // psi_true = psi_ins + phi_D, 故 dz = psi_ins - psi_meas = -phi_D
    // phi-angle model gives psi_true = psi_ins + phi_D, hence H = -1 on yaw error
    Eigen::Matrix<double, 1, 1> dz_mag;
    dz_mag(0, 0) = dz;
    Eigen::Matrix<double, 1, RANK> H_mag = Eigen::Matrix<double, 1, RANK>::Zero();
    H_mag(0, PHI_ID + 2) = -1.0;
    Eigen::Matrix<double, 1, 1> R_mag;
    R_mag(0, 0) = magstd * magstd;

    EKFUpdate(dz_mag, H_mag, R_mag);
    updstat_.magupd++;
}

/* 气压高度观测常数 (BMP585) */
/* barometric height observation constants (BMP585) */
static const double BARO_AIR_GAS_CONST = 287.05287;  // 干空气气体常数, J/(kg·K)
static const double BARO_GRAVITY       = 9.80665;    // 标准重力, m/s^2
static const double BARO_GNSS_FRESH_S  = 5.0;        // GNSS 新鲜判据 (距最近更新), s
static const double BARO_TEMP_MIN_C    = -100.0;     // 温度有效下限 (低于视为无效), degC
static const double BARO_TEMP_TAU_S    = 60.0;       // 测高温度滑动平均时间常数 (C9, 滤噪声/跟温变)
static const double BARO_PREF_MIN_PA   = 15000.0;    // 锚定压强合理下限, Pa (BMP585 量程+裕量)
static const double BARO_PREF_MAX_PA   = 135000.0;   // 锚定压强合理上限, Pa
static const double BARO_MODEL_MAX_H   = 10000.0;    // 高度模型/参考高度合理上限, m (BMP585 300hPa 量程顶 ≈9.2km)

void GIEngine::baroUpdate(BARO &barodata) {

    barodata.isvalid = false;

    if (!options_.baroenable) {
        return;
    }

    // 模型自愈: 高度模型输出不合理 (锚定期吃到垃圾参考高度/压强, 典型于
    // 启动竞态的垃圾-但-有限值窗口) 时拆掉锚定重新来, 避免坏锚定把气压
    // 观测永久挡在门限外 (NaN 与任何比较均为假, fabs(NaN)<上限 也拒绝)
    // self-heal: an insane model height means the anchor captured garbage;
    // drop the anchor and re-anchor on plausible samples
    if (baroanchored_ && !(fabs(barohgt_) < BARO_MODEL_MAX_H)) {
        baroanchored_ = false;
    }

    // 首个样本锚定参考点: 高度优先取最近 GNSS 观测的等效高度 (独立于
    // INS 融合状态), 尚无 GNSS 观测时退回当前 INS 位置 (初始化时来自
    // GNSS 播种); 温度用芯片实测 (无效时退化为 ISA 标准大气 288.15K)。
    // 锚定输入必须过合理性门 (压强量程/参考高度界), 不合理等下一个样本
    // anchor at the first sample: height from the last GNSS observation
    // (independent of the fused INS state), falling back to current INS
    // position; anchor inputs are sanity-gated
    if (!baroanchored_) {
        if (!(barodata.pressure > BARO_PREF_MIN_PA && barodata.pressure < BARO_PREF_MAX_PA)) {
            updstat_.barorej++;
            return;
        }
        // 参考高度取"新鲜的"最近 GNSS 观测高度, 不新鲜 (含从未更新) 时退回
        // 当前 INS 位置 (引擎刚播种, INS 高度即 GNSS 种子, 天然新鲜) ——
        // 陈旧的 lastgnssalt_ (失锁数小时前的值) 不允许毒化首锚
        double hrefc = pvacur_.pos[2];

        if (lastgnssupdate_ > 0.0 &&
            (barodata.time - lastgnssupdate_) < BARO_GNSS_FRESH_S)
            hrefc = lastgnssalt_;
        if (!(fabs(hrefc) < BARO_MODEL_MAX_H)) {
            updstat_.barorej++;
            return;
        }
        baropref_     = barodata.pressure;
        barohref_     = hrefc;
        barotref_     = (barodata.temp > BARO_TEMP_MIN_C) ? (barodata.temp + 273.15) : 288.15;
        barotema_     = barotref_;      // 温度 EMA 从锚定值起步
        barobias_     = 0.0;
        barohgt_      = hrefc;
        baroanchored_ = true;
        barolastt_    = barodata.time;
        return;
    }

    // 测高方程: 相对锚定点的高度 (压强下降 -> 高度上升), 相对模型不受 QNH/天气影响
    // NaN 安全: !(p > 0) 同时拒绝 NaN 与非正值
    // hypsometric equation: height relative to the anchor point (NaN-safe)
    if (!(barodata.pressure > 0.0)) {
        updstat_.barorej++;
        return;
    }

    // 温度滑动平均 (C9): 测高方程原用锚定时刻的常数温度, 穿层飞行 ΔT/T
    // ~3%/10K 的比例误差直接作用在高度差上; 有效温度按 BARO_TEMP_TAU_S
    // 慢速跟踪, 无效样本 (<下限) 保持旧值
    if (barodata.temp > BARO_TEMP_MIN_C && barolastt_ > 0.0) {
        double dtt = barodata.time - barolastt_;

        if (dtt > 0.0 && dtt < 60.0) {
            double at = dtt / (BARO_TEMP_TAU_S + dtt);

            barotema_ += at * ((barodata.temp + 273.15) - barotema_);
        }
    }

    double hraw =
        barohref_ + BARO_AIR_GAS_CONST * barotema_ / BARO_GRAVITY * log(baropref_ / barodata.pressure);

    // GNSS 新鲜时慢速再锚定偏差到 GNSS 观测高度 (而非 INS 融合高度,
    // 切断 "气压基准跟随 INS <-> INS 被气压观测拽动" 的正反馈), 跟踪
    // 天气漂移; GNSS 失锁期间偏差保持, 气压观测继续为垂直通道提供阻尼
    // slowly re-anchor the bias toward the last GNSS observed height (NOT
    // the fused INS height, breaking the baro<->INS positive feedback loop)
    // while GNSS is fresh; the bias holds during GNSS outage
    double dt = barodata.time - barolastt_;
    if (dt > 0.0 && dt < 60.0 && lastgnssupdate_ > 0.0 &&
        (barodata.time - lastgnssupdate_) < BARO_GNSS_FRESH_S) {
        double alpha = dt / options_.barotau;
        if (alpha > 1.0) {
            alpha = 1.0;
        }
        barobias_ += alpha * ((lastgnssalt_ - hraw) - barobias_);
    }
    barolastt_ = barodata.time;

    // 新息 = 气压高度测量 - INS高度 (含再锚定偏差)。符号必须与 GNSS
    // 位置观测在 U 轴上的实际约定一致: Earth::DR 的 U 对角为 -1, GNSS
    // 的 dz(2) 实为 (测量-预测); 若此处取 (INS-baro), 位置反馈会把高度
    // 推离气压基准而非拉向它 —— 2026-09-29 实测恒定 -0.17m/s 下沉根因
    // innovation = barometric height - INS height (measurement minus
    // predicted, matching the effective GNSS U-axis sign through DR(-1))
    barohgt_ = hraw + barobias_;
    double dz = barohgt_ - pvacur_.pos[2];

    if (fabs(dz) > options_.barogatem) {
        updstat_.barorej++;
        return;
    }

    // 入滤限速: 模型 (锚定/再锚定/测高方程) 每样本照常更新, 但 EKF 融合
    // 限制到 1/barofusedt。BMP585 ~100Hz 全量入滤会以信息速率压倒 GNSS
    // 垂直通道 (10Hz), 塌缩垂直方差使 GNSS 失去绝对高度基准权
    // rate-limit the EKF fusion: the model updates on every sample, only
    // the filter update is throttled so GNSS keeps vertical authority
    if (barodata.time - barofuset_ < options_.barofusedt) {
        updstat_.baroskip++;
        return;
    }
    barofuset_ = barodata.time;

    // 构造气压高度观测矩阵: 与 GNSS 位置观测同号, 仅天向分量
    // measurement matrix: same sign as GNSS position update, up component only
    Eigen::Matrix<double, 1, 1> dz_baro;
    dz_baro(0, 0) = dz;
    Eigen::Matrix<double, 1, RANK> H_baro = Eigen::Matrix<double, 1, RANK>::Zero();
    H_baro(0, P_ID + 2) = 1.0;
    Eigen::Matrix<double, 1, 1> R_baro;
    R_baro(0, 0) = options_.barostd * options_.barostd;

    EKFUpdate(dz_baro, H_baro, R_baro);
    updstat_.baroupd++;
}

/* ZUPT 新息门限 (m/s, 3D 模长): 静止检测 (原始 IMU 判据) 是第一道门,
 * 新息门只拦"静止误判但传感器未察觉"的极端情形 —— 门限取宽 (3 m/s):
 * 静止确认期速度漂到 0.5~3 m/s 正是滤波内部误差, ZUPT 是修正手段而
 * 不是被保护对象, 门限过紧会大量拒绝使其失去修正权 (实测 0.5 门限
 * 拒绝率 70%, 速度在 ±5 m/s 徘徊); 5 连拒放行保留为最后通道 */
static const double ZUPT_INNOV_MAX_MPS = 3.0;

void GIEngine::zuptUpdate(ZUPT &zuptdata) {

    zuptdata.isvalid = false;

    // 新息 = 预测速度 - 0 (与 GNSS 观测同号约定, stateFeedback: vel -= dx)
    // innovation = predicted velocity - zero
    Eigen::Vector3d dz = pvacur_.vel;

    if (dz.norm() > ZUPT_INNOV_MAX_MPS) {
        if (zuptrej_streak_ < 5) {
            zuptrej_streak_++;
            updstat_.zuptrej++;
            return;
        }
        zuptrej_streak_ = 0;        // 放行通道: 强制修正发散的速度
    } else {
        zuptrej_streak_ = 0;
    }

    // 3 维零速观测: H = [0 I 0] (V_ID), R = std^2 * I
    Eigen::Matrix<double, 3, 1> dz_zupt;
    dz_zupt(0, 0)     = dz[0];
    dz_zupt(1, 0)     = dz[1];
    dz_zupt(2, 0)     = dz[2];
    Eigen::Matrix<double, 3, RANK> H_zupt = Eigen::Matrix<double, 3, RANK>::Zero();
    H_zupt(0, V_ID)         = 1.0;
    H_zupt(1, V_ID + 1)     = 1.0;
    H_zupt(2, V_ID + 2)     = 1.0;
    Eigen::Matrix3d R_zupt;
    R_zupt = zuptdata.std * zuptdata.std * Eigen::Matrix3d::Identity();

    EKFUpdate(dz_zupt, H_zupt, R_zupt);
    updstat_.zuptupd++;
}

int GIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const {

    if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {
        // 更新时间靠近imutime1
        // updatetime is near to imutime1
        return 1;
    } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {
        // 更新时间靠近imutime2
        // updatetime is near to imutime2
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        // 更新时间在imutime1和imutime2之间, 但不靠近任何一个
        // updatetime is between imutime1 and imutime2, but not near to either
        return 3;
    } else {
        // 更新时间不在imutimt1和imutime2之间，且不靠近任何一个
        // updatetime is not bewteen imutime1 and imutime2, and not near to either.
        return 0;
    }
}

void GIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}

void GIEngine::EKFUpdate(const Eigen::Ref<const Eigen::MatrixXd> &dz,
                         const Eigen::Ref<const Eigen::MatrixXd> &H,
                         const Eigen::Ref<const Eigen::MatrixXd> &R) {

    assert(H.cols() == Cov_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

#ifdef KF_GINS_EMBEDDED
    // float32 更新: (H·P·Hᵀ+R) 用 Cholesky 求逆避免通用矩阵求逆,
    // Joseph 形式协方差更新保持数值稳定; 非正定丢弃本次观测
    // (失败计入 updfail: P 失去正定后所有观测源都会持续失败, 桥接侧
    //  据此触发拆引擎重对准 —— 静默丢弃会把 EKF 退化为纯惯导盲推)
    if (kf_math::update(H.rows(), dz.data(), H.data(), R.data(), dx_.data()))
        updstat_.updok++;
    else
        updstat_.updfail++;
    kf_math::cov_to_double(Cov_.data());
#else
    // 计算Kalman增益
    // Compute Kalman Gain
    auto temp         = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    // update system error state and covariance
    Eigen::MatrixXd I;
    I.resizeLike(Cov_);
    I.setIdentity();
    I = I - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    // if state feedback is performed after every update, dx_ is always zero before the update
    // the following formula can be simplified as : dx_ = K * dz;
    dx_  = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
#endif
}

void GIEngine::stateFeedback() {

    Eigen::Vector3d vectemp;

    // 位置误差反馈
    // posisiton error feedback
    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Earth::DRi(pvacur_.pos);

    /* 调试黑匣子: 单拍位置修正 >100m = 数值爆炸特征 (静态平台任何观测
     * 都不该产生此量级修正), 节流 0.5s 上报观测源与各状态组修正量
     * (钩子为固件侧实现, 仅 KF_GINS_EMBEDDED 下声明) */
#ifdef KF_GINS_EMBEDDED
    {
        double dm = delta_r.norm();
        if (dm > 100.0) {
            static double lastlog = -1.0e9;
            if (timestamp_ - lastlog > 0.5) {
                lastlog = timestamp_;
                kf_gins_dx_surge(timestamp_, lastsrc_, dm,
                                 dx_.block(V_ID, 0, 3, 1).norm(),
                                 dx_.block(PHI_ID, 0, 3, 1).norm());
            }
        }
    }
#endif
    pvacur_.pos -= Dr_inv * delta_r;

    // 速度误差反馈
    // velocity error feedback
    vectemp = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= vectemp;

    // 姿态误差反馈
    // attitude error feedback
    vectemp                = dx_.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(vectemp);
    pvacur_.att.qbn        = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn        = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler      = Rotation::matrix2euler(pvacur_.att.cbn);

    // IMU零偏误差反馈
    // IMU bias error feedback
    vectemp = dx_.block(BG_ID, 0, 3, 1);
    imuerror_.gyrbias += vectemp;
    vectemp = dx_.block(BA_ID, 0, 3, 1);
    imuerror_.accbias += vectemp;

    // IMU比例因子误差反馈
    // IMU sacle error feedback
    vectemp = dx_.block(SG_ID, 0, 3, 1);
    imuerror_.gyrscale += vectemp;
    vectemp = dx_.block(SA_ID, 0, 3, 1);
    imuerror_.accscale += vectemp;

    // 误差状态反馈到系统状态后,将误差状态清零
    // set 'dx' to zero after feedback error state to system state
    dx_.setZero();
}

NavState GIEngine::getNavState() {

    NavState state;

    state.pos      = pvacur_.pos;
    state.vel      = pvacur_.vel;
    state.euler    = pvacur_.att.euler;
    state.imuerror = imuerror_;

    return state;
}
