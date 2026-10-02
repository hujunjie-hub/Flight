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

#ifndef GI_ENGINE_H
#define GI_ENGINE_H

#include <Eigen/Dense>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

#ifdef KF_GINS_EMBEDDED
/* 嵌入式: 协方差异常不退出进程, 由固件侧实现告警钩子 (gins_bridge.cpp) */
extern "C" void kf_gins_cov_warning(double timestamp);
/* 调试黑匣子: 单拍状态反馈大修正 (播种后位置爆炸取证, 2026-10-02) */
extern "C" void kf_gins_dx_surge(double t, int src, double dpos,
                                 double dvel, double dphi);
#include "kf_math.h"                       /* checkCov 自愈后回灌 P (A4) */
#endif

class GIEngine {

public:
    explicit GIEngine(GINSOptions &options);

    ~GIEngine() = default;

    /**
     * @brief 添加新的IMU数据，(不)补偿IMU误差
     *        add new imudata, do (not) compensate imu error
     * @param [in] imu        新的IMU原始数据
     *                        new raw imudata
     * @param [in] compensate 是否补偿IMU误差
     *                        if compensate imu error to new imudata
     * */
    void addImuData(const IMU &imu, bool compensate = false) {

        imupre_ = imucur_;
        imucur_ = imu;

        if (compensate) {
            imuCompensate(imucur_);
        }
    }

    /**
     * @brief 添加新的GNSS数据
     *        add new gnssdata
     * @param [in] gnss 新的GNSS数据
     *                  new gnssdata
     * */
    void addGnssData(const GNSS &gnss) {

        gnssdata_ = gnss;
        // 暂不进行数据有效性检查，GNSS数据默认有效
        // do not check the validity of gnssdata, the gnssdata is valid by default
        gnssdata_.isvalid = true;
    }

    /**
     * @brief 添加新的磁力计数据 (BMM350, 体坐标系 uT)
     *        add new magnetometer data (BMM350, body frame uT)
     * */
    void addMagData(const MAG &mag) {

        magdata_     = mag;
        magdata_.isvalid = true;
    }

    /**
     * @brief 添加新的气压计数据 (BMP585, 压强 Pa / 温度 degC)
     *        add new barometer data (BMP585, pressure Pa / temp degC)
     * */
    void addBaroData(const BARO &baro) {

        barodata_     = baro;
        barodata_.isvalid = true;
    }

    /**
     * @brief 添加零速修正观测 (静止检测确认后由桥接层注入, 3 维零速)
     *        add new zero-velocity-update observation (ZUPT)
     * */
    void addZuptData(const ZUPT &zupt) {

        zuptdata_     = zupt;
        zuptdata_.isvalid = true;
    }

    /**
     * @brief 磁力计/气压计观测的使用统计 (调试用)
     *        mag/baro measurement usage statistics (for debugging)
     * */
    struct UpdateStat {
        unsigned int magupd  = 0;   // 磁航向观测已更新次数
        unsigned int magrej  = 0;   // 磁航向观测被拒绝次数 (门限/倾角/场强)
        unsigned int magdrop = 0;   // 磁力计观测因时间过旧被丢弃次数
        unsigned int baroupd  = 0;  // 气压高度观测已更新次数
        unsigned int barorej  = 0;  // 气压高度观测被拒绝次数 (门限)
        unsigned int barodrop = 0;  // 气压计观测因时间过旧被丢弃次数
        unsigned int baroskip = 0;  // 气压高度观测因入滤限速跳过次数 (模型仍更新)
        unsigned int vreset   = 0;  // 垂直新息持续超限触发协方差膨胀次数
        unsigned int zuptupd  = 0;  // ZUPT 零速观测已更新次数
        unsigned int zuptrej  = 0;  // ZUPT 零速观测被新息门拒绝次数
        unsigned int updok   = 0;   // 观测更新成功次数 (全部观测源, 桥接侧失败恢复链用)
        unsigned int updfail = 0;   // Cholesky 非正定被丢弃次数 (P 失健康, 无自愈)
        unsigned int gnssdrop = 0;  // 引擎侧过旧被静默丢弃的 GNSS 观测数 (D13, 与桥接侧 stale 分口径)
        unsigned int covheal = 0;   // 协方差对角负值/NaN 自愈复位次数 (A4)
    };
    const UpdateStat &updateStat() const {
        return updstat_;
    }

    /**
     * @brief 获取最近一次气压高度观测的模型高度 (h_raw + 再锚定偏差), m
     *        get model height of the last baro observation (h_raw + bias), m
     * */
    double baroHeight() const {
        return barohgt_;
    }

    /**
     * @brief 处理新的IMU数据
     *        process new imudata
     * */
    void newImuProcess();

    /**
     * @brief 内插增量形式的IMU数据到指定时刻
     *        interpolate incremental imudata to given timestamp
     * @param [in]     imu1      前一时刻IMU数据
     *                           the previous imudata
     * @param [in,out] imu2      当前时刻IMU数据
     *                           the current imudata
     * @param [in]     timestamp 给定内插到的时刻
     *                           given interpolate timestamp
     * @param [in,out] midimu    输出内插时刻的IMU数据
     *                           output imudata at given timestamp
     * */
    static void imuInterpolate(const IMU &imu1, IMU &imu2, const double timestamp, IMU &midimu) {

        if (imu1.time > timestamp || imu2.time < timestamp) {
            return;
        }

        double lamda = (timestamp - imu1.time) / (imu2.time - imu1.time);

        midimu.time   = timestamp;
        midimu.dtheta = imu2.dtheta * lamda;
        midimu.dvel   = imu2.dvel * lamda;
        midimu.dt     = timestamp - imu1.time;

        imu2.dtheta = imu2.dtheta - midimu.dtheta;
        imu2.dvel   = imu2.dvel - midimu.dvel;
        imu2.dt     = imu2.dt - midimu.dt;
    }

    /**
     * @brief 获取当前时间
     *        get current time
     * */
    double timestamp() const {
        return timestamp_;
    }

    /**
     * @brief 倾角补偿磁航向 (去航向水平旋转 Ry(pitch)*Rx(roll), 与 yaw 无关,
     *        供 magUpdate 与桥接层 NOGNSS 播种 yaw 初始化共用)
     *        tilt-compensated magnetic heading (yaw-independent)
     * @param[in] mag_ut    体坐标系磁场, uT
     * @param[in] roll      横滚角, rad
     * @param[in] pitch     俯仰角, rad
     * @param[in] decl_rad  磁偏角, rad (东偏为正)
     * @return 真航向, rad
     * */
    static double levelMagHeading(const Eigen::Vector3d &mag_ut, double roll,
                                  double pitch, double decl_rad);

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    NavState getNavState();

    /**
     * @brief 获取当前状态协方差
     *        get current state covariance
     * */
    Eigen::MatrixXd getCovariance() {
        return Cov_;
    }

private:
    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate, const NavState &initstate_std);

    /**
     * @brief 当前IMU误差补偿到IMU数据中
     *        componsate imu error to the imudata
     * @param [in,out] imu 需要补偿的IMU数据
     *                     imudata to be compensated
     * */
    void imuCompensate(IMU &imu);

    /**
     * @brief 判断是否需要更新,以及更新哪一时刻系统状态
     *        determine if we should do upate and which navstate to update
     * @param [in] imutime1   上一IMU状态时间
     *                        the last state time
     * @param [in] imutime2   当前IMU状态时间
     *                        the current state time
     * @param [in] updatetime 状态更新的时间
     *                        time to update state
     * @return 0: 不需要更新
     *            donot need update
     *         1: 需要更新上一IMU状态
     *            update the last navstate
     *         2: 需要更新当前IMU状态
     *            update the current navstate
     *         3: 需要将IMU进行内插到状态更新时间
     *            need interpolate imudata to updatetime
     * */
    int isToUpdate(double imutime1, double imutime2, double updatetime) const;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(IMU &imupre, IMU &imucur);

    /**
     * @brief 分发观测更新 (GNSS/磁力计/气压计)
     *        dispatch observation update (gnss/mag/baro)
     * @param [in] src 观测源 (OBS_GNSS / OBS_MAG / OBS_BARO)
     *                 observation source
     * */
    void obsUpdate(int src);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     *        update state using gnss position
     * @param [in,out] gnssdata
     * */
    void gnssUpdate(GNSS &gnssdata);

    /**
     * @brief 使用磁航向观测更新系统状态 (倾角补偿 + 磁偏角, 1维 yaw 观测)
     *        update state using magnetic heading (tilt-compensated + declination, 1D yaw)
     * @param [in,out] magdata
     * */
    void magUpdate(MAG &magdata);

    /**
     * @brief 使用气压高度观测更新系统状态 (测高方程 + GNSS 锚定, 1维天向位置观测)
     *        update state using barometric height (hypsometric + GNSS-anchored, 1D up)
     * @param[in,out] barodata
     * */
    void baroUpdate(BARO &barodata);

    /**
     * @brief 使用零速观测更新系统状态 (静止部署, 3维速度观测, 使零偏/
     *        水平姿态可观 —— 无 GNSS 下速度积分发散的根治手段)
     *        update state using zero-velocity observation (ZUPT, 3D velocity)
     * @param[in,out] zuptdata
     * */
    void zuptUpdate(ZUPT &zuptdata);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Qd  传播噪声矩阵
     *                     propagation noise matrix
     * */
    void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

    /**
     * @brief Kalman 更新
     *        Kalman Filter Update process
     * @param [in] dz 观测新息
     *                measurement innovation
     * @param [in] H  观测矩阵
     *                measurement matrix
     * @param [in] R  观测噪声阵
     *                measurement noise matrix
     * */
    void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     * */
    void stateFeedback();

    /**
     * @brief 检查协方差对角线元素是否都为正
     *        Check if covariance diagonal elements are all positive
     * */
#ifdef KF_GINS_EMBEDDED
    /**
     * @brief 检查协方差对角线并自愈 (A4)
     *        负对角只告警会让数值继续恶化 (更新链路对非正定 P 无恢复,
     *        vreset 只治垂直新息不治根因); 检出负值/NaN 即把 P 整体复位
     *        回初始方差 (状态估计保留, 只重置不确定度), 滤波以保守置信
     *        重新收敛。告警钩子保持, 桥接侧计数照旧。
     *        NaN 与任何比较均为假, 判据用 !(x >= 0) 同时拦负值与 NaN。
     *        check covariance diagonal and self-heal by resetting P
     *        to its initial variances (state estimates are kept)
     * */
    void checkCov() {

        for (int i = 0; i < RANK; i++) {
            if (!(Cov_(i, i) >= 0.0)) {
                kf_gins_cov_warning(timestamp_);
                Cov_.setZero();
                if (Pdiag0_.size() == RANK)
                    Cov_.diagonal() = Pdiag0_;
                else
                    for (int k = 0; k < RANK; k++) Cov_(k, k) = 1.0;
                kf_math::set_P(Cov_.data());   // 权威 P 副本在 f32 内核, 回灌
                updstat_.covheal++;
                break;
            }
        }
    }
#else
    void checkCov() {

        for (int i = 0; i < RANK; i++) {
            if (Cov_(i, i) < 0) {
                std::cout << "Covariance is negative at " << std::setprecision(10) << timestamp_ << " !" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }
#endif

private:
    GINSOptions options_;

    double timestamp_;

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    // updata time align error
    const double TIME_ALIGN_ERR = 0.001;

    // IMU和GNSS原始数据
    // raw imudata and gnssdata
    IMU imupre_;
    IMU imucur_;
    GNSS gnssdata_;

    // 磁力计和气压计原始数据 (BMM350 / BMP585)
    // raw magnetometer and barometer data (BMM350 / BMP585)
    MAG magdata_;
    BARO barodata_;

    // 零速修正观测 (ZUPT, 桥接层静止检测注入)
    // zero-velocity-update observation (injected by bridge on static detection)
    ZUPT zuptdata_;
    int zuptrej_streak_ = 0;                // 新息门连续拒绝计数 (放行通道)
    int lastsrc_ = 0;                       // 当前正在处理的观测源 (黑匣子)

    // 气压高度模型状态: 首个样本锚定参考气压/高度/温度, 之后按测高方程外推;
    // barobias_ 在 GNSS 新鲜时慢速再锚定到 GNSS 观测高度 (抵御天气漂移,
    // 且不与 INS 融合高度构成正反馈)
    // barometric height model: anchored at first sample, extrapolated by
    // hypsometric equation; barobias_ slowly re-anchored toward the last GNSS
    // observed height while GNSS is fresh (no feedback loop with INS state)
    bool baroanchored_ = false;
    double baropref_    = 100000.0;   // 锚定参考压强, Pa
    double barohref_    = 0.0;        // 锚定参考高度, m
    double barotref_    = 288.15;     // 锚定参考温度, K
    double barobias_    = 0.0;        // 高度偏差 (再锚定), m
    double barohgt_     = 0.0;        // 最近一次气压高度观测量, m (调试)
    double barolastt_   = 0.0;        // 上次气压观测时间, s
    double lastgnssupdate_ = -1.0;    // 最近一次 GNSS 更新时间, s
    double lastgnssalt_    = 0.0;     // 最近一次 GNSS 观测的杆臂补偿高度, m (再锚定目标)
    double barofuset_      = 0.0;     // 上次气压观测入滤时刻, s (入滤限速)
    double vinnov_t_       = -1.0;    // GNSS 垂直新息超限起始时刻, s (健康监测)
    double barotema_       = 288.15;  // 测高方程滑动平均温度, K (C9: 穿层飞行温变补偿)
    Eigen::VectorXd Pdiag0_;          // 初始协方差对角 (A4: checkCov 自愈复位基准)

    // 磁/气压观测使用统计
    UpdateStat updstat_;

    // 观测源标识 (newImuProcess 多观测源时序调度用)
    // observation source id for multi-source scheduling in newImuProcess
    enum ObsSrc { OBS_NONE = 0, OBS_GNSS = 1, OBS_MAG = 2, OBS_BARO = 3, OBS_ZUPT = 4 };

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    PVA pvapre_;
    ImuError imuerror_;

    // Kalman滤波相关
    // ekf variables
    Eigen::MatrixXd Cov_;
    Eigen::MatrixXd Qc_;
    Eigen::MatrixXd dx_;

#ifdef KF_GINS_EMBEDDED
    // 嵌入式: F/G 构造时一次分配, 1kHz 传播期原地重建, 消除逐历元堆分配
    // embedded: F/G allocated once at construction, rebuilt in-place each epoch
    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;
#endif

    const int RANK      = 21;
    const int NOISERANK = 18;

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
};

#endif // GI_ENGINE_H
