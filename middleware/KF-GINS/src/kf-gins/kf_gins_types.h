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

#ifndef KF_GINS_TYPES_H
#define KF_GINS_TYPES_H

#include <Eigen/Dense>

#ifndef KF_GINS_EMBEDDED
#include <iomanip>
#include <iostream>
#endif

#include "common/angle.h"

typedef struct Attitude {
    Eigen::Quaterniond qbn;
    Eigen::Matrix3d cbn;
    Eigen::Vector3d euler;
} Attitude;

typedef struct PVA {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Attitude att;
} PVA;

typedef struct ImuError {
    Eigen::Vector3d gyrbias;
    Eigen::Vector3d accbias;
    Eigen::Vector3d gyrscale;
    Eigen::Vector3d accscale;
} ImuError;

typedef struct NavState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d euler;

    ImuError imuerror;
} NavState;

typedef struct ImuNoise {
    Eigen::Vector3d gyr_arw;
    Eigen::Vector3d acc_vrw;
    Eigen::Vector3d gyrbias_std;
    Eigen::Vector3d accbias_std;
    Eigen::Vector3d gyrscale_std;
    Eigen::Vector3d accscale_std;
    double corr_time;
} ImuNoise;

typedef struct GINSOptions {

    // 初始状态和状态标准差
    // initial state and state standard deviation
    NavState initstate;
    NavState initstate_std;

    // IMU噪声参数
    // imu noise parameters
    ImuNoise imunoise;

    // 安装参数
    // install parameters
    Eigen::Vector3d antlever = {0, 0, 0};

    // 磁力计航向观测参数 (BMM350)
    // magnetometer heading-aid parameters (BMM350)
    bool magenable      = false;  // 是否启用磁航向观测
    double magdecl      = 0.0;    // 磁偏角, rad, 东偏为正
    double magstd       = 0.0;    // 磁航向观测噪声标准差, rad
    double maggaterad   = 0.0;    // 磁航向新息门限, rad
    double magfusedt    = 0.1;    // 磁航向观测入滤最小间隔, s (0=不限速)

    // 气压计高度观测参数 (BMP585)
    // barometer height-aid parameters (BMP585)
    bool baroenable     = false;  // 是否启用气压高度观测
    double barostd      = 0.0;    // 气压高度观测噪声标准差, m
    double barogatem    = 0.0;    // 气压高度新息门限, m
    double barotau      = 300.0;  // GNSS 有效时气压基准再锚定时间常数, s
    double barofusedt   = 0.5;    // 气压观测入滤最小间隔, s (0=不限速)

    // GNSS 水平速度观测参数 (RMC 地速/航迹角, 本工程嵌入扩展)
    double gnssvelstd   = 0.0;    // GNSS 水平速度观测噪声标准差, m/s (0=不融合)

#ifndef KF_GINS_EMBEDDED
    void print_options() {
        std::cout << "---------------KF-GINS Options:---------------" << std::endl;

        // 打印初始状态
        // print initial state
        std::cout << " - Initial State: " << std::endl;
        std::cout << '\t' << "- initial position: ";
        std::cout << std::setprecision(12) << initstate.pos[0] * R2D << "  ";
        std::cout << std::setprecision(12) << initstate.pos[1] * R2D << "  ";
        std::cout << std::setprecision(6) << initstate.pos[2] << " [deg, deg, m] " << std::endl;
        std::cout << '\t' << "- initial velocity: " << initstate.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude: " << initstate.euler.transpose() * R2D << " [deg] " << std::endl;
        std::cout << '\t' << "- initial gyrbias : " << initstate.imuerror.gyrbias.transpose() * R2D * 3600
                  << " [deg/h] " << std::endl;
        std::cout << '\t' << "- initial accbias : " << initstate.imuerror.accbias.transpose() * 1e5 << " [mGal] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrscale: " << initstate.imuerror.gyrscale.transpose() * 1e6 << " [ppm] "
                  << std::endl;
        std::cout << '\t' << "- initial accscale: " << initstate.imuerror.accscale.transpose() * 1e6 << " [ppm] "
                  << std::endl;

        // 打印初始状态标准差
        // print initial state STD
        std::cout << " - Initial State STD: " << std::endl;
        std::cout << '\t' << "- initial position std: " << initstate_std.pos.transpose() << " [m] " << std::endl;
        std::cout << '\t' << "- initial velocity std: " << initstate_std.vel.transpose() << " [m/s] " << std::endl;
        std::cout << '\t' << "- initial attitude std: " << initstate_std.euler.transpose() * R2D << " [deg] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrbias std: " << initstate_std.imuerror.gyrbias.transpose() * R2D * 3600
                  << " [deg/h] " << std::endl;
        std::cout << '\t' << "- initial accbias std: " << initstate_std.imuerror.accbias.transpose() * 1e5 << " [mGal] "
                  << std::endl;
        std::cout << '\t' << "- initial gyrscale std: " << initstate_std.imuerror.gyrscale.transpose() * 1e6
                  << " [ppm] " << std::endl;
        std::cout << '\t' << "- initial accscale std: " << initstate_std.imuerror.accscale.transpose() * 1e6
                  << " [ppm] " << std::endl;

        // 打印IMU噪声参数
        // print IMU noise parameters
        std::cout << " - IMU noise: " << std::endl;
        std::cout << '\t' << "- arw: " << imunoise.gyr_arw.transpose() * R2D * 60 << " [deg/sqrt(h)] " << std::endl;
        std::cout << '\t' << "- vrw: " << imunoise.acc_vrw.transpose() * 60 << " [m/s/sqrt(h)] " << std::endl;
        std::cout << '\t' << "- gyrbias  std: " << imunoise.gyrbias_std.transpose() * R2D * 3600 << " [deg/h] "
                  << std::endl;
        std::cout << '\t' << "- accbias  std: " << imunoise.accbias_std.transpose() * 1e5 << " [mGal] " << std::endl;
        std::cout << '\t' << "- gyrscale std: " << imunoise.gyrscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
        std::cout << '\t' << "- accscale std: " << imunoise.accscale_std.transpose() * 1e6 << " [ppm] " << std::endl;
        std::cout << '\t' << "- correlation time: " << imunoise.corr_time / 3600.0 << " [h] " << std::endl;

        // 打印GNSS天线杆臂
        // print GNSS antenna leverarm
        std::cout << " - Antenna leverarm: " << antlever.transpose() << " [m] " << std::endl << std::endl;
    }
#endif /* KF_GINS_EMBEDDED */

} GINSOptions;

#endif // KF_GINS_TYPES_H
