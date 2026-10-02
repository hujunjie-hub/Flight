/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct GNSS {
    double time;

    Vector3d blh;
    Vector3d std;

    /* 水平速度观测 (本工程嵌入扩展, RMC 地速/航迹角分解的 N/E, m/s):
     * NMEA 无垂向速度, 仅 2 维; velvalid=0 时引擎只做位置更新 */
    Eigen::Vector2d velne = Eigen::Vector2d::Zero();
    bool velvalid = false;

    /* 默认初始化 (2026-10-02 爆炸根因修复): 引擎成员槽从堆构造, 无默认
     * 值时 isvalid 为垃圾字节 —— 约半数引擎出生即带一个位置=(0,0,0) 的
     * "幽灵 GNSS 观测", 播种后首拍 (t=0.001s) 以 K≈1 把位置拽到 null
     * island (实测 11984km/23088km/55516km 爆炸)。观测槽只能经 add*Data
     * 显式置有效 */
    bool isvalid = false;
} GNSS;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

/* 三轴磁力计观测 (BMM350), 体坐标系, 补偿后磁感应强度, 单位 uT */
typedef struct MAG {
    double time;

    Vector3d mag;

    /* 观测方差缩放 (本工程嵌入扩展): 数据链质量标志的降权接口,
     * quality 干扰位置 >1 (方差 x std_scale^2), 默认 1.0 正常权重 */
    double std_scale = 1.0;

    bool isvalid = false;   /* 默认初始化, 见 GNSS 注释 (幽灵观测根因) */
} MAG;

/* 气压计观测 (BMP585): 压强 Pa, 芯片温度 degC (测高方程用) */
typedef struct BARO {
    double time;

    double pressure;
    double temp;

    bool isvalid = false;   /* 默认初始化, 见 GNSS 注释 (幽灵观测根因) */
} BARO;

/* 零速修正观测 (ZUPT, 本工程嵌入扩展): 静止检测确认后由桥接层注入,
 * 载体速度 = 0 的 3 维速度观测 (NED), m/s */
typedef struct ZUPT {
    double time;

    double std;         /* 逐轴观测噪声标准差, m/s */

    bool isvalid = false;   /* 默认初始化, 见 GNSS 注释 (幽灵观测根因) */
} ZUPT;

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

#endif // TYPES_H
