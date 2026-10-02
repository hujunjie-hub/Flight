/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 传感器标定参数域 (W25Q64 分区 calib, 见 param_part.h)
 *
 * 原 calibration/calib_store.c (STM32H723 片内 Flash 扇区 7) 迁移而来,
 * 对外 API 原样保留 (mag_calib / baro_calib / gins_bridge 调用不变):
 *   - RAM 镜像 struct calib_data: 各校准模块改自己字段后 calib_store_save()
 *   - 掉电保持改存 W25Q64 追加式记录区 (掉电原子/磨损均衡见 param_part.h)
 *   - 一次性导入: 首次上电若 calib 分区无记录而片内扇区 7 留有旧格式
 *     有效记录 (烧写固件不擦该扇区), 解析后转存 W25Q64 —— 2026-10-02
 *     实测标定的磁力计/气压计/加计零偏参数不因迁移丢失; 导入与否由
 *     sys 分区 migrated 标志记住, `param erase calib` 后不会复活旧值
 */

#ifndef __PARAM_CALIB_H__
#define __PARAM_CALIB_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 校准参数 RAM 镜像: 各校准模块修改自己的字段后调 calib_store_save() */
struct calib_data
{
    /* 磁力计 (BMM350, 椭球拟合) */
    rt_bool_t   mag_valid;              /* 参数有效并参与校正 */
    float       mag_bias_ut[3];         /* 硬磁偏置 (µT, 传感器框架) */
    float       mag_softiron[3][3];     /* 软磁矩阵 S, 校正 = S (raw - bias) */
    float       mag_radius_ut;          /* 等效球半径 (µT) */
    float       mag_resid;              /* 拟合代数残差 rms (相对量) */
    float       mag_maxratio;           /* 主轴半径最大/最小比 */
    rt_uint16_t mag_samples;            /* 拟合用样本数 */

    /* 气压计 (BMP585, 基准偏移) */
    rt_bool_t   baro_valid;
    float       baro_offset_pa;         /* 压强偏移 (Pa), p_cal = p_raw + offset */
    float       baro_cal_temp;          /* 标定时芯片温度 (°C) */
    float       baro_mean_pa;           /* 标定时实测均值 (Pa) */

    /* 加计零偏 (KF-GINS EKF 收敛估值快照, C8: 上电作引擎初值,
     * 初始对准误差 0.7° -> ~0.2°) */
    rt_bool_t   acc_valid;
    float       acc_bias_mgal[3];       /* 体坐标系 FRD, mGal (1 mGal = 1e-5 m/s²) */
    float       acc_cal_temp;           /* 采集时 IMU 内部温度 (°C, 温度标签) */
};

/* 上电加载 (幂等, 多次调用只读一次 W25Q64); 无有效记录时清零镜像 */
rt_err_t calib_store_init(void);

/* 取 RAM 镜像 (永不为空, 调用方只改字段不换指针) */
struct calib_data *calib_store_ram(void);

/* 把 RAM 镜像追加写入 W25Q64 calib 分区并读回校验 */
rt_err_t calib_store_save(void);

/* 擦除 calib 分区并清零 RAM 镜像 (恢复出厂) */
rt_err_t calib_store_erase(void);

/* 简易十进制浮点解析 (项目风格: 不引入 libc strtod/atof, 见 um982_nmea.c 注) */
double calib_parse_num(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_CALIB_H__ */
