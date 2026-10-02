/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 校准参数持久化 (STM32H723 片内 Flash 扇区 7)
 *
 * 磁力计/气压计校准参数统一存在一条记录里, 掉电保持:
 *   地址   0x080E0000 (1MB 器件的最后一个 128KB 扇区, 链接脚本已把
 *          ROM 截到 896KB, 见 board/linker_scripts/ 下的 link.lds/.icf/.sct)
 *   格式   128 字节/条 x 1024 槽追加式日志:
 *          magic u32 | ver u16 | flags u16 | seq u32 | crc32 u32 头
 *          + 打包参数体 (小端); CRC 覆盖 ver/flags/seq + 参数体
 *   写入   新记录追加到下一空闲槽 (不擦旧记录, 掉电至多损失最新一条),
 *          读回校验; 扇区写满 (每 1024 次 save) 才整擦一次 —— 掉电原子
 *          性与磨损均衡由此同时获得。上电扫描取 "CRC 通过且 seq 最大"
 *          的记录; 升级前的旧单记录格式 (crc 只盖参数体) 兼容读取。
 *
 * HAL flash 驱动未编入本固件 (CMake/SCons 均未含 stm32h7xx_hal_flash*.c),
 * 这里直接按 RM0468 操作 FLASH 寄存器 (CMSIS 位定义), 擦写期间代码
 * 若从 Flash 取指会停等到操作完成 —— 只在用户主动保存时发生, 属维护
 * 动作, 期间 SysTick 可能丢失若干 tick。save/erase 由互斥锁串行化
 * (pack+写原子, 防 mag/baro 线程并发覆盖)。
 */

#ifndef __CALIB_STORE_H__
#define __CALIB_STORE_H__

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

/* 上电加载 (幂等, 多次调用只读一次 Flash); 无有效记录时清零镜像 */
rt_err_t calib_store_init(void);

/* 取 RAM 镜像 (永不为空, 调用方只改字段不换指针) */
struct calib_data *calib_store_ram(void);

/* 把 RAM 镜像擦写入 Flash 并读回校验 */
rt_err_t calib_store_save(void);

/* 擦除 Flash 记录并清零 RAM 镜像 (恢复出厂) */
rt_err_t calib_store_erase(void);

/* 简易十进制浮点解析 (项目风格: 不引入 libc strtod/atof, 见 um982_nmea.c 注) */
double calib_parse_num(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __CALIB_STORE_H__ */
