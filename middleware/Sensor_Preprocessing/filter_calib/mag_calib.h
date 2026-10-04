/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMM350 磁力计椭球校准 (硬磁偏置 + 软磁矩阵)
 *
 * 校准流程 (FinSH, 见 README.md):
 *   magcal start [秒]   启动采集: 期间手持整机缓慢旋转/翻转, 尽量覆盖
 *                       所有姿态 (画 8 字 + 三轴各朝天地转), 结束后自动
 *                       拟合并保存到 W25Q64 (param_calib), 立即生效
 *   magcal show/on/off/clear
 *
 * 数据链: 采集输入来自 middleware/Sensor_Preprocessing/process_data 的 mag_data 环形缓冲区 (100Hz,
 * 样本已换算 µT/传感器坐标系/未校准), 采集线程 wait+pop 消费并做
 * 最小位移筛选; 拟合出的参数由 mag_calib_apply() 在 gins 桥接的
 * 喂引擎路径上生效:
 *
 *   mag_data 环形缓冲区 (µT) -> [采集] 椭球拟合 -> 参数
 *   驱动 mGauss -> µT -> mag_calib_apply (传感器框架校正)
 *              -> 轴映射 -> KF-GINS 磁航向观测
 *
 * 单位约定: 偏置 µT, 软磁矩阵无量纲, 校正输出仍为 µT。
 */

#ifndef __MAG_CALIB_H__
#define __MAG_CALIB_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 拟合质量与生效状态快照 (magcal show / 调试用) */
struct mag_calib_status
{
    rt_bool_t   valid;              /* 有有效参数 */
    rt_bool_t   enabled;            /* 是否参与校正 (valid && enabled 才校正) */
    rt_bool_t   busy;               /* 采集进行中 */
    double      bias_ut[3];
    double      softiron[3][3];
    double      radius_ut;
    double      resid;
    double      maxratio;
    rt_uint32_t samples;
};

int  mag_calib_init(void);          /* INIT 自动调用: 从 W25Q64 加载 (param_calib) */
void mag_calib_apply(const double raw_ut[3], double out_ut[3]);
                                    /* 无效/关闭时原样直通 */
rt_bool_t mag_calib_active(void);   /* 当前是否在校正输出 */
void mag_calib_get_status(struct mag_calib_status *st);

/* 采集窗口结束后自动拟合-校验-保存; 返回 RT_EOK 表示已生效 */
rt_err_t mag_calib_start(rt_uint32_t seconds);
void mag_calib_cancel(void);
rt_err_t mag_calib_set_enable(rt_bool_t on);

#ifdef __cplusplus
}
#endif

#endif /* __MAG_CALIB_H__ */
