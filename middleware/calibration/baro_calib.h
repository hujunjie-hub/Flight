/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP585 气压计基准偏移校准
 *
 * MEMS 气压计绝对精度有限且随温度漂移, 融合算法只用相对测高, 但一个
 * 固定的出厂/安装偏移会让气压高度观测量整体偏移。本模块用"单点基准"
 * 校准该偏移: 已知当地准确气压 (气象站 QNH 换算到本地, 或另一只参考
 * 气压计读数) 或已知当地海拔 (按 ISA 大气推气压), 静置采集一段时间
 * 取均值, 偏移 = 参考值 − 实测均值, 保存后对喂入引擎的压强生效:
 *
 * 采集输入来自 middleware/data 的 baro_data 环形缓冲区 (100Hz,
 * 样本含 Pa 气压与芯片温度):
 *   驱动 Pa -> baro_calib_apply -> KF-GINS 气压观测(Pa)
 *   (校准只在 Pa 域加基准偏移; Pa→高度换算在 GIEngine::baroUpdate 内)
 *
 * FinSH 流程 (见 README.md):
 *   barocal ref <pa>      给定参考气压 (Pa)
 *   barocal refalt <m>    给定已知海拔 (m), 模块按 ISA 推参考气压
 *   barocal start [sec]   静置采集 (默认 30s), 结束自动算偏移并保存生效
 *   barocal show/on/off/clear
 */

#ifndef __BARO_CALIB_H__
#define __BARO_CALIB_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 拟合/标定状态快照 (barocal show / 调试用) */
struct baro_calib_status
{
    rt_bool_t   valid;
    rt_bool_t   enabled;
    rt_bool_t   busy;
    double      offset_pa;
    double      cal_temp;           /* 标定时芯片温度 (°C) */
    double      mean_pa;            /* 标定时实测均值 (Pa) */
};

int    baro_calib_init(void);       /* INIT 自动调用: 从 Flash 加载 */
double baro_calib_apply(double pa_raw);
                                     /* 无效/关闭时原样直通 */
rt_bool_t baro_calib_active(void);
void   baro_calib_get_status(struct baro_calib_status *st);

/* 设定参考气压 (Pa); 也可由海拔推算: baro_calib_ref_from_alt(米) */
void     baro_calib_set_ref(double ref_pa);
double   baro_calib_ref_from_alt(double alt_m);

/* 静置采集 seconds 秒 (0=默认), 结束自动算偏移-校验-保存-生效 */
rt_err_t baro_calib_start(rt_uint32_t seconds);
void     baro_calib_cancel(void);
rt_err_t baro_calib_set_enable(rt_bool_t on);

#ifdef __cplusplus
}
#endif

#endif /* __BARO_CALIB_H__ */
