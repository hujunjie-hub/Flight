/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 时间同步基座 (README "时间同步框架"章节的实现)
 *
 * 双时间体系:
 *   T_MCU  TIM2 32bit @1MHz 自由计数 + 溢出累计 -> 64 位 µs, 上电即有效,
 *          永不回拨/复位; PPS 校准只估计映射参数, 不修改 TIM2 计数值
 *   T_UTC  UM982 NMEA 语句时标 + PPS 整秒对齐 (Unix 纪元 µs)
 *
 * 硬件 (Flight.ioc, 硬件配置以此为准):
 *   TIM2: PSC=274 (275MHz/275 = 1MHz), ARR=0xFFFFFFFF, 向上自由计数,
 *         溢出中断累计高位 (32bit @1MHz ≈ 71.6min 回绕一次)
 *   TIM2_CH1 = PA0: PPS 输入捕获 (上升沿), 脉冲沿时刻硬件锁存进 CCR1,
 *         消除中断响应延迟抖动 (TIM2_IRQn 抢占优先级 1, 见 board.c)
 *
 * PPS 配对与滑窗校准 (解析线程每秒调用 timebase_pps_pair):
 *   NMEA 整秒语句 (定位有效, GNSS 质量门槛由调用方先判) 与最近一次 PPS
 *   捕获沿配对, 四道门槛任一违例丢弃本对并计数:
 *     1. 定位有效 (调用方判定: pos_valid 且 fix_type 达单点以上)
 *     2. 语句到达距该 PPS 沿 < 0.9s
 *     3. 相邻配对间隔与 UTC 间隔一致 (1s ± 1ms 按间隔等比放宽;
 *        UTC 间隔 > 1.5s 视为失锁后再捕获, 重开滑窗)
 *     4. 斜率合法性 |scale-1| <= 100ppm
 *   配对成功写入 pps_ref 滑窗 (窗口 3, 旧移新进); 映射由窗口最小二乘拟合,
 *   残差 > 10µs 的配对剔除并重拟合, 剩余不足 2 对则映射回退等待新配对。
 *
 * 降级两态模型 (GPS 缺失与降级):
 *   校准态    滑窗满 (3 对) 且持续配对, clock_map 有效, GNSS 样本正常换算
 *   自由运行  TIM2 照常计数打戳, 传感器链路完全不受影响;
 *             失锁 < 10s 沿用冻结映射 (>20ppm 晶振 -> <=200µs 误差),
 *             失锁 > 10s 映射作废 (T_event 置 0), 直至滑窗重建
 *
 * 回绕防护 (对上层透明):
 *   竞态修正   读 CNT/CCR1 后检查 UIF, 已置位且计数值已回绕则高位 +1
 *              (双读溢出计数 + 半量程判别, ISR/线程上下文均安全)
 *   擦写漏计   irq_window_begin/end 供"长关中断窗口"核对补计 —— 原为
 *              片内 Flash 扇区 7 擦写设计; 2026-10-02 标定参数迁移
 *              W25Q64 (param_calib) 后擦写不再关中断, 当前无调用方,
 *              接口保留备用 (未来任何长关中断场景可复用)
 *
 * FinSH: timebase  查看时基/PPS/配对/映射状态
 */

#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 配置 ------------------------- */

/* pps_ref 滑窗大小 (README: 窗口 3, 单点野值被门槛与回归残差双重压制) */
#define TIMEBASE_PPS_WINDOW         3

/* 失锁保持阈值: 距最近一次成功配对超过该时长, 映射作废 (设计值 10s) */
#define TIMEBASE_PPS_HOLD_S         10

/* ------------------------- 数据结构 ------------------------- */

/* PPS 配对: 该秒沿的本地时刻 (TIM2_CH1 硬件捕获) 与 UTC 时刻 (NMEA 整秒) */
struct timebase_pair
{
    rt_uint64_t t_mcu_pps;      /* T_MCU,PPS: PPS 秒沿的本地时刻, us */
    rt_uint64_t t_utc_pps;      /* T_UTC,PPS: 该秒沿对应的 UTC 时间, us */
};

/* UTC 与 T_MCU 仿射映射 (滑窗拟合, 基准点取最新配对 PPS2) */
struct clock_map
{
    rt_uint64_t t_mcu_base;     /* 基准点 T_MCU,PPS2, us */
    rt_uint64_t t_utc_base;     /* 基准点 T_UTC,PPS2, us */
    double      scale;          /* 标度因子 (理想 1, 偏离即晶振频差, 典型 ±20ppm) */
    rt_uint8_t  valid;          /* 就绪标志 (滑窗满且未因失锁超时作废) */
};

/* 状态快照 (FinSH 展示) */
struct timebase_status
{
    rt_uint64_t now_us;         /* 当前 T_MCU, us */
    rt_uint32_t tim_cnt;        /* TIM2->CNT 原始计数 */
    rt_uint32_t ovf_count;      /* 溢出累计次数 */
    rt_uint32_t erase_repair;   /* 擦写窗口回绕补计次数 */

    rt_uint32_t pps_caps;       /* PPS 硬件捕获次数 */
    rt_uint32_t pps_last_age_us;/* 距最近一次 PPS 捕获, us */

    rt_uint32_t pair_ok;        /* 配对成功次数 */
    rt_uint32_t rej_no_pps;     /* 无 PPS 捕获可用 */
    rt_uint32_t rej_arrival;    /* 门槛2: 语句到达距 PPS 沿 >= 0.9s */
    rt_uint32_t rej_interval;   /* 门槛3: 相邻间隔与 UTC 间隔不一致 */
    rt_uint32_t rej_slope;      /* 门槛4: |scale-1| > 100ppm */
    rt_uint32_t rej_residual;   /* 拟合残差剔除的配对数 */
    rt_uint32_t restart_cnt;    /* UTC 间隔 > 1.5s 触发的滑窗重建次数 */

    rt_uint8_t  pps_valid_cnt;  /* 滑窗内有效配对数 (0~3) */
    rt_uint8_t  map_valid;      /* 映射就绪 (含 10s 失锁判定) */
    rt_uint32_t map_age_ms;     /* 距最近一次成功配对, ms */
    struct timebase_pair win[TIMEBASE_PPS_WINDOW];
    struct clock_map map;
};

/* ------------------------- 接口 ------------------------- */

/*
 * 当前 T_MCU (64 位 µs, 上电起单调递增)。ISR/线程上下文均安全;
 * ADIS DR EXTI 等打戳路径在事件 ISR 内直接调用。
 */
rt_uint64_t timebase_now_us(void);

/*
 * PPS 配对 (GNSS 解析线程, 每整秒语句调用一次):
 *   t_utc_pps   语句时标整秒对应的 UTC, us (utc_sec * 1000000)
 *   t_arrival   语句组装完成时刻的 T_MCU, us (timebase_now_us)
 * 定位有效等 GNSS 质量门槛由调用方先判 (门槛1); 本函数做门槛 2~4、
 * 滑窗维护与最小二乘拟合。返回 RT_EOK = 配对成功。
 */
rt_err_t timebase_pps_pair(rt_uint64_t t_utc_pps, rt_uint64_t t_arrival);

/* UTC -> T_MCU (仿射映射换算)。映射未就绪/作废时返回 RT_FALSE 并置 0。 */
rt_bool_t timebase_utc_to_mcu(rt_uint64_t t_utc_us, rt_uint64_t *t_mcu_us);

/* T_MCU -> UTC (反向换算, 供日志/导出对齐外部数据, 不参与融合)。 */
rt_bool_t timebase_mcu_to_utc(rt_uint64_t t_mcu_us, rt_uint64_t *t_utc_us);

/* 映射是否就绪 (滑窗满 且 距最近配对未超 TIMEBASE_PPS_HOLD_S) */
rt_bool_t timebase_map_valid(void);

/*
 * 关中断窗口回绕核对 (任意需要长时间关中断的驱动前后调用; 原调用方
 * calib_store 迁移 W25Q64 后不再关中断, 保留为通用工具):
 *   begin 关中断前取当前时戳快照; end 开中断后核对 CNT 单调性,
 *         检测到回绕未被溢出中断通知即补计, 消除 +71.6min 级静默跳变。
 */
rt_uint64_t timebase_irq_window_begin(void);
void timebase_irq_window_end(rt_uint64_t snap_us);

/* 状态快照 (FinSH `timebase`) */
void timebase_get_status(struct timebase_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __TIMEBASE_H__ */
