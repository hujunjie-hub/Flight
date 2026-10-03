/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 电调输出 (PWM / DShot) —— 板级引擎接口
 *
 * 控制链末级 (见根 README "飞行控制" 章): u[4] 归一化电机量 (mixer 输出)
 *   PWM:   400Hz, 1000~2000us 脉宽
 *   DShot: 150/300/600, 16bit 帧 (dshot_enc), TIM 更新事件 DMA 经 DMAR
 *          寄存器突发装载 CCR1..4 (一拍 4 电机同步), 帧尾附 1 bit 低电平
 *          复位段, 帧间输出恒低
 *
 * 硬件资源全为占位 (dshot_hw.h, 硬件方案落地后调整); 本模块不建线程,
 * 由未来 ctl 任务按输出周期调 dshot_out_write()。
 *
 * 安全约定:
 *   - init 后默认 **disarm** (输出恒低), `dshot arm` 后才接受 write;
 *   - write 输入非有限值按 0 处理并计数 (电机停转值), 不输出野值;
 *   - disarm 立即停 DShot DMA + CCR 清零 (恒低) / PWM CCR 清零。
 */
#ifndef __DSHOT_H__
#define __DSHOT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 输出协议 */
enum dshot_out_proto
{
    DSHOT_OUT_PWM = 0,      /* 传统 PWM 400Hz/1000-2000us */
    DSHOT_OUT_DSHOT150,
    DSHOT_OUT_DSHOT300,
    DSHOT_OUT_DSHOT600,
};

/* 初始化输出引擎 (TIM1/PE9/PE11/PE13/PE14 + DMA1_Stream5, 见 dshot_hw.h),
 * 初始为 disarm 态。
 * 重复调用返回 -RT_EBUSY (先 dshot_out_deinit)。成功 0。 */
int dshot_out_init(enum dshot_out_proto proto);

void dshot_out_deinit(void);

/* 使能输出 (此后 write 生效) / 立即停转并禁输出 */
int dshot_out_arm(void);
void dshot_out_disarm(void);

/* 写 4 电机归一化量 [0,1] (>1 截 1)。仅 armed 态生效, 返回 0;
 * 未初始化/未 arm 返回 -RT_ERROR; 非有限输入按 0 (停转) 计数处理。 */
int dshot_out_write(const double u[4]);

/* 全通道发 11bit 原始值 (0..47 为特殊命令: 停转/BEEP/换向/保存设置,
 * 见 dshot_enc.h); PWM 协议下 0..47 映射为 1000us 静默。 */
int dshot_out_write_raw(unsigned value);

rt_bool_t dshot_out_armed(void);
rt_bool_t dshot_out_inited(void);

struct dshot_out_status
{
    rt_bool_t  inited;
    rt_bool_t  armed;
    int        proto;
    rt_uint32_t write_cnt;
    rt_uint32_t bad_cnt;       /* 非有限输入被拒/截 0 次数 */
    rt_uint32_t busy_cnt;      /* 上一帧未发完又来写的次数 (输出率过高) */
};

void dshot_out_get_status(struct dshot_out_status *st);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_H__ */
