/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 四旋翼控制模型 (quad model) — 控制链执行顶层 (FMT 模式移植)
 *
 * ---------------------------------------------------------------------------
 * 定位 (借鉴 FMT, 见 FMT_README §8.3/§12.3 与 ref/FMT-Firmware)
 * ---------------------------------------------------------------------------
 * 把已落地的三级纯 C 控制律 (position_mpc 外环 / attitude_so3 内环 /
 * control_allocation 混控) 组装成一台可运行的四旋翼控制模型, 是 FMT
 * "controller 模型 + vehicle 任务调度" 结构在本工程 (无 FMS/uMCN) 的
 * 对应物:
 *
 *   FMT  mc_controller + task_vehicle          本工程 quad_model
 *   ─────────────────────────────────────    ────────────────────────────
 *   fmt_model_info_t{period, info}             quad_model_info (内外环双周期)
 *   PERIOD_EXECUTE3 按 period 门控             ctl 线程按 ms 时间门控分频
 *   FMS_Out.status 输出分档                    quad_model_status 状态机
 *   (Disarm 停转/Standby 怠速/Arm 闭环)        (同左, 怠速值待硬件定)
 *   FMS_Out.reset 复位积分器                   状态转换时 mpc/att reset
 *   Control_Out_Bus 广播                       quad_model_out 快照
 *
 * 级联与拍频 (与 §12.3 my_controller 同型, 数据源为 gins 快照而非 FMS):
 *
 *   "ctl" 线程 (prio 8, IMU 事件驱动)
 *     ├ 20Hz  pilot_overlay     摇杆叠加: RC/虚拟摇杆 -> 滑设定点+v_ref
 *     │        (死区外=速度指令, 全中位=位置保持; 见下安全约定)
 *     ├ 20Hz  mpc_pos_gins_step  外环: p/v -> a_cmd (NED)
 *     ├ 500Hz att_pid_gins_step  内环: a_cmd+yaw -> thrust_n+alpha_b
 *     │        (a_cmd 无效时内环自动取 0 = 悬停姿态目标)
 *     ├ 500Hz mixer_cmd_step_att 混控: -> u_norm[4]
 *     └ 500Hz 输出门控: dry-run 只进快照; power 使能后写 dshot_out_write
 *
 * ---------------------------------------------------------------------------
 * 安全约定 (默认双重锁定)
 * ---------------------------------------------------------------------------
 * - 上电默认 DISARM + dry-run: 输出硬件不初始化 (dshot 引脚/TIM 见 dshot_hw.h,
 *   见 dshot_hw.h), 控制闭环结果只进 quad_model_out 快照 —— 台架可全程
 *   验证 u_norm 序列而不接电调;
 * - `quad power on` 才初始化输出引擎 (dshot_out_init), `quad arm` 过预检
 *   (gins ready + 未降级 + hold 已设) 后进入 ARM; power 未开时 arm 允许
 *   但保持 dry-run (仅观察);
 * - ARM 期内环/混控连续 25 拍 (50ms) 无有效输出 -> 自动 DISARM + 停转;
 *   非有限输入由 dshot_output 按 0 (停转值) 拒绝, 不出野值。
 * - 遥控 (middleware/data/rc_data, CRSF/ELRS + QGC 虚拟摇杆双源):
 *   真实 RC ch5 低 = kill 立即 disarm; ARM 期 RC 曾在线后失联 >3s =
 *   失效保护自动 disarm; 双源并存真实 RC 优先 (FMT "RC 失联才接管"同义)。
 *
 * FinSH 命令: quad [hold | arm | disarm | power on|off]
 *   hold    捕当前位置+当前航向为悬停目标 (arm 预检项之一)
 *   arm/disarm/power 见上; 无参数打印状态
 */
#ifndef __QUAD_MODEL_H__
#define __QUAD_MODEL_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------- 配置 ---------------------------- */

#define QUAD_THREAD_PRIO        8       /* 计划值: 低于 imudata(7), 高于 gins(9) */
#define QUAD_THREAD_STACK       4096    /* ulog 格式化尖峰教训 (README 栈预算) */
#define QUAD_THREAD_TICK        10

/* ---------------------------- 模型信息 ---------------------------- */

/* FMT fmt_model_info_t 的分频版: 内外环各自周期 (ms) */
struct quad_model_info
{
    rt_uint32_t period_mpc_ms;      /* 位置外环 (MPC) 周期 */
    rt_uint32_t period_att_ms;      /* 姿态内环+混控+输出周期 */
    const char *info;
};

extern const struct quad_model_info quad_model_info;

/* ---------------------------- 飞行状态 ---------------------------- */

/* FMT VehicleStatus 语义的输出分档 */
enum quad_model_status
{
    QUAD_MODEL_DISARM = 0,      /* 停转: 输出层静默, 控制器保持复位 */
    QUAD_MODEL_STANDBY,         /* 待命: 同 DISARM (DShot 怠速值待硬件定) */
    QUAD_MODEL_ARM,             /* 飞行: 三级闭环, 输出生效 */
};

/* ---------------------------- 输出快照 ---------------------------- */

/* 最近一拍全链快照 (关中断拷贝, 任意线程可读; VOFA/日志/地面站消费) */
struct quad_model_out
{
    rt_uint8_t  status;         /* enum quad_model_status */
    rt_uint8_t  dry_run;        /* 1 = 输出硬件未使能, u_norm 仅快照 */
    rt_uint8_t  mpc_ok;         /* 最近一拍外环有效 */
    rt_uint8_t  att_ok;         /* 最近一拍内环+混控有效 */

    rt_uint32_t mpc_cnt;        /* 累计外环步数 */
    rt_uint32_t att_cnt;        /* 累计内环步数 */
    rt_uint32_t out_drop;       /* ARM 期无效拍被压停转的次数 */
    rt_uint8_t  bad_streak;     /* 连续无效拍计数 (达 25 自动 disarm) */

    double      a_cmd[3];       /* 外环期望加速度 NED m/s^2 */
    double      thrust_n;       /* 总推力 N */
    double      alpha_b[3];     /* 角加速度指令 rad/s^2 */
    double      att_err_deg;    /* SO(3) 总姿态误差 deg */
    double      u_norm[4];      /* 归一化电机量 [0,1] */
    rt_int8_t   n_sat;          /* 本拍触界电机数 */
};

/* ---------------------------- 接口 ---------------------------- */

/* 创建 "ctl" 线程 (main() 装配; 默认 DISARM + dry-run) */
int quad_model_init(void);

/* 悬停目标: 捕当前位置为 MPC 参考点+零偏差设定点, 航向取当前值。
 * gins 未就绪返回 -RT_ERROR (arm 预检项)。 */
int quad_model_hold_here(void);

/* 进入 ARM: 预检 gins ready/未降级/hold 已设; power 未开时允许但 dry-run。
 * 成功 0, 预检失败返回 -RT_ERROR。 */
int quad_model_arm(void);

/* 退出到 DISARM: 停转 + 复位控制器 (幂等) */
void quad_model_disarm(void);

/* 使能/关闭输出硬件 (dshot_out_init/deinit; 默认关 = dry-run) */
int quad_model_power_enable(rt_bool_t on);

enum quad_model_status quad_model_get_status(void);

/* 全链快照 (关中断拷贝) */
void quad_model_get_out(struct quad_model_out *out);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_MODEL_H__ */
