/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SO(3) 姿态误差 <-> KF-GINS 桥接
 *
 * 数据流:
 *   当前姿态  <- gins_bridge_get_solution() (KF-GINS 融合结果, rpy 单位 deg)
 *   期望姿态  <- so3_target_set_rpy_*()     (上层制导/遥控/任务设定, 关中断发布)
 *   姿态误差  <- so3_att_error()            (体轴 e_b 供控制律, 导航系 e_n 供记录)
 *
 * 目标姿态未设定 / GINS 未就绪时, so3_att_error() 返回 RT_FALSE 且输出清零,
 * 控制律须检查返回值 (零误差输出不能作为"已对准"的依据)。
 */
#ifndef __SO3_GINS_H__
#define __SO3_GINS_H__

#include <rtthread.h>
#include "so3.h"

struct gins_solution;             /* 前置声明, 见 middleware/gins/gins_bridge.h */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 期望(目标)姿态 ------------------------- */

/* 设定目标姿态 (欧拉角)。deg/rad 两个版本; yaw 连续值或 ±180 内值均可,
 * SO(3) 组合对缠绕不敏感。目标由本模块保存, 任意线程可设定。 */
rt_bool_t so3_target_set_rpy_deg(double roll, double pitch, double yaw);
rt_bool_t so3_target_set_rpy_rad(double roll, double pitch, double yaw);

/* 设定目标姿态 (单位四元数, 体->导航) */
rt_bool_t so3_target_set_quat(const struct so3_quat *q);

/* 取目标姿态快照 (关中断拷贝)。返回是否有效; q 可为 NULL (仅查询有效性) */
rt_bool_t so3_target_get(struct so3_quat *q);

/* 清除目标 (此后 so3_att_error 返回 RT_FALSE) */
void so3_target_clear(void);

/* ------------------------- 当前姿态 (KF-GINS) ------------------------- */

/*
 * 读 KF-GINS 最新融合姿态并转为四元数 (体->导航)。
 * 返回 RT_FALSE 表示引擎未就绪 (对准中/等定位), *q 清零。
 */
rt_bool_t so3_current_quat(struct so3_quat *q);

/* 同上, 输出欧拉角 rpy[3] (rad, wrap 到 (-π,π]); 注意 GINS 原始 yaw 可能是
 * 连续值/[0,2π), 本函数已归一化。 */
rt_bool_t so3_current_rpy_rad(double rpy[3]);

/* ------------------------- 姿态误差 ------------------------- */

/*
 * 一步计算姿态误差: 当前 = KF-GINS 最新融合结果, 期望 = so3_target。
 * 返回 RT_FALSE (GINS 未就绪或目标未设定) 时 *out 清零。
 * out->e_b 喂控制律, out->e_n 用于日志/回传, out->angle 为总误差角 (rad)。
 */
rt_bool_t so3_att_error(struct so3_att_err *out);

/* ------------------------- 测试注入口 (板上注入测试) ------------------------- */

/*
 * 注入固定的当前姿态快照, 代替真实 gins 桥接 (板上注入测试用;
 * 主机交叉验证见 build_host/so3_xcheck.py); 传 NULL 恢复真实桥接。
 * 固件正常运行不调用。
 */
void so3_test_inject(const struct gins_solution *sol);

#ifdef __cplusplus
}
#endif

#endif /* __SO3_GINS_H__ */
