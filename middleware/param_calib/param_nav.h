/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 导航参数域 (W25Q64 分区 nav, 见 param_part.h)
 *
 * 把 gins_config.h 中需要按部署地/安装姿态现场调整的编译期参数转为
 * 掉电保持的运行期参数:
 *   - 磁偏角 (GINS_MAG_DECL_DEG): 真航向 = 磁航向 - 磁偏角
 *   - 无 GNSS 部署位置 (GINS_NOGNSS_LAT/LON/ALT_DEG): NOGNSS 模式播种
 *     与位置看门狗基准 (仅 GINS_NOGNSS_MODE 编入时被消费)
 *   - IMU/磁力计轴向映射 (GINS_AXIS_SRC/SIGN, GINS_MAG_AXIS_*):
     安装朝向重排, 消费点在 middleware/data 入环前处理链
 *   - 磁航向观测开关 (GINS_MAG_ENABLE)
 *
 * RAM 镜像以 gins_config.h 宏为缺省值静态初始化 —— 无记录时行为与
 * 纯编译期参数完全一致; FinSH `nav set` 修改镜像, `nav save` 持久化。
 * 其余 gins_config.h 参数 (ZUPT 门限/先验 std/噪声等) 为调参档位,
 * 仍走编译期, 后续有需要再扩 payload (ver 递增, 兼容读旧记录)。
 */

#ifndef __PARAM_NAV_H__
#define __PARAM_NAV_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nav_params
{
    float       mag_decl_deg;       /* 磁偏角 (东偏为正, deg) */
    double      nognss_lat;         /* 无 GNSS 部署纬度 (deg) */
    double      nognss_lon;         /* 经度 (deg) */
    double      nognss_alt;         /* 椭球高 (m) */

    rt_int8_t   imu_axis_src[3];    /* IMU 轴重排: 体轴 i <- 传感器轴 src[i] */
    float       imu_axis_sign[3];   /* 体轴 i 符号 (±1) */

    rt_int8_t   mag_axis_src[3];    /* 磁力计轴重排 (软磁校正之后应用) */
    float       mag_axis_sign[3];

    rt_uint8_t  mag_enable;         /* 磁航向观测开关 (0/1) */
};

/* 上电加载 (INIT_COMPONENT_EXPORT, 幂等): 无记录时保持宏缺省值 */
rt_err_t param_nav_init(void);

/* 参数镜像 (永为有效缺省或已加载值); 供 gins_bridge / data 层热路径
 * 直接读取, 勿在 ISR 中写 */
const struct nav_params *param_nav(void);

/* 镜像序列化追加写入 nav 分区 (nav set 后调用) */
rt_err_t param_nav_save(void);

/* 从 nav 分区重读 (放弃 RAM 修改); 无记录时回到宏缺省值 */
rt_err_t param_nav_load(void);

/* 镜像恢复宏缺省值 (不写 Flash, `nav reset` 后可再 save 固化) */
void param_nav_defaults(void);

/* 镜像当前是否被 nav set 修改而未 save */
rt_bool_t param_nav_dirty(void);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_NAV_H__ */
