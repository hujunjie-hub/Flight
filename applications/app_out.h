/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USART1 调试输出层公共设施 (vofa / imuout / gnssout / gins_fused_data /
 * magout / barout 六条输出链路共用, 见各自 out_*.c)
 */

#ifndef __APP_OUT_H__
#define __APP_OUT_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 创建 USART1 共享写互斥并清除 STREAM 标志, main() 里最先调用 (各链路
 * 线程启动前)。互斥不存在时 app_out_write() 退化为无锁直写 (NULL 容忍)。
 */
void app_out_init(void);

/* 整段加锁写出 (帧/行不被其他数据链插断), 内含 STREAM 标志兜底清除 */
void app_out_write(rt_device_t dev, const void *buf, rt_size_t len);

/* 追加一个带标记浮点字段 (前导空格): "label:-i.ffffff" 拆成 符号+整数+6 位定点
 * (tiny klibc 无 %f, 同 gins_bridge.cpp 做法; 定点 1e6 下满量程 < 2^27,
 * ULP <= 4µ, 远小于传感器 LSB, 无有效精度损失) */
void app_out_cat_fx(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, float v);

/* 追加带标记 double 字段, 7 位定点小数 (lat/lon, ~1cm 分辨率; |x|<214deg) */
void app_out_cat_d7(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, double v);

/* 追加带标记 double 字段, 3 位定点小数 (alt 等; x1e6 在高度 >2147m 时溢出 int32) */
void app_out_cat_d3(char *buf, rt_size_t size, rt_size_t *off,
                    const char *label, double v);

/* USART1 实际波特率 (board.c 统一配置, 各链路日志展示用) */
#define APP_OUT_BAUD_TEXT       "460800"

/* --------------------- 各输出链路初始化 (main() 依次调用) --------------------- */

int vofa_link_init(void);             /* KF-GINS 解算快照 -> JustFloat 二进制帧 */
int imuout_link_init(void);           /* IMU 原始样本 -> 带标记文本 (IMUOUT_ENABLE 门控) */
int gnssout_link_init(void);          /* UM982 定位解 -> 带标记文本 */
int gins_fused_data_link_init(void);  /* KF-GINS 融合结果 -> 带标记文本 */
int magout_link_init(void);           /* BMM350 原始+校准 -> 带标记文本 */
int barout_link_init(void);           /* BMP585 原始+校准 -> 带标记文本 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_OUT_H__ */
