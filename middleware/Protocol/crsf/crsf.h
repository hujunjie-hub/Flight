/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CRSF (Crossfire / ExpressLRS) 遥控链路 — 纯协议解析层
 *
 * ---------------------------------------------------------------------------
 * 定位 (与 um982_nmea / mavlink 同约定: 不占串口、不开线程)
 * ---------------------------------------------------------------------------
 * 串口接收线程 (middleware/Sensor_Preprocessing/process_data/rc_data.c 的 rcrx) 把字节逐个喂进
 * crsf_feed(), 帧同步/CRC/通道解包在本层完成, 解出遥控通道帧时返回 1
 * 并填充归一化通道值; 其余帧类型 (链路统计等) 只计数不解码。
 *
 * 帧格式 (ExpressLRS CRSF 协议, 420000 8N1 半双工):
 *   [sync 0xC8][len][type][payload ...][crc8]
 *   len = type + payload + crc 的字节数 (2..62); crc8 = poly 0xD5,
 *   init 0, 覆盖 type + payload。
 * 本层只消费接收方向; CRSF telemetry (飞控→接收机) 为后续项。
 *
 * RC_CHANNELS_PACKED (type 0x16, payload 22B): 16 通道 × 11bit LSB 优先
 * 连续打包; 原始值域 172..1811, 中点 992 —— 归一化为 [-1,1]:
 *   ch_norm = (raw - 992) / 819.5
 *
 * 通道序约定 (发射机 AETR, ELRS 缺省): ch1=A(副翼, 右=+) ch2=E(升降,
 * 推杆=−) ch3=T(油门) ch4=R(方向, 右=+); ch5-8 = 开关/ Aux。
 * rc_data 层负责把 AETR 换算成本工程统一轴约定 (见 rc_data.h)。
 */
#ifndef __CRSF_H__
#define __CRSF_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_MAX_FRAME          64      /* sync+len + 62 字节载荷上限 */
#define CRSF_NUM_CHAN           16
#define CRSF_TYPE_RC_CHANNELS   0x16    /* 22B: 16ch × 11bit */

struct crsf_channels
{
    float ch[CRSF_NUM_CHAN];    /* [-1,1], 中点 0 (未用通道亦归一化) */
};

struct crsf_stats
{
    rt_uint32_t frames;         /* CRC 通过的总帧数 */
    rt_uint32_t ch_frames;      /* 其中遥控通道帧 (0x16) */
    rt_uint32_t crc_err;        /* CRC 错帧数 */
};

/*
 * 喂入 1 字节 (单线程喂入); 解出完整遥控通道帧时返回 1 并填充 out,
 * 其余情况返回 0。CRC 错帧丢弃并计数。
 */
int crsf_feed(rt_uint8_t byte, struct crsf_channels *out);

/* 统计快照 */
void crsf_get_stats(struct crsf_stats *st);

/* 复位解析状态机与统计 */
void crsf_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H__ */
