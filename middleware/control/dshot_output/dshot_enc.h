/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DShot 帧编码 —— 纯 C, 无 RT-Thread 依赖, 可主机测试/复用
 *
 * 帧格式 (11+1+4 bit, 高位先发):
 *   [11 bit 节流/命令值][1 bit 遥测请求][4 bit CRC]
 *   CRC = (packet ^ packet>>4 ^ packet>>8) & 0xF, packet = value<<1 | telem
 *   (算法与 ref/FMT-Firmware src/hal/actuator/actuator.c dshot_pack_frame
 *    一致, 编码正确性由 build_host/mpc_xcheck.py 对拍)
 *
 * 节流值语义: 0 = 停转命令; 1..47 = 特殊命令 (BEEP/换向/3D/保存设置);
 * 48..2047 = 线性节流 (48 起转, 2047 满推)。
 */
#ifndef __DSHOT_ENC_H__
#define __DSHOT_ENC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_CMD_MOTOR_STOP       0
#define DSHOT_CMD_BEEP1            1
#define DSHOT_CMD_BEEP2            2
#define DSHOT_CMD_BEEP3            3
#define DSHOT_CMD_BEEP4            4
#define DSHOT_CMD_BEEP5            5
#define DSHOT_CMD_ESC_INFO         6
#define DSHOT_CMD_SPIN_DIR_1       7       /* 正向 */
#define DSHOT_CMD_SPIN_DIR_2       8       /* 反向 */
#define DSHOT_CMD_3D_MODE_OFF      9
#define DSHOT_CMD_3D_MODE_ON       10
#define DSHOT_CMD_SETTINGS_REQUEST 11
#define DSHOT_CMD_SAVE_SETTINGS    12
#define DSHOT_CMD_MAX              47

#define DSHOT_THR_MIN              48      /* 起转 */
#define DSHOT_THR_MAX              2047    /* 满推 */

/* value(11bit) + telem -> 16bit 帧 (含 CRC) */
unsigned short dshot_enc_frame(unsigned value, int telem);

/* 归一化节流 [0,1] -> 11bit 值 (48..2047; 0 映射为停转命令 0) */
unsigned dshot_enc_throttle(double u_norm);

/* 归一化节流 -> 完整帧 */
unsigned short dshot_enc_frame_norm(double u_norm, int telem);

/* 是否特殊命令 (0..47) */
int dshot_enc_is_cmd(unsigned value);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_ENC_H__ */
