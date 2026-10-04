/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DShot 帧编码实现 (格式与算法见 dshot_enc.h 头注)
 */
#include "dshot_enc.h"

unsigned short dshot_enc_frame(unsigned value, int telem)
{
    unsigned packet;

    value &= 0x07FFU;
    packet = (value << 1) | (telem ? 1U : 0U);
    unsigned csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0FU;

    return (unsigned short)((packet << 4) | csum);
}

unsigned dshot_enc_throttle(double u_norm)
{
    if (!(u_norm > 0.0))
        return 0;                       /* 停转命令 */
    if (u_norm > 1.0)
        u_norm = 1.0;
    return (unsigned)(DSHOT_THR_MIN + u_norm * (DSHOT_THR_MAX - DSHOT_THR_MIN) + 0.5);
}

unsigned short dshot_enc_frame_norm(double u_norm, int telem)
{
    return dshot_enc_frame(dshot_enc_throttle(u_norm), telem);
}

int dshot_enc_is_cmd(unsigned value)
{
    return value <= DSHOT_CMD_MAX;
}
