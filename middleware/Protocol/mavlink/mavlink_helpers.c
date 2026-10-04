/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink 官方库 helper 函数单点编译
 *
 * mavlink_link.h 定义了 MAVLINK_SEPARATE_HELPERS: 所有协议头里
 * protocol.h 只给出外部声明, 函数体 (含通道收发缓冲 m_mavlink_buffer/
 * m_mavlink_status 与 CRC 表查找 mavlink_get_msg_entry) 只在本编译单元
 * 由 mavlink_helpers.h 定义一次, 固件里不会每个包含 mavlink_link.h 的
 * 文件都复制一份缓冲与代码。
 */

#include "mavlink_link.h"

#include "mavlink_helpers.h"          /* 唯一定义点 (MAVLINK_HELPER 为空) */
