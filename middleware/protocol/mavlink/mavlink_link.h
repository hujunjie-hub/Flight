/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink v2 协议接入 (地面站遥测/指令链路)
 *
 * ---------------------------------------------------------------------------
 * 定位
 * ---------------------------------------------------------------------------
 * 本模块与 um982_nmea 同定位: 纯协议层, 不占串口、不开线程。
 * 收方向的字节由串口/数传驱动的接收线程喂进 mavlink_link_feed(),
 * 帧同步、v1/v2 兼容、CRC 校验由 MAVLink 官方 C 库完成; 解出的完整报文
 * 分发给已注册的处理器回调。发方向调用方用官方库的
 * mavlink_msg_xxx_pack() 组报文后交 mavlink_link_send_msg() 序列化,
 * 经注入的 sender 回调写到链路 (串口驱动 rt_device_write 等)。
 *
 *   地面站 ──串口/数传──> 驱动接收线程 ──> mavlink_link_feed()
 *                                │                  │ v2 帧同步 + CRC + v1 兼容
 *                                │                  ▼
 *                                │          mavlink_msg_handler_t 回调
 *                                │          (按 msgid 过滤, 0 = 通配)
 *                                ▼
 *   地面站 <──驱动写── mavlink_link_set_sender() 注入的 sender
 *                          <── mavlink_link_send_msg() (组帧+发序号)
 *
 * MAVLink 官方 C 库 (本目录核心头 + common/ 方言) 取自 ref/FMT-Firmware
 * 的生成件 (common.xml, 2020-08-18; 历史来源为 doc/IMUTest 时代的 FMT
 * 拷贝, 参考工程已迁至 ref/); helper 函数以
 * MAVLINK_SEPARATE_HELPERS 方式单点编译在 mavlink_helpers.c, 通道
 * 缓冲只此一份 (MAVLINK_COMM_NUM_BUFFERS=1)。
 *
 * ---------------------------------------------------------------------------
 * 线程模型
 * ---------------------------------------------------------------------------
 * - feed 只应有一个喂入线程 (通道 0 解析状态机无锁); 处理器回调在喂入
 *   线程上下文执行, 只做解码/入队等快动作, 勿阻塞。
 * - 报文的 pack 与 send 链路 (current_tx_seq 递增) 无锁, 多线程并发
 *   发送需在调用侧自行串行。
 * - 统计读取 mavlink_link_get_data 式短关中断快照, 任意线程可调。
 *
 * ---------------------------------------------------------------------------
 * 硬件 (尚未绑定)
 * ---------------------------------------------------------------------------
 * 板级当前只启用 USART1 (console + VOFA, PA9/PA10) 与 USART2 (UM982,
 * 见 ../nmea/um982_nmea.h); MAVLink 尚未占用物理串口。绑定后接线:
 *   - 接收: 驱动线程读到字节后 mavlink_link_feed(buf, n);
 *   - 发送: mavlink_link_set_sender(写函数), 建议 57600+ (遥测),
 *     心跳 1Hz 起步 (QGC 以心跳识别本机)。
 *
 * FinSH 命令: mavlink [sendhb | v1 on|off]  查看统计 / 发心跳 / 切 v1 帧。
 */

#ifndef __MAVLINK_LINK_H__
#define __MAVLINK_LINK_H__

#include <rtthread.h>

/* MAVLink 官方库编译配置: 必须在包含方言头之前定义。
 * helper 单点编译 (mavlink_helpers.c), 单链路只留 1 份通道缓冲。 */
#ifndef MAVLINK_SEPARATE_HELPERS
#define MAVLINK_SEPARATE_HELPERS
#endif
#ifndef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS  1
#endif

#include "common/mavlink.h"           /* v2.0 库, common 方言 (同目录) */

#ifdef __cplusplus
extern "C" {
#endif


/* ------------------------- 配置 ------------------------- */

/* 是否启用 (置 0 时本模块编译为空, 节省 ROM/RAM) */
#define MAVLINK_LINK_ENABLE         1

/* 本机标识: 心跳/遥测报文的 sysid 与 compid */
#define MAVLINK_LINK_SYS_ID         1
#define MAVLINK_LINK_COMP_ID        MAV_COMP_ID_AUTOPILOT1

/* 心跳内容 (mavlink_link_send_heartbeat 用, 按机型修改 TYPE) */
#define MAVLINK_LINK_HB_TYPE        MAV_TYPE_GENERIC
#define MAVLINK_LINK_HB_AUTOPILOT   MAV_AUTOPILOT_GENERIC
#define MAVLINK_LINK_HB_STATUS      MAV_STATE_ACTIVE

/* 处理器回调表容量 (attach 满则 -RT_EFULL) */
#define MAVLINK_LINK_MAX_HANDLERS   8

/* ------------------------- 统计 ------------------------- */

struct mavlink_link_stats
{
    rt_uint32_t rx_bytes;       /* 喂入字节总数 */
    rt_uint32_t rx_packets;     /* CRC 通过的完整报文 */
    rt_uint32_t rx_crc_err;     /* CRC 错误帧 */
    rt_uint32_t rx_handled;     /* 至少命中一个处理器的报文 */
    rt_uint32_t tx_packets;
    rt_uint32_t tx_bytes;
    rt_uint32_t tx_drop;        /* 未注入 sender 而丢弃的发送 */
    rt_uint32_t last_msgid;     /* 最近一条有效报文的 msgid */
    rt_uint8_t  last_seq;       /* 最近一条有效报文的 seq */
    rt_uint8_t  reserved;       /* 对齐 */
};

/* ------------------------- 接口 ------------------------- */

/* 喂入链路收到的原始字节 (单线程喂入), 返回本次解出的完整有效报文数。
 * CRC 错误帧被丢弃并计入 rx_crc_err, 不进处理器。 */
rt_size_t mavlink_link_feed(const rt_uint8_t *buf, rt_size_t len);

/* 注册报文处理器: msgid 为 0 时通配所有报文; 同一回调重复注册时更新
 * 其过滤 msgid。处理器在喂入线程上下文被调, 需快进快出。 */
typedef void (*mavlink_msg_handler_t)(const mavlink_message_t *msg);
rt_err_t mavlink_link_attach(mavlink_msg_handler_t handler, rt_uint16_t msgid);
void mavlink_link_detach(mavlink_msg_handler_t handler);

/* 注入发送通道 (如串口驱动写函数); RT_NULL 时发送被丢弃计 tx_drop。
 * sender 在发送调用方上下文执行, 应整帧写出 (一次 rt_device_write)。 */
typedef void (*mavlink_link_sender_t)(const rt_uint8_t *buf, rt_size_t len);
void mavlink_link_set_sender(mavlink_link_sender_t sender);

/* 组好的报文 (mavlink_msg_xxx_pack 产物) 序列化成 v2/v1 帧并经 sender
 * 发出, 返回写出的字节数。pack 与 send 需在同一上下文串行调用。 */
rt_size_t mavlink_link_send_msg(mavlink_message_t *msg);

/* 发一帧心跳 (内容见 MAVLINK_LINK_HB_* 配置); sender 未注入时返回 0。 */
rt_size_t mavlink_link_send_heartbeat(void);

/* 发送帧格式切换: v1 帧 (0xFE, 无签名/扩展字段) 兼容老地面站, 默认 v2。 */
void mavlink_link_set_out_v1(rt_bool_t enable);

/* 本机 sysid/compid: 供调用方 mavlink_msg_xxx_pack 组包使用 */
rt_uint8_t mavlink_link_sysid(void);
rt_uint8_t mavlink_link_compid(void);

/* 取统计快照 (短临界区拷贝, 可在任意线程调用) */
void mavlink_link_get_stats(struct mavlink_link_stats *out);

/* 复位统计与通道解析状态 (不动已注册的处理器与 sender) */
void mavlink_link_reset(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_LINK_H__ */
