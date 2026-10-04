/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * MAVLink v2 协议接入实现 (见 mavlink_link.h)
 *
 * 纯协议, 无串口/线程: 喂入侧单线程驱动通道 0 解析状态机, 解出的报文
 * 分发给处理器表; 发送侧把 pack 好的报文序列化后经注入的 sender 写出。
 * 统计与 um982_nmea 同手法, 读侧短关中断拷贝快照。
 */

#include "mavlink_link.h"

#if MAVLINK_LINK_ENABLE

/* ------------------------- 内部状态 ------------------------- */

static struct
{
    mavlink_link_sender_t sender;               /* 发送通道, RT_NULL = 丢弃 */

    struct
    {
        mavlink_msg_handler_t h;                /* RT_NULL = 空槽 */
        rt_uint16_t msgid;                      /* 过滤, 0 = 通配 */
    } handlers[MAVLINK_LINK_MAX_HANDLERS];

    struct mavlink_link_stats stats;
} link;

/* 处理器分发 (喂入线程上下文); 表项指针读原子, 与 attach/detach 并发安全 */
static void dispatch(const mavlink_message_t *msg)
{
    rt_uint8_t k, hit = 0;

    for (k = 0; k < MAVLINK_LINK_MAX_HANDLERS; k++)
    {
        mavlink_msg_handler_t h = link.handlers[k].h;

        if (h != RT_NULL &&
            (link.handlers[k].msgid == 0 ||
             link.handlers[k].msgid == msg->msgid))
        {
            h(msg);          /* 经局部变量调用: 判空与调用间被 detach 置 NULL 会空跳 */
            hit = 1;
        }
    }
    if (hit)
        link.stats.rx_handled++;
}

/* ------------------------- 对外接口 ------------------------- */

rt_size_t mavlink_link_feed(const rt_uint8_t *buf, rt_size_t len)
{
    mavlink_message_t msg;
    mavlink_status_t st;
    rt_size_t got = 0, i;

    if (buf == RT_NULL)
        return 0;

    for (i = 0; i < len; i++)
    {
        rt_uint8_t ret = mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st);

        if (ret == MAVLINK_FRAMING_OK)
        {
            got++;
            link.stats.rx_packets++;
            link.stats.last_msgid = msg.msgid;
            link.stats.last_seq = msg.seq;
            dispatch(&msg);
        }
        else if (ret == MAVLINK_FRAMING_BAD_CRC)
        {
            /* 库约定: 坏 CRC 帧也会拷出报文 (供转发), 这里按错误丢弃 */
            link.stats.rx_crc_err++;
        }
        link.stats.rx_bytes++;
    }
    return got;
}

rt_err_t mavlink_link_attach(mavlink_msg_handler_t handler, rt_uint16_t msgid)
{
    rt_base_t level;
    rt_uint8_t k, slot = MAVLINK_LINK_MAX_HANDLERS;

    if (handler == RT_NULL)
        return -RT_ERROR;

    level = rt_hw_interrupt_disable();
    for (k = 0; k < MAVLINK_LINK_MAX_HANDLERS; k++)
    {
        if (link.handlers[k].h == handler)
        {
            /* 重复注册: 只更新过滤 */
            link.handlers[k].msgid = msgid;
            rt_hw_interrupt_enable(level);
            return RT_EOK;
        }
        if (link.handlers[k].h == RT_NULL && slot == MAVLINK_LINK_MAX_HANDLERS)
            slot = k;
    }
    if (slot == MAVLINK_LINK_MAX_HANDLERS)
    {
        rt_hw_interrupt_enable(level);
        return -RT_EFULL;
    }
    link.handlers[slot].h = handler;
    link.handlers[slot].msgid = msgid;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

void mavlink_link_detach(mavlink_msg_handler_t handler)
{
    rt_base_t level;
    rt_uint8_t k;

    if (handler == RT_NULL)
        return;

    level = rt_hw_interrupt_disable();
    for (k = 0; k < MAVLINK_LINK_MAX_HANDLERS; k++)
    {
        if (link.handlers[k].h == handler)
            link.handlers[k].h = RT_NULL;
    }
    rt_hw_interrupt_enable(level);
}

void mavlink_link_set_sender(mavlink_link_sender_t sender)
{
    link.sender = sender;
}

rt_size_t mavlink_link_send_msg(mavlink_message_t *msg)
{
    rt_uint8_t buf[MAVLINK_MAX_PACKET_LEN];     /* 280B, 栈上帧缓冲 */
    rt_size_t n;

    if (msg == RT_NULL)
        return 0;

    if (link.sender == RT_NULL)
    {
        link.stats.tx_drop++;
        return 0;
    }

    n = mavlink_msg_to_send_buffer(buf, msg);
    link.sender(buf, n);
    link.stats.tx_packets++;
    link.stats.tx_bytes += (rt_uint32_t)n;
    return n;
}

rt_size_t mavlink_link_send_heartbeat(void)
{
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(mavlink_link_sysid(), mavlink_link_compid(), &msg,
                               MAVLINK_LINK_HB_TYPE, MAVLINK_LINK_HB_AUTOPILOT,
                               0, 0, MAVLINK_LINK_HB_STATUS);
    return mavlink_link_send_msg(&msg);
}

void mavlink_link_set_out_v1(rt_bool_t enable)
{
    mavlink_status_t *st = mavlink_get_channel_status(MAVLINK_COMM_0);

    if (enable)
        st->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    else
        st->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}

/* 本机标识: 编译期宏定缺省, mavlink_link_set_ids() 运行期覆盖
 * (多机同链路时经 `gcs id` 改, 持久化在 param_sys 域, 见 mavgcs.c) */
static rt_uint8_t s_sysid  = MAVLINK_LINK_SYS_ID;
static rt_uint8_t s_compid = MAVLINK_LINK_COMP_ID;

rt_uint8_t mavlink_link_sysid(void)
{
    return s_sysid;
}

rt_uint8_t mavlink_link_compid(void)
{
    return s_compid;
}

void mavlink_link_set_ids(rt_uint8_t sysid, rt_uint8_t compid)
{
    s_sysid = sysid;
    s_compid = compid;
}

void mavlink_link_get_stats(struct mavlink_link_stats *out)
{
    rt_base_t level;

    if (out == RT_NULL)
        return;

    level = rt_hw_interrupt_disable();
    *out = link.stats;
    rt_hw_interrupt_enable(level);
}

void mavlink_link_reset(void)
{
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    rt_memset(&link.stats, 0, sizeof(link.stats));
    rt_hw_interrupt_enable(level);
    mavlink_reset_channel_status(MAVLINK_COMM_0);
}

#endif /* MAVLINK_LINK_ENABLE */

/* ------------------------- FinSH 调试命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH) && MAVLINK_LINK_ENABLE
#include <finsh.h>

#define LOG_TAG "mavlink"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

static void mavlink(int argc, char **argv)
{
    struct mavlink_link_stats st;

    mavlink_link_get_stats(&st);

    if (argc >= 2 && rt_strcmp(argv[1], "sendhb") == 0)
    {
        LOG_I("heartbeat sent: %d bytes", mavlink_link_send_heartbeat());
        return;
    }
    if (argc >= 3 && rt_strcmp(argv[1], "v1") == 0)
    {
        mavlink_link_set_out_v1(rt_strcmp(argv[2], "on") == 0);
        LOG_I("out frame: %s", rt_strcmp(argv[2], "on") == 0 ? "v1" : "v2");
        return;
    }

    LOG_I("=== MAVLink v2 link (sysid=%u compid=%u) ===",
          mavlink_link_sysid(), mavlink_link_compid());
    LOG_I("tx      : packets=%u bytes=%u drop=%u (drop>0 = sender 未注入)",
          st.tx_packets, st.tx_bytes, st.tx_drop);
    LOG_I("rx      : bytes=%u packets=%u crc_err=%u handled=%u",
          st.rx_bytes, st.rx_packets, st.rx_crc_err, st.rx_handled);
    LOG_I("          last msgid=%u seq=%u", st.last_msgid, st.last_seq);
    LOG_I("tx      : packets=%u bytes=%u drop=%u",
          st.tx_packets, st.tx_bytes, st.tx_drop);
    LOG_I("usage   : mavlink [sendhb | v1 on|off]");
}
MSH_CMD_EXPORT(mavlink, MAVLink 链路统计与调试);
#endif
