/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * QGC 地面站链路装配 (MAVLink v2, 阶段 0: USART1 复用, FMT_README §13)
 *
 * ---------------------------------------------------------------------------
 * 定位与数据流
 * ---------------------------------------------------------------------------
 * 纯装配层: 协议 (帧同步/CRC/pack) 在 middleware/protocol/mavlink (官方库
 * + mavlink_link 适配层), 数据在 gins 桥接快照 / gnss_data 镜像, 本文件只
 * 做三件事 —— 串口收发、周期消息装配、握手应答:
 *
 *   QGC ──数传电台──> USART2 (PD5/PD6) RX ──rx_indicate──> "mavgcs" 线程
 *          └─ mavlink_link_feed() → handler (COMMAND_LONG 握手/心跳记录)
 *   QGC <──app_out_write(整帧互斥)── mavlink_link_send_msg() ← 各 pack
 *
 * 与 FMT (ref/FMT-Firmware) 的对应关系 (FMT_README §13.1-13.2):
 *   mavgcs 线程 ≈ mav_rx + mavgcs 两线程合一 (单 GCS 通道, 无 OBC);
 *   周期消息表 ≈ mavgcs_init() 的 mavproxy_register_msg 集;
 *   握手应答 (AUTOPILOT_VERSION/PROTOCOL_VERSION/COMMAND_ACK) 与
 *   HEARTBEAT "伪装 PX4" (autopilot=PX4, flight_sw_version=v1.10.0) 均
 *   照抄 FMT task_comm.c/mavgcs.c 行为 (QGC 机架识别依赖此伪装)。
 *
 * ---------------------------------------------------------------------------
 * 串口 (2026-10-04 阶段 1: 物理换口到数传电台)
 * ---------------------------------------------------------------------------
 * USART2 (PD5 TX/PD6 RX @115200 8N1, 数传电台专用, Flight.ioc):
 *   - 发向: 每条 MAVLink 帧一次 app_out_write() (uart1wr 互斥 + STREAM
 *     兜底, 互斥与 console 共用属无害串行), 帧内不与日志交错;
 *   - 收向: uart2 独占本模块 (无 finsh 抢占), `gcs on` 起 rx_indicate
 *     唤醒线程, 上行字节全部进 mavlink_link_feed; `gcs off` 恢复原
 *     回调 (为 NULL 时等价空操作), 与阶段 0 console 共口行为一致。
 *   - 波特率: 打开设备后显式下发 GCS_UART_BAUD (默认 115200, 电台侧
 *     改波特率时同步改此宏与 GCS_UART_BAUD_TEXT)。
 *
 * 消息集 (FMT_README §13.4 装配表): HEARTBEAT+SYS_STATUS 1Hz,
 * ATTITUDE_QUATERNION 10Hz, LOCAL_POSITION_NED/GLOBAL_POSITION_INT/
 * VFR_HUD/GPS_RAW_INT 5Hz, STATUSTEXT 事件式。
 *
 * 上行指令 (2026-10-03 增, §13.5 任务#3 "QGC 点地图→定点" 通路):
 *   - COMMAND_LONG 519/520 握手应答;
 *   - COMMAND_LONG 400 解锁/上锁 -> quad_model 状态机 (带预检, 拒绝回
 *     MAV_RESULT_DENIED, QGC 弹窗显示原因);
 *   - COMMAND_INT DO_REPOSITION (QGC 地图 "Go to location") -> LLA 钳位
 *     换算 -> mpc_pos_gins_set_sp_lla() 三维定点 (param4 有航向则同拍设
 *     期望 yaw); 关中断执行, 与 ctl 线程的 mpc/att step 互斥;
 *   - SET_MODE: 仅接受 POSCTL (本机唯一模式 = MPC 三维位置保持, 值取
 *     FMT px4_custom_mode.h 编码), 其余忽略 (SET_MODE 无 ACK, 心跳
 *     custom_mode 如实上报)。
 *   - MANUAL_CONTROL (QGC 虚拟摇杆) -> rc_data 发布 (真实 CRSF 在线时
 *     忽略, 双源真实优先); RC_CHANNELS 5Hz 回显摇杆量。
 *   - GPS_RTCM_DATA (QGC NTRIP 改正数) -> 分片重组后整包回注 UART4
 *     (UM982 同口, RTK 链; `gcs rtcm off` 可关)。
 *   心跳同步升级: armed/ACTIVE 直接反映 quad_model 状态机。
 *   参数/任务协议 (param_calib/Mission) 仍为后续项。
 *
 * FinSH 命令: gcs [on | off]   查看状态 / 启停会话
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <math.h>

#include "mavlink_link.h"           /* MAVLink v2 适配层 (含官方库配置) */
#include "app_out.h"                /* app_out_write(): USART1 共享写 */
#include "gins_bridge.h"            /* gins_bridge_get_solution(): 解算快照 */
#include "gnss_data.h"              /* gnss_data_peek_latest(): 原始 GNSS */
#include "um982_nmea.h"             /* enum gnss_fix_type (GGA 定位质量) */
#include "imu_data.h"               /* imu_data_peek_latest(): 陀螺角速率 */
#include "so3.h"                    /* so3_euler_to_quat(): rpy->quat */
#include "quad_model.h"             /* quad_model_arm/disarm/get_status(): 上行指令 */
#include "mpc_pos_gins.h"           /* mpc_pos_gins_set_sp_lla(): DO_REPOSITION 定点 */
#include "att_pid_gins.h"           /* att_pid_gins_set_yaw_deg(): 期望航向 */
#include "rc_data.h"                /* rc_data_publish(): MANUAL_CONTROL 虚拟摇杆 */

/* ulog 日志: LOG_E/LOG_W/LOG_I/LOG_D, 行尾自动补 \r\n */
#define LOG_TAG "gcs"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

#define GCS_UART_DEV            "uart2"     /* 数传电台专用 (PD5 TX/PD6 RX) */
#define GCS_UART_BAUD_TEXT      "115200"    /* QGC 串口连接参数 (电台侧同步) */
#define GCS_UART_BAUD           115200u     /* 打开设备后显式下发 (电台默认值) */

#define GCS_THREAD_PRIO         12          /* 与 vofa 同级: 低于 gins(9), 高于 FinSH(20) */
#define GCS_THREAD_STACK        4096
#define GCS_THREAD_TICK         10

#define GCS_POLL_TICKS_MS       20          /* 线程节拍 (10Hz 消息粒度足够) */
#define GCS_IDLE_TICKS_MS       500         /* 会话关闭时的慢轮询 */

#define GCS_PERIOD_HB_MS        1000        /* HEARTBEAT + SYS_STATUS */
#define GCS_PERIOD_ATT_MS       100         /* ATTITUDE_QUATERNION 10Hz */
#define GCS_PERIOD_NAV_MS       200         /* LOCAL/GLOBAL/VFR/GPS 5Hz */

/* 本地 NED 原点球面换算 (同 mpc_pos_gins.c 的 latlon_to_ned 公式) */
#define GCS_EARTH_RE_M          6378137.0
#define GCS_D2R                 0.017453292519943295

/* QGC 握手伪装参数 (照抄 FMT mavgcs.c: QGC 机架识别依赖 PX4 版本号) */
#define GCS_PX4_SW_VERSION      ((1u << 24) | (10u << 16))   /* v1.10.0 */

/* PX4 custom_mode 编码 (值取自 FMT px4_custom_mode.h): 本机唯一支持
 * POSCTL (= MPC 三维位置保持), custom_mode = MAIN<<16 | SUB<<24 */
#define PX4_CUSTOM_MAIN_MODE_POSCTL          3
#define PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL    0
#define GCS_CUSTOM_MODE         ((PX4_CUSTOM_MAIN_MODE_POSCTL << 16) | \
                                 (PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL << 24))

/* DO_REPOSITION 安全钳位界 (与 mpc_pos_gins_set_sp_lla 内钳位一致) */
#define GCS_REPO_HORIZ_MAX_M    50.0
#define GCS_REPO_VERT_MAX_M     10.0

/* rad -> deg (不依赖 M_PI 宏: 工程 _POSIX_C_SOURCE=1 下 newlib 隐藏之) */
#define GCS_RAD2DEG             57.29577951308232

/* RTCM 改正数回注口 = UM982 同口 (UART4 双向, gnss_data 已 RDWR 打开) */
#define GCS_RTCM_DEV            "uart4"

/* MANUAL_CONTROL 无效轴哨兵值 (INT16_MAX) 判定阈值 */
#define GCS_MC_AXIS_INVALID     30000

/* SYS_STATUS 传感器位: 板载件常置 present/enabled, health 按观测计数 */
#define GCS_SENSOR_BITS         (MAV_SYS_STATUS_SENSOR_3D_GYRO | \
                                 MAV_SYS_STATUS_SENSOR_3D_ACCEL | \
                                 MAV_SYS_STATUS_SENSOR_3D_MAG | \
                                 MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | \
                                 MAV_SYS_STATUS_SENSOR_GPS)

/* ---------------------------- 运行状态 ---------------------------- */

static struct
{
    rt_device_t uart;
    rt_thread_t thread;
    rt_sem_t    rx_sem;

    rt_bool_t   on;                     /* 会话开关 (默认关: uart2 数传专用口) */

    /* RX 接管: 保存 finsh 的 rx_indicate, gcs off 时归还 */
    rt_err_t  (*saved_rx_ind)(rt_device_t dev, rt_size_t size);
    rt_bool_t   rx_taken;

    /* 周期节拍 */
    rt_tick_t   tick_hb;
    rt_tick_t   tick_att;
    rt_tick_t   tick_nav;

    /* 本地 NED 原点 (首个 ready 且经纬度有效的解算快照) */
    rt_bool_t   origin_valid;
    double      lat0_deg, lon0_deg, alt0_m;

    /* STATUSTEXT 边沿检测 (初值 -1: 首拍只采样不发) */
    rt_int8_t   prev_ready, prev_degraded, prev_nognss;
    rt_uint32_t prev_nan_cnt;

    rt_tick_t   last_gcs_hb;            /* 最近一帧地面站心跳 */
    rt_uint32_t tx_cnt;                 /* 本会话累计发送帧数 */

    /* RTCM 改正数回注 (QGC NTRIP -> UART4 -> UM982) */
    rt_device_t rtcm_uart;
    rt_bool_t   rtcm_on;                /* 默认开, `gcs rtcm off` 关 */
    rt_uint32_t rtcm_bytes, rtcm_pkts, rtcm_drop;
} ctx;

/* 前置声明: 下方上行 handler 组包后经发送通道发出 */
static rt_size_t gcs_send(mavlink_message_t *msg);

/* RTCM 分片重组 (GPS_RTCM_DATA: 4 x 180B, 乱序/换序即弃等重发) */
static struct
{
    rt_uint8_t  seq;                    /* 5 bit sequence ID */
    rt_uint16_t len;                    /* 已组装字节数 */
    rt_bool_t   busy;
    rt_uint8_t  buf[4 * 180];
} rtcm_asm;

/* 收完一条完整 RTCM 报文 -> 整包写 UART4 (UM982 NMEA 同口) */
static void gcs_rtcm_forward(const rt_uint8_t *data, rt_uint16_t len)
{
    ctx.rtcm_pkts++;
    ctx.rtcm_bytes += len;
    if (!ctx.rtcm_on || ctx.rtcm_uart == RT_NULL)
    {
        ctx.rtcm_drop++;
        return;
    }
    if (rt_device_write(ctx.rtcm_uart, 0, data, len) != len)
        ctx.rtcm_drop++;
}

static void gcs_handle_gps_rtcm(const mavlink_message_t *msg)
{
    mavlink_gps_rtcm_data_t r;

    mavlink_msg_gps_rtcm_data_decode(msg, &r);

    if (!(r.flags & 0x01))
    {
        /* 未分片: 单片即完整报文 */
        gcs_rtcm_forward(r.data, r.len);
        return;
    }

    {
        rt_uint8_t frag = (r.flags >> 1) & 0x03;
        rt_uint8_t seq = (r.flags >> 3) & 0x1F;

        if (!rtcm_asm.busy || rtcm_asm.seq != seq || frag == 0)
        {
            rtcm_asm.seq = seq;
            rtcm_asm.len = 0;
            rtcm_asm.busy = RT_TRUE;
        }
        /* 只接受顺序到达的分片 (frag 序号 = 已组装长度/180) */
        if ((rt_uint16_t)frag * 180u != rtcm_asm.len ||
            rtcm_asm.len + r.len > sizeof(rtcm_asm.buf))
            return;

        memcpy(&rtcm_asm.buf[rtcm_asm.len], r.data, r.len);
        rtcm_asm.len += r.len;

        /* 收到非满片 (= 末片) 或 4 片全满即冲刷 */
        if (r.len < 180 || rtcm_asm.len >= sizeof(rtcm_asm.buf))
        {
            gcs_rtcm_forward(rtcm_asm.buf, rtcm_asm.len);
            rtcm_asm.busy = RT_FALSE;
        }
    }
}

/*
 * MANUAL_CONTROL (QGC 虚拟摇杆): 轴 [-1000,1000] -> rc_data 统一约定。
 * 真实 CRSF 在线 (500ms 内) 时忽略, 与 FMT "RC 失联才接管" 同义。
 * buttons bit0-3 -> aux[0..3] (按下 1 / 释放 0, 不会触发 kill 负值)。
 */
static void gcs_handle_manual_control(const mavlink_message_t *msg)
{
    mavlink_manual_control_t m;
    struct rc_data rc;
    float axis[4], aux[4] = {0};

    mavlink_msg_manual_control_decode(msg, &m);
    if (m.target != 0 && m.target != mavlink_link_sysid())
        return;

    rc_data_get_latest(&rc);
    if (rc.src == RC_SRC_CRSF && rc.valid)
        return;                         /* 真实 RC 优先 */

    /* MAVLink 轴: x=前飞+ y=右飞+ z=推力+ r=逆时针+ (INT16_MAX=无效);
     * 本工程统一约定 (rc_data.h): yaw 右转为 + -> r 取反 */
    axis[0] = (m.x > GCS_MC_AXIS_INVALID || m.x < -GCS_MC_AXIS_INVALID)
              ? 0.0f : (float)m.x / 1000.0f;
    axis[1] = (m.y > GCS_MC_AXIS_INVALID || m.y < -GCS_MC_AXIS_INVALID)
              ? 0.0f : (float)m.y / 1000.0f;
    axis[3] = (m.z > GCS_MC_AXIS_INVALID || m.z < -GCS_MC_AXIS_INVALID)
              ? 0.0f : (float)m.z / 1000.0f;
    axis[2] = (m.r > GCS_MC_AXIS_INVALID || m.r < -GCS_MC_AXIS_INVALID)
              ? 0.0f : -(float)m.r / 1000.0f;

    for (int i = 0; i < 4; i++)
        aux[i] = (m.buttons & (1u << i)) ? 1.0f : 0.0f;

    rc_data_publish(RC_SRC_VIRTUAL, axis, aux);
}

/* RC_CHANNELS 5Hz: rc_data 快照回显 (QGC 摇杆/无线电页) */
static void gcs_send_rc_channels(void)
{
    mavlink_message_t msg;
    struct rc_data r;
    rt_uint16_t ch[8];
    int i;

    rc_data_get_latest(&r);
    for (i = 0; i < 4; i++)
        ch[i] = (rt_uint16_t)(1500 + r.axis[i] * 500.0f);
    for (i = 4; i < 8; i++)
        ch[i] = (rt_uint16_t)(1500 + r.aux[i - 4] * 500.0f);

    mavlink_msg_rc_channels_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                 &msg, rt_tick_get_millisecond(), 8,
                                 ch[0], ch[1], ch[2], ch[3],
                                 ch[4], ch[5], ch[6], ch[7],
                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 r.valid ? 25 : 0);
    gcs_send(&msg);
}

/* ---------------------------- 发送通道 ---------------------------- */

/* mavlink_link sender 注入: 整帧一次写出, 与 ulog 行互斥不插帧 */
static void gcs_tx(const rt_uint8_t *buf, rt_size_t len)
{
    app_out_write(ctx.uart, buf, len);
}

static rt_size_t gcs_send(mavlink_message_t *msg)
{
    rt_size_t n = mavlink_link_send_msg(msg);
    if (n > 0)
        ctx.tx_cnt++;
    return n;
}

/* ---------------------------- 握手应答 (上行处理) ----------------------------
 * 处理器在 mavgcs 线程 (feed 调用方) 上下文执行, 可直接组帧回发。 */

static void gcs_send_autopilot_version(void)
{
    mavlink_message_t msg;
    static const rt_uint8_t zero8[8] = {0};
    static const rt_uint8_t zero18[18] = {0};

    /* capabilities 只声明实际支持的: MAVLink2 + 参数浮点协议 (阶段 2) */
    mavlink_msg_autopilot_version_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                       &msg,
                                       MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                       MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT,
                                       GCS_PX4_SW_VERSION,      /* flight  */
                                       GCS_PX4_SW_VERSION,      /* middleware */
                                       0,                       /* os */
                                       1,                       /* board */
                                       zero8, zero8, zero8,
                                       0, 0, 0, zero18);
    gcs_send(&msg);
}

static void gcs_send_protocol_version(void)
{
    mavlink_message_t msg;
    static const rt_uint8_t zero8[8] = {0};

    mavlink_msg_protocol_version_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                      &msg, 200, 100, 200, zero8, zero8);
    gcs_send(&msg);
}

static void gcs_send_command_ack(rt_uint16_t command, uint8_t result,
                                 uint8_t target_sys, uint8_t target_comp)
{
    mavlink_message_t msg;

    mavlink_msg_command_ack_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                 &msg, command, result,
                                 0, 0, target_sys, target_comp);
    gcs_send(&msg);
}

static void gcs_handle_command_long(const mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;

    mavlink_msg_command_long_decode(msg, &cmd);

    /* 目标为本机或广播才应答 (QGC 常以 sysid 定向) */
    if (cmd.target_system != 0 &&
        cmd.target_system != mavlink_link_sysid())
        return;

    switch ((int)cmd.command)
    {
    case MAV_CMD_REQUEST_PROTOCOL_VERSION:
        gcs_send_command_ack(cmd.command, MAV_RESULT_ACCEPTED,
                             msg->sysid, msg->compid);
        gcs_send_protocol_version();
        break;
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        gcs_send_command_ack(cmd.command, MAV_RESULT_ACCEPTED,
                             msg->sysid, msg->compid);
        gcs_send_autopilot_version();
        break;
    case MAV_CMD_COMPONENT_ARM_DISARM:
    {
        /* param1: 0=上锁 1=解锁; param2=21196 强制 (视同普通请求, 预检不豁免) */
        uint8_t result = MAV_RESULT_DENIED;

        if (cmd.param1 > 0.5f)
        {
            if (quad_model_arm() == RT_EOK)
                result = MAV_RESULT_ACCEPTED;
        }
        else
        {
            quad_model_disarm();
            result = MAV_RESULT_ACCEPTED;
        }
        gcs_send_command_ack(cmd.command, result, msg->sysid, msg->compid);
        break;
    }
    default:
        break;      /* 其余指令 (takeoff/land/校准等) 后续按需实现 */
    }
}

/*
 * COMMAND_INT: QGC 地图 "Go to location" 走 MAV_CMD_DO_REPOSITION
 * (x/y = lat/lon * 1e7, z = 高度 m, param4 = 期望航向 rad, NaN 保持)。
 * 高度口径与 GLOBAL_POSITION_INT 上报一致 (gins 椭球高, QGC 往返自洽)。
 * 关中断执行设定: 与 ctl 线程的 mpc/att step 互斥 (桥接层约定 set/step
 * 单执行者; 中断锁期间调度器不切线程)。
 */
static void gcs_handle_command_int(const mavlink_message_t *msg)
{
    mavlink_command_int_t c;

    mavlink_msg_command_int_decode(msg, &c);

    if (c.target_system != 0 &&
        c.target_system != mavlink_link_sysid())
        return;

    switch ((int)c.command)
    {
    case MAV_CMD_DO_REPOSITION:
    {
        double lat = (double)c.x * 1.0e-7;
        double lon = (double)c.y * 1.0e-7;
        rt_err_t rc;

        if (c.frame != MAV_FRAME_GLOBAL &&
            c.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT)
        {
            gcs_send_command_ack(c.command, MAV_RESULT_DENIED,
                                 msg->sysid, msg->compid);
            return;
        }

        {
            rt_base_t lv = rt_hw_interrupt_disable();

            rc = mpc_pos_gins_set_sp_lla(lat, lon, (double)c.z);
            if (rc == RT_EOK && !isnan(c.param4))
                att_pid_gins_set_yaw_deg(c.param4 * GCS_RAD2DEG);
            rt_hw_interrupt_enable(lv);
        }

        if (rc == RT_EOK)
        {
            double sp[3], vv[3];

            mpc_pos_gins_get_sp(sp, vv);
            LOG_I("reposition -> sp NED (%d %d %d) m (钳位界 h%d/v%d m)",
                  (int)sp[0], (int)sp[1], (int)sp[2],
                  (int)GCS_REPO_HORIZ_MAX_M, (int)GCS_REPO_VERT_MAX_M);
            gcs_send_command_ack(c.command, MAV_RESULT_ACCEPTED,
                                 msg->sysid, msg->compid);
        }
        else
        {
            LOG_W("reposition denied: gins not ready");
            gcs_send_command_ack(c.command, MAV_RESULT_DENIED,
                                 msg->sysid, msg->compid);
        }
        break;
    }
    default:
        break;
    }
}

/* SET_MODE (无 ACK, 接受与否由心跳 custom_mode 反映): 仅 POSCTL 有效 */
static void gcs_handle_set_mode(const mavlink_message_t *msg)
{
    mavlink_set_mode_t m;

    mavlink_msg_set_mode_decode(msg, &m);

    if (m.target_system != 0 &&
        m.target_system != mavlink_link_sysid())
        return;

    if (m.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    {
        unsigned main_mode = (m.custom_mode >> 16) & 0xFF;

        if (main_mode != PX4_CUSTOM_MAIN_MODE_POSCTL)
            LOG_W("set_mode: main_mode %u unsupported (POSCTL only)",
                  main_mode);
    }
}

static void gcs_on_msg(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        ctx.last_gcs_hb = rt_tick_get();
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        gcs_handle_command_long(msg);
        break;
    case MAVLINK_MSG_ID_COMMAND_INT:
        gcs_handle_command_int(msg);
        break;
    case MAVLINK_MSG_ID_SET_MODE:
        gcs_handle_set_mode(msg);
        break;
    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        gcs_handle_manual_control(msg);
        break;
    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
        gcs_handle_gps_rtcm(msg);
        break;
    default:
        break;
    }
}

/* ---------------------------- 周期消息装配 ---------------------------- */

/* 姿态四元数 + 体轴角速率: gins 欧拉 (deg) -> quat, 陀螺直通 */
static void gcs_send_attitude(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    double rpy_rad[3] = { sol->roll * GCS_D2R,
                          sol->pitch * GCS_D2R,
                          sol->yaw * GCS_D2R };
    struct so3_quat q;
    struct imu_sample imu;
    rt_uint32_t seq;
    float p = 0.0f, qr = 0.0f, r = 0.0f;

    so3_euler_to_quat(rpy_rad, &q);
    if (imu_data_peek_latest(&imu, &seq) == RT_EOK)
    {
        p = imu.gyro[0];
        qr = imu.gyro[1];
        r = imu.gyro[2];
    }

    /* repr_offset_q: 不支持处按协议发 [0,0,0,0] (pack 内 memcpy, 不可传 NULL) */
    {
        static const float zero_q[4] = {0.0f};

        mavlink_msg_attitude_quaternion_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                             &msg, rt_tick_get_millisecond(),
                                             (float)q.w, (float)q.x, (float)q.y,
                                             (float)q.z, p, qr, r, zero_q);
    }
    gcs_send(&msg);
}

/* 捕获本地 NED 原点 (首个 ready 解算快照), 输出北/东/下 (m) */
static void gcs_local_ned(const struct gins_solution *sol,
                          double pos_ned[3], double *rel_alt)
{
    if (!ctx.origin_valid && sol->ready &&
        fabs(sol->latitude) > 0.01 && fabs(sol->longitude) > 0.01)
    {
        ctx.lat0_deg = sol->latitude;
        ctx.lon0_deg = sol->longitude;
        ctx.alt0_m = sol->altitude;
        ctx.origin_valid = RT_TRUE;
        LOG_I("home origin: lat:%d lon:%d alt:%d dm",
              (int)(ctx.lat0_deg * 10.0), (int)(ctx.lon0_deg * 10.0),
              (int)(ctx.alt0_m * 10.0));
    }

    if (ctx.origin_valid)
    {
        pos_ned[0] = (sol->latitude - ctx.lat0_deg) * GCS_D2R * GCS_EARTH_RE_M;
        pos_ned[1] = (sol->longitude - ctx.lon0_deg) * GCS_D2R
                     * GCS_EARTH_RE_M * cos(ctx.lat0_deg * GCS_D2R);
        pos_ned[2] = -(sol->altitude - ctx.alt0_m);
        *rel_alt = sol->altitude - ctx.alt0_m;
    }
    else
    {
        pos_ned[0] = pos_ned[1] = pos_ned[2] = 0.0;
        *rel_alt = 0.0;
    }
}

static void gcs_send_local_position(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    double pos_ned[3], rel_alt;

    gcs_local_ned(sol, pos_ned, &rel_alt);

    mavlink_msg_local_position_ned_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                        &msg, rt_tick_get_millisecond(),
                                        (float)pos_ned[0], (float)pos_ned[1],
                                        (float)pos_ned[2],
                                        (float)sol->vn, (float)sol->ve,
                                        (float)sol->vd);
    gcs_send(&msg);
}

static void gcs_send_global_position(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    double pos_ned[3], rel_alt;
    rt_uint32_t hdg;

    gcs_local_ned(sol, pos_ned, &rel_alt);
    hdg = (rt_uint32_t)fmod(sol->yaw * 100.0, 36000.0);

    mavlink_msg_global_position_int_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                         &msg, rt_tick_get_millisecond(),
                                         (rt_int32_t)(sol->latitude * 1.0e7),
                                         (rt_int32_t)(sol->longitude * 1.0e7),
                                         (rt_int32_t)(sol->altitude * 1000.0),
                                         (rt_int32_t)(rel_alt * 1000.0),
                                         (rt_int16_t)(sol->vn * 100.0),
                                         (rt_int16_t)(sol->ve * 100.0),
                                         (rt_int16_t)(sol->vd * 100.0),
                                         (rt_uint16_t)hdg);
    gcs_send(&msg);
}

static void gcs_send_vfr_hud(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    float gs = (float)sqrt(sol->vn * sol->vn + sol->ve * sol->ve);

    mavlink_msg_vfr_hud_pack(mavlink_link_sysid(), mavlink_link_compid(),
                             &msg, 0.0f, gs, (rt_int16_t)sol->yaw, 0,
                             (float)sol->altitude, (float)-sol->vd);
    gcs_send(&msg);
}

/* gnss_fix_type (um982_nmea.h, GGA 定位质量) -> GPS_FIX_TYPE (MAVLink) */
static uint8_t gcs_map_fix_type(rt_uint8_t fix)
{
    switch (fix)
    {
    case GNSS_FIX_GPS:      return GPS_FIX_TYPE_3D_FIX;
    case GNSS_FIX_DGPS:     return GPS_FIX_TYPE_DGPS;
    case GNSS_FIX_PPS:      return GPS_FIX_TYPE_3D_FIX;
    case GNSS_FIX_RTK_FIX:  return GPS_FIX_TYPE_RTK_FIXED;
    case GNSS_FIX_RTK_FLOAT:return GPS_FIX_TYPE_RTK_FLOAT;
    default:                return GPS_FIX_TYPE_NO_GPS;
    }
}

static void gcs_send_gps_raw(void)
{
    mavlink_message_t msg;
    struct gnss_sample g;
    rt_uint32_t seq;

    if (gnss_data_peek_latest(&g, &seq) != RT_EOK)
        return;

    /* eph = HDOP*100 (cm); 无垂向/航向信息处填未知值 */
    mavlink_msg_gps_raw_int_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                 &msg,
                                 (rt_uint64_t)rt_tick_get_millisecond() * 1000u,
                                 gcs_map_fix_type(g.fix_type),
                                 (rt_int32_t)(g.latitude_deg * 1.0e7),
                                 (rt_int32_t)(g.longitude_deg * 1.0e7),
                                 (rt_int32_t)(g.altitude_m * 1000.0),
                                 (rt_uint16_t)(g.hdop * 100.0f),
                                 UINT16_MAX, UINT16_MAX, UINT16_MAX,
                                 g.satellites,
                                 (rt_int32_t)(g.altitude_m * 1000.0),
                                 UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
                                 0);
    gcs_send(&msg);
}

static void gcs_send_sys_status(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    rt_uint32_t health = 0;

    /* health 位与 present 对齐: 对应观测已入引擎才算健康 */
    if (sol->imu_cnt > 0)
        health |= MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    if (sol->mag_cnt > 0)
        health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    if (sol->baro_cnt > 0)
        health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (sol->gnss_cnt > 0)
        health |= MAV_SYS_STATUS_SENSOR_GPS;

    /* 无电池监测/负载遥测: 数值未知位填默认 (QGC 电池区显示 0, 台架可接受) */
    mavlink_msg_sys_status_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                &msg, GCS_SENSOR_BITS, GCS_SENSOR_BITS, health,
                                0, 0, -1, -1, 0, 0, 0, 0, 0, 0);
    gcs_send(&msg);
}

static void gcs_send_heartbeat(const struct gins_solution *sol)
{
    mavlink_message_t msg;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t sys_state = MAV_STATE_STANDBY;

    RT_UNUSED(sol);

    /* armed 态如实反映 quad_model 状态机 (QGC 解锁按钮/模式栏回显) */
    if (quad_model_get_status() == QUAD_MODEL_ARM)
    {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        sys_state = MAV_STATE_ACTIVE;
    }

    mavlink_msg_heartbeat_pack(mavlink_link_sysid(), mavlink_link_compid(),
                               &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                               base_mode, GCS_CUSTOM_MODE, sys_state);
    gcs_send(&msg);
}

/* STATUSTEXT 事件: gins 关键状态边沿 (初拍只采样; 文本 <= 50 字节) */
static void gcs_send_statustext(uint8_t severity, const char *text)
{
    mavlink_message_t msg;

    mavlink_msg_statustext_pack(mavlink_link_sysid(), mavlink_link_compid(),
                                &msg, severity, text, 0, 0);
    gcs_send(&msg);
}

static void gcs_check_status_edges(const struct gins_solution *sol)
{
    rt_int8_t ready = sol->ready ? 1 : 0;
    rt_int8_t degraded = sol->degraded ? 1 : 0;
    rt_int8_t nognss = sol->nognss ? 1 : 0;

    if (ctx.prev_ready >= 0)
    {
        if (ready != ctx.prev_ready)
            gcs_send_statustext(ready ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                                ready ? "GINS ready" : "GINS solution lost");
        if (degraded != ctx.prev_degraded && degraded)
            gcs_send_statustext(MAV_SEVERITY_ERROR, "GINS degraded (stale output)");
        if (nognss != ctx.prev_nognss)
            gcs_send_statustext(MAV_SEVERITY_NOTICE,
                                nognss ? "NOGNSS mode entered" : "GNSS recovered");
        if (sol->nan_cnt != ctx.prev_nan_cnt)
            gcs_send_statustext(MAV_SEVERITY_CRITICAL, "GINS non-finite solution!");
    }
    ctx.prev_ready = ready;
    ctx.prev_degraded = degraded;
    ctx.prev_nognss = nognss;
    ctx.prev_nan_cnt = sol->nan_cnt;
}

/* ---------------------------- 输出线程 ---------------------------- */

static rt_err_t gcs_rx_ind(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);
    if (ctx.rx_sem != RT_NULL)
        rt_sem_release(ctx.rx_sem);
    return RT_EOK;
}

static void gcs_thread_entry(void *parameter)
{
    static rt_uint8_t rxb[128];

    RT_UNUSED(parameter);

    while (1)
    {
        rt_size_t n;

        rt_sem_take(ctx.rx_sem,
                    rt_tick_from_millisecond(ctx.on ? GCS_POLL_TICKS_MS
                                                    : GCS_IDLE_TICKS_MS));

        if (!ctx.on)
            continue;

        /* 收向: 排空 uart2 接收缓冲 (电台链路本模块独占) */
        while ((n = rt_device_read(ctx.uart, 0, rxb, sizeof(rxb))) > 0)
            mavlink_link_feed(rxb, n);

        /* 发向: 周期节拍 (tick 回绕安全: 无符号减法) */
        {
            struct gins_solution sol;
            rt_tick_t now = rt_tick_get();

            gins_bridge_get_solution(&sol);

            if (now - ctx.tick_hb >= rt_tick_from_millisecond(GCS_PERIOD_HB_MS))
            {
                ctx.tick_hb = now;
                gcs_send_heartbeat(&sol);
                gcs_send_sys_status(&sol);
                gcs_check_status_edges(&sol);
            }
            if (now - ctx.tick_att >= rt_tick_from_millisecond(GCS_PERIOD_ATT_MS))
            {
                ctx.tick_att = now;
                gcs_send_attitude(&sol);
            }
            if (now - ctx.tick_nav >= rt_tick_from_millisecond(GCS_PERIOD_NAV_MS))
            {
                ctx.tick_nav = now;
                gcs_send_local_position(&sol);
                gcs_send_global_position(&sol);
                gcs_send_vfr_hud(&sol);
                gcs_send_gps_raw();
                gcs_send_rc_channels();
            }
        }
    }
}

/* ---------------------------- 会话启停 ---------------------------- */

static rt_err_t gcs_session_start(void)
{
    if (ctx.rx_taken)
        return RT_EOK;

    /* 接管 uart2 RX: 保存原 rx_indicate (电台口无 finsh, 通常为 NULL,
     * `gcs off` 恢复; 接口与阶段 0 console 共口时完全一致) */
    ctx.saved_rx_ind = ctx.uart->rx_indicate;
    if (rt_device_set_rx_indicate(ctx.uart, gcs_rx_ind) != RT_EOK)
    {
        ctx.saved_rx_ind = RT_NULL;
        return -RT_ERROR;
    }
    ctx.rx_taken = RT_TRUE;

    ctx.tick_hb = ctx.tick_att = ctx.tick_nav = rt_tick_get() -
                   rt_tick_from_millisecond(1000);    /* 立即发首拍 */
    ctx.on = RT_TRUE;
    rt_sem_release(ctx.rx_sem);                       /* 踢醒慢轮询 */

    return RT_EOK;
}

static void gcs_session_stop(void)
{
    ctx.on = RT_FALSE;

    if (ctx.rx_taken)
    {
        rt_device_set_rx_indicate(ctx.uart, ctx.saved_rx_ind);
        ctx.saved_rx_ind = RT_NULL;
        ctx.rx_taken = RT_FALSE;
    }
}

/* ---------------------------- 初始化 ---------------------------- */

int mavgcs_link_init(void)
{
    if (ctx.thread != RT_NULL)
        return RT_EOK;

    ctx.uart = rt_device_find(GCS_UART_DEV);
    if (ctx.uart == RT_NULL)
    {
        LOG_E("gcs: uart \"%s\" not found", GCS_UART_DEV);
        return -RT_ERROR;
    }
    if (!(ctx.uart->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        if (rt_device_open(ctx.uart, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            LOG_E("gcs: open uart \"%s\" failed", GCS_UART_DEV);
            ctx.uart = RT_NULL;
            return -RT_ERROR;
        }
        /* 电台波特率显式下发 (驱动注册默认 115200, 电台改参数时同步宏) */
        {
            struct serial_configure cfg;
            if (rt_device_control(ctx.uart, RT_SERIAL_CTRL_GET_CONFIG, &cfg) == RT_EOK)
            {
                cfg.baud_rate = GCS_UART_BAUD;
                (void)rt_device_control(ctx.uart, RT_DEVICE_CTRL_CONFIG, &cfg);
            }
        }
    }

    ctx.rx_sem = rt_sem_create("gcsrx", 0, RT_IPC_FLAG_FIFO);
    if (ctx.rx_sem == RT_NULL)
        return -RT_ENOMEM;

    /* RTCM 回注口: UART4 由 gnss_data 打开 (RDWR), 此处只取句柄写字节 */
    ctx.rtcm_uart = rt_device_find(GCS_RTCM_DEV);
    ctx.rtcm_on = RT_TRUE;
    if (ctx.rtcm_uart == RT_NULL)
        LOG_W("rtcm fwd: \"%s\" not found (UM982 链路未启动?)", GCS_RTCM_DEV);

    mavlink_link_set_sender(gcs_tx);          /* 发送通道注入 */
    mavlink_link_attach(gcs_on_msg, 0);       /* 通配: 按 msgid 分派 */

    ctx.thread = rt_thread_create("mavgcs", gcs_thread_entry, RT_NULL,
                                   GCS_THREAD_STACK, GCS_THREAD_PRIO,
                                   GCS_THREAD_TICK);
    if (ctx.thread == RT_NULL)
        return -RT_ENOMEM;
    rt_thread_startup(ctx.thread);

    LOG_I("gcs: QGC link on %s @%s 8N1 (sysid %u, default off, `gcs on` to start)",
          GCS_UART_DEV, GCS_UART_BAUD_TEXT, mavlink_link_sysid());
    return RT_EOK;
}

/* ---------------------------- FinSH 命令 ---------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>

static void gcs(int argc, char **argv)
{
    struct mavlink_link_stats st;

    if (argc >= 2)
    {
        if (!rt_strcmp(argv[1], "on"))
        {
            rt_err_t err = gcs_session_start();
            if (err != RT_EOK)
            {
                LOG_E("gcs on failed: %d", (int)err);
                return;
            }
            LOG_I("gcs on: uart2 (数传电台) RX taken over "
                  "until `gcs off`");
            return;
        }
        if (!rt_strcmp(argv[1], "off"))
        {
            gcs_session_stop();
            LOG_I("gcs off: uart2 RX restored");
            return;
        }
        if (!rt_strcmp(argv[1], "rtcm") && argc >= 3)
        {
            ctx.rtcm_on = !rt_strcmp(argv[2], "on") ? RT_TRUE : RT_FALSE;
            LOG_I("rtcm forward %s -> %s (UM982 回注)",
                  ctx.rtcm_on ? "on" : "off", GCS_RTCM_DEV);
            return;
        }
    }

    mavlink_link_get_stats(&st);

    LOG_I("=== QGC GCS link (FMT_README ch13, 阶段1 数传电台) ===");
    LOG_I("session : %s, uart %s @ %s 8N1, rx_taken=%d",
          ctx.on ? "ON" : "off", GCS_UART_DEV, GCS_UART_BAUD_TEXT,
          (int)ctx.rx_taken);
    LOG_I("tx      : %u frames this boot", ctx.tx_cnt);
    LOG_I("rx      : bytes=%u pkts=%u crc_err=%u handled=%u lastmsgid=%u",
          st.rx_bytes, st.rx_packets, st.rx_crc_err, st.rx_handled,
          st.last_msgid);
    LOG_I("gcs hb  : %s", (ctx.last_gcs_hb == 0) ? "never" :
          ((rt_tick_get() - ctx.last_gcs_hb <
            rt_tick_from_millisecond(5000)) ? "recent (<5s)" : "stale"));
    LOG_I("origin  : %s", ctx.origin_valid ? "captured" : "waiting first ready fix");
    LOG_I("uplink  : arm/disarm (400) + reposition (192, 钳位 h%d/v%d m) "
          "+ set_mode (POSCTL) + virtual stick + rtcm", (int)GCS_REPO_HORIZ_MAX_M,
          (int)GCS_REPO_VERT_MAX_M);
    LOG_I("rtcm    : %s, %u bytes / %u pkts fwd -> %s (drop %u)",
          ctx.rtcm_on ? "on" : "off", ctx.rtcm_bytes, ctx.rtcm_pkts,
          GCS_RTCM_DEV, ctx.rtcm_drop);
    LOG_I("hint    : QGC serial link -> 数传电台 @ %s, MAVLink2", GCS_UART_BAUD_TEXT);
}
MSH_CMD_EXPORT(gcs, QGC GCS link: gcs [on|off]);

#endif /* RT_USING_FINSH && FINSH_USING_MSH */
