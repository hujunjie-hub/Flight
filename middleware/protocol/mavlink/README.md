# mavlink — MAVLink v2 链路（地面站遥测/指令）

本目录 = **MAVLink v2.0 官方 C 库**（common 方言，头文件）+ **RT-Thread
适配层**（`mavlink_link.c/h`）。定位与 `../nmea` 一致：纯协议层，不开线程、
不占串口。

```
地面站 ──串口/数传──> 驱动接收线程 ──> mavlink_link_feed()   (帧同步/CRC/v1 兼容)
                                              │
                                              ▼ mavlink_msg_handler_t 回调
                                                (mavlink_link_attach, 按 msgid 过滤, 0=通配)

地面站 <──驱动写── sender(mavlink_link_set_sender 注入) <── mavlink_link_send_msg()
                       (mavlink_msg_xxx_pack 组包 + 发序号/序列化)
```

## 官方库（库头文件部分）

pymavlink 生成的头文件库，原样取自本仓库 `ref/FMT-Firmware` 的
`src/lib/mavlink/v2.0/`（common.xml 生成件，MAVLINK_BUILD_DATE
2020-08-18；历史来源为 doc/IMUTest 时代的 FMT 拷贝, 参考工程现已迁至
`ref/`），仅去掉 FMT 专用方言 `fmt/` 目录。**不要手工修改**；
需要其它方言（ardupilotmega 等）或新版报文时用 pymavlink 重新生成后
整目录替换（保留 `common/` 与本目录的适配层 .c/.h）：

```bash
pip install pymavlink
python -m pymavlink.tools.mavgen --lang C --wire-protocol 2.0 \
    -o <dialect> common.xml     # 生成头文件, 替换本目录对应部分
```

集成方式：`mavlink_link.h` 在包含 `common/mavlink.h` 之前定义
`MAVLINK_SEPARATE_HELPERS` 与 `MAVLINK_COMM_NUM_BUFFERS=1`——helper
函数（帧解析 `mavlink_parse_char`、序列化 `mavlink_msg_to_send_buffer`、
CRC 表查找等）与通道收发缓冲只在 `mavlink_helpers.c` 一处定义，
全固件约 0.8 KB RAM（1 通道）。

## 使用

```c
#include "mavlink_link.h"        /* 不要直接包含库头 (绕过单点编译配置) */

/* 收: 串口接收线程 */
mavlink_link_feed(rxbuf, n);

/* 收: 订阅报文 (喂入线程上下文回调, 快进快出) */
static void on_cmd(const mavlink_message_t *msg)
{
    mavlink_command_long_t cmd;
    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        mavlink_msg_command_long_decode(msg, &cmd);
}
mavlink_link_attach(on_cmd, MAVLINK_MSG_ID_COMMAND_LONG);

/* 发: 注入写函数后组包发送 */
static void uart_send(const rt_uint8_t *buf, rt_size_t len)
{   rt_device_write(dev, 0, buf, len);   }
mavlink_link_set_sender(uart_send);
mavlink_link_send_heartbeat();           /* 1Hz 起步, 地面站靠它识别本机 */

mavlink_message_t msg;
mavlink_msg_attitude_pack(mavlink_link_sysid(), mavlink_link_compid(), &msg,
                          t, roll, pitch, yaw, 0, 0, 0);
mavlink_link_send_msg(&msg);
```

注意：

- **必须经 `mavlink_link.h` 间接包含库头**，直接包含 `common/mavlink.h`
  会每个 .c 复制一份通道缓冲（约 0.8 KB/份）。
- feed 只能单线程喂（解析状态机无锁）；pack+send 链路多线程并发需调用
  侧串行（发序号递增无锁）。
- v1 帧兼容：`mavlink_link_set_out_v1(RT_TRUE)` 切发 v1（收方向 v1/v2
  自动识别）。

## 硬件（待绑定）

板级当前只启用 USART1（console+VOFA）与 UART4（UM982，见 `../nmea`），
MAVLink 尚未绑定物理串口。绑定后在串口接收线程 `feed`、写函数注入
`set_sender` 即可，协议层无需改动。本机标识 `MAVLINK_LINK_SYS_ID=1` /
`COMP_ID=AUTOPILOT1`，心跳内容在 `mavlink_link.h` 配置。

## 构建与调试

- SCons：`mavlink/SConscript`（经 `middleware/protocol/SConscript` 汇入），
  Glob 顶层 `*.c`，含库告警抑制；CMake：根 `CMakeLists.txt` 的
  RT_USING_SENSOR 源列表 + include 路径。两边对含 mavlink 的编译单元都
  抑制了库生成代码固有的 `-Waddress-of-packed-member` 告警。
- FinSH：`mavlink` 看统计（rx/tx/丢帧/最近报文），`mavlink sendhb` 发
  一帧心跳，`mavlink v1 on|off` 切帧格式。
- 验证：`mavlink sendhb` + 统计命令做环回冒烟；地面站链路接入（物理
  串口绑定）后按 `../nmea` 同款方式回归。（utest 框架已于 2026-09-30
  移除, 原 `middleware.protocol.mavlink_link` 用例下线。）
