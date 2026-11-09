# MAVSDK 和地面 MAVLink 路由

Flight 的 MAVLink 系统和组件编号为 `1/1`。RC 使用 USART3，板端数传使用 USART2 的 115200 baud、8N1。板载 HIL 启用时，USART2 供仿真桥以 921600 baud 使用。

| 本机端口 | 用途 |
|---|---|
| UDP 14540 | Linux 路由或直连时由 MAVSDK 监听 |
| UDP 14550 | Linux 路由时由 QGroundControl 监听 |
| TCP 5760 | Windows MAVProxy 的 MAVSDK 端口 |
| TCP 5761 | Windows MAVProxy 的 QGroundControl 端口 |
| UDP 14560 | 路由器接收 Flight Host SIL 地面站流 |
| UDP 14653 | Flight Host 与 FMT-SIM 的动态仿真链路 |

## 连接路由器

Linux 数传使用 [mavlink-router.conf](../../config/mavlink-router.conf)。按实际设备填写 `Device`，然后在项目根目录运行。SIL 使用单独的配置。

```sh
mavlink-routerd -c config/mavlink-router.conf
# SIL 场景改用
mavlink-routerd -c config/mavlink-router-sil.conf
```

Windows 可用 MAVProxy。以下两条命令分别用于数传和 SIL。数传应替换为实际 COM 号。QGC 添加指向 `127.0.0.1:5761` 的 TCP 链路，MAVSDK 连接 5760。使用 TCP 输出可避免 Windows 上 MAVProxy 的 UDP 输出占用客户端的监听端口。

```powershell
mavproxy.exe "--master=COM8,115200" --out=tcpin:127.0.0.1:5761 --out=tcpin:127.0.0.1:5760 --cmd="set mavfwd True"
mavproxy.exe --master=udpin:127.0.0.1:14560 --out=tcpin:127.0.0.1:5761 --out=tcpin:127.0.0.1:5760 --cmd="set mavfwd True"
```

## MAVSDK 动作

动态 SIL 使用 [FMT-SIM 场景](HIL_LINK.md) 启动。路由器监听 14560，启动脚本的 `-GroundPort` 也设为 14560。单独使用 MAVSDK 时，可将 `-GroundPort` 设为 14540 直连。待导航对齐和预解锁检查完成后，在独立场景执行以下任一组动作。

```powershell
# Linux UDP 或直接连接 SIL
uv run tools/flight_mavsdk.py status arm takeoff land disarm --altitude 2
# Windows MAVProxy 的 TCP 链路
uv run tools/flight_mavsdk.py status arm takeoff rtl disarm --altitude 2 --address tcpout://127.0.0.1:5760
```

命令由 MAVSDK Action 发送。动态 SIL 将解锁、上锁、起飞、降落和返航转换成 FMS 请求，按 FMS 进度或结果应答。起飞高度使用相对 Home 的米值。H723 台架可用 `status` 检查连接，并支持版本、公开参数和扩展系统状态查询。收到飞行动作时，台架固件返回带策略原因的 `MAV_RESULT_DENIED`。板端实飞动作依赖签名会话与通过验收的输出策略。

[MAVSDK Action 文档](https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html) · [MAVProxy 路由选项](https://ardupilot.org/mavproxy/docs/getting_started/starting.html) · [mavlink-router 配置参考](https://github.com/mavlink-router/mavlink-router/blob/master/examples/config.sample)
