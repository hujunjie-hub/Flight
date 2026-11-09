# Flight SIL 与 QGroundControl 交付验收

验收环境为 Windows、QGroundControl 5.1.5 和 FMT-SIM 的 Ground/HIL 场景。地面站通过本机回环连接 Flight SIL。任务和飞行动作进入生产 FMS、制导、控制和执行器守卫，执行器守卫的虚拟提交驱动 FMT-SIM。

## 使用

在 Flight 工程中按 [联机说明](HIL_LINK.md#qgroundcontrol-联机)构建和启动。使用交付包时，在 FMT-SIM Ground/HIL 的飞控端口配置为 `127.0.0.1:14653` 并启动仿真器后，从解压目录执行：

```powershell
.\sil_flight.exe 14653 0.62 Ground 1000 14550
```

静态台架可执行 `.\sil_flight.exe 0 0.62 Ground 350 14550`，可验证遥测、参数和日志。将 `Flight SIL Actions.json` 放在 QGroundControl 保存目录的 `MavlinkActions`，在 **Settings → Fly View → MAVLink Actions → Fly View Actions** 选中。用 **Plan → Open** 打开 `.plan` 文件。切换 SIL 实例后若界面仍显示旧载具的 `Comms Lost`，点击 **Disconnect** 让 QGroundControl 重新发现本机动态载具。

## 逐项结果

| QGroundControl 入口 | Flight 行为 | 验收方式与结果 |
|---|---|---|
| Fly 地图、姿态、模式、接地状态 | 位置、姿态、速度、Home、模式和接地状态从 SIL 上报 | 界面联机，显示位置、起飞、飞行、降落和上锁 |
| Actions → SIL 记录 Home | 用当前导航位置设置 Home，FMS 检查上锁状态 | MAVLink 实测成功 |
| Actions → SIL 解锁 | FMS 检查当前预解锁结果 | 界面实测成功 |
| Actions → SIL 起飞至 2 m | 起飞并建立位置保持 | 界面实测成功 |
| Actions → SIL 开始任务 | 仅航点计划从悬停进入任务。含起飞项的完整计划可从地面启动 | 界面实测了仅航点启动及完整返航计划。点击后飞完两航点，返航着陆并上锁，Fly 恢复 `Ready / PreFlight` |
| Actions → SIL 暂停任务、恢复任务 | 暂停时撤销执行令牌并悬停，恢复时核验坐标系和授权 | 动态协议实测成功 |
| Actions → SIL 终止任务 | FMS 终止任务并切回悬停 | 动态协议实测成功 |
| Actions → SIL 返航 | 回到 Home 并接地 | 动态协议实测成功 |
| Actions → SIL 原地降落、SIL 上锁 | 当前位置降落，接地后上锁 | 界面实测成功 |
| Plan → Open、Upload | 打开双航点或四项自动返航文件，向 FMS 上传并提交 | 两种文件均在界面上传，收到 FMS 提交回执 |
| Plan → 从载具下载、清空 | 按原序号回读；拒绝无效替换；经 FMS 清空 | MAVLink 实测成功 |
| Plan → GeoFence | 上锁时上传、回读和清空多边形或圆形。FMS 校验几何、任务航点和制导轨迹 | 原版界面上传和回读了一个包含多边形。协议用例验证越界航点拒绝及非法替换保留旧围栏 |
| Plan → Rally Points | 上锁时上传、回读、清空最多 32 个备降点。仿真进程保存坐标和相对高度 | 原版界面回读 1 个备降点，协议验证独立清空。自动选点和备降飞行待实现 |
| Plan → 自动降落、自动返航计划 | 可选首项起飞，最多 32 个航点，可选末项降落或返航。FMS 确认落地后请求上锁 | 两种完整计划从地面到上锁均通过动态协议实测 |
| Analyze → Onboard Logs → Refresh、选择、Download | 下载稳定快照，支持分段读取 | 界面显示 `Downloaded`。落盘 53,197 字节，独立解码 1,234 帧 |
| Analyze → Onboard Logs → Cancel、Erase All | 结束下载；上锁时可清空本实例的内存日志 | 在隔离静态台架按协议验证 |
| Analyze → Log Viewer → Open .tlog、Pause | 通用日志下载扩展名为 `.bin`。副本使用 `.tlog` 打开，回放前断开实时连接 | 界面载入 1 分 23 秒的日志，显示实时解析的 MAVLink 消息。回放至第 5 秒并暂停 |
| Configure → 参数 | 73 个控制参数可读，`SIM_TLM_HZ` 可写且限制为 1–10 Hz | 参数列表、写入、越界拒绝、核心参数写入拒绝均通过协议验收 |

## 仍未交付的入口

| 入口 | 当前状态 |
|---|---|
| 原版 Fly 内置 Guided 按钮、Generic 载具专用 Configure 页面 | 已提供原版开始、继续、起飞、返航、降落和暂停的 v5.1.5 源码补丁。QGroundControl 定制版构建、原版按钮界面验收和专用 Configure 页面仍待完成 |
| 复杂计划项、实机围栏写入、备降飞行 | 仿真围栏与备降点独立交换。实机围栏仍由产品 Profile 在 FMS 初始化时安装。备降点还未接入自动返航选点 |
| 摇杆接管、传感器校准、电机测试、地面站刷机 | 当前发布版保持权限限制。板级固件已编译，可用于受控 SWD 台架刷写，实机校准和电机测试还需独立台架证据 |
| 视频、地理标记、MAVLink 控制台 | 当前 Flight 仿真链路没有这些后端 |
| H723 串口联调、实物执行器、实机远程授权 | 本包为 Host SIL，实机验收待完成 |

## H723 台架准备

本轮已生成 STM32H723 的 `Flight.elf` 和 `Flight.bin`。链接器报告了已有的混合枚举 ABI 警告。STM32CubeProgrammer 的 ST-LINK 与 DFU 枚举均未发现设备，Windows 也未枚举出串口。

H723 使用项目的 DIRECT_BSP 启动布局。QGroundControl Firmware 页的下载和刷写仍待 Flight 专用后端。板级 SWD 流程使用本包的 ELF，连接时按 ST-LINK 序列号选择目标并确认目标型号为 STM32H723，然后执行编程与校验。工具的设备枚举和编程选项见 [STM32CubeProgrammer 命令说明](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html)。

板端启动后，接 USART2、115200 8N1。在 Flight 工程根目录运行 `python tests/simulation/verify_h723_serial.py --list` 查看串口，再用 `--port COM7` 换成实际端口测试。脚本验证 MAVLink 2 版本、上锁心跳、系统状态和三个板端只读参数。它需要 `pyserial` 和工程已生成的 `fcu.py`。同一串口由测试脚本和地面站分别独占。测试完成后在 QGroundControl → Comm Links 添加 115200 的 Serial 链路。

## 验收证据

交付包包含动态自动降落和返航、暂停恢复、终止降落的文本记录，以及通过 QGroundControl 实际下载的可回放日志副本。各记录对应的运行参数和复测命令见 [联机说明](HIL_LINK.md#qgroundcontrol-联机)。

同一个仿真实例在返航上锁后再次起飞时，曾因控制器沿用上一架次的电机推力变化率基准而进入保护。控制器现在收到 FMS 的空转许可时结束上一架次的推力记录。修复后，同一进程完成两架次从起飞到返航上锁的动态协议验收。第二架次等 FMS 重新确认接地后启动。主机算法共 2,358 项检查通过，Cortex-M7 编译和链接检查通过。
