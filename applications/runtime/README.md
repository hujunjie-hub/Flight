# Flight 固件运行集成

QUAD_X4，Schema 6.0 / Port 0x00010906。

本轮按用户授权完成固件侧运行接入，采用“暂用现有参数，缺失项禁止解锁”。`applications/main.c` 调用 `runtime_start()`，不再只有空循环；`.config` / `rtconfig.h` 已启用运行期及M2～M6依赖。M7固定BENCH_INHIBIT，能力中的物理可用电机mask为0，既不自动解锁，也不向DShot提交占空比或油门帧。Host的SIM策略只在`FLIGHT_HOST_TEST`下编译，H723拒绝该策略。

各任务在 `applications/tasks/` 的对应子目录实现周期工作和私有工作区。`runtime.c` 管理台架配置装配和模块启停，`runtime_data.c` 管理共享邮箱、快照、事件和心跳。地面站由 `applications/tasks/communication/task_gcs.c` 从应用快照发布状态。M1 提供串口与 MAVLink 封帧，M2 提供传感器接入和标准测量。目录映射见 [工程目录](../../PROJECT_STRUCTURE.md)。

## 构建和观察

在Flight根目录执行（当前环境已有SCons）：

```powershell
build/m4_m5_tools/Scripts/python.exe -m SCons -Q -j8
```

默认GNU Arm release / `-O2`，通过PATH查找工具链，当前工作站回退到`D:/STM32CubeCLT/GNU-tools-for-STM32/bin`。可用`RTT_EXEC_PATH`覆盖。输出为根目录`Flight.elf`、`Flight.bin`、`Flight.map`。未烧录板卡。

UART1 控制台可用 `flight_status` 查看调度、对准和拒绝计数，用 `link_status` 查看 GNSS 和串口计数，用 `sensors` 查看慢传感器读数及时间拒绝，用 `driver_status` 查看底层设备，用 `list_thread` 查看目标栈水位。

## 运行所有权与交接

| Owner | 触发/频率 | 工作与交接 |
|---|---|---|
| ins，优先级7，8 KiB栈 | IMU 发布事件或 1 ms 超时 | 消费 M2 标准 IMU 样本；M3 传播与分片 shadow；发布导航快照；调用 M6 快环 |
| mission，12，8 KiB | 20 ms超时或首个GNSS候选事件 | 串行M4→立即发布Permit→M5；下一周期消费READY/feasibility；发布完整决策/参考bundle |
| mpc，13，8 KiB | 20 ms | 从独立复制的完整bundle调用MPC；不占用快环，不借用变化中的Port指针 |
| actuator，6，2 KiB | 1 ms | 独立核对Permit/Demand时效及驱动故障；发布Status/Capability/Receipt快照；BENCH无物理帧 |
| io，18，4 KiB | 4 ms | 有界UART接收、UM982原生解码、遥测TX队列、慢传感器诊断 |
| supervisor，优先级20 | 100 ms | 检查五个owner心跳；全部正常时启用/喂IWDG；500 ms心跳超期停止喂狗并抑制输出 |

跨任务 mutex 只覆盖固定内存复制。算法、求解器、I2C 和 UART 操作在锁外。快环使用 try-lock，每次刷新导航和执行器快照，规划 bundle 换版时才复制其中的制导、许可和控制模型字段。锁繁忙时沿用上一份数据，仍检查原 TTL。IMU 和规划通知分别最多积压一次唤醒，样本和请求仍保存在各自的缓冲中。监督任务读取各 owner 的心跳后再取评估时刻，避免抢占导致新心跳被判为来自未来。生命周期初始化先于线程启动。

M3运行期使用400 us/5 ms记账及最多16个数值操作的分片cursor；每片只推进一个IMU传播或一个量测更新，可续接新IMU边界。journal变化撤销重建，过期版本/错误边界不得提交。**单个不可分数值操作的目标WCET尚未测量，因此这不是400 us硬上限或RTA通过证据。** 同步路径保留给Host参考测试。

M7具有Permit撤销watermark、时钟/来源/布局/全套epoch绑定、序号单调性、TTL和控制心跳守卫、物理限幅/变化率保护、单调标定曲线映射、DShot编码及命令级Receipt。`actuator_apply`只接收最新Demand，不能生成COMMITTED；只有`actuator_guard_frame_boundary`收到实际提交成功才能生成Receipt。该提交路径用Host驱动验证。当前H723没有注册实机飞行失效策略或推力标定，物理路径持续禁止；不能把1 ms软件唤醒伪装为400 Hz实际帧边界。所有RPM/实测推力反馈有效位保持0。

## 器件和通信

* ADIS16505 的 SPI DMA 环由驱动管理，应用通过原生设备读取 burst；使用驱动DRDY事件的32位序号，避免器件16位DATA_CNTR回绕破坏导航。安装矩阵、group delay和噪声仍为明确的临时台架配置。
* UM982 UART4和PPS固定运行。新增原生`BESTNAVA`/`RECTIMEA`解析，校验CRC32、GPS/FINE、完整字段及独立位置/速度质量；MSL加undulation得到椭球高，速度转NED。非零速度latency不冒用位置时刻，拒绝该速度字段。启动只请求易失日志输出，不保存Flash或重启接收机。
* RECTIME只接受有效UTC及可明确配对的PPS；GPS周/周内时间借已验证的UTC映射转换。未映射、过期、接收机精度不足、超过100 us的不确定度均拒绝融合；无Fix/PPS/UTC不阻止本地IMU对准。CRC golden来自Unicore N4 R1.13官方例帧，尚无本机接收机配置回读/实采证明。
* BMP585按实际寄存器顺序将温度/压力交换到M2约定；保留驱动当前“轮询接收时刻，10 ms不确定度”的事实，只报告诊断，不将其假充物理采样中心用于融合。BMM350读取官方OTP补偿后的微特斯拉，仅作传感器坐标系诊断，MAG aiding保持关闭。
* UART2发送HEARTBEAT及全部13类FCU消息：主状态10 Hz，传感器/控制/RC/链路/资源/存储/构建信息轮转，事件有界发送。字段偏移/CRC_EXTRA由固定pymavlink从`spec/contracts/FCU.xml`生成；不把内部C结构直接当网络载荷。TX固定队列保存DMA期间的缓冲区，UART拥塞不阻塞控制。ABI消息发送完整ARM ABI SHA256；未有Git提交，commit_valid保持0。尚未测量的WCET/栈/CPU/AXI与SD/NOR/free字段均不设置valid位。
* 依 06 规范，当前产品为 `REMOTE_MUTATION=DISABLED`。板载串口仅公开 `FLT_PROFILE`、`FLT_CONTRACT` 和 `FLT_BENCH` 三项只读参数，对已知参数的写入请求返回当前值。会话调参由独立的 Host SIL 地面站适配器提供。ELRS 输出帧格式和通道映射尚未确认，当前用于接收和诊断。可信适配器的有界 typed request、operator 和 mission 入口已接到 M4。

## 修复的对接问题

M2样本原来`valid_until_us=0`，进入M3必被判过期；现从物理样本时刻给出TTL。M4原生时钟决策原来使用INVALID时间质量，现与M6校验一致。M6已接纳M5的自动垂向着陆kind=5，并限制到明确LAND/SAFE_ACTION+ALT_HOLD，不要求虚构水平位置。ControlModel ID与规范allow-list对齐为0x1501。Profile补齐各实现ID，静止对准的NOT_READY不再误计为坏IMU。

源级隔离由Sensor Integrity管理，导航失效由M3公开能力表达。被隔离的辅助源可在同一预算中执行独立shadow probation：从历史先验重建，计算真实NIS，不写active状态或P，不进入active journal，shadow-only候选在commit层再次被拒绝。Health仅消费匹配boot/time/state/policy/calibration的、新鲜且未重复的公开证据；映射换代/降级会撤销待计算probe或使证据不可用。满足数量与2 s连续观察期后先进入DEGRADED，再经过普通融合证据与2 s观察期恢复NORMAL。仅接收/解码成功不能恢复隔离源；硬器件故障需要明确RECOVERED前提；单IMU没有伪造的独立恢复证据。

晚到的首个可信GNSS位置先由M3暂存，绝不直接融合到无WGS84锚的本地系。M4正常update后，仅在DISARMED且无有效Permit时按候选时刻授权ORIGIN_REBUILD；导航owner复核候选、时钟、映射和原点ID后执行，递增state/origin epoch并重新静止对准。首个候选事件唤醒规划owner，避免固定20 ms相位始终错过量测TTL。后续新快照使旧参考/Permit失效。

32项固定RAM事件FIFO记录启动门禁、源健康/隔离/恢复、原点重建、FMS权限/状态变化与owner心跳超时；IRQ临界区只保护短复制，UART仅在队列有余量时取走。溢出保留已有事件并累计drop，UART拥塞不影响控制。**该队列不持久化，不满足STORAGE_EVENT_LOG解锁条件；SD/NOR掉电与存储格式验证仍待完成。** ModuleHealth错误字段按Status的int32补码保存，通用遥测reason仍使用正值注册表。

## 验证与边界

集成验证清单及产物 Hash 由 `tests/runtime/verify_firmware.py` 写入 `build/firmware_integration/manifest.json`。验证入口如下。

```powershell
python tests/runtime/run_tests.py
uv run --no-project --with pymavlink==2.4.49 python tests/runtime/check_wire.py
wsl.exe -d Ubuntu-26.04 -- python3 tests/runtime/run_tests.py --sanitize
python tests/algorithms/run_tests.py
pwsh -NoProfile -File tests/navigation/run_host_tests.ps1 -OutDir build/firmware_integration/replay
python tests/abi/abi_report.py
```

既有验证覆盖确定性 Host 调度、传感器隔离与恢复、原点授权、事件队列、M7 模拟提交、FCU 线协议、算法回归和 ABI。验证脚本核对证据对应的源码 Hash。

未执行：板上启动与栈水位、真实burst/PVT/波形、GPIO/器件采样延迟标定、Cache压力、WCET/RTA、SD/NOR掉电/压力、HIL/飞行及非作者审核。现有GNU Arm/newlib链接仍报告enum-size属性警告，保留完整日志；当前 ABI 证据见 `tests/abi/evidence/abi_compare.json`。

用户缺少的真实数据仍明确保留：质量/惯量/几何/旋向与推力标定、传感器安装/标定/group delay、ELRS实际协议和通道、电流计型号/能源证据、实机输出失效策略。软件能够构建并进入台架调度，不表示已经具备飞行条件。没有修改MATLAB目录，也没有把旧任务台账或独立审核状态自动改为完成。

协议来源：[Unicore N4 R1.13](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.13.pdf)，[MAVLink序列化](https://mavlink.io/en/guide/serialization.html)。
