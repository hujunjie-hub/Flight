# middleware/Sensor_Preprocessing/filter_calib — 磁力计与气压计校准

对 `middleware/Sensor_Drivers` 驱动读出的 BMM350 磁力计 / BMP585 气压计数据做校准,
校准后的数据送入 KF-GINS 融合 (gins 桥接的喂引擎路径上生效)。校准参数
持久化在 **W25Q64 calib 分区** (`middleware/Sensor_Preprocessing/param_calib`, 2026-10-02 从
片内 Flash 扇区 7 迁移; 旧记录首次上电自动导入一次, 存储布局/记录纪律
见 `../param_calib/README.md`, 本 README 只写校准本身)。

```
数据链 (磁):  mag_bmm350 驱动 (mGauss) -> mag_data 采集线程 (100Hz):
              µT 单位换算 -> 量程守卫 (|body|<1e6µT, 拒 NaN/inf/huge,
              防止 EMA 低通闩锁) -> mag_calib_apply (传感器框架椭球校正;
              调用侧须显式 float→double 转换, (const double*)raw 强转会
              读越界产出 ~1e10 垃圾 —— 2026-09-29 修复, 曾使磁链路全程报废)
              -> 轴映射到体系前右下 (软磁矩阵与轴重排不可交换, 顺序不能调)
              -> 一阶 EMA 低通 (GINS_MAG_LPF_TAU_S, 0 旁路) -> 环形缓冲区
              [magcal 采集] 从同环取 raw 字段椭球拟合 -> 参数 (W25Q64 calib 分区)
              生效路径 (gins 桥接 ginsaux 线程): 环内 cal 字段 (校准+轴映射+
              低通后, 体系 FRD) -> KF-GINS 磁航向观测 (观测时标用样本自带
              µs 采样时刻, 并前移低通群延迟 τ)
数据链 (压):  baro_bmp585 驱动 (Pa) -> baro_data 环形缓冲区 (100Hz, Pa + 芯片温度)
              -> [barocal 采集] 基准偏移 -> 参数 (W25Q64 calib 分区)
              生效路径 (gins 桥接 ginsaux 线程): baro_data 环形缓冲区
              -> baro_calib_apply (+偏移) -> KF-GINS 气压高度观测
```

校准采集线程从 `middleware/Sensor_Preprocessing/process_data` 的环形缓冲区取数 (`mag_data_pop` /
`baro_data_pop`, 阻塞等待 + 逐个消费), 不再直接打开传感器设备; 采集
开始前 flush 掉积压旧样本, 连续 3s 无样本判数据链路中断并中止。
磁校准的拟合输入是环内 `mag_sample.mag[]` —— 传感器坐标系的原始值
(环内同时携带入环前已处理的 `cal[]` 供引擎消费), 因此重新校准不会
被已有参数污染。

注意: 软磁矩阵与轴重排不可交换, 因此椭球校准必须在传感器 (芯片) 框架内
完成后再做轴映射, 顺序不能调 (该顺序由 mag_data 采集线程固定保证)。

## 文件

| 文件 | 说明 |
| --- | --- |
| `ellipsoid_fit.c/h` | 椭球代数拟合适算 (纯 C99, 无 RT-Thread 依赖, 可主机端测试) |
| `mag_calib.c/h` | 磁力计校准: 采集线程 + 拟合门限校验 + 应用 + `magcal` 命令 |
| `baro_calib.c/h` | 气压计校准: 基准偏移 + `barocal` 命令 |

参数的存取/持久化不在本目录: RAM 镜像 `struct calib_data` 与
`calib_store_init/save` API 见 `middleware/Sensor_Preprocessing/param_calib/param_calib.h`
(API 名沿用旧片内 Flash 时代的 calib_store_*, 调用方无需感知迁移)。

## 磁力计椭球校准用法

原理: 器件绕各姿态旋转时, 无误差样本应落在以地磁场模长为半径的球面上;
硬磁 (机体附加磁场) 使球心偏离原点, 软磁 (磁导率各向异性) 把球拉成椭球。
最小二乘拟合椭球得偏置 `b` 与校正矩阵 `S`, 校正输出 = `S (raw − b)`。
样本只增量累加进 9×9 法方程, 不保存原始数据, 内存开销固定 (~700 B)。

方法边界: 椭球法把椭球恢复成"球"。对称软磁 (轴间比例失配) 下方向可完
整恢复; 若畸变含旋转耦合 (非对称矩阵), 校正结果相对真值差一个固定未知
正交旋转 —— 该模糊度是纯椭球拟合的固有极限, 需外部基准 (加计/GNSS)
才能消除, 实际装机中该旋转分量通常很小。

操作 (FinSH, 整机装配完成后进行, 远离铁磁物):

```
magcal start          # 默认 45s, 期间手持整机缓慢画 8 字并翻转,
                      # 尽量让三个轴都朝天/朝地各转一圈
magcal show           # 查看偏置/软磁矩阵/拟合质量
magcal off            # 临时停用 (直通)
magcal clear          # 清除参数
```

采集结束自动拟合, 通过质量门限 (样本数、各轴覆盖、等效半径 15~110 µT、
主轴半径比 ≤ 3、偏置模长 ≤ 150 µT、代数残差 ≤ 0.08) 后写入 W25Q64
(calib 分区) 并立即生效; 不通过则保留旧参数并打印原因。
2026-10-02 本机已完成标定 (硬磁 ~25µT, 校正后场强与当地预期一致)。

## 气压计基准偏移校准用法

原理: 静置采集一段时间取气压均值, 与"当地准确气压"之差即安装偏移。

```
barocal ref 101325       # 方式一: 直接给参考气压 (Pa), 如气象站 QNH
barocal refalt 50.5      # 方式二: 给当地海拔 (m), 按 ISA 推参考气压
barocal start 30         # 静置采集 30s (默认), 自动算偏移并保存生效
barocal show | off | clear
```

门限: 采集窗口气压标准差 ≤ 12 Pa (否则判未静置), |偏移| ≤ 2500 Pa。

## 参数存储

- 位置: W25Q64 calib 分区 (16KB, 128×128B 追加式记录槽, 分区表与
  记录格式见 `../param_calib/README.md`); RAM 镜像为
  `struct calib_data` (param_calib.h)。
- 写入: 校准成功时自动 `calib_store_save()` 追加记录并读回校验
  (CRC32, 撕裂/坏记录上电扫描自动跳过)。OCTOSPI 间接模式访问,
  **擦写不关中断、不停四条数据链** —— 迁移前"擦写期间 CPU 停等 ~1s"
  的旧限制已不存在。
- 恢复出厂: FinSH `param erase calib` (param_calib 命令)。
- 历史: 2026-10-02 前参数存片内 Flash 扇区 7 (0x080E0000, 128B 记录,
  整扇区擦除 + flash word 编程), 迁移时旧记录一次性导入 W25Q64,
  导入与否由 sys 分区标志记住, 不会复活。

## 主机测试

`build_host/ell_test.exe` —— 椭球拟合数学的主机端单元测试 (编译本目录
`ellipsoid_fit.c`, 对称软磁/旋转耦合两类合成场景分别验证方向余弦与
模长一致性)。磁链路端到端验证走板上 `magcal` 标定 + SWD 回归。

## 待实现: 加计六面标定 (规程草案, 2026-10-02 提出, 存储部分 2026-10-03
按 param_calib 重写)

背景: 本机加计 | ‖f‖-g | 随摆放姿态在 0.02~0.18 m/s² 变化 (±1.8%,
轴失准+比例因子量级), 是 ZUPT 加计门限被迫放宽到 0.30 与对准精度
~0.1-0.2° 误差的来源。原方案受"片内 Flash 记录仅剩 16 字节只够对角
比例因子"限制 —— 迁移 W25Q64 后该约束消失, 可按需存完整参数:

1. 采集: `acccal start [sec]` (待实现), 静止门控 (陀螺 w² EMA <
   4e-3 (rad/s)² 持续 0.3s) 下逐 FRD 体轴跟踪 running min/max;
   用户六面静置 (每轴朝上/朝下各 ~10s)。
2. 计算: scale_k = 2g/(max_k - min_k), 合理性界 |scale-1| < 3%;
   偏置不在此标 (C8 的 EKF 零偏快照已覆盖, 避免双源冲突)。
3. 存储: 扩 param_calib calib 域 payload (ver 递增, 兼容读旧记录):
   加 `acc_scale f32×3` + 有效标志, 与既有 `acc_bias_mgal` 同域。
4. 应用: imu_data.c 校准链 (轴映射之后) 逐轴乘 scale; mag/baro/gins
   全链路自动受益。
5. 轴失准 (非对角项) 不覆盖 —— 姿态相关残差由 EKF 的 SA 先验
   (GINS_IMU_AS_STD=10000ppm, 2026-10-02 放宽) 在线吸收。

## 待执行: 磁航向绝对精度指北核对 (需户外/已知方位)

标定后磁航向的稳定性/重复性已优秀 (yaw circstd 0.02-0.06°), 椭球标定
2026-10-02 已完成; 轴向映射与磁偏角**均已运行期化** (W25Q64 nav 分区,
缺省 = gins_config.h 编译期宏)。绝对精度未经真北基准核对, 核对流程
(5 分钟):

1. 户外开阔地 (远离铁磁 >5m), 板子静止水平放, `gins` 读 yaw_1。
2. 用手机指南针/已知地标方位测同点真方位 yaw_true (修正当地磁偏角)。
3. 若 |yaw_1 - yaw_true| > 5°: 轴映射嫌疑 —— 将板子精确旋转 90°
   (用地面直角参考线), `gins` 读 yaw_2; |Δyaw| ≠ 90°±2° 说明映射
   非正交/符号错, 用 `nav set maxis <sx sy sz kx ky kz>` 现场修正
   (立即生效) 并 `nav save` 持久化; 若 Δyaw≈90° 但 yaw_1 偏差恒定,
   用 `nav set decl <deg>` 改磁偏角补偿。
4. 核对后在此 README 记录核对日期与残差。
