# param_calib — 飞控参数分区存储 (W25Q64)

按分区把飞控参数持久化到外置 W25Q64 (8MB, OCTOSPI1, 驱动见
`middleware/Sensor_Drivers/sensor_w25q64.c`)。分层:

```
param_part.c   分区引擎: 分区表副本 + 追加式记录 (magic/ver/seq/CRC32)
               + 写满整擦回卷, 掉电原子 + 磨损均衡 (通用记录纪律)
param_calib.c  calib 域: 传感器标定 (磁椭球/气压偏移/加计零偏),
               calib_store_* API 与原 calibration/calib_store 完全一致
param_nav.c    nav 域: 导航参数 (磁偏角/NOGNSS 部署位置/轴向映射/磁开关),
               镜像缺省 = gins_config.h 编译期宏, `nav set` 现场改
param_sys.c    sys 域: 启动计数/固件标识 (FNV-1a of __DATE__ __TIME__)
               / 片内 Flash 旧标定已迁移标志
```

## 分区布局 (编译期权威在 param_part.c `s_defs`)

| 分区 | 偏移 | 大小 | 槽 | 内容 |
| --- | --- | --- | --- | --- |
| ptbl | 0x000000 | 4KB | — | 分区表自描述副本 (工具/自检用, 非记录区) |
| calib | 0x001000 | 16KB | 128×128B | 磁椭球 (bias/softiron/radius) + 气压偏移 + 加计零偏 (mGal, 温度标签) |
| nav | 0x005000 | 16KB | 128×128B | 磁偏角 / NOGNSS 部署位置 / IMU+磁轴向映射 / 磁观测开关 |
| sys | 0x009000 | 8KB | 128×64B | 启动计数 (每次上电一条) / 固件标识 / 迁移标志 |
| ctrl | 0x00B000 | 20KB | 80×256B | 预留 (middleware/Control 控制律) |
| — | 0x010000..0x7FFFFF | ~7.9MB | — | 未分配 (未来黑匣子/航点, 建议从片尾向前规划) |

记录槽格式 (小端, 见 param_part.h): `magic 'PARM' | ver u16 | part u16 |
seq u32 | len u16 | rsv u16 | crc32 u32 | payload`; CRC 覆盖 ver/part/seq/len
与 payload。上电扫描取 "CRC 通过且 seq 最大" 的记录, 撕裂/坏记录跳过;
追加写满整擦回卷, 掉电至多损失最新一条。

## 与原片内 Flash 方案 (calibration/calib_store, 已删) 的差异

- **不关中断**: OCTOSPI 间接模式访问, 编程/擦除期间 CPU 照常运行,
  四链 EXTI/DMA 不停摆, 无需 DCache 开关与 timebase 回绕核对窗口。
- **容量与分区**: 单一 128KB 扇区 → 独立分区按域擦除, 新增 nav/sys/ctrl。
- **一次性导入**: 首次上电若 calib 分区无记录而片内扇区 7 (0x080E0000)
  留有旧格式有效记录, 解析后转存 W25Q64 (2026-10-02 实测标定不丢失);
  完成与否由 sys 分区标志记住, `param erase calib` 后不会复活旧值。
- **降级**: W25Q64 缺失时引擎告警一次, load 返回无记录 (各域用缺省值),
  save 返回 -RT_EIO, 系统其余部分不受影响。

## FinSH 命令

- `param` — 分区表与各分区状态 (seq/槽位); `param erase <calib|nav|sys|ctrl|all>` 恢复出厂
- `nav` — 导航参数查看; `nav set decl|lat|lon|alt|mag <v>`、
  `nav set iaxis|maxis <sx sy sz kx ky kz>` (轴向重排+符号, 立即生效)、
  `nav save|load|reset`
- `sysinfo` — 启动计数/固件标识/迁移标志
- 标定域沿用原命令: `magcal` / `barocal` / `accbias save|clear`

## 构建接线

- SCons: `middleware/SConscript` 自动收录本目录 SConscript (DefineGroup
  'ParamCalib', CPPPATH 含 sensor/gins)。
- CMake: `CMakeLists.txt` 的 `RT_PARAMCALIB_SOURCES` 与上者保持同步
  (三处源清单约定见根 README)。
- 消费方: `../filter_calib/mag_calib.c`、`../filter_calib/baro_calib.c`、
  `gins/gins_bridge.cpp` (include param_calib.h); `gins_bridge.cpp`、
  `middleware/Sensor_Preprocessing/process_data/imu_data.c`、`middleware/Sensor_Preprocessing/process_data/mag_data.c` (include param_nav.h)。
