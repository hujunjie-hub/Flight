# protocol — 协议解析层

固件协议解析层。**纯协议模块**（不开线程、不占串口），按协议族一目录，
每个子目录自带 `SConscript` 与 `README.md`：

```
protocol/
├── nmea/     UM982 NMEA PVT 解析 (组合导航的 GNSS 观测输入) → nmea/README.md
├── mavlink/  MAVLink v2 链路 (地面站遥测/指令, 官方库 + 适配层) → mavlink/README.md
└── SConscript  SCons 构建入口 (逐子目录汇入, 同 middleware/SConscript 模式)
```

## 目录约定

- **接入方式**：串口驱动读到字节后喂各协议解析器的 feed 接口，解析结果
  快照/回调分发；发方向经注入的 sender 写回链路。协议层对硬件一无所知。
- **测试**：`<模块>/test/tc_*.c`（utest，板上运行），由根 `CMakeLists.txt`
  的 `RT_USING_UTEST` 源列表编入；SCons 只编目录顶层 `*.c`，不编测试。
- **新增协议**：新建子目录（.c/.h + SConscript + README + test/），根
  `CMakeLists.txt` 补对应源文件与 include 路径即可（双构建系统见仓库
  根目录说明）。
