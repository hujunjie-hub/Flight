# 平台服务测试

`tools/run_platform_tests.ps1`运行原始采集队列与时间服务的Host单元测试及ARM编译。当前设备源、IRQ/DMA绑定与完整镜像资源审计由根目录执行`python tests/drivers/check_drivers.py`。

早期Gate B快照工具依赖当时的驱动结构，已随原始内容移至`spec/history/implementation/platform/tools`，仅用于历史追溯；不作为现行源码的验收入口。
