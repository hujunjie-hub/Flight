# middleware/Vehicle_Model/dshot_output — 电调输出 (PWM / DShot)

[control_allocation](../control_allocation/README.md) 的 `u[4]` 归一化
电机量 → 4 路 PWM (400Hz, 1000~2000µs) 或 DShot150/300/600 帧。

## 文件

| 文件 | 说明 |
|------|------|
| `dshot_enc.h` / `dshot_enc.c` | 纯 C 帧编码: 11bit 值 + 遥测位 + CRC4 (算法与 ref/FMT-Firmware actuator.c 一致); 无 RT-Thread 依赖, 可主机测试 |
| `dshot.h` / `dshot.c`         | 板级引擎: TIM PWM / TIM+DMAR 突发 DShot 双协议 + FinSH `dshot` |
| `dshot_hw.h`                  | **硬件配置 (全部占位, 见下)** |

## ⚠ 硬件配置全部为占位值 (硬件方案落地后改 dshot_hw.h)

电机输出引脚**尚未在 doc/Flight.xlsx 引脚表 / Flight.ioc 分配**,
当前占位: **TIM4 CH1..4 @ PB6/PB7/PB8/PB9 (AF2) + DMA1 Stream5
(TIM4_UP, DMAMUX 请求 32)**。选点依据: PB6/7 因 BMM350 迁 I2C4 腾空、
PB8/9 未用、TIM4 空闲、DMA1 Stream5 空闲 (板级映射见
`libraries/HAL_Drivers/drivers/config/h7/dma_config.h`)。
落地后按 `dshot_hw.h` 头注的 4 步清单调整 (引脚/通道/AF/DMA 宏; 若移出
TIM4 需 grep "TIM4" 同步; CubeMX 补外设后核对 hal_msp.c 无冲突段;
board.c NVIC 表按需补行)。

## 实现要点

- **PWM**: TIM 1MHz 计数 @400Hz, `CCR = 1000+u·1000` µs, 写即生效;
  disarm → CCR=0 恒低 (无脉冲 = 停转)。
- **DShot**: TIM 位率计数 (DShot600@275MHz: ARR=458, bit1≈75%/bit0≈37.5%
  占空), 每帧 17 组 × 4 电机 CCR 值 (16 bit + 复位低) 打包进 32B 对齐
  DMA 缓冲 (DCache Clean), `HAL_DMA_Start` 后由 TIM 更新事件经 **DMAR
  突发装载 CCR1..4** (四电机严格同步); 帧间恒低; 上一帧未完成时的写
  丢弃并 `busy_cnt++` (输出周期须 > 帧长, DShot600 一帧 ≈28µs)。
- **安全**: init 后默认 disarm; `arm` 前一切 write 被拒; 非有限输入按
  停转值处理并计数。
- 值域: 0=停转, 1..47=命令 (BEEP/换向 `raw 7|8`/3D/保存设置),
  48..2047=节流 (`dshot_enc.h` 命令表, 与 FMT 一致)。

## FinSH `dshot`

```
dshot init dshot600    # 或 pwm/dshot300/dshot150 (初始 disarm 恒低)
dshot arm              # 使能输出 (确认桨已拆!)
dshot w 0.3 0.3 0.3 0.3   # 4 电机归一化 [0,1]
dshot raw 5            # 全通道命令帧 (5=BEEP5; 7/8=换向)
dshot                  # 状态 (含 DShot 位时序计数/busy 计数)
dshot disarm | deinit
```

## 验证

- 主机 (`build_host/mpc_xcheck.py`): 帧编码对拍 DShot 规范公式 (500 随机
  值零差异) + 黄金值 (2047→0xFFEE, 48→0x0606) + 节流映射端点/中点;
  2026-10-03 全过。
- **板上未验证** (硬件未接): TIM/DMAR/DMA 通路上板后用示波器/逻辑分析
  仪核对 DShot600 时序 (1.67µs 位周期, 75%/37.5% 占空) 与 PWM 脉宽,
  再接电调。
