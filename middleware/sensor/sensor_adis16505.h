/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADIS16505 精密 MEMS IMU (陀螺仪 + 加速度计 + 温度) RT-Thread 传感器驱动
 *
 * 硬件连接 (doc/Flight.xlsx 接口配置页):
 *   SPI1: SCLK=PA5, DOUT(MISO)=PA6, DIN(MOSI)=PA7, 模式3, 954.861 kHz
 *   CS   = PC4 (GPIO 软件片选)
 *   RESET= PC5 (GPIO, 高电平空闲, 低有效)
 *   DR   = PA4 (EXTI4, 数据就绪中断, 抢占优先级 2)
 *
 * 采样链路 (DR 中断 + SPI DMA, 中断驱动, 统一执行流程见根 README):
 *   DR 上升沿 EXTI ISR  -> ① 读 TIM2 合成 64 位 T_MCU 时戳 cnt_event
 *                        ② 检查 DMA 空闲 ③ CS 拉低 + 启动 22 字节
 *                          burst DMA, 立即退出
 *   DMA 完成中断 (hook) -> CS 拉高 + D-Cache 失效 + 缓冲入队 + 通知
 *                          处理线程, 立即退出
 *   处理线程 (优先级 6) -> 校验和校验 + DIAG_STAT + DATA_CNTR 连续性检查
 *                          + 原始数据解析 (挂接 ISR 捕获的时戳)
 *                          -> 最新快照
 *   (SPI1 为 ADIS 独占, burst 走 HAL DMA 直连, 初始化后不经 rt_spi 框架;
 *    运行期寄存器访问先挂起链路再 HAL 轮询, 保证与 burst 互斥)
 *
 * 量程: 由 RANG_MDL(0x5E)[3:2] 自动识别 (-1:±125°/s, -2:±500°/s, -3:±2000°/s),
 *       加速度计固定 ±78.3 m/s² (≈±8 g)。
 * 采样率: 内部 2000 Hz, DEC_RATE 降采样, 默认 1000 Hz (Excel 清单)。
 *
 * 注册设备 (RT-Thread sensor 框架):
 *   acce_adis16505  单位 mG       (1 LSB = 78/32000 m/s² = 0.248575 mG)
 *   gyro_adis16505  单位 mdps     (-2 型 1 LSB = 0.025 dps = 25 mdps)
 *   temp_adis16505  单位 0.1°C    (1 LSB = 0.1°C)
 *
 * 用法:
 *   - 轮询: rt_device_open + rt_device_read 返回 DR 触发更新的最新快照
 *     (DR 停止时处理线程看门狗轮询重同步)。
 *   - 事件: rt_device_set_rx_indicate 后打开设备, 每次采样校验入库后调用
 *     cb (线程上下文, size=1), 消费方在 cb 中释放信号量再取快照。
 */

#ifndef __SENSOR_ADIS16505_H__
#define __SENSOR_ADIS16505_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <rtdevice.h>

/* ------------------------- 板级配置 (按需修改) ------------------------- */

/* SPI 总线与挂载的 SPI 设备名 */
#define ADIS16505_SPI_BUS_NAME      "spi1"
#define ADIS16505_SPI_DEVICE_NAME   "spi10"

/* 片选/复位/数据就绪引脚 */
#define ADIS16505_CS_PIN            GET_PIN(C, 4)
#define ADIS16505_RST_PIN           GET_PIN(C, 5)
#define ADIS16505_DR_PIN            GET_PIN(A, 4)

/* DR 中断抢占优先级 (Excel 清单: 2; 高于 DMA1_Stream0/1 完成中断的 3) */
#define ADIS16505_DR_IRQ_PRIO       2

/* 采样处理线程: 优先级高于使用者线程 (main=10), 栈 1KB */
#define ADIS16505_DR_THREAD_PRIO    6
#define ADIS16505_DR_THREAD_STACK   1024
#define ADIS16505_DR_THREAD_TICK    10

/* 处理线程等待超时 ms: 超时进入看门狗 (DMA 卡死复位 / DR 停止轮询重同步) */
#define ADIS16505_PROC_WAIT_MS      5

/* SPI 时钟: 普通读写上限 2.1 MHz, burst 读上限 1.1 MHz。
 * 61.111 MHz 内核时钟 /64 = 954.861 kHz, 与 CubeMX 配置一致。 */
#define ADIS16505_SPI_MAX_HZ        954861

/* 默认输出数据率 Hz (内部 2000Hz 降采样: 2000/(DEC_RATE+1)) */
#define ADIS16505_DEFAULT_ODR       1000

/* 上电后硬件复位等待时间 ms (数据手册: reset 后 310ms 内勿访问) */
#define ADIS16505_RESET_DELAY_MS    310

/* PROD_ID 校验值 */
#define ADIS16505_PROD_ID           16505

/* ------------------------- 驱动接口 ------------------------- */

int rt_hw_adis16505_init(void);

/* 一次 DR 触发 burst 采样的完整快照 (时戳在 DR EXTI ISR 内捕获, 见下) */
struct adis16505_snapshot
{
    rt_uint64_t t_event_us;     /* T_MCU 时戳, us: DR 上升沿时刻 (EXTI ISR
                                 * 读 TIM2 + 溢出累计合成, middleware/timebase) */
    rt_int16_t  gyro[3];        /* 原始 16-bit (mdps = raw × 量程灵敏度) */
    rt_int16_t  acce[3];        /* 原始 16-bit (mG = raw × 0.248575) */
    rt_int16_t  temp;           /* 0.1°C/LSB */
    rt_uint16_t data_cntr;      /* 芯片数据计数器 (采样序号, 16bit 回绕) */
    rt_uint16_t cntr_gap;       /* 距上一入库样本的 DATA_CNTR 增量
                                 * (1=连续, >1 丢拍) */
    rt_uint16_t diag_stat;      /* 状态标志 */
};

/* DR->DMA 链路健康统计 */
struct adis16505_dr_stats
{
    rt_uint32_t miss;           /* DR 边沿到来时 DMA 未空闲 (上一帧未完成) */
    rt_uint32_t drop;           /* DATA_CNTR 连续性检测到的累计丢拍数 */
    rt_uint32_t chk_err;        /* burst 校验和错误帧数 */
    rt_uint32_t diag_err;       /* DIAG_STAT 置位帧数 */
    rt_uint32_t dma_err;        /* SPI/DMA 错误次数 */
    rt_uint32_t overrun;        /* 处理线程消费不及, 丢弃的帧数 */
    rt_uint32_t recover;        /* 看门狗强制复位传输状态次数 */
};

/* 取最近一次采样快照 (线程安全), DR 未触发过时返回 -RT_EEMPTY */
rt_err_t adis16505_get_snapshot(struct adis16505_snapshot *snap);

/* 陀螺当前灵敏度 (mdps/LSB, 量程随 RANG_MDL 型号自动识别)。
 * 供数据缓冲层做单位换算, 避免各处复制量程表;
 * 加速度/温度灵敏度固定: 78/32000 m/s²/LSB 与 0.1°C/LSB (见文件头说明)。 */
float adis16505_gyro_lsb_mdps(void);

/* 取 DR->DMA 链路健康统计 (任意线程安全) */
void adis16505_get_dr_stats(struct adis16505_dr_stats *st);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_ADIS16505_H__ */
