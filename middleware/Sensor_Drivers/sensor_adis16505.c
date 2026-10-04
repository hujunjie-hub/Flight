/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ADIS16505 IMU RT-Thread 传感器驱动实现 (SPI1, DR 中断直启 DMA burst)
 *
 * 移植自 Analog Devices no-OS 驱动 (doc/ADIS16505/no-OS) 与数据手册
 * Rev.C (doc/ADIS16505/adis16505.pdf):
 *  - 寄存器读: 两帧 16-bit, 第一帧发地址(bit7=0), 第二帧收数据, 帧间 tSTALL≥16us
 *  - 寄存器写: 每字节地址一帧, bit7=1, LSB 在低地址
 *  - burst 读: DIN=0x6800 后 CS 保持低连续读出 10 段 (含校验和), 时钟≤1.1MHz
 *  - DR 数据就绪: MSC_CTRL bit0 默认 1, DR 空闲高/脉冲低, 上升沿表示就绪
 *  - 所有寄存器均在页 0, 无需翻页
 *
 * 采样链路 (中断驱动, 两个 ISR 内都只做必须的事, 立即退出):
 *   DR EXTI ISR (抢占优先级 2):
 *     ① 读 TIM2 合成 64 位 T_MCU 时戳 cnt_event (timebase, 时戳锚定在
 *        硬件事件时刻, 与总线传输/线程调度延迟解耦)
 *     ② 检查 DMA 空闲 (上一帧未完成则计 miss 丢弃)
 *     ③ CS 拉低 + HAL_SPI_TransmitReceive_DMA 启动 22 字节 burst
 *   DMA 完成中断 (drv_spi 框架钩子, DMA1_Stream0, 抢占优先级 3):
 *     ① CS 拉高, D-Cache 失效接收缓冲
 *     ② 接收槽位入 SPSC 队列, 信号量通知处理线程
 *   处理线程 (优先级 6):
 *     校验和校验 -> DIAG_STAT 检查 -> DATA_CNTR 连续性检查 -> 原始数据
 *     解析 (挂接 ISR 按槽位捕获的 cnt_event) -> 最新快照入库 + 通知已
 *     打开的框架设备;
 *     等待超时进入看门狗: DMA 卡死强制复位传输状态 / DR 停止时轮询
 *     burst 重同步 (数据链路自恢复, 轮询路径时戳取 burst 完成时刻)。
 *
 * SPI1 为 ADIS 独占: burst 走 HAL DMA 直连 (初始化后不经 rt_spi 框架,
 * 框架仅用于上电阶段的寄存器配置); 运行期寄存器访问 (自检/ODR) 先挂起
 * DR 链路 (关 EXTI + 等 DMA 空闲) 再 HAL 轮询, 保证与 burst 互斥。
 */

#include "sensor_adis16505.h"
#include <drivers/sensor.h>
#include "drv_gpio.h"
#include "drv_spi.h"
#include "timebase.h"                   /* middleware/Sensor_Drivers T_MCU 时基 */

#define LOG_TAG "sensor.adis"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 寄存器定义 (页0) ------------------------- */
#define ADIS_REG_DIAG_STAT      0x02    /* 状态/错误标志 */
#define ADIS_REG_TEMP_OUT       0x1C    /* 温度 0.1°C/LSB */
#define ADIS_REG_DATA_CNTR      0x22
#define ADIS_REG_RANG_MDL       0x5E    /* [3:2] 陀螺量程型号识别 */

/* 内部 flash 零偏校正寄存器缓存 (上电配置期读取, `adisbias` 打印):
 * XG/YG/ZG_BIAS 0x40-0x45, XA/YA/ZA_BIAS 0x46-0x4B, DIAG_STAT 0x02,
 * FILT_CTRL 0x5C, RANG_MDL 0x5E。lo/hi 为 32-bit 两补码校正词的低/高 16b */
static struct
{
    rt_bool_t   valid;
    rt_uint16_t diag, filt, rang;
    rt_uint16_t g_lo[3], g_hi[3];
    rt_uint16_t a_lo[3], a_hi[3];
} s_bias_cache;
#define ADIS_REG_MSC_CTRL       0x60    /* bit0 DR 极性(默认1: 上升沿就绪) */
#define ADIS_REG_DEC_RATE       0x64    /* 降采样 2000/(DEC+1) Hz */
#define ADIS_REG_GLOB_CMD       0x68    /* bit7 软复位, bit2 自检 */

/* DATA_CNTR 回绕式大 gap = 芯片自复位特征 (计数器归零重新累计,
 * gap = 65536-last+新值, 远超真实丢拍量级): 触发运行配置重写。
 * 芯片复位后 DEC_RATE 回默认 0 → ODR 静默从 1000Hz 变 2000Hz,
 * DR 自己恢复翻转所以链路能自愈, 但负载翻倍且元数据失配
 * (2026-10-01 139.6s 自复位掉线故障的配置侧闭环) */
#define ADIS_CNTR_RESET_GAP     32768

/* 帧格式: 高字节 = 地址(bit7: 1写/0读), 低字节 = 数据
 * 注意: R/W 位 0=读 1=写 (数据手册 Figure 40, 如 DIN=0x0C00 读 Z_GYRO_LOW) */
#define ADIS_CMD_READ(reg)      (rt_uint8_t)(0x00u | (reg))
#define ADIS_CMD_WRITE(reg)     (rt_uint8_t)(0x80u | (reg))

/* burst 命令 0x6800, 响应 10 段: DIAG + 6轴 + TEMP + CNTR + CHECKSUM */
#define ADIS_BURST_TOTAL        22      /* 2 命令 + 20 数据字节 */

/* burst DMA 缓冲: 32B 对齐 + 32B 长度 (cache 行整数倍, 见 drv_spi.c 说明) */
#define ADIS_DMA_BUF_SIZE       32

/* 完成队列容量 (SPSC 环, 实际可用 3 个槽位序号 + ping-pong×2) */
#define ADIS_QUEUE_SIZE         4

/* 帧间停顿 (数据手册表2: tSTALL ≥ 16us) */
#define ADIS_STALL_US           16

/* HAL 轮询传输超时 ms */
#define ADIS_SPI_TIMEOUT_MS     20

/* GLOB_CMD 自检位与等待 */
#define ADIS_GLOB_CMD_SELF_TEST 0x0004
#define ADIS_SELF_TEST_DELAY_MS 24

/* 陀螺量程/灵敏度 (RANG_MDL[3:2]: 0=±125°/s, 1=±500°/s, 3=±2000°/s) */
struct adis_gyro_range
{
    rt_int32_t  range_mdps;     /* 满量程 mdps */
    float       lsb_mdps;       /* mdps/LSB */
};
static const struct adis_gyro_range gyro_ranges[4] =
{
    {  125000, 6.25f },     /* 00: ADIS16505-1 */
    {  500000, 25.0f },     /* 01: ADIS16505-2 */
    {  500000, 25.0f },     /* 10: 保留, 按 -2 处理 */
    { 2000000, 100.0f },    /* 11: ADIS16505-3 */
};

/* 加速度计: 1 LSB = 78/32000 m/s² = 0.248575 mG (±78.3 m/s² ≈ ±8g) */
#define ADIS_ACCE_LSB_MG        (78.0f / 32000.0f / 9.80665f * 1000.0f)
#define ADIS_ACCE_RANGE_MG      7986

/* burst 命令帧 (0x6800 + 全 0 填充), 链路启动时填好并冲刷 cache 一次 */
static rt_uint8_t adis_tx_buf[ADIS_DMA_BUF_SIZE] rt_align(32);
/* ping-pong 接收缓冲: DMA 完成中断写入一槽, 处理线程消费另一槽 */
static rt_uint8_t adis_rx_buf[2][ADIS_DMA_BUF_SIZE] rt_align(32);
/* 每槽位 DR 沿时戳 (EXTI ISR 按槽捕获, 处理线程随帧取出配对) */
static rt_uint64_t adis_slot_t0[2];

/* ------------------------- 设备结构 ------------------------- */

/* 快照结构 adis16505_snapshot 定义在 sensor_adis16505.h */

struct adis16505_device
{
    struct rt_spi_device *spi;      /* 框架设备 (仅上电配置阶段使用) */
    SPI_HandleTypeDef    *hspi;     /* HAL 句柄 (链路启动后直连) */
    rt_uint16_t prod_id;
    rt_uint16_t dec_rate;        /* 当前降采样值 (0~1999) */
    rt_uint8_t  gyro_idx;        /* 量程索引 (RANG_MDL[3:2]) */

    /* DR->DMA 链路状态 */
    rt_bool_t   dr_mode;            /* 链路已启用 (寄存器访问路径切换) */
    volatile rt_bool_t chain_on;    /* 链路工作开关 (挂起/恢复) */
    volatile rt_bool_t dma_busy;    /* 一次 burst DMA 进行中 */
    volatile rt_uint8_t cur_slot;   /* 下一次 DMA 写入的 ping-pong 槽 */

    /* 完成队列: DMA 完成中断生产 / 处理线程消费 (SPSC 环) */
    volatile rt_uint8_t q_head, q_tail;
    rt_uint8_t  q_slot[ADIS_QUEUE_SIZE];
    struct rt_semaphore proc_sem;   /* 完成通知 */

    rt_thread_t dr_thread;

    /* 线程级 HAL 传输互斥: 看门狗轮询 (dr_thread) / fetch 兜底轮询
     * (imu_data 线程) / 运行期寄存器访问 (FinSH/control) 三条路径都会
     * 执行 chain_suspend + HAL 传输序列, rt_sensor 框架的 module->lock
     * 只串行化 read/control, 管不到看门狗 —— 无此锁时 HAL 状态机的
     * check-then-set 非原子, 并发传输会互相拆台。ISR 路径不持此锁。 */
    struct rt_mutex xfer_lock;

    /* 最新快照 (处理线程写, 读取方拷贝, 关中断保护) */
    struct adis16505_snapshot sample;
    rt_bool_t   sample_valid;
    rt_uint16_t last_cntr;          /* DATA_CNTR 连续性基准 */
    rt_bool_t   cntr_init;
    volatile rt_bool_t reconfig_req; /* 芯片自复位特征检出, 待重写运行配置 */
    rt_tick_t   reconfig_tick;      /* 重写退避基准 */

    struct adis16505_dr_stats stats;

    /* 轮询模式: 同一 tick 内多个传感器共享一次 burst */
    rt_tick_t  last_tick;
};

static struct adis16505_device adis_dev;
static struct rt_sensor_module sensor_module;   /* acce/gyro/temp 共用一个模块 */

/* ------------------------- CS 与 HAL 轮询传输 ------------------------- */

static void adis_cs_low(void)
{
    rt_pin_write(ADIS16505_CS_PIN, PIN_LOW);
}

static void adis_cs_high(void)
{
    rt_pin_write(ADIS16505_CS_PIN, PIN_HIGH);
}

/* HAL 直连全双工传输 (调用方已保证与 burst DMA 互斥) */
static rt_err_t adis_hal_xfer(const rt_uint8_t *tx, rt_uint8_t *rx, rt_uint16_t len)
{
    if (HAL_SPI_TransmitReceive(adis_dev.hspi, (rt_uint8_t *)tx, rx, len,
                                ADIS_SPI_TIMEOUT_MS) != HAL_OK)
        return -RT_EIO;
    return RT_EOK;
}

/* ------------------------- 链路挂起/恢复 ------------------------- */

/*
 * 运行期寄存器访问前挂起 DR 链路: 关 DR 中断并等待在传 burst 结束。
 * 挂起期间的 DR 边沿丢失由 DATA_CNTR 连续性检查如实计数。
 */
static rt_err_t adis_chain_suspend(void)
{
    rt_uint32_t wait = 0;

    if (!adis_dev.dr_mode)
        return RT_EOK;

    rt_pin_irq_enable(ADIS16505_DR_PIN, PIN_IRQ_DISABLE);
    /* 一次 burst DMA ≤ 200us, 2ms 等待上限足够 */
    while (adis_dev.dma_busy && wait < 40u)
    {
        rt_hw_us_delay(50);
        wait++;
    }
    if (adis_dev.dma_busy)
        return -RT_ETIMEOUT;
    return RT_EOK;
}

static void adis_chain_resume(void)
{
    if (adis_dev.dr_mode)
        rt_pin_irq_enable(ADIS16505_DR_PIN, PIN_IRQ_ENABLE);
}

/* ------------------------- 寄存器读写 ------------------------- */

/* 读 16-bit 寄存器: 命令帧 + 数据帧, 帧间停顿 */
static rt_err_t adis_read_reg16(rt_uint8_t reg, rt_uint16_t *val)
{
    rt_uint8_t cmd[2]  = {ADIS_CMD_READ(reg), 0x00};
    rt_uint8_t zero[2] = {0x00, 0x00};
    rt_uint8_t rx[2];
    rt_err_t ret;

    if (adis_dev.dr_mode)
    {
        /* 链路运行期: HAL 轮询直连, 与 burst 互斥 (xfer_lock 跨线程串行化) */
        rt_mutex_take(&adis_dev.xfer_lock, RT_WAITING_FOREVER);
        ret = adis_chain_suspend();
        if (ret != RT_EOK)
        {
            rt_mutex_release(&adis_dev.xfer_lock);
            return ret;
        }
        ret = adis_hal_xfer(cmd, rx, 2);
        rt_hw_us_delay(ADIS_STALL_US);
        if (ret == RT_EOK)
            ret = adis_hal_xfer(zero, rx, 2);
        rt_hw_us_delay(ADIS_STALL_US);
        adis_chain_resume();
        rt_mutex_release(&adis_dev.xfer_lock);
    }
    else
    {
        /* 上电配置阶段: 走 rt_spi 框架 (总线配置由框架在首次传输时应用),
         * 始终全双工 (TX-only 会留下未读 RX 数据触发 OVR, 见 HAL 错误路径) */
        if (rt_spi_transfer(adis_dev.spi, cmd, rx, 2) != 2)
            return -RT_EIO;
        rt_hw_us_delay(ADIS_STALL_US);

        if (rt_spi_transfer(adis_dev.spi, zero, rx, 2) != 2)
            return -RT_EIO;
        rt_hw_us_delay(ADIS_STALL_US);
        ret = RT_EOK;
    }

    if (ret == RT_EOK)
        *val = (rt_uint16_t)((rx[0] << 8) | rx[1]);
    return ret;
}

/* 写 16-bit 寄存器: 低地址写 LSB, 高地址写 MSB */
static rt_err_t adis_write_reg16(rt_uint8_t reg, rt_uint16_t val)
{
    rt_uint8_t cmd[2];
    rt_uint8_t rx[2];
    rt_err_t ret = RT_EOK;

    cmd[0] = ADIS_CMD_WRITE(reg);
    cmd[1] = (rt_uint8_t)(val & 0xFF);

    if (adis_dev.dr_mode)
    {
        rt_mutex_take(&adis_dev.xfer_lock, RT_WAITING_FOREVER);
        ret = adis_chain_suspend();
        if (ret != RT_EOK)
        {
            rt_mutex_release(&adis_dev.xfer_lock);
            return ret;
        }
        ret = adis_hal_xfer(cmd, rx, 2);
        rt_hw_us_delay(ADIS_STALL_US);
        if (ret == RT_EOK)
        {
            cmd[0] = ADIS_CMD_WRITE(reg + 1);
            cmd[1] = (rt_uint8_t)(val >> 8);
            ret = adis_hal_xfer(cmd, rx, 2);
        }
        rt_hw_us_delay(ADIS_STALL_US);
        adis_chain_resume();
        rt_mutex_release(&adis_dev.xfer_lock);
    }
    else
    {
        if (rt_spi_transfer(adis_dev.spi, cmd, rx, 2) != 2)
            return -RT_EIO;
        rt_hw_us_delay(ADIS_STALL_US);

        cmd[0] = ADIS_CMD_WRITE(reg + 1);
        cmd[1] = (rt_uint8_t)(val >> 8);
        if (rt_spi_transfer(adis_dev.spi, cmd, rx, 2) != 2)
            return -RT_EIO;
        rt_hw_us_delay(ADIS_STALL_US);
    }

    return ret;
}

/* ------------------------- burst 帧校验/解析 ------------------------- */

/* 校验和 = DIAG..DATA_CNTR 共 18 字节之和 (16-bit 减法归零校验) */
static rt_bool_t adis_frame_valid(const rt_uint8_t *rx)
{
    rt_uint16_t checksum = (rt_uint16_t)((rx[20] << 8) | rx[21]);
    rt_uint8_t i;

    for (i = 2; i < 20; i++)
        checksum = (rt_uint16_t)(checksum - rx[i]);
    return (checksum == 0) ? RT_TRUE : RT_FALSE;
}

/*
 * 帧布局: [0:2]=命令, [2:4]=DIAG_STAT, [4:16]=6 轴(每轴 MSB 在前),
 *         [16:18]=TEMP_OUT, [18:20]=DATA_CNTR, [20:22]=CHECKSUM。
 * t0 为本帧 DR 沿时戳 (EXTI ISR 按槽位捕获; 轮询路径为 burst 完成时刻)。
 */
static void adis_frame_parse(const rt_uint8_t *rx, rt_uint64_t t0,
                             struct adis16505_snapshot *out)
{
    out->t_event_us = t0;
    out->gyro[0]     = (rt_int16_t)((rx[4] << 8) | rx[5]);
    out->gyro[1]     = (rt_int16_t)((rx[6] << 8) | rx[7]);
    out->gyro[2]     = (rt_int16_t)((rx[8] << 8) | rx[9]);
    out->acce[0]     = (rt_int16_t)((rx[10] << 8) | rx[11]);
    out->acce[1]     = (rt_int16_t)((rx[12] << 8) | rx[13]);
    out->acce[2]     = (rt_int16_t)((rx[14] << 8) | rx[15]);
    out->temp        = (rt_int16_t)((rx[16] << 8) | rx[17]);
    out->data_cntr   = (rt_uint16_t)((rx[18] << 8) | rx[19]);
    out->diag_stat   = (rt_uint16_t)((rx[2] << 8) | rx[3]);
    out->cntr_gap    = 1;              /* 连续性由入库函数填写 */
}

/* 快照写入缓存 (关中断保护, 保证多读者取到完整快照) */
static void adis_sample_store(const struct adis16505_snapshot *in)
{
    rt_base_t level = rt_hw_interrupt_disable();

    adis_dev.sample = *in;
    adis_dev.sample_valid = RT_TRUE;
    rt_hw_interrupt_enable(level);
}

static void adis_sample_load(struct adis16505_snapshot *out)
{
    rt_base_t level = rt_hw_interrupt_disable();

    *out = adis_dev.sample;
    rt_hw_interrupt_enable(level);
}

/*
 * 样本入库: DATA_CNTR 连续性检查 (丢拍如实计数) + 快照存储 +
 * 通知已打开的框架设备。校验通过后的唯一入口, 轮询重同步路径复用;
 * DATA_CNTR 与上一入库样本相同 (纯探活的重复读) 时直接忽略。
 */
static void adis_snapshot_commit(struct adis16505_snapshot *snap, rt_bool_t indicate)
{
    rt_uint16_t gap;
    rt_uint8_t i;

    if (adis_dev.cntr_init)
    {
        if (snap->data_cntr == adis_dev.last_cntr)
            return;                     /* 重复样本 (轮询探活), 不入库 */
        gap = (rt_uint16_t)(snap->data_cntr - adis_dev.last_cntr);
        snap->cntr_gap = gap;
        if (gap > 1u)
            adis_dev.stats.drop += (rt_uint32_t)(gap - 1u);
        if (gap >= ADIS_CNTR_RESET_GAP)
            adis_dev.reconfig_req = RT_TRUE;   /* 处理线程重写 DEC_RATE */
    }
    else
    {
        adis_dev.cntr_init = RT_TRUE;
    }
    adis_dev.last_cntr = snap->data_cntr;

    if (snap->diag_stat != 0u)
        adis_dev.stats.diag_err++;

    adis_sample_store(snap);

    /* 通知设置了 rx_indicate 且已打开的设备 (线程上下文, size=1) */
    if (indicate)
    {
        for (i = 0; i < sensor_module.sen_num; i++)
        {
            struct rt_sensor_device *sen = sensor_module.sen[i];
            if (sen->parent.ref_count > 0 && sen->parent.rx_indicate != RT_NULL)
                sen->parent.rx_indicate(&sen->parent, 1);
        }
    }
}

/* 校验并入库一帧 (处理线程路径, t0 为该槽位 DR 沿时戳) */
static void adis_process_frame(const rt_uint8_t *rx, rt_uint64_t t0)
{
    struct adis16505_snapshot snap;

    if (!adis_frame_valid(rx))
    {
        adis_dev.stats.chk_err++;
        return;                     /* 坏帧直接丢弃, 等下一 DR 拍 */
    }

    adis_frame_parse(rx, t0, &snap);
    adis_snapshot_commit(&snap, RT_TRUE);
}

/* ------------------------- DR 中断 -> DMA burst 链路 ------------------------- */

/*
 * DR 数据就绪中断 (抢占优先级 2, 可抢占 DMA 完成中断):
 * ① 读 TIM2 捕获 DR 沿时戳 cnt_event ② 检查 DMA 空闲 ③ 启动 burst DMA,
 * 立即退出。ping-pong 槽占用规则: cur_slot 不在完成队列中 (完成时翻转),
 * 队列满 2 个未消费槽时不再启动, 防止覆盖处理线程正在解析的缓冲。
 */
static void adis_dr_isr(void *args)
{
    rt_uint8_t qcnt;
    rt_uint8_t slot;

    RT_UNUSED(args);

    /* ① 先捕获 DR 沿时戳 cnt_event (槽位稍后确定, 时戳与事件解耦) */
    rt_uint64_t t_event = timebase_now_us();

    /* ② DMA 忙/SPI 未就绪/两槽均未消费: 本拍放弃 */
    if (!adis_dev.chain_on || adis_dev.dma_busy ||
        adis_dev.hspi->State != HAL_SPI_STATE_READY)
    {
        adis_dev.stats.miss++;
        return;
    }
    qcnt = (rt_uint8_t)((adis_dev.q_head - adis_dev.q_tail) & (ADIS_QUEUE_SIZE - 1u));
    if (qcnt >= 2u)
    {
        adis_dev.stats.overrun++;
        return;
    }

    /* ③ 启动 burst DMA: CS 拉低, 发 0x6800 并同步收 22 字节 */
    slot = adis_dev.cur_slot;
    adis_slot_t0[slot] = t_event;
    adis_dev.dma_busy = RT_TRUE;
    adis_cs_low();
    if (HAL_SPI_TransmitReceive_DMA(adis_dev.hspi, adis_tx_buf,
                                    adis_rx_buf[slot],
                                    ADIS_BURST_TOTAL) != HAL_OK)
    {
        adis_cs_high();
        adis_dev.dma_busy = RT_FALSE;
        adis_dev.stats.dma_err++;
    }
}

/*
 * drv_spi 框架完成钩子 (HAL_SPI_TxRxCpltCallback 内调用, DMA ISR 上下文):
 * ① 判断链路归属 ② CS 拉高 + 缓冲区失效入队 ③ 通知处理线程, 立即退出。
 * 覆盖 drv_spi.c 的弱定义, SPI1 事件被本驱动消费, 不进框架 completion。
 */
rt_bool_t stm32_spi_txrx_cplt_hook(SPI_HandleTypeDef *hspi)
{
    rt_uint8_t done, head;

    if (hspi->Instance != SPI1 || !adis_dev.chain_on)
        return RT_FALSE;

    done = adis_dev.cur_slot;
    adis_cs_high();
    /* DMA 写后的接收缓冲按整 cache 行失效 (缓冲 32B 对齐 + 32B 长度) */
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_INVALIDATE, adis_rx_buf[done],
                         ADIS_DMA_BUF_SIZE);

    head = adis_dev.q_head;
    if (((rt_uint8_t)((head + 1u - adis_dev.q_tail)) & (ADIS_QUEUE_SIZE - 1u)) == 0u)
    {
        /* 队列满 (处理线程阻塞 >2 拍): 丢弃本帧并暂停链路, 线程排空后
         * 由下一 DR 边沿自然重启。overrun 与 DR ISR (更高抢占优先级)
         * 并发自增, 关中断避免读-改-写丢失 */
        rt_base_t level = rt_hw_interrupt_disable();

        adis_dev.stats.overrun++;
        rt_hw_interrupt_enable(level);
        adis_dev.dma_busy = RT_FALSE;
        return RT_TRUE;
    }

    adis_dev.q_slot[head] = done;
    adis_dev.q_head = (rt_uint8_t)((head + 1u) & (ADIS_QUEUE_SIZE - 1u));
    adis_dev.cur_slot = (rt_uint8_t)(done ^ 1u);
    adis_dev.dma_busy = RT_FALSE;
    rt_sem_release(&adis_dev.proc_sem);
    return RT_TRUE;
}

/* SPI/DMA 错误钩子 (drv_spi.c HAL_SPI_ErrorCallback 尾部调用, 框架已唤醒
 * 等待者): 计数并复位链路状态, 下一 DR 边沿重启 */
void stm32_spi_error_hook(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1 || !adis_dev.chain_on)
        return;

    adis_cs_high();
    adis_dev.dma_busy = RT_FALSE;
    adis_dev.stats.dma_err++;
}

/*
 * 看门狗 (处理线程等待超时进入): 传输卡死强制复位 / DR 停止轮询重同步。
 * 轮询 burst 仅在 DATA_CNTR 有新值时入库 (纯探活不产生伪样本)。
 */
static void adis_watchdog(void)
{
    rt_uint8_t rx[ADIS_BURST_TOTAL];
    struct adis16505_snapshot snap;

    if (!adis_dev.chain_on)
        return;

    rt_mutex_take(&adis_dev.xfer_lock, RT_WAITING_FOREVER);

    if (adis_dev.dma_busy)
    {
        /* 传输卡死 (DMA/SPI 异常且错误回调未触发): 强制复位传输状态 */
        HAL_SPI_Abort(adis_dev.hspi);
        if (adis_dev.hspi->State != HAL_SPI_STATE_READY)
            adis_dev.hspi->State = HAL_SPI_STATE_READY;
        if (adis_dev.hspi->Lock != HAL_UNLOCKED)
        {
            /* Abort 失败路径 HAL 可能仍持锁: 不解开则此后所有传输入口
             * 直接返回 HAL_BUSY, 链路永久死亡, 与自恢复目标相悖 */
            __HAL_UNLOCK(adis_dev.hspi);
        }
        adis_cs_high();
        adis_dev.dma_busy = RT_FALSE;
        adis_dev.stats.recover++;
        rt_mutex_release(&adis_dev.xfer_lock);
        return;
    }

    /* DR 停止/断线: 主动 burst 重同步 (链路挂起保证与 DMA 互斥),
     * 时戳取 burst 完成时刻 (轮询路径无 DR 沿) */
    if (adis_chain_suspend() == RT_EOK)
    {
        if (adis_hal_xfer(adis_tx_buf, rx, ADIS_BURST_TOTAL) == RT_EOK &&
            adis_frame_valid(rx))
        {
            adis_frame_parse(rx, timebase_now_us(), &snap);
            if (snap.data_cntr != adis_dev.last_cntr)   /* 仅新数据入库 */
                adis_snapshot_commit(&snap, RT_TRUE);
        }
        adis_chain_resume();
    }
    rt_mutex_release(&adis_dev.xfer_lock);
}

/*
 * 芯片自复位后的运行配置重写 (处理线程上下文, 1s 退避):
 * 复用 adis_set_odr 的寄存器访问路径 (内部自带 xfer_lock + 链路挂起)。
 * 当前固件写入的运行配置仅 DEC_RATE 一项 (MSC_CTRL 只读不改, 芯片
 * 默认 DR 极性与驱动期望一致, 复位后无需重写)。
 */
static rt_err_t adis_set_odr(rt_uint32_t odr);   /* 定义在 control 段 */

#define ADIS_RECONFIG_BACKOFF_MS  1000

static void adis_reapply_config(void)
{
    if (adis_set_odr(ADIS16505_DEFAULT_ODR) == RT_EOK)
    {
        adis_dev.stats.reconfig++;
        adis_dev.reconfig_req = RT_FALSE;
        LOG_I("adis: chip reset detected, DEC_RATE rewritten (ODR=%dHz)",
              ADIS16505_DEFAULT_ODR);
    }
    else
    {
        /* 重写失败保持请求, 下一退避窗口重试 (SPI 恢复中) */
        LOG_W("adis: DEC_RATE rewrite failed, retry in %dms",
              ADIS_RECONFIG_BACKOFF_MS);
    }
}

/* 处理线程: 等待 DMA 完成通知, 排空完成队列逐帧校验入库 */
static void adis_dr_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        if (rt_sem_take(&adis_dev.proc_sem,
                        rt_tick_from_millisecond(ADIS16505_PROC_WAIT_MS)) != RT_EOK)
        {
            adis_watchdog();
            continue;
        }

        /* 排空积压帧 (DMA 完成中断持续生产, SPSC 环), 槽位时戳随帧配对。
         * q_tail 必须在解析完成后才推进: 解析期间该槽在队列中仍记为占用,
         * DR ISR 的 qcnt>=2 守卫才会拒绝向此槽发起新 DMA —— 若先推进再
         * 解析, 队列空出 1 格后新 DMA 会覆盖正在解析的缓冲 (帧新旧混合 +
         * 64 位槽位时戳撕裂)。守卫拒绝的拍计入 overrun, 与消费阻塞同义。 */
        while (adis_dev.q_tail != adis_dev.q_head)
        {
            rt_uint8_t tail = adis_dev.q_tail;
            rt_uint8_t slot = adis_dev.q_slot[tail];

            adis_process_frame(adis_rx_buf[slot], adis_slot_t0[slot]);
            adis_dev.q_tail = (rt_uint8_t)((tail + 1u) & (ADIS_QUEUE_SIZE - 1u));
        }

        /* 芯片自复位检出 (DATA_CNTR 回绕式 gap): 重写运行配置 */
        if (adis_dev.reconfig_req &&
            (rt_tick_get() - adis_dev.reconfig_tick) >=
                rt_tick_from_millisecond(ADIS_RECONFIG_BACKOFF_MS))
        {
            adis_dev.reconfig_tick = rt_tick_get();
            adis_reapply_config();
        }
    }
}

/* 配置 DR 引脚中断、DMA 链路资源并启动处理线程 */
static rt_err_t adis_dr_start(void)
{
    rt_uint16_t msc_ctrl = 0;
    rt_uint8_t  irq_mode;
    DMA_HandleTypeDef *dma_rx, *dma_tx;

    /* MSC_CTRL bit0: 1(默认)=DR 空闲高/脉冲低, 上升沿就绪; 0=反之 */
    if (adis_read_reg16(ADIS_REG_MSC_CTRL, &msc_ctrl) != RT_EOK)
        msc_ctrl = 0x00C1;              /* 读取失败按默认值处理 */

    if (msc_ctrl & 0x0001)
    {
        rt_pin_mode(ADIS16505_DR_PIN, PIN_MODE_INPUT_PULLUP);
        irq_mode = PIN_IRQ_MODE_RISING;
    }
    else
    {
        rt_pin_mode(ADIS16505_DR_PIN, PIN_MODE_INPUT_PULLDOWN);
        irq_mode = PIN_IRQ_MODE_FALLING;
    }

    /* CubeMX 生成的 SPI1 DMA 为 HALFWORD 对齐, 本链路按字节搬 22B 帧 */
    dma_rx = adis_dev.hspi->hdmarx;
    dma_tx = adis_dev.hspi->hdmatx;
    dma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_rx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    dma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_tx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    if (HAL_DMA_Init(dma_rx) != HAL_OK || HAL_DMA_Init(dma_tx) != HAL_OK)
        return -RT_ERROR;

    /* burst 命令帧固定, 填好并冲刷一次 cache (DMA 只读) */
    rt_memset(adis_tx_buf, 0, sizeof(adis_tx_buf));
    adis_tx_buf[0] = 0x68;
    adis_tx_buf[1] = 0x00;
    rt_hw_cpu_dcache_ops(RT_HW_CACHE_FLUSH, adis_tx_buf, ADIS_DMA_BUF_SIZE);

    adis_dev.q_head  = 0;
    adis_dev.q_tail  = 0;
    adis_dev.cur_slot = 0;
    adis_dev.dma_busy = RT_FALSE;
    adis_dev.cntr_init = RT_FALSE;
    rt_sem_init(&adis_dev.proc_sem, "adispr", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&adis_dev.xfer_lock, "adisx", RT_IPC_FLAG_FIFO);

    if (rt_pin_attach_irq(ADIS16505_DR_PIN, irq_mode, adis_dr_isr, RT_NULL) != RT_EOK)
        return -RT_ERROR;
    if (rt_pin_irq_enable(ADIS16505_DR_PIN, PIN_IRQ_ENABLE) != RT_EOK)
        return -RT_ERROR;
    HAL_NVIC_SetPriority(EXTI4_IRQn, ADIS16505_DR_IRQ_PRIO, 0);

    adis_dev.dr_thread = rt_thread_create("adis_dr", adis_dr_thread_entry, RT_NULL,
                                          ADIS16505_DR_THREAD_STACK,
                                          ADIS16505_DR_THREAD_PRIO,
                                          ADIS16505_DR_THREAD_TICK);
    if (adis_dev.dr_thread == RT_NULL)
    {
        rt_pin_irq_enable(ADIS16505_DR_PIN, PIN_IRQ_DISABLE);
        return -RT_ENOMEM;
    }
    rt_thread_startup(adis_dev.dr_thread);

    /* 先开链路开关再使能中断, 避免 ISR 在状态未就绪时进入 */
    adis_dev.chain_on = RT_TRUE;
    adis_dev.dr_mode = RT_TRUE;
    return RT_EOK;
}

/* ------------------------- 轮询/自恢复路径 ------------------------- */

/* 轮询 burst: HAL 直连 (链路挂起互斥), 新样本入库, 失败立即重试一次 */
static rt_err_t adis_poll_burst(struct adis16505_snapshot *out)
{
    rt_uint8_t rx[ADIS_BURST_TOTAL];
    rt_err_t ret;
    rt_uint8_t i;

    rt_mutex_take(&adis_dev.xfer_lock, RT_WAITING_FOREVER);
    ret = adis_chain_suspend();
    if (ret != RT_EOK)
    {
        rt_mutex_release(&adis_dev.xfer_lock);
        return ret;
    }

    for (i = 0; i < 2; i++)
    {
        if (adis_hal_xfer(adis_tx_buf, rx, ADIS_BURST_TOTAL) == RT_EOK &&
            adis_frame_valid(rx))
        {
            adis_frame_parse(rx, timebase_now_us(), out);
            adis_snapshot_commit(out, RT_FALSE);
            adis_chain_resume();
            rt_mutex_release(&adis_dev.xfer_lock);
            return RT_EOK;
        }
    }
    adis_chain_resume();
    rt_mutex_release(&adis_dev.xfer_lock);
    return -RT_EIO;
}

/*
 * 取最新采样:
 *  - DR 模式: 快照由 DMA 链路持续刷新, 直接拷贝;
 *    DR 停止时处理线程看门狗轮询 burst 自恢复。
 *  - 轮询模式: 同一系统 tick 内多个传感器共享一次 burst。
 */
static rt_err_t adis_update_data(struct adis16505_snapshot *out)
{
    if (adis_dev.dr_mode)
    {
        if (adis_dev.sample_valid)
        {
            adis_sample_load(out);
            return RT_EOK;
        }
        /* 快照尚未产生过: 走下面的轮询 burst */
    }
    else if (adis_dev.sample_valid && rt_tick_get() == adis_dev.last_tick)
    {
        adis_sample_load(out);
        return RT_EOK;
    }

    if (adis_poll_burst(out) != RT_EOK)
        return -RT_EIO;

    adis_dev.last_tick = rt_tick_get();
    return RT_EOK;
}

/* 取最近一次采样快照 (供 AHRS/EKF 等消费方) */
rt_err_t adis16505_get_snapshot(struct adis16505_snapshot *snap)
{
    if (snap == RT_NULL)
        return -RT_EINVAL;

    if (!adis_dev.sample_valid)
        return -RT_EEMPTY;

    adis_sample_load(snap);
    return RT_EOK;
}

/* 陀螺当前灵敏度 mdps/LSB (给数据缓冲层做单位换算, 避免各处复制量程表) */
float adis16505_gyro_lsb_mdps(void)
{
    return gyro_ranges[adis_dev.gyro_idx].lsb_mdps;
}

/* DR->DMA 链路健康统计 */
void adis16505_get_dr_stats(struct adis16505_dr_stats *st)
{
    rt_base_t level;

    if (st == RT_NULL)
        return;

    level = rt_hw_interrupt_disable();
    *st = adis_dev.stats;
    rt_hw_interrupt_enable(level);
}

/* ------------------------- 传感器框架回调 ------------------------- */

static rt_ssize_t adis_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    struct adis16505_snapshot sample;

    if (data == RT_NULL || len == 0)
        return 0;

    if (adis_update_data(&sample) != RT_EOK)
        return 0;

    data->type = sensor->info.type;
    data->timestamp = (rt_uint32_t)(sample.t_event_us / 1000u);  /* T_MCU ms */

    switch (sensor->info.type)
    {
    case RT_SENSOR_CLASS_ACCE:
        data->data.acce.x = (rt_int32_t)(sample.acce[0] * ADIS_ACCE_LSB_MG);
        data->data.acce.y = (rt_int32_t)(sample.acce[1] * ADIS_ACCE_LSB_MG);
        data->data.acce.z = (rt_int32_t)(sample.acce[2] * ADIS_ACCE_LSB_MG);
        break;
    case RT_SENSOR_CLASS_GYRO:
    {
        float lsb = gyro_ranges[adis_dev.gyro_idx].lsb_mdps;
        data->data.gyro.x = (rt_int32_t)(sample.gyro[0] * lsb);
        data->data.gyro.y = (rt_int32_t)(sample.gyro[1] * lsb);
        data->data.gyro.z = (rt_int32_t)(sample.gyro[2] * lsb);
        break;
    }
    case RT_SENSOR_CLASS_TEMP:
        /* 1 LSB = 0.1°C = 1 d°C */
        data->data.temp = sample.temp;
        break;
    default:
        return 0;
    }

    return 1;
}

/* 设置输出数据率: 2000/(DEC_RATE+1) Hz, 写后回读确认 */
static rt_err_t adis_set_odr(rt_uint32_t odr)
{
    rt_uint32_t dec, i;

    if (odr == 0)
        return -RT_EINVAL;
    dec = (2000u + odr / 2u) / odr;   /* 四舍五入: 向下取整在 700Hz 档
                                       * 会取 dec=2 实得 1000Hz (+43%) */
    if (dec < 1)
        dec = 1;
    if (dec > 2000)
        dec = 2000;
    for (i = 0; i < 3; i++)             /* 最多尝试 3 次写+回读 */
    {
        if (adis_write_reg16(ADIS_REG_DEC_RATE, (rt_uint16_t)(dec - 1)) != RT_EOK)
            return -RT_EIO;
        rt_hw_us_delay(30);             /* 数据手册: DEC_RATE 更新 30us */
        if (adis_read_reg16(ADIS_REG_DEC_RATE, &adis_dev.dec_rate) != RT_EOK)
            return -RT_EIO;
        if (adis_dev.dec_rate == dec - 1)
            return RT_EOK;
    }
    return -RT_ERROR;
}

static rt_err_t adis_control(struct rt_sensor_device *sensor, int cmd, void *arg)
{
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        if (arg)
            *(rt_uint8_t *)arg = (rt_uint8_t)adis_dev.prod_id;  /* 框架按 u8 传递, 16505 -> 0x79 */
        return RT_EOK;
    case RT_SENSOR_CTRL_SET_ODR:
        return adis_set_odr((rt_uint32_t)arg & 0xFFFF);
    case RT_SENSOR_CTRL_SET_MODE:
        /* 轮询语义: 数据始终由 DR 中断驱动刷新, 读取即取最新快照 */
        if ((rt_uint32_t)arg == RT_SENSOR_MODE_POLLING)
            return RT_EOK;
        return -RT_ERROR;
    case RT_SENSOR_CTRL_SET_POWER:
        if ((rt_uint32_t)arg == RT_SENSOR_POWER_NORMAL)
            return RT_EOK;
        return -RT_ERROR;               /* ADIS16505 无运行时休眠命令 */
    case RT_SENSOR_CTRL_SELF_TEST:
    {
        rt_uint16_t diag = 0;

        if (adis_write_reg16(ADIS_REG_GLOB_CMD, ADIS_GLOB_CMD_SELF_TEST) != RT_EOK)
            return -RT_EIO;
        rt_thread_mdelay(ADIS_SELF_TEST_DELAY_MS);
        if (adis_read_reg16(ADIS_REG_DIAG_STAT, &diag) != RT_EOK)
            return -RT_EIO;
        return (diag & 0x07FE) ? -RT_ERROR : RT_EOK;
    }
    default:
        return -RT_EINVAL;
    }
}

static struct rt_sensor_ops adis_ops =
{
    .fetch_data = adis_fetch_data,
    .control    = adis_control,
};

/* ------------------------- 初始化与注册 ------------------------- */

static rt_err_t adis_register_sensors(void)
{
    struct rt_sensor_config cfg = {0};
    rt_uint8_t i;

    if (sensor_module.lock)
        return RT_EOK;                  /* 已初始化 */

    cfg.intf.type = RT_SENSOR_INTF_SPI;
    cfg.intf.dev_name = ADIS16505_SPI_DEVICE_NAME;
    cfg.irq_pin.pin = RT_PIN_NONE;      /* DR 中断由驱动内部管理 (PA4/EXTI4) */
    cfg.mode = RT_SENSOR_MODE_POLLING;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    cfg.odr = 2000 / (adis_dev.dec_rate + 1);

    sensor_module.lock = rt_mutex_create("adismd", RT_IPC_FLAG_PRIO);
    if (sensor_module.lock == RT_NULL)
        return -RT_ENOMEM;

    for (i = 0; i < 3; i++)
    {
        struct rt_sensor_device *sen = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sen == RT_NULL)
            goto __fail;

        sen->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;   /* ADI 未在框架枚举中 */
        sen->info.model      = "adis16505";
        sen->info.intf_type  = RT_SENSOR_INTF_SPI;
        sen->info.period_min = 1;
        sen->info.fifo_max   = 0;
        sen->config          = cfg;
        sen->ops             = &adis_ops;
        sen->module          = &sensor_module;

        switch (i)
        {
        case 0: /* 加速度计 ±78.3 m/s² ≈ ±8g */
            sen->info.type      = RT_SENSOR_CLASS_ACCE;
            sen->info.unit      = RT_SENSOR_UNIT_MG;
            sen->info.range_max = ADIS_ACCE_RANGE_MG;
            sen->info.range_min = -ADIS_ACCE_RANGE_MG;
            break;
        case 1: /* 陀螺仪 量程随型号 */
            sen->info.type      = RT_SENSOR_CLASS_GYRO;
            sen->info.unit      = RT_SENSOR_UNIT_MDPS;
            sen->info.range_max = gyro_ranges[adis_dev.gyro_idx].range_mdps;
            sen->info.range_min = -gyro_ranges[adis_dev.gyro_idx].range_mdps;
            break;
        default: /* 温度 */
            sen->info.type      = RT_SENSOR_CLASS_TEMP;
            sen->info.unit      = RT_SENSOR_UNIT_DCELSIUS;
            sen->info.range_max = 850;
            sen->info.range_min = -400;
            break;
        }

        if (rt_hw_sensor_register(sen, "adis16505", RT_DEVICE_FLAG_RDONLY, RT_NULL) != RT_EOK)
        {
            rt_free(sen);
            goto __fail;
        }
        sensor_module.sen[sensor_module.sen_num++] = sen;
    }

    return RT_EOK;

__fail:
    while (sensor_module.sen_num > 0)
    {
        struct rt_sensor_device *sen = sensor_module.sen[--sensor_module.sen_num];
        rt_device_unregister(&sen->parent);
        rt_free(sen);
    }
    rt_mutex_delete(sensor_module.lock);
    sensor_module.lock = RT_NULL;
    return -RT_ERROR;
}

/* 总线一次性准备 (复位引脚 + attach + configure + MasterKeepIOState):
 * attach/configure 不可重入 (重复 attach 同名设备失败), 重试路径复用
 * adis_dev.spi/hspi 直接探测, 仅重打 RST 脉冲 */
static rt_bool_t adis_bus_ready;

static rt_err_t adis_prepare_bus(void)
{
    struct rt_spi_configuration spi_cfg =
    {
        .mode = RT_SPI_MODE_3 | RT_SPI_MSB,
        .data_width = 8,
        .max_hz = ADIS16505_SPI_MAX_HZ,
    };

    if (adis_bus_ready)
        return RT_EOK;

    /* 复位引脚: 产生真实复位脉冲 (低≥10ms) 后回高, 等待启动 */
    rt_pin_mode(ADIS16505_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ADIS16505_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(10);
    rt_pin_write(ADIS16505_RST_PIN, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(ADIS16505_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(ADIS16505_RESET_DELAY_MS);

    /* 挂载 SPI 设备 (CS=PC4) 并配置模式3/8bit/954.861kHz */
    if (rt_hw_spi_device_attach(ADIS16505_SPI_BUS_NAME, ADIS16505_SPI_DEVICE_NAME,
                                ADIS16505_CS_PIN) != RT_EOK)
    {
        LOG_E("SPI device attach failed");
        return -RT_ERROR;
    }
    adis_dev.spi = (struct rt_spi_device *)rt_device_find(ADIS16505_SPI_DEVICE_NAME);
    if (adis_dev.spi == RT_NULL)
    {
        LOG_E("SPI device %s not found", ADIS16505_SPI_DEVICE_NAME);
        return -RT_ERROR;
    }
    rt_spi_configure(adis_dev.spi, &spi_cfg);

    /*
     * MasterKeepIOState (HAL 每次传输结束关闭外设时不释放引脚, 防 CS 仍
     * 为低时 SCLK/MOSI 毛刺导致帧错位) 由 drv_spi 框架在 stm32_spi_init
     * 内对 H7 统一设置, 此处严禁再调 HAL_SPI_Init: 每次 HAL_SPI_Init 都
     * 会重跑 CubeMX 的 HAL_SPI_MspInit, 把 hdmarx/hdmatx 重新 LINKDMA 到
     * board.c 的 CubeMX 全局句柄, 而 DMA 完成中断 (drv_spi.c 的
     * DMA1_Stream0/1_IRQHandler) 派发的是 BSP 自有句柄 —— 句柄错位后
     * 完成回调 (含本驱动的 txrx_cplt_hook) 永不触发, DR->DMA 链路死锁
     * (2026-10-01 SPI 总线移植后踩中, 移植前 BSP 未自带 DMA 句柄)。
     */
    {
        struct stm32_spi *spi_drv = rt_container_of(adis_dev.spi->bus, struct stm32_spi, spi_bus);
        adis_dev.hspi = &spi_drv->handle;   /* 之后驱动直连 HAL, 不经框架传输 */
    }

    adis_bus_ready = RT_TRUE;
    return RT_EOK;
}

/* 每次探测尝试前重打真实复位脉冲 (让重试对芯片状态自愈) */
static void adis_hw_reset_pulse(void)
{
    rt_pin_write(ADIS16505_RST_PIN, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(ADIS16505_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(ADIS16505_RESET_DELAY_MS);
}

int rt_hw_adis16505_init(void)
{
    rt_uint16_t reg = 0;
    rt_err_t last_err = -RT_ERROR;
    rt_uint8_t retry;

    if (adis_prepare_bus() != RT_EOK)
        return -RT_ERROR;
    adis_hw_reset_pulse();

    /* PROD_ID 校验 (0x72, 期望 16505 = 0x4079)。区分两种失败: 框架传输
     * 报错 (rt_spi_transfer 路径问题) vs 传输成功但数据不符 (读 0x0000 =
     * MISO 无数据, 器件/线路侧) —— 2026-10-01 排查时旧日志无法区分 */
    for (retry = 0; retry < 3; retry++)
    {
        last_err = adis_read_reg16(0x72, &adis_dev.prod_id);
        if (last_err == RT_EOK && adis_dev.prod_id == ADIS16505_PROD_ID)
            break;
        rt_thread_mdelay(20);
    }
    if (adis_dev.prod_id != ADIS16505_PROD_ID)
    {
        if (last_err != RT_EOK)
            LOG_E("ADIS16505 probe xfer failed (err=%d), SPI framework path",
                  last_err);
        else
            LOG_E("ADIS16505 not found (PROD_ID=0x%04X), check SPI1 wiring PA5/PA6/PA7, CS=PC4",
                  adis_dev.prod_id);
        return -RT_ERROR;
    }

    /* 识别陀螺量程型号 RANG_MDL[3:2] */
    if (adis_read_reg16(ADIS_REG_RANG_MDL, &reg) == RT_EOK)
        adis_dev.gyro_idx = (reg >> 2) & 0x03;
    else
        adis_dev.gyro_idx = 1;          /* 识别失败按 -2 型处理 */

    /* 缓存内部 flash 零偏校正寄存器 (须在 DR burst 启动前读, 运行期
     * 寄存器读路径与突发并发不可靠; FinSH `adisbias` 打印缓存) */
    {
        static const rt_uint8_t greg[6] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
        static const rt_uint8_t areg[6] = {0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B};
        int i;

        s_bias_cache.valid = RT_TRUE;
        adis_read_reg16(0x02, &s_bias_cache.diag);
        adis_read_reg16(0x5C, &s_bias_cache.filt);
        s_bias_cache.rang = reg;
        for (i = 0; i < 3; i++)
        {
            adis_read_reg16(greg[2 * i],     &s_bias_cache.g_lo[i]);
            adis_read_reg16(greg[2 * i + 1], &s_bias_cache.g_hi[i]);
            adis_read_reg16(areg[2 * i],     &s_bias_cache.a_lo[i]);
            adis_read_reg16(areg[2 * i + 1], &s_bias_cache.a_hi[i]);
        }
    }

    /* 默认输出数据率 (DEC_RATE, Excel 清单 1000Hz) */
    if (adis_set_odr(ADIS16505_DEFAULT_ODR) != RT_EOK)
        adis_dev.dec_rate = 0;          /* 写失败时保持芯片默认 2000Hz */

    /* 启动 DR 中断 -> DMA burst 链路 (失败不阻塞注册, 退化为轮询 burst) */
    if (adis_dr_start() != RT_EOK)
        LOG_W("DR DMA chain setup failed, fallback to polling burst");

    LOG_I("ADIS16505 found, PROD_ID=%d, gyro range ±%d dps, ODR=%d Hz, DR->DMA %s",
          adis_dev.prod_id,
          gyro_ranges[adis_dev.gyro_idx].range_mdps / 1000,
          2000 / (adis_dev.dec_rate + 1),
          adis_dev.dr_mode ? "on" : "off");

    return (adis_register_sensors() == RT_EOK) ? 0 : -RT_ERROR;
}

/*
 * 上电失败无界重试 (2026-10-01, 对齐 BMP585/BMM350 模式): 旧实现 init
 * 一次失败即永久掉线 (imu_data 无设备可开), 板级复位纹波/上电时序造成
 * 的偶发探测失败无自愈路径。重试只重打 RST 脉冲 + 重探测 (总线 attach
 * 一次), 10s 周期; imu_data 侧已同步改为惰性打开, 迟到上线即自动接入。
 */
#define ADIS_INIT_RETRY_MS       10000
#define ADIS_RETRY_THREAD_STACK  3072    /* ulog 格式化尖峰余量 */

static void adis_retry_entry(void *parameter)
{
    rt_uint32_t i = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_thread_mdelay(ADIS_INIT_RETRY_MS);
        i++;
        if (rt_hw_adis16505_init() == 0)
        {
            LOG_I("ADIS16505 online after deferred retry %u", i);
            return;
        }
        if (i % 30u == 0u)
            LOG_W("ADIS16505 still not ready after %u retries (%us), keep trying",
                  i, (rt_uint32_t)(i * ADIS_INIT_RETRY_MS / 1000));
    }
}

static int rt_hw_adis16505_init_and_retry(void)
{
    if (rt_hw_adis16505_init() == 0)
        return 0;

    {
        rt_thread_t retry = rt_thread_create("adisretry", adis_retry_entry, RT_NULL,
                                             ADIS_RETRY_THREAD_STACK, 20, 10);
        if (retry != RT_NULL)
        {
            rt_thread_startup(retry);
            LOG_W("ADIS16505 init failed, background retry every %ds",
                  ADIS_INIT_RETRY_MS / 1000);
        }
    }
    return -RT_ERROR;
}
INIT_DEVICE_EXPORT(rt_hw_adis16505_init_and_retry);

/* ------------------------- FinSH 诊断命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include <string.h>

/* 探测失败取证 (2026-10-01): 区分 "框架传输报错" / "传输成功但 MISO 全零
 * /数据错位" / "偶发可自愈", 并可绕过 rt_spi 框架用 HAL 直连交叉验证 */
static void adisdbg(int argc, char **argv)
{
    rt_uint8_t i;

    /* 总是先保证总线就绪 (adisdbg 可在驱动 init 失败后调用) */
    if (adis_prepare_bus() != RT_EOK)
    {
        rt_kprintf("bus prepare failed\n");
        return;
    }

    rt_kprintf("dev: prod_id=0x%04X dr_mode=%d dec=%d gyro_idx=%d bus_ready=%d\n",
               adis_dev.prod_id, adis_dev.dr_mode, adis_dev.dec_rate,
               adis_dev.gyro_idx, adis_bus_ready);
    rt_kprintf("pin: CS(PC4)=%d RST(PC5)=%d DR=%d\n",
               rt_pin_read(ADIS16505_CS_PIN), rt_pin_read(ADIS16505_RST_PIN),
               rt_pin_read(ADIS16505_DR_PIN));
    if (adis_dev.hspi != RT_NULL)
    {
        SPI_TypeDef *spix = adis_dev.hspi->Instance;
        rt_kprintf("spi: CR1=0x%08X CFG1=0x%08X CFG2=0x%08X SR=0x%08X\n",
                   spix->CR1, spix->CFG1, spix->CFG2, spix->SR);
    }

    if (argc >= 2 && strcmp(argv[1], "probe") == 0)
    {
        /* 框架路径探测 8 轮 (每轮含 RST 复位), 打印每轮返回值与原始读数 */
        for (i = 0; i < 8; i++)
        {
            rt_uint16_t id = 0;
            rt_err_t err;

            if (i == 4)
                adis_hw_reset_pulse();
            err = adis_read_reg16(0x72, &id);
            rt_kprintf("[%d] framework read: err=%d PROD_ID=0x%04X%s\n",
                       i, err, id, (id == ADIS16505_PROD_ID) ? " OK" : "");
            rt_thread_mdelay(100);
        }
    }

    if (argc >= 2 && strcmp(argv[1], "raw") == 0)
    {
        /* HAL 直连 (绕过 rt_spi 框架) 读 PROD_ID: 手动 CS + 两帧轮询传输。
         * 框架失败而 raw 成功 -> 框架路径问题; raw 也全零 -> 器件/线路侧 */
        for (i = 0; i < 4; i++)
        {
            rt_uint8_t cmd[2] = {0x72, 0x00};
            rt_uint8_t zero[2] = {0x00, 0x00};
            rt_uint8_t rx[2];
            rt_err_t e1, e2;

            adis_cs_low();
            e1 = adis_hal_xfer(cmd, rx, 2);
            rt_hw_us_delay(ADIS_STALL_US);
            e2 = adis_hal_xfer(zero, rx, 2);
            rt_hw_us_delay(ADIS_STALL_US);
            adis_cs_high();
            rt_kprintf("[%d] raw read: e1=%d e2=%d rx=0x%02X%02X%s\n",
                       i, e1, e2, rx[0], rx[1],
                       (((rx[0] << 8) | rx[1]) == ADIS16505_PROD_ID) ? " OK" : "");
            rt_thread_mdelay(100);
        }
    }

    if (argc >= 2 && strcmp(argv[1], "reg") == 0)
    {
        /* 裸寄存器读 (2026-10-01 决定性取证): 完全绕开 rt_spi 框架与 HAL,
         * CPU 直接操作 SPI1 寄存器, 微秒级时序复刻驱动读法 (帧间 stall)。
         * 读到 0x4079 -> 芯片在线, 问题在框架/HAL 层; 仍为零 -> MISO 线上
         * 无数据, 硬件层。 */
        SPI_TypeDef *spix = adis_dev.hspi->Instance;
        rt_uint8_t round;

        for (round = 0; round < 4; round++)
        {
            rt_uint8_t rx[4];
            rt_uint32_t t0;
            int i, got = 0;

            if (round == 2)
                adis_hw_reset_pulse();

            /* SPE=0 下配 TSIZE=2, 清标志 */
            CLEAR_BIT(spix->CR1, SPI_CR1_SPE);
            spix->CR2 = 2;
            spix->IFCR = 0xFFFFFFFF;
            SET_BIT(spix->CR1, SPI_CR1_SPE);

            adis_cs_low();
            for (i = 0; i < 4; i++)
            {
                rt_uint8_t b = (i == 0) ? 0x72 : 0x00;

                /* 写 TXDR 后 CSTART 启动 2 字节帧; 忙等 EOT (8.4us*2 @954k) */
                WRITE_REG(spix->TXDR, b);
                WRITE_REG(spix->TXDR, 0x00);
                SET_BIT(spix->CR1, SPI_CR1_CSTART);
                t0 = 0;
                while (!(READ_REG(spix->SR) & SPI_SR_EOT))
                {
                    if (++t0 > 1000000)
                        break;
                }
                WRITE_REG(spix->IFCR, SPI_SR_EOT);

                /* 弹 RX FIFO (帧内 2 字节) */
                while ((READ_REG(spix->SR) & SPI_SR_RXP) && got < 4)
                    rx[got++] = (rt_uint8_t)READ_REG(spix->RXDR);

                /* 帧间 stall >= 16us (ADIS 要求) */
                rt_hw_us_delay(ADIS_STALL_US);
            }
            adis_cs_high();
            CLEAR_BIT(spix->CR1, SPI_CR1_SPE);

            rt_kprintf("[%d] reg read: got=%d rx=%02X %02X %02X %02X%s\n",
                       round, got, rx[0], rx[1], rx[2], rx[3],
                       (got >= 4 && rx[2] == 0x40 && rx[3] == 0x79) ? "  <-- PROD_ID OK!" : "");
        }
    }

    if (argc >= 2 && strcmp(argv[1], "reset") == 0)
    {
        rt_kprintf("[RST pulse + re-probe]\n");
        adis_hw_reset_pulse();
        rt_kprintf("init -> %d\n", rt_hw_adis16505_init());
    }
}
MSH_CMD_EXPORT(adisdbg, ADIS16505 diag: [probe|raw|reg|reset]);

/* 取证命令 (2026-10-01): 打印上电期缓存的内部 flash 零偏校正寄存器。
 * 实测本机: G_BIAS 全零 (固有零偏非编程偏移), RANG_MDL=±500dps (标度
 * 无误), DIAG_STAT 干净 —— 零偏来自传感器本身, 在 0.2%FS 重复性规格内。
 * 换算在主机侧做 (tiny klibc 无 %f) */
static void adisbias(int argc, char **argv)
{
    static const char *ax[3] = {"X", "Y", "Z"};
    int i;

    RT_UNUSED(argc);
    RT_UNUSED(argv);

    if (!s_bias_cache.valid)
    {
        rt_kprintf("bias cache not captured (init not run?)\n");
        return;
    }

    rt_kprintf("DIAG_STAT=0x%04X FILT_CTRL=0x%04X RANG_MDL=0x%04X\n",
               s_bias_cache.diag, s_bias_cache.filt, s_bias_cache.rang);
    for (i = 0; i < 3; i++)
    {
        rt_int32_t gw = (rt_int32_t)(((rt_uint32_t)s_bias_cache.g_hi[i] << 16) |
                                     s_bias_cache.g_lo[i]);
        rt_int32_t aw = (rt_int32_t)(((rt_uint32_t)s_bias_cache.a_hi[i] << 16) |
                                     s_bias_cache.a_lo[i]);

        rt_kprintf("%s: G_BIAS=0x%08X (%d LSB)  A_BIAS=0x%08X (%d LSB)\n",
                   ax[i], (unsigned)gw, (int)gw, (unsigned)aw, (int)aw);
    }
    rt_kprintf("(gyro correction scale 1/2^15 deg/s/LSB; accel scale "
               "unconfirmed; host-side convert)\n");
}
MSH_CMD_EXPORT(adisbias, ADIS16505 internal bias registers (cached at boot));
#endif /* RT_USING_FINSH && FINSH_USING_MSH */
