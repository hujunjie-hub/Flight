/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMP585 气压计 RT-Thread 传感器驱动实现 (I2C4: SCL=PB6/SDA=PB9, INT PE1;
 *           2026-10-04 由 I2C2/PB10/PB11 迁出, I2C2 让位电流计)
 *
 * 寄存器定义与数据格式依据 Bosch BMP585 数据手册 (BST-BMP585-DS003-02):
 *  - TEMP_DATA  (0x1D..0x1F) 小端 24-bit, 单位 (signed,24,16) °C
 *  - PRESS_DATA (0x20..0x22) 小端 24-bit, 单位 (signed,24,6)  Pa
 *  - OSR_CONFIG (0x36): [6] press_en, [5:3] osr_p, [2:0] osr_t
 *  - ODR_CONFIG (0x37): [7] deep_dis, [6:2] odr, [1:0] pwr_mode
 *    pwr_mode: 0=待机 1=正常 2=单次强制 3=连续
 *  - 软复位后 STATUS(0x28)[1] nvm_rdy 置位方可继续配置
 *
 * ---------------------------------------------------------------------------
 * 2026-09-17 上板接线诊断 (三引脚 PB10/PB11/PE13):
 *  - CubeMX 配置正确: PB10=I2C2_SCL / PB11=I2C2_SDA (AF4 开漏), PE13=EXTI13
 *    上升沿 (BMP585 未探测成功前驱动不挂 EXTI, 保持复位默认模拟态属正常)。
 *  - I2C2 总线无外部上拉: PB10/PB11 悬空 (PUPDR=00, 静态读 0), 开漏总线
 *    无法回高, 探测恒失败。固件临时启用内部上拉 (~40k, 见
 *    bmp_i2c2_pullup_boot) 后总线 100% ACK, CHIP_ID=0x51 读到真值。
 *    正式装机必须焊 2.2~4.7kΩ 外部上拉到 3V3 并删除该应急函数。
 *  - 芯片 STATUS=0x01 (nvm_err=1, nvm_rdy 恒 0): NVM 加载失败。该标志
 *    只在掉电重启 (POR) 后清除 —— 需完整断电重上电复测; 上电后仍为
 *    0x01 则检查模块 VDD 供电 (NVM 加载电流大, 欠压即卡死), 供电正常
 *    仍报错多为模块 NVM 损坏需更换。
 * -------------------------------------------------------------------------
 * 2026-09-29 SWD 手动 I2C 复测 (硬件正式接好, 内部上拉方案不变):
 *  1. 总线/传感器全部正常: hwi2c2 就绪, CHIP_ID=0x51, PB10/PB11 AF4
 *     开漏 + 内部上拉生效; 手动配置 OSR/ODR/normal 后压强读数
 *     1002.90 hPa 稳定复现、温度 35.9°C —— 测量功能完好。
 *  2. 根因修正: 上述 "nvm_err=1" 解读有误。实测 STATUS 常读 0x01
 *     (bit0=1 常数, bit1 nvm_rdy=0 常态、偶发 1), 而 nvm_err(bit2)
 *     恒 0。旧驱动把 "100ms 内 nvm_rdy=1" 当硬门禁, 在本模块上必现
 *     软复位超时 -> 初始化失败 -> baro_data 找不到设备 (链路 dead)。
 *     属驱动 bug, 非硬件故障。修复: nvm_err 才硬失败, nvm_rdy 尽力
 *     等待 (LOG_W 放行), 入正常模式后增加压强合理性自检 (含一次整段
 *     配置重试) 兜底 NVM 异常。
 * * -------------------------------------------------------------------------
 * 2026-10-04 硬件迁移: I2C2(PB10/PB11, INT PE13) -> I2C4(PB6/PB9, INT PE1),
 *  I2C2 让位电流计。驱动与总线无耦合 (设备名 hwi2c4 见 sensor_bmp585.h),
 *  下方 2026-09 诊断记录为 I2C2 时期历史, 引脚描述不再适用。
 * -------------------------------------------------------------------------
 */

#include "sensor_bmp585.h"
#include <drivers/sensor.h>
#include "timebase.h"               /* fetch 打戳 (T_MCU ms, 同 ADIS 惯例) */

#define LOG_TAG "sensor.bmp585"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 寄存器定义 ------------------------- */
#define BMP585_REG_CHIP_ID          0x01    /* = 0x51 */
#define BMP585_REG_CHIP_STATUS      0x11
#define BMP585_REG_INT_CONFIG       0x14
#define BMP585_REG_INT_SOURCE       0x15
#define BMP585_REG_INT_STATUS       0x27    /* [0] 数据就绪 */
#define BMP585_REG_STATUS           0x28    /* [1] NVM 就绪 */
#define BMP585_REG_TEMP_DATA_XLSB   0x1D    /* 温度 3 字节 + 压强 3 字节连续 */
#define BMP585_REG_DSP_CONFIG       0x30
#define BMP585_REG_DSP_IIR          0x31    /* [2:0] set_iir_t, [5:3] set_iir_p */
#define BMP585_REG_OSR_CONFIG       0x36
#define BMP585_REG_ODR_CONFIG       0x37
#define BMP585_REG_CMD              0x7E    /* 0xB6 软复位 */

#define BMP585_CHIP_ID_VAL          0x51
#define BMP585_CMD_SOFT_RESET       0xB6

/* 电源模式 (ODR_CONFIG[1:0]) */
#define BMP585_PWR_STANDBY          0x00
#define BMP585_PWR_NORMAL           0x01
#define BMP585_PWR_FORCED           0x02
#define BMP585_PWR_CONTINUOUS       0x03

/* NVM 就绪等待 (尽力而为, 见 bmp_soft_reset 注释) */
#define BMP585_NVM_TIMEOUT_MS       100

/* 上电自检: 压强合理范围 (Pa), 数据手册量程 300~1250 hPa */
#define BMP585_SANITY_PA_MIN        30000
#define BMP585_SANITY_PA_MAX        125000

/* ------------------------- 设备结构 ------------------------- */
struct bmp585_device
{
    struct rt_i2c_bus_device *bus;
    rt_uint8_t  addr;
    rt_uint8_t  chip_id;
    rt_uint8_t  odr_code;               /* 当前 ODR 档位 (0x00~0x1F) */
    rt_bool_t   powered;
};

static struct bmp585_device bmp_dev;
static struct rt_sensor_module sensor_module;   /* baro/temp 共用一个模块 */

/* 运行期健康监测分级计数 (见 bmp_fetch_data): I2C 传输失败 = 硬故障
 * (掉线/欠压回挂起态), 连续 20 次 (200ms@100Hz) 即重配置; 压强超界
 * (饱和垃圾) = 软信号, 维持 200 样本 (2s) 窗口防误判 */
static rt_uint16_t bmp_bad_cnt;
static rt_uint16_t bmp_nak_cnt;
#define BMP585_FETCH_BAD_RECFG_N  200    /* 连续压强超界触发重配置 (100Hz x 2s) */
#define BMP585_FETCH_NAK_RECFG_N  20     /* 连续 I2C 失败触发重配置 (100Hz x 200ms) */

static rt_err_t bmp_config_and_selftest(void);   /* 前置: fetch 重配置用 */

/* ------------------------- I2C 基础读写 ------------------------- */

static rt_err_t bmp_read_regs(rt_uint8_t reg, rt_uint8_t *data, rt_uint16_t len)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = bmp_dev.addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = bmp_dev.addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = data;
    msgs[1].len   = len;

    return (rt_i2c_transfer(bmp_dev.bus, msgs, 2) == 2) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmp_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;

    msg.addr  = bmp_dev.addr;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = 2;

    return (rt_i2c_transfer(bmp_dev.bus, &msg, 1) == 1) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmp_read_reg(rt_uint8_t reg, rt_uint8_t *val)
{
    return bmp_read_regs(reg, val, 1);
}

/* ------------------------- 配置 ------------------------- */

/* ODR 档位表 (ODR_CONFIG[6:2], 数据手册表 8.34) */
static const struct
{
    rt_uint16_t hz_x10;                 /* ×10 以表达 0.5/0.25/0.125Hz 档 */
    rt_uint8_t  code;
} bmp_odr_table[] =
{
    {2400, 0x00}, {2185, 0x01}, {1991, 0x02}, {1792, 0x03},
    {1600, 0x04}, {1493, 0x05}, {1400, 0x06}, {1298, 0x07},
    {1200, 0x08}, {1101, 0x09}, {1002, 0x0A}, { 896, 0x0B},
    { 800, 0x0C}, { 700, 0x0D}, { 600, 0x0E}, { 500, 0x0F},
    { 450, 0x10}, { 400, 0x11}, { 350, 0x12}, { 300, 0x13},
    { 250, 0x14}, { 200, 0x15}, { 150, 0x16}, { 100, 0x17},
    {  50, 0x18}, {  40, 0x19}, {  30, 0x1A}, {  20, 0x1B},
    {  10, 0x1C}, {   5, 0x1D}, {   2, 0x1E}, {   1, 0x1F},
};

/* 最近 ODR 档位 (输入 Hz ×10) */
static rt_uint8_t bmp_odr_code_from_hz_x10(rt_uint32_t hz_x10)
{
    rt_uint32_t best_err = 0xFFFFFFFF;
    rt_uint8_t best = 0x17;
    rt_uint32_t i;

    for (i = 0; i < sizeof(bmp_odr_table) / sizeof(bmp_odr_table[0]); i++)
    {
        rt_uint32_t err = (bmp_odr_table[i].hz_x10 > hz_x10) ?
                          (bmp_odr_table[i].hz_x10 - hz_x10) : (hz_x10 - bmp_odr_table[i].hz_x10);
        if (err < best_err)
        {
            best_err = err;
            best = bmp_odr_table[i].code;
        }
    }
    return best;
}

static rt_err_t bmp_set_power_mode(rt_uint8_t mode)
{
    rt_uint8_t reg;

    if (bmp_read_reg(BMP585_REG_ODR_CONFIG, &reg) != RT_EOK)
        return -RT_EIO;

    reg = (reg & ~0x03) | (mode & 0x03);
    if (bmp_write_reg(BMP585_REG_ODR_CONFIG, reg) != RT_EOK)
        return -RT_EIO;

    bmp_dev.powered = (mode != BMP585_PWR_STANDBY);
    return RT_EOK;
}

static rt_err_t bmp_set_odr_code(rt_uint8_t code)
{
    rt_uint8_t reg;

    if (bmp_read_reg(BMP585_REG_ODR_CONFIG, &reg) != RT_EOK)
        return -RT_EIO;

    reg = (reg & ~0x7C) | ((code & 0x1F) << 2);
    if (bmp_write_reg(BMP585_REG_ODR_CONFIG, reg) != RT_EOK)
        return -RT_EIO;

    bmp_dev.odr_code = code;
    return RT_EOK;
}

static rt_err_t bmp_set_osr(rt_uint8_t osr_p, rt_uint8_t osr_t)
{
    rt_uint8_t reg;

    if (osr_p > 7 || osr_t > 7)
        return -RT_EINVAL;

    reg = 0x40 | ((osr_p & 0x07) << 3) | (osr_t & 0x07);   /* press_en=1 */
    return bmp_write_reg(BMP585_REG_OSR_CONFIG, reg);
}

static rt_err_t bmp_set_iir(rt_uint8_t coeff)
{
    rt_uint8_t reg;

    if (coeff > 7)
        return -RT_EINVAL;

    reg = ((coeff & 0x07) << 3) | (coeff & 0x07);
    return bmp_write_reg(BMP585_REG_DSP_IIR, reg);
}

/* ------------------------- 数据读取 ------------------------- */

/* 读温度压强原始值 (24-bit 有符号小端) */
static rt_err_t bmp_read_raw(rt_int32_t *temp_raw, rt_int32_t *press_raw)
{
    rt_uint8_t buf[6];
    rt_uint32_t t, p;

    if (bmp_read_regs(BMP585_REG_TEMP_DATA_XLSB, buf, 6) != RT_EOK)
        return -RT_EIO;

    t = buf[0] | ((rt_uint32_t)buf[1] << 8) | ((rt_uint32_t)buf[2] << 16);
    p = buf[3] | ((rt_uint32_t)buf[4] << 8) | ((rt_uint32_t)buf[5] << 16);

    *temp_raw  = (t & 0x800000) ? (rt_int32_t)(t - 0x1000000) : (rt_int32_t)t;
    *press_raw = (p & 0x800000) ? (rt_int32_t)(p - 0x1000000) : (rt_int32_t)p;
    return RT_EOK;
}

/* ------------------------- 传感器框架回调 ------------------------- */

/* 同芯片同拍合并读缓存: baro/temp 两个框架设备各 fetch 一次, 同一测量
 * 周期内 (baro_data.c 先读气压再读温度) 第二次直接取缓存, I2C 事务
 * 减半。窗口 2ms << ODR 周期 10ms, 不会合并两个不同测量拍 */
#define BMP585_RAW_CACHE_WIN_MS  2
static rt_int32_t s_cache_temp_raw, s_cache_press_raw;
static rt_tick_t  s_cache_tick;
static rt_bool_t  s_cache_valid;

static rt_ssize_t bmp_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    rt_int32_t temp_raw, press_raw;

    if (data == RT_NULL || len == 0 || !bmp_dev.powered)
        return 0;

    {
        rt_tick_t now = rt_tick_get();

        if (s_cache_valid &&
            (now - s_cache_tick) < rt_tick_from_millisecond(BMP585_RAW_CACHE_WIN_MS))
        {
            temp_raw  = s_cache_temp_raw;
            press_raw = s_cache_press_raw;
        }
        else
        {
            if (bmp_read_raw(&temp_raw, &press_raw) != RT_EOK)
            {
                /* I2C 失败 (NAK/超时) 硬故障通道: 快速重配置 */
                if (++bmp_nak_cnt >= BMP585_FETCH_NAK_RECFG_N)
                {
                    bmp_nak_cnt = 0;
                    LOG_W("BMP585 I2C fail x%d, reconfiguring",
                          BMP585_FETCH_NAK_RECFG_N);
                    (void)bmp_config_and_selftest();
                }
                return 0;
            }
            bmp_nak_cnt = 0;
            s_cache_temp_raw = temp_raw;
            s_cache_press_raw = press_raw;
            s_cache_tick = now;
            s_cache_valid = RT_TRUE;
        }
    }

    data->type = sensor->info.type;
    data->timestamp = (rt_uint32_t)(timebase_now_us() / 1000u);  /* T_MCU ms */

    if (sensor->info.type == RT_SENSOR_CLASS_BARO)
    {
        /* (signed,24,6) Pa: 1 LSB = 1/64 Pa */
        float pa = press_raw / 64.0f;

        /* 运行期压强合理性: 欠压窗口 "配置 ACK 但不生效" 会让压强读出
         * 0x7FFFFF 饱和值 (1310hPa) —— init 期自检挡不住运行期复发,
         * 饱和垃圾会原样进入气压高度解算。超限样本丢弃并计数, 连续
         * 超限走快速重配置 (不软复位, 见 bmp_try_init 注释) */
        if (pa < (float)BMP585_SANITY_PA_MIN || pa > (float)BMP585_SANITY_PA_MAX)
        {
            if (++bmp_bad_cnt >= BMP585_FETCH_BAD_RECFG_N)
            {
                bmp_bad_cnt = 0;
                LOG_W("BMP585 pressure implausible (%.1f hPa) x%d, reconfiguring",
                      (double)(pa / 100.0f), BMP585_FETCH_BAD_RECFG_N);
                (void)bmp_config_and_selftest();  /* 失败则下轮坏样本再试 */
            }
            return 0;
        }
        bmp_bad_cnt = 0;

        data->data.baro = (rt_int32_t)(pa + (pa >= 0 ? 0.5f : -0.5f));
    }
    else    /* RT_SENSOR_CLASS_TEMP: (signed,24,16) °C -> 0.1°C */
    {
        float dc = temp_raw / 6553.6f;
        data->data.temp = (rt_int32_t)(dc + (dc >= 0 ? 0.5f : -0.5f));
    }

    return 1;
}

static rt_err_t bmp_control(struct rt_sensor_device *sensor, int cmd, void *arg)
{
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        if (arg)
            *(rt_uint8_t *)arg = bmp_dev.chip_id;
        return RT_EOK;
    case RT_SENSOR_CTRL_SET_ODR:
    {
        rt_uint32_t odr = (rt_uint32_t)arg & 0xFFFF;
        return bmp_set_odr_code(bmp_odr_code_from_hz_x10(odr * 10));
    }
    case RT_SENSOR_CTRL_SET_MODE:
        if ((rt_uint32_t)arg == RT_SENSOR_MODE_POLLING)
            return RT_EOK;
        return -RT_ERROR;               /* INT 引脚(PE1) 未使用 */
    case RT_SENSOR_CTRL_SET_POWER:
        if ((rt_uint32_t)arg == RT_SENSOR_POWER_NORMAL || (rt_uint32_t)arg == RT_SENSOR_POWER_HIGH)
            return bmp_set_power_mode(BMP585_PWR_NORMAL);
        if ((rt_uint32_t)arg == RT_SENSOR_POWER_DOWN || (rt_uint32_t)arg == RT_SENSOR_POWER_LOW)
            return bmp_set_power_mode(BMP585_PWR_STANDBY);
        return -RT_EINVAL;
    default:
        return -RT_EINVAL;
    }
}

static struct rt_sensor_ops bmp_ops =
{
    .fetch_data = bmp_fetch_data,
    .control    = bmp_control,
};

/* ------------------------- 初始化与注册 ------------------------- */

static rt_err_t bmp_register_sensors(void)
{
    struct rt_sensor_config cfg = {0};

    if (sensor_module.lock)
        return RT_EOK;                  /* 已初始化 */

    cfg.intf.type = RT_SENSOR_INTF_I2C;
    cfg.intf.dev_name = BMP585_I2C_BUS_NAME;
    cfg.irq_pin.pin = RT_PIN_NONE;      /* 轮询模式, INT(PE1) 未使用 */
    cfg.mode = RT_SENSOR_MODE_POLLING;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    cfg.odr = BMP585_DEFAULT_ODR_HZ;

    sensor_module.lock = rt_mutex_create("bmpmd", RT_IPC_FLAG_PRIO);
    if (sensor_module.lock == RT_NULL)
        return -RT_ENOMEM;

    for (rt_uint8_t i = 0; i < 2; i++)
    {
        struct rt_sensor_device *sen = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sen == RT_NULL)
            goto __fail;

        sen->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sen->info.model      = "bmp585";
        sen->info.intf_type  = RT_SENSOR_INTF_I2C;
        sen->info.period_min = 10;      /* 100Hz -> 10ms */
        sen->info.fifo_max   = 0;
        sen->config          = cfg;
        sen->ops             = &bmp_ops;
        sen->module          = &sensor_module;

        if (i == 0)                     /* 气压计 300~1250 hPa */
        {
            sen->info.type      = RT_SENSOR_CLASS_BARO;
            sen->info.unit      = RT_SENSOR_UNIT_PA;
            sen->info.range_max = 125000;
            sen->info.range_min = 30000;
        }
        else                            /* 芯片温度 */
        {
            sen->info.type      = RT_SENSOR_CLASS_TEMP;
            sen->info.unit      = RT_SENSOR_UNIT_DCELSIUS;
            sen->info.range_max = 850;
            sen->info.range_min = -400;
        }

        if (rt_hw_sensor_register(sen, "bmp585", RT_DEVICE_FLAG_RDONLY, RT_NULL) != RT_EOK)
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

/* 软复位并等待 NVM (2026-09-29 修正: 见文件头诊断记录 3)
 *
 * 本模块实测 STATUS 的 nvm_rdy(bit1) 长期读 0、仅偶发置 1, 而测量功能
 * 完全正常 (SWD 手动 I2C: 压强 1002.90hPa 稳定 / 温度 35.9°C) 且
 * nvm_err(bit2) 恒 0。把 nvm_rdy 当硬门禁会必现初始化失败。
 * 现改为: nvm_err 置位才是硬失败 (NVM 内容损坏, 数据不可信);
 * nvm_rdy 只做尽力等待 (等到了最好, 等不到放行, 由上层压强自检兜底)。 */
static rt_err_t bmp_soft_reset(void)
{
    rt_uint8_t status = 0;
    rt_uint8_t last = 0xFF;
    rt_uint8_t retry;
    rt_uint32_t acks = 0, ok = 0;

    /* 数据手册: I2C 写复位命令期间不回 ACK, 写失败属正常, 忽略结果 */
    bmp_write_reg(BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
    rt_thread_mdelay(2);

    for (retry = 0; retry < BMP585_NVM_TIMEOUT_MS; retry++)
    {
        if (bmp_read_reg(BMP585_REG_STATUS, &status) == RT_EOK)
        {
            acks++;
            last = status;
            if (status & 0x04)          /* nvm_err: 硬失败 */
            {
                LOG_E("NVM error after soft reset (status=0x%02X), "
                      "power-cycle the module; persistent error = damaged NVM", status);
                return -RT_ERROR;
            }
            if (status & 0x02)          /* nvm_rdy */
            {
                ok++;
                return RT_EOK;
            }
        }
        rt_thread_mdelay(1);
    }

    /* 未观察到 nvm_rdy 但 nvm_err=0: 放行, 压强自检兜底
     * (诊断: ACK 次数 + 最后状态值。ACK 正常但 nvm_rdy 恒 0 多为模块
     * 供电不足 (NVM 读取电流大, 欠压即卡死); 状态值乱跳则总线信号劣化) */
    LOG_W("nvm_rdy not observed after reset: ack=%u/%u last_status=0x%02X, "
          "continue (nvm_err=0)", acks, BMP585_NVM_TIMEOUT_MS, last);
    return RT_EOK;
}

/* 尝试一个地址: 读 CHIP_ID */
static rt_bool_t bmp_probe(rt_uint8_t addr)
{
    rt_uint8_t id = 0;

    bmp_dev.addr = addr;
    if (bmp_read_reg(BMP585_REG_CHIP_ID, &id) != RT_EOK)
        return RT_FALSE;
    if (id != BMP585_CHIP_ID_VAL)
        return RT_FALSE;

    bmp_dev.chip_id = id;
    return RT_TRUE;
}

/* 台架应急内部上拉 bmp_i2c2_pullup_boot 已于 2026-09-29 删除:
 * PB10/PB11 外部上拉已焊装并经 SWD 验证 (关内部上拉后两线仍高、
 * I2C 维持 100.0 obs/s)。历史实现见 git/README 记录。 */

/* 配置 (待机态写入) + 进入正常模式 + 压强合理性自检, 最多两轮
 * (自检失败回待机重配: 覆盖配置写被芯片内部状态吞掉的情况) */
static rt_err_t bmp_config_and_selftest(void)
{
    s_cache_valid = RT_FALSE;   /* 重配置后旧原始值不再同源 */
    for (rt_uint8_t attempt = 0; attempt < 2; attempt++)
    {
        rt_int32_t t_raw, p_raw;

        if (bmp_set_power_mode(BMP585_PWR_STANDBY) != RT_EOK)
            return -RT_EIO;
        if (bmp_set_osr(BMP585_DEFAULT_OSR_P, BMP585_DEFAULT_OSR_T) != RT_EOK)
            return -RT_EIO;
        if (bmp_set_iir(BMP585_IIR_COEFF) != RT_EOK)
            return -RT_EIO;
        if (bmp_set_odr_code(bmp_odr_code_from_hz_x10(BMP585_DEFAULT_ODR_HZ * 10)) != RT_EOK)
            return -RT_EIO;
        if (bmp_set_power_mode(BMP585_PWR_NORMAL) != RT_EOK)
            return -RT_EIO;

        rt_thread_mdelay(30);           /* 首次转换完成 */
        if (bmp_read_raw(&t_raw, &p_raw) == RT_EOK)
        {
            rt_int32_t pa = p_raw / 64;

            if (pa >= BMP585_SANITY_PA_MIN && pa <= BMP585_SANITY_PA_MAX)
            {
                LOG_I("BMP585 ready (addr=0x%02X, id=0x%02X, ODR=%dHz, %d.%02u hPa)",
                      bmp_dev.addr, bmp_dev.chip_id, BMP585_DEFAULT_ODR_HZ,
                      pa / 100, (rt_uint32_t)(pa % 100));
                return RT_EOK;
            }

            LOG_W("pressure self-test implausible: %d Pa (attempt %d)", pa, attempt + 1);
        }
        else
        {
            LOG_W("pressure self-test read failed (attempt %d)", attempt + 1);
        }
    }

    return -RT_ERROR;
}

/* 延迟重试参数: 板级复位伴随的 3V3 纹波会使传感器欠压复位, NVM 重载
 * 停摆数分钟到十几分钟不等才自愈 (期间配置写 ACK 但不生效, 压强读
 * 0x7FFFFF 饱和值, 温度正常; 见文件头 2026-09-29 记录)。boot 期自检
 * 必落在此窗口内, 由后台线程无界重试兜底 (实测自愈时间 3~7min+ 波动,
 * 有界窗口必输); baro_data 侧同样无界等待设备。
 * 重试只走快速路径 (不做软复位 —— 复位会重新触发 NVM 停摆)。 */
#define BMP585_INIT_RETRY_MS       10000
#define BMP585_RETRY_THREAD_STACK  3072    /* ulog 格式化尖峰余量 */

/* 核心初始化 (探测 -> 配置+自检 -> 注册); 幂等, 重试线程复用 */
static rt_err_t bmp_try_init(rt_bool_t allow_reset)
{
    /* 地址探测 (SDO=0 -> 0x46, SDO=1 -> 0x47) */
    if (!bmp_probe(BMP585_I2C_ADDR_DEFAULT) && !bmp_probe(BMP585_I2C_ADDR_ALT))
    {
        LOG_E("BMP585 not found, check I2C4 wiring PB6=SCL PB9=SDA (2026-10-04 迁自 I2C2)");
        return -RT_ERROR;
    }

    /* 快速路径: 不软复位, 直接从芯片当前状态配置 + 自检 (2026-09-29:
     * 本模块软复位后 NVM 重载停摆数分钟, 配置丢失/数据垃圾; 而上电场景
     * 芯片早已在 POR 阶段自行完成初始化, 复位纯属有害。复位只留给
     * 芯片状态确实垃圾的兜底路径) */
    if (bmp_config_and_selftest() == RT_EOK)
        return (bmp_register_sensors() == RT_EOK) ? RT_EOK : -RT_ERROR;

    if (!allow_reset)
        return -RT_ERROR;

    /* 兜底: 软复位 (nvm_err 硬门禁, nvm_rdy 尽力等待) 后再试 */
    if (bmp_soft_reset() != RT_EOK)
        return -RT_ERROR;
    if (bmp_config_and_selftest() == RT_EOK)
        return (bmp_register_sensors() == RT_EOK) ? RT_EOK : -RT_ERROR;

    return -RT_ERROR;
}

static void bmp_retry_entry(void *parameter)
{
    rt_uint32_t i = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_thread_mdelay(BMP585_INIT_RETRY_MS);
        i++;
        if (bmp_try_init(RT_FALSE) == RT_EOK)
        {
            LOG_I("BMP585 online after deferred retry %u", i);
            return;
        }
        if (i % 30u == 0u)
            LOG_W("BMP585 still not ready after %u retries (%us), keep trying",
                  i, (rt_uint32_t)(i * BMP585_INIT_RETRY_MS / 1000));
    }
}

int rt_hw_bmp585_init(void)
{
    rt_thread_t retry;

    bmp_dev.bus = (struct rt_i2c_bus_device *)rt_device_find(BMP585_I2C_BUS_NAME);
    if (bmp_dev.bus == RT_NULL)
    {
        LOG_E("I2C bus %s not found", BMP585_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    if (bmp_try_init(RT_TRUE) == RT_EOK)
        return 0;

    /* 立即初始化失败: 传感器多在复位后 NVM 停摆窗口内, 后台延迟重试 */
    retry = rt_thread_create("bmpretry", bmp_retry_entry, RT_NULL,
                             BMP585_RETRY_THREAD_STACK, 20, 10);
    if (retry != RT_NULL)
        rt_thread_startup(retry);

    return -RT_ERROR;
}
/* ------------------------- 时间戳接口 ------------------------- */

/*
 * (原 bmp585_get_stamp 随 pps_sync 移除, 时间基座待重建后重新提供。)
 */

#if BMP585_ENABLE
INIT_DEVICE_EXPORT(rt_hw_bmp585_init);
#endif

/* ------------------------- FinSH 诊断命令 ------------------------- */

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include <string.h>

static void bmp_dbg_dump(const char *tag)
{
    static const struct { char name[10]; rt_uint8_t reg; } regs[] =
    {
        { "CHIP_ID",   0x01 },
        { "CHIP_STAT", 0x11 },
        { "INT_CFG",   0x14 },
        { "INT_SRC",   0x15 },
        { "INT_STAT",  0x27 },
        { "STATUS",    0x28 },
        { "DSP_IIR",   0x31 },
        { "OSR_CFG",   0x36 },
        { "ODR_CFG",   0x37 },
    };
    rt_uint8_t v;
    rt_uint32_t i;

    rt_kprintf("[%s]\n", tag);
    for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
    {
        if (bmp_read_reg(regs[i].reg, &v) == RT_EOK)
            rt_kprintf("  %-10s(0x%02X)=0x%02X\n", regs[i].name, regs[i].reg, v);
        else
            rt_kprintf("  %-10s(0x%02X)=ERR\n", regs[i].name, regs[i].reg);
    }
}

/* STATUS(0x28): bit1=nvm_rdy(复位值 1), bit2=nvm_err, bit3=nvm_cmd_err */
static void bmp585dbg(int argc, char **argv)
{
    rt_uint8_t status, prev;
    rt_int32_t t_raw, p_raw;
    int i;

    if (bmp_dev.bus == RT_NULL)
    {
        bmp_dev.bus = (struct rt_i2c_bus_device *)rt_device_find(BMP585_I2C_BUS_NAME);
        if (bmp_dev.bus == RT_NULL)
        {
            rt_kprintf("I2C bus %s not found\n", BMP585_I2C_BUS_NAME);
            return;
        }
    }

    if (bmp_dev.chip_id == 0)
    {
        if (!bmp_probe(BMP585_I2C_ADDR_DEFAULT) && !bmp_probe(BMP585_I2C_ADDR_ALT))
        {
            rt_kprintf("probe 0x%02X/0x%02X fail\n",
                       BMP585_I2C_ADDR_DEFAULT, BMP585_I2C_ADDR_ALT);
            return;
        }
    }
    rt_kprintf("bus=%s addr=0x%02X id=0x%02X\n",
               BMP585_I2C_BUS_NAME, bmp_dev.addr, bmp_dev.chip_id);

    /* 连读 STATUS 观察稳定性 (区分真实芯片状态 vs 总线误码) */
    prev = 0xFF;
    for (i = 0; i < 8; i++)
    {
        if (bmp_read_reg(BMP585_REG_STATUS, &status) != RT_EOK)
            status = 0xFF;
        rt_kprintf("STATUS[%d]=0x%02X%s\n", i, status,
                   (status != prev) ? "" : " (同上)");
        prev = status;
        rt_thread_mdelay(5);
    }
    bmp_dbg_dump("regs");

    if (argc >= 2 && strcmp(argv[1], "reset") == 0)
    {
        rt_kprintf("[soft reset 0xB6 -> 0x7E]\n");
        (void)bmp_write_reg(BMP585_REG_CMD, BMP585_CMD_SOFT_RESET);
        rt_thread_mdelay(5);
        bmp_dbg_dump("after reset");
    }

    if (argc >= 2 && strcmp(argv[1], "test") == 0)
    {
        /* 旁路 nvm_rdy 门限: 直接配置并读数据, 验证芯片是否实际可用 */
        rt_kprintf("[bypass test: osr/iir/odr/normal]\n");
        rt_kprintf("set_osr=%d set_iir=%d set_odr=%d set_pwr=%d\n",
                   bmp_set_osr(BMP585_DEFAULT_OSR_P, BMP585_DEFAULT_OSR_T),
                   bmp_set_iir(BMP585_IIR_COEFF),
                   bmp_set_odr_code(bmp_odr_code_from_hz_x10(BMP585_DEFAULT_ODR_HZ * 10)),
                   bmp_set_power_mode(BMP585_PWR_NORMAL));
        for (i = 0; i < 6; i++)
        {
            rt_int32_t tc_x100, pa;

            rt_thread_mdelay(30);
            if (bmp_read_raw(&t_raw, &p_raw) != RT_EOK)
            {
                rt_kprintf("read raw ERR\n");
                continue;
            }
            tc_x100 = t_raw * 100 / 65536;      /* (signed,24,16) degC */
            pa      = p_raw / 64;               /* (signed,24,6) Pa */
            rt_kprintf("P=%d.%02u hPa  T=%d.%02u C  (raw t=%d p=%d)\n",
                       pa / 100, (rt_uint32_t)(pa % 100),
                       tc_x100 / 100,
                       (rt_uint32_t)(tc_x100 < 0 ? -tc_x100 : tc_x100) % 100u,
                       t_raw, p_raw);
        }
        bmp_dbg_dump("after test");
    }
}
MSH_CMD_EXPORT(bmp585dbg, BMP585 diag: [reset|test]);
#endif /* RT_USING_FINSH && FINSH_USING_MSH */
