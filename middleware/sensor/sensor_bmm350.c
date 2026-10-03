/*
 * Copyright (c) 2026 RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BMM350 磁力计 RT-Thread 传感器驱动实现 (I2C1: SCL=PB8/SDA=PB7, INT PB5;
 *           2026-10-04 由 I2C4/PF14/PF15 迁回, 寄存器时序与总线无耦合)
 *
 * 寄存器访问与补偿算法按 Bosch Sensortec BMM350 API (BSD-3) 移植:
 *  - I2C 读时序: 先返回 2 个 dummy 字节, 之后才是寄存器数据
 *  - 初始化: 软复位 -> CHIP_ID -> 下载 32 字 OTP 系数 -> OTP 断电
 *            -> 磁复位(BR/FGR 序列) -> 配 ODR/平均 -> 正常模式
 *  - 数据: 0x31 起连续读 12 字节 (XYZ 温度各 24-bit 小端),
 *          用 OTP 系数做灵敏度/偏置/温漂/交叉轴补偿
 */

#include "sensor_bmm350.h"
#include <drivers/sensor.h>
#include "board.h"                      /* rt_hw_us_delay */
#include "timebase.h"                   /* fetch 打戳 (T_MCU ms, 同 ADIS 惯例) */
#include <math.h>                       /* fabsf/isfinite (fetch 合理性检查) */

#define LOG_TAG "sensor.bmm350"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

/* ------------------------- 寄存器定义 ------------------------- */
#define BMM350_REG_CHIP_ID          0x00
#define BMM350_REG_ERR_REG          0x02
#define BMM350_REG_PMU_CMD_AGGR_SET 0x04    /* [3:0] ODR, [5:4] 平均次数 */
#define BMM350_REG_PMU_CMD_AXIS_EN  0x05    /* [2:0] XYZ 使能 */
#define BMM350_REG_PMU_CMD          0x06    /* 电源模式命令 */
#define BMM350_REG_PMU_CMD_STATUS_0 0x07
#define BMM350_REG_INT_CTRL         0x2E    /* [7] DRDY状态使能 [3] INT脚输出 [1]极性 [2]推挽 [0]脉冲 */
#define BMM350_REG_INT_STATUS       0x30    /* [2] 数据就绪 */
#define BMM350_REG_MAG_X_XLSB       0x31    /* X/Y/Z/温度 12 字节连续 */
#define BMM350_REG_OTP_CMD_REG      0x50
#define BMM350_REG_OTP_DATA_MSB_REG 0x52
#define BMM350_REG_OTP_DATA_LSB_REG 0x53
#define BMM350_REG_OTP_STATUS_REG   0x55    /* [0] 命令完成, [7:5] 错误 */
#define BMM350_REG_CMD              0x7E    /* 0xB6 软复位 */

#define BMM350_CHIP_ID_VAL          0x33

/* PMU 命令 */
#define BMM350_PMU_CMD_SUS          0x00    /* 挂起 */
#define BMM350_PMU_CMD_NM           0x01    /* 正常 */
#define BMM350_PMU_CMD_UPD_OAE      0x02    /* 更新 ODR/平均 */
#define BMM350_PMU_CMD_FGR          0x05    /* 快速磁复位 */
#define BMM350_PMU_CMD_BR           0x07    /* 磁复位 */

/* 关键延时 (Bosch API, 单位 us) */
#define BMM350_STARTUP_DELAY_US     3000
#define BMM350_SOFT_RESET_DELAY_US  24000
#define BMM350_GOTO_SUSPEND_DELAY_US 6000
#define BMM350_BR_DELAY_US          14000
#define BMM350_FGR_DELAY_US         18000
#define BMM350_SUS_TO_NORMAL_US     38000
#define BMM350_UPD_OAE_DELAY_US     1000

/* OTP */
#define BMM350_OTP_DATA_LENGTH      32
#define BMM350_OTP_CMD_DIR_READ     0x20
#define BMM350_OTP_CMD_PWR_OFF      0x80
#define BMM350_OTP_STATUS_ERR_MSK   0xE0
#define BMM350_OTP_STATUS_CMD_DONE  0x01

/* ODR 档位代码 */
#define BMM350_ODR_400HZ            0x2
#define BMM350_ODR_200HZ            0x3
#define BMM350_ODR_100HZ            0x4
#define BMM350_ODR_50HZ             0x5
#define BMM350_ODR_25HZ             0x6
#define BMM350_ODR_12_5HZ           0x7

/* I2C 读 dummy 字节数 */
#define BMM350_DUMMY_BYTES          2

/* 量程: X/Y ±1300 uT, Z ±2500 uT (换算 mGauss) */
#define BMM350_RANGE_XY_MGAUSS      13000
#define BMM350_RANGE_Z_MGAUSS       25000

/* ------------------------- 设备结构 ------------------------- */

/* OTP 下载得到的补偿系数 (数值含义见 Bosch API bmm350_struct.h) */
struct bmm350_mag_comp
{
    float offset[3];        /* uT */
    float t_offs;           /* °C */
    float sens[3];          /* 1 */
    float t_sens;           /* 1 */
    float tco[3];           /* uT/°C */
    float tcs[3];           /* 1/°C */
    float t0;               /* °C */
    float cross_xy;         /* 1 */
    float cross_yx;         /* 1 */
    float cross_zx;         /* 1 */
    float cross_zy;         /* 1 */
};

struct bmm350_device
{
    struct rt_i2c_bus_device *bus;
    rt_uint8_t  addr;
    rt_uint8_t  chip_id;
    rt_uint8_t  axis_en;
    rt_uint16_t otp_data[BMM350_OTP_DATA_LENGTH];
    struct bmm350_mag_comp comp;
    rt_uint8_t  odr_code;               /* 当前 ODR 档位 */
    rt_uint8_t  avg;                    /* 当前平均档位 */
    rt_bool_t   powered;                /* 是否处于正常模式 */
};

static struct bmm350_device bmm_dev;
static struct rt_sensor_module sensor_module;   /* mag/temp 共用一个模块 */

/* ------------------------- I2C 基础读写 ------------------------- */

static rt_err_t bmm_read_regs(rt_uint8_t reg, rt_uint8_t *data, rt_uint16_t len)
{
    struct rt_i2c_msg msgs[2];
    rt_uint8_t buf[BMM350_DUMMY_BYTES + 16];
    rt_ssize_t ret;

    /* len (rt_uint16_t) 整数提升为 int, 直接与 sizeof(size_t) 比较会触发
     * 符号比较告警; 先转入无符号域统一类型 */
    if ((rt_size_t)len + BMM350_DUMMY_BYTES > sizeof(buf))
        return -RT_EINVAL;

    msgs[0].addr  = bmm_dev.addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    msgs[1].addr  = bmm_dev.addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = buf;
    msgs[1].len   = len + BMM350_DUMMY_BYTES;

    ret = rt_i2c_transfer(bmm_dev.bus, msgs, 2);
    if (ret != 2)
        return -RT_EIO;

    /* 跳过前 2 个 dummy 字节 */
    for (rt_uint16_t i = 0; i < len; i++)
        data[i] = buf[i + BMM350_DUMMY_BYTES];

    return RT_EOK;
}

static rt_err_t bmm_write_regs(rt_uint8_t reg, const rt_uint8_t *data, rt_uint16_t len)
{
    struct rt_i2c_msg msg;
    rt_uint8_t buf[8];

    if ((rt_size_t)len + 1 > sizeof(buf))
        return -RT_EINVAL;

    buf[0] = reg;
    for (rt_uint16_t i = 0; i < len; i++)
        buf[i + 1] = data[i];

    msg.addr  = bmm_dev.addr;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = len + 1;

    return (rt_i2c_transfer(bmm_dev.bus, &msg, 1) == 1) ? RT_EOK : -RT_EIO;
}

static rt_err_t bmm_read_reg(rt_uint8_t reg, rt_uint8_t *val)
{
    return bmm_read_regs(reg, val, 1);
}

static rt_err_t bmm_write_reg(rt_uint8_t reg, rt_uint8_t val)
{
    return bmm_write_regs(reg, &val, 1);
}

/* 延时: ≥1ms 用 rt_thread_mdelay 让出 CPU, 否则忙等 (均保证不低于要求时长) */
static void bmm_delay_us(rt_uint32_t us)
{
    if (us >= 1000)
        rt_thread_mdelay((us + 999) / 1000);
    else
        rt_hw_us_delay(us);
}

/* ------------------------- OTP 系数下载 (Bosch 算法) ------------------------- */

/* 无符号数按指定位宽转有符号 */
static rt_int32_t bmm_fix_sign(rt_uint32_t inval, rt_uint8_t bits)
{
    rt_int32_t val = (rt_int32_t)inval;
    rt_int32_t half = 1 << (bits - 1);

    if (val >= half)
        val -= half * 2;
    return val;
}

static rt_err_t bmm_read_otp_word(rt_uint8_t addr, rt_uint16_t *word)
{
    rt_uint8_t status = 0, msb = 0, lsb = 0;
    rt_uint8_t i;

    if (bmm_write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_DIR_READ | (addr & 0x1F)) != RT_EOK)
        return -RT_EIO;

    for (i = 0; i < 100; i++)           /* 有界轮询, 防总线异常死循环 */
    {
        rt_hw_us_delay(300);
        if (bmm_read_reg(BMM350_REG_OTP_STATUS_REG, &status) != RT_EOK)
            return -RT_EIO;
        if (status & BMM350_OTP_STATUS_ERR_MSK)
        {
            LOG_E("OTP read error, addr=%d status=0x%02X", addr, status);
            return -RT_EIO;
        }
        if (status & BMM350_OTP_STATUS_CMD_DONE)
            break;
    }
    if (i >= 100)
        return -RT_ETIMEOUT;

    if (bmm_read_reg(BMM350_REG_OTP_DATA_MSB_REG, &msb) != RT_EOK ||
        bmm_read_reg(BMM350_REG_OTP_DATA_LSB_REG, &lsb) != RT_EOK)
        return -RT_EIO;

    *word = (rt_uint16_t)((msb << 8) | lsb);
    return RT_EOK;
}

/* 由 OTP 字更新补偿系数 (与 Bosch update_mag_off_sens 一致) */
static void bmm_update_comp(void)
{
    const rt_uint16_t *otp = bmm_dev.otp_data;
    struct bmm350_mag_comp *c = &bmm_dev.comp;
    rt_uint16_t off_x, off_y, off_z, t_off;
    rt_uint8_t sens_x, sens_y, sens_z, t_sens;
    rt_uint8_t tco_x, tco_y, tco_z;
    rt_uint8_t tcs_x, tcs_y, tcs_z;
    rt_uint8_t cross_xy, cross_yx, cross_zx, cross_zy;

#define LSB_MSK 0x00FFu
#define MSB_MSK 0xFF00u

    off_x = otp[0x0E] & 0x0FFF;
    off_y = ((otp[0x0E] & 0xF000) >> 4) + (otp[0x0F] & LSB_MSK);
    off_z = (otp[0x0F] & 0x0F00) + (otp[0x10] & LSB_MSK);
    t_off = otp[0x0D] & LSB_MSK;

    c->offset[0] = bmm_fix_sign(off_x, 12);
    c->offset[1] = bmm_fix_sign(off_y, 12);
    c->offset[2] = bmm_fix_sign(off_z, 12);
    c->t_offs = bmm_fix_sign(t_off, 8) / 5.0f;

    sens_x = (otp[0x10] & MSB_MSK) >> 8;
    sens_y = otp[0x11] & LSB_MSK;
    sens_z = (otp[0x11] & MSB_MSK) >> 8;
    t_sens = (otp[0x0D] & MSB_MSK) >> 8;

    c->sens[0] = bmm_fix_sign(sens_x, 8) / 256.0f;
    c->sens[1] = bmm_fix_sign(sens_y, 8) / 256.0f + 0.01f;
    c->sens[2] = bmm_fix_sign(sens_z, 8) / 256.0f;
    c->t_sens = bmm_fix_sign(t_sens, 8) / 512.0f;

    tco_x = otp[0x12] & LSB_MSK;
    tco_y = otp[0x13] & LSB_MSK;
    tco_z = otp[0x14] & LSB_MSK;
    c->tco[0] = bmm_fix_sign(tco_x, 8) / 32.0f;
    c->tco[1] = bmm_fix_sign(tco_y, 8) / 32.0f;
    c->tco[2] = bmm_fix_sign(tco_z, 8) / 32.0f;

    tcs_x = (otp[0x12] & MSB_MSK) >> 8;
    tcs_y = (otp[0x13] & MSB_MSK) >> 8;
    tcs_z = (otp[0x14] & MSB_MSK) >> 8;
    c->tcs[0] = bmm_fix_sign(tcs_x, 8) / 16384.0f;
    c->tcs[1] = bmm_fix_sign(tcs_y, 8) / 16384.0f;
    c->tcs[2] = bmm_fix_sign(tcs_z, 8) / 16384.0f - 0.0001f;

    c->t0 = bmm_fix_sign(otp[0x18], 16) / 512.0f + 23.0f;

    cross_xy = otp[0x15] & LSB_MSK;
    cross_yx = (otp[0x15] & MSB_MSK) >> 8;
    cross_zx = otp[0x16] & LSB_MSK;
    cross_zy = (otp[0x16] & MSB_MSK) >> 8;
    c->cross_xy = bmm_fix_sign(cross_xy, 8) / 800.0f;
    c->cross_yx = bmm_fix_sign(cross_yx, 8) / 800.0f;
    c->cross_zx = bmm_fix_sign(cross_zx, 8) / 800.0f;
    c->cross_zy = bmm_fix_sign(cross_zy, 8) / 800.0f;

#undef LSB_MSK
#undef MSB_MSK
}

static rt_err_t bmm_otp_dump(void)
{
    rt_uint16_t word = 0;

    for (rt_uint8_t i = 0; i < BMM350_OTP_DATA_LENGTH; i++)
    {
        if (bmm_read_otp_word(i, &word) != RT_EOK)
            return -RT_EIO;
        bmm_dev.otp_data[i] = word;
    }

    bmm_update_comp();
    return RT_EOK;
}

/* ------------------------- 模式配置 ------------------------- */

/* 读 PMU_CMD_STATUS_0: [3] 是否正常模式, [7:5] 最近命令 */
static rt_err_t bmm_get_pmu_status(rt_uint8_t *status)
{
    return bmm_read_reg(BMM350_REG_PMU_CMD_STATUS_0, status);
}

/* 磁复位序列: (必要时先挂起) BR -> FGR, 期间校验命令回读 */
static rt_err_t bmm_magnetic_reset(void)
{
    rt_uint8_t status = 0, restore_normal = 0;

    if (bmm_get_pmu_status(&status) != RT_EOK)
        return -RT_EIO;
    if (status & 0x08)                  /* 当前为正常模式, 磁复位仅可在挂起态执行 */
    {
        restore_normal = 1;
        if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUS) != RT_EOK)
            return -RT_EIO;
        bmm_delay_us(BMM350_GOTO_SUSPEND_DELAY_US);
    }

    if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR) != RT_EOK)
        return -RT_EIO;
    bmm_delay_us(BMM350_BR_DELAY_US);
    if (bmm_get_pmu_status(&status) != RT_EOK || (status >> 5) != BMM350_PMU_CMD_BR)
        return -RT_EIO;

    if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR) != RT_EOK)
        return -RT_EIO;
    bmm_delay_us(BMM350_FGR_DELAY_US);
    if (bmm_get_pmu_status(&status) != RT_EOK || (status >> 5) != BMM350_PMU_CMD_FGR)
        return -RT_EIO;

    if (restore_normal)
    {
        if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_NM) != RT_EOK)
            return -RT_EIO;
        bmm_delay_us(BMM350_SUS_TO_NORMAL_US);
        bmm_dev.powered = RT_TRUE;
    }

    return RT_EOK;
}

/* 设置 ODR 与平均档位 (超限组合自动降档, 与 Bosch API 一致) */
static rt_err_t bmm_set_odr_avg(rt_uint8_t odr_code, rt_uint8_t avg)
{
    rt_uint8_t reg;

    if (odr_code == BMM350_ODR_400HZ && avg >= 1)
        avg = 0;
    else if (odr_code == BMM350_ODR_200HZ && avg >= 2)
        avg = 1;
    else if (odr_code == BMM350_ODR_100HZ && avg >= 3)
        avg = 2;

    reg = (odr_code & 0x0F) | ((avg & 0x03) << 4);
    if (bmm_write_reg(BMM350_REG_PMU_CMD_AGGR_SET, reg) != RT_EOK)
        return -RT_EIO;

    if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE) != RT_EOK)
        return -RT_EIO;
    bmm_delay_us(BMM350_UPD_OAE_DELAY_US);

    bmm_dev.odr_code = odr_code;
    bmm_dev.avg = avg;
    return RT_EOK;
}

static rt_err_t bmm_enable_axes(void)
{
    bmm_dev.axis_en = 0x07;             /* XYZ 全开 */
    return bmm_write_reg(BMM350_REG_PMU_CMD_AXIS_EN, bmm_dev.axis_en);
}

static rt_err_t bmm_set_normal_mode(void)
{
    if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_NM) != RT_EOK)
        return -RT_EIO;
    bmm_delay_us(BMM350_SUS_TO_NORMAL_US);
    bmm_dev.powered = RT_TRUE;
    return RT_EOK;
}

static rt_err_t bmm_set_suspend_mode(void)
{
    if (bmm_write_reg(BMM350_REG_PMU_CMD, BMM350_PMU_CMD_SUS) != RT_EOK)
        return -RT_EIO;
    bmm_delay_us(BMM350_GOTO_SUSPEND_DELAY_US);
    bmm_dev.powered = RT_FALSE;
    return RT_EOK;
}

/* ------------------------- 数据读取与补偿 ------------------------- */

/* 读取并补偿, out[0..2]=磁感应强度 uT, out[3]=温度 °C (Bosch 算法) */
static rt_err_t bmm_read_compensated(float out[4])
{
    rt_uint8_t raw[12];
    rt_int32_t rx, ry, rz, rt_;
    float m[3], temp;
    /* LSB->uT / °C 换算系数 (Bosch API update_default_coefiecents) */
    const float lsb_xy = 1000000.0f / 1048576.0f /
                         (14.55f * 19.46f * (1.0f / 1.5f) * 0.714607238769531f);
    const float lsb_z  = 1000000.0f / 1048576.0f /
                         (9.0f * 31.0f * (1.0f / 1.5f) * 0.714607238769531f);
    const float lsb_t  = 1.0f /
                         (0.00204f * (1.0f / 1.5f) * 0.714607238769531f * 1048576.0f);
    const struct bmm350_mag_comp *c = &bmm_dev.comp;
    float cx, cy, cz;

    if (bmm_read_regs(BMM350_REG_MAG_X_XLSB, raw, 12) != RT_EOK)
        return -RT_EIO;

    rx = bmm_fix_sign(raw[0] + ((rt_uint32_t)raw[1] << 8) + ((rt_uint32_t)raw[2] << 16), 24);
    ry = bmm_fix_sign(raw[3] + ((rt_uint32_t)raw[4] << 8) + ((rt_uint32_t)raw[5] << 16), 24);
    rz = bmm_fix_sign(raw[6] + ((rt_uint32_t)raw[7] << 8) + ((rt_uint32_t)raw[8] << 16), 24);
    rt_ = bmm_fix_sign(raw[9] + ((rt_uint32_t)raw[10] << 8) + ((rt_uint32_t)raw[11] << 16), 24);

    m[0] = rx * lsb_xy;
    m[1] = ry * lsb_xy;
    m[2] = rz * lsb_z;
    temp = rt_ * lsb_t;

    /* 温度偏移修正 */
    if (temp > 0.0f)
        temp -= 25.49f;
    else if (temp < 0.0f)
        temp += 25.49f;

    /* 温度补偿 */
    temp = (1.0f + c->t_sens) * temp + c->t_offs;
    for (int i = 0; i < 3; i++)
    {
        m[i] *= 1.0f + c->sens[i];
        m[i] += c->offset[i];
        m[i] += c->tco[i] * (temp - c->t0);
        m[i] /= 1.0f + c->tcs[i] * (temp - c->t0);
    }

    /* 交叉轴补偿 */
    cx = (m[0] - c->cross_xy * m[1]) / (1.0f - c->cross_yx * c->cross_xy);
    cy = (m[1] - c->cross_yx * m[0]) / (1.0f - c->cross_yx * c->cross_xy);
    cz = m[2] + (m[0] * (c->cross_yx * c->cross_zy - c->cross_zx) -
                 m[1] * (c->cross_zy - c->cross_xy * c->cross_zx)) /
                (1.0f - c->cross_yx * c->cross_xy);

    out[0] = cx;
    out[1] = cy;
    out[2] = cz;
    out[3] = temp;
    return RT_EOK;
}

/* ------------------------- 传感器框架回调 ------------------------- */

/* 运行期健康监测: 连续 I2C 失败 / 数据超量程 (芯片欠压复位后回挂起态,
 * powered 标志与芯片实际状态失同步, 坏数据会静默直通椭球校正与融合)
 * 触发重走配置序列 (磁复位+轴使能+ODR+正常模式)。分级判定:
 *   - I2C 传输失败 = 硬故障 (掉线/挂起), 恢复动作明确廉价, 连续
 *     20 次 (200ms@100Hz) 即重配置, 不等超界窗口 (3s 磁断供对罗盘
 *     辅助航向偏长);
 *   - 数据超界 = 软信号, 维持 300 样本 (3s) 窗口防误判。 */
#define BMM350_FETCH_BAD_RECFG_N   300
#define BMM350_FETCH_NAK_RECFG_N   20
#define BMM350_RANGE_UT_MAX        2500.0f    /* XY 量程 2000uT + 裕量 */
static rt_uint16_t bmm_bad_cnt;             /* 数据超界窗口计数 */
static rt_uint16_t bmm_nak_cnt;             /* I2C 连续失败计数 */

static rt_err_t bmm_recover_runtime(void)
{
    LOG_W("BMM350 unhealthy (I2C fail x%d / out-of-range x%d), reconfiguring",
          BMM350_FETCH_NAK_RECFG_N, BMM350_FETCH_BAD_RECFG_N);

    if (bmm_magnetic_reset() != RT_EOK)
        return -RT_EIO;
    if (bmm_enable_axes() != RT_EOK)
        return -RT_EIO;
    if (bmm_set_odr_avg(BMM350_ODR_100HZ, BMM350_DEFAULT_AVG) != RT_EOK)
        return -RT_EIO;
    return bmm_set_normal_mode();
}

static rt_ssize_t bmm_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    float out[4];

    if (data == RT_NULL || len == 0 || !bmm_dev.powered)
        return 0;

    if (bmm_read_compensated(out) != RT_EOK)
    {
        /* I2C 失败 (NAK/超时) 硬故障通道: 快速重配置 */
        if (++bmm_nak_cnt >= BMM350_FETCH_NAK_RECFG_N)
        {
            bmm_nak_cnt = 0;
            (void)bmm_recover_runtime();    /* 失败则下轮 NAK 再试 */
        }
        return 0;
    }
    bmm_nak_cnt = 0;

    /* 数据合理性: 补偿后超量程 = 芯片状态垃圾 (欠压复位/配置丢失),
     * isfinite 拦补偿发散 (交叉轴分母趋零时) */
    if (!isfinite(out[0]) || !isfinite(out[1]) || !isfinite(out[2]) ||
        fabsf(out[0]) > BMM350_RANGE_UT_MAX ||
        fabsf(out[1]) > BMM350_RANGE_UT_MAX ||
        fabsf(out[2]) > BMM350_RANGE_UT_MAX)
        goto __bad;

    bmm_bad_cnt = 0;

    data->type = sensor->info.type;
    data->timestamp = (rt_uint32_t)(timebase_now_us() / 1000u);  /* T_MCU ms */

    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        /* 1 uT = 10 mGauss */
        data->data.mag.x = (rt_int32_t)(out[0] * 10.0f + (out[0] >= 0 ? 0.5f : -0.5f));
        data->data.mag.y = (rt_int32_t)(out[1] * 10.0f + (out[1] >= 0 ? 0.5f : -0.5f));
        data->data.mag.z = (rt_int32_t)(out[2] * 10.0f + (out[2] >= 0 ? 0.5f : -0.5f));
    }
    else    /* RT_SENSOR_CLASS_TEMP: 0.1°C */
    {
        data->data.temp = (rt_int32_t)(out[3] * 10.0f + (out[3] >= 0 ? 0.5f : -0.5f));
    }

    return 1;

__bad:
    if (++bmm_bad_cnt >= BMM350_FETCH_BAD_RECFG_N)
    {
        bmm_bad_cnt = 0;
        (void)bmm_recover_runtime();    /* 失败则下轮坏样本再试 */
    }
    return 0;
}

/* ODR 档位表: Hz -> 代码 (Bosch 量化档位) */
static const struct
{
    rt_uint16_t hz;
    rt_uint8_t  code;
} odr_table[] =
{
    {400, BMM350_ODR_400HZ},
    {200, BMM350_ODR_200HZ},
    {100, BMM350_ODR_100HZ},
    {50,  BMM350_ODR_50HZ},
    {25,  BMM350_ODR_25HZ},
    {13,  BMM350_ODR_12_5HZ},           /* 12.5 Hz */
};

static rt_err_t bmm_control(struct rt_sensor_device *sensor, int cmd, void *arg)
{
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        if (arg)
            *(rt_uint8_t *)arg = bmm_dev.chip_id;
        return RT_EOK;
    case RT_SENSOR_CTRL_SET_ODR:
    {
        rt_uint32_t odr = (rt_uint32_t)arg & 0xFFFF;
        rt_uint8_t best = odr_table[0].code;
        rt_int32_t best_err = 0x7FFFFFFF;

        for (rt_uint32_t i = 0; i < sizeof(odr_table) / sizeof(odr_table[0]); i++)
        {
            rt_int32_t err = (rt_int32_t)odr_table[i].hz - (rt_int32_t)odr;
            if (err < 0)
                err = -err;
            if (err < best_err)
            {
                best_err = err;
                best = odr_table[i].code;
            }
        }
        return bmm_set_odr_avg(best, bmm_dev.avg);
    }
    case RT_SENSOR_CTRL_SET_MODE:
        if ((rt_uint32_t)arg == RT_SENSOR_MODE_POLLING)
            return RT_EOK;
        return -RT_ERROR;               /* INT 引脚(PB5) 未使用 */
    case RT_SENSOR_CTRL_SET_POWER:
        if ((rt_uint32_t)arg == RT_SENSOR_POWER_NORMAL || (rt_uint32_t)arg == RT_SENSOR_POWER_HIGH)
            return bmm_set_normal_mode();
        if ((rt_uint32_t)arg == RT_SENSOR_POWER_DOWN || (rt_uint32_t)arg == RT_SENSOR_POWER_LOW)
            return bmm_set_suspend_mode();
        return -RT_EINVAL;
    default:
        return -RT_EINVAL;
    }
}

static struct rt_sensor_ops bmm_ops =
{
    .fetch_data = bmm_fetch_data,
    .control    = bmm_control,
};

/* ------------------------- 初始化与注册 ------------------------- */

static rt_err_t bmm_register_sensors(void)
{
    struct rt_sensor_config cfg = {0};

    if (sensor_module.lock)
        return RT_EOK;                  /* 已初始化 */

    cfg.intf.type = RT_SENSOR_INTF_I2C;
    cfg.intf.dev_name = BMM350_I2C_BUS_NAME;
    cfg.irq_pin.pin = RT_PIN_NONE;      /* 轮询模式, INT(PB5) 未使用 */
    cfg.mode = RT_SENSOR_MODE_POLLING;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    cfg.odr = BMM350_DEFAULT_ODR_HZ;

    sensor_module.lock = rt_mutex_create("bmmmd", RT_IPC_FLAG_PRIO);
    if (sensor_module.lock == RT_NULL)
        return -RT_ENOMEM;

    for (rt_uint8_t i = 0; i < 2; i++)
    {
        struct rt_sensor_device *sen = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sen == RT_NULL)
            goto __fail;

        sen->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sen->info.model      = "bmm350";
        sen->info.intf_type  = RT_SENSOR_INTF_I2C;
        sen->info.period_min = 10;      /* 100Hz -> 10ms */
        sen->info.fifo_max   = 0;
        sen->config          = cfg;
        sen->ops             = &bmm_ops;
        sen->module          = &sensor_module;

        if (i == 0)                     /* 磁力计 */
        {
            sen->info.type      = RT_SENSOR_CLASS_MAG;
            sen->info.unit      = RT_SENSOR_UNIT_MGAUSS;
            sen->info.range_max = BMM350_RANGE_XY_MGAUSS;
            sen->info.range_min = -BMM350_RANGE_XY_MGAUSS;
        }
        else                            /* 芯片温度 */
        {
            sen->info.type      = RT_SENSOR_CLASS_TEMP;
            sen->info.unit      = RT_SENSOR_UNIT_DCELSIUS;
            sen->info.range_max = 850;
            sen->info.range_min = -400;
        }

        if (rt_hw_sensor_register(sen, "bmm350", RT_DEVICE_FLAG_RDONLY, RT_NULL) != RT_EOK)
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

/* 尝试一个地址: 软复位 + 校验 CHIP_ID */
static rt_bool_t bmm_probe(rt_uint8_t addr)
{
    rt_uint8_t id = 0;
    rt_uint8_t rst = 0xB6;

    bmm_dev.addr = addr;
    if (bmm_write_regs(BMM350_REG_CMD, &rst, 1) != RT_EOK)
        return RT_FALSE;
    bmm_delay_us(BMM350_SOFT_RESET_DELAY_US);

    if (bmm_read_reg(BMM350_REG_CHIP_ID, &id) != RT_EOK)
        return RT_FALSE;
    if (id != BMM350_CHIP_ID_VAL)
        return RT_FALSE;

    bmm_dev.chip_id = id;
    return RT_TRUE;
}

/* ------------------------- 时间戳接口 ------------------------- */

/*
 * (原 bmm350_get_stamp 随 pps_sync 移除, 时间基座待重建后重新提供。)
 */

int rt_hw_bmm350_init(void)
{
    rt_uint8_t err_reg = 0;

    bmm_dev.bus = (struct rt_i2c_bus_device *)rt_device_find(BMM350_I2C_BUS_NAME);
    if (bmm_dev.bus == RT_NULL)
    {
        LOG_E("I2C bus %s not found", BMM350_I2C_BUS_NAME);
        return -RT_ERROR;
    }

    /* 上电延时 + 地址探测 (ADSEL 低 0x14 / 高 0x15) */
    bmm_delay_us(BMM350_STARTUP_DELAY_US);
    if (!bmm_probe(BMM350_I2C_ADDR_DEFAULT) && !bmm_probe(BMM350_I2C_ADDR_ALT))
    {
        LOG_E("BMM350 not found, check I2C1 wiring PB8=SCL PB7=SDA");
        return -RT_ERROR;
    }

    /* OTP 系数下载并断电 */
    if (bmm_otp_dump() != RT_EOK)
    {
        LOG_E("BMM350 OTP dump failed");
        return -RT_ERROR;
    }
    if (bmm_write_reg(BMM350_REG_OTP_CMD_REG, BMM350_OTP_CMD_PWR_OFF) != RT_EOK)
        return -RT_ERROR;

    /* 磁复位 (BR/FGR) */
    if (bmm_magnetic_reset() != RT_EOK)
    {
        LOG_E("BMM350 magnetic reset failed");
        return -RT_ERROR;
    }

    if (bmm_enable_axes() != RT_EOK)
        return -RT_ERROR;

    /* 默认 ODR/平均, 进入正常模式 */
    if (bmm_set_odr_avg(BMM350_ODR_100HZ, BMM350_DEFAULT_AVG) != RT_EOK)
        return -RT_ERROR;
    if (bmm_set_normal_mode() != RT_EOK)
        return -RT_ERROR;

    /* 读取错误寄存器确认无异常 */
    if (bmm_read_reg(BMM350_REG_ERR_REG, &err_reg) == RT_EOK && err_reg != 0)
        LOG_W("BMM350 ERR_REG=0x%02X", err_reg);

    LOG_I("BMM350 ready (addr=0x%02X, id=0x%02X)", bmm_dev.addr, bmm_dev.chip_id);

    return (bmm_register_sensors() == RT_EOK) ? 0 : -RT_ERROR;
}

/* ------------------------- 上电失败延迟重试 ------------------------- */

/*
 * BMM350 与 BMP585 同处一个 3V3 电源域: 板级复位纹波造成的欠压复位
 * 同样会让本芯片在探测/OTP/配置任一步失败 (BMP585 实测自愈 3~7min+)。
 * 旧实现 init 一次性失败即永久掉线 (mag_data 层也不再尝试)。init 全
 * 流程可重入 (软复位/OTP/磁复位/注册幂等), 后台线程无界重试兜底;
 * 期间 powered=FALSE, fetch 空转。
 */
#define BMM350_INIT_RETRY_MS        10000
#define BMM350_RETRY_THREAD_STACK   3072    /* ulog 格式化尖峰余量 */

static void bmm_retry_entry(void *parameter)
{
    rt_uint32_t i = 0;

    RT_UNUSED(parameter);

    while (1)
    {
        rt_thread_mdelay(BMM350_INIT_RETRY_MS);
        i++;
        if (rt_hw_bmm350_init() == 0)
        {
            LOG_I("BMM350 online after deferred retry %u", i);
            return;
        }
        if (i % 30u == 0u)
            LOG_W("BMM350 still not ready after %u retries (%us), keep trying",
                  i, (rt_uint32_t)(i * BMM350_INIT_RETRY_MS / 1000));
    }
}

static int rt_hw_bmm350_init_and_retry(void)
{
    if (rt_hw_bmm350_init() == 0)
        return 0;

    {
        rt_thread_t retry = rt_thread_create("bmmretry", bmm_retry_entry, RT_NULL,
                                             BMM350_RETRY_THREAD_STACK, 20, 10);
        if (retry != RT_NULL)
        {
            rt_thread_startup(retry);
            LOG_W("BMM350 init failed, background retry every %ds",
                  BMM350_INIT_RETRY_MS / 1000);
        }
    }
    return -RT_ERROR;
}
#if BMM350_ENABLE
INIT_DEVICE_EXPORT(rt_hw_bmm350_init_and_retry);
#endif
