/*
 * Bosch Sensortec SensorAPI license (BSD-3-Clause), retained in full.
 *
 * Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* BMP585 barometer on I2C4.
 *
 * Native RT-Thread device "barometer", built on the RT-Thread device framework only:
 * rt_device registration and lifecycle, register transactions through the I2C bus
 * framework (rt_i2c_transfer), the data-ready line through the PIN framework and
 * sample time through the "timebase" device.
 *
 * The Bosch SensorAPI is fused into this file: the part code supplies register access,
 * OTP/trim and compensation; the driver below owns the bus binding, the device
 * lifecycle and the M2-facing read/control interface.
 */

#include "bmp585.h"

static uint32_t read_le24(const uint8_t *data)
{
    return ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
}

sensor_status_t bmp585_decode_payload(const uint8_t data[6], BarometerReading *reading)
{
    if (!data || !reading)
        return SENSOR_ERR_ARGUMENT;
    /* The device emits temperature followed by pressure. */
    reading->temperature_c = (double)read_le24(data) / 65536.0;
    reading->pressure_pa = (double)read_le24(data + 3) / 64.0;
    return SENSOR_OK;
}

#if !defined(FLIGHT_DRIVER_CODEC_TEST)
#include "timer_capture.h"

#include <board.h>
#include <string.h>

#include <rtdevice.h>
#include <rthw.h>
#include <math.h>

/*
 * Bosch SensorAPI provenance
 * ----------------------------
 * Upstream : https://github.com/boschsensortec/BMP5_SensorAPI
 * Commit   : 3bd7c260419e4d08d7fa28343f27e0e6dd276411
 * Files    : bmp5_defs.h + bmp5.c (BSD-3-Clause)
 * Fused    : 19 of 47 functions kept (the call closure of this driver).
 * Dropped  : bmp5_extract_fifo_data, bmp5_get_fifo_configuration, bmp5_get_fifo_data, bmp5_get_fifo_len, bmp5_get_oor_configuration, bmp5_get_osr_odr_eff, bmp5_get_sensor_data, bmp5_nvm_read, bmp5_nvm_write, bmp5_set_fifo_configuration, bmp5_set_iir_config, bmp5_set_oor_configuration, bmp5_soft_reset, get_nvm_data, is_fifo_frame_empty, nvm_write_addr, parse_raw_pressure, parse_raw_temperature, set_fifo_iir_config, set_fifo_threshold, set_iir_config, set_nvm_addr, set_oor_iir_count_limit, unpack_press_temp_frame, unpack_pressure_frame, unpack_sensor_data, unpack_temperature_frame, validate_fifo_frame
 */

/* ===== Bosch definitions, fused from bmp5_defs.h ===== */

/*! @file bmp5_defs.h
 *  @brief Sensor Driver for BMP5 sensor
 */

/******************************************************************************/
/*! @name       HEADER INCLUDES                           */
/******************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/******************************************************************************/
/*! @name       COMMON MACROS                         */
/******************************************************************************/
#if (!defined(UINT8_C) && !defined(INT8_C))
#endif
#if (!defined(UINT16_C) && !defined(INT16_C))
#endif
#if (!defined(INT32_C) && !defined(UINT32_C))
#endif
#if (!defined(INT64_C) && !defined(UINT64_C))
#endif

#if (!defined(UINT8_C) && !defined(INT8_C))
#define INT8_C(x)    (x)
#define UINT8_C(x)   (x##U)
#endif

#if (!defined(UINT16_C) && !defined(INT16_C))
#define INT16_C(x)   (x)
#define UINT16_C(x)  (x##U)
#endif

#if (!defined(INT32_C) && !defined(UINT32_C))
#define INT32_C(x)   (x)
#define UINT32_C(x)  (x##U)
#endif

#if (!defined(INT64_C) && !defined(UINT64_C))
#define INT64_C(x)   (x##LL)
#define UINT64_C(x)  (x##ULL)
#endif


/*! @name C standard macros */
#ifndef NULL
#define NULL         ((void *) 0)
#endif

/******************************************************************************/
/*! @name        Compiler switch macros Definitions                */
/******************************************************************************/

/* Note: In case neither BMP5_USE_FIXED_POINT nor BMP5_USE_FLOATING_POINT was defined by the build system,
 * floating-point arithmetic (float) will be used by default.
 *
 * WARNING: On targets without a hardware FPU (e.g. Cortex-M0, M0+, M3) every floating-point
 * operation is emulated in software (soft-float), adding 20-100 cycles per arithmetic
 * instruction.  If your target lacks a hardware FPU, define BMP5_USE_FIXED_POINT in your
 * build system (compiler flag: -DBMP5_USE_FIXED_POINT) to avoid this overhead.
 */
#ifndef BMP5_USE_FIXED_POINT /*< Check if floating point (using BMP5_USE_FIXED_POINT) is enabled */
#ifndef BMP5_USE_FLOATING_POINT /*< If fixed point is not enabled then enable BMP5_USE_FLOATING_POINT */
#define BMP5_USE_FLOATING_POINT
#endif
#endif

#ifdef BMP5_USE_FIXED_POINT /*< If Fixed point is defined set BMP5_FIXED_POINT_DIGIT_PRECISION */
#ifdef BMP5_FIXED_POINT_DIGIT_PRECISION
#if BMP5_FIXED_POINT_DIGIT_PRECISION > 6 /* If BMP5_FIXED_POINT_DIGIT_PRECISION(provided by user) is greater than 6 then
                                          * set precision value to 6 digits */
#undef BMP5_FIXED_POINT_DIGIT_PRECISION
#define BMP5_FIXED_POINT_DIGIT_PRECISION  UINT8_C(6)
#endif
#endif

/* By default BMP5_FIXED_POINT_DIGIT_PRECISION will be 6 digits */
/* User can vary the precision between 0 to 6 digits */
#ifndef BMP5_FIXED_POINT_DIGIT_PRECISION
#define BMP5_FIXED_POINT_DIGIT_PRECISION  UINT8_C(6)
#endif
#endif

/******************************************************************************/
/*! @name        GENERAL MACRO DEFINITIONS                */
/******************************************************************************/

/*!
 * BMP5_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BMP5_INTF_RET_TYPE
#define BMP5_INTF_RET_TYPE                        int8_t
#endif

/*!
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMP5_INTF_RET_SUCCESS
#define BMP5_INTF_RET_SUCCESS                     INT8_C(0)
#endif

/*! @name BOOLEAN TYPES */
#ifndef TRUE
#define TRUE                                      UINT8_C(0x01)
#endif

#ifndef FALSE
#define FALSE                                     UINT8_C(0x00)
#endif

#ifndef ABS
#define ABS(a)                                    ((a) > 0 ? (a) : -(a))    /*!< Absolute value */
#endif

/*! @name UTILITY MACROS */
#define BMP5_SET_LOW_BYTE                         UINT16_C(0x00FF)
#define BMP5_SET_HIGH_BYTE                        UINT16_C(0xFF00)

/*! @name BIT SLICE GET AND SET FUNCTIONS */
#define BMP5_GET_BITSLICE(regvar, bitname) \
    (((regvar) & bitname##_MSK) >> bitname##_POS)

#define BMP5_SET_BITSLICE(regvar, bitname, val) \
    (((regvar) & ~bitname##_MSK) | \
     (((val) << bitname##_POS) & bitname##_MSK))

#define BMP5_GET_LSB(var)                         (uint8_t)((var) & BMP5_SET_LOW_BYTE)
#define BMP5_GET_MSB(var)                         (uint8_t)(((var) & BMP5_SET_HIGH_BYTE) >> 8)

#define BMP5_SET_BIT_VAL_0(reg_data, bitname)     ((reg_data) & ~(bitname##_MSK))

#define BMP5_SET_BITS_POS_0(reg_data, bitname, data) \
    (((reg_data) & ~(bitname##_MSK)) | \
     ((data) & (bitname##_MSK)))

#define BMP5_GET_BITS_POS_0(reg_data, bitname)    ((reg_data) & (bitname##_MSK))

/*! @name Chip id of BMP5 */
#define BMP5_CHIP_ID_PRIM                         UINT8_C(0x50)
#define BMP5_CHIP_ID_SEC                          UINT8_C(0x51)

/*! @name API success code */
#define BMP5_OK                                   INT8_C(0)

/*! @name API error codes */
#define BMP5_E_NULL_PTR                           INT8_C(-1)
#define BMP5_E_COM_FAIL                           INT8_C(-2)
#define BMP5_E_DEV_NOT_FOUND                      INT8_C(-3)
#define BMP5_E_INVALID_CHIP_ID                    INT8_C(-4)
#define BMP5_E_NVM_NOT_READY                      INT8_C(-5)
#define BMP5_E_POR_SOFTRESET                      INT8_C(-6)
#define BMP5_E_INVALID_POWERMODE                  INT8_C(-7)
#define BMP5_E_INVALID_THRESHOLD                  INT8_C(-8)
#define BMP5_E_FIFO_FRAME_EMPTY                   INT8_C(-9)
#define BMP5_E_NVM_INVALID_ADDR                   INT8_C(-10)

#define BMP5_ENABLE                               UINT8_C(0x01)
#define BMP5_DISABLE                              UINT8_C(0x00)

/*! @name Register addresses */
#define BMP5_REG_CHIP_ID                          UINT8_C(0x01)
#define BMP5_REG_REV_ID                           UINT8_C(0x02)
#define BMP5_REG_CHIP_STATUS                      UINT8_C(0x11)
#define BMP5_REG_DRIVE_CONFIG                     UINT8_C(0x13)
#define BMP5_REG_INT_CONFIG                       UINT8_C(0x14)
#define BMP5_REG_INT_SOURCE                       UINT8_C(0x15)
#define BMP5_REG_FIFO_CONFIG                      UINT8_C(0x16)
#define BMP5_REG_FIFO_COUNT                       UINT8_C(0x17)
#define BMP5_REG_FIFO_SEL                         UINT8_C(0x18)
#define BMP5_REG_TEMP_DATA_XLSB                   UINT8_C(0x1D)
#define BMP5_REG_TEMP_DATA_LSB                    UINT8_C(0x1E)
#define BMP5_REG_TEMP_DATA_MSB                    UINT8_C(0x1F)
#define BMP5_REG_PRESS_DATA_XLSB                  UINT8_C(0x20)
#define BMP5_REG_PRESS_DATA_LSB                   UINT8_C(0x21)
#define BMP5_REG_PRESS_DATA_MSB                   UINT8_C(0x22)
#define BMP5_REG_INT_STATUS                       UINT8_C(0x27)
#define BMP5_REG_STATUS                           UINT8_C(0x28)
#define BMP5_REG_FIFO_DATA                        UINT8_C(0x29)
#define BMP5_REG_NVM_ADDR                         UINT8_C(0x2B)
#define BMP5_REG_NVM_DATA_LSB                     UINT8_C(0x2C)
#define BMP5_REG_NVM_DATA_MSB                     UINT8_C(0x2D)
#define BMP5_REG_DSP_CONFIG                       UINT8_C(0x30)
#define BMP5_REG_DSP_IIR                          UINT8_C(0x31)
#define BMP5_REG_OOR_THR_P_LSB                    UINT8_C(0x32)
#define BMP5_REG_OOR_THR_P_MSB                    UINT8_C(0x33)
#define BMP5_REG_OOR_RANGE                        UINT8_C(0x34)
#define BMP5_REG_OOR_CONFIG                       UINT8_C(0x35)
#define BMP5_REG_OSR_CONFIG                       UINT8_C(0x36)
#define BMP5_REG_ODR_CONFIG                       UINT8_C(0x37)
#define BMP5_REG_OSR_EFF                          UINT8_C(0x38)
#define BMP5_REG_CMD                              UINT8_C(0x7E)

/*! @name I2C addresses */
#define BMP5_I2C_ADDR_PRIM                        UINT8_C(0x46)
#define BMP5_I2C_ADDR_SEC                         UINT8_C(0x47)

/*! @name NVM addresses */
#define BMP5_NVM_START_ADDR                       UINT8_C(0x20)
#define BMP5_NVM_END_ADDR                         UINT8_C(0x22)

/*! @name Interface settings */
#define BMP5_SPI_RD_MASK                          UINT8_C(0x80)

/*! @name Delay definition */
#define BMP5_DELAY_US_SOFT_RESET                  UINT16_C(2000)
#define BMP5_DELAY_US_STANDBY                     UINT16_C(2500)
#define BMP5_DELAY_US_NVM_READY_READ              UINT8_C(800)
#define BMP5_DELAY_US_NVM_READY_WRITE             UINT16_C(10000)

/*! @name Soft reset command */
#define BMP5_SOFT_RESET_CMD                       UINT8_C(0xB6)

/*! NVM command */
#define BMP5_NVM_FIRST_CMND                       UINT8_C(0x5D)
#define BMP5_NVM_READ_ENABLE_CMND                 UINT8_C(0xA5)
#define BMP5_NVM_WRITE_ENABLE_CMND                UINT8_C(0xA0)

/*! @name Deepstandby enable/disable */
#define BMP5_DEEP_ENABLED                         UINT8_C(0)
#define BMP5_DEEP_DISABLED                        UINT8_C(1)

/*! @name ODR settings */
#define BMP5_ODR_240_HZ                           UINT8_C(0x00)
#define BMP5_ODR_218_5_HZ                         UINT8_C(0x01)
#define BMP5_ODR_199_1_HZ                         UINT8_C(0x02)
#define BMP5_ODR_179_2_HZ                         UINT8_C(0x03)
#define BMP5_ODR_160_HZ                           UINT8_C(0x04)
#define BMP5_ODR_149_3_HZ                         UINT8_C(0x05)
#define BMP5_ODR_140_HZ                           UINT8_C(0x06)
#define BMP5_ODR_129_8_HZ                         UINT8_C(0x07)
#define BMP5_ODR_120_HZ                           UINT8_C(0x08)
#define BMP5_ODR_110_1_HZ                         UINT8_C(0x09)
#define BMP5_ODR_100_2_HZ                         UINT8_C(0x0A)
#define BMP5_ODR_89_6_HZ                          UINT8_C(0x0B)
#define BMP5_ODR_80_HZ                            UINT8_C(0x0C)
#define BMP5_ODR_70_HZ                            UINT8_C(0x0D)
#define BMP5_ODR_60_HZ                            UINT8_C(0x0E)
#define BMP5_ODR_50_HZ                            UINT8_C(0x0F)
#define BMP5_ODR_45_HZ                            UINT8_C(0x10)
#define BMP5_ODR_40_HZ                            UINT8_C(0x11)
#define BMP5_ODR_35_HZ                            UINT8_C(0x12)
#define BMP5_ODR_30_HZ                            UINT8_C(0x13)
#define BMP5_ODR_25_HZ                            UINT8_C(0x14)
#define BMP5_ODR_20_HZ                            UINT8_C(0x15)
#define BMP5_ODR_15_HZ                            UINT8_C(0x16)
#define BMP5_ODR_10_HZ                            UINT8_C(0x17)
#define BMP5_ODR_05_HZ                            UINT8_C(0x18)
#define BMP5_ODR_04_HZ                            UINT8_C(0x19)
#define BMP5_ODR_03_HZ                            UINT8_C(0x1A)
#define BMP5_ODR_02_HZ                            UINT8_C(0x1B)
#define BMP5_ODR_01_HZ                            UINT8_C(0x1C)
#define BMP5_ODR_0_5_HZ                           UINT8_C(0x1D)
#define BMP5_ODR_0_250_HZ                         UINT8_C(0x1E)
#define BMP5_ODR_0_125_HZ                         UINT8_C(0x1F)

/*! @name Oversampling for temperature and pressure */
#define BMP5_OVERSAMPLING_1X                      UINT8_C(0x00)
#define BMP5_OVERSAMPLING_2X                      UINT8_C(0x01)
#define BMP5_OVERSAMPLING_4X                      UINT8_C(0x02)
#define BMP5_OVERSAMPLING_8X                      UINT8_C(0x03)
#define BMP5_OVERSAMPLING_16X                     UINT8_C(0x04)
#define BMP5_OVERSAMPLING_32X                     UINT8_C(0x05)
#define BMP5_OVERSAMPLING_64X                     UINT8_C(0x06)
#define BMP5_OVERSAMPLING_128X                    UINT8_C(0x07)

/*! @name IIR filter for temperature and pressure */
#define BMP5_IIR_FILTER_BYPASS                    UINT8_C(0x00)
#define BMP5_IIR_FILTER_COEFF_1                   UINT8_C(0x01)
#define BMP5_IIR_FILTER_COEFF_3                   UINT8_C(0x02)
#define BMP5_IIR_FILTER_COEFF_7                   UINT8_C(0x03)
#define BMP5_IIR_FILTER_COEFF_15                  UINT8_C(0x04)
#define BMP5_IIR_FILTER_COEFF_31                  UINT8_C(0x05)
#define BMP5_IIR_FILTER_COEFF_63                  UINT8_C(0x06)
#define BMP5_IIR_FILTER_COEFF_127                 UINT8_C(0x07)

/*! Fifo frame configuration */
#define BMP5_FIFO_EMPTY                           UINT8_C(0X7F)
#define BMP5_FIFO_MAX_THRESHOLD_P_T_MODE          UINT8_C(0x0F)
#define BMP5_FIFO_MAX_THRESHOLD_P_MODE            UINT8_C(0x1F)

/*! @name Macro is used to bypass both iir_t and iir_p together */
#define BMP5_IIR_BYPASS                           UINT8_C(0xC0)

/*! @name Pressure Out-of-range count limit */
#define BMP5_OOR_COUNT_LIMIT_1                    UINT8_C(0x00)
#define BMP5_OOR_COUNT_LIMIT_3                    UINT8_C(0x01)
#define BMP5_OOR_COUNT_LIMIT_7                    UINT8_C(0x02)
#define BMP5_OOR_COUNT_LIMIT_15                   UINT8_C(0x03)

/*! @name Interrupt configurations */
#define BMP5_INT_MODE_PULSED                      UINT8_C(0)
#define BMP5_INT_MODE_LATCHED                     UINT8_C(1)

#define BMP5_INT_POL_ACTIVE_LOW                   UINT8_C(0)
#define BMP5_INT_POL_ACTIVE_HIGH                  UINT8_C(1)

#define BMP5_INT_OD_PUSHPULL                      UINT8_C(0)
#define BMP5_INT_OD_OPENDRAIN                     UINT8_C(1)

/*! @name NVM and Interrupt status asserted macros */
#define BMP5_INT_ASSERTED_DRDY                    UINT8_C(0x01)
#define BMP5_INT_ASSERTED_FIFO_FULL               UINT8_C(0x02)
#define BMP5_INT_ASSERTED_FIFO_THRES              UINT8_C(0x04)
#define BMP5_INT_ASSERTED_PRESSURE_OOR            UINT8_C(0x08)
#define BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE  UINT8_C(0x10)
#define BMP5_INT_NVM_RDY                          UINT8_C(0x02)
#define BMP5_INT_NVM_ERR                          UINT8_C(0x04)
#define BMP5_INT_NVM_CMD_ERR                      UINT8_C(0x08)

/*! @name Interrupt configurations */
#define BMP5_INT_MODE_MSK                         UINT8_C(0x01)

#define BMP5_INT_POL_MSK                          UINT8_C(0x02)
#define BMP5_INT_POL_POS                          UINT8_C(1)

#define BMP5_INT_OD_MSK                           UINT8_C(0x04)
#define BMP5_INT_OD_POS                           UINT8_C(2)

#define BMP5_INT_EN_MSK                           UINT8_C(0x08)
#define BMP5_INT_EN_POS                           UINT8_C(3)

#define BMP5_INT_DRDY_EN_MSK                      UINT8_C(0x01)

#define BMP5_INT_FIFO_FULL_EN_MSK                 UINT8_C(0x02)
#define BMP5_INT_FIFO_FULL_EN_POS                 UINT8_C(1)

#define BMP5_INT_FIFO_THRES_EN_MSK                UINT8_C(0x04)
#define BMP5_INT_FIFO_THRES_EN_POS                UINT8_C(2)

#define BMP5_INT_OOR_PRESS_EN_MSK                 UINT8_C(0x08)
#define BMP5_INT_OOR_PRESS_EN_POS                 UINT8_C(3)

/*! @name ODR configuration */
#define BMP5_ODR_MSK                              UINT8_C(0x7C)
#define BMP5_ODR_POS                              UINT8_C(2)

/*! @name OSR configurations */
#define BMP5_TEMP_OS_MSK                          UINT8_C(0x07)

#define BMP5_PRESS_OS_MSK                         UINT8_C(0x38)
#define BMP5_PRESS_OS_POS                         UINT8_C(3)

/*! @name Pressure enable */
#define BMP5_PRESS_EN_MSK                         UINT8_C(0x40)
#define BMP5_PRESS_EN_POS                         UINT8_C(6)

/*! @name IIR configurations */
#define BMP5_SET_IIR_TEMP_MSK                     UINT8_C(0x07)
#define BMP5_SET_IIR_TEMP_POS                     UINT8_C(0)

#define BMP5_SET_IIR_PRESS_MSK                    UINT8_C(0x38)
#define BMP5_SET_IIR_PRESS_POS                    UINT8_C(3)

#define BMP5_OOR_SEL_IIR_PRESS_MSK                UINT8_C(0x80)
#define BMP5_OOR_SEL_IIR_PRESS_POS                UINT8_C(7)

#define BMP5_SHDW_SET_IIR_TEMP_MSK                UINT8_C(0x08)
#define BMP5_SHDW_SET_IIR_TEMP_POS                UINT8_C(3)

#define BMP5_SHDW_SET_IIR_PRESS_MSK               UINT8_C(0x20)
#define BMP5_SHDW_SET_IIR_PRESS_POS               UINT8_C(5)

#define BMP5_SET_FIFO_IIR_TEMP_MSK                UINT8_C(0x10)
#define BMP5_SET_FIFO_IIR_TEMP_POS                UINT8_C(4)

#define BMP5_SET_FIFO_IIR_PRESS_MSK               UINT8_C(0x40)
#define BMP5_SET_FIFO_IIR_PRESS_POS               UINT8_C(6)

#define BMP5_IIR_FLUSH_FORCED_EN_MSK              UINT8_C(0x04)
#define BMP5_IIR_FLUSH_FORCED_EN_POS              UINT8_C(2)

/*! @name Effective OSR configurations and ODR valid status */
#define BMP5_OSR_TEMP_EFF_MSK                     UINT8_C(0x07)

#define BMP5_OSR_PRESS_EFF_MSK                    UINT8_C(0x38)
#define BMP5_OSR_PRESS_EFF_POS                    UINT8_C(3)

#define BMP5_ODR_IS_VALID_MSK                     UINT8_C(0x80)
#define BMP5_ODR_IS_VALID_POS                     UINT8_C(7)

/*! @name Powermode */
#define BMP5_POWERMODE_MSK                        UINT8_C(0x03)

#define BMP5_DEEP_DISABLE_MSK                     UINT8_C(0x80)
#define BMP5_DEEP_DISABLE_POS                     UINT8_C(7)

/*! @name Fifo configurations */
#define BMP5_FIFO_THRESHOLD_MSK                   UINT8_C(0x1F)

#define BMP5_FIFO_MODE_MSK                        UINT8_C(0x20)
#define BMP5_FIFO_MODE_POS                        UINT8_C(5)

#define BMP5_FIFO_DEC_SEL_MSK                     UINT8_C(0x1C)
#define BMP5_FIFO_DEC_SEL_POS                     UINT8_C(2)

#define BMP5_FIFO_COUNT_MSK                       UINT8_C(0x3F)

#define BMP5_FIFO_FRAME_SEL_MSK                   UINT8_C(0x03)

/*! @name Out-of-range configuration */
#define BMP5_OOR_THR_P_LSB_MSK                    UINT32_C(0x0000FF)

#define BMP5_OOR_THR_P_MSB_MSK                    UINT32_C(0x00FF00)

#define BMP5_OOR_THR_P_XMSB_MSK                   UINT32_C(0x010000)
#define BMP5_OOR_THR_P_XMSB_POS                   UINT16_C(16)

/* Macro to mask xmsb value of oor threshold from register(0x35) value */
#define BMP5_OOR_THR_P_XMSB_REG_MSK               UINT8_C(0x01)

#define BMP5_OOR_COUNT_LIMIT_MSK                  UINT8_C(0xC0)
#define BMP5_OOR_COUNT_LIMIT_POS                  UINT8_C(6)

/*! @name NVM configuration */
#define BMP5_NVM_ADDR_MSK                         UINT8_C(0x3F)

#define BMP5_NVM_PROG_EN_MSK                      UINT8_C(0x40)
#define BMP5_NVM_PROG_EN_POS                      UINT8_C(6)

#define BMP5_NVM_DATA_LSB_MSK                     UINT16_C(0x00FF)

#define BMP5_NVM_DATA_MSB_MSK                     UINT16_C(0xFF00)

/******************************************************************************/
/*!  @name         TYPEDEF DEFINITIONS                                        */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] read_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef BMP5_INTF_RET_TYPE (*bmp5_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] read_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef BMP5_INTF_RET_TYPE (*bmp5_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*bmp5_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*!  @name         ENUMERATION DEFINITIONS                                      */
/******************************************************************************/

/*!
 * @brief Enumerator for interface selection
 */
enum bmp5_intf {
    /*! SPI interface */
    BMP5_SPI_INTF,
    /*! I2C interface */
    BMP5_I2C_INTF,
    /*! I3C interface */
    BMP5_I3C_INTF
};

/*!
 * @brief Enumerator for powermode selection
 */
enum bmp5_powermode {
    /*! Standby powermode */
    BMP5_POWERMODE_STANDBY,
    /*! Normal powermode */
    BMP5_POWERMODE_NORMAL,
    /*! Forced powermode */
    BMP5_POWERMODE_FORCED,
    /*! Continuous powermode */
    BMP5_POWERMODE_CONTINUOUS,
    /*! Deep standby powermode */
    BMP5_POWERMODE_DEEP_STANDBY
};

/*!
 * @brief Enumerator for interrupt enable disable
 */
enum bmp5_intr_en_dis {
    /*! Interrupt diable */
    BMP5_INTR_DISABLE = BMP5_DISABLE,
    /*! Interrupt enable */
    BMP5_INTR_ENABLE = BMP5_ENABLE
};

/*!
 * @brief Enumerator for interrupt mode
 */
enum bmp5_intr_mode {
    /*! Interrupt mode - pulsed */
    BMP5_PULSED = BMP5_INT_MODE_PULSED,
    /*! Interrupt mode - latched */
    BMP5_LATCHED = BMP5_INT_MODE_LATCHED
};

/*!
 * @brief Enumerator for interrupt polarity
 */
enum bmp5_intr_polarity {
    /*! Interrupt polarity - active low */
    BMP5_ACTIVE_LOW = BMP5_INT_POL_ACTIVE_LOW,
    /*! Interrupt polarity - active high */
    BMP5_ACTIVE_HIGH = BMP5_INT_POL_ACTIVE_HIGH
};

/*!
 * @brief Enumerator for interrupt drive
 */
enum bmp5_intr_drive {
    /*! Interrupt drive - push-pull */
    BMP5_INTR_PUSH_PULL = BMP5_INT_OD_PUSHPULL,
    /*! Interrupt drive - open drain */
    BMP5_INTR_OPEN_DRAIN = BMP5_INT_OD_OPENDRAIN
};

/*!
 * @brief Enumerator for fifo frame selection
 */
enum bmp5_fifo_frame_sel {
    /*! Fifo disabled */
    BMP5_FIFO_NOT_ENABLED,
    /*! Fifo temperature data only enabled */
    BMP5_FIFO_TEMPERATURE_DATA,
    /*! Fifo pressure data only enabled */
    BMP5_FIFO_PRESSURE_DATA,
    /*! Fifo pressure and temperature data enabled */
    BMP5_FIFO_PRESS_TEMP_DATA
};

/*!
 * @brief Enumerator for fifo decimation factor(downsampling) selection
 */
enum bmp5_fifo_dec_sel {
    /*! Fifo downsampling disabled */
    BMP5_FIFO_NO_DOWNSAMPLING,
    /*! Fifo 2X downsampling */
    BMP5_FIFO_DOWNSAMPLING_2X,
    /*! Fifo 4X downsampling */
    BMP5_FIFO_DOWNSAMPLING_4X,
    /*! Fifo 8X downsampling */
    BMP5_FIFO_DOWNSAMPLING_8X,
    /*! Fifo 16X downsampling */
    BMP5_FIFO_DOWNSAMPLING_16X,
    /*! Fifo 32X downsampling */
    BMP5_FIFO_DOWNSAMPLING_32X,
    /*! Fifo 64X downsampling */
    BMP5_FIFO_DOWNSAMPLING_64X,
    /*! Fifo 128X downsampling */
    BMP5_FIFO_DOWNSAMPLING_128X
};

/*!
 * @brief Enumerator for fifo mode selection
 */
enum bmp5_fifo_mode {
    /*! Fifo mode - streaming */
    BMP5_FIFO_MODE_STREAMING,
    /*! Fifo mode - stop on full */
    BMP5_FIFO_MODE_STOP_ON_FULL
};

/******************************************************************************/
/*!  @name         STRUCTURE DEFINITIONS                                      */
/******************************************************************************/

/*!
 * @brief OSR, ODR and pressure configuration structure
 */
struct bmp5_osr_odr_press_config
{
    /*! Temperature oversampling
     * Assignable macros :
     * - BMP5_OVERSAMPLING_1X
     * - BMP5_OVERSAMPLING_2X
     * - BMP5_OVERSAMPLING_4X
     * - BMP5_OVERSAMPLING_8X
     * - BMP5_OVERSAMPLING_16X
     * - BMP5_OVERSAMPLING_32X
     * - BMP5_OVERSAMPLING_64X
     * - BMP5_OVERSAMPLING_128X
     */
    uint8_t osr_t;

    /*! Pressure oversampling
     * Assignable macros :
     * - BMP5_OVERSAMPLING_1X
     * - BMP5_OVERSAMPLING_2X
     * - BMP5_OVERSAMPLING_4X
     * - BMP5_OVERSAMPLING_8X
     * - BMP5_OVERSAMPLING_16X
     * - BMP5_OVERSAMPLING_32X
     * - BMP5_OVERSAMPLING_64X
     * - BMP5_OVERSAMPLING_128X
     */
    uint8_t osr_p;

    /*! Enable pressure
     * BMP5_ENABLE  = Enables pressure data
     * BMP5_DISABLE = Disables pressure data
     */
    uint8_t press_en;

    /*! Output Data Rate */
    uint8_t odr;
};

/*!
 * @brief IIR configuration structure
 */
struct bmp5_iir_config
{
    /*! Temperature IIR
     * Assignable macros :
     * - BMP5_IIR_FILTER_BYPASS
     * - BMP5_IIR_FILTER_COEFF_1
     * - BMP5_IIR_FILTER_COEFF_3
     * - BMP5_IIR_FILTER_COEFF_7
     * - BMP5_IIR_FILTER_COEFF_15
     * - BMP5_IIR_FILTER_COEFF_31
     * - BMP5_IIR_FILTER_COEFF_63
     * - BMP5_IIR_FILTER_COEFF_127
     */
    uint8_t set_iir_t;

    /*! Pressure IIR
     * Assignable macros :
     * - BMP5_IIR_FILTER_BYPASS
     * - BMP5_IIR_FILTER_COEFF_1
     * - BMP5_IIR_FILTER_COEFF_3
     * - BMP5_IIR_FILTER_COEFF_7
     * - BMP5_IIR_FILTER_COEFF_15
     * - BMP5_IIR_FILTER_COEFF_31
     * - BMP5_IIR_FILTER_COEFF_63
     * - BMP5_IIR_FILTER_COEFF_127
     */
    uint8_t set_iir_p;

    /*! Shadow IIR selection for temperature
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t shdw_set_iir_t;

    /*! Shadow IIR selection for pressure
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t shdw_set_iir_p;

    /*! IIR flush in forced mode enable
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t iir_flush_forced_en;
};

/*!
 * @brief Effective OSR configuration and ODR valid status structure
 */
struct bmp5_osr_odr_eff
{
    /*! Effective temperature OSR */
    uint8_t osr_t_eff;

    /*! Effective pressure OSR */
    uint8_t osr_p_eff;

    /*! If asserted, the ODR parametrization is valid */
    uint8_t odr_is_valid;
};

/*!
 * @brief BMP5 interrupt source selection.
 */
struct bmp5_int_source_select
{
    /*! Data ready interrupt enable
     * BMP5_ENABLE  = Enables data ready interrupt
     * BMP5_DISABLE = Disables data ready interrupt
     */
    uint8_t drdy_en;

    /*! Fifo full interrupt enable
     * BMP5_ENABLE  = Enables fifo full interrupt
     * BMP5_DISABLE = Disables fifo full interrupt
     */
    uint8_t fifo_full_en;

    /*! Fifo threshold interrupt enable
     * BMP5_ENABLE  = Enables fifo threshold interrupt
     * BMP5_DISABLE = Disables fifo threshold interrupt
     */
    uint8_t fifo_thres_en;

    /*! Pressure out of range interrupt enable
     * BMP5_ENABLE  = Enables pressure out of range interrupt
     * BMP5_DISABLE = Disables pressure out of range interrupt
     */
    uint8_t oor_press_en;
};

/*!
 * @brief BMP5 fifo configurations.
 */
struct bmp5_fifo
{
    /*! Pointer to fifo data */
    uint8_t *data;

    /*! Length of user defined bytes of fifo to be read */
    uint16_t length;

    /*! Fifo frame data source selection
     * Assignable macros :
     * - BMP5_FIFO_NOT_ENABLED
     * - BMP5_FIFO_TEMPERATURE_DATA
     * - BMP5_FIFO_PRESSURE_DATA
     * - BMP5_FIFO_PRESS_TEMP_DATA
     */
    uint8_t frame_sel;

    /*! Fifo decimation factor(downsampling) selection
     * Assignable macros :
     * - BMP5_FIFO_NO_DOWNSAMPLING
     * - BMP5_FIFO_DOWNSAMPLING_2X
     * - BMP5_FIFO_DOWNSAMPLING_4X
     * - BMP5_FIFO_DOWNSAMPLING_8X
     * - BMP5_FIFO_DOWNSAMPLING_16X
     * - BMP5_FIFO_DOWNSAMPLING_32X
     * - BMP5_FIFO_DOWNSAMPLING_64X
     * - BMP5_FIFO_DOWNSAMPLING_128X
     */
    uint8_t dec_sel;

    /*! Fifo frame count */
    uint8_t fifo_count;

    /*! Fifo mode selection
     * Assignable macros :
     * - BMP5_FIFO_MODE_STREAMING
     * - BMP5_FIFO_MODE_STOP_ON_FULL
     */
    uint8_t mode;

    /*! Threshold for fifo */
    uint8_t threshold;

    /*! Fifo temperature IIR
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t set_fifo_iir_t;

    /*! Fifo pressure IIR
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t set_fifo_iir_p;
};

/*!
 * @brief BMP5 Out-of-range pressure configuration.
 */
struct bmp5_oor_press_configuration
{
    /*! Out-of-range pressure threshold */
    uint32_t oor_thr_p;

    /*! Out-of-range pressure range
     *  Range can span up to +/- 255 Pa around the threshold value.
     */
    uint8_t oor_range_p;

    /*! Out-of-range pressure count limit
     * Assignable macros :
     * - BMP5_OOR_COUNT_LIMIT_1
     * - BMP5_OOR_COUNT_LIMIT_3
     * - BMP5_OOR_COUNT_LIMIT_7
     * - BMP5_OOR_COUNT_LIMIT_15
     */
    uint8_t cnt_lim;

    /*! Out-of-range pressure IIR
     * Assignable macros :
     * - BMP5_ENABLE
     * - BMP5_DISABLE
     */
    uint8_t oor_sel_iir_p;
};

/*!
 * @brief BMP5 sensor data structure which comprises of temperature and pressure.
 */


/*!
 * @brief BMP5 sensor data structure which comprises of temperature and pressure in floating point with data type as
 *  float for pressure and temperature.
 */
struct bmp5_sensor_data
{
    /*! Pressure data */
    float pressure;

    /*! Temperature data */
    float temperature;
};


/*!
 * @brief API device structure
 */
struct bmp5_dev
{
    /*! Chip ID */
    uint8_t chip_id;

    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /*! Read function pointer */
    bmp5_read_fptr_t read;

    /*! Write function pointer */
    bmp5_write_fptr_t write;

    /*! Delay function pointer */
    bmp5_delay_us_fptr_t delay_us;

    /*! To store interface pointer error */
    BMP5_INTF_RET_TYPE intf_rslt;

    /*! Type of Interface  */
    enum bmp5_intf intf;
};


/* ===== Bosch SensorAPI, fused from bmp5.c ===== */
static int8_t null_ptr_check(const struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMP5_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMP5_OK;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate the chip id of the sensor.
 */

static int8_t bmp5_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMP5_OK) && (reg_data != NULL))
    {
        if (dev->intf == BMP5_SPI_INTF)
        {
            /* If interface selected is SPI */
            reg_addr = reg_addr | BMP5_SPI_RD_MASK;
        }

        /* Read the data from the reg_addr */
        dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != BMP5_INTF_RET_SUCCESS)
        {
            /* Failure case */
            rslt = BMP5_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */

static int8_t bmp5_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t idx = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMP5_OK) && (reg_data != NULL))
    {
        if (dev->intf == BMP5_SPI_INTF)
        {
            /* Write the data to the reg_addr */
            do
            {
                dev->intf_rslt = dev->write((reg_addr + idx), &reg_data[idx], 1, dev->intf_ptr);
                idx++;
            } while ((idx < len) && (dev->intf_rslt == BMP5_INTF_RET_SUCCESS));
        }
        else if ((dev->intf == BMP5_I2C_INTF) || (dev->intf == BMP5_I3C_INTF))
        {
            /* Write the data to the reg_addr */
            dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);
        }

        if (dev->intf_rslt != BMP5_INTF_RET_SUCCESS)
        {
            /* Failure case */
            rslt = BMP5_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */

static int8_t bmp5_configure_interrupt(enum bmp5_intr_mode int_mode,
                                enum bmp5_intr_polarity int_pol,
                                enum bmp5_intr_drive int_od,
                                enum bmp5_intr_en_dis int_en,
                                struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt = BMP5_OK;

    /* Variable to get interrupt configuration */
    uint8_t reg_data = 0;

    /* Variable to set interrupt source */
    uint8_t int_source = 0;

    /* Variable to get interrupt status */
    uint8_t int_status = 0;

    /* Get interrupt configuration */
    rslt = bmp5_get_regs(BMP5_REG_INT_CONFIG, &reg_data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Any change between latched/pulsed mode has to be applied while interrupt is disabled */
        /* Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00) */
        rslt = bmp5_set_regs(BMP5_REG_INT_SOURCE, &int_source, 1, dev);

        if (rslt == BMP5_OK)
        {
            /* Step 2 : Read the INT_STATUS register to clear the status */
            rslt = bmp5_get_regs(BMP5_REG_INT_STATUS, &int_status, 1, dev);

            if (rslt == BMP5_OK)
            {
                /* Step 3 : Set the desired mode in INT_CONFIG.int_mode */
                reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_INT_MODE, int_mode);
                reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_POL, int_pol);
                reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_OD, int_od);
                reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_EN, int_en);

                /* Finally transfer the interrupt configurations */
                rslt = bmp5_set_regs(BMP5_REG_INT_CONFIG, &reg_data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the configuration for oversampling of temperature, oversampling of
 *  pressure and ODR configuration along with pressure enable.
 */

static int8_t bmp5_get_iir_config(struct bmp5_iir_config *iir_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt = BMP5_OK;

    /* Variable to get IIR config */
    uint8_t reg_data[2];

    if (iir_cfg != NULL)
    {
        /* Get IIR configuration */
        rslt = bmp5_get_regs(BMP5_REG_DSP_CONFIG, reg_data, 2, dev);

        if (rslt == BMP5_OK)
        {
            iir_cfg->shdw_set_iir_t = BMP5_GET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_TEMP);
            iir_cfg->shdw_set_iir_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_SHDW_SET_IIR_PRESS);
            iir_cfg->iir_flush_forced_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_IIR_FLUSH_FORCED_EN);

            iir_cfg->set_iir_t = BMP5_GET_BITS_POS_0(reg_data[1], BMP5_SET_IIR_TEMP);
            iir_cfg->set_iir_p = BMP5_GET_BITSLICE(reg_data[1], BMP5_SET_IIR_PRESS);
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the configuration for IIR of temperature and pressure.
 *
 *  @note If IIR value for both temperature and pressure is set a value other than bypass then powermode is set
 *  as standby mode, as IIR with value other than bypass without disabling deep-standby mode makes powermode invalid.
 */

static int8_t bmp5_get_interrupt_status(uint8_t *int_status, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;

    if (int_status != NULL)
    {
        rslt = bmp5_get_regs(BMP5_REG_INT_STATUS, int_status, 1, dev);
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to get powermode of the sensor.
 */

static int8_t bmp5_get_osr_odr_press_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt = BMP5_OK;

    /* Variable to store OSR and ODR config */
    uint8_t reg_data[2];

    if (osr_odr_press_cfg != NULL)
    {
        /* Get OSR and ODR configuration in burst read */
        rslt = bmp5_get_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);

        if (rslt == BMP5_OK)
        {
            osr_odr_press_cfg->osr_t = BMP5_GET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS);
            osr_odr_press_cfg->osr_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_OS);
            osr_odr_press_cfg->press_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_EN);
            osr_odr_press_cfg->odr = BMP5_GET_BITSLICE(reg_data[1], BMP5_ODR);
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the configuration for oversampling temperature, oversampling of
 *  pressure and ODR configuration along with pressure enable.
 *
 *  @note If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
 *  without disabling deep-standby mode makes powermode invalid.
 */

static int8_t check_deepstandby_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t fifo_frame_sel = 0;
    struct bmp5_iir_config iir_cfg = { 0 };
    struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

    rslt = bmp5_get_regs(BMP5_REG_FIFO_SEL, &fifo_frame_sel, 1, dev);

    if (rslt == BMP5_OK)
    {
        fifo_frame_sel = BMP5_GET_BITS_POS_0(fifo_frame_sel, BMP5_FIFO_FRAME_SEL);

        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, dev);

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_iir_config(&iir_cfg, dev);
        }
    }

    /* As per datasheet odr should be less than 5Hz. But register value for 5Hz is less than 4Hz and so,
     * thus in this below condition odr is checked whether greater than 5Hz macro.
     */
    if (rslt == BMP5_OK)
    {
        if ((osr_odr_press_cfg.odr > BMP5_ODR_05_HZ) && (fifo_frame_sel == BMP5_DISABLE) &&
            (iir_cfg.set_iir_t == BMP5_IIR_FILTER_BYPASS) && (iir_cfg.set_iir_p == BMP5_IIR_FILTER_BYPASS))
        {
            *powermode = BMP5_POWERMODE_DEEP_STANDBY;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set sensor in deep standby mode.
 */

static int8_t bmp5_get_power_mode(enum bmp5_powermode *powermode, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t deep_dis;
    uint8_t reg_data;
    uint8_t pwrmode;

    if (powermode != NULL)
    {
        /* Read the power mode register */
        rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

        if (rslt == BMP5_OK)
        {
            pwrmode = BMP5_GET_BITS_POS_0(reg_data, BMP5_POWERMODE);

            switch (pwrmode)
            {
                case BMP5_POWERMODE_STANDBY:

                    /* Getting deep disable status */
                    deep_dis = BMP5_GET_BITSLICE(reg_data, BMP5_DEEP_DISABLE);

                    /* Checking deepstandby status only when powermode is in standby mode */

                    /* If deep_dis = 0(BMP5_DEEP_ENABLED) then deepstandby mode is enabled.
                     * If deep_dis = 1(BMP5_DEEP_DISABLED) then deepstandby mode is disabled
                     */
                    if (deep_dis == BMP5_DEEP_ENABLED)
                    {
                        rslt = check_deepstandby_mode(powermode, dev);
                    }
                    else
                    {
                        *powermode = BMP5_POWERMODE_STANDBY;
                    }

                    break;
                case BMP5_POWERMODE_NORMAL:
                    *powermode = BMP5_POWERMODE_NORMAL;
                    break;
                case BMP5_POWERMODE_FORCED:
                    *powermode = BMP5_POWERMODE_FORCED;
                    break;
                case BMP5_POWERMODE_CONTINUOUS:
                    *powermode = BMP5_POWERMODE_CONTINUOUS;
                    break;
                default:
                    rslt = BMP5_E_INVALID_POWERMODE;
                    break;
            }
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to set powermode of the sensor.
 */

static int8_t get_nvm_status(uint8_t *nvm_status, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;

    if (nvm_status != NULL)
    {
        rslt = bmp5_get_regs(BMP5_REG_STATUS, nvm_status, 1, dev);
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

static int8_t nvm_ready_check(struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t nvm_status;

    rslt = get_nvm_status(&nvm_status, dev);

    if (rslt == BMP5_OK)
    {
        /* Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed */
        if ((nvm_status & BMP5_INT_NVM_RDY) && (!(nvm_status & BMP5_INT_NVM_ERR)))
        {
            rslt = BMP5_OK;
        }
        else
        {
            rslt = BMP5_E_NVM_NOT_READY;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to check if sensor is in deepstandby mode.
 */

static int8_t validate_chip_id(uint8_t chip_id, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;

    if ((chip_id == BMP5_CHIP_ID_PRIM) || (chip_id == BMP5_CHIP_ID_SEC))
    {
        /* Updating chip_id in device structure */
        dev->chip_id = chip_id;
        rslt = BMP5_OK;
    }
    else
    {
        rslt = BMP5_E_DEV_NOT_FOUND;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate the NVM readiness.
 */

static int8_t bmp5_init(struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t reg_data;
    uint8_t chip_id;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMP5_OK)
    {
        dev->chip_id = 0;

        if (dev->intf == BMP5_SPI_INTF)
        {
            /* Performing a single read via SPI of registers,
             * e.g. registers CHIP_ID, before the actual
             * SPI communication with the device.
             */
            rslt = bmp5_get_regs(BMP5_REG_CHIP_ID, &reg_data, 1, dev);
        }

        if (rslt == BMP5_OK)
        {
            /* Read chip_id */
            rslt = bmp5_get_regs(BMP5_REG_CHIP_ID, &chip_id, 1, dev);

            if (rslt == BMP5_OK)
            {
                if (chip_id != 0)
                {
                    /* Validate NVM readiness */
                    rslt = nvm_ready_check(dev);
                }
                else
                {
                    rslt = BMP5_E_INVALID_CHIP_ID;
                }

                if (rslt == BMP5_OK)
                {
                    rslt = validate_chip_id(chip_id, dev);
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */

static int8_t bmp5_int_source_select(const struct bmp5_int_source_select *int_source_select, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t reg_data;

    if (int_source_select != NULL)
    {
        rslt = bmp5_get_regs(BMP5_REG_INT_SOURCE, &reg_data, 1, dev);

        if (rslt == BMP5_OK)
        {
            reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_INT_DRDY_EN, int_source_select->drdy_en);

            reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_FIFO_FULL_EN, int_source_select->fifo_full_en);

            reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_FIFO_THRES_EN, int_source_select->fifo_thres_en);

            reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_INT_OOR_PRESS_EN, int_source_select->oor_press_en);

            rslt = bmp5_set_regs(BMP5_REG_INT_SOURCE, &reg_data, 1, dev);
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
* @brief This API is used to configure the interrupt settings.
*/

static int8_t set_deep_standby_mode(struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t reg_data;

    rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Setting deep_dis = 0(BMP5_DEEP_ENABLED) enables the deep standby mode */
        reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_DEEP_DISABLE);

        /* Set ODR less then 5Hz - ODR used is 1Hz */
        reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_ODR, BMP5_ODR_01_HZ);

        /* Write the value to the odr config register(0x37) */
        rslt = bmp5_set_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_DSP_IIR, &reg_data, 1, dev);

            if (rslt == BMP5_OK)
            {
                /* Set iir_t and iir_p as Bypass(0x00) */

                /* The register holds only iir_t and iir_p and the last 2 bits are reserved.
                 * Thus using the macro BMP5_IIR_BYPASS(0xC0) the register value is set as zero.
                 */
                reg_data = reg_data & BMP5_IIR_BYPASS;

                /* Write the value to the IIR register(0x31) */
                rslt = bmp5_set_regs(BMP5_REG_DSP_IIR, &reg_data, 1, dev);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_FIFO_SEL, &reg_data, 1, dev);

            if (rslt == BMP5_OK)
            {
                /* Disable fifo frame selct */
                reg_data = BMP5_SET_BIT_VAL_0(reg_data, BMP5_FIFO_FRAME_SEL);

                /* Write the value to the fifo select register(0x18) */
                rslt = bmp5_set_regs(BMP5_REG_FIFO_SEL, &reg_data, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set sensor in standby or normal or forced or continuous powermode.
 */

static int8_t set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    uint8_t reg_data;

    rslt = bmp5_get_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);

    if (rslt == BMP5_OK)
    {
        /* Setting deep_dis = 1(BMP5_DEEP_DISABLED) disables the deep standby mode */
        reg_data = BMP5_SET_BITSLICE(reg_data, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);

        reg_data = BMP5_SET_BITS_POS_0(reg_data, BMP5_POWERMODE, powermode);

        rslt = bmp5_set_regs(BMP5_REG_ODR_CONFIG, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API is used to set sensor in standby powermode when powermode is deepstandby mode.
 */

static int8_t bmp5_set_power_mode(enum bmp5_powermode powermode, struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    enum bmp5_powermode lst_pwrmode;

    /* Existing power mode of the device is received in lst_pwrmode */
    rslt = bmp5_get_power_mode(&lst_pwrmode, dev);

    if (rslt == BMP5_OK)
    {
        /* If the sensor is not in standby mode set the device to
         *  standby mode.
         */
        if (lst_pwrmode != BMP5_POWERMODE_STANDBY)
        {
            /* Device should be set to standby before transiting to
             * forced mode or normal mode or continuous mode.
             */
            rslt = set_power_mode(BMP5_POWERMODE_STANDBY, dev);

            if (rslt == BMP5_OK)
            {
                /* Give t_standby(as per data sheet) time for device to go into standby mode */
                dev->delay_us(BMP5_DELAY_US_STANDBY, dev->intf_ptr);
            }
        }

        /* Set the desired power mode */
        if (rslt == BMP5_OK)
        {
            switch (powermode)
            {
                case BMP5_POWERMODE_DEEP_STANDBY:
                    rslt = set_deep_standby_mode(dev);
                    break;
                case BMP5_POWERMODE_STANDBY:

                    /* Since switching between powermodes require sensor to be in standby mode
                     * it is performed above. So it is not explicitly performed here.
                     */
                    break;
                case BMP5_POWERMODE_NORMAL:
                case BMP5_POWERMODE_FORCED:
                case BMP5_POWERMODE_CONTINUOUS:
                    rslt = set_power_mode(powermode, dev);
                    break;
                default:
                    rslt = BMP5_E_INVALID_POWERMODE;
                    break;
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the temperature(deg C) or both pressure(Pa) and temperature(deg C) data from the
 * sensor and store it in the bmp5_sensor_data structure instance passed by the user.
 */

static int8_t set_standby_mode(struct bmp5_dev *dev)
{
    int8_t rslt = BMP5_OK;
    enum bmp5_powermode pwrmode;

    rslt = bmp5_get_power_mode(&pwrmode, dev);

    if (rslt == BMP5_OK)
    {
        if (pwrmode == BMP5_POWERMODE_DEEP_STANDBY)
        {
            rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
        }
    }

    return rslt;
}


/*!
 * @brief This internal API is used to set the IIR for temperature and pressure.
 */

static int8_t bmp5_set_osr_odr_press_config(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt = 0;

    /* Variable to set ODR and OSR config */
    uint8_t reg_data[2] = { 0 };

    if (osr_odr_press_cfg != NULL)
    {
        /* If ODR is set to a value higher than 5Hz then powermode is set as standby mode, as ODR value greater than 5HZ
         * without disabling deep-standby mode makes powermode invalid.
         * NOTE: Register value for 5Hz is greater compared to ODRs higher than it. Thus in this below condition odr
         * is checked whether less than 5Hz macro.
         */
        if (osr_odr_press_cfg->odr < BMP5_ODR_05_HZ)
        {
            rslt = set_standby_mode(dev);
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);

            if (rslt == BMP5_OK)
            {
                reg_data[0] = BMP5_SET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS, osr_odr_press_cfg->osr_t);
                reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_OS, osr_odr_press_cfg->osr_p);
                reg_data[0] = BMP5_SET_BITSLICE(reg_data[0], BMP5_PRESS_EN, osr_odr_press_cfg->press_en);
                reg_data[1] = BMP5_SET_BITSLICE(reg_data[1], BMP5_ODR, osr_odr_press_cfg->odr);

                /* Set ODR and OSR configuration */
                rslt = bmp5_set_regs(BMP5_REG_OSR_CONFIG, reg_data, 2, dev);
            }
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API gets the configuration for IIR of temperature and pressure.
 */


/* ===== RT-Thread device driver ===== */

#define BMP585_PAYLOAD_BYTES 6u

static uint8_t g_baro_address;

static BMP5_INTF_RET_TYPE bmp5_bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *bus)
{
    if (!bus || !data || len == 0u || len > 128u)
    {
        return -1;
    }

    struct rt_i2c_msg messages[] = {
        { .addr = g_baro_address, .flags = RT_I2C_WR, .buf = &reg, .len = 1u },
        { .addr = g_baro_address, .flags = RT_I2C_RD, .buf = data, .len = (rt_uint16_t)len },
    };
    return rt_i2c_transfer(bus, messages, 2u) == 2 ? BMP5_INTF_RET_SUCCESS : -1;
}

static BMP5_INTF_RET_TYPE bmp5_bus_write(uint8_t reg, const uint8_t *data, uint32_t len, void *bus)
{
    uint8_t bytes[33];
    if (!bus || !data || len == 0u || len >= sizeof(bytes))
    {
        return -1;
    }

    bytes[0] = reg;
    memcpy(bytes + 1, data, len);
    struct rt_i2c_msg message = {
        .addr = g_baro_address, .flags = RT_I2C_WR, .buf = bytes, .len = (rt_uint16_t)(len + 1u),
    };
    return rt_i2c_transfer(bus, &message, 1u) == 1 ? BMP5_INTF_RET_SUCCESS : -1;
}

static void bmp5_bus_delay_us(uint32_t us, void *bus)
{
    (void)bus;
    if (us >= 1000u)
        rt_thread_mdelay((us + 999u) / 1000u);
    else
        rt_hw_us_delay(us);
}

static struct bmp5_dev g_baro;
static struct rt_device g_baro_dev;
static struct rt_mutex g_baro_lock;
static rt_device_t g_timebase;
static struct barometer_stats g_baro_stats;
static uint32_t g_baro_sequence;
static uint8_t g_baro_ready; /* part configured, bus and timebase held */

static int timebase_now(uint64_t *ticks)
{
    struct timebase_instant instant;
    if (!g_timebase)
    {
        return -1;
    }
    if (rt_device_read(g_timebase, 0, &instant, sizeof(instant)) != (rt_ssize_t)sizeof(instant))
    {
        return -1;
    }
    *ticks = instant.ticks;
    return 0;
}

static rt_err_t baro_configure(void)
{
    struct bmp5_osr_odr_press_config cfg;
    struct bmp5_osr_odr_press_config actual;
    struct bmp5_int_source_select irq_source;
    uint8_t readback;

    struct rt_i2c_bus_device *bus = rt_i2c_bus_device_find(BAROMETER_I2C_BUS_NAME);
    if (!bus)
    {
        return -RT_EIO;
    }

    g_baro.intf_ptr = bus;
    g_baro.read = bmp5_bus_read;
    g_baro.write = bmp5_bus_write;
    g_baro.delay_us = bmp5_bus_delay_us;
    g_baro.intf = BMP5_I2C_INTF;
    for (g_baro_address = BMP5_I2C_ADDR_PRIM; g_baro_address <= BMP5_I2C_ADDR_SEC; ++g_baro_address)
    {
        if (bmp5_init(&g_baro) == BMP5_OK)
        {
            break;
        }
    }
    if (g_baro_address > BMP5_I2C_ADDR_SEC || g_baro.chip_id != BMP5_CHIP_ID_SEC)
    {
        return -RT_EIO;
    }

    memset(&cfg, 0, sizeof(cfg));
    memset(&actual, 0, sizeof(actual));
    cfg.osr_t = BMP5_OVERSAMPLING_1X;
    cfg.osr_p = BMP5_OVERSAMPLING_8X;
    cfg.press_en = BMP5_ENABLE;
    cfg.odr = BMP5_ODR_100_2_HZ;
    if (bmp5_set_osr_odr_press_config(&cfg, &g_baro) ||
        bmp5_get_osr_odr_press_config(&actual, &g_baro) ||
        memcmp(&cfg, &actual, sizeof(cfg)))
    {
        return -RT_EIO;
    }

    memset(&irq_source, 0, sizeof(irq_source));
    irq_source.drdy_en = BMP5_ENABLE;
    if (bmp5_configure_interrupt(BMP5_LATCHED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL,
                                 BMP5_INTR_ENABLE, &g_baro) ||
        bmp5_int_source_select(&irq_source, &g_baro) ||
        bmp5_get_regs(BMP5_REG_INT_SOURCE, &readback, 1, &g_baro) ||
        !(readback & BMP5_INT_DRDY_EN_MSK) ||
        bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &g_baro))
    {
        return -RT_EIO;
    }
    return RT_EOK;
}

static rt_err_t baro_init(rt_device_t dev)
{
    (void)dev;

    if (g_baro_ready)
    {
        return RT_EOK;
    }

    g_timebase = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (!g_timebase || rt_device_open(g_timebase, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        g_timebase = NULL;
        return -RT_EIO;
    }
    if (rt_mutex_init(&g_baro_lock, "baro_lock", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        goto close_timebase;
    }
    if (baro_configure() != RT_EOK)
    {
        goto detach_mutex;
    }

    rt_pin_mode(BAROMETER_DATA_READY_PIN, PIN_MODE_INPUT);
    g_baro_ready = 1u;
    return RT_EOK;

detach_mutex:
    (void)rt_mutex_detach(&g_baro_lock);
close_timebase:
    (void)rt_device_close(g_timebase);
    g_timebase = NULL;
    return -RT_EIO;
}

static rt_err_t baro_open(rt_device_t dev, rt_uint16_t oflag)
{
    (void)dev;
    (void)oflag;

    return g_baro_ready ? RT_EOK : -RT_EIO;
}

static rt_err_t baro_close(rt_device_t dev)
{
    (void)dev;
    return RT_EOK;
}

static rt_ssize_t baro_read(rt_device_t dev, rt_off_t pos, void *buffer,
                            rt_size_t size)
{
    struct barometer_raw_sample *sample = buffer;
    uint8_t ready = 0u;
    int result;
    (void)dev;
    (void)pos;

    if (!sample || size < sizeof(*sample))
    {
        return -RT_EINVAL;
    }
    memset(sample, 0, sizeof(*sample));
    if (!g_baro_ready)
    {
        return -RT_EIO;
    }

    rt_mutex_take(&g_baro_lock, RT_WAITING_FOREVER);
    result = bmp5_get_interrupt_status(&ready, &g_baro);
    if (!result && !(ready & BMP5_INT_ASSERTED_DRDY))
    {
        rt_mutex_release(&g_baro_lock);
        return 0;
    }
    if (!result)
    {
        result = bmp5_get_regs(BMP5_REG_TEMP_DATA_XLSB, sample->data,
                               BMP585_PAYLOAD_BYTES, &g_baro);
    }
    if (!result && timebase_now(&sample->capture_tick))
    {
        result = -1;
    }
    sample->length = BMP585_PAYLOAD_BYTES;
    sample->sequence = ++g_baro_sequence;
    if (result)
    {
        g_baro_stats.errors++;
    }
    else
    {
        g_baro_stats.samples++;
    }
    rt_mutex_release(&g_baro_lock);
    return result ? -RT_EIO : (rt_ssize_t)sizeof(*sample);
}

static rt_err_t baro_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;
    switch (cmd)
    {
    case BAROMETER_CTRL_GET_STATS:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(struct barometer_stats *)args = g_baro_stats;
        return RT_EOK;
    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_device_ops baro_ops =
{
    .init = baro_init,
    .open = baro_open,
    .close = baro_close,
    .read = baro_read,
    .control = baro_control,
};

static int baro_register(void)
{
    g_baro_dev.type = RT_Device_Class_Sensor;
    g_baro_dev.ops = &baro_ops;
    return rt_device_register(&g_baro_dev, BAROMETER_DEVICE_NAME,
                              RT_DEVICE_FLAG_RDONLY);
}
INIT_DEVICE_EXPORT(baro_register);

#endif /* !FLIGHT_DRIVER_CODEC_TEST */
