/*
 * Bosch Sensortec SensorAPI license (BSD-3-Clause), retained in full.
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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

/* BMM350 three-axis magnetometer on I2C1.
 *
 * Native RT-Thread device "mag_sensor", built on the RT-Thread device framework only:
 * rt_device registration and lifecycle, register transactions through the I2C bus
 * framework (rt_i2c_transfer), the data-ready line through the PIN framework and
 * sample time through the "timebase" device.
 *
 * The Bosch SensorAPI is fused into this file: the part code supplies register access,
 * OTP/trim and compensation; the driver below owns the bus binding, the device
 * lifecycle and the M2-facing read/control interface.
 */

#include "bmm350.h"
#include "timer_capture.h"

#include <board.h>
#include <string.h>

#include <rtdevice.h>
#include <rthw.h>

/*
 * Bosch SensorAPI provenance
 * ----------------------------
 * Upstream : https://github.com/boschsensortec/BMM350_SensorAPI
 * Commit   : 3daf377ccaf589319c0d41af192105e9275987a2
 * Files    : bmm350_defs.h + bmm350.c (BSD-3-Clause)
 * Fused    : 22 of 34 functions kept (the call closure of this driver).
 * Dropped  : bmm350_api_version, bmm350_perform_self_test, bmm350_read_sensortime, bmm350_set_ctrl_user, bmm350_set_i2c_wdt, bmm350_set_int_ctrl_ibi, bmm350_set_pad_drive, bmm350_set_tmr_selftest_user, bmm350_soft_reset, self_test_config, self_test_entry_config, self_test_xy_axis
 */

/* ===== Bosch definitions, fused from bmm350_defs.h ===== */


/**************************** API Version *******************************/
#define BMM350_VER_MAJOR   1
#define BMM350_VER_MINOR   10
#define BMM350_VER_BUGFIX  0

/*************************** Header files *******************************/

#include <stdint.h>
#include <stddef.h>

/***************************** Common Macros *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#endif
#if !defined(UINT16_C) && !defined(INT16_C)
#endif
#if !defined(INT32_C) && !defined(UINT32_C)
#endif
#if !defined(INT64_C) && !defined(UINT64_C)
#endif

/*! C standard macros */
#ifndef NULL
#define NULL         ((void *) 0)
#endif

/******************************************************************************/
/*! @name        Compiler switch macros Definitions                           */
/******************************************************************************/

/* Comment/Uncomment this macro to disable/enable Fixed Point Computation */
/*#define BMM350_USE_FIXED_POINT*/

/************************* General Macro definitions ***************************/

/* Macro to SET and GET BITS of a register*/
#define BMM350_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BMM350_GET_BITS(reg_data, bitname)          ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

#define BMM350_GET_BITS_POS_0(reg_data, bitname)    (reg_data & (bitname##_MSK))

#define BMM350_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#ifndef BMM350_INTF_RET_TYPE
#define BMM350_INTF_RET_TYPE                        int8_t
#endif

#ifndef BMM350_DISABLE_POST_SOLDER_CORR
#define BMM350_POST_SOLDER_CORR
#endif

/*! Chip id of BMM350 */
#define BMM350_CHIP_ID                              UINT8_C(0x33)

/********** LSB to uT Compensation Coefficients for Fixed Point ***********/
#if F16_FRAC_BITS == 16
#endif

/************************* Sensor Shuttle Variant **************************/
#define BMM350_LEGACY_SHUTTLE_VARIANT_ID            UINT8_C(0x10)
#define BMM350_CURRENT_SHUTTLE_VARIANT_ID           UINT8_C(0x11)

/********************* Sensor interface success code **********************/
#define BMM350_INTF_RET_SUCCESS                     INT8_C(0)

/************************* API success code **************************/
#define BMM350_OK                                   INT8_C(0)

/* API error codes */
#define BMM350_E_NULL_PTR                           INT8_C(-1)
#define BMM350_E_COM_FAIL                           INT8_C(-2)
#define BMM350_E_DEV_NOT_FOUND                      INT8_C(-3)
#define BMM350_E_INVALID_CONFIG                     INT8_C(-4)
#define BMM350_E_BAD_PAD_DRIVE                      INT8_C(-5)
#define BMM350_E_RESET_UNFINISHED                   INT8_C(-6)
#define BMM350_E_INVALID_INPUT                      INT8_C(-7)
#define BMM350_E_SELF_TEST_INVALID_AXIS             INT8_C(-8)
#define BMM350_E_OTP_BOOT                           INT8_C(-9)
#define BMM350_E_OTP_PAGE_RD                        INT8_C(-10)
#define BMM350_E_OTP_PAGE_PRG                       INT8_C(-11)
#define BMM350_E_OTP_SIGN                           INT8_C(-12)
#define BMM350_E_OTP_INV_CMD                        INT8_C(-13)
#define BMM350_E_OTP_UNDEFINED                      INT8_C(-14)
#define BMM350_E_ALL_AXIS_DISABLED                  INT8_C(-15)
#define BMM350_E_PMU_CMD_VALUE                      INT8_C(-16)

#define BMM350_NO_ERROR                             UINT8_C(0)

/************************* Sensor delay time settings in microseconds **************************/
#define BMM350_SOFT_RESET_DELAY                     UINT32_C(24000)
#define BMM350_MAGNETIC_RESET_DELAY                 UINT32_C(40000)
#define BMM350_START_UP_TIME_FROM_POR               UINT32_C(3000)

#define BMM350_GOTO_SUSPEND_DELAY                   UINT32_C(6000)
#define BMM350_SUSPEND_TO_NORMAL_DELAY              UINT32_C(38000)

#define BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY       UINT32_C(15000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY        UINT32_C(17000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY        UINT32_C(20000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY        UINT32_C(28000)

#define BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY  UINT32_C(4000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY   UINT32_C(5000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY   UINT32_C(9000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY   UINT32_C(16000)

#define BMM350_UPD_OAE_DELAY                        UINT16_C(1000)

#define BMM350_BR_DELAY                             UINT16_C(14000)
#define BMM350_FGR_DELAY                            UINT16_C(18000)

/************************ Length macros ************************/
#define BMM350_OTP_DATA_LENGTH                      UINT8_C(32)
#define BMM350_READ_BUFFER_LENGTH                   UINT8_C(127)
#define BMM350_MAG_TEMP_DATA_LEN                    UINT8_C(12)

/************************ Averaging macros **********************/
#define BMM350_AVG_NO_AVG                           UINT8_C(0x0)
#define BMM350_AVG_2                                UINT8_C(0x1)
#define BMM350_AVG_4                                UINT8_C(0x2)
#define BMM350_AVG_8                                UINT8_C(0x3)

/******************************* ODR **************************/
#define BMM350_ODR_400HZ                            UINT8_C(0x2)
#define BMM350_ODR_200HZ                            UINT8_C(0x3)
#define BMM350_ODR_100HZ                            UINT8_C(0x4)
#define BMM350_ODR_50HZ                             UINT8_C(0x5)
#define BMM350_ODR_25HZ                             UINT8_C(0x6)
#define BMM350_ODR_12_5HZ                           UINT8_C(0x7)
#define BMM350_ODR_6_25HZ                           UINT8_C(0x8)
#define BMM350_ODR_3_125HZ                          UINT8_C(0x9)
#define BMM350_ODR_1_5625HZ                         UINT8_C(0xA)

/********************* Power modes *************************/
#define BMM350_PMU_CMD_SUS                          UINT8_C(0x00)
#define BMM350_PMU_CMD_NM                           UINT8_C(0x01)
#define BMM350_PMU_CMD_UPD_OAE                      UINT8_C(0x02)
#define BMM350_PMU_CMD_FM                           UINT8_C(0x03)
#define BMM350_PMU_CMD_FM_FAST                      UINT8_C(0x04)
#define BMM350_PMU_CMD_FGR                          UINT8_C(0x05)
#define BMM350_PMU_CMD_FGR_FAST                     UINT8_C(0x06)
#define BMM350_PMU_CMD_BR                           UINT8_C(0x07)
#define BMM350_PMU_CMD_BR_FAST                      UINT8_C(0x08)

#define BMM350_PMU_STATUS_0                         UINT8_C(0x0)

#define BMM350_DISABLE                              UINT8_C(0x0)
#define BMM350_ENABLE                               UINT8_C(0x1)

#define BMM350_CMD_NOP                              UINT8_C(0x0)
#define BMM350_CMD_SOFTRESET                        UINT8_C(0xB6)

#define BMM350_INT_MODE_LATCHED                     UINT8_C(0x1)
#define BMM350_INT_MODE_PULSED                      UINT8_C(0x0)

#define BMM350_INT_POL_ACTIVE_HIGH                  UINT8_C(0x1)
#define BMM350_INT_POL_ACTIVE_LOW                   UINT8_C(0x0)

#define BMM350_INT_OD_PUSHPULL                      UINT8_C(0x1)
#define BMM350_INT_OD_OPENDRAIN                     UINT8_C(0x0)

#define BMM350_INT_OUTPUT_EN_OFF                    UINT8_C(0x0)
#define BMM350_INT_OUTPUT_EN_ON                     UINT8_C(0x1)

#define BMM350_INT_DRDY_EN                          UINT8_C(0x1)
#define BMM350_INT_DRDY_DIS                         UINT8_C(0x0)

#define BMM350_LSB_MASK                             UINT16_C(0x00FF)
#define BMM350_MSB_MASK                             UINT16_C(0xFF00)

/********************** Pad drive strength ************************/
#define BMM350_PAD_DRIVE_WEAKEST                    UINT8_C(0)
#define BMM350_PAD_DRIVE_STRONGEST                  UINT8_C(7)

/******************* I2C Register Addresses ***********************/

/*! Register to set I2C address to LOW */
#define BMM350_I2C_ADSEL_SET_LOW                    UINT8_C(0x14)

/*! Register to set I2C address to HIGH */
#define BMM350_I2C_ADSEL_SET_HIGH                   UINT8_C(0x15)

#define BMM350_DUMMY_BYTES                          UINT8_C(2)

/********************** Register Addresses ************************/

#define BMM350_REG_CHIP_ID                          UINT8_C(0x00)
#define BMM350_REG_ERR_REG                          UINT8_C(0x02)
#define BMM350_REG_PAD_CTRL                         UINT8_C(0x03)
#define BMM350_REG_PMU_CMD_AGGR_SET                 UINT8_C(0x04)
#define BMM350_REG_PMU_CMD_AXIS_EN                  UINT8_C(0x05)
#define BMM350_REG_PMU_CMD                          UINT8_C(0x06)
#define BMM350_REG_PMU_CMD_STATUS_0                 UINT8_C(0x07)
#define BMM350_REG_PMU_CMD_STATUS_1                 UINT8_C(0x08)
#define BMM350_REG_I3C_ERR                          UINT8_C(0x09)
#define BMM350_REG_I2C_WDT_SET                      UINT8_C(0x0A)
#define BMM350_REG_TRSDCR_REV_ID                    UINT8_C(0x0D)
#define BMM350_REG_INT_CTRL                         UINT8_C(0x2E)
#define BMM350_REG_INT_CTRL_IBI                     UINT8_C(0x2F)
#define BMM350_REG_INT_STATUS                       UINT8_C(0x30)
#define BMM350_REG_MAG_X_XLSB                       UINT8_C(0x31)
#define BMM350_REG_MAG_X_LSB                        UINT8_C(0x32)
#define BMM350_REG_MAG_X_MSB                        UINT8_C(0x33)
#define BMM350_REG_MAG_Y_XLSB                       UINT8_C(0x34)
#define BMM350_REG_MAG_Y_LSB                        UINT8_C(0x35)
#define BMM350_REG_MAG_Y_MSB                        UINT8_C(0x36)
#define BMM350_REG_MAG_Z_XLSB                       UINT8_C(0x37)
#define BMM350_REG_MAG_Z_LSB                        UINT8_C(0x38)
#define BMM350_REG_MAG_Z_MSB                        UINT8_C(0x39)
#define BMM350_REG_TEMP_XLSB                        UINT8_C(0x3A)
#define BMM350_REG_TEMP_LSB                         UINT8_C(0x3B)
#define BMM350_REG_TEMP_MSB                         UINT8_C(0x3C)
#define BMM350_REG_SENSORTIME_XLSB                  UINT8_C(0x3D)
#define BMM350_REG_SENSORTIME_LSB                   UINT8_C(0x3E)
#define BMM350_REG_SENSORTIME_MSB                   UINT8_C(0x3F)
#define BMM350_REG_OTP_CMD_REG                      UINT8_C(0x50)
#define BMM350_REG_OTP_DATA_MSB_REG                 UINT8_C(0x52)
#define BMM350_REG_OTP_DATA_LSB_REG                 UINT8_C(0x53)
#define BMM350_REG_OTP_STATUS_REG                   UINT8_C(0x55)
#define BMM350_REG_TMR_SELFTEST_USER                UINT8_C(0x60)
#define BMM350_REG_CTRL_USER                        UINT8_C(0x61)
#define BMM350_REG_CMD                              UINT8_C(0x7E)

/*********************** Macros for bit masking ***************************/

#define BMM350_CHIP_ID_OTP_MSK                      UINT8_C(0xf)
#define BMM350_CHIP_ID_OTP_POS                      UINT8_C(0x0)
#define BMM350_CHIP_ID_FIXED_MSK                    UINT8_C(0xf0)
#define BMM350_CHIP_ID_FIXED_POS                    UINT8_C(0x4)
#define BMM350_PMU_CMD_ERROR_MSK                    UINT8_C(0x1)
#define BMM350_PMU_CMD_ERROR_POS                    UINT8_C(0x0)
#define BMM350_DRV_MSK                              UINT8_C(0x7)
#define BMM350_DRV_POS                              UINT8_C(0x0)
#define BMM350_AVG_MSK                              UINT8_C(0x30)
#define BMM350_AVG_POS                              UINT8_C(0x4)
#define BMM350_ODR_MSK                              UINT8_C(0xf)
#define BMM350_ODR_POS                              UINT8_C(0x0)
#define BMM350_PMU_CMD_MSK                          UINT8_C(0xf)
#define BMM350_PMU_CMD_POS                          UINT8_C(0x0)
#define BMM350_EN_X_MSK                             UINT8_C(0x01)
#define BMM350_EN_X_POS                             UINT8_C(0x0)
#define BMM350_EN_Y_MSK                             UINT8_C(0x02)
#define BMM350_EN_Y_POS                             UINT8_C(0x1)
#define BMM350_EN_Z_MSK                             UINT8_C(0x04)
#define BMM350_EN_Z_POS                             UINT8_C(0x2)
#define BMM350_EN_XYZ_MSK                           UINT8_C(0x7)
#define BMM350_EN_XYZ_POS                           UINT8_C(0x0)
#define BMM350_PMU_CMD_BUSY_MSK                     UINT8_C(0x1)
#define BMM350_PMU_CMD_BUSY_POS                     UINT8_C(0x0)
#define BMM350_ODR_OVWR_MSK                         UINT8_C(0x2)
#define BMM350_ODR_OVWR_POS                         UINT8_C(0x1)
#define BMM350_AVG_OVWR_MSK                         UINT8_C(0x4)
#define BMM350_AVG_OVWR_POS                         UINT8_C(0x2)
#define BMM350_PWR_MODE_IS_NORMAL_MSK               UINT8_C(0x8)
#define BMM350_PWR_MODE_IS_NORMAL_POS               UINT8_C(0x3)
#define BMM350_CMD_IS_ILLEGAL_MSK                   UINT8_C(0x10)
#define BMM350_CMD_IS_ILLEGAL_POS                   UINT8_C(0x4)
#define BMM350_PMU_CMD_VALUE_MSK                    UINT8_C(0xE0)
#define BMM350_PMU_CMD_VALUE_POS                    UINT8_C(0x5)
#define BMM350_PMU_ODR_S_MSK                        UINT8_C(0xf)
#define BMM350_PMU_ODR_S_POS                        UINT8_C(0x0)
#define BMM350_PMU_AVG_S_MSK                        UINT8_C(0x30)
#define BMM350_PMU_AVG_S_POS                        UINT8_C(0x4)
#define BMM350_I3C_ERROR_0_MSK                      UINT8_C(0x1)
#define BMM350_I3C_ERROR_0_POS                      UINT8_C(0x0)
#define BMM350_I3C_ERROR_3_MSK                      UINT8_C(0x8)
#define BMM350_I3C_ERROR_3_POS                      UINT8_C(0x3)
#define BMM350_I2C_WDT_EN_MSK                       UINT8_C(0x1)
#define BMM350_I2C_WDT_EN_POS                       UINT8_C(0x0)
#define BMM350_I2C_WDT_SEL_MSK                      UINT8_C(0x2)
#define BMM350_I2C_WDT_SEL_POS                      UINT8_C(0x1)
#define BMM350_TRSDCR_REV_ID_OTP_MSK                UINT8_C(0x3)
#define BMM350_TRSDCR_REV_ID_OTP_POS                UINT8_C(0x0)
#define BMM350_TRSDCR_REV_ID_FIXED_MSK              UINT8_C(0xfc)
#define BMM350_TRSDCR_REV_ID_FIXED_POS              UINT8_C(0x2)
#define BMM350_INT_MODE_MSK                         UINT8_C(0x1)
#define BMM350_INT_MODE_POS                         UINT8_C(0x0)
#define BMM350_INT_POL_MSK                          UINT8_C(0x2)
#define BMM350_INT_POL_POS                          UINT8_C(0x1)
#define BMM350_INT_OD_MSK                           UINT8_C(0x4)
#define BMM350_INT_OD_POS                           UINT8_C(0x2)
#define BMM350_INT_OUTPUT_EN_MSK                    UINT8_C(0x8)
#define BMM350_INT_OUTPUT_EN_POS                    UINT8_C(0x3)
#define BMM350_DRDY_DATA_REG_EN_MSK                 UINT8_C(0x80)
#define BMM350_DRDY_DATA_REG_EN_POS                 UINT8_C(0x7)
#define BMM350_DRDY_INT_MAP_TO_IBI_MSK              UINT8_C(0x1)
#define BMM350_DRDY_INT_MAP_TO_IBI_POS              UINT8_C(0x0)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_MSK   UINT8_C(0x10)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_POS   UINT8_C(0x4)
#define BMM350_DRDY_DATA_REG_MSK                    UINT8_C(0x4)
#define BMM350_DRDY_DATA_REG_POS                    UINT8_C(0x2)
#define BMM350_DATA_X_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_X_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_X_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_X_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_X_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_X_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Y_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Y_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Y_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Y_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Y_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Y_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Z_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Z_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Z_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Z_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Z_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Z_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_T_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_T_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_T_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_T_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_T_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_T_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_7_0_MSK                      UINT8_C(0xff)
#define BMM350_DATA_ST_7_0_POS                      UINT8_C(0x0)
#define BMM350_DATA_ST_15_8_MSK                     UINT8_C(0xff)
#define BMM350_DATA_ST_15_8_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_23_16_MSK                    UINT8_C(0xff)
#define BMM350_DATA_ST_23_16_POS                    UINT8_C(0x0)
#define BMM350_ST_IGEN_EN_MSK                       UINT8_C(0x1)
#define BMM350_ST_IGEN_EN_POS                       UINT8_C(0x0)
#define BMM350_ST_N_MSK                             UINT8_C(0x2)
#define BMM350_ST_N_POS                             UINT8_C(0x1)
#define BMM350_ST_P_MSK                             UINT8_C(0x4)
#define BMM350_ST_P_POS                             UINT8_C(0x2)
#define BMM350_IST_EN_X_MSK                         UINT8_C(0x8)
#define BMM350_IST_EN_X_POS                         UINT8_C(0x3)
#define BMM350_IST_EN_Y_MSK                         UINT8_C(0x10)
#define BMM350_IST_EN_Y_POS                         UINT8_C(0x4)
#define BMM350_CFG_SENS_TIM_AON_MSK                 UINT8_C(0x1)
#define BMM350_CFG_SENS_TIM_AON_POS                 UINT8_C(0x0)

/****************************** OTP MACROS ***************************/
#define BMM350_OTP_CMD_DIR_READ                     UINT8_C(0x20)
#define BMM350_OTP_CMD_DIR_PRGM_1B                  UINT8_C(0x40)
#define BMM350_OTP_CMD_DIR_PRGM                     UINT8_C(0x60)
#define BMM350_OTP_CMD_PWR_OFF_OTP                  UINT8_C(0x80)
#define BMM350_OTP_CMD_EXT_READ                     UINT8_C(0xA0)
#define BMM350_OTP_CMD_EXT_PRGM                     UINT8_C(0xE0)
#define BMM350_OTP_CMD_MSK                          UINT8_C(0xE0)
#define BMM350_OTP_WORD_ADDR_MSK                    UINT8_C(0x1F)

#define BMM350_OTP_STATUS_ERROR_MSK                 UINT8_C(0xE0)
#define BMM350_OTP_STATUS_ERROR(val)                (val & BMM350_OTP_STATUS_ERROR_MSK)
#define BMM350_OTP_STATUS_NO_ERROR                  UINT8_C(0x00)
#define BMM350_OTP_STATUS_BOOT_ERR                  UINT8_C(0x20)
#define BMM350_OTP_STATUS_PAGE_RD_ERR               UINT8_C(0x40)
#define BMM350_OTP_STATUS_PAGE_PRG_ERR              UINT8_C(0x60)
#define BMM350_OTP_STATUS_SIGN_ERR                  UINT8_C(0x80)
#define BMM350_OTP_STATUS_INV_CMD_ERR               UINT8_C(0xA0)
#define BMM350_OTP_STATUS_CMD_DONE                  UINT8_C(0x01)

/****************************** OTP indices ***************************/
#define BMM350_TEMP_OFF_SENS                        UINT8_C(0x0D)

#define BMM350_MAG_OFFSET_X                         UINT8_C(0x0E)
#define BMM350_MAG_OFFSET_Y                         UINT8_C(0x0F)
#define BMM350_MAG_OFFSET_Z                         UINT8_C(0x10)

#define BMM350_MAG_SENS_X                           UINT8_C(0x10)
#define BMM350_MAG_SENS_Y                           UINT8_C(0x11)
#define BMM350_MAG_SENS_Z                           UINT8_C(0x11)

#define BMM350_MAG_TCO_X                            UINT8_C(0x12)
#define BMM350_MAG_TCO_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCO_Z                            UINT8_C(0x14)

#define BMM350_MAG_TCS_X                            UINT8_C(0x12)
#define BMM350_MAG_TCS_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCS_Z                            UINT8_C(0x14)

#define BMM350_MAG_DUT_T_0                          UINT8_C(0x18)

#define BMM350_CROSS_X_Y                            UINT8_C(0x15)
#define BMM350_CROSS_Y_X                            UINT8_C(0x15)
#define BMM350_CROSS_Z_X                            UINT8_C(0x16)
#define BMM350_CROSS_Z_Y                            UINT8_C(0x16)

#ifdef BMM350_POST_SOLDER_CORR
#define BMM350_SENS_CORR_Y                          (0.01f)
#define BMM350_TCS_CORR_Z                           (0.0001f)
#endif

/**************************** Signed bit macros **********************/
#define BMM350_SIGNED_8_BIT                         UINT8_C(8)
#define BMM350_SIGNED_12_BIT                        UINT8_C(12)
#define BMM350_SIGNED_16_BIT                        UINT8_C(16)
#define BMM350_SIGNED_21_BIT                        UINT8_C(21)
#define BMM350_SIGNED_24_BIT                        UINT8_C(24)

/**************************** Self-test macros **********************/
#define BMM350_SELF_TEST_DISABLE                    UINT8_C(0x00)
#define BMM350_SELF_TEST_POS_X                      UINT8_C(0x0D)
#define BMM350_SELF_TEST_NEG_X                      UINT8_C(0x0B)
#define BMM350_SELF_TEST_POS_Y                      UINT8_C(0x15)
#define BMM350_SELF_TEST_NEG_Y                      UINT8_C(0x13)

/**************************** PMU command status 0 macros **********************/
#define BMM350_PMU_CMD_STATUS_0_SUS                 UINT8_C(0x00)
#define BMM350_PMU_CMD_STATUS_0_NM                  UINT8_C(0x01)
#define BMM350_PMU_CMD_STATUS_0_UPD_OAE             UINT8_C(0x02)
#define BMM350_PMU_CMD_STATUS_0_FM                  UINT8_C(0x03)
#define BMM350_PMU_CMD_STATUS_0_FM_FAST             UINT8_C(0x04)
#define BMM350_PMU_CMD_STATUS_0_FGR                 UINT8_C(0x05)
#define BMM350_PMU_CMD_STATUS_0_FGR_FAST            UINT8_C(0x06)
#define BMM350_PMU_CMD_STATUS_0_BR                  UINT8_C(0x07)
#define BMM350_PMU_CMD_STATUS_0_BR_FAST             UINT8_C(0x07)

/****************************** Enumerators ***************************/
enum bmm350_interrupt_enable_disable {
    BMM350_DISABLE_INTERRUPT = BMM350_DISABLE,
    BMM350_ENABLE_INTERRUPT = BMM350_ENABLE
};

enum bmm350_power_modes {
    BMM350_SUSPEND_MODE = BMM350_PMU_CMD_SUS,
    BMM350_NORMAL_MODE = BMM350_PMU_CMD_NM,
    BMM350_FORCED_MODE = BMM350_PMU_CMD_FM,
    BMM350_FORCED_MODE_FAST = BMM350_PMU_CMD_FM_FAST
};

enum bmm350_data_rates {
    BMM350_DATA_RATE_400HZ    = BMM350_ODR_400HZ,
    BMM350_DATA_RATE_200HZ    = BMM350_ODR_200HZ,
    BMM350_DATA_RATE_100HZ    = BMM350_ODR_100HZ,
    BMM350_DATA_RATE_50HZ     = BMM350_ODR_50HZ,
    BMM350_DATA_RATE_25HZ     = BMM350_ODR_25HZ,
    BMM350_DATA_RATE_12_5HZ   = BMM350_ODR_12_5HZ,
    BMM350_DATA_RATE_6_25HZ   = BMM350_ODR_6_25HZ,
    BMM350_DATA_RATE_3_125HZ  = BMM350_ODR_3_125HZ,
    BMM350_DATA_RATE_1_5625HZ = BMM350_ODR_1_5625HZ
};

enum bmm350_intr_en_dis {
    BMM350_INTR_DISABLE = BMM350_DISABLE,
    BMM350_INTR_ENABLE = BMM350_ENABLE
};

enum bmm350_intr_map {
    BMM350_UNMAP_FROM_PIN = BMM350_DISABLE,
    BMM350_MAP_TO_PIN = BMM350_ENABLE
};
enum bmm350_intr_latch {
    BMM350_PULSED = BMM350_INT_MODE_PULSED,
    BMM350_LATCHED = BMM350_INT_MODE_LATCHED
};

enum bmm350_intr_polarity {
    BMM350_ACTIVE_LOW = BMM350_INT_POL_ACTIVE_LOW,
    BMM350_ACTIVE_HIGH = BMM350_INT_POL_ACTIVE_HIGH
};

enum bmm350_intr_drive {
    BMM350_INTR_OPEN_DRAIN = BMM350_INT_OD_OPENDRAIN,
    BMM350_INTR_PUSH_PULL = BMM350_INT_OD_PUSHPULL
};

enum bmm350_drdy_int_map_to_ibi {
    BMM350_IBI_DISABLE = BMM350_DISABLE,
    BMM350_IBI_ENABLE = BMM350_ENABLE
};

enum bmm350_clear_drdy_int_status_upon_ibi {
    BMM350_NOCLEAR_ON_IBI = BMM350_DISABLE,
    BMM350_CLEAR_ON_IBI = BMM350_ENABLE
};

enum bmm350_i2c_wdt_en {
    BMM350_I2C_WDT_DIS = BMM350_DISABLE,
    BMM350_I2C_WDT_EN = BMM350_ENABLE
};

enum bmm350_i2c_wdt_sel {
    BMM350_I2C_WDT_SEL_SHORT = BMM350_DISABLE,
    BMM350_I2C_WDT_SEL_LONG = BMM350_ENABLE
};

enum bmm350_performance_parameters {
    BMM350_NO_AVERAGING = BMM350_AVG_NO_AVG,
    BMM350_AVERAGING_2 = BMM350_AVG_2,
    BMM350_AVERAGING_4 = BMM350_AVG_4,
    BMM350_AVERAGING_8 = BMM350_AVG_8,
    /*lint -e849*/
    BMM350_ULTRALOWNOISE = BMM350_AVG_8,
    BMM350_LOWNOISE = BMM350_AVG_4,
    BMM350_REGULARPOWER = BMM350_AVG_2,
    BMM350_LOWPOWER = BMM350_AVG_NO_AVG
};

enum bmm350_st_igen_en {
    BMM350_ST_IGEN_DIS = BMM350_DISABLE,
    BMM350_ST_IGEN_EN = BMM350_ENABLE
};

enum bmm350_st_n {
    BMM350_ST_N_DIS = BMM350_DISABLE,
    BMM350_ST_N_EN = BMM350_ENABLE
};

enum bmm350_st_p {
    BMM350_ST_P_DIS = BMM350_DISABLE,
    BMM350_ST_P_EN = BMM350_ENABLE
};

enum bmm350_ist_en_x {
    BMM350_IST_X_DIS = BMM350_DISABLE,
    BMM350_IST_X_EN = BMM350_ENABLE
};

enum bmm350_ist_en_y {
    BMM350_IST_Y_DIS = BMM350_DISABLE,
    BMM350_IST_Y_EN = BMM350_ENABLE
};

enum bmm350_ctrl_user {
    BMM350_CFG_SENS_TIM_AON_DIS = BMM350_DISABLE,
    BMM350_CFG_SENS_TIM_AON_EN = BMM350_ENABLE
};

enum bmm350_x_axis_en_dis {
    BMM350_X_DIS = BMM350_DISABLE,
    BMM350_X_EN = BMM350_ENABLE
};

enum bmm350_y_axis_en_dis {
    BMM350_Y_DIS = BMM350_DISABLE,
    BMM350_Y_EN = BMM350_ENABLE
};

enum bmm350_z_axis_en_dis {
    BMM350_Z_DIS = BMM350_DISABLE,
    BMM350_Z_EN = BMM350_ENABLE
};

enum bmm350_boot_status {
    BMM350_BOOT_NOT_DONE = BMM350_DISABLE,
    BMM350_BOOT_DONE = BMM350_ENABLE
};

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 * retval = 0 -> Success
 * retval < 0 -> Failure
 *
 */
typedef BMM350_INTF_RET_TYPE (*bmm350_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * retval = 0 -> Success
 * retval < 0 -> Failure
 *
 */
typedef BMM350_INTF_RET_TYPE (*bmm350_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
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
typedef void (*bmm350_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/* Pre-declaration */
struct bmm350_dev;

/*!
 * @brief Function pointer for the magnetic reset and wait override
 *
 * @param[in]  dev              : Structure instance of bmm350_dev.
 * @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
typedef int8_t (*bmm350_mraw_override_t)(struct bmm350_dev *dev);

/*************************  STRUCTURE DEFINITIONS *************************/

/*!
 * @brief bmm350 api version
 */
struct bmm350_version
{
    /*! Version Major */
    uint8_t major;

    /*! Version Minor */
    uint8_t minor;

    /*! Version Bugfix */
    uint8_t bugfix;

};


/*!
 * @brief bmm350 compensated magnetometer data and temperature data
 */
struct bmm350_mag_temp_data
{
    /*! Compensated mag X data */
    float x;

    /*! Compensated mag Y data */
    float y;

    /*! Compensated mag Z data */
    float z;

    /*! Temperature */
    float temperature;
};

/*!
 * @brief bmm350 magnetometer dut offset coefficient structure
 */
struct bmm350_dut_offset_coef
{
    /*! Temperature offset */
    float t_offs;

    /*! Offset x-axis */
    float offset_x;

    /*! Offset y-axis */
    float offset_y;

    /*! Offset z-axis */
    float offset_z;
};

/*!
 * @brief bmm350 magnetometer dut sensitivity coefficient structure
 */
struct bmm350_dut_sensit_coef
{
    /*! Temperature sensitivity */
    float t_sens;

    /*! Sensitivity x-axis */
    float sens_x;

    /*! Sensitivity y-axis */
    float sens_y;

    /*! Sensitivity z-axis */
    float sens_z;
};

/*!
 * @brief bmm350 magnetometer dut tco structure
 */
struct bmm350_dut_tco
{
    float tco_x;
    float tco_y;
    float tco_z;
};

/*!
 * @brief bmm350 magnetometer dut tcs structure
 */
struct bmm350_dut_tcs
{
    float tcs_x;
    float tcs_y;
    float tcs_z;
};

/*!
 * @brief bmm350 magnetometer cross axis compensation structure
 */
struct bmm350_cross_axis
{
    float cross_x_y;
    float cross_y_x;
    float cross_z_x;
    float cross_z_y;
};

/*!
 * @brief bmm350 magnetometer compensate structure
 */
struct bmm350_mag_compensate
{
    /*! Structure to store dut offset coefficient */
    struct bmm350_dut_offset_coef dut_offset_coef;

    /*! Structure to store dut sensitivity coefficient */
    struct bmm350_dut_sensit_coef dut_sensit_coef;

    /*! Structure to store dut tco */
    struct bmm350_dut_tco dut_tco;

    /*! Structure to store dut tcs */
    struct bmm350_dut_tcs dut_tcs;

    /*! Initialize T0_reading parameter */
    float dut_t0;

    /*! Structure to define cross axis compensation */
    struct bmm350_cross_axis cross_axis;
};

/*!
 * @brief bmm350 self-test structure
 */
struct bmm350_self_test
{
    /* Variable to store self-test high data on x-axis */
    float out_ust_xh;

    /* Variable to store self-test high data on y-axis */
    float out_ust_yh;

    /* Variable to store self-test low data on x-axis */
    float out_ust_xl;

    /* Variable to store self-test low data on y-axis */
    float out_ust_yl;

    /* Variable to store self-test data on x-axis */
    float out_ust_x;

    /* Variable to store self-test data on y-axis */
    float out_ust_y;
};


/*!
 * @brief bmm350 un-compensated (raw) magnetometer data, signed integer
 */
struct bmm350_raw_mag_data
{
    /*! Raw mag X data */
    int32_t raw_xdata;

    /*! Raw mag Y data */
    int32_t raw_ydata;

    /*! Raw mag Z data */
    int32_t raw_zdata;

    /*! Raw mag temperature value */
    int32_t raw_data_t;
};

/*!
 * @brief bmm350 device structure
 */
struct bmm350_dev
{
    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void* intf_ptr;

    /*! Chip Id of BMM350 */
    uint8_t chip_id;

    /*! Bus read function pointer */
    bmm350_read_fptr_t read;

    /*! Bus write function pointer */
    bmm350_write_fptr_t write;

    /*! delay(in us) function pointer */
    bmm350_delay_us_fptr_t delay_us;

    /*! To store interface pointer error */
    BMM350_INTF_RET_TYPE intf_rslt;

    /*! Variable to store status of axes enabled */
    uint8_t axis_en;

    /*! Structure for mag compensate */
    struct bmm350_mag_compensate mag_comp;

    /*! Array to store OTP data */
    uint16_t otp_data[BMM350_OTP_DATA_LENGTH];

    /*! Variant ID */
    uint8_t var_id;

    /*! Magnetic reset and wait override */
    bmm350_mraw_override_t mraw_override;

    /*! Variable to track the boot status */
    uint8_t boot_done_status;

    /*! Variable to enable/disable xy bit reset */
    uint8_t enable_auto_br;
};

/*!
 * @brief bmm350 PMU command status 0 structure
 */
struct bmm350_pmu_cmd_status_0
{
    /*! The previous PMU CMD is still in processing */
    uint8_t pmu_cmd_busy;

    /*! The previous PMU_CMD_AGGR_SET.odr has been overwritten */
    uint8_t odr_ovwr;

    /*! The previous PMU_CMD_AGGR_SET.avg has been overwritten */
    uint8_t avr_ovwr;

    /*! The chip is in normal power mode */
    uint8_t pwr_mode_is_normal;

    /*! CMD value is not allowed */
    uint8_t cmd_is_illegal;

    /*! Stores the latest PMU_CMD code processed */
    uint8_t pmu_cmd_value;
};


/* ===== Bosch SensorAPI, fused from bmm350.c ===== */
static int8_t null_ptr_check(const struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMM350_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMM350_OK;
    }

    return rslt;
}

/*!
 *  @brief This internal API converts the raw data from the IC data registers to signed integer
 */

static int8_t bmm350_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to define temporary length */
    uint16_t temp_len = len + BMM350_DUMMY_BYTES;

    /* Variable to define temporary buffer */
    uint8_t temp_buf[BMM350_READ_BUFFER_LENGTH];

    /* Variable to define loop */
    uint16_t index = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMM350_OK) && (reg_data != NULL))
    {
        /* Read the data from the reg_addr */
        dev->intf_rslt = dev->read(reg_addr, temp_buf, temp_len, dev->intf_ptr);

        if (dev->intf_rslt == BMM350_INTF_RET_SUCCESS)
        {
            /* Copy data after dummy byte indices */
            while (index < len)
            {
                reg_data[index] = temp_buf[index + BMM350_DUMMY_BYTES];
                index++;
            }
        }
        else
        {
            rslt = BMM350_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 */

static int8_t bmm350_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMM350_OK) && (reg_data != NULL) && (len != 0))
    {
        /* Write the data to the reg_addr */
        dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != BMM350_INTF_RET_SUCCESS)
        {
            rslt = BMM350_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of sensor.
 */

static int8_t bmm350_configure_interrupt(enum bmm350_intr_latch latching,
                                  enum bmm350_intr_polarity polarity,
                                  enum bmm350_intr_drive drivertype,
                                  enum bmm350_intr_map map_nomap,
                                  struct bmm350_dev *dev)
{
    /* Variable to get interrupt control configuration */
    uint8_t reg_data = 0;

    /* Variable to store the function result */
    int8_t rslt;

    /* Get interrupt control configuration */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &reg_data, 1, dev);

    if (rslt == BMM350_OK)
    {
        reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_INT_MODE, latching);
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_POL, polarity);
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OD, drivertype);
        reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OUTPUT_EN, map_nomap);

        /* Finally transfer the interrupt configurations */
        rslt = bmm350_set_regs(BMM350_REG_INT_CTRL, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API is used to read uncompensated mag and temperature data.
 */

static int8_t bmm350_delay_us(uint32_t period_us, const struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMM350_OK)
    {
        dev->delay_us(period_us, dev->intf_ptr);
    }

    return rslt;
}

/*!
 * @brief This API is used to perform soft-reset of the sensor
 * where all the registers are reset to their default values
 */

static int8_t bmm350_enable_axes(enum bmm350_x_axis_en_dis en_x,
                          enum bmm350_y_axis_en_dis en_y,
                          enum bmm350_z_axis_en_dis en_z,
                          struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to store axis data */
    uint8_t data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMM350_OK)
    {
        if ((en_x == BMM350_X_DIS) && (en_y == BMM350_Y_DIS) && (en_z == BMM350_Z_DIS))
        {
            rslt = BMM350_E_ALL_AXIS_DISABLED;

            /* Assign axis_en with all axis disabled status */
            dev->axis_en = BMM350_DISABLE;
        }
        else
        {
            data = (en_x & BMM350_EN_X_MSK);
            data = BMM350_SET_BITS(data, BMM350_EN_Y, en_y);
            data = BMM350_SET_BITS(data, BMM350_EN_Z, en_z);

            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD_AXIS_EN, &data, 1, dev);

            if (rslt == BMM350_OK)
            {
                /* Assign axis_en with the axis selection done */
                dev->axis_en = data;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to enable or disable the data ready interrupt
 */

static int8_t bmm350_enable_interrupt(enum bmm350_interrupt_enable_disable enable_disable, struct bmm350_dev *dev)
{
    /* Variable to get interrupt control configuration */
    uint8_t reg_data = 0;

    /* Variable to store the function result */
    int8_t rslt;

    /* Get interrupt control configuration */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &reg_data, 1, dev);

    if (rslt == BMM350_OK)
    {
        reg_data = BMM350_SET_BITS(reg_data, BMM350_DRDY_DATA_REG_EN, (uint8_t)enable_disable);

        /* Finally transfer the interrupt configurations */
        rslt = bmm350_set_regs(BMM350_REG_INT_CTRL, &reg_data, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API is used to configure the interrupt control settings
 */

static int32_t fix_sign(uint32_t inval, int8_t number_of_bits)
{
    int32_t power = 0;
    int32_t retval;

    switch (number_of_bits)
    {
        case BMM350_SIGNED_8_BIT:
            power = 128; /* 2^7 */
            break;

        case BMM350_SIGNED_12_BIT:
            power = 2048; /* 2^11 */
            break;

        case BMM350_SIGNED_16_BIT:
            power = 32768; /* 2^15 */
            break;

        case BMM350_SIGNED_21_BIT:
            power = 1048576; /* 2^20 */
            break;

        case BMM350_SIGNED_24_BIT:
            power = 8388608; /* 2^23 */
            break;

        default:
            power = 0;
            break;
    }

    retval = (int32_t)inval;

    if (retval >= power)
    {
        retval = retval - (power * 2);
    }

    return retval;
}

/*!
 * @brief This internal API is used to read OTP word
 */

static int8_t bmm350_read_uncomp_mag_temp_data(struct bmm350_raw_mag_data *raw_data, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t mag_data[12] = { 0 };

    uint32_t raw_mag_x, raw_mag_y, raw_mag_z, raw_temp;

    if (raw_data != NULL)
    {
        /* Get uncompensated mag data */
        rslt = bmm350_get_regs(BMM350_REG_MAG_X_XLSB, mag_data, BMM350_MAG_TEMP_DATA_LEN, dev);

        if (rslt == BMM350_OK)
        {
            raw_mag_x = (uint32_t)mag_data[0] + ((uint32_t)mag_data[1] << 8) + ((uint32_t)mag_data[2] << 16);
            raw_mag_y = (uint32_t)mag_data[3] + ((uint32_t)mag_data[4] << 8) + ((uint32_t)mag_data[5] << 16);
            raw_mag_z = (uint32_t)mag_data[6] + ((uint32_t)mag_data[7] << 8) + ((uint32_t)mag_data[8] << 16);
            raw_temp = (uint32_t)mag_data[9] + ((uint32_t)mag_data[10] << 8) + ((uint32_t)mag_data[11] << 16);

            if ((dev->axis_en & BMM350_EN_X_MSK) == BMM350_DISABLE)
            {
                raw_data->raw_xdata = BMM350_DISABLE;
            }
            else
            {
                raw_data->raw_xdata = fix_sign(raw_mag_x, BMM350_SIGNED_24_BIT);
            }

            if ((dev->axis_en & BMM350_EN_Y_MSK) == BMM350_DISABLE)
            {
                raw_data->raw_ydata = BMM350_DISABLE;
            }
            else
            {
                raw_data->raw_ydata = fix_sign(raw_mag_y, BMM350_SIGNED_24_BIT);
            }

            if ((dev->axis_en & BMM350_EN_Z_MSK) == BMM350_DISABLE)
            {
                raw_data->raw_zdata = BMM350_DISABLE;
            }
            else
            {
                raw_data->raw_zdata = fix_sign(raw_mag_z, BMM350_SIGNED_24_BIT);
            }

            raw_data->raw_data_t = fix_sign(raw_temp, BMM350_SIGNED_24_BIT);
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the interrupt control IBI configurations to the sensor.
 */

static void update_default_coefiecents(float *lsb_to_ut_degc)
{
    float bxy_sens, bz_sens, temp_sens, ina_xy_gain_trgt, ina_z_gain_trgt, adc_gain, lut_gain;
    float power;

    bxy_sens = 14.55f;
    bz_sens = 9.0f;
    temp_sens = 0.00204f;

    ina_xy_gain_trgt = 19.46f;

    ina_z_gain_trgt = 31.0;

    adc_gain = 1 / 1.5f;
    lut_gain = 0.714607238769531f;

    power = (float)(1000000.0 / 1048576.0);

    lsb_to_ut_degc[0] = (power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
    lsb_to_ut_degc[1] = (power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
    lsb_to_ut_degc[2] = (power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain));
    lsb_to_ut_degc[3] = 1 / (temp_sens * adc_gain * lut_gain * 1048576);
}

/*!
 * @brief This internal API is used to read OTP data after boot in user mode.
 */

static int8_t read_out_raw_data(float *out_data, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Float variable to convert mag lsb to uT and temp lsb to degC */
    float lsb_to_ut_degc[4];

    struct bmm350_raw_mag_data raw_data = { 0 };

    if (out_data != NULL)
    {
        rslt = bmm350_read_uncomp_mag_temp_data(&raw_data, dev);

        if (rslt == BMM350_OK)
        {
            /* Convert mag lsb to uT and temp lsb to degC */
            update_default_coefiecents(lsb_to_ut_degc);

            out_data[0] = (float)raw_data.raw_xdata * lsb_to_ut_degc[0];
            out_data[1] = (float)raw_data.raw_ydata * lsb_to_ut_degc[1];
            out_data[2] = (float)raw_data.raw_zdata * lsb_to_ut_degc[2];
            out_data[3] = (float)raw_data.raw_data_t * lsb_to_ut_degc[3];

            out_data[3] = (float)(out_data[3] - (1 * 25.49));

        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}


/*!
 * @brief This internal API is used to convert lsb to uT and degC.
 */

static int8_t bmm350_get_compensated_mag_xyz_temp_data(struct bmm350_mag_temp_data *mag_temp_data, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t indx;

    float out_data[4] = { 0.0f };
    float dut_offset_coef[3], dut_sensit_coef[3], dut_tco[3], dut_tcs[3];
    float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;

    if (mag_temp_data != NULL)
    {
        /* Reads raw magnetic x,y and z axis along with temperature */
        rslt = read_out_raw_data(out_data, dev);

        if (rslt == BMM350_OK)
        {
            /* Apply compensation to temperature reading */
            out_data[3] = (1 + dev->mag_comp.dut_sensit_coef.t_sens) * out_data[3] +
                          dev->mag_comp.dut_offset_coef.t_offs;

            /* Store magnetic compensation structure to an array */
            dut_offset_coef[0] = dev->mag_comp.dut_offset_coef.offset_x;
            dut_offset_coef[1] = dev->mag_comp.dut_offset_coef.offset_y;
            dut_offset_coef[2] = dev->mag_comp.dut_offset_coef.offset_z;

            dut_sensit_coef[0] = dev->mag_comp.dut_sensit_coef.sens_x;
            dut_sensit_coef[1] = dev->mag_comp.dut_sensit_coef.sens_y;
            dut_sensit_coef[2] = dev->mag_comp.dut_sensit_coef.sens_z;

            dut_tco[0] = dev->mag_comp.dut_tco.tco_x;
            dut_tco[1] = dev->mag_comp.dut_tco.tco_y;
            dut_tco[2] = dev->mag_comp.dut_tco.tco_z;

            dut_tcs[0] = dev->mag_comp.dut_tcs.tcs_x;
            dut_tcs[1] = dev->mag_comp.dut_tcs.tcs_y;
            dut_tcs[2] = dev->mag_comp.dut_tcs.tcs_z;

            /* Compensate raw magnetic data */
            for (indx = 0; indx < 3; indx++)
            {
                out_data[indx] *= 1 + dut_sensit_coef[indx];
                out_data[indx] += dut_offset_coef[indx];
                out_data[indx] += dut_tco[indx] * (out_data[3] - dev->mag_comp.dut_t0);
                out_data[indx] /= 1 + dut_tcs[indx] * (out_data[3] - dev->mag_comp.dut_t0);
            }

            cr_ax_comp_x = (out_data[0] - dev->mag_comp.cross_axis.cross_x_y * out_data[1]) /
                           (1 - dev->mag_comp.cross_axis.cross_y_x * dev->mag_comp.cross_axis.cross_x_y);
            cr_ax_comp_y = (out_data[1] - dev->mag_comp.cross_axis.cross_y_x * out_data[0]) /
                           (1 - dev->mag_comp.cross_axis.cross_y_x * dev->mag_comp.cross_axis.cross_x_y);
            cr_ax_comp_z =
                (out_data[2] +
                 (out_data[0] *
                  (dev->mag_comp.cross_axis.cross_y_x * dev->mag_comp.cross_axis.cross_z_y -
                   dev->mag_comp.cross_axis.cross_z_x) - out_data[1] *
                  (dev->mag_comp.cross_axis.cross_z_y - dev->mag_comp.cross_axis.cross_x_y *
                   dev->mag_comp.cross_axis.cross_z_x)) /
                 (1 - dev->mag_comp.cross_axis.cross_y_x * dev->mag_comp.cross_axis.cross_x_y));

            out_data[0] = cr_ax_comp_x;
            out_data[1] = cr_ax_comp_y;
            out_data[2] = cr_ax_comp_z;
        }

        if (rslt == BMM350_OK)
        {
            if ((dev->axis_en & BMM350_EN_X_MSK) == BMM350_DISABLE)
            {
                mag_temp_data->x = BMM350_DISABLE;
            }
            else
            {
                mag_temp_data->x = out_data[0];
            }

            if ((dev->axis_en & BMM350_EN_Y_MSK) == BMM350_DISABLE)
            {
                mag_temp_data->y = BMM350_DISABLE;
            }
            else
            {
                mag_temp_data->y = out_data[1];
            }

            if ((dev->axis_en & BMM350_EN_Z_MSK) == BMM350_DISABLE)
            {
                mag_temp_data->z = BMM350_DISABLE;
            }
            else
            {
                mag_temp_data->z = out_data[2];
            }

            mag_temp_data->temperature = out_data[3];
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This function executes FGR and BR sequences to initialize TMR sensor and performs the user self-test.
 */

static int8_t bmm350_get_interrupt_status(uint8_t *drdy_status, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t int_status_reg;

    if (drdy_status != NULL)
    {
        /* Get the status of interrupt */
        rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status_reg, 1, dev);

        if (rslt == BMM350_OK)
        {
            /* Read the interrupt status */
            (*drdy_status) = BMM350_GET_BITS(int_status_reg, BMM350_DRDY_DATA_REG);
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to set the power mode of the sensor
 */

static int8_t bmm350_get_pmu_cmd_status_0(struct bmm350_pmu_cmd_status_0 *pmu_cmd_stat_0, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t reg_data;

    if (pmu_cmd_stat_0 != NULL)
    {
        /* Get PMU command status 0 data */
        rslt = bmm350_get_regs(BMM350_REG_PMU_CMD_STATUS_0, &reg_data, 1, dev);

        if (rslt == BMM350_OK)
        {
            pmu_cmd_stat_0->pmu_cmd_busy = BMM350_GET_BITS_POS_0(reg_data, BMM350_PMU_CMD_BUSY);

            pmu_cmd_stat_0->odr_ovwr = BMM350_GET_BITS(reg_data, BMM350_ODR_OVWR);

            pmu_cmd_stat_0->avr_ovwr = BMM350_GET_BITS(reg_data, BMM350_AVG_OVWR);

            pmu_cmd_stat_0->pwr_mode_is_normal = BMM350_GET_BITS(reg_data, BMM350_PWR_MODE_IS_NORMAL);

            pmu_cmd_stat_0->cmd_is_illegal = BMM350_GET_BITS(reg_data, BMM350_CMD_IS_ILLEGAL);

            pmu_cmd_stat_0->pmu_cmd_value = BMM350_GET_BITS(reg_data, BMM350_PMU_CMD_VALUE);
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}


/****************************************************************************/
/**\name     INTERNAL APIs                                                  */

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */

static int8_t set_powermode(enum bmm350_power_modes powermode, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t reg_data = powermode;
    uint8_t get_avg;

    /* Array to store suspend to forced mode delay */
    uint32_t sus_to_forced_mode[4] =
    { BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY, BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY, BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY,
      BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY };

    /* Array to store suspend to forced mode fast delay */
    uint32_t sus_to_forced_mode_fast[4] =
    { BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY, BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY,
      BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY, BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY };

    uint8_t avg = 0;
    uint32_t delay_us = 0;

    rslt = null_ptr_check(dev);

    if (rslt == BMM350_OK)
    {
        /* Set PMU command configuration to desired power mode */
        rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &reg_data, 1, dev);

        if (rslt == BMM350_OK)
        {
            /* Get average configuration */
            rslt = bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &get_avg, 1, dev);

            if (rslt == BMM350_OK)
            {
                /* Mask the average value */
                avg = ((get_avg & BMM350_AVG_MSK) >> BMM350_AVG_POS);
            }
        }
    }

    if (rslt == BMM350_OK)
    {
        /* Check if desired power mode is normal mode */
        if (powermode == BMM350_NORMAL_MODE)
        {
            delay_us = BMM350_SUSPEND_TO_NORMAL_DELAY;
        }

        /* Check if desired power mode is forced mode */
        if (powermode == BMM350_FORCED_MODE)
        {
            /* Store delay based on averaging mode */
            delay_us = sus_to_forced_mode[avg];
        }

        /* Check if desired power mode is forced mode fast */
        if (powermode == BMM350_FORCED_MODE_FAST)
        {
            /* Store delay based on averaging mode */
            delay_us = sus_to_forced_mode_fast[avg];
        }

        /* Perform delay based on power mode */
        rslt = bmm350_delay_us(delay_us, dev);
    }

    return rslt;
}


static int8_t bmm350_set_powermode(enum bmm350_power_modes powermode, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t last_pwr_mode;
    uint8_t reg_data;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMM350_OK)
    {
        rslt = bmm350_get_regs(BMM350_REG_PMU_CMD, &last_pwr_mode, 1, dev);

        if (rslt == BMM350_OK)
        {
            if (last_pwr_mode > BMM350_PMU_CMD_BR_FAST)
            {
                rslt = BMM350_E_INVALID_CONFIG;
            }

            if ((rslt == BMM350_OK) &&
                ((last_pwr_mode == BMM350_PMU_CMD_NM) || (last_pwr_mode == BMM350_PMU_CMD_UPD_OAE)))
            {
                reg_data = BMM350_PMU_CMD_SUS;

                /* Set PMU command configuration */
                rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &reg_data, 1, dev);

                if (rslt == BMM350_OK)
                {
                    rslt = bmm350_delay_us(BMM350_GOTO_SUSPEND_DELAY, dev);
                }
            }

            if (rslt == BMM350_OK)
            {
                rslt = set_powermode(powermode, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the ODR and averaging factor.
 */

static int8_t bmm350_magnetic_reset_and_wait(struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t pmu_cmd = 0;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0 = { 0 };
    uint8_t restore_normal = BMM350_DISABLE;

    rslt = null_ptr_check(dev);

    if ((rslt == BMM350_OK) && (dev->mraw_override))
    {
        rslt = dev->mraw_override(dev);
    }
    else
    {
        /* Read PMU CMD status */
        rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, dev);

        /* Check the powermode is normal before performing magnetic reset */
        if ((rslt == BMM350_OK) && (pmu_cmd_stat_0.pwr_mode_is_normal == BMM350_ENABLE))
        {
            restore_normal = BMM350_ENABLE;

            /* Reset can only be triggered in suspend */
            rslt = bmm350_set_powermode(BMM350_SUSPEND_MODE, dev);
        }

        if (rslt == BMM350_OK)
        {
            /* Set BR to PMU_CMD register */
            pmu_cmd = BMM350_PMU_CMD_BR;

            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);

            if (rslt == BMM350_OK)
            {
                rslt = bmm350_delay_us(BMM350_BR_DELAY, dev);
            }
        }

        if (rslt == BMM350_OK)
        {
            /* Verify if PMU_CMD_STATUS_0 register has BR set */
            rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, dev);

            if ((rslt == BMM350_OK) && (pmu_cmd_stat_0.pmu_cmd_value != BMM350_PMU_CMD_STATUS_0_BR))
            {
                rslt = BMM350_E_PMU_CMD_VALUE;
            }
        }

        if (rslt == BMM350_OK)
        {
            /* Set FGR to PMU_CMD register */
            pmu_cmd = BMM350_PMU_CMD_FGR;

            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);

            if (rslt == BMM350_OK)
            {
                rslt = bmm350_delay_us(BMM350_FGR_DELAY, dev);
            }
        }

        if (rslt == BMM350_OK)
        {
            /* Verify if PMU_CMD_STATUS_0 register has FGR set */
            rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, dev);

            if ((rslt == BMM350_OK) && (pmu_cmd_stat_0.pmu_cmd_value != BMM350_PMU_CMD_STATUS_0_FGR))
            {
                rslt = BMM350_E_PMU_CMD_VALUE;
            }
        }

        if ((rslt == BMM350_OK) && (restore_normal == BMM350_ENABLE))
        {
            rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, dev);
        }
    }

    return rslt;
}


/*!
 * @brief This API is used to perform compensation for raw magnetometer and temperature data.
 */

static int8_t read_otp_word(uint8_t addr, uint16_t *lsb_msb, struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint8_t otp_cmd, otp_status = 0, otp_err = BMM350_OTP_STATUS_NO_ERROR, lsb = 0, msb = 0;

    if (lsb_msb != NULL)
    {
        /* Set OTP command at specified address */
        otp_cmd = BMM350_OTP_CMD_DIR_READ | (addr & BMM350_OTP_WORD_ADDR_MSK);
        rslt = bmm350_set_regs(BMM350_REG_OTP_CMD_REG, &otp_cmd, 1, dev);
        if (rslt == BMM350_OK)
        {
            do
            {
                rslt = bmm350_delay_us(300, dev);

                if (rslt == BMM350_OK)
                {
                    /* Get OTP status */
                    rslt = bmm350_get_regs(BMM350_REG_OTP_STATUS_REG, &otp_status, 1, dev);

                    otp_err = BMM350_OTP_STATUS_ERROR(otp_status);
                    if (otp_err != BMM350_OTP_STATUS_NO_ERROR)
                    {
                        break;
                    }
                }
            } while ((!(otp_status & BMM350_OTP_STATUS_CMD_DONE)) && (rslt == BMM350_OK));

            if (otp_err != BMM350_OTP_STATUS_NO_ERROR)
            {
                switch (otp_err)
                {
                    case BMM350_OTP_STATUS_BOOT_ERR:
                        rslt = BMM350_E_OTP_BOOT;
                        break;
                    case BMM350_OTP_STATUS_PAGE_RD_ERR:
                        rslt = BMM350_E_OTP_PAGE_RD;
                        break;
                    case BMM350_OTP_STATUS_PAGE_PRG_ERR:
                        rslt = BMM350_E_OTP_PAGE_PRG;
                        break;
                    case BMM350_OTP_STATUS_SIGN_ERR:
                        rslt = BMM350_E_OTP_SIGN;
                        break;
                    case BMM350_OTP_STATUS_INV_CMD_ERR:
                        rslt = BMM350_E_OTP_INV_CMD;
                        break;
                    default:
                        rslt = BMM350_E_OTP_UNDEFINED;
                        break;
                }
            }
        }

        if (rslt == BMM350_OK)
        {
            /* Get OTP MSB data */
            rslt = bmm350_get_regs(BMM350_REG_OTP_DATA_MSB_REG, &msb, 1, dev);
            if (rslt == BMM350_OK)
            {
                /* Get OTP LSB data */
                rslt = bmm350_get_regs(BMM350_REG_OTP_DATA_LSB_REG, &lsb, 1, dev);
                *lsb_msb = ((uint16_t)(msb << 8) | lsb) & 0xFFFF;
            }
        }
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API is used to update magnetometer offset and sensitivity data.
 */

static void update_mag_off_sens(struct bmm350_dev *dev)
{
    uint16_t off_x_lsb_msb, off_y_lsb_msb, off_z_lsb_msb, t_off = 0;
    uint8_t sens_x, sens_y, sens_z, t_sens = 0;
    uint8_t tco_x, tco_y, tco_z = 0;
    uint8_t tcs_x, tcs_y, tcs_z = 0;
    uint8_t cross_x_y, cross_y_x, cross_z_x, cross_z_y = 0;

    off_x_lsb_msb = dev->otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF;
    off_y_lsb_msb = ((dev->otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) +
                    (dev->otp_data[BMM350_MAG_OFFSET_Y] & BMM350_LSB_MASK);
    off_z_lsb_msb = (dev->otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) +
                    (dev->otp_data[BMM350_MAG_OFFSET_Z] & BMM350_LSB_MASK);
    t_off = dev->otp_data[BMM350_TEMP_OFF_SENS] & BMM350_LSB_MASK;

    dev->mag_comp.dut_offset_coef.offset_x = fix_sign(off_x_lsb_msb, BMM350_SIGNED_12_BIT);
    dev->mag_comp.dut_offset_coef.offset_y = fix_sign(off_y_lsb_msb, BMM350_SIGNED_12_BIT);
    dev->mag_comp.dut_offset_coef.offset_z = fix_sign(off_z_lsb_msb, BMM350_SIGNED_12_BIT);
    dev->mag_comp.dut_offset_coef.t_offs = fix_sign(t_off, BMM350_SIGNED_8_BIT) / 5.0f;

    sens_x = (dev->otp_data[BMM350_MAG_SENS_X] & BMM350_MSB_MASK) >> 8;
    sens_y = (dev->otp_data[BMM350_MAG_SENS_Y] & BMM350_LSB_MASK);
    sens_z = (dev->otp_data[BMM350_MAG_SENS_Z] & BMM350_MSB_MASK) >> 8;
    t_sens = (dev->otp_data[BMM350_TEMP_OFF_SENS] & BMM350_MSB_MASK) >> 8;

    dev->mag_comp.dut_sensit_coef.sens_x = fix_sign(sens_x, BMM350_SIGNED_8_BIT) / 256.0f;
    dev->mag_comp.dut_sensit_coef.sens_y = fix_sign(sens_y, BMM350_SIGNED_8_BIT) / 256.0f;
    dev->mag_comp.dut_sensit_coef.sens_z = fix_sign(sens_z, BMM350_SIGNED_8_BIT) / 256.0f;
    dev->mag_comp.dut_sensit_coef.t_sens = fix_sign(t_sens, BMM350_SIGNED_8_BIT) / 512.0f;

    tco_x = (dev->otp_data[BMM350_MAG_TCO_X] & BMM350_LSB_MASK);
    tco_y = (dev->otp_data[BMM350_MAG_TCO_Y] & BMM350_LSB_MASK);
    tco_z = (dev->otp_data[BMM350_MAG_TCO_Z] & BMM350_LSB_MASK);

    dev->mag_comp.dut_tco.tco_x = fix_sign(tco_x, BMM350_SIGNED_8_BIT) / 32.0f;
    dev->mag_comp.dut_tco.tco_y = fix_sign(tco_y, BMM350_SIGNED_8_BIT) / 32.0f;
    dev->mag_comp.dut_tco.tco_z = fix_sign(tco_z, BMM350_SIGNED_8_BIT) / 32.0f;

    tcs_x = (dev->otp_data[BMM350_MAG_TCS_X] & BMM350_MSB_MASK) >> 8;
    tcs_y = (dev->otp_data[BMM350_MAG_TCS_Y] & BMM350_MSB_MASK) >> 8;
    tcs_z = (dev->otp_data[BMM350_MAG_TCS_Z] & BMM350_MSB_MASK) >> 8;

    dev->mag_comp.dut_tcs.tcs_x = fix_sign(tcs_x, BMM350_SIGNED_8_BIT) / 16384.0f;
    dev->mag_comp.dut_tcs.tcs_y = fix_sign(tcs_y, BMM350_SIGNED_8_BIT) / 16384.0f;
    dev->mag_comp.dut_tcs.tcs_z = fix_sign(tcs_z, BMM350_SIGNED_8_BIT) / 16384.0f;

    dev->mag_comp.dut_t0 = (fix_sign(dev->otp_data[BMM350_MAG_DUT_T_0], BMM350_SIGNED_16_BIT) / 512.0f) + 23.0f;

    cross_x_y = (dev->otp_data[BMM350_CROSS_X_Y] & BMM350_LSB_MASK);
    cross_y_x = (dev->otp_data[BMM350_CROSS_Y_X] & BMM350_MSB_MASK) >> 8;
    cross_z_x = (dev->otp_data[BMM350_CROSS_Z_X] & BMM350_LSB_MASK);
    cross_z_y = (dev->otp_data[BMM350_CROSS_Z_Y] & BMM350_MSB_MASK) >> 8;

    dev->mag_comp.cross_axis.cross_x_y = fix_sign(cross_x_y, BMM350_SIGNED_8_BIT) / 800.0f;
    dev->mag_comp.cross_axis.cross_y_x = fix_sign(cross_y_x, BMM350_SIGNED_8_BIT) / 800.0f;
    dev->mag_comp.cross_axis.cross_z_x = fix_sign(cross_z_x, BMM350_SIGNED_8_BIT) / 800.0f;
    dev->mag_comp.cross_axis.cross_z_y = fix_sign(cross_z_y, BMM350_SIGNED_8_BIT) / 800.0f;

}

/*!
 * @brief This internal API is used to read raw magnetic x,y and z axis along with temperature
 */

static int8_t otp_dump_after_boot(struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    uint16_t otp_word = 0;
    uint8_t indx;

    for (indx = 0; indx < BMM350_OTP_DATA_LENGTH; indx++)
    {
        rslt = read_otp_word(indx, &otp_word, dev);
        dev->otp_data[indx] = otp_word;
    }

    dev->var_id = (dev->otp_data[30] & 0x7f00) >> 9;

    /* Set the default auto bit reset configuration */
    dev->enable_auto_br = ((dev->var_id > BMM350_CURRENT_SHUTTLE_VARIANT_ID) ? BMM350_DISABLE : BMM350_ENABLE);

    /* Update magnetometer offset and sensitivity data. */
    update_mag_off_sens(dev);

    return rslt;
}

/*!
 * @brief This internal API is used for self-test entry configuration
 */

static int8_t bmm350_init(struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to get chip id */
    uint8_t chip_id = BMM350_DISABLE;

    /* Variable to store the command to power-off the OTP */
    uint8_t otp_cmd = BMM350_OTP_CMD_PWR_OFF_OTP;

    /* Variable to store soft-reset command */
    uint8_t soft_reset;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMM350_OK)
    {
        dev->chip_id = 0;

        /* Assign axis_en with all axis enabled (BMM350_EN_XYZ_MSK) */
        dev->axis_en = BMM350_EN_XYZ_MSK;

        rslt = bmm350_delay_us(BMM350_START_UP_TIME_FROM_POR, dev);

        if (rslt == BMM350_OK)
        {
            /* Soft-reset */
            soft_reset = BMM350_CMD_SOFTRESET;

            /* Set the command in the command register */
            rslt = bmm350_set_regs(BMM350_REG_CMD, &soft_reset, 1, dev);

            if (rslt == BMM350_OK)
            {
                rslt = bmm350_delay_us(BMM350_SOFT_RESET_DELAY, dev);
            }
        }

        if (rslt == BMM350_OK)
        {
            /* Chip ID of the sensor is read */
            rslt = bmm350_get_regs(BMM350_REG_CHIP_ID, &chip_id, 1, dev);

            if (rslt == BMM350_OK)
            {
                /* Assign chip_id to dev->chip_id */
                dev->chip_id = chip_id;
            }
        }

        /* Check for chip id validity */
        if ((rslt == BMM350_OK) && (dev->chip_id == BMM350_CHIP_ID))
        {
            /* Download OTP memory */
            rslt = otp_dump_after_boot(dev);

            if (rslt == BMM350_OK)
            {
                /* Power off OTP */
                rslt = bmm350_set_regs(BMM350_REG_OTP_CMD_REG, &otp_cmd, 1, dev);

                if (rslt == BMM350_OK)
                {
                    if (dev->boot_done_status != BMM350_BOOT_NOT_DONE)
                    {
                        dev->boot_done_status = BMM350_BOOT_NOT_DONE;
                    }

                    rslt = bmm350_magnetic_reset_and_wait(dev);

                    if (rslt == BMM350_OK)
                    {
                        dev->boot_done_status = BMM350_BOOT_DONE;
                    }
                }
            }
        }
        else
        {
            rslt = BMM350_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */

static int8_t bmm350_set_odr_performance(enum bmm350_data_rates odr,
                                  enum bmm350_performance_parameters performance,
                                  struct bmm350_dev *dev)
{
    /* Variable to store the function result */
    int8_t rslt;

    /* Variable to get PMU command */
    uint8_t reg_data = 0;

    enum bmm350_performance_parameters performance_fix = performance;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if (rslt == BMM350_OK)
    {
        /* Reduce the performance setting when too high for the chosen ODR */
        if ((odr == BMM350_DATA_RATE_400HZ) && (performance >= BMM350_AVERAGING_2))
        {
            performance_fix = BMM350_NO_AVERAGING;
        }
        else if ((odr == BMM350_DATA_RATE_200HZ) && (performance >= BMM350_AVERAGING_4))
        {
            performance_fix = BMM350_AVERAGING_2;
        }
        else if ((odr == BMM350_DATA_RATE_100HZ) && (performance >= BMM350_AVERAGING_8))
        {
            performance_fix = BMM350_AVERAGING_4;
        }

        /* ODR is an enum taking the generated constants from the register map */
        reg_data = ((uint8_t)odr & BMM350_ODR_MSK);

        /* AVG / performance is an enum taking the generated constants from the register map */
        reg_data = BMM350_SET_BITS(reg_data, BMM350_AVG, (uint8_t)performance_fix);

        /* Set PMU command configurations for ODR and performance */
        rslt = bmm350_set_regs(BMM350_REG_PMU_CMD_AGGR_SET, &reg_data, 1, dev);

        if (rslt == BMM350_OK)
        {
            /* Set PMU command configurations to update odr and average */
            reg_data = BMM350_PMU_CMD_UPD_OAE;

            /* Set PMU command configuration */
            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &reg_data, 1, dev);

            if (rslt == BMM350_OK)
            {
                rslt = bmm350_delay_us(BMM350_UPD_OAE_DELAY, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to enable or disable the magnetic
 * measurement of x,y,z axes
 */


/* ===== RT-Thread device driver ===== */

#define BMM350_PAYLOAD_BYTES 12u

static uint8_t g_mag_address;

static BMM350_INTF_RET_TYPE bmm350_bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *bus)
{
    if (!bus || !data || len == 0u || len > 128u)
    {
        return -1;
    }

    struct rt_i2c_msg messages[] = {
        { .addr = g_mag_address, .flags = RT_I2C_WR, .buf = &reg, .len = 1u },
        { .addr = g_mag_address, .flags = RT_I2C_RD, .buf = data, .len = (rt_uint16_t)len },
    };
    return rt_i2c_transfer(bus, messages, 2u) == 2 ? BMM350_INTF_RET_SUCCESS : -1;
}

static BMM350_INTF_RET_TYPE bmm350_bus_write(uint8_t reg, const uint8_t *data, uint32_t len, void *bus)
{
    uint8_t bytes[33];
    if (!bus || !data || len == 0u || len >= sizeof(bytes))
    {
        return -1;
    }

    bytes[0] = reg;
    memcpy(bytes + 1, data, len);
    struct rt_i2c_msg message = {
        .addr = g_mag_address, .flags = RT_I2C_WR, .buf = bytes, .len = (rt_uint16_t)(len + 1u),
    };
    return rt_i2c_transfer(bus, &message, 1u) == 1 ? BMM350_INTF_RET_SUCCESS : -1;
}

static void bmm350_bus_delay_us(uint32_t us, void *bus)
{
    (void)bus;
    if (us >= 1000u)
        rt_thread_mdelay((us + 999u) / 1000u);
    else
        rt_hw_us_delay(us);
}

static struct bmm350_dev g_bmm;
static struct rt_device g_mag_dev;
static struct rt_mutex g_mag_lock;
static rt_device_t g_timebase;
static struct mag_stats g_mag_stats;
static uint32_t g_mag_sequence;
static uint8_t g_mag_ready;  /* part configured, bus and timebase held */
static uint8_t g_mag_opened; /* single owner currently holds the device open */

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

static rt_err_t mag_configure(void)
{
    uint8_t readback;

    struct rt_i2c_bus_device *bus = rt_i2c_bus_device_find(MAG_I2C_BUS_NAME);
    if (!bus)
    {
        return -RT_EIO;
    }

    g_bmm.intf_ptr = bus;
    g_bmm.read = bmm350_bus_read;
    g_bmm.write = bmm350_bus_write;
    g_bmm.delay_us = bmm350_bus_delay_us;
    for (g_mag_address = BMM350_I2C_ADSEL_SET_LOW;
         g_mag_address <= BMM350_I2C_ADSEL_SET_HIGH; ++g_mag_address)
    {
        if (bmm350_init(&g_bmm) == BMM350_OK)
        {
            break;
        }
    }
    if (g_mag_address > BMM350_I2C_ADSEL_SET_HIGH)
    {
        return -RT_EIO;
    }
    if (bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_2, &g_bmm) ||
        bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &g_bmm) ||
        bmm350_configure_interrupt(BMM350_LATCHED, BMM350_ACTIVE_HIGH, BMM350_INTR_PUSH_PULL,
                                   BMM350_MAP_TO_PIN, &g_bmm) ||
        bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &g_bmm) ||
        bmm350_set_powermode(BMM350_NORMAL_MODE, &g_bmm))
    {
        return -RT_EIO;
    }
    if (bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &readback, 1, &g_bmm) ||
        (readback & 0x3fu) != 0x14u)
    {
        return -RT_EIO;
    }
    if (bmm350_get_regs(BMM350_REG_INT_CTRL, &readback, 1, &g_bmm) || !(readback & 0x80u))
    {
        return -RT_EIO;
    }
    return RT_EOK;
}

static rt_err_t mag_init(rt_device_t dev)
{
    (void)dev;

    if (g_mag_ready)
    {
        return RT_EOK;
    }

    g_timebase = rt_device_find(TIMEBASE_DEVICE_NAME);
    if (!g_timebase || rt_device_open(g_timebase, RT_DEVICE_OFLAG_RDONLY) != RT_EOK)
    {
        g_timebase = NULL;
        return -RT_EIO;
    }
    if (rt_mutex_init(&g_mag_lock, "mag_lock", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        goto close_timebase;
    }
    if (mag_configure() != RT_EOK)
    {
        goto detach_mutex;
    }

    rt_pin_mode(MAG_DATA_READY_PIN, PIN_MODE_INPUT);
    g_mag_ready = 1u;
    return RT_EOK;

detach_mutex:
    (void)rt_mutex_detach(&g_mag_lock);
close_timebase:
    (void)rt_device_close(g_timebase);
    g_timebase = NULL;
    return -RT_EIO;
}

static rt_err_t mag_open(rt_device_t dev, rt_uint16_t oflag)
{
    (void)dev;
    (void)oflag;

    if (!g_mag_ready)
    {
        return -RT_EIO;
    }
    g_mag_opened = 1u;
    return RT_EOK;
}

static rt_err_t mag_close(rt_device_t dev)
{
    (void)dev;
    g_mag_opened = 0u;
    return RT_EOK;
}

static rt_ssize_t mag_read(rt_device_t dev, rt_off_t pos, void *buffer,
                           rt_size_t size)
{
    struct mag_raw_sample *sample = buffer;
    uint8_t ready = 0u;
    int result;
    (void)dev;
    (void)pos;

    if (!sample || size < sizeof(*sample))
    {
        return -RT_EINVAL;
    }
    memset(sample, 0, sizeof(*sample));
    if (!g_mag_ready)
    {
        return -RT_EIO;
    }

    rt_mutex_take(&g_mag_lock, RT_WAITING_FOREVER);
    result = bmm350_get_interrupt_status(&ready, &g_bmm);
    if (!result && !ready)
    {
        rt_mutex_release(&g_mag_lock);
        return 0;
    }
    if (!result)
    {
        result = bmm350_get_regs(BMM350_REG_MAG_X_XLSB, sample->data,
                                 BMM350_PAYLOAD_BYTES, &g_bmm);
    }
    if (!result && timebase_now(&sample->capture_tick))
    {
        result = -1;
    }
    sample->length = BMM350_PAYLOAD_BYTES;
    sample->sequence = ++g_mag_sequence;
    if (result)
    {
        g_mag_stats.errors++;
    }
    else
    {
        g_mag_stats.samples++;
    }
    rt_mutex_release(&g_mag_lock);
    return result ? -RT_EIO : (rt_ssize_t)sizeof(*sample);
}

static rt_err_t mag_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;
    switch (cmd)
    {
    case MAG_CTRL_READ_COMPENSATED:
    {
        struct bmm350_mag_temp_data value;
        struct magnetometer_compensated *out = args;
        int result;
        if (!out)
        {
            return -RT_EINVAL;
        }
        if (!g_mag_ready || !g_mag_opened)
        {
            return -RT_EIO;
        }
        rt_mutex_take(&g_mag_lock, RT_WAITING_FOREVER);
        result = bmm350_get_compensated_mag_xyz_temp_data(&value, &g_bmm);
        rt_mutex_release(&g_mag_lock);
        if (result)
        {
            g_mag_stats.errors++;
            return -RT_EIO;
        }
        out->xyz_ut[0] = value.x;
        out->xyz_ut[1] = value.y;
        out->xyz_ut[2] = value.z;
        out->temperature_c = value.temperature;
        return RT_EOK;
    }
    case MAG_CTRL_GET_STATS:
        if (!args)
        {
            return -RT_EINVAL;
        }
        *(struct mag_stats *)args = g_mag_stats;
        return RT_EOK;
    default:
        return -RT_ENOSYS;
    }
}

static const struct rt_device_ops mag_ops =
{
    .init = mag_init,
    .open = mag_open,
    .close = mag_close,
    .read = mag_read,
    .control = mag_control,
};

static int mag_register(void)
{
    g_mag_dev.type = RT_Device_Class_Sensor;
    g_mag_dev.ops = &mag_ops;
    return rt_device_register(&g_mag_dev, MAG_DEVICE_NAME,
                              RT_DEVICE_FLAG_RDONLY);
}
INIT_DEVICE_EXPORT(mag_register);
