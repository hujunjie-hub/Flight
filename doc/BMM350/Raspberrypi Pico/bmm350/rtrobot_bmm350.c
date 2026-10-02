/*********************************************************
 * rtrobot_bmm350.c
 * Copyright (c) 2012 - 2024 RTrobot Inc.
 *  
 * Unless otherwise stated, the use of this software is subject to the following conditions:
 * 1. Any form of redistribution must include the original copyright notice and the following disclaimer.
 * 2. Not for commercial use without explicit written permission, including but not limited to sales, licensing, or commercial support.
 * 3. Any modifications to this software must be clearly marked with attribution and documented in the modified files.
 * 4. If modifications are made, they must be clearly indicated in the modified files.
 * 5. Without explicit written permission, the names of the authors or original contributors may not be used to endorse or promote derived products.
 * 
 * This software is provided "as is," without any warranties of any kind, express or implied, including but not limited to the warranties of merchantability or fitness for a particular purpose.
 * The authors are not liable for any direct, indirect, incidental, special, exemplary, or consequential damages arising in any way out of the use of this software.
 *********************************************************/
#include "rtrobot_bmm350.h"

#include <math.h>
#include <stdio.h>
#include "rtrobot_common.h"
#include "bmm350/bmm350.h"

static uint8_t dev_addr;

void bmm350_delay(uint32_t period, void* intf_ptr)
{
	(void)intf_ptr;
	sleep_us(period);
}

void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
	switch (rslt)
	{
		case BMM350_OK:
			break;

		case BMM350_E_NULL_PTR:
			printf("%s Error [%d] : Null pointer\r\n", api_name, rslt);
			break;
		case BMM350_E_COM_FAIL:
			printf("%s Error [%d] : Communication fail\r\n", api_name, rslt);
			break;
		case BMM350_E_DEV_NOT_FOUND:
			printf("%s Error [%d] : Device not found\r\n", api_name, rslt);
			break;
		case BMM350_E_INVALID_CONFIG:
			printf("%s Error [%d] : Invalid configuration\r\n", api_name, rslt);
			break;
		case BMM350_E_BAD_PAD_DRIVE:
			printf("%s Error [%d] : Bad pad drive\r\n", api_name, rslt);
			break;
		case BMM350_E_RESET_UNFINISHED:
			printf("%s Error [%d] : Reset unfinished\r\n", api_name, rslt);
			break;
		case BMM350_E_INVALID_INPUT:
			printf("%s Error [%d] : Invalid input\r\n", api_name, rslt);
			break;
		case BMM350_E_SELF_TEST_INVALID_AXIS:
			printf("%s Error [%d] : Self-test invalid axis selection\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_BOOT:
			printf("%s Error [%d] : OTP boot\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_PAGE_RD:
			printf("%s Error [%d] : OTP page read\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_PAGE_PRG:
			printf("%s Error [%d] : OTP page prog\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_SIGN:
			printf("%s Error [%d] : OTP sign\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_INV_CMD:
			printf("%s Error [%d] : OTP invalid command\r\n", api_name, rslt);
			break;
		case BMM350_E_OTP_UNDEFINED:
			printf("%s Error [%d] : OTP undefined\r\n", api_name, rslt);
			break;
		case BMM350_E_ALL_AXIS_DISABLED:
			printf("%s Error [%d] : All axis are disabled\r\n", api_name, rslt);
			break;
		case BMM350_E_PMU_CMD_VALUE:
			printf("%s Error [%d] : Unexpected PMU CMD value\r\n", api_name, rslt);
			break;
		default:
			printf("%s Error [%d] : Unknown error code\r\n", api_name, rslt);
			break;
	}
}

void rtrobot_bmm350_init(struct bmm350_dev* dev)
{
	/* Status of api are returned to this variable */
	int8_t rslt = BMM350_OK;

	/* Sensor initialization configuration */
	uint8_t int_ctrl, err_reg_data = 0;
	uint8_t set_int_ctrl;
	struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

	rtrobot_i2c_init();
	/* Update device structure */
	dev_addr = BMM350_I2C_ADSEL_SET_LOW;
	dev->intf_ptr = &dev_addr;
	dev->read = rtrobot_I2C_ReadCommand;
	dev->write = rtrobot_I2C_WriteCommand;
	dev->delay_us = bmm350_delay;

	/* Initialize BMM350 */
	rslt = bmm350_init(dev);
	bmm350_error_codes_print_result("bmm350_init", rslt);
	printf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev->chip_id);
	printf("-------------\nCoefficients\n-------------\n");
	printf("Magnetometer Offset:: X: %.2f, Y: %.2f, Z: %.2f\n",
		dev->mag_comp.dut_offset_coef.offset_x,
		dev->mag_comp.dut_offset_coef.offset_y,
		dev->mag_comp.dut_offset_coef.offset_z);
	printf("Temperature offset %.2f\n", dev->mag_comp.dut_offset_coef.t_offs);
	printf("Magnetometer Sensitivity:: X: %.2f, Y: %.2f, Z: %.2f\n",
		dev->mag_comp.dut_sensit_coef.sens_x,
		dev->mag_comp.dut_sensit_coef.sens_y,
		dev->mag_comp.dut_sensit_coef.sens_z);
	printf("Temperature sensitivity %.2f\n", dev->mag_comp.dut_sensit_coef.t_sens);
	printf("TCO:: X: %.2f, Y: %.2f, Z: %.2f\n",
		dev->mag_comp.dut_tco.tco_x,
		dev->mag_comp.dut_tco.tco_y,
		dev->mag_comp.dut_tco.tco_z);
	printf("TCS:: X: %.4f, Y: %.4f, Z: %.4f\n",
		dev->mag_comp.dut_tcs.tcs_x,
		dev->mag_comp.dut_tcs.tcs_y,
		dev->mag_comp.dut_tcs.tcs_z);
	printf("T0 %.2f\n", dev->mag_comp.dut_t0);
	printf("Cross XY %f\n", dev->mag_comp.cross_axis.cross_x_y);
	printf("Cross YX %f\n", dev->mag_comp.cross_axis.cross_y_x);
	printf("Cross ZX %f\n", dev->mag_comp.cross_axis.cross_z_x);
	printf("Cross ZY %f\n", dev->mag_comp.cross_axis.cross_z_y);
	/* Check PMU busy */
	rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, dev);
	bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

	printf("Expected : 0x07 : PMU cmd busy : 0x0\n");
	printf("Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

	/* Get error data */
	rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, dev);
	bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);

	printf("Expected : 0x02 : Error Register : 0x0\n");
	printf("Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

	/* Configure interrupt settings */
	rslt = bmm350_configure_interrupt(BMM350_PULSED,
		BMM350_ACTIVE_HIGH,
		BMM350_INTR_PUSH_PULL,
		BMM350_UNMAP_FROM_PIN,
		dev);
	bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

	/* Enable data ready interrupt */
	rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, dev);
	bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

	/* Get interrupt settings */
	rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	set_int_ctrl = ((BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | BMM350_ENABLE << 7);

	printf("Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
	printf("Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

	if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
	{
		printf("Data ready enabled\n");
	}

	/* Set ODR and performance */
	rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_50HZ, BMM350_AVERAGING_4, dev);
	bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

	/* Enable all axis */
	rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, dev);
	bmm350_error_codes_print_result("bmm350_enable_axes", rslt);
	rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, dev);
	bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
}

void rtrobot_bmm350_test_compensated_magnetometer(struct bmm350_dev* dev)
{
	uint8_t int_status = 0;
	/* Status of api are returned to this variable */
	int8_t rslt;

	struct bmm350_mag_temp_data mag_temp_data;

	// printf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

	/* Get data ready interrupt status */
	rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	/* Check if data ready interrupt occurred */
	if (int_status & BMM350_DRDY_DATA_REG_MSK)
	{
		rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, dev);
		bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

		printf("%f, %f, %f, %f\n",
			mag_temp_data.x,
			mag_temp_data.y,
			mag_temp_data.z,
			mag_temp_data.temperature);
		printf("$0:%.2f;$1:%.2f;$2:%.2f;\r\n", mag_temp_data.x, mag_temp_data.y, mag_temp_data.z);
	}
}

void rtrobot_bmm350_test_raw(struct bmm350_dev* dev)
{
	uint8_t int_status = 0;
	/* Status of api are returned to this variable */
	int8_t rslt;
	struct bmm350_raw_mag_data raw_data;

	/* Get data ready interrupt status */
	rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	/* Check if data ready interrupt occurred */
	if (int_status & BMM350_DRDY_DATA_REG_MSK)
	{
		/* Get uncompensated mag data */
		rslt = bmm350_read_uncomp_mag_temp_data(&raw_data, dev);
		bmm350_error_codes_print_result("bmm350_read_uncomp_mag_data", rslt);
		uint32_t secs, nano_secs = 0;
		rslt = bmm350_read_sensortime(&secs, &nano_secs, dev);
		bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

		printf("%ld, %ld, %ld, %ld, %lu.%09lu\n",
			(long int)raw_data.raw_xdata,
			(long int)raw_data.raw_ydata,
			(long int)raw_data.raw_zdata,
			(long int)raw_data.raw_data_t,
			(long unsigned int)secs,
			(long unsigned int)nano_secs);
	}
}
