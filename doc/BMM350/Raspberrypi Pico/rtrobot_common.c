/*********************************************************
 * rtrobot_common.c
 * Copyright (c) 2012 - 2024 RTrobot Inc.
 *
 * Unless otherwise stated, the use of this software is subject to the following conditions:
 * 1. Any form of redistribution must include the original copyright notice and the following disclaimer.
 * 2. Not for commercial use without explicit written permission, including but not limited to sales, licensing, or
 *commercial support.
 * 3. Any modifications to this software must be clearly marked with attribution and documented in the modified files.
 * 4. If modifications are made, they must be clearly indicated in the modified files.
 * 5. Without explicit written permission, the names of the authors or original contributors may not be used to endorse
 *or promote derived products.
 *
 * This software is provided "as is," without any warranties of any kind, express or implied, including but not limited
 *to the warranties of merchantability or fitness for a particular purpose. The authors are not liable for any direct,
 *indirect, incidental, special, exemplary, or consequential damages arising in any way out of the use of this software.
 *********************************************************/
#include "rtrobot_common.h"
#include <stdlib.h>
#include "hardware/i2c.h"


/***************************************************************************************************************
i2c master initialization
****************************************************************************************************************/
void rtrobot_i2c_init(void)
{
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

/***************************************************************************************************************
RTrobot I2C Read Command
****************************************************************************************************************/
int8_t rtrobot_I2C_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint32_t length, void *intf_ptr)
{
	uint8_t device_addr = *(uint8_t *)intf_ptr;
	(void)intf_ptr;
	i2c_write_blocking(i2c_default, device_addr, &reg_addr, 1, true);
	int8_t ret = i2c_read_blocking(i2c_default, device_addr, rev_data, length, false);
	if (ret == length)
		return 0;
	return -1;
}

/***************************************************************************************************************
RTrobot I2C Write Command
****************************************************************************************************************/
int8_t rtrobot_I2C_WriteCommand(uint8_t reg_addr, uint8_t const *send_data, uint32_t length, void *intf_ptr)
{
	uint8_t device_addr = *(uint8_t *)intf_ptr;
	(void)intf_ptr;
	uint8_t *reg = malloc(length + 2);
	reg[0] = reg_addr;
	for (int i = 0; i < length; i++)
		reg[i + 1] = send_data[i];
	int8_t ret = i2c_write_blocking(i2c_default, device_addr, reg, length + 1, false);
	free(reg);
	if (ret - 1 == length)
		return 0;
	return -1;
}


