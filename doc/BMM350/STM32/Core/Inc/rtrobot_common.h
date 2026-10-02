/*********************************************************
 * rtrobot_common.h
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
#ifndef RTROBOT_COMMON_H
#define RTROBOT_COMMON_H

#include "main.h"


#define READ_WRITE_LEN  UINT8_C(8)


int8_t rtrobot_I2C_ReadCommand(uint8_t reg_addr, uint8_t* rev_data, uint32_t length, void* intf_ptr);
int8_t rtrobot_I2C_WriteCommand(uint8_t reg_addr, uint8_t const* send_data, uint32_t length, void* intf_ptr);
int8_t rtrobot_SPI_ReadCommand(uint8_t reg_addr, uint8_t* rev_data, uint32_t length, void* intf_ptr);
int8_t rtrobot_SPI_WriteCommand(uint8_t reg_addr, uint8_t const* send_data, uint32_t length, void* intf_ptr);

#endif //RTROBOT_COMMON_H
