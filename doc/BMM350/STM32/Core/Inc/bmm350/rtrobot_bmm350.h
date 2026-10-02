/*********************************************************
 * rtrobot_bmm350.h
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
#ifndef RTROBOT_BMM350_H
#define RTROBOT_BMM350_H
#include "../main.h"
#include "bmm350_defs.h"

void rtrobot_bmm350_init(struct bmm350_dev* dev);
void rtrobot_bmm350_test_raw(struct bmm350_dev* dev);
void rtrobot_bmm350_test_compensated_magnetometer(struct bmm350_dev* dev);

#endif //RTROBOT_BMM350_H
