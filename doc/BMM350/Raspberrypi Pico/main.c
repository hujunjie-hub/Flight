/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "rtrobot_bmm350.h"

int main()
{
    stdio_init_all();

    sleep_ms(5000);
    printf("boot......\r\n");

	struct bmm350_dev rtrobot_bmm350_dev = {0};
	rtrobot_bmm350_init(&rtrobot_bmm350_dev);


    while (true)
    {
		//rtrobot_bmm350_test_raw(&rtrobot_bmm350_dev);
		rtrobot_bmm350_test_compensated_magnetometer(&rtrobot_bmm350_dev);
        sleep_ms(10);
    }
    return 0;
}
