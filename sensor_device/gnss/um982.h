#ifndef UM982_DEVICE_H
#define UM982_DEVICE_H

#include <stdbool.h>
#include <rtdevice.h>

rt_device_t um982_open(void);
bool um982_request_navigation_logs(rt_device_t device);

#endif
