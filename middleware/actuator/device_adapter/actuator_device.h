#ifndef ACTUATOR_DEVICE_H
#define ACTUATOR_DEVICE_H

#include "base_types/common_base.h"

Status actuator_device_start(void);
void actuator_device_close(void);
void actuator_device_inhibit(void);
uint32_t actuator_device_faults(void);
Status actuator_device_commit(const uint16_t commands[4], uint64_t *actual_apply_us);
/* Nonblocking poll: STATUS_NOT_READY while the submitted DShot frame is in flight. */
Status actuator_device_poll_apply(uint64_t *actual_apply_us);

#endif
