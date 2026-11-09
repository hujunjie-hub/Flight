#ifndef HIL_SERIAL_H
#define HIL_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

const char *hil_serial_device_name(void);
bool hil_serial_open(void);
bool hil_serial_is_open(void);
size_t hil_serial_read(uint8_t *buffer, size_t capacity);
size_t hil_serial_write(const uint8_t *data, size_t length);

#endif
