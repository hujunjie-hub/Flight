#ifndef DATA_RADIO_H
#define DATA_RADIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool data_radio_open(void);
void data_radio_close(void);
size_t data_radio_read(uint8_t *buffer, size_t capacity);
/* The TX completion callback releases the next write. Only the I/O task writes. */
bool data_radio_tx_busy(void);
size_t data_radio_write(const uint8_t *data, size_t length);

#endif
