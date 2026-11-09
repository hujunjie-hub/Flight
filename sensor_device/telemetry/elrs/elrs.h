#ifndef ELRS_H
#define ELRS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { ELRS_BIND_FRAME_SIZE = 9 };
void elrs_encode_bind(uint8_t frame[ELRS_BIND_FRAME_SIZE]);
bool elrs_send_bind(void);
bool elrs_open(void);
size_t elrs_read(uint8_t *buffer, size_t capacity);

/* Read the UART and decode CRSF frames.  Returns the number of RC_CHANNELS
 * frames decoded in this call; out_channels always holds the latest values. */
uint32_t elrs_poll_channels(uint16_t out_channels[16], uint8_t *out_link_quality);
uint32_t elrs_channel_frames(void);

#endif
