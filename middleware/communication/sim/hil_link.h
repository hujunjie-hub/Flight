#ifndef HIL_LINK_H
#define HIL_LINK_H

#include <stdbool.h>
#include "base_types/common_base.h"
#include "mavlink/mavlink_hil.h"

EXTERN_C_BEGIN

typedef struct {
    uint32_t rx_frames, rx_bytes, discarded_bytes, tx_frames, tx_dropped;
} HilLinkStats;

const char *hil_link_device_name(void);
Status hil_link_open(void);
bool hil_link_is_open(void);
/* Delivers validated frames synchronously with a fixed UART read budget. */
void hil_link_poll_receive(void (*handle_frame)(const mavlink_hil_frame_t *));
void hil_link_send_actuator(uint64_t now_us, const float controls[4], uint8_t mode);
void hil_link_send_heartbeat(void);
HilLinkStats hil_link_stats(void);

EXTERN_C_END
#endif
