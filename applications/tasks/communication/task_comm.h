#ifndef TASK_COMM_H
#define TASK_COMM_H

#include <stdint.h>

void comm_rc_poll(void);
/* 0: no frames received, 1: fresh frames, 2: stale. */
int comm_rc_status(uint64_t now_us);
uint32_t comm_rc_mapped_mode(const uint16_t raw[16]);
void comm_rc_publish_status(uint8_t *payload);
void comm_rc_print_diagnostics(void);
void comm_radio_receive(void);
void comm_radio_transmit(void);
void comm_gcs_receive(uint8_t id, const uint8_t *payload, uint8_t system, uint8_t component);
void comm_gcs_publish(uint64_t now);

#endif
