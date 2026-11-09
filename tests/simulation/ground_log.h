#ifndef FLIGHT_SIL_GROUND_LOG_H
#define FLIGHT_SIL_GROUND_LOG_H

#include <stddef.h>
#include <stdint.h>

enum {
    GROUND_LOG_REQUEST_LIST = 117,
    GROUND_LOG_ENTRY = 118,
    GROUND_LOG_REQUEST_DATA = 119,
    GROUND_LOG_DATA = 120,
    GROUND_LOG_ERASE = 121,
    GROUND_LOG_REQUEST_END = 122,
    GROUND_FRAME_CAPACITY = 120
};

typedef void (*GroundLogEmit)(uint32_t id, uint8_t crc, const uint8_t *payload, size_t length);

void ground_log_record(uint64_t utc_us, const uint8_t *frame, size_t size);
void ground_log_set_armed(int armed);
void ground_log_receive(unsigned id, const uint8_t *payload, uint8_t system,
                        uint8_t component, GroundLogEmit emit);
void ground_log_poll(GroundLogEmit emit);

#endif
