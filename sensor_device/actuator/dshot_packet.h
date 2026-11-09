#ifndef DSHOT_PACKET_H
#define DSHOT_PACKET_H

#include <stdint.h>

static inline uint16_t dshot_encode_packet(uint16_t throttle, uint8_t telemetry)
{
    uint16_t value;
    if (throttle > 2047u || telemetry > 1u)
    {
        return 0u;
    }
    value = (uint16_t)((throttle << 1) | telemetry);
    return (uint16_t)((value << 4) | ((value ^ (value >> 4) ^ (value >> 8)) & 15u));
}

#endif
