#ifndef FLIGHT_MAVLINK_RTCM_H
#define FLIGHT_MAVLINK_RTCM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { MAVLINK_GPS_RTCM_DATA = 233, MAVLINK_RTCM_CHUNK = 180 };

typedef struct {
    uint8_t data[4 * MAVLINK_RTCM_CHUNK];
    uint64_t started_us;
    uint16_t source;
    uint8_t sequence, received, short_fragment;
    uint8_t lengths[4];
    bool active;
    uint8_t frame[1029];
    uint16_t frame_length, frame_expected;
    uint8_t next_sequence;
    bool stream_active;
    uint16_t stream_source;
    uint64_t stream_time_us;
} MavlinkRtcm;

typedef void (*MavlinkRtcmSink)(const uint8_t *data, size_t length);
void mavlink_rtcm_receive(MavlinkRtcm *rtcm, const uint8_t payload[182], uint16_t source,
                          uint64_t now_us, MavlinkRtcmSink sink);

#endif
