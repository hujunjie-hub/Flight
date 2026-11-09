#include "mavlink_rtcm.h"
#include <string.h>

enum { RTCM_TIMEOUT_US = 500000 };

static bool valid_crc(const uint8_t *data, size_t size)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < size - 3u; ++i) {
        crc ^= (uint32_t)data[i] << 16;
        for (unsigned bit = 0; bit < 8u; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= 0x1864cfbu;
        }
    }
    const uint32_t expected = (uint32_t)data[size - 3u] << 16 |
                              (uint32_t)data[size - 2u] << 8 | data[size - 1u];
    return crc == expected;
}

static void feed(MavlinkRtcm *rtcm, const uint8_t *data, size_t length, MavlinkRtcmSink sink)
{
    for (size_t i = 0; i < length; ++i) {
        const uint8_t byte = data[i];
        if (!rtcm->frame_length && byte != 0xd3u) continue;
        if (rtcm->frame_length == 1u && byte > 3u) {
            rtcm->frame_length = byte == 0xd3u ? 1u : 0u;
            continue;
        }
        rtcm->frame[rtcm->frame_length++] = byte;
        if (rtcm->frame_length == 3u) {
            rtcm->frame_expected = (uint16_t)((rtcm->frame[1] << 8) | byte) + 6u;
        }
        if (rtcm->frame_length >= 6u && rtcm->frame_length == rtcm->frame_expected) {
            if (valid_crc(rtcm->frame, rtcm->frame_length))
                sink(rtcm->frame, rtcm->frame_length);
            rtcm->frame_length = 0u;
        }
    }
}

void mavlink_rtcm_receive(MavlinkRtcm *rtcm, const uint8_t p[182], uint16_t source,
                          uint64_t now_us, MavlinkRtcmSink sink)
{
    const uint8_t flags = p[0], length = p[1];
    const uint8_t seq = flags >> 3;
    if (length > MAVLINK_RTCM_CHUNK) return;

    if ((flags & 1u) == 0u) {
        rtcm->active = false;
        if (!rtcm->stream_active || rtcm->stream_source != source ||
            rtcm->next_sequence != seq || now_us - rtcm->stream_time_us > RTCM_TIMEOUT_US)
            rtcm->frame_length = 0u;
        rtcm->stream_active = true;
        rtcm->stream_source = source;
        rtcm->stream_time_us = now_us;
        rtcm->next_sequence = (seq + 1u) & 31u;
        feed(rtcm, p + 2, length, sink);
        return;
    }

    const uint8_t fragment = (flags >> 1) & 3u;
    if (!rtcm->active || rtcm->source != source || rtcm->sequence != seq ||
        now_us - rtcm->started_us > RTCM_TIMEOUT_US) {
        rtcm->active = true;
        rtcm->source = source;
        rtcm->sequence = seq;
        rtcm->started_us = now_us;
        rtcm->received = 0u;
        rtcm->short_fragment = 4u;
    }
    const uint8_t bit = (uint8_t)(1u << fragment);
    if (rtcm->received & bit) return;
    rtcm->received |= bit;
    rtcm->lengths[fragment] = length;
    memcpy(rtcm->data + fragment * MAVLINK_RTCM_CHUNK, p + 2, length);
    if (length < MAVLINK_RTCM_CHUNK && fragment < rtcm->short_fragment)
        rtcm->short_fragment = fragment;
    const uint8_t last = rtcm->short_fragment < 4u ? rtcm->short_fragment : 3u;
    const uint8_t required = (uint8_t)((1u << (last + 1u)) - 1u);
    if ((rtcm->received & required) != required) return;

    rtcm->stream_active = false;
    rtcm->frame_length = 0u;
    if (rtcm->received == required)
        feed(rtcm, rtcm->data, last * MAVLINK_RTCM_CHUNK + rtcm->lengths[last], sink);
    rtcm->active = false;
    rtcm->frame_length = 0u;
}
