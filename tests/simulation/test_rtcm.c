#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mavlink/mavlink_rtcm.h"

static uint8_t expected[1029];
static size_t expected_length;
static unsigned received;

static void sink(const uint8_t *frame, size_t length)
{
    assert(length == expected_length);
    assert(memcmp(frame, expected, length) == 0);
    ++received;
}

static void make_frame(size_t length)
{
    memset(expected, 0, sizeof(expected));
    expected_length = length;
    const size_t payload = length - 6u;
    expected[0] = 0xd3u;
    expected[1] = (uint8_t)(payload >> 8);
    expected[2] = (uint8_t)payload;
    /* Polynomial long division over the complete padded bitstream. */
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        for (unsigned bit = 8; bit != 0; --bit) {
            const bool carry = (crc & 0x800000u) != 0u;
            crc = ((crc << 1) | ((expected[i] >> (bit - 1u)) & 1u)) & 0xffffffu;
            if (carry) crc ^= 0x864cfbu;
        }
    }
    expected[length - 3u] = (uint8_t)(crc >> 16);
    expected[length - 2u] = (uint8_t)(crc >> 8);
    expected[length - 1u] = (uint8_t)crc;
}

static void send(MavlinkRtcm *rtcm, uint8_t flags, unsigned length, unsigned offset,
                 uint16_t source, uint64_t time)
{
    uint8_t payload[182] = {flags, (uint8_t)length};
    if (length <= MAVLINK_RTCM_CHUNK) memcpy(payload + 2, expected + offset, length);
    mavlink_rtcm_receive(rtcm, payload, source, time, sink);
}

int main(void)
{
    MavlinkRtcm rtcm = {0};
    make_frame(64);
    send(&rtcm, 0, 64, 0, 0xffbe, 0);
    assert(received == 1u);
    expected[12] ^= 1u;
    send(&rtcm, 8, 64, 0, 0xffbe, 1);
    send(&rtcm, 8, 181, 0, 0xffbe, 2);
    assert(received == 1u);

    make_frame(360);
    send(&rtcm, 17, 180, 0, 0xffbe, 3);
    send(&rtcm, 17, 180, 0, 0xffbe, 4);
    send(&rtcm, 21, 0, 360, 0xffbe, 5);
    assert(received == 1u);
    send(&rtcm, 19, 180, 180, 0xffbe, 6);
    assert(received == 2u);
    send(&rtcm, 25, 180, 0, 0xffbe, 7);
    send(&rtcm, 27, 180, 180, 0xffbe, 600000);
    send(&rtcm, 29, 0, 360, 0xffbe, 600001);
    assert(received == 2u);

    make_frame(1029);
    for (unsigned i = 0; i < 6; ++i)
        send(&rtcm, (uint8_t)(((29u + i) & 31u) << 3), i == 5u ? 129u : 180u, i * 180u,
             0xffbe, 700000 + i);
    assert(received == 3u);
    for (unsigned i = 0; i < 6; ++i)
        send(&rtcm, (uint8_t)(i << 3), i == 5u ? 129u : 180u, i * 180u,
             i == 1u ? 0xfebe : 0xffbe, 800000 + i);
    assert(received == 3u);
    puts("RTCM CRC, reassembly, expiry, source isolation and sequence wrap passed");
    return 0;
}
