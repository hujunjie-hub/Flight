#include "elrs.h"

#include "board_device.h"
#include <rtdevice.h>
#include <string.h>

#define CRSF_ADDRESS_FC 0xC8u
#define CRSF_ADDRESS_RADIO 0xEEu
#define CRSF_ADDRESS_RECEIVER 0xEAu
#define CRSF_TYPE_LINK_STATISTICS 0x14u
#define CRSF_TYPE_RC_CHANNELS 0x16u
#define CRSF_MAX_FRAME 64u

static rt_device_t receiver;
static uint8_t parser_buffer[CRSF_MAX_FRAME];
static unsigned parser_length;
static uint16_t channels[16];
static uint8_t link_quality;
static uint32_t channel_frames;

static uint8_t crc8(const uint8_t *data, unsigned length, uint8_t polynomial)
{
    uint8_t crc = 0;
    for (unsigned index = 0; index < length; ++index) {
        crc ^= data[index];
        for (unsigned bit = 0; bit < 8; ++bit) {
            const unsigned shifted = (unsigned)crc << 1;
            crc = (uint8_t)((crc & 0x80u) ? (shifted ^ polynomial) : shifted);
        }
    }
    return crc;
}

static void unpack_channels(const uint8_t *payload)
{
    uint32_t bits = 0;
    unsigned bit_count = 0;
    unsigned offset = 0;
    for (unsigned channel = 0; channel < 16; ++channel) {
        while (bit_count < 11u && offset < 22u) {
            bits |= (uint32_t)payload[offset++] << bit_count;
            bit_count += 8u;
        }
        channels[channel] = (uint16_t)(bits & 0x7FFu);
        bits >>= 11u;
        bit_count -= 11u;
    }
    ++channel_frames;
}

static bool parse_frame(const uint8_t *frame, unsigned total)
{
    if (total < 4u)
        return false;
    const unsigned length = frame[1];
    if (length < 2u || total != length + 2u)
        return false;
    if (crc8(frame + 2, length - 1u, 0xD5u) != frame[length + 1u])
        return false;
    const uint8_t type = frame[2];
    const uint8_t *payload = frame + 3;
    const unsigned payload_length = length - 2u;
    if (type == CRSF_TYPE_RC_CHANNELS && payload_length >= 22u) {
        unpack_channels(payload);
        if (link_quality == 0u)
            link_quality = 100u;
        return true;
    }
    if (type == CRSF_TYPE_LINK_STATISTICS && payload_length >= 3u) {
        /* Uplink LQ is the third byte: RSSI1, RSSI2, LQ, SNR, ... */
        link_quality = payload[2];
        return true;
    }
    return false;
}

bool elrs_open(void)
{
    if (receiver)
        return true;

    struct serial_configure config;
    rt_device_t device = rt_device_find(RC_UART_NAME);
    if (!device || rt_device_control(device, RT_SERIAL_CTRL_GET_CONFIG, &config) != RT_EOK)
        return false;
    if (config.baud_rate != 115200u) {
        config.baud_rate = 115200u;
        if (rt_device_control(device, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK)
            return false;
    }
    if (rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_SERIAL_RX_NON_BLOCKING |
                       RT_SERIAL_TX_BLOCKING) != RT_EOK)
        return false;
    receiver = device;
    return true;
}

void elrs_encode_bind(uint8_t frame[ELRS_BIND_FRAME_SIZE])
{
    static const uint8_t header[] = {CRSF_ADDRESS_FC, 7u, 0x32u, 0xECu, CRSF_ADDRESS_FC, 0x10u, 0x01u};
    memcpy(frame, header, sizeof(header));
    frame[7] = crc8(frame + 2, 5, 0xBAu);
    frame[8] = crc8(frame + 2, 6, 0xD5u);
}

bool elrs_send_bind(void)
{
    uint8_t frame[ELRS_BIND_FRAME_SIZE];
    if (!elrs_open())
        return false;
    elrs_encode_bind(frame);
    return rt_device_write(receiver, 0, frame, sizeof(frame)) == (rt_ssize_t)sizeof(frame);
}

size_t elrs_read(uint8_t *buffer, size_t capacity)
{
    if (!receiver || !buffer || capacity == 0u)
        return 0u;
    const rt_ssize_t count = rt_device_read(receiver, 0, buffer, capacity);
    return count > 0 ? (size_t)count : 0u;
}

uint32_t elrs_poll_channels(uint16_t out_channels[16], uint8_t *out_link_quality)
{
    uint8_t bytes[128];
    uint32_t frames = 0;
    if (!out_channels || !elrs_open())
        return 0;
    const size_t count = elrs_read(bytes, sizeof(bytes));
    for (size_t index = 0; index < count; ++index) {
        const uint8_t byte = bytes[index];
        if (parser_length == 0u && byte != CRSF_ADDRESS_FC &&
            byte != CRSF_ADDRESS_RADIO && byte != CRSF_ADDRESS_RECEIVER)
            continue;
        if (parser_length == 1u) {
            const unsigned expected = (unsigned)byte + 2u;
            if (expected < 4u || expected > CRSF_MAX_FRAME) {
                parser_length = 0;
                continue;
            }
        }
        parser_buffer[parser_length++] = byte;
        if (parser_length >= 2u) {
            const unsigned expected = (unsigned)parser_buffer[1] + 2u;
            if (parser_length == expected) {
                if (parse_frame(parser_buffer, parser_length))
                    ++frames;
                parser_length = 0;
            }
        }
    }
    memcpy(out_channels, channels, sizeof(channels));
    if (out_link_quality)
        *out_link_quality = link_quality;
    return frames;
}

uint32_t elrs_channel_frames(void)
{
    return channel_frames;
}
