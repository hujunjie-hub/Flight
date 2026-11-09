#include "sim/hil_link.h"
#include "telemetry/sim/hil_serial.h"

enum { HIL_RX_CHUNK = 64, HIL_RX_READ_BUDGET = 4, HIL_TX_CAPACITY = 128 };

static mavlink_hil_rx_t rx;
static HilLinkStats stats;
static uint8_t tx_sequence;

const char *hil_link_device_name(void)
{
    return hil_serial_device_name();
}

Status hil_link_open(void)
{
    return hil_serial_open() ? STATUS_OK : STATUS_NOT_READY;
}

bool hil_link_is_open(void)
{
    return hil_serial_is_open();
}

void hil_link_poll_receive(void (*handle_frame)(const mavlink_hil_frame_t *))
{
    uint8_t chunk[HIL_RX_CHUNK];
    for (unsigned round = 0; round < HIL_RX_READ_BUDGET; ++round)
    {
        const size_t count = hil_serial_read(chunk, sizeof(chunk));
        if (count == 0u)
            break;
        stats.rx_bytes += (uint32_t)count;
        (void)mavlink_hil_rx_feed(&rx, chunk, (uint16_t)count);
        mavlink_hil_frame_t frame;
        while (mavlink_hil_rx_next(&rx, &frame))
        {
            ++stats.rx_frames;
            handle_frame(&frame);
        }
    }
}

static void send_frame(const uint8_t *frame, size_t length)
{
    if (length != 0u && hil_serial_write(frame, length) == length)
        ++stats.tx_frames;
    else
        ++stats.tx_dropped;
}

void hil_link_send_actuator(uint64_t now_us, const float controls[4], uint8_t mode)
{
    uint8_t frame[HIL_TX_CAPACITY];
    send_frame(frame, mavlink_hil_build_actuator(now_us, controls, mode, tx_sequence++, frame,
                                               sizeof(frame)));
}

void hil_link_send_heartbeat(void)
{
    uint8_t frame[HIL_TX_CAPACITY];
    send_frame(frame, mavlink_hil_build_heartbeat(tx_sequence++, frame, sizeof(frame)));
}

HilLinkStats hil_link_stats(void)
{
    HilLinkStats snapshot = stats;
    snapshot.discarded_bytes = rx.dropped;
    return snapshot;
}
