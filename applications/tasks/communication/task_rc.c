#include "task_comm.h"
#include "calibration/sensor_calibration_service.h"
#include "mavlink/fcu_mavlink_layout.h"
#include "mavlink/mavlink_frame.h"
#include "messages/common_messages.h"
#include "runtime_internal.h"
#include "parameter_store.h"
#include "telemetry_link/telemetry_link.h"
#include <rtthread.h>
#include <string.h>

/* CRSF raw channel range is 172..1811 (11 bit), centre 992. */
enum {
    CRSF_RAW_MIN = 172,
    CRSF_RAW_MAX = 1811,
    CRSF_RAW_CENTER = 992,
    RC_LINK_TIMEOUT_US = 500000,
    RC_CHANNELS_CRC = 118,
};

static uint32_t received_frames;
static uint16_t channels[16];
static uint32_t last_frame_us;
static uint8_t link_quality;

void comm_rc_poll(void)
{
    uint16_t latest[16];
    uint8_t quality = 0;
    const uint32_t frames = telemetry_link_poll_rc_channels(latest, &quality);
    if (frames) {
        memcpy(channels, latest, sizeof(channels));
        const uint64_t now_us = runtime_now_us();
        last_frame_us = (uint32_t)now_us;
        received_frames += frames;
        sensor_calibration_feed_rc(channels, now_us);
    }
    if (quality)
        link_quality = quality;
    else if (last_frame_us && (uint32_t)runtime_now_us() - last_frame_us > RC_LINK_TIMEOUT_US)
        link_quality = 0;
}

int comm_rc_status(uint64_t now_us)
{
    if (!last_frame_us)
        return 0;
    return (uint32_t)now_us - last_frame_us <= RC_LINK_TIMEOUT_US ? 1 : 2;
}

static uint16_t raw_to_us(uint16_t raw)
{
    if (raw <= CRSF_RAW_MIN)
        return 1000;
    if (raw >= CRSF_RAW_MAX)
        return 2000;
    return (uint16_t)(1000u + ((uint32_t)(raw - CRSF_RAW_MIN) * 1000u) /
                                 (CRSF_RAW_MAX - CRSF_RAW_MIN));
}

/* Convert the configured AUX channel using the same PWM domain as RC_CHANNELS. */
uint32_t comm_rc_mapped_mode(const uint16_t raw[16])
{
    static const char *const slots[] = {
        "RC_MODE_1", "RC_MODE_2", "RC_MODE_3", "RC_MODE_4", "RC_MODE_5", "RC_MODE_6"
    };
    const unsigned channel = (unsigned)parameter_store_value((unsigned)parameter_store_find("RC_MODE_CH"));
    static const char *const auxiliary[] = {
        "RC_AUX_6", "RC_AUX_7", "RC_AUX_8", "RC_AUX_9", "RC_AUX_10", "RC_AUX_11",
        "RC_AUX_12", "RC_AUX_13", "RC_AUX_14", "RC_AUX_15", "RC_AUX_16"
    };
    uint32_t mapped = 0u;
    for (unsigned i = 0; i < sizeof(auxiliary) / sizeof(auxiliary[0]); ++i) {
        const unsigned aux_channel = i + 6u;
        if (aux_channel == channel)
            continue;
        const unsigned mode = (unsigned)parameter_store_value((unsigned)parameter_store_find(auxiliary[i]));
        if (mode == 0u)
            continue;
        mapped = PRODUCT_MODE_SWITCH_MAPPED;
        if (raw_to_us(raw[aux_channel - 1u]) >= 1750u)
            return mapped | mode;
    }
    if (channel < 5u || channel > 16u)
        return mapped;
    const uint16_t pwm = raw_to_us(raw[channel - 1u]);
    const unsigned slot = (pwm > 1230u) + (pwm > 1360u) + (pwm > 1490u) +
                          (pwm > 1620u) + (pwm > 1749u);
    const unsigned mode = (unsigned)parameter_store_value((unsigned)parameter_store_find(slots[slot]));
    return PRODUCT_MODE_SWITCH_MAPPED | mode;
}

static float axis_norm(uint16_t raw)
{
    const float center = (float)CRSF_RAW_CENTER;
    if (raw >= CRSF_RAW_CENTER)
        return ((float)raw - center) / ((float)CRSF_RAW_MAX - center);
    return ((float)raw - center) / (center - (float)CRSF_RAW_MIN);
}

static float throttle_norm(uint16_t raw)
{
    if (raw <= CRSF_RAW_MIN)
        return 0.0f;
    if (raw >= CRSF_RAW_MAX)
        return 1.0f;
    return (float)(raw - CRSF_RAW_MIN) / (float)(CRSF_RAW_MAX - CRSF_RAW_MIN);
}

static void write_standard_rc_channels(uint8_t *payload, uint32_t now_ms,
                                       const uint16_t *raw, uint8_t count, uint8_t quality)
{
    wire_u32(payload, now_ms);
    for (unsigned channel = 0; channel < 18; ++channel)
        wire_u16(payload + 4 + channel * 2u, channel < count ? raw_to_us(raw[channel]) : UINT16_MAX);
    payload[40] = count;
    payload[41] = quality;
}

void comm_rc_publish_status(uint8_t *payload)
{
    const uint32_t now = (uint32_t)runtime_now_us();
    const uint32_t age = last_frame_us ? now - last_frame_us : UINT32_MAX;
    const bool fresh = last_frame_us != 0u && age <= RC_LINK_TIMEOUT_US;

    wire_u32(payload + FCU_RC_INPUT_STATUS_AGE_US, fresh ? age : UINT32_MAX);
    payload[FCU_RC_INPUT_STATUS_RECEIVER_FAILSAFE] = fresh ? 0u : 1u;
    payload[FCU_RC_INPUT_STATUS_LINK_STATE] = fresh ? LINK_STATE_HEALTHY : LINK_STATE_UNKNOWN;
    wire_u32(payload + FCU_RC_INPUT_STATUS_VALID_AXES_MASK, fresh ? 0x0fu : 0u);
    wire_float(payload + FCU_RC_INPUT_STATUS_ROLL_NORM, fresh ? axis_norm(channels[0]) : 0.0f);
    wire_float(payload + FCU_RC_INPUT_STATUS_PITCH_NORM, fresh ? axis_norm(channels[1]) : 0.0f);
    wire_float(payload + FCU_RC_INPUT_STATUS_YAW_NORM, fresh ? axis_norm(channels[3]) : 0.0f);
    wire_float(payload + FCU_RC_INPUT_STATUS_THROTTLE_NORM, fresh ? throttle_norm(channels[2]) : 0.0f);
    telemetry_link_send(FCU_RC_INPUT_STATUS_ID, FCU_RC_INPUT_STATUS_CRC,
                        payload, FCU_RC_INPUT_STATUS_LEN);

    static bool standard_rc_was_fresh;
    if (!fresh && !standard_rc_was_fresh)
        return;
    standard_rc_was_fresh = fresh;
    /* Standard RC_CHANNELS for the QGC channel monitor.  Values are presented
     * in the 1000..2000 us domain QGC expects; the calibration page works with
     * the raw CRSF counts stored in RC_MIN_n/RC_MAX_n. */
    uint8_t p[42];
    write_standard_rc_channels(p, now / 1000u, channels, fresh ? 16u : 0u,
                               fresh ? link_quality : UINT8_MAX);
    telemetry_link_send(RC_CHANNELS, RC_CHANNELS_CRC, p, sizeof(p));
}

void comm_rc_print_diagnostics(void)
{
    const uint32_t age = last_frame_us ? (uint32_t)runtime_now_us() - last_frame_us : UINT32_MAX;
    const uint32_t mapping = age <= RC_LINK_TIMEOUT_US ? comm_rc_mapped_mode(channels) : 0u;
    rt_kprintf("RC frames=%u quality=%u last_age_us=%u mode_switch=0x%x\n",
               received_frames, link_quality, age, mapping);
}
