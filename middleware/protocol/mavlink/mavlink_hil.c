#include "mavlink/mavlink_hil.h"
#include <math.h>
#include <string.h>

#define HIL_HEARTBEAT_LEN 9u
#define HIL_HEARTBEAT_CRC 50u
#define HIL_SENSOR_LEN 64u
#define HIL_SENSOR_CRC 108u
#define HIL_GPS_MAX_LEN 37u
#define HIL_GPS_CRC 124u
#define HIL_ACTUATOR_LEN 81u
#define HIL_ACTUATOR_CRC 47u

/* MAVLink field offsets (payload byte 0 is the first wire byte). */
enum {
    HIL_SENSOR_TIME_USEC = 0,
    HIL_SENSOR_XACC = 8,
    HIL_SENSOR_YACC = 12,
    HIL_SENSOR_ZACC = 16,
    HIL_SENSOR_XGYRO = 20,
    HIL_SENSOR_YGYRO = 24,
    HIL_SENSOR_ZGYRO = 28,
    HIL_SENSOR_XMAG = 32,
    HIL_SENSOR_YMAG = 36,
    HIL_SENSOR_ZMAG = 40,
    HIL_SENSOR_ABS_PRESSURE = 44,
    HIL_SENSOR_DIFF_PRESSURE = 48,
    HIL_SENSOR_PRESSURE_ALT = 52,
    HIL_SENSOR_TEMPERATURE = 56,
    HIL_SENSOR_FIELDS_UPDATED = 60
};

enum {
    HIL_GPS_TIME_USEC = 0,
    HIL_GPS_LAT = 8,
    HIL_GPS_LON = 12,
    HIL_GPS_ALT = 16,
    HIL_GPS_EPH = 20,
    HIL_GPS_EPV = 22,
    HIL_GPS_VEL = 24,
    HIL_GPS_VN = 26,
    HIL_GPS_VE = 28,
    HIL_GPS_VD = 30,
    HIL_GPS_COG = 32,
    HIL_GPS_FIX_TYPE = 34,
    HIL_GPS_SATELLITES = 35,
    HIL_GPS_ID = 36
};

enum {
    HIL_ACTUATOR_TIME_USEC = 0,
    HIL_ACTUATOR_FLAGS = 8,
    HIL_ACTUATOR_CONTROLS = 16, /* 16 floats, 64 bytes */
    HIL_ACTUATOR_MODE = 80
};

static uint16_t crc_extra(uint32_t id)
{
    switch (id)
    {
    case MAVLINK_HIL_MSG_HEARTBEAT: return HIL_HEARTBEAT_CRC;
    case MAVLINK_HIL_MSG_ACTUATOR_CONTROLS: return HIL_ACTUATOR_CRC;
    case MAVLINK_HIL_MSG_SENSOR: return HIL_SENSOR_CRC;
    case MAVLINK_HIL_MSG_GPS: return HIL_GPS_CRC;
    default: return 0u;
    }
}

static uint8_t max_payload(uint32_t id)
{
    switch (id)
    {
    case MAVLINK_HIL_MSG_HEARTBEAT: return HIL_HEARTBEAT_LEN;
    case MAVLINK_HIL_MSG_SENSOR: return HIL_SENSOR_LEN;
    case MAVLINK_HIL_MSG_GPS: return HIL_GPS_MAX_LEN;
    case MAVLINK_HIL_MSG_ACTUATOR_CONTROLS: return HIL_ACTUATOR_LEN;
    default: return 0u;
    }
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t read_u64(const uint8_t *p)
{
    return (uint64_t)read_u32(p) | ((uint64_t)read_u32(p + 4) << 32);
}

static int32_t read_i32(const uint8_t *p)
{
    return (int32_t)read_u32(p);
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_i16(const uint8_t *p)
{
    return (int16_t)read_u16(p);
}

static float read_f32(const uint8_t *p)
{
    uint32_t bits = read_u32(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint16_t frame_crc(const uint8_t *frame, unsigned crc_offset, uint32_t id)
{
    uint16_t crc = 0xFFFFu;
    for (unsigned i = 1u; i < crc_offset; ++i)
        crc = mavlink_crc_accumulate(crc, frame[i]);
    return mavlink_crc_accumulate(crc, (uint8_t)crc_extra(id));
}

static void rx_take(mavlink_hil_rx_t *rx, unsigned count)
{
    if (count && count < rx->length)
        memmove(rx->data, rx->data + count, rx->length - count);
    rx->length = (uint16_t)(rx->length - count);
}

uint16_t mavlink_hil_rx_feed(mavlink_hil_rx_t *rx, const uint8_t *data, uint16_t count)
{
    uint16_t room = (uint16_t)(MAVLINK_HIL_RX_CAPACITY - rx->length);
    uint16_t take = count < room ? count : room;
    if (!data || !take)
    {
        rx->dropped += count;
        return 0u;
    }
    memcpy(rx->data + rx->length, data, take);
    rx->length = (uint16_t)(rx->length + take);
    rx->dropped += (uint16_t)(count - take);
    return take;
}

/* 1: one frame consumed into out. 0: need more bytes (or the buffer is now
 * empty). Unknown message ids are skipped inside the loop. */
int mavlink_hil_rx_next(mavlink_hil_rx_t *rx, mavlink_hil_frame_t *out)
{
    unsigned skipped = 0;
    /* Scan once and compact only after consuming a frame or reaching a suffix
     * that needs more bytes. An unknown header cannot delay a valid frame. */
    for (; skipped < rx->length; ++skipped)
    {
        const uint8_t *frame = rx->data + skipped;
        const unsigned available = rx->length - skipped;
        if (frame[0] != MAVLINK_V2_MAGIC)
            continue;
        if (available < MAVLINK_V2_HEADER_SIZE)
            break;
        const uint8_t payload_length = frame[1];
        const uint32_t id = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16);
        if (!payload_length || payload_length > max_payload(id) || frame[2] != 0u)
            continue;
        const unsigned crc_offset = MAVLINK_V2_HEADER_SIZE + payload_length;
        const unsigned frame_length = crc_offset + MAVLINK_CRC_SIZE;
        if (available < frame_length)
            break;
        if (frame_crc(frame, crc_offset, id) != read_u16(frame + crc_offset))
            continue;
        out->id = id;
        out->sequence = frame[4];
        out->length = payload_length;
        memcpy(out->payload, frame + MAVLINK_V2_HEADER_SIZE, payload_length);
        memset(out->payload + payload_length, 0, sizeof(out->payload) - payload_length);
        rx->dropped += skipped;
        rx_take(rx, skipped + frame_length);
        return 1;
    }
    rx->dropped += skipped;
    rx_take(rx, skipped);
    return 0;
}

int mavlink_hil_frame_is_sensor(const mavlink_hil_frame_t *frame)
{
    return frame->id == MAVLINK_HIL_MSG_SENSOR && frame->length <= HIL_SENSOR_LEN;
}

int mavlink_hil_frame_is_gps(const mavlink_hil_frame_t *frame)
{
    /* A nonzero fix is needed; a zero satellite count may be truncated. */
    return frame->id == MAVLINK_HIL_MSG_GPS && frame->length > HIL_GPS_FIX_TYPE;
}

void mavlink_hil_decode_sensor(const mavlink_hil_frame_t *frame, mavlink_hil_sensor_t *out)
{
    memset(out, 0, sizeof(*out));
    out->sim_time_us = read_u64(frame->payload + HIL_SENSOR_TIME_USEC);
    out->accel_mps2[0] = read_f32(frame->payload + HIL_SENSOR_XACC);
    out->accel_mps2[1] = read_f32(frame->payload + HIL_SENSOR_YACC);
    out->accel_mps2[2] = read_f32(frame->payload + HIL_SENSOR_ZACC);
    out->gyro_radps[0] = read_f32(frame->payload + HIL_SENSOR_XGYRO);
    out->gyro_radps[1] = read_f32(frame->payload + HIL_SENSOR_YGYRO);
    out->gyro_radps[2] = read_f32(frame->payload + HIL_SENSOR_ZGYRO);
    out->mag_gauss[0] = read_f32(frame->payload + HIL_SENSOR_XMAG);
    out->mag_gauss[1] = read_f32(frame->payload + HIL_SENSOR_YMAG);
    out->mag_gauss[2] = read_f32(frame->payload + HIL_SENSOR_ZMAG);
    out->abs_pressure_hpa = read_f32(frame->payload + HIL_SENSOR_ABS_PRESSURE);
    out->pressure_alt_m = read_f32(frame->payload + HIL_SENSOR_PRESSURE_ALT);
    out->temperature_c = read_f32(frame->payload + HIL_SENSOR_TEMPERATURE);
    out->fields_updated = read_u32(frame->payload + HIL_SENSOR_FIELDS_UPDATED);
}

void mavlink_hil_decode_gps(const mavlink_hil_frame_t *frame, mavlink_hil_gps_t *out)
{
    memset(out, 0, sizeof(*out));
    out->sim_time_us = read_u64(frame->payload + HIL_GPS_TIME_USEC);
    out->latitude_deg = (double)read_i32(frame->payload + HIL_GPS_LAT) * 1e-7;
    out->longitude_deg = (double)read_i32(frame->payload + HIL_GPS_LON) * 1e-7;
    out->altitude_m = (float)((double)read_i32(frame->payload + HIL_GPS_ALT) * 1e-3);
    out->eph_m = (float)read_u16(frame->payload + HIL_GPS_EPH) * 0.01f;
    out->epv_m = (float)read_u16(frame->payload + HIL_GPS_EPV) * 0.01f;
    out->velocity_ned_mps[0] = (float)read_i16(frame->payload + HIL_GPS_VN) * 0.01f;
    out->velocity_ned_mps[1] = (float)read_i16(frame->payload + HIL_GPS_VE) * 0.01f;
    out->velocity_ned_mps[2] = (float)read_i16(frame->payload + HIL_GPS_VD) * 0.01f;
    out->fix_type = frame->payload[HIL_GPS_FIX_TYPE];
    out->satellites = frame->payload[HIL_GPS_SATELLITES];
}

/* FMT dialect, mavlink_msg_fmt_external_state.h. */
enum { EXTERNAL_STATE_ID = 12001, EXTERNAL_STATE_CRC = 168, EXTERNAL_STATE_SIZE = 88 };

static int read_float_vector(const uint8_t *p, float *out, unsigned count)
{
    for (unsigned i = 0; i < count; ++i) {
        out[i] = read_f32(p + 4u * i);
        if (!isfinite(out[i]))
            return 0;
    }
    return 1;
}

int mavlink_hil_decode_fmt_state(const uint8_t *frame, size_t size, mavlink_fmt_state_t *out)
{
    if (!frame || !out || size < MAVLINK_V2_HEADER_SIZE + MAVLINK_CRC_SIZE ||
        frame[0] != MAVLINK_V2_MAGIC)
        return 0;

    const unsigned length = frame[1];
    const uint8_t *p = frame + MAVLINK_V2_HEADER_SIZE;
    uint8_t payload[EXTERNAL_STATE_SIZE] = {0};
    uint16_t crc = 0xffff;
    mavlink_fmt_state_t state;
    float q_norm = 0.0f;

    /* The FMT dialect has trailing zero truncation in MAVLink 2. A usable
     * state includes the status byte even when the upper bytes were trimmed. */
    if (frame[2] != 0 || length < 85u || length > sizeof(payload) ||
        size != MAVLINK_V2_HEADER_SIZE + length + MAVLINK_CRC_SIZE ||
        ((unsigned)frame[7] | (unsigned)frame[8] << 8 |
         (unsigned)frame[9] << 16) != EXTERNAL_STATE_ID)
        return 0;
    for (unsigned i = 1; i < MAVLINK_V2_HEADER_SIZE + length; ++i)
        crc = mavlink_crc_accumulate(crc, frame[i]);
    crc = mavlink_crc_accumulate(crc, EXTERNAL_STATE_CRC);
    if (p[length] != (uint8_t)crc || p[length + 1u] != (uint8_t)(crc >> 8))
        return 0;

    memcpy(payload, p, length);
    if (read_i32(payload + 84) == 0 ||
        !read_float_vector(payload + 4, state.quaternion, 4) ||
        !read_float_vector(payload + 20, state.gyro_radps, 3) ||
        !read_float_vector(payload + 32, state.specific_force_ned_mps2, 3) ||
        !read_float_vector(payload + 44, state.velocity_ned_mps, 3) ||
        !read_float_vector(payload + 56, state.position_ned_m, 3))
        return 0;
    for (unsigned i = 0; i < 4; ++i)
        q_norm += state.quaternion[i] * state.quaternion[i];
    if (q_norm < 0.9f || q_norm > 1.1f)
        return 0;
    q_norm = sqrtf(q_norm);
    for (unsigned i = 0; i < 4; ++i)
        state.quaternion[i] /= q_norm;
    state.latitude_deg = (double)read_i32(payload + 68) * 1e-7;
    state.longitude_deg = (double)read_i32(payload + 72) * 1e-7;
    state.altitude_m = (float)read_i32(payload + 76) * 0.001f;
    if (fabs(state.latitude_deg) > 90.0 || fabs(state.longitude_deg) > 180.0)
        return 0;
    *out = state;
    return 1;
}

size_t mavlink_hil_build_actuator(uint64_t time_us, const float controls[4], uint8_t mode,
                                     uint8_t sequence, uint8_t *out, size_t capacity)
{
    uint8_t payload[HIL_ACTUATOR_LEN];
    unsigned i;
    memset(payload, 0, sizeof(payload));
    wire_u64(payload + HIL_ACTUATOR_TIME_USEC, time_us);
    wire_u64(payload + HIL_ACTUATOR_FLAGS, 0u);
    for (i = 0; i < 4u; ++i)
    {
        wire_float(payload + HIL_ACTUATOR_CONTROLS + 4u * i, controls[i]);
    }
    payload[HIL_ACTUATOR_MODE] = mode;
    return wire_encode(MAVLINK_HIL_MSG_ACTUATOR_CONTROLS, HIL_ACTUATOR_CRC, payload,
                       sizeof(payload), sequence, out, capacity);
}

size_t mavlink_hil_build_heartbeat(uint8_t sequence, uint8_t *out, size_t capacity)
{
    /* custom_mode, type=MAV_TYPE_QUADROTOR, autopilot=GENERIC, base_mode=0,
     * system_status=STANDBY, mavlink_version=3. Wire order sorts by field
     * width, so the u32 custom_mode comes first (offset 0..3) and type is at
     * offset 4. */
    uint8_t payload[HIL_HEARTBEAT_LEN] = {0u, 0u, 0u, 0u, 2u, 0u, 0u, 3u, 3u};
    return wire_encode(MAVLINK_HIL_MSG_HEARTBEAT, HIL_HEARTBEAT_CRC, payload, sizeof(payload), sequence,
                       out, capacity);
}

