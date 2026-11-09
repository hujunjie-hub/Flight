#include "ground_log.h"
#include "mavlink/mavlink_frame.h"
#include <string.h>

enum { RECORD_COUNT = 2048, TIMESTAMP_BYTES = 8, DATA_BYTES = 90, LOG_ID = 0 };
static struct {
    uint64_t utc_us;
    uint8_t frame[GROUND_FRAME_CAPACITY];
    uint8_t size;
} records[RECORD_COUNT];
static unsigned head, count;
static int recording = 1, was_armed;
static struct {
    uint8_t bytes[RECORD_COUNT * (TIMESTAMP_BYTES + GROUND_FRAME_CAPACITY)];
    uint32_t size, offset, end, utc_seconds;
    uint8_t system, component;
} snapshot;

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}

static uint32_t read_u32(const uint8_t *p)
{
    return read_u16(p) | (uint32_t)read_u16(p + 2) << 16;
}

void ground_log_record(uint64_t utc_us, const uint8_t *frame, size_t size)
{
    if (!recording || size > GROUND_FRAME_CAPACITY) return;
    records[head].utc_us = utc_us;
    records[head].size = (uint8_t)size;
    memcpy(records[head].frame, frame, size);
    head = (head + 1) % RECORD_COUNT;
    if (count < RECORD_COUNT) ++count;
}

void ground_log_set_armed(int armed)
{
    if (armed && !was_armed) {
        head = count = 0;
        recording = 1;
    }
    was_armed = armed;
}

static void list_log(const uint8_t *p, uint8_t system, uint8_t component, GroundLogEmit emit)
{
    const unsigned start = read_u16(p);
    snapshot.system = system;
    snapshot.component = component;
    snapshot.size = snapshot.offset = snapshot.end = 0;
    uint8_t entry[14] = {0};
    if (count && start == LOG_ID) {
        const unsigned oldest = (head + RECORD_COUNT - count) % RECORD_COUNT;
        snapshot.utc_seconds = (uint32_t)(records[oldest].utc_us / 1000000u);
        for (unsigned i = 0; i < count; ++i) {
            const unsigned index = (oldest + i) % RECORD_COUNT;
            uint8_t *out = snapshot.bytes + snapshot.size;
            /* QGroundControl telemetry log: big-endian UNIX microseconds, then frame. */
            for (unsigned j = 0; j < TIMESTAMP_BYTES; ++j)
                out[j] = (uint8_t)(records[index].utc_us >> (8u * (TIMESTAMP_BYTES - 1u - j)));
            memcpy(out + TIMESTAMP_BYTES, records[index].frame, records[index].size);
            snapshot.size += TIMESTAMP_BYTES + records[index].size;
        }
        wire_u32(entry, snapshot.utc_seconds);
        wire_u32(entry + 4, snapshot.size);
        wire_u16(entry + 10, 1);
    }
    emit(GROUND_LOG_ENTRY, 56, entry, sizeof(entry));
}

void ground_log_receive(unsigned id, const uint8_t *p, uint8_t system,
                        uint8_t component, GroundLogEmit emit)
{
    if (id == GROUND_LOG_REQUEST_LIST) {
        if (snapshot.offset == snapshot.end ||
            (system == snapshot.system && component == snapshot.component))
            list_log(p, system, component, emit);
        return;
    }
    if (id == GROUND_LOG_ERASE) {
        if (!was_armed) {
            head = count = 0;
            recording = 0;
            snapshot.size = snapshot.offset = snapshot.end = 0;
        }
        return;
    }
    if (system != snapshot.system || component != snapshot.component) return;
    if (id == GROUND_LOG_REQUEST_END) {
        snapshot.offset = snapshot.end = 0;
    } else if (id == GROUND_LOG_REQUEST_DATA && read_u16(p + 8) == LOG_ID) {
        const uint32_t offset = read_u32(p), requested = read_u32(p + 4);
        if (offset >= snapshot.size) {
            uint8_t reply[97] = {0};
            wire_u32(reply, offset);
            emit(GROUND_LOG_DATA, 134, reply, sizeof(reply));
            return;
        }
        const uint32_t remaining = snapshot.size - offset;
        snapshot.offset = offset;
        snapshot.end = offset + (requested < remaining ? requested : remaining);
    }
}

void ground_log_poll(GroundLogEmit emit)
{
    for (unsigned sent = 0; sent < 8 && snapshot.offset < snapshot.end; ++sent) {
        uint8_t reply[97] = {0};
        const uint32_t remaining = snapshot.end - snapshot.offset;
        const uint8_t size = (uint8_t)(remaining < DATA_BYTES ? remaining : DATA_BYTES);
        wire_u32(reply, snapshot.offset);
        reply[6] = size;
        memcpy(reply + 7, snapshot.bytes + snapshot.offset, size);
        emit(GROUND_LOG_DATA, 134, reply, sizeof(reply));
        snapshot.offset += size;
    }
}
