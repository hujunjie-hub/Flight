#ifndef MAVLINK_FRAME_H
#define MAVLINK_FRAME_H
#include <stddef.h>
#include <stdint.h>
#include "mavlink/fcu_mavlink_layout.h"

#define MAVLINK_V2_MAGIC       0xfdu
#define MAVLINK_V2_HEADER_SIZE 10u
#define MAVLINK_CRC_SIZE       2u
#define MAVLINK_ALTITUDE_ID    141u
#define MAVLINK_ALTITUDE_CRC   47u
#define MAVLINK_ALTITUDE_LEN   32u

static inline uint16_t mavlink_crc_accumulate(uint16_t crc, uint8_t byte)
{
    uint8_t value = (uint8_t)(byte ^ (uint8_t)crc);
    value ^= (uint8_t)(value << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)value << 8) ^ ((uint16_t)value << 3) ^ (value >> 4));
}

size_t wire_encode(uint32_t message_id,uint8_t crc_extra,const uint8_t *payload,
                       size_t length,uint8_t sequence,uint8_t *out,size_t capacity);
/* Explicit little-endian packing; public C struct layout is never a wire ABI. */
void wire_u16(uint8_t *p,uint16_t v);
void wire_u32(uint8_t *p,uint32_t v);
void wire_u64(uint8_t *p,uint64_t v);
void wire_float(uint8_t *p,float v);
float wire_read_float(const uint8_t *p);
void wire_altitude(uint8_t p[MAVLINK_ALTITUDE_LEN], uint64_t sample_us,
                   float amsl_m, float local_m, float relative_m);
#endif
