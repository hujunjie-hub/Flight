#ifndef UNICORE_ASCII_H
#define UNICORE_ASCII_H
#include <stdint.h>

#define UNICORE_PVT_POSITION_VALID (1u << 0)
#define UNICORE_PVT_VELOCITY_VALID (1u << 1)
#define UNICORE_PVT_POSITION_COV_VALID (1u << 2)
#define UNICORE_PVT_VELOCITY_COV_VALID (1u << 3)

typedef struct {
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double altitude_msl_m;
    float velocity_ned_mps[3];
    float position_variance_m2[3];
    float velocity_variance_m2ps2[3];
    uint8_t fix_type; /* GNSS_FIX_* */
    uint8_t satellites;
    uint32_t valid_flags; /* UNICORE_PVT_* */
} UnicorePvt;

#define UNICORE_ASCII_LINE_CAPACITY 768u

enum { UNICORE_ASCII_BESTNAV = 1, UNICORE_ASCII_RECTIME = 2 };
typedef struct {
  uint16_t length;
  uint8_t collecting;
  char line[UNICORE_ASCII_LINE_CAPACITY];
  uint32_t valid_frames, crc_errors, malformed_frames, overflows;
} UnicoreAsciiParser;
typedef struct {
  uint8_t kind;
  uint64_t gps_time_ns, utc_time_ns;
  uint32_t receiver_accuracy_ns;
  uint16_t output_delay_ms;
  UnicorePvt pvt;
} UnicoreAsciiMessage;
/* 0 incomplete/other log, 1 decoded, negative rejected. No allocation. */
int unicore_ascii_feed(UnicoreAsciiParser *parser,uint8_t byte,UnicoreAsciiMessage *out);
#endif
