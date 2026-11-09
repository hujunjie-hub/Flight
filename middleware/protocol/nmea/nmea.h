#ifndef NMEA_H
#define NMEA_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    const char *body;
    size_t body_length;
} NmeaSentence;

/* Validate a complete checksummed GNSS sentence. The view borrows the input
 * and covers the address and comma-separated fields between '$' and '*'.
 * A trailing CRLF is accepted but is outside the view. */
bool nmea_parse(const char *line, size_t length, NmeaSentence *out);

#endif
