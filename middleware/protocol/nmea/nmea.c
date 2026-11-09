#include "nmea/nmea.h"

#include <stdint.h>

static int hex_value(char digit)
{
    if (digit >= '0' && digit <= '9')
        return digit - '0';
    if (digit >= 'A' && digit <= 'F')
        return digit - 'A' + 10;
    if (digit >= 'a' && digit <= 'f')
        return digit - 'a' + 10;
    return -1;
}

bool nmea_parse(const char *line, size_t length, NmeaSentence *out)
{
    uint8_t checksum = 0u;
    if (!line || !out || length < 5u || length > 82u || line[0] != '$')
        return false;
    if (line[length - 2u] == '\r' && line[length - 1u] == '\n')
        length -= 2u;
    if (length < 5u || line[length - 3u] != '*')
        return false;

    const int high = hex_value(line[length - 2u]);
    const int low = hex_value(line[length - 1u]);
    if (high < 0 || low < 0)
        return false;
    for (size_t i = 1u; i < length - 3u; ++i)
    {
        const uint8_t byte = (uint8_t)line[i];
        if (byte < ' ' || byte > '~' || byte == '*' || byte == '$')
            return false;
        checksum ^= byte;
    }
    if (checksum != (uint8_t)((high << 4) | low))
        return false;

    out->body = line + 1u;
    out->body_length = length - 4u;
    return true;
}
