#ifndef MAVLINK_ACTION_H
#define MAVLINK_ACTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MAVLINK_ACTION_UNKNOWN,
    MAVLINK_ACTION_INVALID,
    MAVLINK_ACTION_VALID
} MavlinkActionResult;

typedef struct {
    uint16_t product_command;
    float altitude_m;
} MavlinkAction;

/* Decode a CRC-checked, zero-filled COMMAND_LONG or COMMAND_INT payload.
 * Transport authorization and FMS prerequisites belong to the caller. */
MavlinkActionResult mavlink_action_decode(uint16_t command, bool long_format,
                                        const uint8_t *payload, MavlinkAction *action);

uint8_t mavlink_calibration_kind(const uint8_t *payload);

#endif
