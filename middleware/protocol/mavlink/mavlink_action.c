#include "calibration/sensor_calibration_service.h"
#include "mavlink/mavlink_action.h"
#include "mavlink/mavlink_frame.h"
#include "product_contract/product_contract.h"
#include <math.h>

MavlinkActionResult mavlink_action_decode(uint16_t command, bool long_format,
                                        const uint8_t *payload, MavlinkAction *action)
{
    const float param1 = wire_read_float(payload);
    *action = (MavlinkAction){0};
    switch (command)
    {
    case 400: { /* MAV_CMD_COMPONENT_ARM_DISARM */
        const float force = wire_read_float(payload + 4);
        if ((param1 != 0.0f && param1 != 1.0f) || (force != 0.0f && !isnan(force)))
            return MAVLINK_ACTION_INVALID;
        action->product_command = param1 == 1.0f ? PRODUCT_CMD_ARM : PRODUCT_CMD_DISARM;
        break;
    }
    case 22: /* MAV_CMD_NAV_TAKEOFF */
        action->altitude_m = wire_read_float(payload + 24);
        if (!isfinite(action->altitude_m) || action->altitude_m <= 0.0f ||
            (!long_format && payload[32] != 3 && payload[32] != 6))
            return MAVLINK_ACTION_INVALID;
        action->product_command = PRODUCT_CMD_TAKEOFF;
        break;
    case 21: /* MAV_CMD_NAV_LAND */
        action->product_command = PRODUCT_CMD_LAND;
        break;
    case 176: /* MAVSDK legacy RTL mode, custom main AUTO (4), submode RTL (5). */
        if ((param1 != 1.0f && param1 != 129.0f) ||
            wire_read_float(payload + 4) != 4.0f || wire_read_float(payload + 8) != 5.0f)
            return MAVLINK_ACTION_UNKNOWN;
        action->product_command = PRODUCT_CMD_RETURN;
        break;
    case 262: /* MAV_CMD_DO_SET_STANDARD_MODE (MAVSDK return_to_launch) */
        if (param1 != 5.0f) /* MAV_STANDARD_MODE_SAFE_RECOVERY */
            return MAVLINK_ACTION_UNKNOWN;
        /* fall through */
    case 20: /* MAV_CMD_NAV_RETURN_TO_LAUNCH */
        action->product_command = PRODUCT_CMD_RETURN;
        break;
    default:
        return MAVLINK_ACTION_UNKNOWN;
    }
    return MAVLINK_ACTION_VALID;
}

uint8_t mavlink_calibration_kind(const uint8_t *payload)
{
    static const uint8_t kinds[] = {
        SENSOR_CAL_GYRO, SENSOR_CAL_MAG, SENSOR_CAL_BARO, SENSOR_CAL_RC, SENSOR_CAL_ACCEL
    };
    uint8_t kind = SENSOR_CAL_NONE;
    for (unsigned i = 0; i < 7; ++i) {
        const float value = wire_read_float(payload + i * 4u);
        if (value == 0.0f)
            continue;
        if (kind != SENSOR_CAL_NONE || i >= sizeof(kinds))
            return SENSOR_CAL_NONE;
        if (i == 4 && value == 2.0f)
            kind = SENSOR_CAL_LEVEL;
        else if (value == 1.0f)
            kind = kinds[i];
        else
            return SENSOR_CAL_NONE;
    }
    return kind;
}
