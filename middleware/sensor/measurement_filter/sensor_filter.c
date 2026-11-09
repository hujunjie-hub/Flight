#include "measurement_filter/sensor_filter.h"

#include <math.h>

#define SENSOR_DEG_TO_RAD (3.14159265358979323846 / 180.0)

static bool all_finite3(const float v[3])
{
    return isfinite((double)v[0]) && isfinite((double)v[1]) && isfinite((double)v[2]);
}

bool sensor_filter_barometer(const BarometerReading *reading)
{
    return isfinite(reading->pressure_pa) && reading->pressure_pa >= 30000.0 &&
           reading->pressure_pa <= 125000.0 && isfinite(reading->temperature_c) &&
           reading->temperature_c >= -40.0 && reading->temperature_c <= 85.0;
}

bool sensor_filter_magnetometer(const float xyz_ut[3], float temperature_c)
{
    return isfinite(temperature_c) && all_finite3(xyz_ut);
}

static double horizontal_distance_m(double lat0_deg, double lon0_deg, double lat1_deg, double lon1_deg)
{
    double mean_lat = 0.5 * (lat0_deg + lat1_deg) * SENSOR_DEG_TO_RAD;
    double dlat = (lat1_deg - lat0_deg) * SENSOR_DEG_TO_RAD;
    double dlon = (lon1_deg - lon0_deg) * SENSOR_DEG_TO_RAD * cos(mean_lat);
    return sqrt(dlat * dlat + dlon * dlon) * 6378137.0;
}

sensor_status_t sensor_filter_gnss(const sensor_gnss_config_t *config,
                                  const sensor_gnss_state_t *state, const sensor_gnss_pvt_t *pvt,
                                  uint64_t sample_time_us, uint64_t arrival_us)
{
    if (!isfinite(pvt->latitude_deg) || !isfinite(pvt->longitude_deg) || !isfinite(pvt->altitude_m)) {
        return SENSOR_ERR_RANGE;
    }
    if ((pvt->latitude_deg < -90.0) || (pvt->latitude_deg > 90.0) || (pvt->longitude_deg < -180.0) ||
        (pvt->longitude_deg > 180.0)) {
        return SENSOR_ERR_RANGE;
    }
    if (fabs(pvt->altitude_m) > config->max_altitude_m) {
        return SENSOR_ERR_RANGE;
    }
    if (!all_finite3(pvt->velocity_ned_mps)) {
        return SENSOR_ERR_RANGE;
    }

    /* Age is measured with the mapped measurement epoch, not the arrival
     * order: an observation may arrive late but must still be recent. */
    if (arrival_us < sample_time_us) {
        return SENSOR_ERR_STALE;
    }
    if ((arrival_us - sample_time_us) > config->stale_age_us) {
        return SENSOR_ERR_STALE;
    }

    /* Position jump rejection against the previous accepted fix. */
    if (state->has_previous) {
        if (sample_time_us <= state->sample_time_us) {
            return SENSOR_ERR_STALE;
        }
        double dt_s = (double)(sample_time_us - state->sample_time_us) * 1.0e-6;
        double distance_m = horizontal_distance_m(state->latitude_deg, state->longitude_deg,
                                                  pvt->latitude_deg, pvt->longitude_deg);
        /* Allow 25 m of fix-to-fix noise in the speed bound. */
        if (distance_m > config->max_speed_mps * dt_s + 25.0) {
            return SENSOR_ERR_JUMP;
        }
    }

    return SENSOR_OK;
}
