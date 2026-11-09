/* Synthetic covariance/noise, for Host/SIL only; not sensor identification. */
#include "ins_model.h"
#include <cstring>
extern "C" void ins_model_sim_config(ProductEstimatorConfig *out) {
  if (!out)
    return;
  std::memset(out, 0, sizeof(*out));
  out->profile_hash = PRODUCT_PROFILE_ID;
  out->parameter_set_id = 1;
  out->parameter_hash = 0x53494d31;
  out->config_version = 1;
  auto &p = out->payload;
  unsigned k = 0;
  for (int i = 0; i < 22; ++i)
    for (int j = i; j < 22; ++j) {
      double sigma = i < 3    ? 2
                     : i < 6  ? .2
                     : i < 8  ? .01
                     : i == 8 ? .1
                     : i < 12 ? .001
                     : i < 15 ? .01
                     : i < 21 ? .0001
                              : 1;
      p.initial_covariance_upper[k++] = i == j ? sigma * sigma : 0;
    }
  for (int i = 0; i < 3; ++i) {
    p.gyro_noise_density_radps_per_sqrt_hz[i] = .0001;
    p.accel_noise_density_mps2_per_sqrt_hz[i] = .002;
    p.gyro_bias_std_radps[i] = .001;
    p.accel_bias_std_mps2[i] = .01;
    p.gyro_scale_std[i] = p.accel_scale_std[i] = .0001;
    p.gnss_position_floor_m[i] = .1;
    p.gnss_velocity_floor_mps[i] = .05;
  }
  p.bias_correlation_time_s[0] = p.bias_correlation_time_s[1] = 3600;
  p.scale_correlation_time_s[0] = p.scale_correlation_time_s[1] = 3600;
  p.baro_bias_rw_psd_m2ps = .001;
  p.baro_floor_m = .5;
  p.mag_heading_floor_rad = .05;
  p.min_horizontal_field_ut = 10;
  p.magnetic_declination_variance_rad2 = .0001;
  p.initial_latitude_deg = 30;
  p.initial_longitude_deg = 114;
  p.initial_ellipsoid_height_m = 30;
  p.initial_origin_valid_mask = 3;
  p.local_radius_limit_m = 10000;
  p.aiding_mask = PRODUCT_AIDING_GNSS_POSITION | PRODUCT_AIDING_GNSS_VELOCITY |
                  PRODUCT_AIDING_BARO | PRODUCT_AIDING_MAG;
  p.calibration_version = 1;
}
