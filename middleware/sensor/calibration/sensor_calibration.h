#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

void sensor_apply_calibration(const float bias[3], const float scale[3],
                              const float misalignment[3][3], const float raw[3], float out[3]);
void sensor_rotate_vector(const float rotation[3][3], const float in[3], float out[3]);

#endif
