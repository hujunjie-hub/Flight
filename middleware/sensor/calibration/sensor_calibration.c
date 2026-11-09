#include "calibration/sensor_calibration.h"

void sensor_apply_calibration(const float bias[3], const float scale[3], const float misalignment[3][3],
                              const float raw[3], float out[3])
{
    float corrected[3];
    float rotated[3];
    int row;
    int col;

    for (row = 0; row < 3; ++row) {
        corrected[row] = (raw[row] - bias[row]) * scale[row];
    }
    for (row = 0; row < 3; ++row) {
        rotated[row] = 0.0f;
        for (col = 0; col < 3; ++col) {
            rotated[row] += misalignment[row][col] * corrected[col];
        }
    }
    for (row = 0; row < 3; ++row) {
        out[row] = rotated[row];
    }
}

void sensor_rotate_vector(const float rotation[3][3], const float in[3], float out[3])
{
    int row;
    int col;
    for (row = 0; row < 3; ++row) {
        float sum = 0.0f;
        for (col = 0; col < 3; ++col) {
            sum += rotation[row][col] * in[col];
        }
        out[row] = sum;
    }
}
