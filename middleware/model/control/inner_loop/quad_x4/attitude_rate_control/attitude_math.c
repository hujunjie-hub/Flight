#include "attitude_rate_control/controller_internal.h"

static float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static int unit3(float a[3]) {
  float n = sqrtf(dot3(a, a));
  unsigned i;
  if (!isfinite(n) || n < 1e-9f)
    return 0;
  for (i = 0; i < 3; ++i)
    a[i] /= n;
  return 1;
}
static void cross3(const float a[3], const float b[3], float c[3]) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}
int attitude_quat_matrix(const float q[4], float r[9]) {
  float w, x, y, z, n;
  if (!finite_array(q, 4))
    return 0;
  n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (fabsf(n - 1) > 1e-3f)
    return 0;
  n = 1 / sqrtf(n);
  w = q[0] * n;
  x = q[1] * n;
  y = q[2] * n;
  z = q[3] * n;
  r[0] = 1 - 2 * (y * y + z * z);
  r[1] = 2 * (x * y - w * z);
  r[2] = 2 * (x * z + w * y);
  r[3] = 2 * (x * y + w * z);
  r[4] = 1 - 2 * (x * x + z * z);
  r[5] = 2 * (y * z - w * x);
  r[6] = 2 * (x * z - w * y);
  r[7] = 2 * (y * z + w * x);
  r[8] = 1 - 2 * (x * x + y * y);
  return 1;
}
void attitude_relative(const float a[9], const float b[9], float out[9]) {
  int i, j, k;
  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j) {
      float s = 0;
      for (k = 0; k < 3; ++k)
        s += a[3 * k + i] * b[3 * k + j];
      out[3 * i + j] = s;
    }
}
int attitude_so3_log(const float r[9], float axis[3], float out[3]) {
  float v[3] = {r[7] - r[5], r[2] - r[6], r[3] - r[1]};
  float sine = .5f * sqrtf(dot3(v, v));
  float theta = atan2f(sine, clip((r[0] + r[4] + r[8] - 1) * .5f, -1, 1));
  int i, k = 0;
  if (!isfinite(theta))
    return 0;
  if (theta < 1e-6f) {
    for (i = 0; i < 3; ++i)
      out[i] = (.5f + theta * theta / 12) * v[i];
    return 1;
  }
  if (3.14159265358979f - theta < 1e-4f) {
    if (r[4] > r[0])
      k = 1;
    if (r[8] > r[4 * k])
      k = 2;
    out[k] = sqrtf(fmaxf(0, (r[4 * k] + 1) * .5f));
    if (out[k] < 1e-9f)
      return 0;
    for (i = 0; i < 3; ++i)
      if (i != k)
        out[i] = (r[3 * k + i] + r[3 * i + k]) / (4 * out[k]);
    if (!unit3(out))
      return 0;
    /* Use skew sign just outside the singularity, continuity at exact pi. */
    {
      float s = dot3(out, v);
      if (fabsf(s) < 1e-7f)
        s = dot3(out, axis);
      if (s < 0)
        for (i = 0; i < 3; ++i)
          out[i] = -out[i];
    }
  } else {
    float d = 1 / (2 * sine);
    for (i = 0; i < 3; ++i)
      out[i] = d * v[i];
  }
  for (i = 0; i < 3; ++i) {
    axis[i] = out[i];
    out[i] *= theta;
  }
  return finite_array(out, 3);
}
int attitude_from_thrust(const float a[3], float yaw, float b2_old[3], float r[9],
                       float *t) {
  float b3[3] = {-a[0], -a[1], CONTROL_GRAVITY_MPS2 - a[2]}, b1c[3] = {cosf(yaw), sinf(yaw), 0},
        b2[3], b1[3];
  int i;
  *t = m6.config.payload.mass_kg * sqrtf(dot3(b3, b3));
  if (!unit3(b3))
    return 0;
  cross3(b3, b1c, b2);
  if (!unit3(b2)) {
    float d = dot3(b2_old, b3);
    for (i = 0; i < 3; ++i)
      b2[i] = b2_old[i] - d * b3[i];
    if (!unit3(b2)) {
      float axis[3] = {0, 1, 0};
      if (fabsf(b3[1]) > .9f) {
        axis[0] = 1;
        axis[1] = 0;
      }
      cross3(b3, axis, b2);
      if (!unit3(b2))
        return 0;
    }
  }
  cross3(b2, b3, b1);
  for (i = 0; i < 3; ++i) {
    r[3 * i] = b1[i];
    r[3 * i + 1] = b2[i];
    r[3 * i + 2] = b3[i];
    b2_old[i] = b2[i];
  }
  return isfinite(*t);
}
