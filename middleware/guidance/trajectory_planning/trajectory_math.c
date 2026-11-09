/*
 * M5 math: quintic boundary-value solve/sampling, 5th-order Hermite evaluation
 * (the same interpolation a consumer uses on anchor+24 points), yaw/quaternion
 * helpers and geofence geometry.
 */
#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

float guidance_wrap_pi(float angle)
{
  return remainderf(angle, 6.28318530717959f);
}

void guidance_quintic_solve(float p0, float v0, float a0, float pf, float tf, GuidanceQuintic *q)
{
  guidance_quintic_boundary(p0, v0, a0, pf, 0, 0, tf, q);
}

void guidance_quintic_boundary(float p0, float v0, float a0, float pf, float vf, float af,
                          float tf, GuidanceQuintic *q)
{
  double t = tf, d, e, f, t2, t3;
  unsigned i;
  if (!isfinite(t) || t <= 0) {
    for (i = 0; i < 6; ++i) q->c[i] = NAN;
    return;
  }
  t2 = t*t; t3 = t2*t;
  d = (double)pf-p0-(double)v0*t-0.5*(double)a0*t2;
  e = (double)vf-v0-(double)a0*t;
  f = (double)af-a0;
  /* Normalized boundary solve. An absolute determinant threshold incorrectly
   * declares every 20 ms consumer interval singular (det scales as T^9). */
  q->c[0] = p0; q->c[1] = v0; q->c[2] = 0.5f*a0;
  q->c[3] = (10*d-4*e*t+0.5*f*t2)/t3;
  q->c[4] = (-15*d+7*e*t-f*t2)/(t3*t);
  q->c[5] = (6*d-3*e*t+0.5*f*t2)/(t3*t2);
}

float guidance_quintic_eval(const GuidanceQuintic *q, float t, uint8_t derivative)
{
  if (derivative == 0u) {
    return ((((q->c[5] * t + q->c[4]) * t + q->c[3]) * t + q->c[2]) * t + q->c[1]) * t +
           q->c[0];
  }
  if (derivative == 1u) {
    return (((5.0f * q->c[5] * t + 4.0f * q->c[4]) * t + 3.0f * q->c[3]) * t +
            2.0f * q->c[2]) * t + q->c[1];
  }
  return (((20.0f * q->c[5] * t + 12.0f * q->c[4]) * t + 6.0f * q->c[3]) * t +
          2.0f * q->c[2]);
}

float guidance_quintic_jerk(const GuidanceQuintic *q, float t)
{
  return (60.0f * q->c[5] * t + 24.0f * q->c[4]) * t + 6.0f * q->c[3];
}

void guidance_quintic_hermite_eval(float p0, float v0, float a0, float p1, float v1, float a1,
                             float h, float s, float *p, float *v)
{
  float h00 = 1.0f - 10.0f * s * s * s + 15.0f * s * s * s * s - 6.0f * s * s * s * s * s;
  float h10 = s - 6.0f * s * s * s + 8.0f * s * s * s * s - 3.0f * s * s * s * s * s;
  float h20 = 0.5f * s * s - 1.5f * s * s * s + 1.5f * s * s * s * s -
              0.5f * s * s * s * s * s;
  float h01 = 10.0f * s * s * s - 15.0f * s * s * s * s + 6.0f * s * s * s * s * s;
  float h11 = -4.0f * s * s * s + 7.0f * s * s * s * s - 3.0f * s * s * s * s * s;
  float h21 = 0.5f * s * s * s - s * s * s * s + 0.5f * s * s * s * s * s;
  float dh00 = -30.0f * s * s + 60.0f * s * s * s - 30.0f * s * s * s * s;
  float dh10 = 1.0f - 18.0f * s * s + 32.0f * s * s * s - 15.0f * s * s * s * s;
  float dh20 = s - 4.5f * s * s + 6.0f * s * s * s - 2.5f * s * s * s * s;
  float dh01 = 30.0f * s * s - 60.0f * s * s * s + 30.0f * s * s * s * s;
  float dh11 = -12.0f * s * s + 28.0f * s * s * s - 15.0f * s * s * s * s;
  float dh21 = 1.5f * s * s - 4.0f * s * s * s + 2.5f * s * s * s * s;

  if (p != NULL) {
    *p = h00 * p0 + h10 * h * v0 + h20 * h * h * a0 + h01 * p1 + h11 * h * v1 +
         h21 * h * h * a1;
  }
  if (v != NULL) {
    *v = (dh00 * p0 + dh10 * h * v0 + dh20 * h * h * a0 + dh01 * p1 + dh11 * h * v1 +
          dh21 * h * h * a1) /
         h;
  }
}

float guidance_yaw_from_quat(const float q[4])
{
  float w = q[0], x = q[1], y = q[2], z = q[3];
  return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void guidance_quat_from_rpy(float roll, float pitch, float yaw, float q[4])
{
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

uint8_t guidance_geofence_point_ok(const ProductGeofenceSnapshot *fences, uint16_t count,
                             const float p_ned[3], float home_down_m)
{
  uint16_t i;
  if (count > PRODUCT_MAX_GEOFENCE_POLYGONS || (count && fences == NULL)) return 0;
  for (i = 0; i < count; ++i)
    if (!fence_geometry_valid(&fences[i]) ||
        !fence_point_valid(&fences[i], p_ned, home_down_m)) return 0;
  return 1;
}
