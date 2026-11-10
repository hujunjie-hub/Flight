#include "attitude_rate_control/controller_internal.h"

/* Segment lookup and local time. Shared by control_sample_reference() and the
   M5-06 test hooks so the measured local time is the production computation,
   never a duplicated formula. */
Status control_reference_locate_segment(const MotionReference *ref, uint64_t time,
                                        const MotionReferencePoint **a,
                                        const MotionReferencePoint **b,
                                        float *local_seconds,
                                        uint32_t *step_us_out) {
  uint64_t offset;
  uint32_t step_us, horizon_us;
  unsigned i;
  if (!ref || !a || !b || !local_seconds || !step_us_out ||
      ref->point_count != PRODUCT_PREVIEW_COUNT ||
      time < ref->header.timestamp_sample_us || ref->anchor.offset_us)
    return STATUS_INVALID_ARGUMENT;
  offset = time - ref->header.timestamp_sample_us;
  step_us = ref->points[0].offset_us;
  if (step_us != PRODUCT_PREVIEW_STEP_US && step_us != PRODUCT_ROUTE_PREVIEW_STEP_US)
    return STATUS_INVALID_ARGUMENT;
  horizon_us = step_us * ref->point_count;
  if (offset > horizon_us)
    return STATUS_STALE_DATA;
  for (i = 0; i < ref->point_count; ++i)
    if (ref->points[i].offset_us != (i + 1) * step_us)
      return STATUS_INVALID_ARGUMENT;
  i = offset == horizon_us ? ref->point_count - 1u : (unsigned)(offset / step_us);
  *a = i ? &ref->points[i - 1] : &ref->anchor;
  *b = &ref->points[i];
  if (((*a)->point.valid_mask & 31u) != 31u ||
      ((*b)->point.valid_mask & 31u) != 31u ||
      ((*a)->valid_mask & 31u) != 31u || ((*b)->valid_mask & 31u) != 31u)
    return STATUS_INVALID_ARGUMENT;
  if (!finite_array((*a)->point.position_ned_m, 11) ||
      !finite_array((*b)->point.position_ned_m, 11))
    return STATUS_NUMERIC_ERROR;
  *local_seconds = (float)(offset - (*a)->offset_us) * 1e-6f;
  *step_us_out = step_us;
  return STATUS_OK;
}

Status control_sample_reference(const MotionReference *ref, uint64_t time,
                    ProductMotionPoint *out) {
  const MotionReferencePoint *a, *b;
  uint32_t step_us;
  unsigned j;
  float s, h, c[6];
  Status status;
  if (!ref || !out)
    return STATUS_INVALID_ARGUMENT;
  status = control_reference_locate_segment(ref, time, &a, &b, &s, &step_us);
  if (status != STATUS_OK)
    return status;
  h = step_us * 1e-6f;
  memset(out, 0, sizeof(*out));
  out->valid_mask = 31;
  for (j = 0; j < 3; ++j) {
    /* Quintic Hermite in actual seconds, matches p/v/a at both endpoints. */
    float p0 = a->point.position_ned_m[j], v0 = a->point.velocity_ned_mps[j],
          a0 = a->point.acceleration_ned_mps2[j];
    float d = (b->point.position_ned_m[j] - p0) - h * v0 - .5f * h * h * a0;
    float v = b->point.velocity_ned_mps[j] - v0 - h * a0,
          ac = b->point.acceleration_ned_mps2[j] - a0;
    c[0] = p0;
    c[1] = v0;
    c[2] = .5f * a0;
    c[3] = 10 * d / (h * h * h) - 4 * v / (h * h) + ac / (2 * h);
    c[4] = -15 * d / (h * h * h * h) + 7 * v / (h * h * h) - ac / (h * h);
    c[5] = 6 * d / (h * h * h * h * h) - 3 * v / (h * h * h * h) +
           ac / (2 * h * h * h);
    out->position_ned_m[j] =
        ((((c[5] * s + c[4]) * s + c[3]) * s + c[2]) * s + c[1]) * s + c[0];
    out->velocity_ned_mps[j] =
        (((5 * c[5] * s + 4 * c[4]) * s + 3 * c[3]) * s + 2 * c[2]) * s + c[1];
    out->acceleration_ned_mps2[j] =
        ((20 * c[5] * s + 12 * c[4]) * s + 6 * c[3]) * s + 2 * c[2];
  }
  {
    float dy = remainderf(b->point.yaw_rad - a->point.yaw_rad, 6.28318530718f),
          z = s / h;
    float y0 = a->point.yaw_rad, w0 = a->point.yaw_rate_radps,
          w1 = b->point.yaw_rate_radps;
    out->yaw_rad = y0 + (3 * z * z - 2 * z * z * z) * dy +
                   (z - 2 * z * z + z * z * z) * h * w0 +
                   (-z * z + z * z * z) * h * w1;
    out->yaw_rate_radps = (6 * z - 6 * z * z) * dy / h +
                          (1 - 4 * z + 3 * z * z) * w0 +
                          (-2 * z + 3 * z * z) * w1;
  }
  return finite_array(out->position_ned_m, 11) ? STATUS_OK
                                                  : STATUS_NUMERIC_ERROR;
}
