/*
 * M5-06 independent continuous trajectory safety validator.
 *
 * This file deliberately does not call the M5-03 A* edge checker, M5-05
 * smoother, or the legacy Guidance fence validator. It reconstructs the
 * consumer's quintic Hermite curve from each anchor/preview pair, converts
 * it to Bernstein form, and proves swept-volume clearance with bounded DFS.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)

#include <math.h>
#include <string.h>
#include "trajectory_planning/m5_trajectory_safety.h"
#include "messages/message_ids.h"
#include "geofence_geometry/geofence_geometry.h"

#define M5_SAFETY_POINT_MASK 0x1Fu
#define M5_SAFETY_NAV_MASK (NAV_VALID_HORIZONTAL_POSITION | \
                            NAV_VALID_VERTICAL_POSITION | \
                            NAV_VALID_HORIZONTAL_VELOCITY | \
                            NAV_VALID_VERTICAL_VELOCITY | \
                            NAV_VALID_ACCELERATION)
#define M5_SAFETY_ANCHOR_POSITION_TOL_M 0.05f
#define M5_SAFETY_ANCHOR_VELOCITY_TOL_MPS 0.10f
#define M5_SAFETY_ANCHOR_ACCEL_TOL_MPS2 0.25f
#define M5_SAFETY_CELL_HALF_M (0.5f * M5_MAP_CELL_SIZE_M)
#define M5_SAFETY_GEOMETRY_EPSILON_M 0.00001f
 
static const double choose[6][6] = {
  {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {1.0, 1.0, 0.0, 0.0, 0.0, 0.0},
  {1.0, 2.0, 1.0, 0.0, 0.0, 0.0},
  {1.0, 3.0, 3.0, 1.0, 0.0, 0.0},
  {1.0, 4.0, 6.0, 4.0, 1.0, 0.0},
  {1.0, 5.0, 10.0, 10.0, 5.0, 1.0}
};

typedef struct {
  double low;
  double high;
  uint8_t valid;
} M5SafetyInterval;

typedef struct {
  double position[3][6];
  double position_low[3][6];
  double position_high[3][6];
} M5SafetyBezier;

typedef struct {
  float position[3];
  float velocity[3];
  float position_low[3];
  float position_high[3];
  float speed_bound;
  float acceleration_bound;
  float velocity_axis_bound[3];
  float acceleration_axis_bound[3];
  float velocity_mid_norm;
  float position_rounding_error_m;
  float velocity_rounding_error_mps;
  float half_duration_s;
} M5SafetyCurveBounds;

typedef struct {
  uint8_t unresolved;
  uint8_t unresolved_status;
  uint16_t reserved;
} M5SafetyCheck;

static uint8_t finite3(const float v[3])
{
  return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]));
}


static M5SafetyInterval interval_invalid(void)
{
  M5SafetyInterval value = {0.0, 0.0, 0u};
  return value;
}

static M5SafetyInterval interval_bounds(double low, double high)
{
  M5SafetyInterval value;
  value.low = low;
  value.high = high;
  value.valid = (uint8_t)(isfinite(low) && isfinite(high) && low <= high);
  return value.valid != 0u ? value : interval_invalid();
}

static M5SafetyInterval interval_exact(double value)
{
  return interval_bounds(value, value);
}

static M5SafetyInterval interval_add(
    M5SafetyInterval a, M5SafetyInterval b)
{
  if (a.valid == 0u || b.valid == 0u) return interval_invalid();
  return interval_bounds(
      nextafter(a.low + b.low, -INFINITY),
      nextafter(a.high + b.high, INFINITY));
}

static M5SafetyInterval interval_sub(
    M5SafetyInterval a, M5SafetyInterval b)
{
  if (a.valid == 0u || b.valid == 0u) return interval_invalid();
  return interval_bounds(
      nextafter(a.low - b.high, -INFINITY),
      nextafter(a.high - b.low, INFINITY));
}

static M5SafetyInterval interval_mul(
    M5SafetyInterval a, M5SafetyInterval b)
{
  double products[4], low, high;
  unsigned i;
  if (a.valid == 0u || b.valid == 0u) return interval_invalid();
  products[0] = a.low * b.low;
  products[1] = a.low * b.high;
  products[2] = a.high * b.low;
  products[3] = a.high * b.high;
  for (i = 0u; i < 4u; ++i)
    if (!isfinite(products[i])) return interval_invalid();
  low = products[0];
  high = products[0];
  for (i = 1u; i < 4u; ++i) {
    low = fmin(low, products[i]);
    high = fmax(high, products[i]);
  }
  return interval_bounds(nextafter(low, -INFINITY),
                         nextafter(high, INFINITY));
}

static M5SafetyInterval interval_div(
    M5SafetyInterval a, M5SafetyInterval b)
{
  double quotients[4], low, high;
  unsigned i;
  if (a.valid == 0u || b.valid == 0u ||
      (b.low <= 0.0 && b.high >= 0.0)) return interval_invalid();
  quotients[0] = a.low / b.low;
  quotients[1] = a.low / b.high;
  quotients[2] = a.high / b.low;
  quotients[3] = a.high / b.high;
  for (i = 0u; i < 4u; ++i)
    if (!isfinite(quotients[i])) return interval_invalid();
  low = quotients[0];
  high = quotients[0];
  for (i = 1u; i < 4u; ++i) {
    low = fmin(low, quotients[i]);
    high = fmax(high, quotients[i]);
  }
  return interval_bounds(nextafter(low, -INFINITY),
                         nextafter(high, INFINITY));
}

static M5SafetyInterval interval_scale(M5SafetyInterval a, double scale)
{
  return interval_mul(a, interval_exact(scale));
}

static M5SafetyInterval interval_evaluate_bernstein(
    const M5SafetyInterval *control, unsigned degree, double u)
{
  M5SafetyInterval work[6];
  M5SafetyInterval u_i = interval_exact(u);
  M5SafetyInterval one_minus_u = interval_sub(interval_exact(1.0), u_i);
  unsigned i, level;
  if (control == NULL || degree > 5u || !isfinite(u) ||
      u < 0.0 || u > 1.0 || u_i.valid == 0u ||
      one_minus_u.valid == 0u) return interval_invalid();
  for (i = 0u; i <= degree; ++i) {
    if (control[i].valid == 0u) return interval_invalid();
    work[i] = control[i];
  }
  for (level = 1u; level <= degree; ++level) {
    for (i = 0u; i <= degree - level; ++i) {
      work[i] = interval_add(
          interval_mul(one_minus_u, work[i]),
          interval_mul(u_i, work[i + 1u]));
      if (work[i].valid == 0u) return interval_invalid();
    }
  }
  return work[0];
}

static double interval_abs_max(M5SafetyInterval value)
{
  if (value.valid == 0u) return INFINITY;
  return fmax(fabs(value.low), fabs(value.high));
}

static double sum_square_upper(double sum, double value)
{
  double square;
  if (value == 0.0) return sum;
  square = nextafter(value * value, INFINITY);
  return nextafter(sum + square, INFINITY);
}

static float round_up_float(double value)
{
  float rounded = (float)value;
  if (!isfinite(rounded)) return rounded;
  if ((double)rounded < value) rounded = nextafterf(rounded, INFINITY);
  return rounded;
}

static float round_down_float(double value)
{
  float rounded = (float)value;
  if (!isfinite(rounded)) return rounded;
  if ((double)rounded > value) rounded = nextafterf(rounded, -INFINITY);
  return rounded;
}

static float norm3_upper(const float v[3])
{
  const double x = v[0], y = v[1], z = v[2];
  return round_up_float(sqrt(x * x + y * y + z * z));
}

static float norm3_lower(const double x, const double y, const double z)
{
  double value = sqrt(x * x + y * y + z * z);
  float rounded = round_down_float(value);
  return rounded > 0.0f ? nextafterf(rounded, -INFINITY) : 0.0f;
}

static uint64_t earlier_u64(uint64_t a, uint64_t b)
{
  return a < b ? a : b;
}

static uint8_t unresolved_priority(uint8_t status)
{
  switch ((M5TrajectorySafetyStatus)status) {
    case M5_SAFETY_INVALID: return 7u;
    case M5_SAFETY_STALE: return 6u;
    case M5_SAFETY_UNKNOWN: return 5u;
    case M5_SAFETY_STATIC_COLLISION: return 4u;
    case M5_SAFETY_DYNAMIC_COLLISION: return 3u;
    case M5_SAFETY_GEOFENCE_VIOLATION: return 2u;
    default: return 1u;
  }
}

static void mark_unresolved(M5SafetyCheck *check, M5TrajectorySafetyStatus status)
{
  if (check == NULL) return;
  if (check->unresolved == 0u ||
      unresolved_priority((uint8_t)status) >
          unresolved_priority(check->unresolved_status)) {
    check->unresolved_status = (uint8_t)status;
  }
  check->unresolved = 1u;
}

static M5TrajectorySafetyStatus reject(
    M5TrajectorySafetyResult *result,
    M5TrajectorySafetyStatus status,
    M5TrajectorySafetyReason reason,
    uint32_t offset_us)
{
  result->status = (uint8_t)status;
  result->passed = 0u;
  result->reason_code = (uint16_t)reason;
  result->first_violation_offset_us = offset_us;
  return status;
}

static uint8_t header_valid(
    const ContractHeader *h,
    const ContractHeader *nav,
    uint16_t message_type,
    uint64_t now_us,
    uint64_t minimum_valid_until_us)
{
  if (h == NULL || nav == NULL) return 0u;
  return (uint8_t)(
      h->message_type == message_type &&
      h->profile_id == PRODUCT_PROFILE_ID &&
      h->schema_major == CONTRACT_SCHEMA_MAJOR &&
      h->schema_minor == CONTRACT_SCHEMA_MINOR &&
      h->boot_id != 0u && h->boot_id == nav->boot_id &&
      h->time_epoch != 0u && h->time_epoch == nav->time_epoch &&
      h->frame_id == FRAME_LOCAL_NED &&
      h->clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC &&
      (h->validity_flags & (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) ==
          (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) &&
      h->timestamp_sample_us <= h->timestamp_publish_us &&
      h->timestamp_publish_us <= now_us &&
      h->valid_until_us > now_us &&
      h->valid_until_us >= minimum_valid_until_us);
}

static uint8_t finite_motion_point(const MotionReferencePoint *p)
{
  return (uint8_t)(p != NULL &&
      p->valid_mask == M5_SAFETY_POINT_MASK &&
      p->point.valid_mask == M5_SAFETY_POINT_MASK &&
      finite3(p->point.position_ned_m) &&
      finite3(p->point.velocity_ned_mps) &&
      finite3(p->point.acceleration_ned_mps2) &&
      isfinite(p->point.yaw_rad) &&
      isfinite(p->point.yaw_rate_radps));
}

static uint8_t validate_envelope(const SafetyEnvelope *e)
{
  const ProductSafetyEnvelopePayload *p;
  unsigned axis;
  if (e == NULL) return 0u;
  p = &e->vehicle_extension;
  if ((p->valid_mask & (PRODUCT_ENVELOPE_VALID_VELOCITY |
                        PRODUCT_ENVELOPE_VALID_ACCELERATION)) !=
      (PRODUCT_ENVELOPE_VALID_VELOCITY | PRODUCT_ENVELOPE_VALID_ACCELERATION) ||
      e->safety_envelope_id == 0u ||
      !isfinite(e->max_linear_speed_mps) || e->max_linear_speed_mps <= 0.0f ||
      !isfinite(e->max_linear_accel_mps2) || e->max_linear_accel_mps2 <= 0.0f) {
    return 0u;
  }
  for (axis = 0u; axis < 3u; ++axis) {
    if (!isfinite(p->max_velocity_ned_mps[axis]) ||
        p->max_velocity_ned_mps[axis] <= 0.0f ||
        !isfinite(p->max_acceleration_ned_mps2[axis]) ||
        p->max_acceleration_ned_mps2[axis] <= 0.0f) return 0u;
  }
  return 1u;
}

static uint8_t validate_reference_shape(
    const M5TrajectorySafetyInput *in,
    uint64_t *proof_end_us,
    uint64_t *valid_until_us)
{
  const MotionReference *r = in->candidate_reference;
  const NavigationSnapshot *nav = in->navigation;
  const VehicleDirective *dir = in->directive;
  const SafetyEnvelope *env = in->safety_envelope;
  uint64_t preview_us = (uint64_t)M5_SAFETY_INTERVAL_COUNT * PRODUCT_PREVIEW_STEP_US;
  unsigned i, axis;
  uint32_t required_nav = M5_SAFETY_NAV_MASK;

  if (r == NULL || nav == NULL || dir == NULL || env == NULL ||
      in->map == NULL || in->evaluation_time_us == 0u ||
      in->expected_map_revision == 0u || in->reserved0 != 0u ||
      in->geofence_count > PRODUCT_MAX_GEOFENCE_POLYGONS ||
      (in->geofence_count != 0u && in->geofences == NULL) ||
      in->response_delay_us > M5_SAFETY_MAX_RESPONSE_DELAY_US ||
      !isfinite(in->max_dynamic_obstacle_accel_mps2) ||
      in->max_dynamic_obstacle_accel_mps2 < 0.0f ||
      in->max_dynamic_obstacle_accel_mps2 > M5_SAFETY_MAX_DYNAMIC_ACCEL_MPS2) {
    return 0u;
  }

  if (!header_valid(&nav->header, &nav->header, MSG_NAVIGATION_SNAPSHOT,
                    in->evaluation_time_us, in->evaluation_time_us + 1u) ||
      nav->header.timestamp_sample_us > in->evaluation_time_us ||
      in->evaluation_time_us - nav->header.timestamp_sample_us > 5000u ||
      nav->state_epoch == 0u || nav->origin_id == 0u ||
      (nav->core.valid_mask & required_nav) != required_nav ||
      !finite3(nav->core.position_ned_m) ||
      !finite3(nav->core.velocity_ned_mps) ||
      !finite3(nav->core.acceleration_ned_mps2) ||
      !finite3(nav->core.position_std_m) ||
      !finite3(nav->core.velocity_std_mps)) {
    return 0u;
  }
  for (axis = 0u; axis < 3u; ++axis) {
    if (nav->core.position_std_m[axis] < 0.0f ||
        nav->core.velocity_std_mps[axis] < 0.0f) return 0u;
  }

  if (!header_valid(&r->header, &nav->header, MSG_MOTION_REFERENCE,
                    in->evaluation_time_us, in->evaluation_time_us + 1u) ||
      r->header.source_id != SOURCE_M5 ||
      r->header.timestamp_sample_us != nav->header.timestamp_sample_us ||
      r->reference_id == 0u || r->reference_epoch == 0u ||
      r->state_epoch != nav->state_epoch || r->origin_id != nav->origin_id ||
      r->authority_epoch == 0u || r->directive_epoch == 0u ||
      r->point_count != M5_SAFETY_INTERVAL_COUNT ||
      r->reference_kind != GUIDANCE_REFERENCE_TRAJECTORY ||
      r->reserved != 0u || r->anchor.offset_us != 0u ||
      !finite_motion_point(&r->anchor)) {
    return 0u;
  }

  if (!header_valid(&dir->header, &nav->header, MSG_VEHICLE_DIRECTIVE,
                    in->evaluation_time_us, in->evaluation_time_us + 1u) ||
      dir->directive_expires_at_us <= in->evaluation_time_us ||
      dir->state_epoch != nav->state_epoch ||
      dir->directive_epoch != r->directive_epoch ||
      dir->authority_epoch != r->authority_epoch ||
      (dir->guidance_mode_code != PRODUCT_GUIDANCE_MODE_POS_HOLD &&
       dir->guidance_mode_code != PRODUCT_GUIDANCE_MODE_TRAJECTORY) ||
      dir->safety_envelope_id != env->safety_envelope_id ||
      (dir->operation_code != PRODUCT_OP_PILOT &&
       dir->operation_code != PRODUCT_OP_HOLD &&
       dir->operation_code != PRODUCT_OP_MISSION &&
       dir->operation_code != PRODUCT_OP_RETURN &&
       dir->operation_code != PRODUCT_OP_LAND &&
       dir->operation_code != PRODUCT_OP_SAFE_ACTION)) {
    return 0u;
  }

  if (!header_valid(&env->header, &nav->header, MSG_SAFETY_ENVELOPE,
                    in->evaluation_time_us, in->evaluation_time_us + 1u) ||
      !validate_envelope(env)) {
    return 0u;
  }

  if (dir->operation_code == PRODUCT_OP_MISSION) {
    if (r->mission_epoch == 0u || r->current_item_id == 0u ||
        r->execution_token == 0u) return 0u;
  } else if (r->execution_token != 0u) {
    return 0u;
  }

  if (preview_us > UINT64_MAX - r->header.timestamp_sample_us ||
      (uint64_t)in->response_delay_us >
          UINT64_MAX - r->header.timestamp_sample_us - preview_us) {
    return 0u;
  }
  *proof_end_us = r->header.timestamp_sample_us + preview_us +
                  (uint64_t)in->response_delay_us;

  /* The reference and authority must remain valid through the entire
     preview plus response-delay horizon, not merely at validation time. */
  if (r->header.valid_until_us < *proof_end_us ||
      dir->header.valid_until_us < *proof_end_us ||
      dir->directive_expires_at_us < *proof_end_us ||
      env->header.valid_until_us < *proof_end_us) {
    return 0u;
  }

  if (in->map->initialized == 0u || in->map->valid == 0u ||
      in->map->degraded != 0u || in->map->reserved0 != 0u ||
      !finite3(in->map->center_ned_m) ||
      in->map->map_revision != in->expected_map_revision ||
      in->map->boot_id != nav->header.boot_id ||
      in->map->time_epoch != nav->header.time_epoch ||
      in->map->origin_id != nav->origin_id ||
      in->map->state_epoch != nav->state_epoch ||
      in->map->max_dynamic_prediction_us < M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US ||
      !isfinite(in->map->vehicle_radius_m) || in->map->vehicle_radius_m <= 0.0f ||
      !isfinite(in->map->clearance_margin_m) || in->map->clearance_margin_m < 0.0f ||
      !isfinite(in->map->uncertainty_inflation_factor) ||
      in->map->uncertainty_inflation_factor <= 0.0f ||
      in->map->source_deadline_us < *proof_end_us ||
      !m5_local_map_is_ready(in->map, nav, in->evaluation_time_us)) {
    return 0u;
  }

  if (in->geofence_count != 0u) {
    const ProductHomeTargetSnapshot *home = &dir->payload.home;
    if (home->validity != PRODUCT_HOME_VALID ||
        home->frame_id != FRAME_LOCAL_NED ||
        home->origin_id != nav->origin_id ||
        home->state_epoch != nav->state_epoch ||
        home->home_expires_at_us < *proof_end_us ||
        !finite3(home->position_ned_m) ||
        !finite3(home->position_std_m)) return 0u;
    for (axis = 0u; axis < 3u; ++axis)
      if (home->position_std_m[axis] < 0.0f) return 0u;
    for (i = 0u; i < in->geofence_count; ++i) {
      const ProductGeofenceSnapshot *f = &in->geofences[i];
      if (!header_valid(&f->header, &nav->header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
                        in->evaluation_time_us, *proof_end_us) ||
          f->validity != PRODUCT_GEOFENCE_VALID ||
          f->frame_id != FRAME_LOCAL_NED ||
          f->origin_id != nav->origin_id ||
          f->state_epoch != nav->state_epoch || f->fence_id == 0u ||
          f->reserved0 != 0u || !fence_geometry_valid(f)) return 0u;
    }
  }

  if (!finite_motion_point(&r->anchor)) return 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    float grid_coordinate = in->map->center_ned_m[axis] / M5_MAP_CELL_SIZE_M;
    if (!isfinite(grid_coordinate) ||
        fabsf(grid_coordinate - roundf(grid_coordinate)) > 1e-4f) return 0u;
  }

  for (axis = 0u; axis < 3u; ++axis) {
    if (fabsf(r->anchor.point.position_ned_m[axis] -
              nav->core.position_ned_m[axis]) > M5_SAFETY_ANCHOR_POSITION_TOL_M ||
        fabsf(r->anchor.point.velocity_ned_mps[axis] -
              nav->core.velocity_ned_mps[axis]) > M5_SAFETY_ANCHOR_VELOCITY_TOL_MPS ||
        fabsf(r->anchor.point.acceleration_ned_mps2[axis] -
              nav->core.acceleration_ned_mps2[axis]) >
              M5_SAFETY_ANCHOR_ACCEL_TOL_MPS2) return 0u;
  }

  for (i = 0u; i < M5_SAFETY_INTERVAL_COUNT; ++i) {
    const MotionReferencePoint *p = &r->points[i];
    if (p->offset_us != (i + 1u) * PRODUCT_PREVIEW_STEP_US ||
        !finite_motion_point(p)) return 0u;
  }

  *valid_until_us = earlier_u64(
      earlier_u64(r->header.valid_until_us, dir->header.valid_until_us),
      earlier_u64(env->header.valid_until_us, nav->header.valid_until_us));
  *valid_until_us = earlier_u64(*valid_until_us, dir->directive_expires_at_us);
  *valid_until_us = earlier_u64(*valid_until_us, in->map->source_deadline_us);
  if (in->geofence_count != 0u) {
    *valid_until_us = earlier_u64(*valid_until_us,
        in->directive->payload.home.home_expires_at_us);
  }
  for (i = 0u; i < in->geofence_count; ++i)
    *valid_until_us = earlier_u64(*valid_until_us, in->geofences[i].header.valid_until_us);
  return (uint8_t)(*valid_until_us > in->evaluation_time_us);
}

static double evaluate_bernstein(const double *control, unsigned degree, double u)
{
  double work[6];
  unsigned i, level;
  for (i = 0u; i <= degree; ++i) work[i] = control[i];
  for (level = 1u; level <= degree; ++level)
    for (i = 0u; i <= degree - level; ++i)
      work[i] = (1.0 - u) * work[i] + u * work[i + 1u];
  return work[0];
}

static uint8_t build_curve(
    const MotionReferencePoint *a,
    const MotionReferencePoint *b,
    double segment_duration_s,
    float u0,
    float u1,
    M5SafetyBezier *curve)
{
  double t0 = segment_duration_s * (double)u0;
  double dt = segment_duration_s * ((double)u1 - (double)u0);
  double t_power[6], dt_power[6];
  unsigned axis, i, k, j;
  if (!(dt > 0.0) || !isfinite(dt) || !isfinite(t0)) return 0u;
  t_power[0] = 1.0;
  dt_power[0] = 1.0;
  for (i = 1u; i <= 5u; ++i) {
    t_power[i] = t_power[i - 1u] * t0;
    dt_power[i] = dt_power[i - 1u] * dt;
  }

  for (axis = 0u; axis < 3u; ++axis) {
    double c[6], local_power[6] = {0};
    double p0 = a->point.position_ned_m[axis];
    double v0 = a->point.velocity_ned_mps[axis];
    double a0 = a->point.acceleration_ned_mps2[axis];
    double p1 = b->point.position_ned_m[axis];
    double v1 = b->point.velocity_ned_mps[axis];
    double a1 = b->point.acceleration_ned_mps2[axis];
    double h = segment_duration_s;
    double dp = p1 - (p0 + v0 * h + 0.5 * a0 * h * h);
    double dv = v1 - (v0 + a0 * h);
    double da = a1 - a0;

    c[0] = p0;
    c[1] = v0;
    c[2] = 0.5 * a0;
    c[3] = 10.0 * dp / (h * h * h) -
           4.0 * dv / (h * h) + 0.5 * da / h;
    c[4] = -15.0 * dp / (h * h * h * h) +
           7.0 * dv / (h * h * h) - da / (h * h);
    c[5] = 6.0 * dp / (h * h * h * h * h) -
           3.0 * dv / (h * h * h * h) + 0.5 * da / (h * h * h);

    for (k = 0u; k <= 5u; ++k) {
      for (i = k; i <= 5u; ++i)
        local_power[k] += c[i] * choose[i][k] * t_power[i - k] * dt_power[k];
    }
    for (j = 0u; j <= 5u; ++j) {
      double value = 0.0;
      for (k = 0u; k <= j; ++k)
        value += local_power[k] * choose[j][k] / choose[5][k];
      if (!isfinite(value)) return 0u;
      curve->position[axis][j] = value;
    }

    {
      M5SafetyInterval hp = interval_exact(h);
      M5SafetyInterval h2 = interval_mul(hp, hp);
      M5SafetyInterval h3 = interval_mul(h2, hp);
      M5SafetyInterval h4 = interval_mul(h3, hp);
      M5SafetyInterval h5 = interval_mul(h4, hp);
      M5SafetyInterval p0_i = interval_exact(p0);
      M5SafetyInterval v0_i = interval_exact(v0);
      M5SafetyInterval a0_i = interval_exact(a0);
      M5SafetyInterval p1_i = interval_exact(p1);
      M5SafetyInterval v1_i = interval_exact(v1);
      M5SafetyInterval a1_i = interval_exact(a1);
      M5SafetyInterval dp_i, dv_i, da_i, c_i[6];
      M5SafetyInterval t0_i, dt_i, t_power_i[6], dt_power_i[6];
      M5SafetyInterval local_power_i[6], bezier_i[6];
      M5SafetyInterval predicted_i;

      if (h2.valid == 0u || h3.valid == 0u ||
          h4.valid == 0u || h5.valid == 0u) return 0u;
      predicted_i = interval_add(
          interval_add(p0_i, interval_mul(v0_i, hp)),
          interval_mul(interval_scale(a0_i, 0.5), h2));
      dp_i = interval_sub(p1_i, predicted_i);
      dv_i = interval_sub(v1_i,
          interval_add(v0_i, interval_mul(a0_i, hp)));
      da_i = interval_sub(a1_i, a0_i);

      c_i[0] = p0_i;
      c_i[1] = v0_i;
      c_i[2] = interval_scale(a0_i, 0.5);
      c_i[3] = interval_add(
          interval_sub(
              interval_div(interval_scale(dp_i, 10.0), h3),
              interval_div(interval_scale(dv_i, 4.0), h2)),
          interval_div(interval_scale(da_i, 0.5), hp));
      c_i[4] = interval_add(
          interval_add(
              interval_div(interval_scale(dp_i, -15.0), h4),
              interval_div(interval_scale(dv_i, 7.0), h3)),
          interval_div(interval_scale(da_i, -1.0), h2));
      c_i[5] = interval_add(
          interval_add(
              interval_div(interval_scale(dp_i, 6.0), h5),
              interval_div(interval_scale(dv_i, -3.0), h4)),
          interval_div(interval_scale(da_i, 0.5), h3));

      t0_i = interval_mul(hp, interval_exact((double)u0));
      dt_i = interval_mul(hp,
          interval_sub(interval_exact((double)u1), interval_exact((double)u0)));
      t_power_i[0] = interval_exact(1.0);
      dt_power_i[0] = interval_exact(1.0);
      for (i = 1u; i <= 5u; ++i) {
        t_power_i[i] = interval_mul(t_power_i[i - 1u], t0_i);
        dt_power_i[i] = interval_mul(dt_power_i[i - 1u], dt_i);
      }
      for (k = 0u; k <= 5u; ++k) {
        local_power_i[k] = interval_exact(0.0);
        for (i = k; i <= 5u; ++i) {
          M5SafetyInterval term = interval_mul(c_i[i],
              interval_exact(choose[i][k]));
          term = interval_mul(term, t_power_i[i - k]);
          term = interval_mul(term, dt_power_i[k]);
          local_power_i[k] = interval_add(local_power_i[k], term);
        }
      }
      for (j = 0u; j <= 5u; ++j) {
        bezier_i[j] = interval_exact(0.0);
        for (k = 0u; k <= j; ++k) {
          M5SafetyInterval term = interval_mul(local_power_i[k],
              interval_exact(choose[j][k]));
          term = interval_div(term, interval_exact(choose[5][k]));
          bezier_i[j] = interval_add(bezier_i[j], term);
        }
        if (bezier_i[j].valid == 0u || c_i[0].valid == 0u ||
            c_i[1].valid == 0u || c_i[2].valid == 0u ||
            c_i[3].valid == 0u || c_i[4].valid == 0u ||
            c_i[5].valid == 0u || t_power_i[j].valid == 0u) return 0u;
        curve->position_low[axis][j] =
            fmin(bezier_i[j].low, curve->position[axis][j]);
        curve->position_high[axis][j] =
            fmax(bezier_i[j].high, curve->position[axis][j]);
      }
    }
  }
  return 1u;
}

static uint8_t curve_bounds(
    const M5SafetyBezier *curve,
    double duration_s,
    M5SafetyCurveBounds *bounds)
{
  double speed2 = 0.0, acceleration2 = 0.0, velocity_mid2 = 0.0;
  double position_rounding2 = 0.0, velocity_rounding2 = 0.0;
  M5SafetyInterval duration_interval;
  unsigned axis, i;

  if (curve == NULL || bounds == NULL ||
      !(duration_s > 0.0) || !isfinite(duration_s)) return 0u;
  duration_interval = interval_bounds(
      nextafter(duration_s, -INFINITY), nextafter(duration_s, INFINITY));
  if (duration_interval.valid == 0u) return 0u;

  for (axis = 0u; axis < 3u; ++axis) {
    M5SafetyInterval position_control[6];
    M5SafetyInterval velocity_control[5];
    M5SafetyInterval acceleration_control[4];
    M5SafetyInterval position_mid_interval, velocity_mid_interval;
    double position_low = INFINITY, position_high = -INFINITY;
    double velocity_abs = 0.0, acceleration_abs = 0.0;
    double velocity_mid_abs;
    double velocity_mid_control[5];
    double position_mid, velocity_mid;

    for (i = 0u; i < 6u; ++i) {
      position_control[i] = interval_bounds(
          curve->position_low[axis][i], curve->position_high[axis][i]);
      if (position_control[i].valid == 0u ||
          !isfinite(curve->position[axis][i])) return 0u;
      position_low = fmin(position_low, position_control[i].low);
      position_high = fmax(position_high, position_control[i].high);
    }

    for (i = 0u; i < 5u; ++i) {
      M5SafetyInterval difference = interval_sub(
          position_control[i + 1u], position_control[i]);
      velocity_control[i] = interval_div(
          interval_scale(difference, 5.0), duration_interval);
      if (velocity_control[i].valid == 0u) return 0u;
      velocity_abs = fmax(velocity_abs,
                          interval_abs_max(velocity_control[i]));
      velocity_mid_control[i] = 5.0 *
          (curve->position[axis][i + 1u] -
           curve->position[axis][i]) / duration_s;
      if (!isfinite(velocity_mid_control[i])) return 0u;
    }

    for (i = 0u; i < 4u; ++i) {
      M5SafetyInterval difference = interval_sub(
          velocity_control[i + 1u], velocity_control[i]);
      acceleration_control[i] = interval_div(
          interval_scale(difference, 4.0), duration_interval);
      if (acceleration_control[i].valid == 0u) return 0u;
      acceleration_abs = fmax(acceleration_abs,
                              interval_abs_max(acceleration_control[i]));
    }

    position_mid_interval =
        interval_evaluate_bernstein(position_control, 5u, 0.5);
    velocity_mid_interval =
        interval_evaluate_bernstein(velocity_control, 4u, 0.5);
    if (position_mid_interval.valid == 0u ||
        velocity_mid_interval.valid == 0u) return 0u;

    position_mid = evaluate_bernstein(curve->position[axis], 5u, 0.5);
    velocity_mid = evaluate_bernstein(velocity_mid_control, 4u, 0.5);
    bounds->position[axis] = (float)position_mid;
    bounds->velocity[axis] = (float)velocity_mid;
    if (!isfinite(position_mid) || !isfinite(velocity_mid) ||
        !isfinite(bounds->position[axis]) ||
        !isfinite(bounds->velocity[axis])) return 0u;

    {
      double position_error = fmax(
          fabs((double)bounds->position[axis] - position_mid_interval.low),
          fabs(position_mid_interval.high - (double)bounds->position[axis]));
      double velocity_error = fmax(
          fabs((double)bounds->velocity[axis] - velocity_mid_interval.low),
          fabs(velocity_mid_interval.high - (double)bounds->velocity[axis]));
      velocity_mid_abs = interval_abs_max(velocity_mid_interval);
      position_rounding2 = sum_square_upper(position_rounding2, position_error);
      velocity_rounding2 = sum_square_upper(velocity_rounding2, velocity_error);
      velocity_mid2 = sum_square_upper(velocity_mid2, velocity_mid_abs);
    }

    bounds->position_low[axis] = round_down_float(position_low);
    bounds->position_high[axis] = round_up_float(position_high);
    bounds->velocity_axis_bound[axis] = round_up_float(velocity_abs);
    bounds->acceleration_axis_bound[axis] = round_up_float(acceleration_abs);
    if (!isfinite(bounds->position_low[axis]) ||
        !isfinite(bounds->position_high[axis]) ||
        !isfinite(bounds->velocity_axis_bound[axis]) ||
        !isfinite(bounds->acceleration_axis_bound[axis])) return 0u;

    speed2 = sum_square_upper(speed2, velocity_abs);
    acceleration2 = sum_square_upper(acceleration2, acceleration_abs);
  }

  bounds->speed_bound = round_up_float(nextafter(sqrt(speed2), INFINITY));
  bounds->acceleration_bound =
      round_up_float(nextafter(sqrt(acceleration2), INFINITY));
  bounds->velocity_mid_norm =
      round_up_float(nextafter(sqrt(velocity_mid2), INFINITY));
  bounds->position_rounding_error_m =
      round_up_float(nextafter(sqrt(position_rounding2), INFINITY));
  bounds->velocity_rounding_error_mps =
      round_up_float(nextafter(sqrt(velocity_rounding2), INFINITY));
  bounds->half_duration_s = round_up_float(0.5 * duration_s);
  return (uint8_t)(isfinite(bounds->speed_bound) &&
      isfinite(bounds->acceleration_bound) &&
      isfinite(bounds->velocity_mid_norm) &&
      isfinite(bounds->position_rounding_error_m) &&
      isfinite(bounds->velocity_rounding_error_mps));
}

static M5TrajectorySafetyStatus state_status(uint8_t state)
{
  switch (state) {
    case M5_MAP_CELL_FREE_CONFIRMED: return M5_SAFETY_PASS;
    case M5_MAP_CELL_UNKNOWN: return M5_SAFETY_UNKNOWN;
    case M5_MAP_CELL_STALE: return M5_SAFETY_STALE;
    case M5_MAP_CELL_INVALID: return M5_SAFETY_INVALID;
    case M5_MAP_CELL_OCCUPIED_CONFIRMED: return M5_SAFETY_STATIC_COLLISION;
    default: return M5_SAFETY_INVALID;
  }
}

static uint8_t map_cell_encoding_valid(const M5MapCell *cell)
{
  if (cell == NULL || cell->log_odds < -32 || cell->log_odds > 32) return 0u;
  switch (cell->state) {
    case M5_MAP_CELL_UNKNOWN:
      return (uint8_t)(cell->log_odds > -4 && cell->log_odds < 4);
    case M5_MAP_CELL_FREE_CONFIRMED:
      return (uint8_t)(cell->log_odds <= -4 && cell->source_mask != 0u);
    case M5_MAP_CELL_OCCUPIED_CONFIRMED:
      return (uint8_t)(cell->log_odds >= 4 && cell->source_mask != 0u);
    case M5_MAP_CELL_STALE:
      return 1u;
    default:
      return 0u;
  }
}

static uint16_t state_reason(M5TrajectorySafetyStatus status)
{
  switch (status) {
    case M5_SAFETY_UNKNOWN: return M5_SAFETY_REASON_MAP_UNKNOWN;
    case M5_SAFETY_STALE: return M5_SAFETY_REASON_MAP_STALE;
    case M5_SAFETY_INVALID: return M5_SAFETY_REASON_MAP_CELL_INVALID;
    case M5_SAFETY_STATIC_COLLISION: return M5_SAFETY_REASON_STATIC_COLLISION;
    default: return M5_SAFETY_REASON_CLEARANCE_UNPROVEN;
  }
}

static uint8_t point_outside_map(
    const M5LocalMap *map,
    const float p[3],
    float radius)
{
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    float low = map->center_ned_m[axis] - M5_MAP_HALF_EXTENT_M;
    float high = map->center_ned_m[axis] + M5_MAP_HALF_EXTENT_M;
    if (p[axis] - radius < low || p[axis] + radius >= high) return 1u;
  }
  return 0u;
}


static uint8_t point_aabb_distance_lower(
    const float point[3],
    const float center[3],
    float half_extent,
    float point_rounding_error,
    float *distance_lower)
{
  double distance2 = 0.0;
  unsigned axis;
  float geometric_lower;

  if (point == NULL || center == NULL || distance_lower == NULL ||
      !finite3(point) || !finite3(center) ||
      !isfinite(half_extent) || half_extent < 0.0f ||
      !isfinite(point_rounding_error) || point_rounding_error < 0.0f) {
    return 0u;
  }

  for (axis = 0u; axis < 3u; ++axis) {
    double delta = fabs((double)point[axis] - center[axis]) -
                   (double)half_extent;
    if (delta > 0.0) distance2 += delta * delta;
  }
  geometric_lower = norm3_lower(sqrt(distance2), 0.0, 0.0);
  *distance_lower = fmaxf(0.0f, round_down_float(
      (double)geometric_lower - point_rounding_error));
  return (uint8_t)isfinite(*distance_lower);
}

static uint8_t dynamic_relative_sweep_bound(
    float curve_speed_bound,
    float obstacle_speed_bound,
    float velocity_std_mps,
    float relative_speed_bound,
    float relative_acceleration_bound,
    float half_duration_s,
    float *sweep_bound)
{
  float speed_sweep, acceleration_sweep;

  if (sweep_bound == NULL ||
      !isfinite(curve_speed_bound) || curve_speed_bound < 0.0f ||
      !isfinite(obstacle_speed_bound) || obstacle_speed_bound < 0.0f ||
      !isfinite(velocity_std_mps) || velocity_std_mps < 0.0f ||
      !isfinite(relative_speed_bound) || relative_speed_bound < 0.0f ||
      !isfinite(relative_acceleration_bound) ||
          relative_acceleration_bound < 0.0f ||
      !isfinite(half_duration_s) || half_duration_s < 0.0f) {
    return 0u;
  }

  speed_sweep = round_up_float(
      ((double)curve_speed_bound + obstacle_speed_bound +
       velocity_std_mps) * half_duration_s);
  acceleration_sweep = round_up_float(
      (double)relative_speed_bound * half_duration_s +
      0.5 * relative_acceleration_bound * half_duration_s *
          half_duration_s +
      (double)velocity_std_mps * half_duration_s);
  if (!isfinite(speed_sweep) || !isfinite(acceleration_sweep)) return 0u;

  *sweep_bound = fminf(speed_sweep, acceleration_sweep);
  return (uint8_t)isfinite(*sweep_bound);
}

static M5TrajectorySafetyStatus check_static_map(
    const M5TrajectorySafetyInput *in,
    const M5SafetyCurveBounds *curve,
    float swept_radius,
    float inflation,
    uint64_t proof_end_us,
    M5TrajectorySafetyResult *result,
    M5SafetyCheck *check)
{
  const M5LocalMap *map = in->map;
  float search_radius = round_up_float((double)swept_radius + inflation);
  float low[3], high[3];
  int lo[3], hi[3];
  unsigned axis, x, y, z;

   if (!isfinite(search_radius) || search_radius < 0.0f)
    return M5_SAFETY_NUMERIC_FAILURE;

  if (point_outside_map(map, curve->position,
                       round_up_float((double)inflation +
                                      curve->position_rounding_error_m)))
    return M5_SAFETY_UNKNOWN;

  for (axis = 0u; axis < 3u; ++axis) {
    float map_low = map->center_ned_m[axis] - M5_MAP_HALF_EXTENT_M;
    float map_high = map->center_ned_m[axis] + M5_MAP_HALF_EXTENT_M;
    low[axis] = round_down_float((double)curve->position[axis] - search_radius);
    high[axis] = round_up_float((double)curve->position[axis] + search_radius);
    if (low[axis] < map_low || high[axis] >= map_high) {
      mark_unresolved(check, M5_SAFETY_UNKNOWN);
      return M5_SAFETY_PASS;
    }
    lo[axis] = (int)floor(((double)low[axis] - map_low) / M5_MAP_CELL_SIZE_M);
    hi[axis] = (int)floor(((double)high[axis] - map_low) / M5_MAP_CELL_SIZE_M);
    if (lo[axis] < 0 || hi[axis] >= (int)M5_MAP_DIM)
      return M5_SAFETY_UNKNOWN;
  }

  for (z = (unsigned)lo[2]; z <= (unsigned)hi[2]; ++z) {
    for (y = (unsigned)lo[1]; y <= (unsigned)hi[1]; ++y) {
      for (x = (unsigned)lo[0]; x <= (unsigned)hi[0]; ++x) {
        uint16_t voxel = (uint16_t)((z * M5_MAP_DIM + y) * M5_MAP_DIM + x);
        uint8_t state;
        float center[3], distance;
        M5TrajectorySafetyStatus status;
        if (++result->cell_checks > M5_SAFETY_MAX_CELL_CHECKS)
          return M5_SAFETY_BUDGET_EXCEEDED;
        if (!map_cell_encoding_valid(&map->cells[voxel]) ||
            !m5_local_map_voxel_state(map, voxel, proof_end_us, &state))
          return M5_SAFETY_INVALID;
        status = state_status(state);
        m5_local_map_voxel_center(map, voxel, center);
        if (!point_aabb_distance_lower(curve->position, center,
                M5_SAFETY_CELL_HALF_M, curve->position_rounding_error_m,
                &distance)) return M5_SAFETY_NUMERIC_FAILURE;

        if (distance <= inflation && status != M5_SAFETY_PASS)
          return status;
        if (distance <= search_radius && status != M5_SAFETY_PASS) {
          mark_unresolved(check, status);
        }
      }
    }
  }
  return M5_SAFETY_PASS;
}

static M5TrajectorySafetyStatus check_geofences(
    const M5TrajectorySafetyInput *in,
    const M5SafetyCurveBounds *curve,
    float swept_radius,
    float inflation,
    M5TrajectorySafetyResult *result,
    M5SafetyCheck *check)
{
  const float home_down = in->directive->payload.home.position_ned_m[2];
  unsigned i;
  for (i = 0u; i < in->geofence_count; ++i) {
    const ProductGeofenceSnapshot *f = &in->geofences[i];
    float clearance, height_mid, height_low, height_high;
    float min_height, max_height;
    result->geofence_checks++;
    clearance = round_down_float((double)fence_clearance(
        f, curve->position[0], curve->position[1]) -
        M5_SAFETY_GEOMETRY_EPSILON_M);
    height_mid = home_down - curve->position[2];
    height_low = round_down_float((double)home_down - curve->position_high[2]);
    height_high = round_up_float((double)home_down - curve->position_low[2]);
    min_height = round_up_float((double)f->min_height_above_home_m +
        f->vertical_margin_m + inflation);
    max_height = round_down_float((double)f->max_height_above_home_m -
        f->vertical_margin_m - inflation);
    if (!isfinite(clearance) || !isfinite(height_mid) ||
        !isfinite(height_low) || !isfinite(height_high) ||
        !isfinite(min_height) || !isfinite(max_height) ||
        min_height >= max_height) return M5_SAFETY_NUMERIC_FAILURE;

    if (clearance <= inflation || height_mid < min_height ||
        height_mid > max_height) return M5_SAFETY_GEOFENCE_VIOLATION;

    if (clearance <= round_up_float((double)inflation + swept_radius) ||
        height_low < min_height || height_high > max_height) {
      mark_unresolved(check, M5_SAFETY_GEOFENCE_VIOLATION);
    }
  }
  return M5_SAFETY_PASS;
}

static M5TrajectorySafetyStatus validate_tracks(
    const M5TrajectorySafetyInput *in,
    uint64_t proof_end_us,
    uint64_t *valid_until_us)
{
  const M5LocalMap *map = in->map;
  const uint64_t reference_time = in->candidate_reference->header.timestamp_sample_us;
  unsigned i, j;
  uint8_t has_track = 0u;
  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i) {
    const M5DynamicTrack *t = &map->tracks[i];
    uint64_t prediction_limit;
    if (t->id == 0u) continue;
    has_track = 1u;
    if (t->source_id == 0u || t->flags != 0u ||
        t->sample_us > in->evaluation_time_us ||
        t->sample_us > reference_time + 5000u ||
        (reference_time > t->sample_us &&
         reference_time - t->sample_us > M5_MAP_STALE_US) ||
        t->expires_us < proof_end_us || t->expires_us <= t->sample_us ||
        t->expires_us - t->sample_us > map->max_dynamic_prediction_us ||
        !finite3(t->position_ned_m) || !finite3(t->velocity_ned_mps) ||
        !isfinite(t->radius_m) || t->radius_m < 0.0f ||
        !isfinite(t->position_std_m) || t->position_std_m < 0.0f ||
        !isfinite(t->velocity_std_mps) || t->velocity_std_mps < 0.0f) {
      return M5_SAFETY_STALE;
    }
    if (t->sample_us > UINT64_MAX - map->max_dynamic_prediction_us)
      return M5_SAFETY_INVALID;
    prediction_limit = t->sample_us + map->max_dynamic_prediction_us;
    if (t->expires_us > prediction_limit) return M5_SAFETY_INVALID;
    *valid_until_us = earlier_u64(*valid_until_us, t->expires_us);
    for (j = i + 1u; j < M5_MAP_MAX_TRACKS; ++j) {
      if (map->tracks[j].id == t->id &&
          map->tracks[j].source_id == t->source_id) return M5_SAFETY_INVALID;
    }
  }
  if (has_track && in->max_dynamic_obstacle_accel_mps2 <= 0.0f)
    return M5_SAFETY_INVALID;
  return M5_SAFETY_PASS;
}

static M5TrajectorySafetyStatus check_dynamic_tracks(
    const M5TrajectorySafetyInput *in,
    const M5SafetyCurveBounds *curve,
    float inflation,
    float horizon_mid_s,
    M5TrajectorySafetyResult *result,
    M5SafetyCheck *check)
{
  const M5LocalMap *map = in->map;
  const MotionReference *r = in->candidate_reference;
  const float delay_s = (float)in->response_delay_us * 1e-6f;
  const float dynamic_accel = in->max_dynamic_obstacle_accel_mps2;
  unsigned i, axis;
  float half_dt = curve->half_duration_s;

  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i) {
    const M5DynamicTrack *t = &map->tracks[i];
    float dt, obstacle[3], relative_velocity[3];
    float obstacle_speed, relative_speed, relative_accel;
    float uncertainty_horizon, uncertainty, combined_radius;
    float relative_sweep, distance;
    double obstacle_rounding2 = 0.0;
    if (t->id == 0u) continue;
    if (++result->track_checks > M5_SAFETY_MAX_WORK_UNITS * M5_MAP_MAX_TRACKS)
      return M5_SAFETY_BUDGET_EXCEEDED;

    {
      uint64_t interval_mid_us = r->header.timestamp_sample_us +
          (uint64_t)(horizon_mid_s * 1000000.0f);
      double dt_exact;
      if (interval_mid_us >= t->sample_us)
        dt_exact = (double)(interval_mid_us - t->sample_us) * 1e-6;
      else
        dt_exact = -(double)(t->sample_us - interval_mid_us) * 1e-6;
      dt = (float)dt_exact;
      for (axis = 0u; axis < 3u; ++axis) {
        double predicted = (double)t->position_ned_m[axis] +
                           (double)t->velocity_ned_mps[axis] * dt_exact;
        obstacle[axis] = (float)predicted;
        obstacle_rounding2 += ((double)obstacle[axis] - predicted) *
                              ((double)obstacle[axis] - predicted);
        relative_velocity[axis] = curve->velocity[axis] - t->velocity_ned_mps[axis];
      }
    }
    if (!finite3(obstacle) || !finite3(relative_velocity) || !isfinite(dt))
      return M5_SAFETY_NUMERIC_FAILURE;

    obstacle_speed = norm3_upper(t->velocity_ned_mps);
    relative_speed = round_up_float((double)norm3_upper(relative_velocity) +
        curve->velocity_rounding_error_mps + t->velocity_std_mps);
    relative_accel = round_up_float((double)curve->acceleration_bound + dynamic_accel);
    uncertainty_horizon = round_up_float(fabs((double)dt) + half_dt + delay_s);
    uncertainty = round_up_float((double)map->uncertainty_inflation_factor *
        ((double)t->position_std_m +
         (double)uncertainty_horizon * t->velocity_std_mps +
         0.5 * dynamic_accel * uncertainty_horizon * uncertainty_horizon));
    /* During response delay the obstacle's known nominal velocity also moves
       it. Inflate by that bounded displacement; acceleration and velocity
       uncertainty are already included above. */
    combined_radius = round_up_float((double)t->radius_m + inflation + uncertainty +
        (double)obstacle_speed * delay_s + sqrt(obstacle_rounding2));
    if (!dynamic_relative_sweep_bound(curve->speed_bound, obstacle_speed,
            t->velocity_std_mps, relative_speed, relative_accel, half_dt,
            &relative_sweep)) return M5_SAFETY_NUMERIC_FAILURE;
    {
      double dx = (double)curve->position[0] - obstacle[0];
      double dy = (double)curve->position[1] - obstacle[1];
      double dz = (double)curve->position[2] - obstacle[2];
      distance = norm3_lower(dx, dy, dz);
      distance = fmaxf(0.0f, round_down_float((double)distance -
          curve->position_rounding_error_m - round_up_float(sqrt(obstacle_rounding2))));
    }
    if (!isfinite(combined_radius) || !isfinite(relative_sweep) ||
        !isfinite(distance)) return M5_SAFETY_NUMERIC_FAILURE;
    if (distance <= combined_radius) return M5_SAFETY_DYNAMIC_COLLISION;
    if (distance <= round_up_float((double)combined_radius + relative_sweep)) {
      mark_unresolved(check, M5_SAFETY_DYNAMIC_COLLISION);
    }
  }
  return M5_SAFETY_PASS;
}

static uint8_t compute_inflation(
    const M5TrajectorySafetyInput *in,
    const M5SafetyCurveBounds *curve,
    float horizon_mid_s,
    float *inflation)
{
  const M5LocalMap *map = in->map;
  const NavigationSnapshot *nav = in->navigation;
  float position_std[3], velocity_std[3];
  float home_std[3] = {0.0f, 0.0f, 0.0f};
  /* Bound uncertainty at the far edge of this sub-interval and through
     the response delay, not only at the interval midpoint. */
  double uncertainty_horizon = (double)horizon_mid_s +
      (double)curve->half_duration_s +
      (double)in->response_delay_us * 1e-6;
  double delay_s = (double)in->response_delay_us * 1e-6;
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    position_std[axis] = nav->core.position_std_m[axis];
    velocity_std[axis] = nav->core.velocity_std_mps[axis];
  }
  if (in->geofence_count != 0u) {
    for (axis = 0u; axis < 3u; ++axis)
      home_std[axis] = in->directive->payload.home.position_std_m[axis];
  }
  {
    double position_norm = sqrt((double)position_std[0] * position_std[0] +
        (double)position_std[1] * position_std[1] +
        (double)position_std[2] * position_std[2]);
    double velocity_norm = sqrt((double)velocity_std[0] * velocity_std[0] +
        (double)velocity_std[1] * velocity_std[1] +
        (double)velocity_std[2] * velocity_std[2]);
    double home_norm = sqrt((double)home_std[0] * home_std[0] +
        (double)home_std[1] * home_std[1] +
        (double)home_std[2] * home_std[2]);
    double uncertainty = position_norm + uncertainty_horizon * velocity_norm + home_norm;
    double delay_motion = curve->speed_bound * delay_s +
        0.5 * curve->acceleration_bound * delay_s * delay_s;
    *inflation = round_up_float((double)map->vehicle_radius_m +
        map->clearance_margin_m +
        (double)map->uncertainty_inflation_factor * uncertainty + delay_motion);
  }
  return (uint8_t)(isfinite(*inflation) && *inflation > 0.0f);
}

M5TrajectorySafetyStatus m5_trajectory_safety_validate(
    const M5TrajectorySafetyInput *input,
    M5TrajectorySafetyWorkspace *workspace,
    M5TrajectorySafetyResult *result)
{
  unsigned count = 0u;
  uint64_t proof_end_us = 0u, valid_until_us = 0u;
  const MotionReference *reference;
  const uint32_t step_us = PRODUCT_PREVIEW_STEP_US;

  if (result == NULL) return M5_SAFETY_INVALID;
  memset(result, 0, sizeof(*result));
  if (input == NULL || workspace == NULL)
    return reject(result, M5_SAFETY_INVALID, M5_SAFETY_REASON_INPUT_INVALID, 0u);
  reference = input->candidate_reference;
  if (reference != NULL) {
    result->reference_id = reference->reference_id;
    result->reference_epoch = reference->reference_epoch;
    result->state_epoch = reference->state_epoch;
    result->origin_id = reference->origin_id;
  }
  result->verified_at_us = input->evaluation_time_us;

  if (!validate_reference_shape(input, &proof_end_us, &valid_until_us)) {
    return reject(result, M5_SAFETY_INVALID,
                  M5_SAFETY_REASON_REFERENCE_INVALID, 0u);
  }
  result->map_revision = input->map->map_revision;
  {
    M5TrajectorySafetyStatus tracks =
        validate_tracks(input, proof_end_us, &valid_until_us);
    if (tracks != M5_SAFETY_PASS) {
      M5TrajectorySafetyReason reason =
          (tracks == M5_SAFETY_STALE) ? M5_SAFETY_REASON_DYNAMIC_STALE :
          M5_SAFETY_REASON_DYNAMIC_INVALID;
      return reject(result, tracks, reason, 0u);
    }
  }

  memset(workspace, 0, sizeof(*workspace));
  {
    unsigned i;
    for (i = M5_SAFETY_INTERVAL_COUNT; i > 0u; --i) {
      workspace->stack[count++] = (M5TrajectorySafetyWorkItem){
        .interval = (uint8_t)(i - 1u), .depth = 0u, .u0 = 0.0f, .u1 = 1.0f
      };
    }
  }

  while (count != 0u) {
    M5TrajectorySafetyWorkItem item = workspace->stack[--count];
    const MotionReferencePoint *a = item.interval == 0u ?
        &reference->anchor : &reference->points[item.interval - 1u];
    const MotionReferencePoint *b = &reference->points[item.interval];
    M5SafetyBezier bezier;
    M5SafetyCurveBounds curve;
    M5SafetyCheck check = {0u, M5_SAFETY_PASS, 0u};
    M5TrajectorySafetyStatus status;
    float h = (float)step_us * 1e-6f;
    float local_mid = round_up_float(((double)item.interval +
        0.5 * ((double)item.u0 + (double)item.u1)) * (double)h);
    double local_duration = (double)h *
        ((double)item.u1 - (double)item.u0);
    float swept_radius, inflation;
    uint32_t offset_us = (uint32_t)item.interval * step_us +
        (uint32_t)((double)item.u0 * step_us);

    if (++result->work_units > M5_SAFETY_MAX_WORK_UNITS)
      return reject(result, M5_SAFETY_BUDGET_EXCEEDED,
                    M5_SAFETY_REASON_BUDGET, offset_us);

    if (!build_curve(a, b, h, item.u0, item.u1, &bezier) ||
        !curve_bounds(&bezier, local_duration, &curve))
      return reject(result, M5_SAFETY_NUMERIC_FAILURE,
                    M5_SAFETY_REASON_NUMERIC, offset_us);

    {
      const ProductSafetyEnvelopePayload *e =
          &input->safety_envelope->vehicle_extension;
      unsigned axis;
      if (curve.speed_bound > input->safety_envelope->max_linear_speed_mps + 1e-4f ||
          curve.acceleration_bound >
              input->safety_envelope->max_linear_accel_mps2 + 1e-4f) {
        return reject(result, M5_SAFETY_DYNAMICS_VIOLATION,
                      M5_SAFETY_REASON_DYNAMICS, offset_us);
      }
      for (axis = 0u; axis < 3u; ++axis) {
        if (curve.velocity_axis_bound[axis] >
                e->max_velocity_ned_mps[axis] + 1e-4f ||
            curve.acceleration_axis_bound[axis] >
                e->max_acceleration_ned_mps2[axis] + 1e-4f) {
          return reject(result, M5_SAFETY_DYNAMICS_VIOLATION,
                        M5_SAFETY_REASON_DYNAMICS, offset_us);
        }
      }
    }

    swept_radius = round_up_float(fmin(
        (double)curve.speed_bound * curve.half_duration_s,
        (double)curve.velocity_mid_norm * curve.half_duration_s +
        0.5 * curve.acceleration_bound * curve.half_duration_s *
            curve.half_duration_s) + curve.position_rounding_error_m);
    if (!isfinite(swept_radius) || swept_radius < 0.0f ||
        !compute_inflation(input, &curve, local_mid, &inflation))
      return reject(result, M5_SAFETY_NUMERIC_FAILURE,
                    M5_SAFETY_REASON_NUMERIC, offset_us);

    status = check_geofences(input, &curve, swept_radius, inflation,
                             result, &check);
    if (status != M5_SAFETY_PASS) {
      M5TrajectorySafetyReason reason =
          (status == M5_SAFETY_GEOFENCE_VIOLATION) ?
          M5_SAFETY_REASON_GEOFENCE_VIOLATION : M5_SAFETY_REASON_NUMERIC;
      return reject(result, status, reason, offset_us);
    }

    status = check_static_map(input, &curve, swept_radius, inflation,
                              proof_end_us, result, &check);
    if (status == M5_SAFETY_BUDGET_EXCEEDED)
      return reject(result, status, M5_SAFETY_REASON_BUDGET, offset_us);
    if (status != M5_SAFETY_PASS)
      return reject(result, status, state_reason(status), offset_us);

    status = check_dynamic_tracks(input, &curve, inflation, local_mid,
                                  result, &check);
    if (status == M5_SAFETY_BUDGET_EXCEEDED)
      return reject(result, status, M5_SAFETY_REASON_BUDGET, offset_us);
    if (status != M5_SAFETY_PASS) {
      M5TrajectorySafetyReason reason =
          (status == M5_SAFETY_DYNAMIC_COLLISION) ?
          M5_SAFETY_REASON_DYNAMIC_COLLISION : M5_SAFETY_REASON_NUMERIC;
      return reject(result, status, reason, offset_us);
    }

    if (check.unresolved == 0u) continue;
    if (item.depth >= M5_SAFETY_MAX_DEPTH) {
      M5TrajectorySafetyStatus unresolved =
          (M5TrajectorySafetyStatus)check.unresolved_status;
      M5TrajectorySafetyReason reason = M5_SAFETY_REASON_CLEARANCE_UNPROVEN;
      if (unresolved == M5_SAFETY_UNKNOWN) reason = M5_SAFETY_REASON_MAP_UNKNOWN;
      else if (unresolved == M5_SAFETY_STALE) reason = M5_SAFETY_REASON_MAP_STALE;
      else if (unresolved == M5_SAFETY_INVALID) reason = M5_SAFETY_REASON_MAP_CELL_INVALID;
      else if (unresolved == M5_SAFETY_STATIC_COLLISION) reason = M5_SAFETY_REASON_STATIC_COLLISION;
      else if (unresolved == M5_SAFETY_DYNAMIC_COLLISION) reason = M5_SAFETY_REASON_DYNAMIC_COLLISION;
      else if (unresolved == M5_SAFETY_GEOFENCE_VIOLATION) reason = M5_SAFETY_REASON_GEOFENCE_VIOLATION;
      if (unresolved == M5_SAFETY_PASS) unresolved = M5_SAFETY_CLEARANCE_UNPROVEN;
      return reject(result, unresolved, reason, offset_us);
    }
    if (count + 2u > M5_SAFETY_STACK_CAPACITY)
      return reject(result, M5_SAFETY_CAPACITY_EXCEEDED,
                    M5_SAFETY_REASON_CAPACITY, offset_us);

    {
      float middle = 0.5f * (item.u0 + item.u1);
      workspace->stack[count++] = (M5TrajectorySafetyWorkItem){
        .interval = item.interval, .depth = (uint8_t)(item.depth + 1u),
        .u0 = middle, .u1 = item.u1
      };
      workspace->stack[count++] = (M5TrajectorySafetyWorkItem){
        .interval = item.interval, .depth = (uint8_t)(item.depth + 1u),
        .u0 = item.u0, .u1 = middle
      };
    }
  }

  result->status = M5_SAFETY_PASS;
  result->passed = 1u;
  result->reason_code = M5_SAFETY_REASON_NONE;
  result->first_violation_offset_us = 0u;
  result->valid_until_us = valid_until_us;
  return M5_SAFETY_PASS;
}


#if defined(FLIGHT_M5_SAFETY_TEST_API)
#include "m5_trajectory_safety_test_api.h"

uint8_t m5_trajectory_safety_test_curve_bounds(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float position_low[3],
    float position_high[3],
    float *speed_bound,
    float *acceleration_bound)
{
  MotionReferencePoint start, end;
  M5SafetyBezier bezier;
  M5SafetyCurveBounds bounds;
  double local_duration;
  unsigned axis;

  if (start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      position_low == NULL || position_high == NULL ||
      speed_bound == NULL || acceleration_bound == NULL ||
      !finite3(start_position) || !finite3(start_velocity) ||
      !finite3(start_acceleration) || !finite3(end_position) ||
      !finite3(end_velocity) || !finite3(end_acceleration) ||
      !isfinite(duration_s) || duration_s <= 0.0f ||
      !isfinite(u0) || !isfinite(u1) || u0 < 0.0f ||
      u1 > 1.0f || u1 <= u0) return 0u;

  memset(&start, 0, sizeof(start));
  memset(&end, 0, sizeof(end));
  for (axis = 0u; axis < 3u; ++axis) {
    start.point.position_ned_m[axis] = start_position[axis];
    start.point.velocity_ned_mps[axis] = start_velocity[axis];
    start.point.acceleration_ned_mps2[axis] = start_acceleration[axis];
    end.point.position_ned_m[axis] = end_position[axis];
    end.point.velocity_ned_mps[axis] = end_velocity[axis];
    end.point.acceleration_ned_mps2[axis] = end_acceleration[axis];
  }

  if (!build_curve(&start, &end, (double)duration_s,
                   (float)u0, (float)u1, &bezier)) return 0u;
  local_duration = (double)duration_s * ((double)u1 - (double)u0);
  if (!curve_bounds(&bezier, local_duration, &bounds)) return 0u;

  for (axis = 0u; axis < 3u; ++axis) {
    position_low[axis] = bounds.position_low[axis];
    position_high[axis] = bounds.position_high[axis];
  }
  *speed_bound = bounds.speed_bound;
  *acceleration_bound = bounds.acceleration_bound;
  return 1u;
}


uint8_t m5_trajectory_safety_test_curve_eval(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float local_u,
    float position[3],
    float velocity[3],
    float acceleration[3])
{
  MotionReferencePoint start, end;
  M5SafetyBezier bezier;
  float local_duration;
  unsigned axis, i;

  if (start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      position == NULL || velocity == NULL || acceleration == NULL ||
      !finite3(start_position) || !finite3(start_velocity) ||
      !finite3(start_acceleration) || !finite3(end_position) ||
      !finite3(end_velocity) || !finite3(end_acceleration) ||
      !isfinite(duration_s) || duration_s <= 0.0f ||
      !isfinite(u0) || !isfinite(u1) || !isfinite(local_u) ||
      u0 < 0.0f || u1 > 1.0f || u1 <= u0 ||
      local_u < 0.0f || local_u > 1.0f) return 0u;

  memset(&start, 0, sizeof(start));
  memset(&end, 0, sizeof(end));
  for (axis = 0u; axis < 3u; ++axis) {
    start.point.position_ned_m[axis] = start_position[axis];
    start.point.velocity_ned_mps[axis] = start_velocity[axis];
    start.point.acceleration_ned_mps2[axis] = start_acceleration[axis];
    end.point.position_ned_m[axis] = end_position[axis];
    end.point.velocity_ned_mps[axis] = end_velocity[axis];
    end.point.acceleration_ned_mps2[axis] = end_acceleration[axis];
  }
  if (!build_curve(&start, &end, (double)duration_s,
                   (float)u0, (float)u1, &bezier)) return 0u;
  local_duration = duration_s * (u1 - u0);
  if (!(local_duration > 0.0f) || !isfinite(local_duration)) return 0u;

  for (axis = 0u; axis < 3u; ++axis) {
    double velocity_control[5], acceleration_control[4];
    double p_value, v_value, a_value;
    for (i = 0u; i < 5u; ++i) {
      velocity_control[i] = 5.0 *
          (bezier.position[axis][i + 1u] -
           bezier.position[axis][i]) / local_duration;
      if (!isfinite(velocity_control[i])) return 0u;
    }
    for (i = 0u; i < 4u; ++i) {
      acceleration_control[i] = 4.0 *
          (velocity_control[i + 1u] - velocity_control[i]) /
          local_duration;
      if (!isfinite(acceleration_control[i])) return 0u;
    }
    p_value = evaluate_bernstein(bezier.position[axis], 5u, local_u);
    v_value = evaluate_bernstein(velocity_control, 4u, local_u);
    a_value = evaluate_bernstein(acceleration_control, 3u, local_u);
    position[axis] = (float)p_value;
    velocity[axis] = (float)v_value;
    acceleration[axis] = (float)a_value;
    if (!isfinite(position[axis]) || !isfinite(velocity[axis]) ||
        !isfinite(acceleration[axis])) return 0u;
  }
  return 1u;
}


uint8_t m5_trajectory_safety_test_rounding_bounds(
    double value, float *lower, float *upper)
{
  if (lower == NULL || upper == NULL || !isfinite(value)) return 0u;
  *lower = round_down_float(value);
  *upper = round_up_float(value);
  return (uint8_t)(isfinite(*lower) && isfinite(*upper) && *lower <= *upper);
}

float m5_trajectory_safety_test_norm_lower_bound(
    double x, double y, double z)
{
  return norm3_lower(x, y, z);
}

uint8_t m5_trajectory_safety_test_point_aabb_distance_lower(
    const float point[3],
    const float center[3],
    float half_extent,
    float point_rounding_error,
    float *distance_lower)
{
  return point_aabb_distance_lower(point, center, half_extent,
                                   point_rounding_error, distance_lower);
}

uint8_t m5_trajectory_safety_test_dynamic_relative_sweep_bound(
    float curve_speed_bound,
    float obstacle_speed_bound,
    float velocity_std_mps,
    float relative_speed_bound,
    float relative_acceleration_bound,
    float half_duration_s,
    float *sweep_bound)
{
  return dynamic_relative_sweep_bound(curve_speed_bound, obstacle_speed_bound,
      velocity_std_mps, relative_speed_bound, relative_acceleration_bound,
      half_duration_s, sweep_bound);
}

#endif /* FLIGHT_M5_SAFETY_TEST_API */

#endif /* FLIGHT_M5_HOST_PLANNER */
