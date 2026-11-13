/*
 * M5-06 independent continuous trajectory safety validator.
 *
 * This file deliberately does not call the M5-03 A* edge checker, M5-05
 * smoother, or the legacy Guidance fence validator. It reconstructs the
 * consumer's quintic Hermite curve from each anchor/preview pair, converts
 * it to Bernstein form, and proves swept-volume clearance with bounded DFS.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)

#include <float.h>
#include <fenv.h>
#if (defined(__x86_64__) || defined(__i386__)) && defined(__SSE__)
#include <xmmintrin.h>
#endif
#include <math.h>
#include <stdint.h>
#include <string.h>

_Static_assert(FLT_RADIX == 2, "M5 safety requires radix-2 floating point");
_Static_assert(FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 &&
               FLT_MAX_EXP == 128,
               "M5 safety requires IEEE-754 binary32 float");
_Static_assert(DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 &&
               DBL_MAX_EXP == 1024,
               "M5 safety requires IEEE-754 binary64 double");
_Static_assert(FLT_HAS_SUBNORM == 1 && FLT_TRUE_MIN == 0x1p-149f,
               "M5 safety requires IEEE-754 binary32 subnormals");
_Static_assert(DBL_HAS_SUBNORM == 1,
               "M5 safety requires IEEE-754 binary64 subnormals");
_Static_assert(FLT_EVAL_METHOD == 0,
               "M5 safety requires evaluation at the declared type precision");
_Static_assert(sizeof(float) == sizeof(uint32_t),
               "M5 safety requires 32-bit float storage");
_Static_assert(sizeof(double) == sizeof(uint64_t),
               "M5 safety requires 64-bit double storage");

/* Some supported GCC Arm releases do not advertise __STDC_IEC_559__ even
   though their radix/precision/exponent/subnormal properties satisfy the
   binary32/binary64 requirements asserted above. Avoid relying on that
   optional conformance macro for target compilation. */
#include "trajectory_planning/m5_trajectory_safety.h"
#include "messages/message_ids.h"
#include "geofence_geometry/geofence_geometry.h"
#if defined(FLIGHT_M5_SAFETY_TEST_API)
/* The test API reads the production SafetyEnvelope source instead of keeping a
   second copy of the product limits. This is Host/SIL-only; it does not link
   the validator into the H723 product image. */
#include "fms_contract/fms_model.h"
#include "flight_policy/fms_policy.h"
#include "m5_trajectory_safety_test_api.h"

static _Thread_local M5SafetyTestConsumerRejectStage consumer_rejection;
static _Thread_local uint32_t consumer_rejection_counts[
    M5_TEST_CONSUMER_REJECT_STAGE_COUNT];

static void trace_consumer_rejection(M5SafetyTestConsumerRejectStage stage)
{
  unsigned index = (unsigned)stage;
  if (index > 0u && index < M5_TEST_CONSUMER_REJECT_STAGE_COUNT)
    consumer_rejection_counts[index] = 1u;
  if (consumer_rejection == M5_TEST_CONSUMER_REJECT_NONE)
    consumer_rejection = stage;
}

#define M5_TRACE_NONFINITE(value, stage) \
  do { if (!isfinite(value)) trace_consumer_rejection(stage); } while (0)
static void trace_consumer_horner_rejection(unsigned degree)
{
  switch (degree) {
    case 5u: trace_consumer_rejection(M5_TEST_CONSUMER_REJECT_POSITION_HORNER); break;
    case 4u: trace_consumer_rejection(M5_TEST_CONSUMER_REJECT_VELOCITY_HORNER); break;
    case 3u: trace_consumer_rejection(M5_TEST_CONSUMER_REJECT_ACCELERATION_HORNER); break;
    default: break;
  }
}
#define M5_TRACE_HORNER(degree) trace_consumer_horner_rejection(degree)
#else
#define M5_TRACE_NONFINITE(value, stage) ((void)0)
#define M5_TRACE_HORNER(degree) ((void)0)
#endif

_Static_assert(PRODUCT_PREVIEW_STEP_US <= 1000000u,
               "M5 safety preview step must stay within one second");

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
  /* Physical local-time span of this sub-interval. */
  double start_time_s;
  double end_time_s;
  /* Third derivative of the binary32 consumer polynomial in physical time:
     jerk(s) = jerk_power[axis][2]*s^2 + jerk_power[axis][1]*s +
               jerk_power[axis][0]. */
  double jerk_power[3][3];
  /* Bounds on |binary32 consumer evaluation - exact polynomial| over the
     sub-interval, per axis and derivative order (MR-009). */
  double eval_position_error[3];
  double eval_velocity_error[3];
  double eval_acceleration_error[3];
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
  float jerk_axis_bound[3];
  float jerk_bernstein_bound[3];
  float jerk_tail_bound[3];
  float jerk_consumer_bound[3];
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

/*
 * All safety bounds use outward arithmetic. The float-root helpers avoid
 * libm sqrt accuracy assumptions: positive binary32 bit patterns are ordered,
 * and the square of any binary32 value is exact in binary64 (48 significand
 * bits <= the binary64 53-bit significand).
 */
static double add_upper_nonnegative(double a, double b)
{
  if (!(a >= 0.0) || !(b >= 0.0) || !isfinite(a) || !isfinite(b))
    return INFINITY;
  if (a == 0.0) return b;
  if (b == 0.0) return a;
  return nextafter(a + b, INFINITY);
}

static double multiply_upper_nonnegative(double a, double b)
{
  double product;
  if (!(a >= 0.0) || !(b >= 0.0) || !isfinite(a) || !isfinite(b))
    return INFINITY;
  if (a == 0.0 || b == 0.0) return 0.0;
  product = a * b;
  return nextafter(product, INFINITY);
}

/*
 * Upper bound the magnitude of one round-to-nearest binary32 result.
 *
 * For a normal exact result, |fl(x)| <= (1+u)|x|, u=2^-24.
 * For a subnormal exact result, |fl(x)| <= |x| + 2^-149. The split below
 * covers both cases. The caller must first prove that the exact operation
 * cannot cross the round-to-nearest overflow midpoint; after that proof the
 * actual result is finite, so capping this bound at FLT_MAX is sound.
 */
static double binary32_rounded_upper_nonnegative(double exact_upper)
{
  double rounded_upper;
  if (!(exact_upper >= 0.0) || !isfinite(exact_upper)) return INFINITY;
  if (exact_upper == 0.0) return 0.0;
  if (exact_upper < (double)FLT_MIN) {
    rounded_upper = add_upper_nonnegative(exact_upper, 0x1p-149);
  } else {
    rounded_upper = multiply_upper_nonnegative(exact_upper, 1.0 + 0x1p-24);
  }
  if (!isfinite(rounded_upper)) return INFINITY;
  return (rounded_upper > (double)FLT_MAX)
      ? (double)FLT_MAX : rounded_upper;
}

/* gamma_2 = 2u/(1-2u), u = 2^-24, evaluated outward:
   numerator up, denominator down, quotient up. */
static double gamma2_upper(void)
{
  double nu2 = 2.0 * 0x1p-24;
  double denominator;
  double gamma2;
  nu2 = nextafter(nu2, INFINITY);
  denominator = nextafter(1.0 - nu2, -INFINITY);
  if (!(denominator > 0.0) || !isfinite(denominator)) return INFINITY;
  gamma2 = nu2 / denominator;
  return nextafter(gamma2, INFINITY);
}

static double sum_square_upper(double sum, double value)
{
  return add_upper_nonnegative(
      sum, multiply_upper_nonnegative(value, value));
}

static double sum_square_lower(double sum, double value)
{
  double square, total;
  if (!(sum >= 0.0) || !(value >= 0.0) || isnan(sum) || isnan(value))
    return 0.0;
  if (value == 0.0) return sum;
  square = nextafter(value * value, -INFINITY);
  if (!(square > 0.0)) square = 0.0;
  if (sum == 0.0) return square;
  total = nextafter(sum + square, -INFINITY);
  return total > 0.0 ? total : 0.0;
}

static double abs_difference_upper(double a, double b)
{
  double difference = fabs(a - b);
  return difference > 0.0 ? nextafter(difference, INFINITY) : 0.0;
}

static double microseconds_to_seconds_upper(uint64_t microseconds)
{
  double integer_upper;
  if (microseconds == 0u) return 0.0;
  /* One ULP covers uint64_t -> binary64 conversion before scaling. The
     multiplier is the next binary64 value above the exact 1e-6 rational. */
  integer_upper = nextafter((double)microseconds, INFINITY);
  return multiply_upper_nonnegative(
      integer_upper, 1.0000000000000002e-6);
}

static float round_up_float(double value)
{
  float rounded;
  if (isnan(value)) return (float)value;
  if (value > (double)FLT_MAX) return INFINITY;
  if (value < -(double)FLT_MAX) return -FLT_MAX;
  rounded = (float)value;
  if (!isfinite(rounded)) return rounded;
  if ((double)rounded < value) rounded = nextafterf(rounded, INFINITY);
  return rounded;
}

static float round_down_float(double value)
{
  float rounded;
  if (isnan(value)) return (float)value;
  if (value > (double)FLT_MAX) return FLT_MAX;
  if (value < -(double)FLT_MAX) return -INFINITY;
  rounded = (float)value;
  if (!isfinite(rounded)) return rounded;
  if ((double)rounded > value) rounded = nextafterf(rounded, -INFINITY);
  return rounded;
}

static float positive_float_from_bits(uint32_t bits)
{
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static float sqrt_upper_float(double value)
{
  const uint32_t max_bits = UINT32_C(0x7f7fffff);
  const double max_square = (double)FLT_MAX * (double)FLT_MAX;
  uint32_t low = 0u, high = max_bits;

  if (!(value >= 0.0) || !isfinite(value)) return INFINITY;
  if (value == 0.0) return 0.0f;
  if (value > max_square) return INFINITY;

  /* Find the least binary32 f for which the exact binary64 product f*f
     encloses value. No rounded sqrt result participates in the proof. */
  while (low < high) {
    uint32_t middle = low + (high - low) / 2u;
    float candidate = positive_float_from_bits(middle);
    double square = (double)candidate * (double)candidate;
    if (square >= value) high = middle;
    else low = middle + 1u;
  }
  return positive_float_from_bits(low);
}

static float sqrt_lower_float(double value)
{
  const uint32_t max_bits = UINT32_C(0x7f7fffff);
  const double max_square = (double)FLT_MAX * (double)FLT_MAX;
  uint32_t low = 0u, high = max_bits;

  if (!(value >= 0.0)) return 0.0f;
  if (!isfinite(value) || value >= max_square) return FLT_MAX;

  /* Find the greatest binary32 f for which the exact product f*f is no
     greater than value. This is a conservative lower square-root bound. */
  while (low < high) {
    uint32_t middle = low + (high - low + 1u) / 2u;
    float candidate = positive_float_from_bits(middle);
    double square = (double)candidate * (double)candidate;
    if (square <= value) low = middle;
    else high = middle - 1u;
  }
  return positive_float_from_bits(low);
}

static float norm3_upper(const float v[3])
{
  double sum = 0.0;
  unsigned axis;
  if (v == NULL) return INFINITY;
  for (axis = 0u; axis < 3u; ++axis) {
    double component, square;
    if (!isfinite(v[axis])) return INFINITY;
    component = (double)v[axis];
    /* A binary32 x converted to binary64 has an exact x*x product. */
    square = component * component;
    if (!isfinite(square)) return INFINITY;
    if (square != 0.0)
      sum = sum == 0.0 ? square : add_upper_nonnegative(sum, square);
  }
  return sqrt_upper_float(sum);
}

static float norm3_upper_double(double x, double y, double z)
{
  const double components[3] = {x, y, z};
  double sum = 0.0;
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    double magnitude;
    if (!isfinite(components[axis])) return INFINITY;
    magnitude = fabs(components[axis]);
    if (magnitude > 0.0) magnitude = nextafter(magnitude, INFINITY);
    sum = sum_square_upper(sum, magnitude);
  }
  return sqrt_upper_float(sum);
}

static float norm3_lower(const double x, const double y, const double z)
{
  const double components[3] = {x, y, z};
  double sum = 0.0;
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    double magnitude;
    if (!isfinite(components[axis])) return 0.0f;
    magnitude = fabs(components[axis]);
    if (magnitude > 0.0) {
      magnitude = nextafter(magnitude, -INFINITY);
      if (!(magnitude > 0.0)) magnitude = 0.0;
    }
    sum = sum_square_lower(sum, magnitude);
  }
  return sqrt_lower_float(sum);
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
    if ((p->valid_mask & PRODUCT_ENVELOPE_VALID_JERK) != 0u &&
        (!isfinite(p->max_jerk_ned_mps3[axis]) ||
         p->max_jerk_ned_mps3[axis] <= 0.0f)) return 0u;
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

/*
 * MR-009: the consumer (control_sample_reference) evaluates the same quintic in
 * binary32 with Horner's rule. The validator proves the binary64 polynomial, so
 * it must add a bound on |consumer_value - exact_value| over the sub-interval.
 *
 * consumer_power_coefficients() replicates the consumer's binary32 coefficient
 * computation operation for operation. consumer_eval_error_bound() then bounds
 *   (a) the difference between the consumer's coefficients and the exact ones,
 *   (b) the separate binary32 Horner multiply/add roundings.
 *
 * The M6 production flags in rtconfig.py pin -ffp-contract=off,
 * -frounding-math and -fexcess-precision=standard. The CI gate also compiles
 * control_reference.c with those exact production flags and scans the emitted
 * Cortex-M7 object for fused multiply/add instructions. A future toolchain or
 * flag change that emits one fails the gate and requires a new proof review.
 * The bound is evaluated at s_max = h * u1 and is therefore valid for the whole
 * sub-interval; subdivision cannot reduce a systematic bias, which is why this
 * term must not be absorbed by clearance_margin_m.
 */
static void consumer_power_coefficients(
    const MotionReferencePoint *a,
    const MotionReferencePoint *b,
    float h,
    unsigned axis,
    float c[6])
{
  float p0 = a->point.position_ned_m[axis];
  float v0 = a->point.velocity_ned_mps[axis];
  float a0 = a->point.acceleration_ned_mps2[axis];
  float p1 = b->point.position_ned_m[axis];
  float v1 = b->point.velocity_ned_mps[axis];
  float a1 = b->point.acceleration_ned_mps2[axis];
  float h2 = h * h;
  float h3 = h2 * h;
  float h4 = h3 * h;
  float h5 = h4 * h;
  float d = ((p1 - p0) - h * v0) - ((0.5f * h) * h) * a0;
  float dv = (v1 - v0) - h * a0;
  float da = a1 - a0;

  c[0] = p0;
  c[1] = v0;
  c[2] = 0.5f * a0;
  c[3] = 10 * d / h3 - 4 * dv / h2 + da / (2 * h);
  c[4] = -15 * d / h4 + 7 * dv / h3 - da / h2;
  c[5] = 6 * d / h5 - 3 * dv / h4 + da / (2 * h3);
#if defined(FLIGHT_M5_SAFETY_TEST_API)
  for (unsigned i = 0u; i < 6u; ++i)
    M5_TRACE_NONFINITE(c[i], M5_TEST_CONSUMER_REJECT_COEFFICIENTS);
#endif
}

/*
 * Rounding counts for the separate multiply/add form of the consumer
 * (control_reference.c), counted operation by operation:
 *
 *   position     ((((c5*s + c4)*s + c3)*s + c2)*s + c1)*s + c0
 *                5 multiplies + 5 adds                              = 10
 *   velocity     (((5c5*s + 4c4)*s + 3c3)*s + 2c2)*s + c1
 *                4 coefficient multiplies + 4 s multiplies + 4 adds = 12
 *   acceleration ((20c5*s + 12c4)*s + 6c3)*s + 2c2
 *                4 coefficient multiplies + 3 s multiplies + 3 adds = 10
 *
 * If the production toolchain ever contracts these into fused multiply-add the
 * count drops (FMA is strictly more accurate), but the computed coefficients
 * would differ from consumer_power_coefficients(); tests/guidance/
 * check_consumer_fp_contraction.py fails in that case and forces a review.
 */
#define M5_CONSUMER_POSITION_ROUNDINGS 10u
#define M5_CONSUMER_VELOCITY_ROUNDINGS 12u
#define M5_CONSUMER_ACCELERATION_ROUNDINGS 10u

/*
 * Bound |consumer_polynomial(s) - exact_polynomial(s)| for s in [0, s_max].
 *
 * `exact` is an outward interval enclosure of the true real coefficients
 * produced by the validator's interval arithmetic, never a binary64 nominal
 * value. The bound is
 *
 *   sum_i max(|cf_i - c_low_i|, |cf_i - c_high_i|) * s_max^i
 *     + gamma_n * sum_i |cf_i| * s_max^i
 *
 * with gamma_n = n*u / (1 - n*u), u = 2^-24. Each binary32 operation is
 * bounded by u*|exact| + 2^-149, so the gamma term and the separately
 * propagated subnormal floor cover normal and subnormal roundings. Before
 * applying that model, every actual Horner multiply and add is checked against
 * the binary32 round-to-nearest overflow midpoint. Inputs outside the model
 * are rejected, not guessed.
 */
static double consumer_eval_error_bound(
    const float consumer[6],
    const M5SafetyInterval exact[6],
    unsigned degree,
    unsigned rounding_count,
    double s_eval_max)
{
  const double unit = 0x1p-24; /* binary32 unit roundoff, round-to-nearest */
  double coefficient_term = 0.0;
  double magnitude_term = 0.0;
  double power = 1.0;
  double gamma, result;
  unsigned i;

  if (!(s_eval_max >= 0.0) || !isfinite(s_eval_max) || rounding_count == 0u)
    return INFINITY;
  for (i = 0u; i <= degree; ++i) {
    double cf = (double)consumer[i];
    double low_distance, high_distance, difference, magnitude;
    if (!isfinite(cf) || exact[i].valid == 0u) return INFINITY;
    low_distance = fabs(cf - exact[i].low);
    high_distance = fabs(cf - exact[i].high);
    difference = fmax(low_distance, high_distance);
    difference = difference > 0.0 ? nextafter(difference, INFINITY) : 0.0;
    magnitude = fabs(cf);
    coefficient_term = add_upper_nonnegative(coefficient_term,
        multiply_upper_nonnegative(difference, power));
    magnitude_term = add_upper_nonnegative(magnitude_term,
        multiply_upper_nonnegative(magnitude, power));
    power = multiply_upper_nonnegative(power, s_eval_max);
  }
  if (!isfinite(coefficient_term) || !isfinite(magnitude_term))
    return INFINITY;
  {
    /*
     * Bound the rounded Horner chain, not only its ideal-real recurrence.
     *
     * For IEEE binary32 round-to-nearest/ties-to-even, the overflow midpoint
     * is 2^128 - 2^103. At the midpoint the even result is 2^128, which
     * overflows to infinity; therefore each exact product and sum upper bound
     * must be strictly below it.
     *
     * R_n = |c_n|
     * P_k = s_eval_max * R_(k+1)
     * R_k = |c_k| + round32_upper(P_k)
     *
     * round32_upper bounds the actual rounded product/sum and caps at FLT_MAX
     * only after the corresponding exact operation is proven finite. This
     * propagates each prior binary32 rounding into the next stage.
     */
    const double overflow_midpoint = 0x1p128 - 0x1p103;
    double rounded_h = fabs((double)consumer[degree]);
    if (!isfinite(rounded_h)) return INFINITY;
    for (i = degree; i-- > 0u;) {
      double product_exact = multiply_upper_nonnegative(rounded_h, s_eval_max);
      double product_rounded;
      double sum_exact;
      if (!isfinite(product_exact) || product_exact >= overflow_midpoint) {
        M5_TRACE_HORNER(degree);
        return INFINITY;
      }
      product_rounded =
          binary32_rounded_upper_nonnegative(product_exact);
      if (!isfinite(product_rounded)) {
        M5_TRACE_HORNER(degree);
        return INFINITY;
      }
      sum_exact = add_upper_nonnegative(
          fabs((double)consumer[i]), product_rounded);
      if (!isfinite(sum_exact) || sum_exact >= overflow_midpoint) {
        M5_TRACE_HORNER(degree);
        return INFINITY;
      }
      rounded_h = binary32_rounded_upper_nonnegative(sum_exact);
      if (!isfinite(rounded_h)) {
        M5_TRACE_HORNER(degree);
        return INFINITY;
      }
    }
  }
  if (magnitude_term > 0.0) {
    /* Overflow risk in the Horner chain: the gamma model no longer applies. */
    if (magnitude_term > (double)FLT_MAX * 0.0625) return INFINITY;
    /* Possible cancellation into the subnormal range: reject conservatively. */
    if (magnitude_term < (double)FLT_MIN * 1024.0) return INFINITY;
  }
  {
    /* gamma_n = nu/(1-nu) is increasing for 0 <= nu < 1. Round nu upward,
       the denominator downward, and the quotient upward. n is exact in
       binary64 for the supported counts (10 or 12). */
    double nu = (double)rounding_count * unit;
    double denominator;
    nu = nextafter(nu, INFINITY);
    denominator = nextafter(1.0 - nu, -INFINITY);
    if (!(denominator > 0.0) || !isfinite(denominator)) return INFINITY;
    gamma = nu / denominator;
    gamma = nextafter(gamma, INFINITY);
  }
  result = add_upper_nonnegative(coefficient_term,
      multiply_upper_nonnegative(magnitude_term, gamma));
  /* Absolute floor. A rounding at Horner step k is multiplied by s for every
     later step, so its contribution is at most 2^-149 * s^(n-k); the worst
     case is bounded by n * 2^-149 * max(1, s_eval_max)^n. For s_eval_max <= 1 this is
     exactly the R5 term n * 2^-149 (no amplification), which is why the
     production validator additionally requires s_eval_max <= 1. */
  {
    double scale = s_eval_max > 1.0 ? s_eval_max : 1.0; /* max(1, s_eval_max) */
    double amplification = 1.0;
    unsigned step;
    for (step = 0u; step < rounding_count; ++step)
      amplification = multiply_upper_nonnegative(amplification, scale);
    result = add_upper_nonnegative(result,
        multiply_upper_nonnegative((double)rounding_count * 0x1p-149,
                                   amplification));
  }
  return result;
}

/* |d^k/dt^k| upper bound of the power-basis polynomial on [0, s_max]. */
static double polynomial_derivative_magnitude(
    const float c[6], unsigned degree, unsigned order, double s_max)
{
  double total = 0.0;
  unsigned i, k;

  if (!(s_max >= 0.0) || !isfinite(s_max)) return INFINITY;
  for (i = order; i <= degree; ++i) {
    double factor = 1.0;
    double power = 1.0;
    for (k = 0u; k < order; ++k) factor *= (double)(i - k);
    for (k = 0u; k + order < i; ++k)
      power = multiply_upper_nonnegative(power, s_max);
    total = add_upper_nonnegative(total,
        multiply_upper_nonnegative(fabs((double)c[i]) * factor, power));
  }
  return total;
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
  /* Store an outward physical span. The consumer polynomial is evaluated from
     these bounds, so a rounded-down end must not clip the gamma_2 tail. */
  curve->start_time_s = nextafter(t0, -INFINITY);
  curve->end_time_s = nextafter(t0 + dt, INFINITY);
  if (!isfinite(curve->start_time_s) || !isfinite(curve->end_time_s))
    return 0u;
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

      /* MR-009: bound the binary32 consumer evaluation of this axis against
         the interval coefficients c_i computed above (never the binary64
         nominal c[]).

         Timing model. The consumer computes
             h   = RN(step_us   * 1e-6f)
             s_c = RN(offset_us * 1e-6f)
         with both integers exactly representable in binary32. With
         c = RN(1e-6) = 1e-6(1+d0), |d0| <= u, this is
             h   = step   * c * (1+e1),  |e1| <= u
             s_c = offset * c * (1+e2),  |e2| <= u.
         Using s_exact = h * (offset/step) = offset * c * (1+e1) absorbs the
         first error, so s_c/s_exact = (1+e2)/(1+e1) and therefore
             |s_c - s_exact| <= gamma_2 * s_exact,  gamma_2 = 2u/(1-2u).
         With s_exact <= s_max = h*u1 every consumer time lies in
             [0, s_eval_max],  s_eval_max = s_max + gamma_2*s_max.
         All value and derivative bounds are evaluated on that enlarged
         interval; the residual shift contributes delta_s = gamma_2*s_max times
         the derivative bound. The old R5 form (2u+u^2)*s <= 2u*s_max was wrong
         because 2u+u^2 > 2u. */
      {
        float cf[6], vf[6], af[6];
        M5SafetyInterval vc[6], ac[6];
        double s_max = multiply_upper_nonnegative(h, (double)u1);
        double gamma2 = gamma2_upper();
        double delta_s, s_eval_max;

        delta_s = multiply_upper_nonnegative(gamma2, s_max);
        s_eval_max = add_upper_nonnegative(s_max, delta_s);
        if (!isfinite(s_eval_max) || !(s_eval_max > 0.0)) return 0u;
        /* consumer_eval_error_bound() carries the general max(1,s)^n
           amplification term, so the subnormal floor is valid for any finite
           s_eval_max produced here (the test API may pass durations above one
           second). The product contract still bounds the preview step: the
           static assertion below keeps PRODUCT_PREVIEW_STEP_US <= 1 s, and
           validate_reference_shape requires every preview offset to be an
           exact multiple of that step, so the validated curve always has
           h = 0.02 s and s_eval_max ~ 0.0200001. */
        if (!isfinite(s_eval_max)) return 0u;

        consumer_power_coefficients(a, b, (float)h, axis, cf);
        /* Stop at coefficient construction failure; do not contaminate later
           derivative/Horner branch traces with non-finite coefficients. */
        for (k = 0u; k < 6u; ++k)
          if (!isfinite(cf[k])) return 0u;
        curve->jerk_power[axis][0] = 6.0 * (double)cf[3];
        curve->jerk_power[axis][1] = 24.0 * (double)cf[4];
        curve->jerk_power[axis][2] = 60.0 * (double)cf[5];
        for (k = 0u; k < 6u; ++k) {
          vf[k] = 0.0f;
          af[k] = 0.0f;
          vc[k] = interval_exact(0.0);
          ac[k] = interval_exact(0.0);
        }
        for (k = 0u; k < 5u; ++k) {
          vf[k] = (float)(k + 1u) * cf[k + 1u];
          M5_TRACE_NONFINITE(vf[k], M5_TEST_CONSUMER_REJECT_VELOCITY_SCALING);
          if (!isfinite(vf[k])) return 0u;
          vc[k] = interval_scale(c_i[k + 1u], (double)(k + 1u));
        }
        for (k = 0u; k < 4u; ++k) {
          af[k] = (float)((k + 1u) * (k + 2u)) * cf[k + 2u];
          M5_TRACE_NONFINITE(af[k], M5_TEST_CONSUMER_REJECT_ACCELERATION_SCALING);
          if (!isfinite(af[k])) return 0u;
          ac[k] = interval_scale(c_i[k + 2u],
                                 (double)((k + 1u) * (k + 2u)));
        }
        /* The time-shift derivative belongs to each consumer output polynomial.
           vf/af contain the consumer's rounded coefficient scaling; differentiating
           cf twice/three times would silently omit that rounding. */
        curve->eval_position_error[axis] = add_upper_nonnegative(
            consumer_eval_error_bound(cf, c_i, 5u,
                M5_CONSUMER_POSITION_ROUNDINGS, s_eval_max),
            multiply_upper_nonnegative(delta_s,
                polynomial_derivative_magnitude(cf, 5u, 1u, s_eval_max)));
        curve->eval_velocity_error[axis] = add_upper_nonnegative(
            consumer_eval_error_bound(vf, vc, 4u,
                M5_CONSUMER_VELOCITY_ROUNDINGS, s_eval_max),
            multiply_upper_nonnegative(delta_s,
                polynomial_derivative_magnitude(vf, 4u, 1u, s_eval_max)));
        curve->eval_acceleration_error[axis] = add_upper_nonnegative(
            consumer_eval_error_bound(af, ac, 3u,
                M5_CONSUMER_ACCELERATION_ROUNDINGS, s_eval_max),
            multiply_upper_nonnegative(delta_s,
                polynomial_derivative_magnitude(af, 3u, 1u, s_eval_max)));
        if (!isfinite(curve->eval_position_error[axis]) ||
            !isfinite(curve->eval_velocity_error[axis]) ||
            !isfinite(curve->eval_acceleration_error[axis])) return 0u;
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

    /* MR-009: the consumed binary32 curve differs from this binary64 curve.
       Widen the position hull, the velocity/acceleration bounds and the
       rounding radii by the consumer evaluation envelope. */
    position_low = nextafter(
        position_low - curve->eval_position_error[axis], -INFINITY);
    position_high = nextafter(
        position_high + curve->eval_position_error[axis], INFINITY);
    velocity_abs = add_upper_nonnegative(velocity_abs,
        curve->eval_velocity_error[axis]);
    acceleration_abs = add_upper_nonnegative(acceleration_abs,
        curve->eval_acceleration_error[axis]);

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
          abs_difference_upper((double)bounds->position[axis],
                                position_mid_interval.low),
          abs_difference_upper(position_mid_interval.high,
                                (double)bounds->position[axis]));
      double velocity_error = fmax(
          abs_difference_upper((double)bounds->velocity[axis],
                                velocity_mid_interval.low),
          abs_difference_upper(velocity_mid_interval.high,
                                (double)bounds->velocity[axis]));
      velocity_mid_abs = add_upper_nonnegative(
          interval_abs_max(velocity_mid_interval),
          curve->eval_velocity_error[axis]);
      position_rounding2 = sum_square_upper(position_rounding2, position_error);
      velocity_rounding2 = sum_square_upper(velocity_rounding2, velocity_error);
      position_rounding2 = sum_square_upper(position_rounding2,
          curve->eval_position_error[axis]);
      velocity_rounding2 = sum_square_upper(velocity_rounding2,
          curve->eval_velocity_error[axis]);
      velocity_mid2 = sum_square_upper(velocity_mid2, velocity_mid_abs);
    }

    /* Third derivative. A degree-5 Bernstein position has a degree-2 Bernstein
       jerk with control points
           60/d^3 * (B[k+3] - 3B[k+2] + 3B[k+1] - B[k]).
       Three conservative bounds are combined:
         1. the Bernstein convex hull on the original [0,1] interval;
         2. the same polynomial converted to power basis and evaluated with
            interval Horner on the consumer tail [1, 1+gamma_2*ratio],
            ratio = end_time/duration;
         3. the consumer's own binary32 coefficients in physical seconds.
       All three use outward interval arithmetic; taking their maximum covers
       the consumed reference. A failed interval is a NUMERIC_FAILURE. */
    {
      M5SafetyInterval duration_i, duration_cubed, scale;
      M5SafetyInterval jerk_control[3];
      M5SafetyInterval u_i, tail_value;
      M5SafetyInterval s_i, consumer_value;
      M5SafetyInterval power0, power1, power2;
      double ratio, u_tail, s_tail, gamma2;
      double bernstein_abs = 0.0;
      double tail_abs = 0.0;
      double consumer_abs = 0.0;
      double jerk_abs;
      unsigned k;

      duration_i = interval_bounds(
          nextafter(duration_s, -INFINITY),
          nextafter(duration_s, INFINITY));
      duration_cubed = interval_mul(
          interval_mul(duration_i, duration_i), duration_i);
      scale = interval_div(interval_exact(60.0), duration_cubed);
      if (duration_i.valid == 0u || duration_cubed.valid == 0u ||
          scale.valid == 0u) return 0u;

      for (k = 0u; k < 3u; ++k) {
        M5SafetyInterval delta3 = interval_sub(
            interval_add(position_control[k + 3u],
                         interval_scale(position_control[k + 1u], 3.0)),
            interval_add(interval_scale(position_control[k + 2u], 3.0),
                         position_control[k]));
        jerk_control[k] = interval_mul(scale, delta3);
        if (jerk_control[k].valid == 0u) return 0u;
        bernstein_abs = fmax(bernstein_abs,
                             interval_abs_max(jerk_control[k]));
      }

      /* Bernstein-to-power: b(u) = b0 + 2(b1-b0)u + (b0-2b1+b2)u^2. */
      power0 = jerk_control[0];
      power1 = interval_scale(
          interval_sub(jerk_control[1], jerk_control[0]), 2.0);
      power2 = interval_add(
          interval_sub(jerk_control[0],
                       interval_scale(jerk_control[1], 2.0)),
          jerk_control[2]);
      if (power1.valid == 0u || power2.valid == 0u) return 0u;

      gamma2 = gamma2_upper();
      /* The sub-interval ends at normalized coordinate 1; the consumer may
         extend by gamma_2*s_end, i.e. by gamma_2*(end_time/duration) in the
         normalized coordinate. */
      ratio = curve->end_time_s / duration_s;
      ratio = nextafter(ratio, INFINITY);
      if (!isfinite(ratio) || ratio <= 0.0) return 0u;
      u_tail = add_upper_nonnegative(1.0,
          multiply_upper_nonnegative(gamma2, ratio));
      if (!isfinite(u_tail) || u_tail < 1.0) return 0u;
      if (u_tail > 1.0) {
        u_i = interval_bounds(1.0, u_tail);
        if (u_i.valid == 0u) return 0u;
        tail_value = interval_add(
            interval_mul(power2, u_i), power1);
        tail_value = interval_add(
            interval_mul(tail_value, u_i), power0);
        if (tail_value.valid == 0u) return 0u;
        tail_abs = interval_abs_max(tail_value);
      }

      /* Consumer polynomial in physical seconds, using the exact binary32
         coefficients returned by consumer_power_coefficients(). */
      s_tail = add_upper_nonnegative(curve->end_time_s,
          multiply_upper_nonnegative(gamma2, curve->end_time_s));
      if (!isfinite(s_tail) || s_tail < curve->end_time_s) return 0u;
      s_i = interval_bounds(
          nextafter(curve->start_time_s, -INFINITY),
          nextafter(s_tail, INFINITY));
      if (s_i.valid == 0u) return 0u;
      consumer_value = interval_add(
          interval_mul(interval_exact(curve->jerk_power[axis][2]), s_i),
          interval_exact(curve->jerk_power[axis][1]));
      consumer_value = interval_add(
          interval_mul(consumer_value, s_i),
          interval_exact(curve->jerk_power[axis][0]));
      if (consumer_value.valid == 0u) return 0u;
      consumer_abs = interval_abs_max(consumer_value);

      bounds->jerk_bernstein_bound[axis] = round_up_float(bernstein_abs);
      bounds->jerk_tail_bound[axis] = round_up_float(tail_abs);
      bounds->jerk_consumer_bound[axis] = round_up_float(consumer_abs);
      jerk_abs = fmax(bernstein_abs, fmax(tail_abs, consumer_abs));
      bounds->jerk_axis_bound[axis] = round_up_float(jerk_abs);
      if (!isfinite(bounds->jerk_axis_bound[axis]) ||
          !isfinite(bounds->jerk_bernstein_bound[axis]) ||
          !isfinite(bounds->jerk_tail_bound[axis]) ||
          !isfinite(bounds->jerk_consumer_bound[axis])) return 0u;
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

  bounds->speed_bound = sqrt_upper_float(speed2);
  bounds->acceleration_bound = sqrt_upper_float(acceleration2);
  bounds->velocity_mid_norm = sqrt_upper_float(velocity_mid2);
  bounds->position_rounding_error_m = sqrt_upper_float(position_rounding2);
  bounds->velocity_rounding_error_mps = sqrt_upper_float(velocity_rounding2);
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
  double corrected_lower;

  if (point == NULL || center == NULL || distance_lower == NULL ||
      !finite3(point) || !finite3(center) ||
      !isfinite(half_extent) || half_extent < 0.0f ||
      !isfinite(point_rounding_error) || point_rounding_error < 0.0f) {
    return 0u;
  }

  for (axis = 0u; axis < 3u; ++axis) {
    double separation = fabs((double)point[axis] - (double)center[axis]);
    double delta;
    if (separation > 0.0) separation = nextafter(separation, -INFINITY);
    delta = nextafter(separation - (double)half_extent, -INFINITY);
    if (delta > 0.0) distance2 = sum_square_lower(distance2, delta);
  }
  geometric_lower = sqrt_lower_float(distance2);
  corrected_lower = nextafter(
      (double)geometric_lower - (double)point_rounding_error, -INFINITY);
  *distance_lower = fmaxf(0.0f, round_down_float(corrected_lower));
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
  double speed_sum, speed_sweep_upper, acceleration_sweep_upper;
  double acceleration_term, velocity_term;

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

  speed_sum = add_upper_nonnegative(
      add_upper_nonnegative(curve_speed_bound, obstacle_speed_bound),
      velocity_std_mps);
  speed_sweep_upper = multiply_upper_nonnegative(
      speed_sum, half_duration_s);
  acceleration_term = multiply_upper_nonnegative(
      multiply_upper_nonnegative(
          multiply_upper_nonnegative(0.5, relative_acceleration_bound),
          half_duration_s),
      half_duration_s);
  velocity_term = multiply_upper_nonnegative(
      velocity_std_mps, half_duration_s);
  acceleration_sweep_upper = add_upper_nonnegative(
      add_upper_nonnegative(
          multiply_upper_nonnegative(relative_speed_bound, half_duration_s),
          acceleration_term),
      velocity_term);
  speed_sweep = round_up_float(speed_sweep_upper);
  acceleration_sweep = round_up_float(acceleration_sweep_upper);
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
    float clearance, height_low, height_high;
    float min_height, max_height;
    result->geofence_checks++;
    /* MR-009: the consumed binary32 curve can sit up to
       position_rounding_error_m closer to the fence than this binary64 point,
       so the clearance is shrunk by that bound before any comparison. The
       vertical bounds already use position_low/high, which include the same
       envelope (and the axis-aligned norm is a conservative radial slack). */
    clearance = round_down_float((double)fence_clearance(
        f, curve->position[0], curve->position[1]) -
        M5_SAFETY_GEOMETRY_EPSILON_M -
        (double)curve->position_rounding_error_m);
    height_low = round_down_float((double)home_down - curve->position_high[2]);
    height_high = round_up_float((double)home_down - curve->position_low[2]);
    min_height = round_up_float((double)f->min_height_above_home_m +
        f->vertical_margin_m + inflation);
    max_height = round_down_float((double)f->max_height_above_home_m -
        f->vertical_margin_m - inflation);
    if (!isfinite(clearance) || !isfinite(height_low) || !isfinite(height_high) ||
        !isfinite(min_height) || !isfinite(max_height) ||
        min_height >= max_height) return M5_SAFETY_NUMERIC_FAILURE;

    if (clearance <= inflation || height_low < min_height ||
        height_high > max_height) return M5_SAFETY_GEOFENCE_VIOLATION;

    if (clearance <= round_up_float((double)inflation + swept_radius)) {
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

static uint8_t predict_dynamic_track_position(
    const float position_ned_m[3],
    const float velocity_ned_mps[3],
    M5SafetyInterval local_mid_offset_us,
    double local_mid_offset_us_nominal,
    double sample_offset_us,
    float predicted_position[3],
    float *position_error_m,
    double *time_from_sample_upper_s)
{
  const M5SafetyInterval seconds_per_microsecond = interval_bounds(
      nextafter(1.0e-6, -INFINITY),
      nextafter(1.0e-6, INFINITY));
  M5SafetyInterval delta_us, delta_s;
  double nominal_dt_s, position_error2 = 0.0;
  unsigned axis;

  if (position_ned_m == NULL || velocity_ned_mps == NULL ||
      predicted_position == NULL || position_error_m == NULL ||
      time_from_sample_upper_s == NULL ||
      local_mid_offset_us.valid == 0u ||
      !isfinite(local_mid_offset_us_nominal) ||
      !isfinite(sample_offset_us) ||
      !finite3(position_ned_m) || !finite3(velocity_ned_mps)) return 0u;

  delta_us = interval_sub(local_mid_offset_us,
                          interval_exact(sample_offset_us));
  delta_s = interval_mul(delta_us, seconds_per_microsecond);
  if (delta_us.valid == 0u || delta_s.valid == 0u) return 0u;

  nominal_dt_s =
      (local_mid_offset_us_nominal - sample_offset_us) * 1.0e-6;
  if (!isfinite(nominal_dt_s)) return 0u;

  *time_from_sample_upper_s = nextafter(
      fmax(fabs(delta_s.low), fabs(delta_s.high)), INFINITY);
  if (!isfinite(*time_from_sample_upper_s)) return 0u;

  for (axis = 0u; axis < 3u; ++axis) {
    M5SafetyInterval exact_position = interval_exact(
        (double)position_ned_m[axis]);
    M5SafetyInterval exact_velocity = interval_exact(
        (double)velocity_ned_mps[axis]);
    M5SafetyInterval displacement = interval_mul(
        exact_velocity, delta_s);
    M5SafetyInterval exact_prediction = interval_add(
        exact_position, displacement);
    double nominal_prediction =
        (double)position_ned_m[axis] +
        (double)velocity_ned_mps[axis] * nominal_dt_s;
    double error_low, error_high, error_bound;

    if (displacement.valid == 0u || exact_prediction.valid == 0u ||
        !isfinite(nominal_prediction)) return 0u;

    predicted_position[axis] = (float)nominal_prediction;
    if (!isfinite(predicted_position[axis])) return 0u;

    error_low = abs_difference_upper(
        (double)predicted_position[axis], exact_prediction.low);
    error_high = abs_difference_upper(
        (double)predicted_position[axis], exact_prediction.high);
    error_bound = fmax(error_low, error_high);
    position_error2 = sum_square_upper(position_error2, error_bound);
  }

  *position_error_m = sqrt_upper_float(position_error2);
  return (uint8_t)isfinite(*position_error_m);
}

static M5TrajectorySafetyStatus check_dynamic_tracks(
    const M5TrajectorySafetyInput *in,
    const M5SafetyCurveBounds *curve,
    float inflation,
    double local_mid_offset_us,
    M5SafetyInterval local_mid_offset_us_interval,
    M5TrajectorySafetyResult *result,
    M5SafetyCheck *check)
{
  const M5LocalMap *map = in->map;
  const MotionReference *r = in->candidate_reference;
  const double delay_s =
      microseconds_to_seconds_upper(in->response_delay_us);
  const float dynamic_accel = in->max_dynamic_obstacle_accel_mps2;
  unsigned i, axis;
  float half_dt = curve->half_duration_s;

  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i) {
    const M5DynamicTrack *t = &map->tracks[i];
    float obstacle[3];
    double relative_velocity[3], sample_offset_us;
    double time_from_sample_upper_s, uncertainty_horizon, uncertainty;
    float obstacle_speed, relative_speed, relative_accel;
    float combined_radius, obstacle_rounding_error;
    float relative_sweep, distance;
    uint64_t sample_delta_us;
    if (t->id == 0u) continue;
    if (++result->track_checks > M5_SAFETY_MAX_WORK_UNITS * M5_MAP_MAX_TRACKS)
      return M5_SAFETY_BUDGET_EXCEEDED;

    if (t->sample_us >= r->header.timestamp_sample_us) {
      sample_delta_us = t->sample_us - r->header.timestamp_sample_us;
      sample_offset_us = (double)sample_delta_us;
    } else {
      sample_delta_us = r->header.timestamp_sample_us - t->sample_us;
      sample_offset_us = -(double)sample_delta_us;
    }
    /* validate_tracks() constrains this relative epoch delta to less than
       one second, so uint64 -> binary64 is exact and safely signed. */
    if (sample_delta_us > UINT64_C(9007199254740991) ||
        !predict_dynamic_track_position(t->position_ned_m,
            t->velocity_ned_mps, local_mid_offset_us_interval,
            local_mid_offset_us, sample_offset_us, obstacle,
            &obstacle_rounding_error, &time_from_sample_upper_s)) {
      return M5_SAFETY_NUMERIC_FAILURE;
    }
    for (axis = 0u; axis < 3u; ++axis) {
      relative_velocity[axis] =
          (double)curve->velocity[axis] - (double)t->velocity_ned_mps[axis];
    }
    if (!finite3(obstacle) ||
        !isfinite(relative_velocity[0]) ||
        !isfinite(relative_velocity[1]) ||
        !isfinite(relative_velocity[2])) return M5_SAFETY_NUMERIC_FAILURE;

    obstacle_speed = norm3_upper(t->velocity_ned_mps);
    relative_speed = round_up_float(add_upper_nonnegative(
        (double)norm3_upper_double(relative_velocity[0],
            relative_velocity[1], relative_velocity[2]),
        add_upper_nonnegative(curve->velocity_rounding_error_mps,
                              t->velocity_std_mps)));
    relative_accel = round_up_float(add_upper_nonnegative(
        curve->acceleration_bound, dynamic_accel));
    uncertainty_horizon = add_upper_nonnegative(
        add_upper_nonnegative(time_from_sample_upper_s, half_dt), delay_s);
    uncertainty = multiply_upper_nonnegative(
        (double)map->uncertainty_inflation_factor,
        add_upper_nonnegative(
            add_upper_nonnegative((double)t->position_std_m,
                multiply_upper_nonnegative(uncertainty_horizon,
                                           t->velocity_std_mps)),
            multiply_upper_nonnegative(
                multiply_upper_nonnegative(
                    multiply_upper_nonnegative(0.5, dynamic_accel),
                    uncertainty_horizon),
                uncertainty_horizon)));
    /* The prediction helper encloses time conversion and position
       multiply/add; its radius covers the chosen float point to both interval
       endpoints. During response delay the obstacle's nominal velocity also
       moves it. */
    combined_radius = round_up_float(add_upper_nonnegative(
        add_upper_nonnegative(
            add_upper_nonnegative((double)t->radius_m, inflation),
            add_upper_nonnegative(uncertainty,
                multiply_upper_nonnegative(obstacle_speed, delay_s))),
        obstacle_rounding_error));
    if (!dynamic_relative_sweep_bound(curve->speed_bound, obstacle_speed,
            t->velocity_std_mps, relative_speed, relative_accel, half_dt,
            &relative_sweep)) return M5_SAFETY_NUMERIC_FAILURE;
    {
      double dx = (double)curve->position[0] - obstacle[0];
      double dy = (double)curve->position[1] - obstacle[1];
      double dz = (double)curve->position[2] - obstacle[2];
      distance = norm3_lower(dx, dy, dz);
      {
        double distance_lower = nextafter(
            (double)distance - (double)curve->position_rounding_error_m,
            -INFINITY);
        distance_lower = nextafter(
            distance_lower - (double)obstacle_rounding_error, -INFINITY);
        distance = fmaxf(0.0f, round_down_float(distance_lower));
      }
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
  double delay_s = microseconds_to_seconds_upper(in->response_delay_us);
  double uncertainty_horizon = add_upper_nonnegative(
      add_upper_nonnegative((double)horizon_mid_s,
                            (double)curve->half_duration_s),
      delay_s);
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
    double position_norm = (double)norm3_upper(position_std);
    double velocity_norm = (double)norm3_upper(velocity_std);
    double home_norm = (double)norm3_upper(home_std);
    double uncertainty = add_upper_nonnegative(
        add_upper_nonnegative(position_norm,
            multiply_upper_nonnegative(uncertainty_horizon, velocity_norm)),
        home_norm);
    double delay_motion = add_upper_nonnegative(
        multiply_upper_nonnegative(curve->speed_bound, delay_s),
        multiply_upper_nonnegative(
            multiply_upper_nonnegative(
                multiply_upper_nonnegative(0.5, curve->acceleration_bound),
                delay_s),
            delay_s));
    *inflation = round_up_float(add_upper_nonnegative(
        add_upper_nonnegative((double)map->vehicle_radius_m,
                              (double)map->clearance_margin_m),
        add_upper_nonnegative(
            multiply_upper_nonnegative(
                (double)map->uncertainty_inflation_factor, uncertainty),
            delay_motion)));
  }
  return (uint8_t)(isfinite(*inflation) && *inflation > 0.0f);
}

/*
 * MR-009 runtime assumption: every error bound assumes round-to-nearest with
 * IEEE subnormals enabled (no flush-to-zero). Compile flags alone do not prove
 * the runtime state, so the validator reads the actual control register.
 *
 * Cortex-M7: FPSCR.RMode[23:22] must be 00b, FZ[24] and DN[25] must be 0.
 * Host/SIL: the C99 rounding mode must be FE_TONEAREST.
 *
 * A different mode returns NUMERIC_FAILURE instead of silently using a bound
 * that no longer applies.
 */
static uint8_t floating_environment_is_supported(void)
{
#if defined(__arm__) && defined(__ARM_FP) && (__ARM_FP != 0)
  uint32_t fpscr = 0u;
  __asm__ volatile ("vmrs %0, fpscr" : "=r" (fpscr));
  return (uint8_t)(((fpscr >> 22) & 3u) == 0u &&  /* RMode = round to nearest */
                   ((fpscr >> 24) & 1u) == 0u &&  /* FZ: flush-to-zero off */
                   ((fpscr >> 25) & 1u) == 0u);   /* DN: default NaN off */
#elif (defined(__x86_64__) || defined(__i386__)) && defined(__SSE__)
  {
    /* Host/SIL x86: the MXCSR rounding control, flush-to-zero and
       denormals-are-zero bits must all match the error model. */
    unsigned csr = (unsigned)_mm_getcsr();
    return (uint8_t)(fegetround() == FE_TONEAREST &&
                     (csr & (3u << 13)) == 0u &&  /* RC = round to nearest */
                     (csr & (1u << 15)) == 0u &&  /* FTZ = 0 */
                     (csr & (1u << 6)) == 0u);    /* DAZ = 0 */
  }
#else
  return (uint8_t)(fegetround() == FE_TONEAREST);
#endif
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
  if (!floating_environment_is_supported())
    return reject(result, M5_SAFETY_NUMERIC_FAILURE,
                  M5_SAFETY_REASON_NUMERIC, 0u);
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
        /* JERK participates whenever the payload declares it (spec/02). A
           limit of zero is rejected by validate_envelope(), never skipped. */
        if ((e->valid_mask & PRODUCT_ENVELOPE_VALID_JERK) != 0u &&
            curve.jerk_axis_bound[axis] > e->max_jerk_ned_mps3[axis]) {
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

    {
      const double local_mid_fraction =
          0.5 * ((double)item.u0 + (double)item.u1);
      const double local_mid_offset_us =
          ((double)item.interval + local_mid_fraction) * (double)step_us;
      M5SafetyInterval local_mid_offset_us_interval = interval_mul(
          interval_add(interval_exact((double)item.interval),
                       interval_exact(local_mid_fraction)),
          interval_exact((double)step_us));
      status = check_dynamic_tracks(input, &curve, inflation,
          local_mid_offset_us, local_mid_offset_us_interval,
          result, &check);
    }
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
void m5_trajectory_safety_test_reset_consumer_rejection(void)
{
  consumer_rejection = M5_TEST_CONSUMER_REJECT_NONE;
  memset(consumer_rejection_counts, 0, sizeof(consumer_rejection_counts));
}

M5SafetyTestConsumerRejectStage m5_trajectory_safety_test_consumer_rejection(void)
{
  return consumer_rejection;
}

void m5_trajectory_safety_test_consumer_rejection_counts(
    uint32_t counts[M5_TEST_CONSUMER_REJECT_STAGE_COUNT])
{
  if (counts != NULL)
    memcpy(counts, consumer_rejection_counts, sizeof(consumer_rejection_counts));
}

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


uint8_t m5_trajectory_safety_test_curve_jerk(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float jerk_axis_bound[3])
{
  MotionReferencePoint start, end;
  M5SafetyBezier bezier;
  M5SafetyCurveBounds bounds;
  double local_duration;
  unsigned axis;

  if (start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      jerk_axis_bound == NULL ||
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
  for (axis = 0u; axis < 3u; ++axis)
    jerk_axis_bound[axis] = bounds.jerk_axis_bound[axis];
  return 1u;
}


void m5_trajectory_safety_test_jerk_status_values(
    uint8_t *test_ok, uint8_t *test_invalid, uint8_t *test_build,
    uint8_t *test_bounds, uint8_t *production_invalid,
    uint8_t *production_numeric)
{
  if (test_ok != NULL) *test_ok = M5_TEST_JERK_STATUS_OK;
  if (test_invalid != NULL) *test_invalid = M5_TEST_JERK_STATUS_INVALID_INPUT;
  if (test_build != NULL) *test_build = M5_TEST_JERK_STATUS_BUILD_NUMERIC;
  if (test_bounds != NULL) *test_bounds = M5_TEST_JERK_STATUS_BOUNDS_NUMERIC;
  if (production_invalid != NULL) *production_invalid = M5_SAFETY_INVALID;
  if (production_numeric != NULL) *production_numeric = M5_SAFETY_NUMERIC_FAILURE;
}

uint8_t m5_trajectory_safety_test_curve_jerk_components(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float bernstein_bound[3],
    float tail_bound[3],
    float consumer_bound[3],
    float total_bound[3], uint8_t *status)
{
  MotionReferencePoint start, end;
  M5SafetyBezier bezier;
  M5SafetyCurveBounds bounds;
  double local_duration;
  unsigned axis;

  if (start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      bernstein_bound == NULL || tail_bound == NULL ||
      consumer_bound == NULL || total_bound == NULL || status == NULL ||
      !finite3(start_position) || !finite3(start_velocity) ||
      !finite3(start_acceleration) || !finite3(end_position) ||
      !finite3(end_velocity) || !finite3(end_acceleration) ||
      !isfinite(duration_s) || duration_s <= 0.0f ||
      !isfinite(u0) || !isfinite(u1) || u0 < 0.0f ||
      u1 > 1.0f || u1 <= u0) {
    if (status != NULL) *status = M5_TEST_JERK_STATUS_INVALID_INPUT;
    return 0u;
  }

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
  *status = M5_TEST_JERK_STATUS_OK;
  if (!build_curve(&start, &end, (double)duration_s,
                   (float)u0, (float)u1, &bezier)) {
    *status = M5_TEST_JERK_STATUS_BUILD_NUMERIC;
    return 0u;
  }
  local_duration = (double)duration_s * ((double)u1 - (double)u0);
  if (!curve_bounds(&bezier, local_duration, &bounds)) {
    *status = M5_TEST_JERK_STATUS_BOUNDS_NUMERIC;
    return 0u;
  }
  for (axis = 0u; axis < 3u; ++axis) {
    bernstein_bound[axis] = bounds.jerk_bernstein_bound[axis];
    tail_bound[axis] = bounds.jerk_tail_bound[axis];
    consumer_bound[axis] = bounds.jerk_consumer_bound[axis];
    total_bound[axis] = bounds.jerk_axis_bound[axis];
  }
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
  double local_duration;
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
  local_duration = (double)duration_s * ((double)u1 - (double)u0);
  if (!(local_duration > 0.0) || !isfinite(local_duration)) return 0u;

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


uint8_t m5_trajectory_safety_test_curve_envelope(
    const float start_position[3],
    const float start_velocity[3],
    const float start_acceleration[3],
    const float end_position[3],
    const float end_velocity[3],
    const float end_acceleration[3],
    float duration_s,
    float u0,
    float u1,
    float *position_error_m,
    float *velocity_error_mps,
    float *acceleration_error_mps2)
{
  MotionReferencePoint start, end;
  M5SafetyBezier bezier;
  double position2 = 0.0, velocity2 = 0.0, acceleration2 = 0.0;
  unsigned axis;

  if (start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      position_error_m == NULL || velocity_error_mps == NULL ||
      acceleration_error_mps2 == NULL ||
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

  for (axis = 0u; axis < 3u; ++axis) {
    position2 = sum_square_upper(position2, bezier.eval_position_error[axis]);
    velocity2 = sum_square_upper(velocity2, bezier.eval_velocity_error[axis]);
    acceleration2 = sum_square_upper(acceleration2,
                                     bezier.eval_acceleration_error[axis]);
  }
  *position_error_m = sqrt_upper_float(position2);
  *velocity_error_mps = sqrt_upper_float(velocity2);
  *acceleration_error_mps2 = sqrt_upper_float(acceleration2);
  return (uint8_t)(isfinite(*position_error_m) &&
      isfinite(*velocity_error_mps) &&
      isfinite(*acceleration_error_mps2));
}


double m5_trajectory_safety_test_consumer_error_bound(
    const float consumer[6],
    unsigned degree,
    unsigned rounding_count,
    double s_max,
    const double exact_low[6],
    const double exact_high[6])
{
  M5SafetyInterval exact[6];
  unsigned i;
  if (consumer == NULL || exact_low == NULL || exact_high == NULL ||
      degree > 5u) return INFINITY;
  for (i = 0u; i < 6u; ++i) {
    exact[i].low = exact_low[i];
    exact[i].high = exact_high[i];
    exact[i].valid = (uint8_t)(isfinite(exact_low[i]) && isfinite(exact_high[i]) &&
                               exact_low[i] <= exact_high[i]);
  }
  return consumer_eval_error_bound(consumer, exact, degree, rounding_count, s_max);
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

float m5_trajectory_safety_test_norm_upper_bound(
    float x, float y, float z)
{
  const float v[3] = {x, y, z};
  return norm3_upper(v);
}

uint8_t m5_trajectory_safety_test_predict_dynamic_position(
    const float position_ned_m[3],
    const float velocity_ned_mps[3],
    double local_mid_offset_us,
    int64_t sample_offset_us,
    float predicted_position[3],
    float *position_error_m)
{
  M5SafetyInterval local_mid_interval;
  double time_from_sample_upper_s;
  if (!isfinite(local_mid_offset_us) ||
      sample_offset_us > INT64_C(9007199254740991) ||
      sample_offset_us < -INT64_C(9007199254740991)) return 0u;
  local_mid_interval = interval_bounds(
      nextafter(local_mid_offset_us, -INFINITY),
      nextafter(local_mid_offset_us, INFINITY));
  return predict_dynamic_track_position(position_ned_m, velocity_ned_mps,
      local_mid_interval, local_mid_offset_us, (double)sample_offset_us,
      predicted_position, position_error_m, &time_from_sample_upper_s);
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


uint8_t m5_trajectory_safety_test_interval_op(
    uint8_t operation,
    double a_low, double a_high,
    double b_low, double b_high,
    double *result_low, double *result_high)
{
  M5SafetyInterval a = interval_bounds(a_low, a_high);
  M5SafetyInterval b = interval_bounds(b_low, b_high);
  M5SafetyInterval result;

  if (result_low == NULL || result_high == NULL ||
      a.valid == 0u || b.valid == 0u) return 0u;
  switch (operation) {
    case 0u: result = interval_add(a, b); break;
    case 1u: result = interval_sub(a, b); break;
    case 2u: result = interval_mul(a, b); break;
    case 3u: result = interval_div(a, b); break;
    default: return 0u;
  }
  if (result.valid == 0u) return 0u;
  *result_low = result.low;
  *result_high = result.high;
  return 1u;
}

/* Declared in control_reference.c / controller_internal.h; this unit never
   includes the controller-private header, so the prototype is repeated here.
   Both translation units are linked into the same Host test binary. */
extern Status control_sample_reference(const MotionReference *ref,
                                       uint64_t time,
                                       ProductMotionPoint *out);
extern float control_reference_convert_local_time(uint64_t local_us,
                                                  float seconds_per_us);
extern float control_reference_convert_local_time_ex(uint64_t local_us,
                                                     float seconds_per_us,
                                                     double additive_seconds);
extern uint8_t control_reference_time_within_gamma2(float local_seconds,
                                                    uint64_t local_us);

uint8_t m5_trajectory_safety_test_time_conversion(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint8_t mode,
    float *seconds, uint8_t *within_gamma2, double *true_seconds,
    double *actual_error)
{
  uint64_t local_us;
  uint64_t true_local_us;
  float scale = 1e-6f;
  double additive_seconds = 0.0;

  if (seconds == NULL || within_gamma2 == NULL || true_seconds == NULL ||
      actual_error == NULL || interval >= PRODUCT_PREVIEW_COUNT ||
      (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
       step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) ||
      offset_us > step_us || mode > 3u) return 0u;
  true_local_us = offset_us;
  local_us = offset_us;
  *true_seconds = (double)true_local_us / 1000000.0;
  if (mode == 1u) {
    local_us = (uint64_t)interval * step_us + offset_us;
  } else if (mode == 2u) {
    /* Exact additive seconds, not an integer-microsecond substitute. */
    additive_seconds = 4.0 * gamma2_upper() * *true_seconds;
  } else if (mode == 3u) {
    scale = 1e-6f * (float)step_us / (float)(step_us + 1000u);
  }
  *seconds = control_reference_convert_local_time_ex(
      local_us, scale, additive_seconds);
  *within_gamma2 = control_reference_time_within_gamma2(*seconds,
                                                        true_local_us);
  *actual_error = fabs((double)*seconds - *true_seconds);
  return (uint8_t)(isfinite(*seconds) && (*within_gamma2 <= 1u) &&
                   isfinite(*actual_error));
}

uint8_t m5_trajectory_safety_test_consumer_time(
    uint32_t step_us, uint32_t offset_us, float *seconds)
{
  MotionReference reference;
  ProductMotionPoint sampled;
  unsigned i, axis;
  const uint64_t base_us = 1000000u;

  if (seconds == NULL) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  if (offset_us > step_us) return 0u;

  memset(&reference, 0, sizeof(reference));
  reference.header.timestamp_sample_us = base_us;
  reference.point_count = PRODUCT_PREVIEW_COUNT;
  reference.anchor.offset_us = 0u;
  reference.anchor.valid_mask = 0x1Fu;
  reference.anchor.point.valid_mask = 0x1Fu;
  for (axis = 0u; axis < 3u; ++axis) {
    reference.anchor.point.position_ned_m[axis] = 0.0f;
    reference.anchor.point.velocity_ned_mps[axis] = axis == 0u ? 1.0f : 0.0f;
    reference.anchor.point.acceleration_ned_mps2[axis] = 0.0f;
  }
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    MotionReferencePoint *point = &reference.points[i];
    point->offset_us = (i + 1u) * step_us;
    point->valid_mask = 0x1Fu;
    point->point.valid_mask = 0x1Fu;
    for (axis = 0u; axis < 3u; ++axis) {
      /* Consistent linear motion: p_i = i*h with v = 1 makes d = dv = da = 0,
         so the consumer polynomial is exactly p(s) = s. */
      point->point.position_ned_m[axis] =
          axis == 0u ? (float)(i + 1u) * ((float)step_us * 1e-6f) : 0.0f;
      point->point.velocity_ned_mps[axis] = axis == 0u ? 1.0f : 0.0f;
      point->point.acceleration_ned_mps2[axis] = 0.0f;
    }
  }
  if (control_sample_reference(&reference, base_us + offset_us, &sampled) !=
      STATUS_OK) return 0u;
  *seconds = sampled.position_ned_m[0];
  return (uint8_t)isfinite(*seconds);
}

static void safety_test_fill_reference(MotionReference *reference,
                                       uint32_t step_us, uint64_t base_us)
{
  unsigned i, axis;
  memset(reference, 0, sizeof(*reference));
  reference->header.timestamp_sample_us = base_us;
  reference->point_count = PRODUCT_PREVIEW_COUNT;
  reference->anchor.offset_us = 0u;
  reference->anchor.valid_mask = 0x1Fu;
  reference->anchor.point.valid_mask = 0x1Fu;
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    reference->points[i].offset_us = (i + 1u) * step_us;
    reference->points[i].valid_mask = 0x1Fu;
    reference->points[i].point.valid_mask = 0x1Fu;
  }
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    for (axis = 0u; axis < 3u; ++axis) {
      reference->points[i].point.position_ned_m[axis] = 0.0f;
      reference->points[i].point.velocity_ned_mps[axis] = 0.0f;
      reference->points[i].point.acceleration_ned_mps2[axis] = 0.0f;
    }
  }
}

static void safety_test_set_state(MotionReferencePoint *point,
                                  const float position[3],
                                  const float velocity[3],
                                  const float acceleration[3])
{
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    point->point.position_ned_m[axis] = position[axis];
    point->point.velocity_ned_mps[axis] = velocity[axis];
    point->point.acceleration_ned_mps2[axis] = acceleration[axis];
  }
}

uint8_t m5_trajectory_safety_test_consumer_interval_sample(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float position[3], float *seconds)
{
  MotionReference reference, linear;
  ProductMotionPoint sampled;
  MotionReferencePoint *a;
  MotionReferencePoint *b;
  uint64_t time;
  unsigned axis;

  if (position == NULL || seconds == NULL || start_position == NULL ||
      start_velocity == NULL || start_acceleration == NULL ||
      end_position == NULL || end_velocity == NULL ||
      end_acceleration == NULL) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  if (interval >= PRODUCT_PREVIEW_COUNT || offset_us > step_us) return 0u;

  safety_test_fill_reference(&reference, step_us, base_us);
  a = interval == 0u ? &reference.anchor : &reference.points[interval - 1u];
  b = &reference.points[interval];
  safety_test_set_state(a, start_position, start_velocity, start_acceleration);
  safety_test_set_state(b, end_position, end_velocity, end_acceleration);
  time = base_us + (uint64_t)interval * step_us + offset_us;
  if (control_sample_reference(&reference, time, &sampled) != STATUS_OK) return 0u;
  for (axis = 0u; axis < 3u; ++axis) position[axis] = sampled.position_ned_m[axis];

  /* Companion unit-velocity reference: its x output is exactly s_c. */
  {
    float zero[3] = {0.0f, 0.0f, 0.0f};
    float unit[3] = {1.0f, 0.0f, 0.0f};
    float at_h[3] = {(float)step_us * 1e-6f, 0.0f, 0.0f};
    safety_test_fill_reference(&linear, step_us, base_us);
    safety_test_set_state(&linear.anchor, zero, unit, zero);
    for (axis = 0u; axis < PRODUCT_PREVIEW_COUNT; ++axis) {
      at_h[0] = (float)(axis + 1u) * ((float)step_us * 1e-6f);
      safety_test_set_state(&linear.points[axis], at_h, unit, zero);
    }
    if (control_sample_reference(&linear, time, &sampled) != STATUS_OK) return 0u;
    *seconds = sampled.position_ned_m[0];
  }
  return (uint8_t)(isfinite(*seconds) && isfinite(position[0]) &&
                   isfinite(position[1]) && isfinite(position[2]));
}

extern Status control_reference_locate_segment(const MotionReference *ref,
                                               uint64_t time,
                                               const MotionReferencePoint **a,
                                               const MotionReferencePoint **b,
                                               float *local_seconds,
                                               uint32_t *step_us_out);

uint8_t m5_trajectory_safety_test_consumer_local_seconds(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    float *seconds)
{
  MotionReference reference;
  const MotionReferencePoint *a = NULL;
  const MotionReferencePoint *b = NULL;
  uint32_t measured_step = 0u;
  float local_seconds = 0.0f;
  uint64_t time;

  if (seconds == NULL || interval >= PRODUCT_PREVIEW_COUNT || offset_us > step_us)
    return 0u;
  safety_test_fill_reference(&reference, step_us, base_us);
  time = base_us + (uint64_t)interval * step_us + offset_us;
  if (control_reference_locate_segment(&reference, time, &a, &b,
                                       &local_seconds, &measured_step) != STATUS_OK)
    return 0u;
  if (measured_step != step_us) return 0u;
  *seconds = local_seconds;
  return (uint8_t)isfinite(*seconds);
}

extern void control_reference_hermite_coefficients(
    const float p0[3], const float v0[3], const float a0[3], const float p1[3],
    const float v1[3], const float a1[3], float h, float c[3][6]);

uint8_t m5_trajectory_safety_test_consumer_coefficients(
    uint32_t step_us, uint32_t interval, const float start_position[3],
    const float start_velocity[3], const float start_acceleration[3],
    const float end_position[3], const float end_velocity[3],
    const float end_acceleration[3], float coefficients[18])
{
  float c[3][6];
  unsigned axis, k;

  if (coefficients == NULL || start_position == NULL || start_velocity == NULL ||
      start_acceleration == NULL || end_position == NULL ||
      end_velocity == NULL || end_acceleration == NULL) return 0u;
  if (interval >= PRODUCT_PREVIEW_COUNT) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  control_reference_hermite_coefficients(start_position, start_velocity,
                                         start_acceleration, end_position,
                                         end_velocity, end_acceleration,
                                         (float)step_us * 1e-6f, c);
  for (axis = 0u; axis < 3u; ++axis)
    for (k = 0u; k < 6u; ++k)
      coefficients[axis * 6u + k] = c[axis][k];
  return 1u;
}

uint8_t m5_trajectory_safety_test_consumer_full_sample(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float position[3], float velocity[3], float acceleration[3], float *seconds)
{
  MotionReference reference;
  ProductMotionPoint sampled;
  const MotionReferencePoint *a = NULL;
  const MotionReferencePoint *b = NULL;
  uint32_t measured_step = 0u;
  float local_seconds = 0.0f;
  uint64_t time;
  unsigned axis;

  if (position == NULL || velocity == NULL || acceleration == NULL ||
      seconds == NULL) return 0u;
  if (interval >= PRODUCT_PREVIEW_COUNT || offset_us > step_us) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  safety_test_fill_reference(&reference, step_us, base_us);
  a = interval == 0u ? &reference.anchor : &reference.points[interval - 1u];
  b = &reference.points[interval];
  {
    MotionReferencePoint *start_point = interval == 0u
        ? &reference.anchor : &reference.points[interval - 1u];
    safety_test_set_state(start_point, start_position, start_velocity,
                          start_acceleration);
  }
  safety_test_set_state(&reference.points[interval], end_position, end_velocity,
                        end_acceleration);
  time = base_us + (uint64_t)interval * step_us + offset_us;
  if (control_sample_reference(&reference, time, &sampled) != STATUS_OK) return 0u;
  if (control_reference_locate_segment(&reference, time, &a, &b, &local_seconds,
                                       &measured_step) != STATUS_OK) return 0u;
  if (measured_step != step_us) return 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    position[axis] = sampled.position_ned_m[axis];
    velocity[axis] = sampled.velocity_ned_mps[axis];
    acceleration[axis] = sampled.acceleration_ned_mps2[axis];
  }
  *seconds = local_seconds;
  return (uint8_t)(isfinite(local_seconds) && isfinite(position[0]) &&
                   isfinite(velocity[0]) && isfinite(acceleration[0]));
}

uint8_t m5_trajectory_safety_test_active_time(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    uint32_t *active_interval, uint64_t *local_us, uint64_t *global_us,
    float *local_seconds)
{
  MotionReference reference;
  const MotionReferencePoint *a = NULL;
  const MotionReferencePoint *b = NULL;
  uint32_t measured_step = 0u;
  uint64_t time;
  uint64_t global;
  float seconds = 0.0f;

  if (active_interval == NULL || local_us == NULL || global_us == NULL ||
      local_seconds == NULL) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  if (interval >= PRODUCT_PREVIEW_COUNT || offset_us > step_us) return 0u;
  safety_test_fill_reference(&reference, step_us, base_us);
  global = (uint64_t)interval * step_us + offset_us;
  time = base_us + global;
  if (control_reference_locate_segment(&reference, time, &a, &b, &seconds,
                                       &measured_step) != STATUS_OK)
    return 0u;
  if (measured_step != step_us || a == NULL) return 0u;
  *active_interval = (a == &reference.anchor)
                         ? 0u
                         : (uint32_t)((a - reference.points) + 1);
  *global_us = global;
  *local_us = global - a->offset_us;
  *local_seconds = seconds;
  return (uint8_t)(isfinite(seconds) && *local_us <= global);
}

uint8_t m5_trajectory_safety_test_active_segment(
    uint32_t step_us, uint32_t interval, uint32_t offset_us, uint64_t base_us,
    const float start_position[3], const float start_velocity[3],
    const float start_acceleration[3], const float end_position[3],
    const float end_velocity[3], const float end_acceleration[3],
    float endpoints[18], float coefficients[18], float *seconds,
    uint32_t *active_interval)
{
  MotionReference reference;
  const MotionReferencePoint *a = NULL;
  const MotionReferencePoint *b = NULL;
  uint32_t measured_step = 0u;
  uint64_t time;
  float c[3][6];
  float local_seconds = 0.0f;
  unsigned axis, k;

  if (endpoints == NULL || coefficients == NULL || seconds == NULL ||
      active_interval == NULL) return 0u;
  if (interval >= PRODUCT_PREVIEW_COUNT || offset_us > step_us) return 0u;
  if (step_us != (uint32_t)PRODUCT_PREVIEW_STEP_US &&
      step_us != (uint32_t)PRODUCT_ROUTE_PREVIEW_STEP_US) return 0u;
  safety_test_fill_reference(&reference, step_us, base_us);
  {
    MotionReferencePoint *start_point = interval == 0u
        ? &reference.anchor : &reference.points[interval - 1u];
    safety_test_set_state(start_point, start_position, start_velocity,
                          start_acceleration);
  }
  safety_test_set_state(&reference.points[interval], end_position, end_velocity,
                        end_acceleration);
  time = base_us + (uint64_t)interval * step_us + offset_us;
  if (control_reference_locate_segment(&reference, time, &a, &b, &local_seconds,
                                       &measured_step) != STATUS_OK) return 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    endpoints[axis * 6u + 0u] = a->point.position_ned_m[axis];
    endpoints[axis * 6u + 1u] = a->point.velocity_ned_mps[axis];
    endpoints[axis * 6u + 2u] = a->point.acceleration_ned_mps2[axis];
    endpoints[axis * 6u + 3u] = b->point.position_ned_m[axis];
    endpoints[axis * 6u + 4u] = b->point.velocity_ned_mps[axis];
    endpoints[axis * 6u + 5u] = b->point.acceleration_ned_mps2[axis];
  }
  control_reference_hermite_coefficients(a->point.position_ned_m,
                                         a->point.velocity_ned_mps,
                                         a->point.acceleration_ned_mps2,
                                         b->point.position_ned_m,
                                         b->point.velocity_ned_mps,
                                         b->point.acceleration_ned_mps2,
                                         (float)measured_step * 1e-6f, c);
  for (axis = 0u; axis < 3u; ++axis)
    for (k = 0u; k < 6u; ++k)
      coefficients[axis * 6u + k] = c[axis][k];
  *seconds = local_seconds;
  *active_interval = (uint32_t)((time - base_us) / step_us);
  if (*active_interval >= PRODUCT_PREVIEW_COUNT)
    *active_interval = PRODUCT_PREVIEW_COUNT - 1u;
  return (uint8_t)(isfinite(local_seconds) && isfinite(coefficients[0]));
}

uint8_t m5_trajectory_safety_test_product_envelope(
    float *max_linear_speed_mps, float *max_linear_accel_mps2,
    float max_velocity_ned_mps[3], float max_acceleration_ned_mps2[3],
    float max_jerk_ned_mps3[3], uint32_t *valid_mask,
    uint32_t *safety_envelope_id, uint32_t *config_version,
    uint32_t *parameter_hash, uint32_t *profile_hash, uint32_t *profile_id)
{
  ProductFmsConfig config;
  SafetyEnvelope envelope;
  unsigned axis;

  if (max_linear_speed_mps == NULL || max_linear_accel_mps2 == NULL ||
      max_velocity_ned_mps == NULL || max_acceleration_ned_mps2 == NULL ||
      max_jerk_ned_mps3 == NULL || valid_mask == NULL ||
      safety_envelope_id == NULL || config_version == NULL ||
      parameter_hash == NULL || profile_hash == NULL ||
      profile_id == NULL) return 0u;
  fms_model_default_config(&config);
  if (fms_product_validate_config(&config) != STATUS_OK) return 0u;
  if (fms_product_build_safety_envelope(&fms_quad_x4_policy, &config, 1u,
                                        &envelope) != STATUS_OK) {
    return 0u;
  }
  if (envelope.safety_envelope_id == 0u ||
      envelope.vehicle_extension.valid_mask != 0x7Fu ||
      !isfinite(envelope.max_linear_speed_mps) ||
      !isfinite(envelope.max_linear_accel_mps2)) {
    return 0u;
  }
  *max_linear_speed_mps = envelope.max_linear_speed_mps;
  *max_linear_accel_mps2 = envelope.max_linear_accel_mps2;
  for (axis = 0u; axis < 3u; ++axis) {
    max_velocity_ned_mps[axis] =
        envelope.vehicle_extension.max_velocity_ned_mps[axis];
    max_acceleration_ned_mps2[axis] =
        envelope.vehicle_extension.max_acceleration_ned_mps2[axis];
    max_jerk_ned_mps3[axis] =
        envelope.vehicle_extension.max_jerk_ned_mps3[axis];
  }
  *valid_mask = envelope.vehicle_extension.valid_mask;
  *safety_envelope_id = envelope.safety_envelope_id;
  *config_version = config.config_version;
  *parameter_hash = config.parameter_hash;
  *profile_hash = config.profile_hash;
  *profile_id = envelope.header.profile_id;
  return 1u;
}

uint8_t m5_trajectory_safety_test_product_limits(
    float *max_linear_speed_mps, float *max_linear_accel_mps2,
    float max_velocity_ned_mps[3], float max_acceleration_ned_mps2[3],
    float max_jerk_ned_mps3[3])
{
  uint32_t valid_mask, envelope_id, config_version, parameter_hash, profile_hash;
  uint32_t profile_id;
  return m5_trajectory_safety_test_product_envelope(
      max_linear_speed_mps, max_linear_accel_mps2, max_velocity_ned_mps,
      max_acceleration_ned_mps2, max_jerk_ned_mps3, &valid_mask,
      &envelope_id, &config_version, &parameter_hash, &profile_hash,
      &profile_id);
}

#endif /* FLIGHT_M5_SAFETY_TEST_API */

#endif /* FLIGHT_M5_HOST_PLANNER */
