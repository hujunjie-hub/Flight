/* Continuous bounds on the actual consumer Hermite curves. No sampling test
 * is used as a substitute for a bound. H723's admitted profile is NONE. */
#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

static const uint8_t binomial[6][6] = {
    {1}, {1, 1}, {1, 2, 1}, {1, 3, 3, 1},
    {1, 4, 6, 4, 1}, {1, 5, 10, 10, 5, 1}
};

uint8_t guidance_quintic_bounds(const GuidanceQuintic *q, float t0, float t1, uint8_t derivative,
                           float *lo, float *hi)
{
    double c[6] = {0}, a[6] = {0}, min = INFINITY, max = -INFINITY, scale = 0;
    double t_power[6] = {1}, h_power[6] = {1};
    unsigned n, i, k, d;
    if (derivative > 3 || !isfinite(t0) || !isfinite(t1) || t1 < t0) return 0;
    n = 5u - derivative;
    for (i = 1; i <= 5; ++i) {
        t_power[i] = t_power[i-1]*t0;
        h_power[i] = h_power[i-1]*((double)t1-t0);
    }
    for (i = derivative; i <= 5; ++i) {
        double v = q->c[i];
        for (d = 0; d < derivative; ++d) v *= i - d;
        c[i - derivative] = v;
    }
    for (k = 0; k <= n; ++k) {
        for (i = k; i <= n; ++i)
            a[k] += c[i] * binomial[i][k] * t_power[i-k];
        a[k] *= h_power[k];
        scale += fabs(a[k]);
    }
    for (i = 0; i <= n; ++i) {
        double b = 0;
        for (k = 0; k <= i; ++k) b += a[k] * binomial[i][k] / binomial[n][k];
        if (!isfinite(b)) return 0;
        min = fmin(min, b);
        max = fmax(max, b);
    }
    /* Round outwards, including cancellation in the power-to-Bernstein map. */
    *lo = nextafterf((float)(min - 1e-12 * (1 + scale)), -INFINITY);
    *hi = nextafterf((float)(max + 1e-12 * (1 + scale)), INFINITY);
    return (uint8_t)(isfinite(*lo) && isfinite(*hi));
}

static float absmax(float lo, float hi) { return fmaxf(fabsf(lo), fabsf(hi)); }

uint8_t guidance_has_landing_plane(const GuidanceInput *in)
{
    const uint16_t operation = in->directive->operation_code;
    /* The provisional simulation policy uses a flat Home reference plane. */
    return in->directive->policy_id == 2u && (operation == PRODUCT_OP_LAND || operation == PRODUCT_OP_RETURN) &&
           guidance_home_ok(in);
}

uint8_t guidance_landing_speed_ok(const GuidanceInput *in, const GuidanceQuintic *down,
                                  float t0, float t1)
{
    float lo, hi;
    const float brake_down = in->directive->payload.home.position_ned_m[2] -
                             GUIDANCE_LANDING_BRAKE_HEIGHT_M;
    float start_down, end_down, check_t;
    unsigned iteration;
    if (!guidance_has_landing_plane(in))
        return 1u;
    if (!guidance_quintic_bounds(down, t0, t1, 0, &lo, &hi))
        return 0u;
    if (hi < brake_down)
        return 1u;

    start_down = guidance_quintic_eval(down, t0, 0);
    end_down = guidance_quintic_eval(down, t1, 0);
    if (!isfinite(start_down) || !isfinite(end_down))
        return 0u;
    /* Allow a faster approach above the braking band. Within the final 20 cm,
     * constrain speed early enough to enter the 6 cm contact-sensor band safely. */
    check_t = t0;
    if (start_down < brake_down) {
        float left = t0, right = t1, velocity_lo, velocity_hi;
        /* Prove the curve is nondecreasing in NED-down before deciding whether it
         * crossed the band. If it is monotone and the endpoint is still above the
         * band, the conservative Bernstein position hull alone is not a crossing. */
        if (!guidance_quintic_bounds(down, t0, t1, 1, &velocity_lo, &velocity_hi) ||
            velocity_lo < -1e-5f)
            return 0u;
        if (end_down < brake_down)
            return 1u;
        for (iteration = 0; iteration < 16u; ++iteration) {
            const float mid = 0.5f * (left + right);
            if (guidance_quintic_eval(down, mid, 0) >= brake_down)
                right = mid;
            else
                left = mid;
        }
        check_t = right;
    }
    return guidance_quintic_bounds(down, check_t, t1, 1, &lo, &hi) &&
           lo >= -GUIDANCE_LANDING_MAX_VERTICAL_SPEED_MPS - 1e-5f &&
           hi <= GUIDANCE_LANDING_MAX_VERTICAL_SPEED_MPS + 1e-5f;
}

static float absmin(float lo, float hi)
{
    return lo <= 0 && hi >= 0 ? 0 : fminf(fabsf(lo), fabsf(hi));
}

uint8_t guidance_curve_dynamics(const GuidanceQuintic q[4], float t0, float t1,
                           const SafetyEnvelope *envelope,
                           const ProductGuidanceConfig *config, uint16_t *constraint)
{
    const ProductSafetyEnvelopePayload *e = &envelope->vehicle_extension;
    const ProductGuidanceConfigPayload *cfg = &config->payload;
    float acc_lo[3], acc_hi[3], v2 = 0, a2 = 0, horizontal2 = 0;
    float thrust_min2 = 0, thrust_max2 = 0, lo, hi;
    unsigned axis, derivative;
    for (axis = 0; axis < 3; ++axis) {
        const float limit[3] = {
            fminf(e->max_velocity_ned_mps[axis], cfg->max_velocity_ned_mps[axis]),
            fminf(e->max_acceleration_ned_mps2[axis], cfg->max_acceleration_ned_mps2[axis]),
            fminf(e->max_jerk_ned_mps3[axis], cfg->max_jerk_ned_mps3[axis])};
        for (derivative = 1; derivative <= 3; ++derivative) {
            float bound;
            if (!guidance_quintic_bounds(&q[axis], t0, t1, (uint8_t)derivative, &lo, &hi)) {
                *constraint = GUIDANCE_PLAN_CONSTRAINT_DYNAMICS; return 0;
            }
            bound = absmax(lo, hi);
            if (bound > limit[derivative-1] + 1e-5f) {
                *constraint = (uint16_t)(GUIDANCE_PLAN_CONSTRAINT_SPEED + derivative-1); return 0;
            }
            if (derivative == 1) v2 += bound * bound;
            if (derivative == 2) {
                a2 += bound * bound; acc_lo[axis] = lo; acc_hi[axis] = hi;
            }
        }
    }
    if (v2 > envelope->max_linear_speed_mps * envelope->max_linear_speed_mps + 1e-5f) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_SPEED; return 0;
    }
    if (a2 > envelope->max_linear_accel_mps2 * envelope->max_linear_accel_mps2 + 1e-5f) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_ACCELERATION; return 0;
    }
    for (axis = 0; axis < 3; ++axis) {
        float l = axis == 2 ? cfg->gravity_mps2 - acc_hi[axis] : acc_lo[axis];
        float h = axis == 2 ? cfg->gravity_mps2 - acc_lo[axis] : acc_hi[axis];
        float amin = absmin(l, h), amax = absmax(l, h);
        thrust_min2 += amin * amin; thrust_max2 += amax * amax;
        if (axis < 2) horizontal2 += amax * amax;
    }
    lo = cfg->gravity_mps2 - acc_hi[2];
    hi = tanf(e->max_tilt_rad);
    if (lo <= 0 || horizontal2 > hi * hi * lo * lo + 1e-5f) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_TILT; return 0;
    }
    if (cfg->mass_kg * sqrtf(thrust_min2) < e->min_total_thrust_n - 1e-5f ||
        cfg->mass_kg * sqrtf(thrust_max2) > e->max_total_thrust_n + 1e-5f) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_THRUST; return 0;
    }
    if (!guidance_quintic_bounds(&q[3], t0, t1, 1, &lo, &hi) ||
        absmax(lo, hi) > envelope->max_yaw_rate_radps + 1e-5f) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_DYNAMICS; return 0;
    }
    return 1;
}

typedef struct {
    uint8_t interval, depth;
    float u0, u1;
} GuidanceValidationWork;

enum {
    FENCE_VALIDATION_MAX_DEPTH = 7,
    /* Splitting a DFS node leaves one additional entry pending per level. */
    VALIDATION_STACK_CAPACITY = GUIDANCE_PREVIEW_POINTS + GUIDANCE_VALIDATION_MAX_DEPTH
};
CONTRACT_STATIC_ASSERT(VALIDATION_STACK_CAPACITY >= 1 + FENCE_VALIDATION_MAX_DEPTH,
                       "validation stack must also fit a single fence root");
static GuidanceValidationWork work[VALIDATION_STACK_CAPACITY];

static uint8_t fence_inflation(const GuidanceInput *in, const ProductGuidanceConfig *config,
                                float horizon, float *horizontal, float *vertical)
{
    float std[3], delay = (in->evaluation_time_us-in->control_time_us)*1e-6f;
    float radius = config->payload.vehicle_radius_m+config->payload.clearance_margin_m;
    unsigned axis;
    for (axis = 0; axis < 3; ++axis) {
        float p = in->navigation->core.position_std_m[axis];
        float v = in->navigation->core.velocity_std_mps[axis];
        if (!isfinite(p) || !isfinite(v) || p < 0 || v < 0) return 0;
        std[axis] = p+(horizon+delay)*v;
    }
    *horizontal = radius+config->payload.uncertainty_inflation_factor*hypotf(std[0], std[1])+
        delay*in->safety_envelope->max_linear_speed_mps;
    *vertical = radius+config->payload.uncertainty_inflation_factor*std[2]+
        delay*in->safety_envelope->vehicle_extension.max_velocity_ned_mps[2];
    return (uint8_t)(isfinite(*horizontal) && isfinite(*vertical));
}

uint8_t guidance_plan_fences(const GuidanceInput *in, const ProductGuidanceConfig *config,
                        const GuidanceQuintic q[4], float duration)
{
    unsigned count = 1, processed = 0, axis, i;
    float home_d = in->directive->payload.home.position_ned_m[2];
    float horizontal_guard, vertical_guard;
    if (in->geofence_count == 0) return 1;
    if (!fence_inflation(in, config, duration, &horizontal_guard, &vertical_guard)) return 0;
    work[0] = (GuidanceValidationWork){.u1 = duration};
    while (count) {
        GuidanceValidationWork item = work[--count];
        float lo[3], hi[3], p[3], mid = (item.u0+item.u1)*0.5f;
        uint8_t proved = 1;
        if (++processed > GUIDANCE_VALIDATION_WORK_UNITS) return 0;
        for (axis = 0; axis < 3; ++axis) {
            if (!guidance_quintic_bounds(&q[axis], item.u0, item.u1, 0, &lo[axis], &hi[axis])) return 0;
            p[axis] = guidance_quintic_eval(&q[axis], mid, 0);
        }
        for (i = 0; i < in->geofence_count; ++i) {
            const ProductGeofenceSnapshot *f = &in->geofences[i];
            float dx = fmaxf(p[0]-lo[0], hi[0]-p[0]);
            float dy = fmaxf(p[1]-lo[1], hi[1]-p[1]);
            float clearance = fence_clearance(f, p[0], p[1])-horizontal_guard;
            float min_h = f->min_height_above_home_m+f->vertical_margin_m+vertical_guard;
            float max_h = f->max_height_above_home_m-f->vertical_margin_m-vertical_guard;
            if (clearance < 0 || home_d-p[2] < min_h || home_d-p[2] > max_h) return 0;
            if (clearance <= sqrtf(dx*dx+dy*dy) || home_d-hi[2] < min_h-1e-5f ||
                home_d-lo[2] > max_h+1e-5f) proved = 0;
        }
        if (proved) continue;
        if (item.depth >= FENCE_VALIDATION_MAX_DEPTH || count + 2 > VALIDATION_STACK_CAPACITY) return 0;
        work[count++] = (GuidanceValidationWork){.depth = item.depth + 1, .u0 = mid, .u1 = item.u1};
        work[count++] = (GuidanceValidationWork){.depth = item.depth + 1, .u0 = item.u0, .u1 = mid};
    }
    return 1;
}

uint8_t guidance_validate_reference(const GuidanceInput *in, const ProductGuidanceConfig *config,
                               const MotionReference *ref, uint16_t *reason,
                               uint16_t *constraint, uint32_t *offset)
{
    unsigned i, axis, count = 0, processed = 0;
    const uint32_t step_us = ref->points[0].offset_us;
    float home_d = in->directive->payload.home.position_ned_m[2];
    float horizontal_guard = 0, vertical_guard = 0;
    *reason = GUIDANCE_PLAN_REASON_NONE; *constraint = GUIDANCE_PLAN_CONSTRAINT_NONE; *offset = 0;
    if (ref->point_count != GUIDANCE_PREVIEW_POINTS || ref->anchor.offset_us != 0 ||
        (step_us != GUIDANCE_PREVIEW_US && step_us != PRODUCT_ROUTE_PREVIEW_STEP_US)) {
        *constraint = GUIDANCE_PLAN_CONSTRAINT_TIME; goto invalid;
    }
    if (in->geofence_count && !fence_inflation(in, config, step_us * GUIDANCE_PREVIEW_POINTS * 1e-6f,
                                             &horizontal_guard, &vertical_guard)) {
        *reason = M5_VALIDATION_UNCERTAINTY_UNBOUNDED; *constraint = GUIDANCE_PLAN_CONSTRAINT_GEOFENCE; return 0;
    }
    for (i = GUIDANCE_PREVIEW_POINTS; i > 0; --i)
        work[count++] = (GuidanceValidationWork){.interval = (uint8_t)(i - 1), .u1 = 1};
    while (count != 0) {
        GuidanceValidationWork item = work[--count];
        const MotionReferencePoint *a = item.interval == 0 ? &ref->anchor :
            &ref->points[item.interval-1];
        const MotionReferencePoint *b = &ref->points[item.interval];
        GuidanceQuintic q[4];
        float h, t0, t1, mid, p[3], lower[3], upper[3];
        uint8_t proved = 1;
        uint16_t dynamic_constraint = GUIDANCE_PLAN_CONSTRAINT_NONE;
        *offset = a->offset_us + (uint32_t)(item.u0 * step_us);
        if (++processed > GUIDANCE_VALIDATION_WORK_UNITS) {
            *reason = GUIDANCE_PLAN_REASON_BUDGET_EXCEEDED; *constraint = GUIDANCE_PLAN_CONSTRAINT_CAPACITY;
            return 0;
        }
        if (a->offset_us != (uint32_t)item.interval * step_us ||
            b->offset_us != (uint32_t)(item.interval+1u) * step_us ||
            a->valid_mask != 0x1Fu || b->valid_mask != 0x1Fu) {
            *constraint = GUIDANCE_PLAN_CONSTRAINT_TIME; goto invalid;
        }
        h = step_us * 1e-6f;
        t0 = h * item.u0; t1 = h * item.u1; mid = (t0+t1)*0.5f;
        for (axis = 0; axis < 3; ++axis) {
            guidance_quintic_boundary(a->point.position_ned_m[axis], a->point.velocity_ned_mps[axis],
                a->point.acceleration_ned_mps2[axis], b->point.position_ned_m[axis],
                b->point.velocity_ned_mps[axis], b->point.acceleration_ned_mps2[axis], h, &q[axis]);
            p[axis] = guidance_quintic_eval(&q[axis], mid, 0);
            if (!guidance_quintic_bounds(&q[axis], t0, t1, 0, &lower[axis], &upper[axis]) ||
                !isfinite(p[axis])) { *constraint = GUIDANCE_PLAN_CONSTRAINT_DYNAMICS; goto invalid; }
        }
        /* Consumer yaw is cubic Hermite with shortest-angle unwrapping. */
        memset(&q[3], 0, sizeof(q[3]));
        q[3].c[0] = a->point.yaw_rad;
        q[3].c[1] = a->point.yaw_rate_radps;
        {
            float delta = guidance_wrap_pi(b->point.yaw_rad-a->point.yaw_rad);
            q[3].c[2] = (3*delta/h - 2*a->point.yaw_rate_radps-b->point.yaw_rate_radps)/h;
            q[3].c[3] = (-2*delta/h + a->point.yaw_rate_radps+b->point.yaw_rate_radps)/(h*h);
        }
        if (!guidance_curve_dynamics(q, t0, t1, in->safety_envelope, config, &dynamic_constraint))
            proved = 0;
        if (proved && !guidance_landing_speed_ok(in, &q[2], t0, t1)) {
            dynamic_constraint = GUIDANCE_PLAN_CONSTRAINT_SPEED;
            proved = 0;
        }
        for (i = 0; i < in->geofence_count; ++i) {
            const ProductGeofenceSnapshot *f = &in->geofences[i];
            float dx = fmaxf(p[0]-lower[0], upper[0]-p[0]);
            float dy = fmaxf(p[1]-lower[1], upper[1]-p[1]);
            float clearance = fence_clearance(f, p[0], p[1])-horizontal_guard;
            float min_h = f->min_height_above_home_m+f->vertical_margin_m+vertical_guard;
            float max_h = f->max_height_above_home_m-f->vertical_margin_m-vertical_guard;
            if (clearance < 0 || home_d-p[2] < min_h || home_d-p[2] > max_h) {
                *reason = GUIDANCE_PLAN_REASON_GEOFENCE_VIOLATION; *constraint = GUIDANCE_PLAN_CONSTRAINT_GEOFENCE;
                return 0;
            }
            if (clearance <= sqrtf(dx*dx+dy*dy) || home_d-upper[2] < min_h-1e-5f ||
                home_d-lower[2] > max_h+1e-5f) proved = 0;
        }
        if (proved) continue;
        if (item.depth >= GUIDANCE_VALIDATION_MAX_DEPTH) {
            *reason = dynamic_constraint ? GUIDANCE_PLAN_REASON_ENVELOPE_INVALID : GUIDANCE_PLAN_REASON_CLEARANCE_UNPROVEN;
            *constraint = dynamic_constraint ? dynamic_constraint : GUIDANCE_PLAN_CONSTRAINT_GEOFENCE;
            return 0;
        }
        if (count + 2 > VALIDATION_STACK_CAPACITY) {
            *reason = GUIDANCE_PLAN_REASON_BUDGET_EXCEEDED; *constraint = GUIDANCE_PLAN_CONSTRAINT_CAPACITY; return 0;
        }
        {
            float u = (item.u0+item.u1)*0.5f;
            work[count++] = (GuidanceValidationWork){.interval = item.interval, .depth = item.depth + 1,
                                                   .u0 = u, .u1 = item.u1};
            work[count++] = (GuidanceValidationWork){.interval = item.interval, .depth = item.depth + 1,
                                                   .u0 = item.u0, .u1 = u};
        }
    }
    *offset = 0;
    return 1;
invalid:
    *reason = GUIDANCE_PLAN_REASON_NUMERIC;
    return 0;
}
