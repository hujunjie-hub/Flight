#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

static uint8_t finite3(const float v[3])
{ return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2])); }

static uint8_t route_operation(const GuidanceInput *in)
{
    const uint16_t operation = in->directive->operation_code;
    return operation == PRODUCT_OP_MISSION || operation == PRODUCT_OP_RETURN;
}

/* M5 time is quantized upward to the 200 ms planning lattice, capped at 31 bins. */
static float quantize_duration_up(float seconds)
{
    double ticks;
    if (!isfinite(seconds) || seconds <= 0.0f) return INFINITY;
    {
        const double time_bin_s = (double)GUIDANCE_TIME_BIN_US * 1e-6;
        ticks = ceil((double)seconds / time_bin_s - 2e-7);
        if (!isfinite(ticks) || ticks < 1.0 || ticks > GUIDANCE_MAX_TIME_BINS) return INFINITY;
        return (float)(ticks * time_bin_s);
    }
}

static uint8_t target(const GuidanceInput *in, GuidanceState *s, float p[3], float *yaw)
{
    const ProductDirectivePayload *d = &in->directive->payload;
    memcpy(p, s->ctx.target_ned, sizeof(s->ctx.target_ned)); *yaw = s->ctx.target_yaw_rad;
    if (in->directive->operation_code == PRODUCT_OP_MISSION) {
        const ProductMissionWaypoint *w = &in->authorized_mission->mission.waypoints[
            in->authorized_mission->current_item_index];
        memcpy(p, w->position_ned_m, sizeof(w->position_ned_m)); *yaw = w->yaw_rad;
    } else if (d->target_valid_mask & PRODUCT_DIRECTIVE_TARGET_POSITION) {
        memcpy(p, d->target_position_ned_m, sizeof(d->target_position_ned_m));
        if (d->target_valid_mask & PRODUCT_DIRECTIVE_TARGET_YAW) *yaw = d->target_yaw_rad;
    } else if (in->directive->operation_code == PRODUCT_OP_RETURN) {
        if (!guidance_home_ok(in)) return 0;
        memcpy(p, d->home.position_ned_m, sizeof(d->home.position_ned_m));
    } else if (in->directive->operation_code == PRODUCT_OP_LAND ||
               in->directive->operation_code == PRODUCT_OP_SAFE_ACTION) {
        if (!guidance_home_ok(in)) return 0; /* origin zero is not ground */
        p[2] = d->home.position_ned_m[2];
        if (in->directive->operation_code == PRODUCT_OP_LAND &&
            !(in->landed_state && guidance_header_ok(&in->landed_state->header,
                                                      &in->navigation->header,
                                                      MSG_LANDED_STATE_SNAPSHOT,
                                                      in->evaluation_time_us) &&
              in->landed_state->state == LANDED_STATE_LANDED)) {
            /* FMAT-0008: keep descending through the Home plane until the land
             * detector reports LANDED. Step from the measured position so the
             * tracking error stays small; the contact model stops the airframe. */
            const float step_z = in->navigation->core.position_ned_m[2] + GUIDANCE_LANDING_STEP_M;
            p[2] = fminf(step_z, d->home.position_ned_m[2] + GUIDANCE_LANDING_PROBE_M);
        }
    }
    return (uint8_t)(finite3(p) && isfinite(*yaw));
}

static void sample(const GuidanceState *s, uint64_t when, ProductMotionPoint *p)
{
    unsigned axis;
    float t = when > s->plan_start_us ? (float)(when-s->plan_start_us)*1e-6f : 0;
    uint8_t ended = t >= s->plan_duration_s;
    t = fminf(t, s->plan_duration_s);
    for (axis = 0; axis < 3; ++axis) {
        p->position_ned_m[axis] = guidance_quintic_eval(&s->plan[axis], t, 0);
        p->velocity_ned_mps[axis] = ended ? 0 : guidance_quintic_eval(&s->plan[axis], t, 1);
        p->acceleration_ned_mps2[axis] = ended ? 0 : guidance_quintic_eval(&s->plan[axis], t, 2);
    }
    p->yaw_rad = guidance_wrap_pi(guidance_quintic_eval(&s->plan[3], t, 0));
    p->yaw_rate_radps = ended ? 0 : guidance_quintic_eval(&s->plan[3], t, 1); p->valid_mask = 0x1Fu;
}

uint8_t guidance_handoff_acceleration(const GuidanceState *s, const GuidanceInput *in,
                                      const GuidanceRefContext *next, float acceleration[3])
{
    const GuidanceRefContext *previous = &s->ctx;
    if (in->directive->policy_id != 2u || !previous->valid || !s->active || !s->plan_valid ||
        previous->kind != GUIDANCE_REFERENCE_TRAJECTORY || next->kind != GUIDANCE_REFERENCE_TRAJECTORY ||
        previous->boot_id != next->boot_id || previous->time_epoch != next->time_epoch ||
        previous->state_epoch != next->state_epoch || previous->origin_id != next->origin_id ||
        in->control_time_us <= s->last_state_time_us || in->control_time_us - s->last_state_time_us > 40000u ||
        !(in->navigation->core.valid_mask & NAV_VALID_ACCELERATION))
        return 0u;

    const float *raw = in->navigation->core.acceleration_ned_mps2;
    if (!finite3(raw) || hypotf(hypotf(raw[0], raw[1]), raw[2]) > 6.0f)
        return 0u;

    ProductMotionPoint point;
    sample(s, in->control_time_us, &point);
    memcpy(acceleration, point.acceleration_ned_mps2, sizeof(point.acceleration_ned_mps2));
    return 1u;
}

static void landing_stage_target(const GuidanceInput *in, GuidanceState *s, float goal[3])
{
    enum { LANDING_APPROACH = 1, LANDING_FINAL = 2 };
    const float approach_down = goal[2] - 0.6f;
    const float *position = in->navigation->core.position_ned_m;
    const float *velocity = in->navigation->core.velocity_ned_mps;
    if (s->landing_stage == 0u)
        s->landing_stage = position[2] < approach_down ? LANDING_APPROACH : LANDING_FINAL;
    if (s->landing_stage == LANDING_APPROACH) {
        if (s->plan_valid && in->control_time_us >= s->plan_start_us &&
            (in->control_time_us - s->plan_start_us) * 1e-6f >= s->plan_duration_s &&
            hypotf(position[0] - goal[0], position[1] - goal[1]) <= 0.30f &&
            hypotf(velocity[0], velocity[1]) <= 0.25f &&
            fabsf(position[2] - approach_down) <= 0.20f && fabsf(velocity[2]) <= 0.20f) {
            s->landing_stage = LANDING_FINAL;
        } else {
            goal[2] = approach_down;
        }
    }
}

static uint8_t plan(GuidanceState *s, const GuidanceInput *in, const float goal[3], float yaw,
                    const float *handoff_acceleration, uint16_t *reason, uint16_t *constraint)
{
    const ProductGuidanceConfigPayload *cfg = &s->config.payload;
    const ProductSafetyEnvelopePayload *e = &in->safety_envelope->vehicle_extension;
    ProductMotionPoint start;
    GuidanceQuintic q[4];
    /* One M5 planning request is bounded to 6.2 s / 31 time bins.
       M4 must split longer mission and return legs into authorized waypoints;
       M5 never silently stretches a local planning horizon. */
    const float max_duration = GUIDANCE_MAX_DURATION_S;
    float duration = 0.2f;
    unsigned axis, attempt;
    memset(&start, 0, sizeof(start));
    if (s->plan_valid) sample(s, in->control_time_us, &start);
    else {
        memcpy(start.position_ned_m, in->navigation->core.position_ned_m, sizeof(start.position_ned_m));
        memcpy(start.velocity_ned_mps, in->navigation->core.velocity_ned_mps, sizeof(start.velocity_ned_mps));
        if (handoff_acceleration != NULL)
            memcpy(start.acceleration_ned_mps2, handoff_acceleration, sizeof(start.acceleration_ned_mps2));
        else if (in->navigation->core.valid_mask & NAV_VALID_ACCELERATION)
            memcpy(start.acceleration_ned_mps2, in->navigation->core.acceleration_ned_mps2,
                   sizeof(start.acceleration_ned_mps2));
        start.yaw_rad = guidance_yaw_from_quat(in->navigation->core.q_nb_wxyz);
    }
    if (!finite3(start.acceleration_ned_mps2)) { *reason = GUIDANCE_PLAN_REASON_NUMERIC; return 0; }
    for (axis = 0; axis < 3; ++axis) {
        float d = fabsf(goal[axis]-start.position_ned_m[axis]);
        duration = fmaxf(duration, d/fminf(cfg->max_velocity_ned_mps[axis], e->max_velocity_ned_mps[axis]));
        duration = fmaxf(duration, sqrtf(d/fminf(cfg->max_acceleration_ned_mps2[axis], e->max_acceleration_ned_mps2[axis])));
        duration = fmaxf(duration, cbrtf(d/fminf(cfg->max_jerk_ned_mps3[axis], e->max_jerk_ned_mps3[axis])));
    }
    duration = fmaxf(duration, fabsf(guidance_wrap_pi(yaw-start.yaw_rad))/in->safety_envelope->max_yaw_rate_radps);
    duration = quantize_duration_up(duration);
    for (attempt = 0; attempt < GUIDANCE_MAX_DURATION_ATTEMPTS && duration <= max_duration + 1e-5f; ++attempt) {
        uint8_t feasible = 1;
        unsigned interval;
        for (axis = 0; axis < 3; ++axis)
            guidance_quintic_solve(start.position_ned_m[axis], start.velocity_ned_mps[axis],
                start.acceleration_ned_mps2[axis], goal[axis], duration, &q[axis]);
        guidance_quintic_solve(start.yaw_rad, start.yaw_rate_radps, 0,
            start.yaw_rad+guidance_wrap_pi(yaw-start.yaw_rad), duration, &q[3]);
        /* Subdivision tightens the convex-hull bound without replacing it by samples. */
        for (interval = 0; interval < 16; ++interval) {
            const float t0 = duration * interval / 16;
            const float t1 = duration * (interval + 1u) / 16;
            if (!guidance_curve_dynamics(q, t0, t1, in->safety_envelope, &s->config, constraint)) {
                feasible = 0; break;
            }
            if (!guidance_landing_speed_ok(in, &q[2], t0, t1)) {
                *constraint = GUIDANCE_PLAN_CONSTRAINT_SPEED;
                feasible = 0; break;
            }
        }
        if (feasible) {
            if (!guidance_plan_fences(in, &s->config, q, duration)) {
                *reason = GUIDANCE_PLAN_REASON_GEOFENCE_VIOLATION; *constraint = GUIDANCE_PLAN_CONSTRAINT_GEOFENCE; return 0;
            }
            memcpy(s->plan, q, sizeof(q)); s->plan_start_us = in->control_time_us;
            s->plan_duration_s = duration; s->plan_valid = 1;
            memcpy(s->ctx.target_ned, goal, sizeof(s->ctx.target_ned)); s->ctx.target_yaw_rad = yaw;
            return 1;
        }
        if (duration >= max_duration) break;
        {
            const float next_duration = quantize_duration_up(duration * 1.25f);
            /* Do not skip the final 6.2 s bin when geometric growth overshoots it. */
            duration = (!isfinite(next_duration) || next_duration > max_duration)
                ? max_duration : next_duration;
        }
    }
    *reason = GUIDANCE_PLAN_REASON_ENVELOPE_INVALID;
    if (*constraint == GUIDANCE_PLAN_CONSTRAINT_NONE) *constraint = GUIDANCE_PLAN_CONSTRAINT_TIME;
    return 0;
}

Status guidance_build_trajectory(GuidanceState *s, const GuidanceInput *in, GuidanceOutput *out,
                            const float *handoff_acceleration, uint16_t *reason, uint16_t *constraint)
{
    MotionReference *r = &out->reference;
    uint32_t step_us = route_operation(in) ? PRODUCT_ROUTE_PREVIEW_STEP_US : GUIDANCE_PREVIEW_US;
    float goal[3], yaw;
    unsigned i;
    GuidanceInput limited;
    SafetyEnvelope envelope;
    *reason = GUIDANCE_PLAN_REASON_NONE; *constraint = GUIDANCE_PLAN_CONSTRAINT_NONE; s->violation_offset_us = 0;
    if (in->directive->operation_code == PRODUCT_OP_MISSION &&
        (in->authorized_mission->mission.flags & PRODUCT_MISSION_REQUIRES_OBSTACLE_AVOIDANCE)) {
        *reason = GUIDANCE_PLAN_REASON_OBSTACLE_UNKNOWN; *constraint = GUIDANCE_PLAN_CONSTRAINT_COLLISION; return STATUS_UNSUPPORTED;
    }
    if (in->directive->operation_code == PRODUCT_OP_MISSION) {
        float limit = in->authorized_mission->mission.waypoints[
            in->authorized_mission->current_item_index].speed_limit_mps;
        envelope = *in->safety_envelope; limited = *in;
        envelope.max_linear_speed_mps = fminf(envelope.max_linear_speed_mps, limit);
        for (i = 0; i < 3; ++i) envelope.vehicle_extension.max_velocity_ned_mps[i] =
            fminf(envelope.vehicle_extension.max_velocity_ned_mps[i], limit);
        limited.safety_envelope = &envelope; in = &limited;
    }
    if ((in->geofence_count && !guidance_home_ok(in)) || !target(in, s, goal, &yaw)) {
        *reason = GUIDANCE_PLAN_REASON_GEOFENCE_INVALID; *constraint = GUIDANCE_PLAN_CONSTRAINT_GEOFENCE; return STATUS_INVALID_ARGUMENT;
    }
    if (guidance_has_landing_plane(in))
        landing_stage_target(in, s, goal);
    if (!s->plan_valid || memcmp(goal, s->ctx.target_ned, sizeof(goal)) || yaw != s->ctx.target_yaw_rad)
        if (!plan(s, in, goal, yaw, handoff_acceleration, reason, constraint)) return STATUS_OUT_OF_RANGE;
    /* Keep the wider spacing through hold/land at a distant waypoint. At 8 m
     * the float position quantum doubles, and 20 ms Hermite intervals can
     * turn that rounding into excessive jerk even on a smooth stop. */
    for (i = 0; i < 3; ++i) {
        if (fabsf(goal[i]) >= 8.0f || fabsf(in->navigation->core.position_ned_m[i]) >= 8.0f)
            step_us = PRODUCT_ROUTE_PREVIEW_STEP_US;
    }
    r->reference_id = s->ref_id; r->reference_epoch = s->ref_epoch;
    r->state_epoch = s->ctx.state_epoch; r->origin_id = s->ctx.origin_id;
    r->authority_epoch = s->ctx.authority_epoch; r->directive_epoch = s->ctx.directive_epoch;
    r->mission_epoch = s->ctx.mission_epoch; r->current_item_id = s->ctx.current_item_id;
    r->execution_token = s->ctx.execution_token; r->reference_kind = GUIDANCE_REFERENCE_TRAJECTORY;
    r->point_count = GUIDANCE_PREVIEW_POINTS;
    for (i = 0; i <= GUIDANCE_PREVIEW_POINTS; ++i) {
        MotionReferencePoint *p = i == 0 ? &r->anchor : &r->points[i-1];
        p->offset_us = i * step_us; p->valid_mask = 0x1Fu;
        sample(s, in->control_time_us+p->offset_us, &p->point);
    }
    if (!guidance_validate_reference(in, &s->config, r, reason, constraint, &s->violation_offset_us))
        return STATUS_OUT_OF_RANGE;
    guidance_publish_header(in, s, &r->header, MSG_MOTION_REFERENCE, GUIDANCE_REFERENCE_TRAJECTORY);
    return STATUS_OK;
}
