#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

Status guidance_build_pilot(GuidanceState *s, const GuidanceInput *in, GuidanceOutput *out)
{
    PilotControlReference *r = &out->pilot_control_reference;
    const ProductOperatorInput *sticks = &in->authorized_operator_input->payload;
    const ProductSafetyEnvelopePayload *e = &in->safety_envelope->vehicle_extension;
    ProductPilotControlReferencePayload *p = &r->vehicle_extension;
    uint16_t kind = s->ctx.kind;
    float dt = s->pilot_valid ? (in->control_time_us-s->last_state_time_us)*1e-6f : 0;
    float roll = sticks->roll_norm*e->max_tilt_rad, pitch = sticks->pitch_norm*e->max_tilt_rad;
    float tilt = hypotf(roll, pitch);
    if (!s->pilot_valid) {
        s->pilot_yaw_rad = guidance_yaw_from_quat(in->navigation->core.q_nb_wxyz);
        s->pilot_height_up_m = -in->navigation->core.position_ned_m[2];
        s->pilot_vertical_up_mps = 0; s->pilot_valid = 1;
    }
    if (kind == GUIDANCE_REFERENCE_PILOT_RATE) {
        p->valid_mask = PRODUCT_PILOT_REF_VALID_RATE | PRODUCT_PILOT_REF_VALID_THROTTLE;
        p->body_rate_frd_radps[0] = sticks->roll_norm*e->max_body_rate_frd_radps[0];
        p->body_rate_frd_radps[1] = sticks->pitch_norm*e->max_body_rate_frd_radps[1];
        p->body_rate_frd_radps[2] = sticks->yaw_norm*e->max_body_rate_frd_radps[2];
        p->collective_throttle_norm = sticks->throttle_norm;
    } else {
        float half, scale, cy, sy;
        if (tilt > e->max_tilt_rad) { roll *= e->max_tilt_rad/tilt; pitch *= e->max_tilt_rad/tilt; }
        tilt = hypotf(roll, pitch); half = tilt*0.5f;
        scale = tilt > 1e-7f ? sinf(half)/tilt : 0.5f;
        s->pilot_yaw_rad = guidance_wrap_pi(s->pilot_yaw_rad+sticks->yaw_norm*in->safety_envelope->max_yaw_rate_radps*dt);
        cy = cosf(s->pilot_yaw_rad*0.5f); sy = sinf(s->pilot_yaw_rad*0.5f);
        p->q_nb_desired_wxyz[0] = cy*cosf(half);
        p->q_nb_desired_wxyz[1] = scale*(cy*roll-sy*pitch);
        p->q_nb_desired_wxyz[2] = scale*(cy*pitch+sy*roll);
        p->q_nb_desired_wxyz[3] = sy*cosf(half);
        p->valid_mask = PRODUCT_PILOT_REF_VALID_ATTITUDE | PRODUCT_PILOT_REF_VALID_YAW_FF;
        p->yaw_rate_ff_radps = sticks->yaw_norm*in->safety_envelope->max_yaw_rate_radps;
        if (kind == GUIDANCE_REFERENCE_ALTITUDE_PILOT) {
            s->pilot_height_up_m += s->pilot_vertical_up_mps*dt;
            s->pilot_vertical_up_mps = (2*sticks->throttle_norm-1)*e->max_velocity_ned_mps[2];
            p->altitude_above_origin_m = s->pilot_height_up_m;
            p->vertical_velocity_up_mps = s->pilot_vertical_up_mps;
            p->valid_mask |= PRODUCT_PILOT_REF_VALID_ALTITUDE_UP | PRODUCT_PILOT_REF_VALID_VERTICAL_UP;
        } else {
            p->valid_mask |= PRODUCT_PILOT_REF_VALID_THROTTLE;
            p->collective_throttle_norm = sticks->throttle_norm;
        }
    }
    r->reference_id = s->ref_id; r->reference_epoch = s->ref_epoch;
    r->state_epoch = s->ctx.state_epoch; r->origin_id = s->ctx.origin_id;
    r->authority_epoch = s->ctx.authority_epoch; r->directive_epoch = s->ctx.directive_epoch;
    r->reference_kind = kind == GUIDANCE_REFERENCE_PILOT_RATE ? PILOT_REFERENCE_RATE :
        kind == GUIDANCE_REFERENCE_PILOT_ATTITUDE ? PILOT_REFERENCE_ATTITUDE : PILOT_REFERENCE_ALTITUDE_ATTITUDE;
    guidance_publish_header(in, s, &r->header, MSG_PILOT_CONTROL_REFERENCE, kind);
    return STATUS_OK;
}

/* Typed altitude-only landing: never feeds invented horizontal coordinates
 * into XYZ tracking. Each bounded descent segment ends at zero v/a. */

Status guidance_build_vertical_land(GuidanceState *s, const GuidanceInput *in,
                                    GuidanceOutput *out, const float *handoff_acceleration,
                                    uint16_t *reason, uint16_t *constraint)
{
    PilotControlReference *r = &out->pilot_control_reference;
    ProductPilotControlReferencePayload *p = &r->vehicle_extension;
    float elapsed = s->plan_valid ? (in->control_time_us-s->plan_start_us)*1e-6f : 0;
    float d, v, a, goal, duration, plane = 0.0f;
    uint8_t home_valid = guidance_home_ok(in);
    unsigned i, attempt;

    *reason = GUIDANCE_PLAN_REASON_NONE;
    *constraint = GUIDANCE_PLAN_CONSTRAINT_NONE;
    /* With no horizontal capability, a geofence swept-volume claim is impossible. */
    if (in->geofence_count) {
        *reason = GUIDANCE_PLAN_REASON_CLEARANCE_UNPROVEN;
        *constraint = GUIDANCE_PLAN_CONSTRAINT_GEOFENCE;
        return STATUS_NOT_READY;
    }
    if (!s->plan_valid || elapsed >= s->plan_duration_s) {
        if (s->plan_valid) {
            d = guidance_quintic_eval(&s->plan[2], s->plan_duration_s, 0);
            v = a = 0;
        } else {
            d = in->navigation->core.position_ned_m[2];
            v = in->navigation->core.velocity_ned_mps[2];
            a = handoff_acceleration != NULL ? handoff_acceleration[2] :
                ((in->navigation->core.valid_mask & NAV_VALID_ACCELERATION) ?
                    in->navigation->core.acceleration_ned_mps2[2] : 0.0f);
        }
        if (!isfinite(a)) {
            *reason = GUIDANCE_PLAN_REASON_NUMERIC;
            *constraint = GUIDANCE_PLAN_CONSTRAINT_DYNAMICS;
            return STATUS_NUMERIC_ERROR;
        }

        goal = d + 1.0f;
        if (home_valid) {
            plane = in->directive->payload.home.position_ned_m[2];
            const float floor_m = in->directive->operation_code == PRODUCT_OP_LAND
                                      ? plane + GUIDANCE_LANDING_PROBE_M
                                      : plane;
            /* Reach Home with zero v/a first. Only if contact evidence is still
             * absent on the next segment do we make the bounded below-plane probe. */
            if (in->directive->operation_code == PRODUCT_OP_LAND &&
                d < plane && goal > plane) {
                goal = plane;
            } else {
                goal = fmaxf(d, fminf(goal, floor_m));
            }
        }
        if (in->landed_state && guidance_header_ok(&in->landed_state->header, &in->navigation->header,
            MSG_LANDED_STATE_SNAPSHOT, in->evaluation_time_us) &&
            in->landed_state->state == LANDED_STATE_LANDED) {
            goal = d;
        }

        duration = 0.2f;
        if (home_valid && in->directive->operation_code == PRODUCT_OP_LAND &&
            d >= plane - 1e-5f && goal > d + 1e-5f) {
            duration = GUIDANCE_LANDING_PROBE_DURATION_S;
        }
        for (attempt = 0; attempt < GUIDANCE_MAX_DURATION_ATTEMPTS &&
             duration <= GUIDANCE_MAX_DURATION_S + 1e-5f; ++attempt) {
            uint8_t valid = 1;
            *constraint = GUIDANCE_PLAN_CONSTRAINT_NONE;
            memset(s->plan, 0, sizeof(s->plan));
            guidance_quintic_solve(d, v, a, goal, duration, &s->plan[2]);
            for (i = 0; i < 16; ++i) {
                if (!guidance_curve_dynamics(s->plan, duration*i/16, duration*(i+1)/16,
                                             in->safety_envelope, &s->config, constraint)) {
                    valid = 0;
                    break;
                }
            }
            if (valid) break;
            duration = ceilf(duration*1.25f/0.2f)*0.2f;
        }
        if (attempt == GUIDANCE_MAX_DURATION_ATTEMPTS ||
            duration > GUIDANCE_MAX_DURATION_S + 1e-5f) {
            s->plan_valid = 0;
            *reason = GUIDANCE_PLAN_REASON_ENVELOPE_INVALID;
            if (*constraint == GUIDANCE_PLAN_CONSTRAINT_NONE) {
                *constraint = GUIDANCE_PLAN_CONSTRAINT_DYNAMICS;
            }
            return STATUS_OUT_OF_RANGE;
        }
        s->plan_duration_s = duration;
        s->plan_start_us = in->control_time_us;
        s->plan_valid = 1;
        elapsed = 0;
        s->pilot_yaw_rad = guidance_yaw_from_quat(in->navigation->core.q_nb_wxyz);
    }
    p->valid_mask = PRODUCT_PILOT_REF_VALID_ATTITUDE | PRODUCT_PILOT_REF_VALID_ALTITUDE_UP |
        PRODUCT_PILOT_REF_VALID_VERTICAL_UP;
    guidance_quat_from_rpy(0, 0, s->pilot_yaw_rad, p->q_nb_desired_wxyz);
    p->altitude_above_origin_m = -guidance_quintic_eval(&s->plan[2], elapsed, 0);
    p->vertical_velocity_up_mps = -guidance_quintic_eval(&s->plan[2], elapsed, 1);
    r->reference_id = s->ref_id;
    r->reference_epoch = s->ref_epoch;
    r->state_epoch = s->ctx.state_epoch;
    r->origin_id = s->ctx.origin_id;
    r->authority_epoch = s->ctx.authority_epoch;
    r->directive_epoch = s->ctx.directive_epoch;
    r->reference_kind = PILOT_REFERENCE_ALTITUDE_ATTITUDE;
    guidance_publish_header(in, s, &r->header, MSG_PILOT_CONTROL_REFERENCE,
                            GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC);
    return STATUS_OK;
}