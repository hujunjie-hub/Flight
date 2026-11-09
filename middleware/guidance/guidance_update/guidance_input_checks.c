#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"

static uint8_t finite3(const float v[3])
{ return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2])); }
static uint64_t earlier(uint64_t a, uint64_t b) { return a < b ? a : b; }

void guidance_publish_header(const GuidanceInput *in, GuidanceState *state, ContractHeader *h,
                         uint16_t type, uint16_t kind)
{
    *h = in->navigation->header;
    h->message_type = type; h->source_id = SOURCE_M5; h->sequence = ++state->sequence;
    h->timestamp_sample_us = in->control_time_us;
    h->timestamp_publish_us = in->evaluation_time_us; h->valid_until_us = guidance_deadline(in, kind);
    h->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}

uint8_t guidance_header_ok(const ContractHeader *h, const ContractHeader *nav, uint16_t type, uint64_t now)
{
    return (uint8_t)(h->message_type == type && h->profile_id == PRODUCT_PROFILE_ID &&
        h->schema_major == CONTRACT_SCHEMA_MAJOR && h->schema_minor == CONTRACT_SCHEMA_MINOR &&
        h->boot_id != 0 && h->boot_id == nav->boot_id && h->time_epoch == nav->time_epoch &&
        h->frame_id == FRAME_LOCAL_NED && h->clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC &&
        (h->validity_flags & (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) ==
            (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) &&
        h->timestamp_sample_us <= h->timestamp_publish_us && h->timestamp_publish_us <= now &&
        h->valid_until_us > now);
}

uint64_t guidance_deadline(const GuidanceInput *in, uint16_t kind)
{
    unsigned i;
    uint64_t t = earlier(in->evaluation_time_us + GUIDANCE_REF_TTL_US,
        in->control_time_us + GUIDANCE_PREVIEW_US * GUIDANCE_PREVIEW_POINTS);
    t = earlier(t, in->directive->header.valid_until_us);
    t = earlier(t, in->directive->directive_expires_at_us);
    t = earlier(t, in->safety_envelope->header.valid_until_us);
    t = earlier(t, in->authority_status->header.valid_until_us);
    if (in->directive->operation_code == PRODUCT_OP_MISSION) {
        t = earlier(t, in->authorized_mission->header.valid_until_us);
        t = earlier(t, in->authorized_mission->mission_expires_at_us);
    }
    if (kind != GUIDANCE_REFERENCE_NONE && in->directive->operation_code == PRODUCT_OP_PILOT)
        t = earlier(t, in->authorized_operator_input->header.valid_until_us);
    for (i = 0; i < in->geofence_count; ++i) t = earlier(t, in->geofences[i].header.valid_until_us);
    return t;
}

uint8_t guidance_home_ok(const GuidanceInput *in)
{
    const ProductHomeTargetSnapshot *h = &in->directive->payload.home;
    return (uint8_t)(h->validity == PRODUCT_HOME_VALID && h->frame_id == FRAME_LOCAL_NED &&
        h->origin_id == in->navigation->origin_id && h->state_epoch == in->navigation->state_epoch &&
        h->home_expires_at_us > in->evaluation_time_us && finite3(h->position_ned_m));
}

static uint8_t envelope_ok(const SafetyEnvelope *e)
{
    const ProductSafetyEnvelopePayload *p = &e->vehicle_extension;
    unsigned i;
    if ((p->valid_mask & 0x3Fu) != 0x3Fu || e->safety_envelope_id == 0 ||
        !isfinite(e->max_linear_speed_mps) || e->max_linear_speed_mps <= 0 ||
        !isfinite(e->max_linear_accel_mps2) || e->max_linear_accel_mps2 <= 0 ||
        !isfinite(e->max_yaw_rate_radps) || e->max_yaw_rate_radps <= 0 ||
        !isfinite(p->max_tilt_rad) || p->max_tilt_rad <= 0 || p->max_tilt_rad >= 1.5707f ||
        !isfinite(p->min_total_thrust_n) || p->min_total_thrust_n < 0 ||
        !isfinite(p->max_total_thrust_n) || p->max_total_thrust_n <= p->min_total_thrust_n) return 0;
    for (i = 0; i < 3; ++i)
        if (!isfinite(p->max_velocity_ned_mps[i]) || p->max_velocity_ned_mps[i] <= 0 ||
            !isfinite(p->max_acceleration_ned_mps2[i]) || p->max_acceleration_ned_mps2[i] <= 0 ||
            !isfinite(p->max_jerk_ned_mps3[i]) || p->max_jerk_ned_mps3[i] <= 0 ||
            !isfinite(p->max_body_rate_frd_radps[i]) || p->max_body_rate_frd_radps[i] <= 0) return 0;
    return 1;
}

Status guidance_prepare_input(const GuidanceInput *in, const GuidanceState *state, uint16_t kind)
{
    const NavigationSnapshot *n = in->navigation;
    const ContractHeader *h;
    uint64_t now = in->evaluation_time_us;
    uint32_t required;
    unsigned i;
    float norm = 0;
    if (!n || !in->directive || !in->safety_envelope || !in->authority_status ||
        now > UINT64_MAX - GUIDANCE_REF_TTL_US - 480000u) return STATUS_INVALID_ARGUMENT;
    h = &n->header;
    if (in->control_time_us != h->timestamp_sample_us || now < in->control_time_us)
        return STATUS_INVALID_ARGUMENT;
    if (!guidance_header_ok(h, h, MSG_NAVIGATION_SNAPSHOT, now) ||
        !guidance_header_ok(&in->directive->header, h, MSG_VEHICLE_DIRECTIVE, now) ||
        !guidance_header_ok(&in->safety_envelope->header, h, MSG_SAFETY_ENVELOPE, now) ||
        !guidance_header_ok(&in->authority_status->header, h, MSG_AUTHORITY_STATUS, now) ||
        now - in->control_time_us > 5000u || in->directive->directive_expires_at_us <= now)
        return STATUS_STALE_DATA;
    if (state->ctx.valid && h->boot_id == state->ctx.boot_id && h->time_epoch == state->ctx.time_epoch &&
        (in->control_time_us < state->last_state_time_us || now < state->last_evaluation_time_us ||
         (in->directive->directive_epoch == state->ctx.directive_epoch && kind == state->ctx.kind &&
          in->navigation->state_epoch == state->ctx.state_epoch &&
          in->control_time_us-state->last_state_time_us > GUIDANCE_REF_TTL_US)))
        return STATUS_STALE_DATA;
    if (!n->state_epoch || !n->origin_id || in->directive->state_epoch != n->state_epoch ||
        !in->directive->directive_epoch || !in->directive->authority_epoch ||
        in->authority_status->authority_epoch != in->directive->authority_epoch) return STATUS_STALE_DATA;
    required = in->directive->required_nav_capabilities;
    if (kind == GUIDANCE_REFERENCE_TRAJECTORY) required |= NAV_VALID_ATTITUDE |
        NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION |
        NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY;
    else if (kind == GUIDANCE_REFERENCE_PILOT_RATE) required |= NAV_VALID_BODY_RATE;
    else if (kind != GUIDANCE_REFERENCE_NONE) required |= NAV_VALID_ATTITUDE;
    if (kind == GUIDANCE_REFERENCE_ALTITUDE_PILOT || kind == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC)
        required |= NAV_VALID_VERTICAL_POSITION | NAV_VALID_VERTICAL_VELOCITY;
    if ((n->core.valid_mask & required) != required) return STATUS_NOT_READY;
    if (kind == GUIDANCE_REFERENCE_TRAJECTORY &&
        (!finite3(n->core.position_ned_m) || !finite3(n->core.velocity_ned_mps))) return STATUS_NUMERIC_ERROR;
    if ((required & NAV_VALID_VERTICAL_POSITION) && !isfinite(n->core.position_ned_m[2])) return STATUS_NUMERIC_ERROR;
    if ((required & NAV_VALID_VERTICAL_VELOCITY) && !isfinite(n->core.velocity_ned_mps[2])) return STATUS_NUMERIC_ERROR;
    if (required & NAV_VALID_ATTITUDE) {
        for (i = 0; i < 4; ++i) norm += n->core.q_nb_wxyz[i] * n->core.q_nb_wxyz[i];
        if (!isfinite(norm) || fabsf(norm-1) > 0.01f) return STATUS_NUMERIC_ERROR;
    }
    if (!envelope_ok(in->safety_envelope) ||
        in->directive->safety_envelope_id != in->safety_envelope->safety_envelope_id)
        return STATUS_INVALID_ARGUMENT;
    if (in->directive->payload.target_valid_mask &
        ~(PRODUCT_DIRECTIVE_TARGET_POSITION | PRODUCT_DIRECTIVE_TARGET_YAW)) return STATUS_INVALID_ARGUMENT;
    if ((in->directive->payload.target_valid_mask & PRODUCT_DIRECTIVE_TARGET_POSITION) &&
        !finite3(in->directive->payload.target_position_ned_m)) return STATUS_NUMERIC_ERROR;
    if ((in->directive->payload.target_valid_mask & PRODUCT_DIRECTIVE_TARGET_YAW) &&
        !isfinite(in->directive->payload.target_yaw_rad)) return STATUS_NUMERIC_ERROR;
    if (in->geofence_count > PRODUCT_MAX_GEOFENCE_POLYGONS ||
        (in->geofence_count && !in->geofences)) return STATUS_INVALID_ARGUMENT;
    for (i = 0; i < in->geofence_count; ++i) {
        const ProductGeofenceSnapshot *f = &in->geofences[i];
        if (!guidance_header_ok(&f->header, h, MSG_PRODUCT_GEOFENCE_SNAPSHOT, now) ||
            f->origin_id != n->origin_id || f->state_epoch != n->state_epoch ||
            !fence_geometry_valid(f)) return STATUS_INVALID_ARGUMENT;
    }
    if (in->directive->operation_code == PRODUCT_OP_MISSION) {
        const AuthorizedMissionSnapshot *m = in->authorized_mission;
        if (!m || !guidance_header_ok(&m->header, h, MSG_AUTHORIZED_MISSION_SNAPSHOT, now) ||
            m->mission_expires_at_us <= now || !m->mission_epoch || !m->execution_token ||
            m->state_epoch != n->state_epoch || m->origin_id != n->origin_id ||
            m->authority_epoch != in->directive->authority_epoch ||
            m->directive_epoch != in->directive->directive_epoch) return STATUS_STALE_DATA;
        if (!m->mission.waypoint_count || m->mission.waypoint_count > PRODUCT_MAX_MISSION_WAYPOINTS ||
            m->current_item_index >= m->mission.waypoint_count ||
            m->current_item_id != m->mission.waypoints[m->current_item_index].waypoint_id)
            return STATUS_INVALID_ARGUMENT;
        if (!finite3(m->mission.waypoints[m->current_item_index].position_ned_m) ||
            !isfinite(m->mission.waypoints[m->current_item_index].yaw_rad) ||
            !isfinite(m->mission.waypoints[m->current_item_index].speed_limit_mps) ||
            m->mission.waypoints[m->current_item_index].speed_limit_mps <= 0) return STATUS_NUMERIC_ERROR;
    }
    if (in->directive->operation_code == PRODUCT_OP_PILOT) {
        const AuthorizedOperatorInput *op = in->authorized_operator_input;
        if (!op || !guidance_header_ok(&op->header, h, MSG_AUTHORIZED_OPERATOR_INPUT, now) ||
            op->authority_epoch != in->directive->authority_epoch ||
            op->input_layout_id != PRODUCT_OPERATOR_INPUT_LAYOUT_ID ||
            op->selected_source_id != in->authority_status->selected_source_id ||
            op->authority_class != 1u || in->authority_status->authority_owner != 1u)
            return STATUS_NOT_PERMITTED;
        if (!isfinite(op->payload.roll_norm) || fabsf(op->payload.roll_norm) > 1 ||
            !isfinite(op->payload.pitch_norm) || fabsf(op->payload.pitch_norm) > 1 ||
            !isfinite(op->payload.yaw_norm) || fabsf(op->payload.yaw_norm) > 1 ||
            !isfinite(op->payload.throttle_norm) || op->payload.throttle_norm < 0 ||
            op->payload.throttle_norm > 1) return STATUS_NUMERIC_ERROR;
    }
    return STATUS_OK;
}
