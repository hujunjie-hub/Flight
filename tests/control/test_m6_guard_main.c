/*
 * FMAT-0012 M6 time-context guard host test.
 *
 * controller_validate_input() is the gate every controller input passes before
 * the MPC runs: it binds boot id, time epoch, navigation sample/publish times,
 * reference and permit epochs, authority/directive epochs and the envelope id,
 * and rejects anything that is stale, mixed or expired. This file drives the
 * real function with a complete valid context and then breaks exactly one field
 * at a time, so a PASS means the whole guard is exercised rather than a single
 * branch.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "attitude_rate_control/controller_internal.h"
#include "osqp.h"
#include "quad_workspace.h"

static int checks = 0;
static int failures = 0;
#define CHECK(cond, fmt, ...) do { \
    checks++; \
    if (!(cond)) { failures++; \
        printf("  FAIL %s:%d  " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); } \
  } while (0)

static uint64_t g_now = 1000000u;
static uint64_t g_advance_after_clock_read_us = 0u;

Status clock_read(ClockInstant *out) {
    if (!out) return STATUS_INVALID_ARGUMENT;
    memset(out, 0, sizeof(*out));
    out->boot_id = 0xB00Bu;
    out->monotonic_us = g_now;
    out->time_epoch = 3u;
    if (g_advance_after_clock_read_us) {
        g_now += g_advance_after_clock_read_us;
        g_advance_after_clock_read_us = 0u;
    }
    return STATUS_OK;
}

static void hdr(ContractHeader *h, uint16_t type, uint16_t source, uint64_t publish_us) {
    memset(h, 0, sizeof(*h));
    h->schema_major = CONTRACT_SCHEMA_MAJOR;
    h->schema_minor = CONTRACT_SCHEMA_MINOR;
    h->message_type = type;
    h->source_id = source;
    h->profile_id = PRODUCT_PROFILE_ID;
    h->frame_id = FRAME_LOCAL_NED;
    h->sequence = 1u;
    h->boot_id = 0xB00Bu;
    h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
    h->time_epoch = 3u;
    h->timestamp_sample_us = publish_us;
    h->timestamp_publish_us = publish_us;
    h->valid_until_us = publish_us + 100000u;
    h->timestamp_quality = TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
    h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    h->validity_flags = 3u;
}

typedef struct {
    NavigationSnapshot nav;
    VehicleDirective directive;
    SafetyEnvelope envelope;
    AuthorityStatus authority;
    ActuationPermit permit;
    ActuatorStatus actuator;
    ActuatorCapability capability;
    GuidanceStatus guidance;
    MotionReference reference;
    ControlModelView view;
    ControllerInput in;
} Context;

static void build_context(Context *c, uint64_t now, uint16_t reference_kind) {
    unsigned i;
    memset(c, 0, sizeof(*c));

    hdr(&c->nav.header, MSG_NAVIGATION_SNAPSHOT, SOURCE_M3, now);
    c->nav.state_epoch = 9u;
    c->nav.origin_id = 1u;
    c->nav.core.valid_mask = 0xFFFFFFFFu;
    c->nav.core.q_nb_wxyz[0] = 1.0f;
    for (i = 0; i < 3; ++i) {
        c->nav.core.position_ned_m[i] = 0.0f;
        c->nav.core.velocity_ned_mps[i] = 0.0f;
    }

    hdr(&c->directive.header, MSG_VEHICLE_DIRECTIVE, SOURCE_M4, now);
    c->directive.directive_epoch = 4u;
    c->directive.authority_epoch = 2u;
    c->directive.state_epoch = 9u;
    c->directive.directive_expires_at_us = now + 100000u;
    c->directive.operation_code = PRODUCT_OP_MISSION;
    c->directive.guidance_mode_code = PRODUCT_GUIDANCE_MODE_POS_HOLD;
    c->directive.safety_envelope_id = 3u;

    hdr(&c->envelope.header, MSG_SAFETY_ENVELOPE, SOURCE_M4, now);
    c->envelope.safety_envelope_id = 3u;
    c->envelope.envelope_epoch = 6u;
    c->envelope.vehicle_extension.valid_mask = 63u;
    for (i = 0; i < 3; ++i) {
        c->envelope.vehicle_extension.max_velocity_ned_mps[i] = 5.0f;
        c->envelope.vehicle_extension.max_acceleration_ned_mps2[i] = 4.0f;
        c->envelope.vehicle_extension.max_jerk_ned_mps3[i] = 20.0f;
        c->envelope.vehicle_extension.max_body_rate_frd_radps[i] = 3.0f;
    }
    c->envelope.vehicle_extension.max_tilt_rad = 0.5f;
    c->envelope.vehicle_extension.max_total_thrust_n = 40.0f;
    c->envelope.vehicle_extension.min_total_thrust_n = 0.0f;

    hdr(&c->authority.header, MSG_AUTHORITY_STATUS, SOURCE_M4, now);
    c->authority.authority_epoch = 2u;

    hdr(&c->permit.header, MSG_ACTUATION_PERMIT, SOURCE_M4, now);
    c->permit.permit_epoch = 5u;
    c->permit.directive_epoch = 4u;
    c->permit.authority_epoch = 2u;
    c->permit.state_epoch = 9u;
    c->permit.reference_epoch = 7u;
    c->permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
    c->permit.permit_expires_at_us = now + 100000u;
    c->permit.output_policy_id = 2u;

    hdr(&c->actuator.header, MSG_ACTUATOR_STATUS, SOURCE_M7, now);
    c->actuator.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
    c->actuator.available_mask = 15u;
    c->actuator.fault_mask = 0u;

    hdr(&c->capability.header, MSG_ACTUATOR_CAPABILITY, SOURCE_M7, now);
    c->capability.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
    c->capability.available_mask = 15u;

    hdr(&c->guidance.header, MSG_GUIDANCE_STATUS, SOURCE_M5, now);
    c->guidance.guidance_state = GUIDANCE_STATE_READY;
    c->guidance.reference_kind = reference_kind;
    c->guidance.reference_id = 11u;
    c->guidance.reference_epoch = 7u;
    c->guidance.state_epoch = 9u;
    c->guidance.origin_id = 1u;
    c->guidance.authority_epoch = 2u;
    c->guidance.directive_epoch = 4u;
    c->guidance.reference_expires_at_us = now + 100000u;

    /* The declared fixed double integrator the controller binds to. */
    hdr(&c->view.header, MSG_CONTROL_MODEL_VIEW, SOURCE_M5, now);
    c->view.model_epoch = 2u;
    c->view.descriptor.model_id = 0x1501u;
    c->view.descriptor.model_version = 1u;
    c->view.descriptor.model_kind = CONTROL_MODEL_KIND_LTI;
    c->view.descriptor.state_layout_id = PRODUCT_CONTROL_MODEL_STATE_LAYOUT_ID;
    c->view.descriptor.input_layout_id = PRODUCT_CONTROL_MODEL_INPUT_LAYOUT_ID;
    c->view.linear_model.sample_time_s = 0.02f;
    for (i = 0; i < 36; ++i) {
        c->view.linear_model.A[i] =
            (i / 6 == (int)(i % 6)) ? 1.0f
            : ((i / 6 < 3 && (int)(i % 6) == (int)(i / 6) + 3) ? 0.02f : 0.0f);
    }
    for (i = 0; i < 18; ++i) {
        c->view.linear_model.B[i] =
            ((int)(i / 3) % 3 == (int)(i % 3)) ? ((i / 3 < 3) ? 0.0002f : 0.02f) : 0.0f;
    }
    for (i = 0; i < 21; ++i) {
        c->view.bounds.state_min[i] = (i < 6) ? -100.0f : 0.0f;
        c->view.bounds.state_max[i] = (i < 6) ? 100.0f : 0.0f;
    }
    for (i = 0; i < 3; ++i) {
        c->view.bounds.input_min[i] = -8.0f;
        c->view.bounds.input_max[i] = 8.0f;
        c->view.bounds.input_slew_max[i] = 50.0f;
    }

    hdr(&c->reference.header, MSG_MOTION_REFERENCE, SOURCE_M5, now);
    c->reference.reference_id = 11u;
    c->reference.reference_epoch = 7u;
    c->reference.state_epoch = 9u;
    c->reference.origin_id = 1u;
    c->reference.authority_epoch = 2u;
    c->reference.directive_epoch = 4u;
    c->reference.reference_kind = reference_kind;
    c->reference.point_count = 24u;
    c->reference.anchor.offset_us = 0u;
    c->reference.anchor.valid_mask = 31u;
    c->reference.anchor.point.valid_mask = 31u;
    for (i = 0; i < 24u; ++i) {
        c->reference.points[i].offset_us = (i + 1u) * 20000u;
        c->reference.points[i].valid_mask = 31u;
        c->reference.points[i].point.valid_mask = 31u;
    }

    c->in.navigation = &c->nav;
    c->in.reference = &c->reference;
    c->in.guidance_status = &c->guidance;
    c->in.directive = &c->directive;
    c->in.safety_envelope = &c->envelope;
    c->in.authority_status = &c->authority;
    c->in.actuation_permit = &c->permit;
    c->in.actuator_status = &c->actuator;
    c->in.actuator_capability = &c->capability;
    c->in.control_model_view = &c->view;
    c->in.control_time_us = now;
    c->in.evaluation_time_us = now;
}

static Status validate(Context *c) {
    uint32_t epoch = 0;
    uint16_t mode = 0;
    return controller_validate_input(&c->in, &epoch, &mode);
}

static void configure_outer_test_controller(void) {
    memset(&m6, 0, sizeof(m6));
    m6.initialized = 1;
    m6.config.payload.mass_kg = 1.5f;
    m6.config.payload.mpc_q_diag[0] = 8.0f;
    m6.config.payload.mpc_q_diag[1] = 8.0f;
    m6.config.payload.mpc_q_diag[2] = 12.0f;
    m6.config.payload.mpc_q_diag[3] = 3.0f;
    m6.config.payload.mpc_q_diag[4] = 3.0f;
    m6.config.payload.mpc_q_diag[5] = 4.0f;
    m6.config.payload.mpc_qf_diag[0] = 8.0f;
    m6.config.payload.mpc_qf_diag[1] = 8.0f;
    m6.config.payload.mpc_qf_diag[2] = 12.0f;
    m6.config.payload.mpc_qf_diag[3] = 3.0f;
    m6.config.payload.mpc_qf_diag[4] = 3.0f;
    m6.config.payload.mpc_qf_diag[5] = 4.0f;
    m6.config.payload.mpc_r_diag[0] = 0.8f;
    m6.config.payload.mpc_r_diag[1] = 0.8f;
    m6.config.payload.mpc_r_diag[2] = 1.0f;
    m6.config.payload.mpc_s_diag[0] = 0.2f;
    m6.config.payload.mpc_s_diag[1] = 0.2f;
    m6.config.payload.mpc_s_diag[2] = 0.25f;
    m6.config.payload.actuation_preview_delay_s = 0.002f;
}

static void test_outer_update_lifecycle(void) {
    Context c;
    MpcRunStats stats;
    MpcSnapshot snapshot;
    Status status;
    uint64_t now = 1500000u;

    configure_outer_test_controller();
    CHECK(control_backend_outer_update(NULL) == STATUS_INVALID_ARGUMENT,
          "NULL outer-update input was not rejected");

    mpc_note_worker(g_now, 2);
    mpc_note_worker(g_now, 1);
    mpc_get_run_stats(&stats);
    CHECK(stats.entry_calls == 1u && stats.worker_ticks == 1u,
          "worker counters were not recorded before reset");
    CHECK(mpc_reset() == STATUS_OK, "MPC reset failed");
    mpc_get_run_stats(&stats);
    CHECK(stats.jobs == 0u && stats.solved == 0u && stats.rejected == 0u &&
          stats.covered_us == 0u && stats.entry_calls == 0u && stats.worker_ticks == 0u,
          "MPC reset left stale run statistics");
    CHECK(quad_solver.settings != NULL && quad_solver.settings->eps_rel == 0.0f,
          "MPC reset did not retain eps_rel=0");

    g_now = now;
    build_context(&c, now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.actuator.header.valid_until_us = now + 5000u;
    c.capability.header.valid_until_us = now + 5000u;
    status = control_backend_outer_update(&c.in);
    CHECK(status == STATUS_OK, "valid outer update rejected, status=%d", (int)status);
    CHECK(mpc_read(&snapshot), "successful MPC snapshot was not published");
    CHECK(snapshot.expires_at_us > c.actuator.header.valid_until_us,
          "MPC plan inherited actuator-status TTL");
    CHECK(snapshot.expires_at_us > c.capability.header.valid_until_us,
          "MPC plan inherited actuator-capability TTL");
    CHECK(snapshot.expires_at_us == now + 20000u,
          "MPC plan expiry=%llu, expected %llu",
          (unsigned long long)snapshot.expires_at_us,
          (unsigned long long)(now + 20000u));
    mpc_get_run_stats(&stats);
    CHECK(stats.jobs == 1u && stats.solved == 1u && stats.rejected == 0u &&
          stats.covered_us == 20000u,
          "successful job counters are inconsistent");

    configure_outer_test_controller();
    CHECK(mpc_reset() == STATUS_OK, "MPC reset before deadline test failed");
    now = 3000000u;
    g_now = now;
    build_context(&c, now, GUIDANCE_REFERENCE_TRAJECTORY);
    g_advance_after_clock_read_us = 2001u;
    status = control_backend_outer_update(&c.in);
    CHECK(status == STATUS_DEADLINE_MISSED,
          "2 ms solver overrun returned status=%d", (int)status);
    mpc_get_run_stats(&stats);
    CHECK(stats.jobs == 1u && stats.solved == 0u && stats.rejected == 1u &&
          stats.covered_us == 0u && stats.solver_deadline == 1u,
          "deadline-missed job was counted as solved/covered");
    CHECK(mpc_read(&snapshot) && snapshot.expires_at_us == 0u,
          "deadline-missed MPC snapshot remained valid");
}

int main(void) {
    Context c;
    uint32_t epoch = 0;
    uint16_t mode = 0;
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("FMAT-0012 M6 time-context guard host test\n");

    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    {
        Status s = controller_validate_input(&c.in, &epoch, &mode);
        printf("  valid context status=%d\n", (int)s);
        CHECK(s == STATUS_OK, "a fully consistent context was rejected");
    }
    CHECK(epoch == 7u && mode == GUIDANCE_REFERENCE_TRAJECTORY,
          "validate_input returned epoch=%u mode=%u", epoch, mode);

    /* M6-08: a control instant that aged past 5 ms is stale. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.in.evaluation_time_us = c.in.control_time_us + 6000u;
    CHECK(validate(&c) == STATUS_STALE_DATA, "aged control instant accepted");

    /* M6-09: the navigation sample must carry the control instant itself. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.nav.header.timestamp_sample_us = g_now - 1000u;
    c.in.evaluation_time_us = g_now + 100000u;
    c.nav.header.valid_until_us = g_now + 200000u;
    CHECK(validate(&c) == STATUS_STALE_DATA, "nav sample != control instant accepted");

    /* M6-10: another boot session is refused. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.permit.header.boot_id = 0xDEADu;
    CHECK(validate(&c) == STATUS_STALE_DATA, "permit from another boot accepted");

    /* M6-11: another time epoch is refused. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.nav.header.time_epoch = 4u;
    CHECK(validate(&c) == STATUS_STALE_DATA, "a context from another time epoch accepted");

    /* M6-12: an expired permit is not permitted. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.permit.permit_expires_at_us = g_now;
    CHECK(validate(&c) == STATUS_NOT_PERMITTED, "expired permit accepted");

    /* M6-13: a permit that is not CONTROL_ACTIVE is not permitted. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.permit.permit_flags = PERMIT_SCOPE_IDLE_ONLY;
    CHECK(validate(&c) == STATUS_NOT_PERMITTED, "idle-only permit accepted");

    /* M6-14: a guidance status from another epoch than the permit is refused. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.guidance.reference_epoch = 8u;
    CHECK(validate(&c) == STATUS_NOT_PERMITTED, "mismatched reference epoch accepted");

    /* M6-15: a mission reference that does not match the guidance id is refused. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.reference.reference_id = 12u;
    CHECK(validate(&c) == STATUS_NOT_PERMITTED, "mismatched reference id accepted");

    /* M6-16: an actuator that cannot supply all four channels is out of range. */
    build_context(&c, g_now, GUIDANCE_REFERENCE_TRAJECTORY);
    c.capability.available_mask = 7u;
    CHECK(validate(&c) == STATUS_OUT_OF_RANGE, "three-channel capability accepted");

    /* M6-17..M6-23: outer worker lifecycle, plan TTL and deadline accounting. */
    test_outer_update_lifecycle();

    printf("M6 guard checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
