/*
 * FMAT-0013 M7 actuation transaction host test.
 *
 * Drives the real actuator guard (middleware/actuator/output_guard/actuator_guard.c)
 * with a stub monotonic clock and a recording commit driver. The guard is the
 * component that actually consumes an ActuationPermit and turns an ActuatorDemand
 * into a physical frame, so these cases cover the rules Issue #13 asks for on the
 * consumer side: permit generation/watermark semantics, no resurrection of a
 * revoked permit, ordering, expiry, and the separation between publishing a
 * demand and applying a frame at the 2.5 ms boundary.
 *
 * Ground stop is intentionally absent: the firmware has no
 * GroundStopCertificate, no thrust ramp and no ZERO_TARGET_APPLIED state, so it
 * cannot be tested here without inventing a parallel implementation.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "output_guard/actuator_guard.h"
#include "contract_checks/algorithm_checks.h"

#define CHECKS 1
static int checks = 0;
static int failures = 0;

#define CHECK(cond, fmt, ...) do { \
    checks++; \
    if (!(cond)) { failures++; \
        printf("  FAIL %s:%d  " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); } \
  } while (0)

static uint64_t g_boot = 0xA1B2C3D4u;
static uint32_t g_epoch = 7u;
static uint64_t g_now_us = 1000000u;
static unsigned g_commit_calls = 0;
static unsigned g_inhibit_calls = 0;
static uint16_t g_last_commands[4];
static uint64_t g_apply_latency_us = 500u;

Status clock_read(ClockInstant *out) {
    if (!out) return STATUS_INVALID_ARGUMENT;
    memset(out, 0, sizeof(*out));
    out->boot_id = g_boot;
    out->monotonic_us = g_now_us;
    out->time_epoch = g_epoch;
    return STATUS_OK;
}

static void stub_lock(void) {}
static void stub_unlock(void) {}
static void stub_inhibit(void) { g_inhibit_calls++; }
static Status stub_commit(const uint16_t commands[4], uint64_t *actual_apply_us) {
    g_commit_calls++;
    memcpy(g_last_commands, commands, sizeof(g_last_commands));
    if (actual_apply_us) *actual_apply_us = g_now_us + g_apply_latency_us;
    return STATUS_OK;
}

static ProductActuatorConfig make_config(void) {
    ProductActuatorConfig config;
    unsigned m;
    memset(&config, 0, sizeof(config));
    config.profile_hash = 0x12345678u;
    config.config_version = 3u;
    config.calibration_hash = 0x44445555u;
    config.calibration_set_id = 2u;
    config.payload.output_policy_id = 2u; /* SIM_CONTROLLED_STOP: the only policy with a driver */
    config.payload.calibration_version = 4u;
    config.payload.esc_bit_rate = 300000u;
    for (m = 0; m < 4; ++m) {
        config.payload.thrust_curve_count[m] = 2u;
        config.payload.thrust_curve_n[m][0] = 0.0f;
        config.payload.thrust_curve_n[m][1] = 10.0f;
        config.payload.dshot_curve_code[m][0] = 48u;
        config.payload.dshot_curve_code[m][1] = 2047u;
        config.payload.min_thrust_n[m] = 0.0f;
        config.payload.max_thrust_n[m] = 10.0f;
        config.payload.max_slew_nps[m] = 1000.0f; /* 2.5 N per 2.5 ms frame */
        config.payload.motor_to_timer_channel[m] = (uint8_t)(m + 1u);
    }
    return config;
}

static void set_now(uint64_t now_us) { g_now_us = now_us; }

static void make_header(ContractHeader *h, uint16_t type, uint16_t source,
                        uint32_t sequence, uint64_t publish_us) {
    memset(h, 0, sizeof(*h));
    h->schema_major = CONTRACT_SCHEMA_MAJOR;
    h->schema_minor = CONTRACT_SCHEMA_MINOR;
    h->message_type = type;
    h->source_id = source;
    h->profile_id = PRODUCT_PROFILE_ID;
    h->frame_id = FRAME_BODY_FRD;
    h->sequence = sequence;
    h->boot_id = g_boot;
    h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
    h->time_epoch = g_epoch;
    h->timestamp_sample_us = publish_us;
    h->timestamp_publish_us = publish_us;
    h->valid_until_us = publish_us + 5000u;
    h->event_time_uncertainty_us = 0u;
    h->timestamp_quality = TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
    h->timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
    h->validity_flags = 3u;
}

static ActuationPermit make_permit(uint32_t epoch, uint32_t sequence, uint64_t publish_us,
                                   uint64_t expires_us, uint32_t flags) {
    ActuationPermit permit;
    memset(&permit, 0, sizeof(permit));
    make_header(&permit.header, MSG_ACTUATION_PERMIT, SOURCE_M4, sequence, publish_us);
    permit.permit_epoch = epoch;
    permit.directive_epoch = 3u;
    permit.authority_epoch = 5u;
    permit.state_epoch = 9u;
    permit.reference_epoch = 11u;
    permit.permit_flags = flags;
    permit.permit_expires_at_us = expires_us;
    permit.reason_code = 0u;
    permit.output_policy_id = 2u;
    return permit;
}

static ActuatorDemand make_demand(const ActuationPermit *permit, uint32_t sequence,
                                  uint64_t publish_us, float thrust_n) {
    ActuatorDemand demand;
    unsigned m;
    memset(&demand, 0, sizeof(demand));
    make_header(&demand.header, MSG_ACTUATOR_DEMAND, SOURCE_M6, sequence, publish_us);
    demand.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
    demand.valid_channel_mask = 15u;
    demand.permit_epoch = permit->permit_epoch;
    demand.directive_epoch = permit->directive_epoch;
    demand.authority_epoch = permit->authority_epoch;
    demand.state_epoch = permit->state_epoch;
    demand.reference_epoch = permit->reference_epoch;
    demand.control_sequence = sequence;
    demand.demand_expires_at_us = publish_us + 5000u;
    demand.demand.valid_mask = 15u;
    for (m = 0; m < 4; ++m) demand.demand.motor_thrust_n[m] = thrust_n;
    return demand;
}

static ActuatorStatus g_status;

static void fresh_guard(const ProductActuatorConfig *config) {
    actuator_stop();
    g_commit_calls = 0;
    g_inhibit_calls = 0;
    memset(g_last_commands, 0, sizeof(g_last_commands));
    set_now(1000000u);
    CHECK(actuator_init(config) == STATUS_OK, "actuator_init rejected a valid configuration");
}

/* M7-01 / M7-02 / M7-03 / M7-04: publish versus apply, boundary cadence and slew. */
static void case_frame_application(void) {
    ProductActuatorConfig config = make_config();
    ActuationPermit permit;
    ActuatorDemand demand;
    ActuatorCommitReceipt receipt;
    Status status;
    uint64_t frame_us = 1000000u;

    printf("M7-01..04 frame boundary and apply separation\n");
    fresh_guard(&config);

    set_now(frame_us);
    permit = make_permit(5u, 1u, frame_us, frame_us + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, frame_us) == STATUS_OK, "valid permit rejected");
    demand = make_demand(&permit, 1u, frame_us, 1.0f);
    status = actuator_apply(&demand, &permit, &g_status);
    CHECK(status == STATUS_OK, "valid demand rejected, status=%d", (int)status);
    CHECK(g_commit_calls == 0u,
          "actuator_apply produced %u commits; publishing a demand is not a commit", g_commit_calls);

    status = actuator_guard_frame_boundary(frame_us);
    CHECK(status == STATUS_OK, "frame boundary rejected the staged demand, status=%d", (int)status);
    CHECK(g_commit_calls == 1u, "expected exactly one commit, got %u", g_commit_calls);

    CHECK(actuator_get_commit_receipt(&receipt) == STATUS_OK, "commit receipt unavailable");
    CHECK(receipt.result_code == ACTUATOR_RECEIPT_COMMITTED, "receipt result=%u", receipt.result_code);
    CHECK(receipt.commit_time_us == frame_us, "commit_time_us=%llu", (unsigned long long)receipt.commit_time_us);
    CHECK(receipt.estimated_apply_time_us >= receipt.commit_time_us,
          "apply time %llu precedes commit time %llu",
          (unsigned long long)receipt.estimated_apply_time_us,
          (unsigned long long)receipt.commit_time_us);
    CHECK(receipt.receipt_expires_at_us == receipt.commit_time_us + 5000u,
          "receipt expiry is not commit+5ms");
    CHECK(fabsf(receipt.committed_demand.motor_thrust_n[0] - 1.0f) < 1e-4f,
          "committed thrust %.4f != demand 1.0", receipt.committed_demand.motor_thrust_n[0]);
    CHECK(receipt.applied_control_sequence == 1u, "receipt sequence=%u", receipt.applied_control_sequence);

    /* Same generation again inside the 2.5 ms cadence must not re-apply. */
    set_now(frame_us + 500u);
    demand = make_demand(&permit, 2u, frame_us + 500u, 2.0f);
    CHECK(actuator_apply(&demand, &permit, &g_status) == STATUS_OK, "renewed demand rejected");
    status = actuator_guard_frame_boundary(frame_us + 500u);
    CHECK(status == STATUS_DEADLINE_MISSED, "a 500 us frame was applied, status=%d", (int)status);
    CHECK(g_commit_calls == 1u, "duplicate frame produced %u commits", g_commit_calls);

    /* The next legal boundary is 2.5 ms after the previous one. */
    set_now(frame_us + 2500u);
    demand = make_demand(&permit, 3u, frame_us + 2500u, 10.0f);
    CHECK(actuator_apply(&demand, &permit, &g_status) == STATUS_OK, "second demand rejected");
    status = actuator_guard_frame_boundary(frame_us + 2500u);
    CHECK(status == STATUS_OK, "second frame boundary failed, status=%d", (int)status);
    CHECK(g_commit_calls == 2u, "expected two commits, got %u", g_commit_calls);
    CHECK(actuator_get_commit_receipt(&receipt) == STATUS_OK, "second receipt unavailable");
    /* 2.5 ms at 1000 N/s lets the force move by 2.5 N from the previous 1.0 N. */
    CHECK(fabsf(receipt.committed_demand.motor_thrust_n[0] - 3.5f) < 1e-3f,
          "slew did not bound the jump: %.4f != 3.5", receipt.committed_demand.motor_thrust_n[0]);
    CHECK((receipt.limit_flags & 1u) != 0u, "slew clipping was not reported in limit_flags");
}

/* M7-05 / M7-06 / M7-07: generation watermark, revocation and ordering. */
static void case_permit_generations(void) {
    ProductActuatorConfig config = make_config();
    ActuationPermit permit;
    uint64_t t = 2000000u;

    printf("M7-05..07 permit watermark, revocation and ordering\n");
    fresh_guard(&config);
    set_now(t);

    permit = make_permit(5u, 1u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_OK, "initial permit rejected");

    /* Revocation is published as generation 6 with the revoked flag. */
    permit = make_permit(6u, 2u, t, t + 100000u, PERMIT_FLAG_REVOKED);
    CHECK(actuator_update_permit(&permit, t) == STATUS_OK, "revocation rejected");
    CHECK(g_inhibit_calls > 0u, "revocation did not inhibit the output");

    /* Replaying the pre-revocation generation must never re-establish output. */
    permit = make_permit(5u, 3u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_STALE_DATA,
          "a revoked generation was accepted again");

    /* The revoked generation itself cannot be renewed as an active permit. */
    permit = make_permit(6u, 4u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_NOT_PERMITTED,
          "a revoked generation was resurrected as active");

    /* Newer generation is accepted, then an out-of-order renewal is refused. */
    permit = make_permit(7u, 100u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_OK, "generation 7 rejected");
    permit = make_permit(7u, 50u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_STALE_DATA,
          "an out-of-order renewal of the same generation was accepted");
}

/* M7-08 / M7-09 / M7-10: clock binding, expiry and stale demands. */
static void case_binding_and_expiry(void) {
    ProductActuatorConfig config = make_config();
    ActuationPermit permit;
    ActuatorDemand demand;
    uint64_t t = 3000000u;
    Status status;

    printf("M7-08..10 boot/time epoch binding, permit expiry, stale demand\n");
    fresh_guard(&config);
    set_now(t);

    permit = make_permit(5u, 1u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    permit.header.boot_id = g_boot + 1u;
    CHECK(actuator_update_permit(&permit, t) == STATUS_NOT_PERMITTED,
          "a permit from another boot was accepted");

    permit = make_permit(5u, 1u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    permit.header.time_epoch = g_epoch + 1u;
    CHECK(actuator_update_permit(&permit, t) == STATUS_NOT_PERMITTED,
          "a permit from another time epoch was accepted");

    /* An expired permit must stop being bound even though the message is current. */
    permit = make_permit(8u, 2u, t, t, PERMIT_SCOPE_CONTROL_ACTIVE);
    g_inhibit_calls = 0;
    (void)actuator_update_permit(&permit, t);
    CHECK(g_inhibit_calls > 0u, "an expired permit did not inhibit the output");
    demand = make_demand(&permit, 10u, t, 5.0f);
    status = actuator_apply(&demand, &permit, &g_status);
    CHECK(status == STATUS_NOT_PERMITTED, "an expired permit still actuated, status=%d", (int)status);

    /* A fresh permit keeps working, but a demand that aged past 5 ms must not. */
    permit = make_permit(9u, 3u, t, t + 100000u, PERMIT_SCOPE_CONTROL_ACTIVE);
    CHECK(actuator_update_permit(&permit, t) == STATUS_OK, "fresh permit rejected");
    demand = make_demand(&permit, 11u, t, 5.0f);
    demand.header.valid_until_us = t + 60000u; /* keep the header valid to isolate the age rule */
    set_now(t + 6000u);
    status = actuator_apply(&demand, &permit, &g_status);
    CHECK(status == STATUS_NOT_PERMITTED, "a 6 ms old demand was accepted, status=%d", (int)status);
    CHECK(actuator_guard_state() == ACTUATOR_GUARD_SAFE_ACTION,
          "stale demand did not enter SAFE_ACTION, state=%u", actuator_guard_state());
}

int main(void) {
    ActuatorDriver driver;
    memset(&driver, 0, sizeof(driver));
    driver.lock = stub_lock;
    driver.unlock = stub_unlock;
    driver.inhibit = stub_inhibit;
    driver.commit = stub_commit;
    printf("FMAT-0013 M7 actuator guard host test\n");
    if (actuator_guard_set_driver(&driver) != STATUS_OK) {
        printf("could not install the host driver\n");
        return 2;
    }
    case_frame_application();
    case_permit_generations();
    case_binding_and_expiry();
    printf("M7 checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
