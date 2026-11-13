#include <string.h>
#include <stdint.h>
#include "fms_interface.h"
#include "contract_checks/algorithm_checks.h"

/* M4 output adapter: a ground-stop certificate is emitted only for the two
 * ground-only disarm decisions that carry a fresh, matching revoked permit.
 * Airborne failsafe revocations must never be translated into a ground ramp. */
Status fms_build_ground_stop_certificate(const FmsOutputView *out, uint64_t now_us,
                                         GroundStopCertificate *certificate)
{
    const ActuationPermit *permit;
    const DecisionEvent *decision;
    uint64_t expires_at;
    uint8_t eligible = 0u;

    if (!out || !certificate) return STATUS_INVALID_ARGUMENT;
    memset(certificate, 0, sizeof(*certificate));
    permit = &out->actuation_permit;
    decision = &out->decision_event;

    if (!header_valid(&permit->header, now_us) ||
        permit->header.message_type != MSG_ACTUATION_PERMIT ||
        permit->header.source_id != SOURCE_M4 ||
        permit->permit_flags != PERMIT_FLAG_REVOKED ||
        permit->permit_epoch == 0u || permit->authority_epoch == 0u ||
        permit->directive_epoch == 0u || permit->output_policy_id != 2u)
        return STATUS_NOT_READY;

    if (!header_valid(&decision->header, now_us) ||
        decision->header.message_type != MSG_DECISION_EVENT ||
        decision->header.source_id != SOURCE_M4 ||
        decision->header.boot_id != permit->header.boot_id ||
        decision->header.time_epoch != permit->header.time_epoch ||
        !header_valid(&out->landed_state.header, now_us) ||
        !header_valid(&out->state_snapshot.header, now_us) ||
        out->landed_state.header.boot_id != permit->header.boot_id ||
        out->state_snapshot.header.boot_id != permit->header.boot_id ||
        out->landed_state.header.time_epoch != permit->header.time_epoch ||
        out->state_snapshot.header.time_epoch != permit->header.time_epoch)
        return STATUS_STALE_DATA;

    if (out->landed_state.state != LANDED_STATE_LANDED ||
        out->state_snapshot.arm_state != ARM_STATE_DISARMED)
        return STATUS_NOT_PERMITTED;

    if (permit->reason_code == FMS_PERMIT_REVOKE_REASON_FIRST_REFERENCE_GROUND_DISARM &&
        decision->decision_code == DECISION_FORCE_SAFE_ACTION &&
        decision->reason_code == FMS_DECISION_REASON_FIRST_REFERENCE_GROUND_DISARM)
        eligible = 1u;
    if (permit->reason_code == FMS_PERMIT_REVOKE_REASON_COMMAND_DISARM &&
        decision->decision_code == DECISION_ACCEPT &&
        decision->reason_code == 0u)
        eligible = 1u;
    if (!eligible) return STATUS_NOT_PERMITTED;

    expires_at = (UINT64_MAX - now_us < 200000u) ? UINT64_MAX : now_us + 200000u;
    certificate->header = permit->header;
    certificate->header.message_type = MSG_GROUND_STOP_CERTIFICATE;
    certificate->header.timestamp_sample_us = now_us;
    certificate->header.timestamp_publish_us = now_us;
    certificate->header.valid_until_us = expires_at;
    certificate->stop_code = ACTUATOR_GROUND_STOP_CODE_ZERO;
    certificate->lifecycle = ACTUATOR_GROUND_STOP_LIFECYCLE_LANDED;
    certificate->ramp_nps = ACTUATOR_GROUND_STOP_MAX_RAMP_NPS;
    certificate->authority_epoch = permit->authority_epoch;
    certificate->directive_epoch = permit->directive_epoch;
    certificate->certificate_expires_at_us = expires_at;
    return STATUS_OK;
}
