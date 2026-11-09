/*
 * Sensor Integrity Manager (FDIR) - source-level isolation and recovery.
 *
 * Owner: Agent B.  Authority: spec 05 §8, spec 03 §5 and spec 09 REQ-FDIR-001
 * .. REQ-FDIR-005.
 *
 * Contract:
 *   - This module owns the per-source health state, the source use policy and
 *     the recovery probation.  It consumes *evidence* (sample rejection
 *     counters, device fault flags, timeouts) and never reads an estimator's
 *     private covariance or residuals.  It cannot modify an active filter
 *     state: it has no handle to one.
 *   - A single rejected observation is not a source failure; isolation needs
 *     a sustained reject streak or a hard device fault.
 *   - With only one instance of a sensor kind there is no redundancy: the
 *     policy degrades/rejects the source and the caller must degrade the
 *     navigation capability instead of pretending a second source exists.
 *   - Recovery goes through PROBATION with a minimum number of accepted
 *     samples and a minimum dwell time; it is not an immediate OK.
 */

#ifndef SENSOR_INTEGRITY_H
#define SENSOR_INTEGRITY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SI_MAX_SOURCES 8u

typedef enum {
    SI_SOURCE_OK = 0,
    SI_SOURCE_SUSPECT = 1,
    SI_SOURCE_ISOLATED = 2,
    SI_SOURCE_PROBATION = 3
} si_state_t;

typedef enum {
    SI_EVIDENCE_SAMPLE_REJECTED = 1, /* one observation failed gating */
    SI_EVIDENCE_SAMPLE_ACCEPTED = 2, /* one observation passed gating */
    SI_EVIDENCE_DEVICE_FAULT = 3,    /* hard device/driver fault */
    SI_EVIDENCE_RECOVERED = 4        /* driver/device reports health again */
} si_evidence_t;

typedef enum {
    SI_POLICY_USABLE = 0,
    SI_POLICY_DEGRADED = 1,
    SI_POLICY_REJECT = 2
} si_policy_t;

typedef struct {
    uint32_t suspect_after_rejects; /* e.g. 3  */
    uint32_t isolate_after_rejects; /* e.g. 10 */
    uint32_t probation_after_accepts; /* e.g. 20 */
    uint64_t isolate_window_us;     /* sustained-reject window, e.g. 2 s */
    uint64_t timeout_us;            /* no valid sample, e.g. 200 ms */
    uint64_t probation_min_us;      /* minimum probation dwell, e.g. 2 s */
} si_config_t;

typedef struct {
    uint32_t source_id;
    bool registered;
    bool critical;
    si_state_t state;
    uint32_t reject_streak;
    uint32_t accept_streak;
    uint64_t first_reject_us;
    uint64_t last_valid_us;
    uint64_t state_since_us;
    uint32_t isolation_count;
    bool device_fault;
    bool recovery_degraded;
    uint32_t last_shadow_sequence;
    uint64_t last_shadow_time_us;
} si_source_t;

typedef struct {
    si_config_t config;
    si_source_t sources[SI_MAX_SOURCES];
    uint32_t count;
} si_manager_t;

typedef enum {
    SI_OK = 0,
    SI_ERR_ARGUMENT = 1,
    SI_ERR_FULL = 2,
    SI_ERR_UNKNOWN_SOURCE = 3
} si_status_t;

/* Outcome of one M2 decode/gate step, as seen by the sensor layer. */
typedef enum {
    SI_M2_OK = 0,
    SI_M2_CHECKSUM = 1, /* payload/transport corruption */
    SI_M2_LENGTH = 2,   /* wrong burst length */
    SI_M2_RANGE = 3,    /* physical range / plausibility failure */
    SI_M2_STALE = 4,    /* no fresh sample or observation too old */
    SI_M2_NOT_CONFIGURED = 5
} si_m2_result_t;

si_status_t si_init(si_manager_t *manager, const si_config_t *config);
si_status_t si_register(si_manager_t *manager, uint32_t source_id, bool critical);

/* Reports one piece of evidence for a source.  'now_us' is FCU monotonic. */
si_status_t si_report(si_manager_t *manager, uint32_t source_id, si_evidence_t evidence,
                      uint64_t now_us);
/* Only the health owner calls this after validating fresh M3 SHADOW_ONLY
 * evidence against boot/time/state/policy/calibration. Decode success alone
 * cannot recover an isolated source. One evidence sequence is consumed once. */
si_status_t si_report_shadow(si_manager_t *manager,uint32_t source_id,bool consistent,
                              uint32_t evidence_sequence,uint64_t now_us);

/* Drives the timeout transitions; call at the health-task rate. */
si_status_t si_tick(si_manager_t *manager, uint64_t now_us);

si_policy_t si_policy(const si_manager_t *manager, uint32_t source_id);
si_state_t si_state(const si_manager_t *manager, uint32_t source_id);

/* False when only one instance of this source is registered: the caller must
 * degrade capability instead of claiming a redundant replacement. */
bool si_has_redundancy(const si_manager_t *manager, uint32_t source_id);

/* Maps one M2 decode/gate outcome onto source evidence:
 *   OK                  -> accepted sample
 *   CHECKSUM / LENGTH   -> hard device/transport fault (immediate isolation)
 *   RANGE / STALE       -> rejected sample (streak rules apply)
 *   NOT_CONFIGURED      -> no source evidence (configuration error)
 */
si_status_t si_report_m2_result(si_manager_t *manager, uint32_t source_id, si_m2_result_t result,
                                uint64_t now_us);

/* True when every registered critical source is usable. */
bool si_critical_usable(const si_manager_t *manager);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_INTEGRITY_H */
