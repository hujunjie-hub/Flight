/*
 * Sensor Integrity Manager implementation (spec 05 §8, spec 09 REQ-FDIR-*).
 *
 * Plain C11, no RTOS or HAL dependency, no heap.
 */

#include "source_health/sensor_integrity.h"

#include <string.h>

static si_source_t *find_source(si_manager_t *manager, uint32_t source_id) {
    uint32_t i;
    for (i = 0; i < manager->count; ++i) {
        if (manager->sources[i].registered && (manager->sources[i].source_id == source_id)) {
            return &manager->sources[i];
        }
    }
    return NULL;
}

static const si_source_t *find_source_const(const si_manager_t *manager, uint32_t source_id) {
    uint32_t i;
    for (i = 0; i < manager->count; ++i) {
        if (manager->sources[i].registered && (manager->sources[i].source_id == source_id)) {
            return &manager->sources[i];
        }
    }
    return NULL;
}

static void enter_state(si_source_t *source, si_state_t state, uint64_t now_us) {
    if (source->state != state) {
        source->state = state;
        source->state_since_us = now_us;
        source->accept_streak = 0u;
    }
}

static bool sustained(const si_manager_t *manager, const si_source_t *source, uint64_t now_us) {
    if (source->first_reject_us == 0u) {
        return false;
    }
    return (now_us >= source->first_reject_us) &&
           ((now_us - source->first_reject_us) <= manager->config.isolate_window_us);
}

si_status_t si_init(si_manager_t *manager, const si_config_t *config) {
    if ((manager == NULL) || (config == NULL)) {
        return SI_ERR_ARGUMENT;
    }
    if ((config->suspect_after_rejects == 0u) ||
        (config->isolate_after_rejects < config->suspect_after_rejects) ||
        (config->probation_after_accepts == 0u) || !(config->timeout_us > 0u)) {
        return SI_ERR_ARGUMENT;
    }
    memset(manager, 0, sizeof(*manager));
    manager->config = *config;
    return SI_OK;
}

si_status_t si_register(si_manager_t *manager, uint32_t source_id, bool critical) {
    si_source_t *existing;
    if (manager == NULL) {
        return SI_ERR_ARGUMENT;
    }
    existing = find_source(manager, source_id);
    if (existing != NULL) {
        existing->critical = critical;
        return SI_OK;
    }
    if (manager->count >= SI_MAX_SOURCES) {
        return SI_ERR_FULL;
    }
    si_source_t *source = &manager->sources[manager->count++];
    memset(source, 0, sizeof(*source));
    source->source_id = source_id;
    source->registered = true;
    source->critical = critical;
    source->state = SI_SOURCE_OK;
    return SI_OK;
}

si_status_t si_report(si_manager_t *manager, uint32_t source_id, si_evidence_t evidence,
                      uint64_t now_us) {
    si_source_t *source;
    if (manager == NULL) {
        return SI_ERR_ARGUMENT;
    }
    source = find_source(manager, source_id);
    if (source == NULL) {
        return SI_ERR_UNKNOWN_SOURCE;
    }

    switch (evidence) {
    case SI_EVIDENCE_SAMPLE_REJECTED:
        /* A single gated observation must never isolate a source. */
        source->accept_streak = 0u;
        if (source->reject_streak == 0u) {
            source->first_reject_us = now_us;
        }
        ++source->reject_streak;

        if (source->state == SI_SOURCE_ISOLATED) {
            /* Already isolated: keep counting, do not re-enter. */
            break;
        }
        if ((source->reject_streak >= manager->config.isolate_after_rejects) &&
            sustained(manager, source, now_us)) {
            enter_state(source, SI_SOURCE_ISOLATED, now_us);
            ++source->isolation_count;
        } else if ((source->reject_streak >= manager->config.suspect_after_rejects) &&
                   (source->state == SI_SOURCE_OK)) {
            enter_state(source, SI_SOURCE_SUSPECT, now_us);
        }
        break;

    case SI_EVIDENCE_SAMPLE_ACCEPTED:
        if(source->state!=SI_SOURCE_PROBATION && source->state!=SI_SOURCE_ISOLATED)
            source->last_valid_us = now_us;
        if (source->state == SI_SOURCE_SUSPECT) {
            if(source->recovery_degraded) {
                ++source->accept_streak;
                if(source->accept_streak<manager->config.probation_after_accepts ||
                   now_us-source->state_since_us<manager->config.probation_min_us) break;
                source->recovery_degraded=false;
            }
            source->reject_streak = 0u;
            source->first_reject_us = 0u;
            enter_state(source, SI_SOURCE_OK, now_us);
            break;
        }
        if ((source->state == SI_SOURCE_PROBATION) || (source->state == SI_SOURCE_ISOLATED)) {
            /* This proves reception/transport only, not recovery consistency. */
            break;
        }
        source->reject_streak = 0u;
        source->first_reject_us = 0u;
        break;

    case SI_EVIDENCE_DEVICE_FAULT:
        /* Hard fault: isolate immediately, no streak needed. */
        source->device_fault = true;
        source->recovery_degraded=false;
        if (source->state != SI_SOURCE_ISOLATED) {
            ++source->isolation_count;
        }
        enter_state(source, SI_SOURCE_ISOLATED, now_us);
        break;

    case SI_EVIDENCE_RECOVERED:
        source->device_fault = false;
        /* Health is a prerequisite for probation, not a promotion. */
        if (source->state == SI_SOURCE_ISOLATED) {
            enter_state(source, SI_SOURCE_PROBATION, now_us);
        }
        break;

    default:
        return SI_ERR_ARGUMENT;
    }
    return SI_OK;
}

si_status_t si_report_shadow(si_manager_t *manager,uint32_t id,bool consistent,
                              uint32_t sequence,uint64_t now_us) {
    si_source_t *source;
    if(!manager || !sequence) return SI_ERR_ARGUMENT;
    source=find_source(manager,id);
    if(!source) return SI_ERR_UNKNOWN_SOURCE;
    if(source->device_fault || (source->state!=SI_SOURCE_ISOLATED && source->state!=SI_SOURCE_PROBATION))
        return SI_ERR_ARGUMENT;
    if((source->last_shadow_sequence && (sequence==source->last_shadow_sequence ||
          (uint32_t)(sequence-source->last_shadow_sequence)>=0x80000000u)) ||
       now_us<source->last_shadow_time_us) return SI_ERR_ARGUMENT;
    if(source->last_shadow_time_us && now_us-source->last_shadow_time_us>manager->config.timeout_us &&
       source->state==SI_SOURCE_PROBATION) {
        ++source->isolation_count; enter_state(source,SI_SOURCE_ISOLATED,now_us);
    }
    source->last_shadow_sequence=sequence; source->last_shadow_time_us=now_us;
    if(!consistent) {
        source->accept_streak=0;
        if(source->state==SI_SOURCE_PROBATION) {
            ++source->isolation_count; enter_state(source,SI_SOURCE_ISOLATED,now_us);
        }
        return SI_OK;
    }
    if(source->state==SI_SOURCE_ISOLATED) enter_state(source,SI_SOURCE_PROBATION,now_us);
    source->last_valid_us=now_us; source->reject_streak=0; source->first_reject_us=0;
    ++source->accept_streak;
    if(source->accept_streak>=manager->config.probation_after_accepts &&
       now_us-source->state_since_us>=manager->config.probation_min_us) {
        enter_state(source,SI_SOURCE_SUSPECT,now_us); source->accept_streak=0;
        source->recovery_degraded=true;
    }
    return SI_OK;
}

si_status_t si_tick(si_manager_t *manager, uint64_t now_us) {
    uint32_t i;
    if (manager == NULL) {
        return SI_ERR_ARGUMENT;
    }
    for (i = 0; i < manager->count; ++i) {
        si_source_t *source = &manager->sources[i];
        if (!source->registered) {
            continue;
        }
        /* Timeout is measured against the last accepted sample; a source that
         * never produced one is not aged out before its first sample. */
        uint64_t reference=source->last_valid_us;
        if(source->state==SI_SOURCE_PROBATION && source->state_since_us>reference)
            reference=source->state_since_us;
        if ((reference != 0u) && (now_us > reference) &&
            ((now_us - reference) > manager->config.timeout_us)) {
            if (source->state != SI_SOURCE_ISOLATED) {
                ++source->isolation_count;
            }
            enter_state(source, SI_SOURCE_ISOLATED, now_us);
            continue;
        }
        /* A probation that produces nothing for the timeout window falls back
         * to isolation: recovery needs continuous evidence. */
        if ((source->state == SI_SOURCE_PROBATION) && (now_us > source->state_since_us) &&
            ((now_us - source->state_since_us) > manager->config.timeout_us) &&
            (source->accept_streak == 0u)) {
            enter_state(source, SI_SOURCE_ISOLATED, now_us);
        }
    }
    return SI_OK;
}

si_policy_t si_policy(const si_manager_t *manager, uint32_t source_id) {
    const si_source_t *source = find_source_const(manager, source_id);
    if (source == NULL) {
        return SI_POLICY_REJECT;
    }
    switch (source->state) {
    case SI_SOURCE_OK:
        return SI_POLICY_USABLE;
    case SI_SOURCE_SUSPECT:
        return SI_POLICY_DEGRADED;
    case SI_SOURCE_ISOLATED:
    case SI_SOURCE_PROBATION:
    default:
        return SI_POLICY_REJECT;
    }
}

si_state_t si_state(const si_manager_t *manager, uint32_t source_id) {
    const si_source_t *source = find_source_const(manager, source_id);
    return (source == NULL) ? SI_SOURCE_ISOLATED : source->state;
}

bool si_has_redundancy(const si_manager_t *manager, uint32_t source_id) {
    const si_source_t *source = find_source_const(manager, source_id);
    uint32_t i;
    uint32_t instances = 0u;
    if ((manager == NULL) || (source == NULL)) {
        return false;
    }
    /* Redundancy is counted over registered instances of the same source id;
     * the QUAD_X4 baseline has one ADIS16505, one BMP585 and one BMM350. */
    for (i = 0; i < manager->count; ++i) {
        if (manager->sources[i].registered && (manager->sources[i].source_id == source_id)) {
            ++instances;
        }
    }
    return instances > 1u;
}

si_status_t si_report_m2_result(si_manager_t *manager, uint32_t source_id, si_m2_result_t result,
                                uint64_t now_us) {
    switch (result) {
    case SI_M2_OK:
        return si_report(manager, source_id, SI_EVIDENCE_SAMPLE_ACCEPTED, now_us);
    case SI_M2_CHECKSUM:
    case SI_M2_LENGTH:
        /* A corrupted burst is a device/bus fact, not a filtering outlier. */
        return si_report(manager, source_id, SI_EVIDENCE_DEVICE_FAULT, now_us);
    case SI_M2_RANGE:
    case SI_M2_STALE:
        return si_report(manager, source_id, SI_EVIDENCE_SAMPLE_REJECTED, now_us);
    case SI_M2_NOT_CONFIGURED:
        return SI_OK; /* configuration problem, not a sensor-health fact */
    default:
        return SI_ERR_ARGUMENT;
    }
}

bool si_critical_usable(const si_manager_t *manager) {
    uint32_t i;
    if (manager == NULL) {
        return false;
    }
    for (i = 0; i < manager->count; ++i) {
        if (manager->sources[i].registered && manager->sources[i].critical &&
            (si_policy(manager, manager->sources[i].source_id) != SI_POLICY_USABLE)) {
            return false;
        }
    }
    return true;
}
