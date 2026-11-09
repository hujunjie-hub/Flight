/*
 * Sensor Integrity Manager tests (spec 05 §8, spec 09 REQ-FDIR-001..005).
 */

#include <stdio.h>
#include <string.h>

#include "source_health/sensor_integrity.h"

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
        }                                                                                          \
    } while (0)

static si_config_t test_config(void) {
    si_config_t config;
    config.suspect_after_rejects = 3u;
    config.isolate_after_rejects = 10u;
    config.probation_after_accepts = 5u;
    config.isolate_window_us = 2000000u;
    config.timeout_us = 200000u;
    config.probation_min_us = 1000000u;
    return config;
}

static si_manager_t make_manager(uint32_t *imu_id) {
    si_manager_t manager;
    const si_config_t config = test_config();
    CHECK(si_init(&manager, &config) == SI_OK);
    CHECK(si_register(&manager, 10u, true) == SI_OK);  /* IMU  */
    CHECK(si_register(&manager, 11u, false) == SI_OK); /* GNSS */
    *imu_id = 10u;
    return manager;
}

static void test_single_rejection_is_not_isolation(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    CHECK(si_state(&manager, imu) == SI_SOURCE_OK);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 1000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_OK);
    CHECK(si_policy(&manager, imu) == SI_POLICY_USABLE);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 2000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_OK);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 3000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_SUSPECT);
    CHECK(si_policy(&manager, imu) == SI_POLICY_DEGRADED);

    /* One accepted sample clears a suspicion; it is not an isolation. */
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_ACCEPTED, 4000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_OK);
}

static void test_sustained_rejection_isolates(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    uint32_t i;
    for (i = 0; i < 9u; ++i) {
        CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 1000u + i * 1000u) == SI_OK);
    }
    CHECK(si_state(&manager, imu) == SI_SOURCE_SUSPECT);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 10000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_ISOLATED);
    CHECK(si_policy(&manager, imu) == SI_POLICY_REJECT);
    CHECK(!si_has_redundancy(&manager, imu));
    CHECK(!si_critical_usable(&manager));
}

static void test_rejection_outside_window_does_not_isolate(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    uint32_t i;
    /* Ten rejections spread over 10 s: no sustained inconsistency. */
    for (i = 0; i < 10u; ++i) {
        CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_REJECTED, 1000u + i * 1000000u) == SI_OK);
    }
    CHECK(si_state(&manager, imu) != SI_SOURCE_ISOLATED);
}

static void test_timeout_isolates(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_ACCEPTED, 1000u) == SI_OK);
    CHECK(si_tick(&manager, 150000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_OK);
    CHECK(si_tick(&manager, 250000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_ISOLATED);
}

static void test_device_fault_and_probation(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    imu=11u; /* GNSS: the single IMU has no independent recovery observation. */
    CHECK(si_report(&manager, imu, SI_EVIDENCE_SAMPLE_ACCEPTED, 1000u) == SI_OK);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_DEVICE_FAULT, 2000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_ISOLATED);
    CHECK(si_report_shadow(&manager,imu,true,1,2500u)==SI_ERR_ARGUMENT);
    CHECK(si_report(&manager,imu,SI_EVIDENCE_SAMPLE_ACCEPTED,2600u)==SI_OK);
    CHECK(si_state(&manager,imu)==SI_SOURCE_ISOLATED);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_RECOVERED, 3000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_PROBATION);
    CHECK(si_policy(&manager, imu) == SI_POLICY_REJECT);
    for(uint32_t i=0;i<=10;++i) {
        uint64_t now=10000u+i*100000u;
        CHECK(si_report_shadow(&manager,imu,true,i+1,now)==SI_OK);
        CHECK(si_tick(&manager,now)==SI_OK);
        if(i<10) CHECK(si_state(&manager,imu)==SI_SOURCE_PROBATION);
    }
    CHECK(si_state(&manager,imu)==SI_SOURCE_SUSPECT);
    CHECK(si_policy(&manager,imu)==SI_POLICY_DEGRADED);
    CHECK(si_report_shadow(&manager,imu,true,11,1020000u)==SI_ERR_ARGUMENT);
    for(uint32_t i=1;i<=11;++i)
        CHECK(si_report(&manager,imu,SI_EVIDENCE_SAMPLE_ACCEPTED,1010000u+i*100000u)==SI_OK);
    CHECK(si_state(&manager,imu)==SI_SOURCE_OK);
    CHECK(si_policy(&manager,imu)==SI_POLICY_USABLE);
}

static void test_probation_times_out(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_DEVICE_FAULT, 1000u) == SI_OK);
    CHECK(si_report(&manager, imu, SI_EVIDENCE_RECOVERED, 2000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_PROBATION);
    /* No accepted sample during the timeout window: back to isolation. */
    CHECK(si_tick(&manager, 300000u) == SI_OK);
    CHECK(si_state(&manager, imu) == SI_SOURCE_ISOLATED);
}

static void test_registry_and_policy_edges(uint32_t imu) {
    si_manager_t manager = make_manager(&imu);
    CHECK(si_report(&manager, 99u, SI_EVIDENCE_SAMPLE_ACCEPTED, 1000u) == SI_ERR_UNKNOWN_SOURCE);
    CHECK(si_policy(&manager, 99u) == SI_POLICY_REJECT);
    CHECK(si_state(&manager, 99u) == SI_SOURCE_ISOLATED);

    si_manager_t bad;
    si_config_t config = test_config();
    config.probation_after_accepts = 0u;
    CHECK(si_init(&bad, &config) == SI_ERR_ARGUMENT);

    si_manager_t full;
    config = test_config();
    CHECK(si_init(&full, &config) == SI_OK);
    for (uint32_t i = 0; i < SI_MAX_SOURCES; ++i) {
        CHECK(si_register(&full, 100u + i, false) == SI_OK);
    }
    CHECK(si_register(&full, 200u, false) == SI_ERR_FULL);

    /* A registered single instance is explicitly not redundant. */
    CHECK(!si_has_redundancy(&manager, 10u));
    CHECK(!si_has_redundancy(&manager, 11u));
    /* Unknown source is still reported as non-redundant. */
    CHECK(!si_has_redundancy(&manager, 42u));
}

static void test_m2_result_mapping(void) {
    si_manager_t manager;
    si_config_t config = test_config();
    CHECK(si_init(&manager, &config) == SI_OK);
    CHECK(si_register(&manager, 10u, true) == SI_OK);

    /* Fresh samples keep the source healthy. */
    CHECK(si_report_m2_result(&manager, 10u, SI_M2_OK, 1000u) == SI_OK);
    CHECK(si_state(&manager, 10u) == SI_SOURCE_OK);

    /* A corrupted burst is a hard fault, not a statistical outlier. */
    CHECK(si_report_m2_result(&manager, 10u, SI_M2_CHECKSUM, 2000u) == SI_OK);
    CHECK(si_state(&manager, 10u) == SI_SOURCE_ISOLATED);

    /* Reception alone never restores a hard-faulted source. */
    uint64_t t = 3000u;
    for (int i = 0; i < 6; ++i) {
        CHECK(si_report_m2_result(&manager, 10u, SI_M2_OK, t) == SI_OK);
        t += 100000u;
    }
    CHECK(si_state(&manager, 10u) == SI_SOURCE_ISOLATED);
    CHECK(si_report(&manager,10u,SI_EVIDENCE_RECOVERED,t)==SI_OK);
    for(uint32_t i=0;i<=10;++i)
        CHECK(si_report_shadow(&manager,10u,true,i+1,t+i*100000u)==SI_OK);
    CHECK(si_state(&manager,10u)==SI_SOURCE_SUSPECT);
    t+=1000000u;
    for(uint32_t i=1;i<=11;++i)
        CHECK(si_report_m2_result(&manager,10u,SI_M2_OK,t+i*100000u)==SI_OK);
    CHECK(si_state(&manager,10u)==SI_SOURCE_OK);
    t+=1100000u;

    /* Range/stale outcomes feed the reject streak. */
    CHECK(si_report_m2_result(&manager, 10u, SI_M2_RANGE, t + 1100000u) == SI_OK);
    CHECK(si_state(&manager, 10u) == SI_SOURCE_OK);
    CHECK(si_report_m2_result(&manager, 10u, SI_M2_STALE, t + 1200000u) == SI_OK);
    CHECK(si_report_m2_result(&manager, 10u, SI_M2_RANGE, t + 1300000u) == SI_OK);
    CHECK(si_state(&manager, 10u) == SI_SOURCE_SUSPECT);

    /* A configuration error is not a sensor-health fact. */
    si_manager_t other;
    CHECK(si_init(&other, &config) == SI_OK);
    CHECK(si_register(&other, 11u, false) == SI_OK);
    CHECK(si_report_m2_result(&other, 11u, SI_M2_NOT_CONFIGURED, 1000u) == SI_OK);
    CHECK(si_state(&other, 11u) == SI_SOURCE_OK);
    CHECK(si_report_m2_result(&other, 99u, SI_M2_OK, 1000u) == SI_ERR_UNKNOWN_SOURCE);
}

int main(void) {
    uint32_t imu = 0u;
    printf("Sensor Integrity Manager tests\n");
    test_single_rejection_is_not_isolation(imu);
    test_sustained_rejection_isolates(imu);
    test_rejection_outside_window_does_not_isolate(imu);
    test_timeout_isolates(imu);
    test_device_fault_and_probation(imu);
    test_probation_times_out(imu);
    test_registry_and_policy_edges(imu);
    test_m2_result_mapping();
    printf("test_health: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}

