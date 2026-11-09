/*
 * Host test for the PLATFORM Time Service core (T191-A-PLATFORM preparation).
 * Simulates the board tick source; no HAL/RTOS dependency.
 */
#include "ports/port_api.h"
#include "systime.h"

#include <stdio.h>
#include <string.h>

static int failures;
static int checks;

#define CHECK(cond) do { \
  checks++; \
  if (!(cond)) { \
    failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
  } \
} while (0)

#define TICK_HZ 10000000u
static uint64_t fake_tick;
static uint32_t fake_epoch = 1u;

static Status fake_source(uint64_t *tick64, uint32_t *time_epoch)
{
  *tick64 = fake_tick;
  *time_epoch = fake_epoch;
  return STATUS_OK;
}

static TimeConfig base_config(void)
{
  TimeConfig config;
  memset(&config, 0, sizeof(config));
  config.boot_id = 0x0102030405060708ull;
  config.time_epoch = 1u;
  config.tick_hz = TICK_HZ;
  config.holdover_rate_bound_ppb = 10000u;
  config.gnss_pps_installed = 1u;
  return config;
}

static void reset_clock(uint64_t tick)
{
  fake_tick = tick;
  fake_epoch = 1u;
}

static Status init_default(void)
{
  TimeConfig config = base_config();
  return systime_init(&config, fake_source);
}

static Status submit_pps(uint64_t tick, uint32_t sequence)
{
  PpsCaptureEvent event;
  memset(&event, 0, sizeof(event));
  event.capture_tick64 = tick;
  event.edge_sequence = sequence;
  event.time_epoch = 1u;
  event.capture_flags = TIMER_CAPTURE_VALID;
  return time_submit_pps(&event);
}

static void test_monotonic_mapping(void)
{
  ClockInstant instant;
  uint64_t mono = 0u;
  uint64_t historical = 0u;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  CHECK(clock_read(&instant) == STATUS_OK);
  CHECK(instant.boot_id == 0x0102030405060708ull);
  CHECK(instant.monotonic_us == 0u);
  CHECK(instant.time_epoch == 1u);

  fake_tick = TICK_HZ; /* exactly 1 s */
  CHECK(clock_read(&instant) == STATUS_OK);
  CHECK(instant.monotonic_us >= 999998u && instant.monotonic_us <= 1000002u);

  CHECK(clock_ticks_to_mono_us(TICK_HZ / 2u, 1u, &historical) == STATUS_OK);
  CHECK(historical >= 499999u && historical <= 500001u);
  CHECK(clock_ticks_to_mono_us(0u, 2u, &mono) == STATUS_STALE_DATA);

  fake_tick += TICK_HZ / 100u; /* 10 ms later: monotonic must not regress */
  CHECK(clock_now_us(&mono) == STATUS_OK);
  CHECK(mono >= 1009998u);

  CHECK(clock_ticks_to_mono_us(1u, 1u, &historical) == STATUS_OK);
  CHECK(historical <= 1u);
}

static void test_pps_lock_and_rate(void)
{
  LocalTimebaseDisciplineStatus discipline;
  TimeServiceStatus status;
  uint32_t i;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);

  for (i = 1u; i <= 9u; i++) {
    CHECK(submit_pps((uint64_t)i * TICK_HZ, i) == STATUS_OK);
  }
  fake_tick = 9ull * TICK_HZ;
  CHECK(time_get_local_discipline_status(&discipline) == STATUS_OK);
  CHECK(discipline.discipline_state == LOCAL_TIMEBASE_LOCKED);
  CHECK(discipline.accepted_interval_count >= 8u);
  CHECK(discipline.rate_correction_ppb >= -500000 && discipline.rate_correction_ppb <= 500000);

  CHECK(time_get_status(&status) == STATUS_OK);
  CHECK(status.state == TIME_QUALITY_ACQUIRING);
  CHECK(status.utc_valid == 0u);
  CHECK(status.local_clock_valid == 1u);
  CHECK(status.frequency_discipline_valid == 1u);
}

static void test_pps_period_gate(void)
{
  LocalTimebaseDisciplineStatus discipline;
  uint64_t tick = 0u;
  uint32_t i;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  /* 1.002 s periods = 2000 ppm: outside the 1000 ppm gate. */
  for (i = 1u; i <= 9u; i++) {
    tick += (TICK_HZ / 1000u) * 1002u;
    if (i == 1u) {
      CHECK(submit_pps(tick, i) == STATUS_OK); /* first edge only seeds */
    } else {
      CHECK(submit_pps(tick, i) == STATUS_OUT_OF_RANGE);
    }
  }
  fake_tick = tick;
  CHECK(time_get_local_discipline_status(&discipline) == STATUS_OK);
  CHECK(discipline.discipline_state != LOCAL_TIMEBASE_LOCKED);
}

static void test_pps_timeout_holdover(void)
{
  LocalTimebaseDisciplineStatus discipline;
  TimeServiceStatus status;
  ClockInstant instant;
  uint64_t expected;
  int32_t applied;
  const uint64_t period = TICK_HZ + TICK_HZ / 10000u; /* oscillator 100 ppm fast */

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  for (uint32_t i = 1u; i <= 9u; i++) {
    fake_tick = i * period;
    CHECK(clock_read(&instant) == STATUS_OK);
    CHECK(submit_pps(fake_tick, i) == STATUS_OK);
  }
  fake_tick = 10u * period;
  CHECK(clock_read(&instant) == STATUS_OK);
  CHECK(time_get_local_discipline_status(&discipline) == STATUS_OK);
  CHECK(discipline.discipline_state == LOCAL_TIMEBASE_LOCKED);
  applied = discipline.rate_correction_ppb;
  CHECK(applied < 0 && applied > -100000); /* correction began but is still slewing */

  fake_tick = 12u * period; /* 3 s without PPS > 2.5 s */
  CHECK(clock_ticks_to_mono_us(fake_tick, instant.time_epoch, &expected) == STATUS_OK);
  CHECK(time_get_status(&status) == STATUS_OK);
  /* No UTC mapping was ever established, so the TimeService state is not
   * HOLDOVER (spec 05 section 7); the local timebase itself is in HOLDOVER. */
  CHECK(status.state == TIME_QUALITY_ACQUIRING);
  CHECK(status.utc_valid == 0u);
  CHECK(status.holdover_duration_us >= 2500000u);
  CHECK(status.local_clock_valid && status.time_epoch == instant.time_epoch);
  CHECK(status.monotonic_now_us == expected && expected > instant.monotonic_us);
  CHECK(time_get_local_discipline_status(&discipline) == STATUS_OK);
  CHECK(discipline.discipline_state == LOCAL_TIMEBASE_HOLDOVER);
  CHECK(discipline.rate_correction_ppb == applied);

  fake_tick = 20u * period;
  CHECK(clock_read(&instant) == STATUS_OK && instant.monotonic_us > expected);
  CHECK(time_get_local_discipline_status(&discipline) == STATUS_OK);
  CHECK(discipline.rate_correction_ppb == applied);
}

static void test_utc_locked(void)
{
  GnssTimeObservation observation;
  TimeServiceStatus status;
  ClockDomainMappingStatus mapping;
  uint32_t i;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  for (i = 1u; i <= 9u; i++) {
    CHECK(submit_pps((uint64_t)i * TICK_HZ, i) == STATUS_OK);
  }
  memset(&observation, 0, sizeof(observation));
  observation.utc_time_ns = 1700000000000000000ull; /* POSIX ns */
  observation.paired_pps_tick64 = 9ull * TICK_HZ;
  observation.pps_edge_sequence = 9u;
  observation.time_epoch = 1u;
  observation.receiver_accuracy_ns = 50000u;
  observation.time_scale = TIME_SCALE_UTC;
  observation.validity_flags = GNSS_TIME_UTC_VALID;
  fake_tick = 9ull * TICK_HZ;
  CHECK(time_submit_gnss_time(&observation) == STATUS_OK);

  CHECK(time_get_status(&status) == STATUS_OK);
  CHECK(status.state == TIME_QUALITY_LOCKED);
  CHECK(status.utc_valid == 1u);
  CHECK(status.utc_estimate_ns == observation.utc_time_ns);
  CHECK(status.utc_uncertainty_ns <= TIME_UTC_LOCK_UNCERTAINTY_NS);
  CHECK(status.utc_mapping_epoch >= 1u);

  CHECK(time_get_mapping_status(CLOCK_DOMAIN_UTC_POSIX, &mapping) == STATUS_OK);
  CHECK(mapping.mapping_state == CLOCK_MAPPING_LOCKED);
  CHECK(mapping.uncertainty_ns <= TIME_UTC_LOCK_UNCERTAINTY_NS);
}

static void test_utc_requires_pps_pairing(void)
{
  GnssTimeObservation observation;
  TimeServiceStatus status;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  memset(&observation, 0, sizeof(observation));
  observation.utc_time_ns = 1700000000000000000ull;
  observation.paired_pps_tick64 = 12345u;
  observation.pps_edge_sequence = 1u;
  observation.time_epoch = 1u;
  observation.time_scale = TIME_SCALE_UTC;
  observation.validity_flags = GNSS_TIME_UTC_VALID;
  CHECK(time_submit_gnss_time(&observation) == STATUS_STALE_DATA);
  CHECK(time_get_status(&status) == STATUS_OK);
  CHECK(status.utc_valid == 0u);
  CHECK(status.state == TIME_QUALITY_LOCAL_ONLY);
  CHECK(time_get_mapping_status(CLOCK_DOMAIN_UTC_POSIX, &(ClockDomainMappingStatus){0}) ==
        STATUS_NOT_READY);
}

static void test_utc_uncertainty_limit(void)
{
  TimeConfig config = base_config();
  GnssTimeObservation observation;
  TimeServiceStatus status;
  uint32_t i;

  config.utc_max_uncertainty_ns = 1000u; /* tight bound for the test */
  config.holdover_rate_bound_ppb = 1000000000u;
  reset_clock(0u);
  CHECK(systime_init(&config, fake_source) == STATUS_OK);
  for (i = 1u; i <= 9u; i++) {
    CHECK(submit_pps((uint64_t)i * TICK_HZ, i) == STATUS_OK);
  }
  memset(&observation, 0, sizeof(observation));
  observation.utc_time_ns = 1700000000000000000ull;
  observation.paired_pps_tick64 = 9ull * TICK_HZ;
  observation.pps_edge_sequence = 9u;
  observation.time_epoch = 1u;
  observation.time_scale = TIME_SCALE_UTC;
  observation.validity_flags = GNSS_TIME_UTC_VALID;
  fake_tick = 9ull * TICK_HZ;
  CHECK(time_submit_gnss_time(&observation) == STATUS_OK);

  CHECK(time_get_status(&status) == STATUS_OK);
  CHECK(status.utc_valid == 1u);

  fake_tick = 10ull * TICK_HZ; /* 1 s age; growth exceeds the 1 us budget */
  CHECK(time_get_status(&status) == STATUS_OK);
  CHECK(status.utc_valid == 0u);
  CHECK(status.state == TIME_QUALITY_DEGRADED);
}

static void test_external_mapping(void)
{
  ExternalClockSyncObservation observation;
  ClockDomainMappingStatus mapping;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  memset(&observation, 0, sizeof(observation));
  observation.source_clock_domain_id = CLOCK_DOMAIN_COMPANION;
  observation.local_time_epoch = 1u;
  observation.local_send_time_us = 0u;
  observation.source_receive_time_ns = 1000000000ull;
  observation.source_send_time_ns = 1000000500ull;
  observation.local_receive_time_us = 100u;
  observation.source_uncertainty_ns = 2000u;
  CHECK(time_submit_external_sync(&observation) == STATUS_OK);
  CHECK(time_get_mapping_status(CLOCK_DOMAIN_COMPANION, &mapping) == STATUS_OK);
  CHECK(mapping.source_clock_domain_id == CLOCK_DOMAIN_COMPANION);
  CHECK(mapping.mapping_state == CLOCK_MAPPING_LOCKED);
  CHECK(mapping.uncertainty_ns >= 2000u);
}

static void test_epoch_change_is_rejected(void)
{
  ClockInstant instant;

  reset_clock(0u);
  CHECK(init_default() == STATUS_OK);
  fake_epoch = 7u;
  CHECK(clock_read(&instant) == STATUS_STALE_DATA);
  CHECK(instant.monotonic_us == 0u && instant.boot_id == 0u);
}

int main(void)
{
  CHECK(clock_read(&(ClockInstant){0}) == STATUS_NOT_READY);

  test_monotonic_mapping();
  test_pps_lock_and_rate();
  test_pps_period_gate();
  test_pps_timeout_holdover();
  test_utc_locked();
  test_utc_requires_pps_pairing();
  test_utc_uncertainty_limit();
  test_external_mapping();
  test_epoch_change_is_rejected();

  CHECK(systime_stop() == STATUS_OK);
  printf("time_service host tests: %d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
