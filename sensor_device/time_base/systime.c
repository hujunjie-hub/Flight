/*
 * PLATFORM Time Service implementation.
 * See systime.h for scope and normative references.
 */
#include "ports/port_api.h"
#include "systime.h"

#include <string.h>

#define TIME_MAX_EXTRAPOLATE_US 60000000ull /* 60 s of forward extrapolation */
#define TIME_REANCHOR_US 1000000ull          /* 1 s continuous re-anchor */

static TimeService time;
static TimeService *time_active;
static uint8_t time_initialized;

static uint64_t time_scale_q32(uint32_t tick_hz, int32_t correction_ppb)
{
  int64_t nominal = ((int64_t)1000000 << 32) / (int64_t)tick_hz;
  int64_t adjusted = nominal * (1000000000LL + (int64_t)correction_ppb) / 1000000000LL;
  if (adjusted <= 0) {
    adjusted = nominal;
  }
  return (uint64_t)adjusted;
}

static int32_t time_clamp_i32(int64_t value, int32_t limit)
{
  if (value > (int64_t)limit) {
    return limit;
  }
  if (value < -(int64_t)limit) {
    return -limit;
  }
  return (int32_t)value;
}

static const TimeSegment *time_find_segment(const TimeService *service, uint64_t tick64)
{
  uint32_t i;
  const TimeSegment *best = NULL;

  for (i = 0u; i < service->segment_count; i++) {
    uint32_t index = (service->segment_first + i) % TIME_SEGMENT_COUNT;
    if (service->segments[index].anchor_tick64 <= tick64) {
      best = &service->segments[index];
    } else {
      break;
    }
  }
  return best;
}

static Status time_map_internal(const TimeService *service, uint64_t tick64, uint64_t *mono_us)
{
  const TimeSegment *segment;
  uint64_t delta;
  uint64_t limit_ticks;

  if (service == NULL || mono_us == NULL || service->segment_count == 0u) {
    return STATUS_NOT_READY;
  }
  segment = time_find_segment(service, tick64);
  if (segment == NULL) {
    return STATUS_STALE_DATA; /* older than the retained mapping history */
  }
  delta = tick64 - segment->anchor_tick64;
  limit_ticks = TIME_MAX_EXTRAPOLATE_US * (uint64_t)service->config.tick_hz / 1000000ull;
  if (delta > limit_ticks) {
    return STATUS_STALE_DATA;
  }
  *mono_us = segment->anchor_mono_us + ((delta * segment->scale_q32) >> 32);
  return STATUS_OK;
}

uint64_t time_map_tick(const TimeService *service, uint64_t tick64)
{
  uint64_t mono = 0u;
  if (time_map_internal(service, tick64, &mono) != STATUS_OK) {
    return 0u;
  }
  return mono;
}

uint32_t time_segment_count(const TimeService *service)
{
  return service != NULL ? service->segment_count : 0u;
}

int32_t time_applied_correction_ppb(const TimeService *service)
{
  return service != NULL ? service->correction_ppb : 0;
}

static void time_append_segment(TimeService *service, uint64_t tick64, uint64_t mono_us)
{
  uint32_t index;

  if (service->segment_count == TIME_SEGMENT_COUNT) {
    service->segment_first = (service->segment_first + 1u) % TIME_SEGMENT_COUNT;
    service->segment_count--;
  }
  index = (service->segment_first + service->segment_count) % TIME_SEGMENT_COUNT;
  service->segments[index].anchor_tick64 = tick64;
  service->segments[index].anchor_mono_us = mono_us;
  service->segments[index].scale_q32 = time_scale_q32(service->config.tick_hz,
                                                         service->correction_ppb);
  service->segments[index].discipline_epoch = service->discipline_epoch;
  service->segment_count++;
}

static void time_apply_slew(TimeService *service, uint64_t mono_us)
{
  int64_t delta_us;
  int64_t max_step_ppb;

  if (mono_us <= service->last_slew_mono_us) {
    return;
  }
  delta_us = (int64_t)(mono_us - service->last_slew_mono_us);
  max_step_ppb = (delta_us * TIME_RATE_SLEW_PPB_PER_S) / 1000000LL;
  if (max_step_ppb <= 0) {
    max_step_ppb = 1; /* make progress on small steps instead of stalling */
  }
  if (service->correction_ppb < service->target_correction_ppb) {
    int64_t next = (int64_t)service->correction_ppb + max_step_ppb;
    service->correction_ppb = (next > service->target_correction_ppb)
                                  ? service->target_correction_ppb
                                  : (int32_t)next;
  } else if (service->correction_ppb > service->target_correction_ppb) {
    int64_t next = (int64_t)service->correction_ppb - max_step_ppb;
    service->correction_ppb = (next < service->target_correction_ppb)
                                  ? service->target_correction_ppb
                                  : (int32_t)next;
  }
  service->last_slew_mono_us = mono_us;
}

static Status time_read_tick(TimeService *service, uint64_t *tick64)
{
  uint32_t epoch = 0u;
  Status status;

  if (service->tick_source == NULL) {
    return STATUS_NOT_READY;
  }
  status = service->tick_source(tick64, &epoch);
  if (status != STATUS_OK) {
    return status;
  }
  if (epoch != service->config.time_epoch) {
    service->local_valid = 0u;
    service->discipline_state = LOCAL_TIMEBASE_INVALID;
    service->utc_valid = 0u;
    return STATUS_STALE_DATA;
  }
  return STATUS_OK;
}

/* Advance/re-anchor the mapping and return the current monotonic time. */
static Status time_now(TimeService *service, uint64_t *mono_us)
{
  uint64_t tick64 = 0u;
  uint64_t mono = 0u;
  Status status;

  if (service == NULL) {
    return STATUS_NOT_READY;
  }
  status = time_read_tick(service, &tick64);
  if (status != STATUS_OK) {
    return status;
  }
  if (service->segment_count == 0u) {
    time_append_segment(service, tick64, 0u);
    service->last_reanchor_mono_us = 0u;
    service->last_slew_mono_us = 0u;
    service->last_mono_us = 0u;
    *mono_us = 0u;
    return STATUS_OK;
  }

  status = time_map_internal(service, tick64, &mono);
  if (status != STATUS_OK) {
    return status;
  }
  if (mono < service->last_mono_us) {
    mono = service->last_mono_us; /* monotonic time never steps backwards */
  }
  service->last_mono_us = mono;

  if (service->pps_ever_locked != 0u && mono >= service->pps_last_valid_mono_us &&
      mono - service->pps_last_valid_mono_us > (uint64_t)service->config.pps_timeout_us) {
    service->discipline_state = LOCAL_TIMEBASE_HOLDOVER;
    service->target_correction_ppb = service->correction_ppb;
  }

  if (mono - service->last_reanchor_mono_us >= TIME_REANCHOR_US) {
    time_apply_slew(service, mono);
    time_append_segment(service, tick64, mono);
    service->last_reanchor_mono_us = mono;
  }

  *mono_us = mono;
  return STATUS_OK;
}

static Status impl_time_service_init(const TimeConfig *config, TickSource tick_source)
{
  uint64_t tick64 = 0u;

  if (config == NULL || tick_source == NULL || config->tick_hz == 0u ||
      config->time_epoch == 0u) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(&time, 0, sizeof(time));
  time.config = *config;
  if (time.config.pps_timeout_us == 0u) {
    time.config.pps_timeout_us = TIME_PPS_TIMEOUT_US;
  }
  if (time.config.utc_timeout_us == 0u) {
    time.config.utc_timeout_us = TIME_UTC_TIMEOUT_US;
  }
  if (time.config.utc_locked_uncertainty_ns == 0u) {
    time.config.utc_locked_uncertainty_ns = TIME_UTC_LOCK_UNCERTAINTY_NS;
  }
  if (time.config.utc_max_uncertainty_ns == 0u) {
    time.config.utc_max_uncertainty_ns = TIME_UTC_MAX_UNCERTAINTY_NS;
  }
  if (time.config.holdover_rate_bound_ppb == 0u) {
    time.config.holdover_rate_bound_ppb = 10000u;
  }
  time.tick_source = tick_source;
  time.correction_ppb = time_clamp_i32(config->initial_rate_correction_ppb,
                                             TIME_RATE_CORRECTION_LIMIT_PPB);
  time.target_correction_ppb = time.correction_ppb;
  time.discipline_epoch = 1u;
  time.discipline_state = LOCAL_TIMEBASE_NOMINAL;
  time.local_valid = 1u;
  time.utc_mapping_epoch = 0u;
  time.utc_offset_ns = 0u;

  if (time_read_tick(&time, &tick64) != STATUS_OK) {
    time.local_valid = 0u;
    time.discipline_state = LOCAL_TIMEBASE_INVALID;
    return STATUS_NOT_READY;
  }
  time_append_segment(&time, tick64, 0u);
  time.last_mono_us = 0u;
  time.last_reanchor_mono_us = 0u;
  time.last_slew_mono_us = 0u;
  time_active = &time;
  time_initialized = 1u;
  return STATUS_OK;
}

static Status impl_time_service_stop(void)
{
  memset(&time, 0, sizeof(time));
  time_active = NULL;
  time_initialized = 0u;
  return STATUS_OK;
}

static Status impl_time_service_reset_epoch(uint32_t new_time_epoch)
{
  if (!time_initialized || new_time_epoch == 0u) {
    return STATUS_NOT_READY;
  }
  time.config.time_epoch = new_time_epoch;
  time.segment_count = 0u;
  time.segment_first = 0u;
  time.discipline_epoch++;
  time.discipline_state = LOCAL_TIMEBASE_NOMINAL;
  time.local_valid = 1u;
  time.utc_have_mapping = 0u;
  time.utc_valid = 0u;
  time.utc_mapping_epoch++;
  time.pps_have_last = 0u;
  time.pps_good_intervals = 0u;
  time.pps_window_count = 0u;
  time.pps_ever_locked = 0u;
  time.last_mono_us = 0u;
  time.last_reanchor_mono_us = 0u;
  time.last_slew_mono_us = 0u;
  return STATUS_OK;
}

static Status impl_clock_read(ClockInstant *out)
{
  uint64_t mono = 0u;
  Status status;

  if (out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  if (!time_initialized) {
    return STATUS_NOT_READY;
  }
  status = time_now(&time, &mono);
  if (status != STATUS_OK) {
    return status;
  }
  out->boot_id = time.config.boot_id;
  out->monotonic_us = mono;
  out->time_epoch = time.config.time_epoch;
  out->reserved = 0u;
  return STATUS_OK;
}

static Status impl_clock_now_us(uint64_t *mono_us)
{
  ClockInstant instant;
  Status status;

  if (mono_us == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  *mono_us = 0u;
  status = clock_read(&instant);
  if (status != STATUS_OK) {
    return status;
  }
  *mono_us = instant.monotonic_us;
  return STATUS_OK;
}

static Status impl_clock_ticks_to_mono_us(uint64_t tick64, uint32_t time_epoch, uint64_t *mono_us)
{
  if (mono_us == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  *mono_us = 0u;
  if (!time_initialized) {
    return STATUS_NOT_READY;
  }
  if (time_epoch != time.config.time_epoch) {
    return STATUS_STALE_DATA;
  }
  return time_map_internal(&time, tick64, mono_us);
}

static uint64_t time_utc_uncertainty_ns(const TimeService *service, uint64_t now_us)
{
  uint64_t age_us;
  uint64_t growth_ns;

  if (service->utc_have_mapping == 0u) {
    return 0u;
  }
  age_us = (now_us > service->utc_anchor_mono_us) ? (now_us - service->utc_anchor_mono_us) : 0u;
  growth_ns = age_us * (uint64_t)service->config.holdover_rate_bound_ppb / 1000ull;
  return (uint64_t)service->utc_accuracy_ns + growth_ns;
}

static Status impl_time_get_status(TimeServiceStatus *out)
{
  uint64_t now_us = 0u;
  uint64_t uncertainty_ns = 0u;
  uint8_t pps_timed_out;
  uint8_t utc_timed_out;
  uint8_t state;

  if (out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  if (!time_initialized || time_now(&time, &now_us) != STATUS_OK) {
    out->state = TIME_QUALITY_INVALID;
    return STATUS_NOT_READY;
  }

  uncertainty_ns = time_utc_uncertainty_ns(&time, now_us);
  pps_timed_out = (time.pps_ever_locked != 0u &&
                   now_us >= time.pps_last_valid_mono_us &&
                   now_us - time.pps_last_valid_mono_us >
                       (uint64_t)time.config.pps_timeout_us)
                      ? 1u
                      : 0u;
  utc_timed_out = (time.utc_have_mapping != 0u &&
                   now_us >= time.utc_last_update_mono_us &&
                   now_us - time.utc_last_update_mono_us >
                       (uint64_t)time.config.utc_timeout_us) ? 1u : 0u;

  time.utc_valid = (time.utc_have_mapping != 0u &&
                       uncertainty_ns <= (uint64_t)time.config.utc_max_uncertainty_ns &&
                       time.local_valid != 0u)
                          ? 1u
                          : 0u;

  if (time.local_valid == 0u) {
    state = TIME_QUALITY_INVALID;
  } else if (time.utc_have_mapping != 0u) {
    if (time.utc_valid == 0u) {
      state = TIME_QUALITY_DEGRADED;
    } else if (pps_timed_out != 0u || utc_timed_out != 0u) {
      state = TIME_QUALITY_HOLDOVER;
    } else if (time.discipline_state == LOCAL_TIMEBASE_LOCKED &&
               uncertainty_ns <= (uint64_t)time.config.utc_locked_uncertainty_ns) {
      state = TIME_QUALITY_LOCKED;
    } else {
      state = TIME_QUALITY_DEGRADED;
    }
  } else if (time.pps_have_last != 0u || time.pps_ever_locked != 0u) {
    state = TIME_QUALITY_ACQUIRING;
  } else {
    state = TIME_QUALITY_LOCAL_ONLY;
  }

  out->monotonic_now_us = now_us;
  out->utc_estimate_ns = (time.utc_have_mapping != 0u)
                             ? (now_us * 1000ull + time.utc_offset_ns)
                             : 0ull;
  out->utc_uncertainty_ns = (time.utc_have_mapping != 0u) ? uncertainty_ns : 0ull;
  out->holdover_duration_us = (pps_timed_out != 0u && now_us >= time.pps_last_valid_mono_us)
                                  ? (now_us - time.pps_last_valid_mono_us)
                                  : 0ull;
  if (utc_timed_out && now_us - time.utc_last_update_mono_us > out->holdover_duration_us)
    out->holdover_duration_us = now_us - time.utc_last_update_mono_us;
  out->last_valid_pps_us = time.pps_last_valid_mono_us;
  out->last_valid_reference_us = time.utc_last_update_mono_us;
  out->estimated_frequency_error_ppb = time.correction_ppb;
  out->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
  out->time_epoch = time.config.time_epoch;
  out->utc_mapping_epoch = time.utc_mapping_epoch;
  out->status_flags = 0u;
  out->state = state;
  out->reference_source = (time.pps_ever_locked != 0u) ? TIME_REF_GNSS_PPS
                                                          : (uint8_t)TIME_REF_NONE;
  out->utc_valid = time.utc_valid;
  out->local_clock_valid = time.local_valid;
  out->frequency_discipline_valid =
      (time.discipline_state != LOCAL_TIMEBASE_INVALID) ? 1u : 0u;
  return STATUS_OK;
}

static Status impl_time_get_local_discipline_status(LocalTimebaseDisciplineStatus *out)
{
  const TimeSegment *newest;

  if (out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  if (!time_initialized || time.segment_count == 0u) {
    return STATUS_NOT_READY;
  }
  newest = &time.segments[(time.segment_first + time.segment_count - 1u) %
                             TIME_SEGMENT_COUNT];
  out->time_epoch = time.config.time_epoch;
  out->discipline_epoch = time.discipline_epoch;
  out->anchor_tick64 = newest->anchor_tick64;
  out->anchor_monotonic_us = newest->anchor_mono_us;
  out->rate_correction_ppb = time.correction_ppb;
  out->rate_uncertainty_ppb = (time.discipline_state == LOCAL_TIMEBASE_LOCKED) ? 1000u
                                                                                  : 500000u;
  out->last_phase_error_ns = 0;
  out->last_valid_pps_tick64 = time.pps_last_tick64;
  out->last_update_monotonic_us = time.last_reanchor_mono_us;
  out->accepted_interval_count = (uint16_t)time.pps_good_intervals;
  out->discipline_state = time.discipline_state;
  out->valid = time.local_valid;
  return STATUS_OK;
}

static Status impl_time_get_mapping_status(uint32_t source_clock_domain_id, ClockDomainMappingStatus *out)
{
  uint64_t now_us = 0u;

  if (out == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(out, 0, sizeof(*out));
  if (!time_initialized) {
    return STATUS_NOT_READY;
  }
  (void)time_now(&time, &now_us);

  if (source_clock_domain_id == CLOCK_DOMAIN_UTC_POSIX && time.utc_have_mapping != 0u) {
    uint64_t uncertainty = time_utc_uncertainty_ns(&time, now_us);
    out->source_clock_domain_id = CLOCK_DOMAIN_UTC_POSIX;
    out->mapping_epoch = time.utc_mapping_epoch;
    out->local_time_epoch = time.config.time_epoch;
    out->anchor_source_time_ns = time.utc_anchor_mono_us * 1000ull + time.utc_offset_ns;
    out->anchor_local_time_us = time.utc_anchor_mono_us;
    out->source_rate_error_ppb = time.utc_rate_ppb;
    out->uncertainty_ns = uncertainty;
    out->last_sync_local_us = time.utc_last_update_mono_us;
    if (time.config.utc_max_uncertainty_ns > time.utc_accuracy_ns &&
        time.config.holdover_rate_bound_ppb > 0u) {
      uint64_t budget_ns = (uint64_t)time.config.utc_max_uncertainty_ns -
                           (uint64_t)time.utc_accuracy_ns;
      uint64_t valid_us = budget_ns * 1000ull /
                          (uint64_t)time.config.holdover_rate_bound_ppb;
      out->valid_until_local_us = time.utc_anchor_mono_us + valid_us;
    } else {
      out->valid_until_local_us = time.utc_anchor_mono_us;
    }
    if (time.utc_valid == 0u) {
      out->mapping_state = CLOCK_MAPPING_DEGRADED;
    } else if (time.discipline_state == LOCAL_TIMEBASE_HOLDOVER) {
      out->mapping_state = CLOCK_MAPPING_HOLDOVER;
    } else if (uncertainty <= time.config.utc_locked_uncertainty_ns &&
               time.discipline_state == LOCAL_TIMEBASE_LOCKED) {
      out->mapping_state = CLOCK_MAPPING_LOCKED;
    } else {
      out->mapping_state = CLOCK_MAPPING_DEGRADED;
    }
    return STATUS_OK;
  }

  if (time.ext_have_mapping != 0u && source_clock_domain_id == time.ext_domain_id) {
    uint64_t age_us = (now_us > time.ext_anchor_local_us) ? (now_us - time.ext_anchor_local_us)
                                                              : 0ull;
    out->source_clock_domain_id = time.ext_domain_id;
    out->mapping_epoch = time.ext_mapping_epoch;
    out->local_time_epoch = time.config.time_epoch;
    out->anchor_source_time_ns = time.ext_anchor_source_ns;
    out->anchor_local_time_us = time.ext_anchor_local_us;
    out->source_rate_error_ppb = time.ext_rate_ppb;
    out->uncertainty_ns = time.ext_uncertainty_ns +
                          age_us * (uint64_t)time.config.holdover_rate_bound_ppb / 1000ull;
    out->last_sync_local_us = time.ext_last_update_us;
    out->valid_until_local_us = 0u;
    out->mapping_state = CLOCK_MAPPING_LOCKED;
    return STATUS_OK;
  }
  return STATUS_NOT_READY;
}

static void time_pps_reacquire(void)
{
  time.pps_window_count = 0u;
  time.pps_good_intervals = 0u;
  time.target_correction_ppb = time.correction_ppb;
  if (time.local_valid) time.discipline_state = LOCAL_TIMEBASE_ACQUIRING;
  /* Keep the last trusted frequency/UTC model and local time epoch. */
}

static Status impl_time_submit_pps(const PpsCaptureEvent *event)
{
  uint64_t tick64;
  uint64_t mono_us = 0u;
  uint64_t nominal;
  int64_t delta;
  int64_t ppm;
  uint32_t i;

  if (event == NULL || !time_initialized) {
    return STATUS_NOT_READY;
  }
  if ((event->capture_flags & TIMER_CAPTURE_VALID) == 0u ||
      (event->capture_flags & (TIMER_CAPTURE_OVERCAPTURE |
                               TIMER_CAPTURE_OVERFLOW_AMBIGUOUS)) != 0u) {
    time.pps_rejected++;
    time_pps_reacquire();
    return STATUS_OUT_OF_RANGE;
  }
  if (event->time_epoch != time.config.time_epoch) {
    time.pps_rejected++;
    return STATUS_STALE_DATA;
  }
  tick64 = event->capture_tick64;
  if (time_map_internal(&time, tick64, &mono_us) != STATUS_OK) {
    time.pps_rejected++;
    return STATUS_STALE_DATA;
  }

  if (time.pps_have_last == 0u) {
    time.pps_have_last = 1u;
    time.pps_last_tick = tick64;
    time.pps_last_tick64 = tick64;
    time.pps_last_sequence = event->edge_sequence;
    time.pps_last_valid_sequence = event->edge_sequence;
    time.pps_last_valid_mono_us = mono_us;
    time.discipline_state = LOCAL_TIMEBASE_ACQUIRING;
    return STATUS_OK;
  }

  if (tick64 <= time.pps_last_tick) {
    time.pps_rejected++;
    time_pps_reacquire();
    return STATUS_STALE_DATA;
  }

  if (event->edge_sequence != time.pps_last_sequence + 1u) {
    time.pps_rejected++;
    time_pps_reacquire();
    time.pps_last_tick = tick64;
    time.pps_last_sequence = event->edge_sequence;
    return STATUS_STALE_DATA;
  }

  delta = (int64_t)(tick64 - time.pps_last_tick);
  nominal = (uint64_t)time.config.tick_hz;
  time.pps_last_tick = tick64;
  time.pps_last_sequence = event->edge_sequence;
  /* Long outages are a new acquisition window, never one enormous period.
   * Bound before multiplication to avoid overflow after a long silence. */
  if (delta > (int64_t)(2u * nominal)) {
    time.pps_rejected++;
    time_pps_reacquire();
    return STATUS_OUT_OF_RANGE;
  }
  ppm = (delta - (int64_t)nominal) * 1000000LL / (int64_t)nominal;

  if (ppm > TIME_PPS_PERIOD_TOLERANCE_PPM || ppm < -TIME_PPS_PERIOD_TOLERANCE_PPM ||
      delta <= 0) {
    time.pps_rejected++;
    time_pps_reacquire();
    return STATUS_OUT_OF_RANGE;
  }
  time.pps_last_tick64 = tick64;
  time.pps_last_valid_sequence = event->edge_sequence;
  time.pps_last_valid_mono_us = mono_us;
  if (time.discipline_state != LOCAL_TIMEBASE_LOCKED)
    time.discipline_state = LOCAL_TIMEBASE_ACQUIRING;

  if (time.pps_window_count == TIME_PPS_WINDOW) {
    for (i = 1u; i < TIME_PPS_WINDOW; i++) {
      time.pps_window[i - 1u] = time.pps_window[i];
    }
    time.pps_window_count--;
  }
  time.pps_window[time.pps_window_count++] = (uint64_t)delta;

  if (time.pps_good_intervals < 0xFFFFFFFFu) {
    time.pps_good_intervals++;
  }

  if (time.pps_window_count == TIME_PPS_WINDOW) {
    int64_t sum = 0;
    int64_t mean;
    uint64_t max_dev = 0u;
    uint64_t residual_us;
    for (i = 0u; i < TIME_PPS_WINDOW; i++) {
      sum += (int64_t)time.pps_window[i];
    }
    mean = sum / (int64_t)TIME_PPS_WINDOW;
    for (i = 0u; i < TIME_PPS_WINDOW; i++) {
      int64_t dev = (int64_t)time.pps_window[i] - mean;
      uint64_t magnitude = (uint64_t)(dev < 0 ? -dev : dev);
      if (magnitude > max_dev) {
        max_dev = magnitude;
      }
    }
    residual_us = max_dev * 1000000ull / (uint64_t)time.config.tick_hz;
    if (residual_us <= TIME_PPS_RESIDUAL_LIMIT_US) {
      int64_t err_ppm = (mean - (int64_t)nominal) * 1000000LL / (int64_t)nominal;
      time.target_correction_ppb = time_clamp_i32(-err_ppm * 1000LL,
                                                         TIME_RATE_CORRECTION_LIMIT_PPB);
      if (time.pps_good_intervals >= TIME_PPS_WINDOW && time.local_valid != 0u) {
        time.discipline_state = LOCAL_TIMEBASE_LOCKED;
        time.pps_ever_locked = 1u;
      }
    } else {
      time_pps_reacquire();
    }
  }
  return STATUS_OK;
}

static Status impl_time_submit_gnss_time(const GnssTimeObservation *observation)
{
  uint64_t mono_at_pps = 0u;
  uint64_t now_us = 0u;
  int64_t new_offset;
  int64_t diff;
  uint64_t paired_tick;

  if (observation == NULL || !time_initialized) {
    return STATUS_NOT_READY;
  }
  if ((observation->validity_flags & GNSS_TIME_UTC_VALID) == 0u ||
      observation->time_scale != TIME_SCALE_UTC) {
    time.utc_rejected++;
    return STATUS_UNSUPPORTED;
  }
  if (observation->time_epoch != time.config.time_epoch ||
      time.pps_have_last == 0u) {
    time.utc_rejected++;
    return STATUS_STALE_DATA;
  }
  paired_tick = observation->paired_pps_tick64;
  if (paired_tick != time.pps_last_tick64 &&
      (paired_tick + 1u != time.pps_last_tick64) &&
      (paired_tick != time.pps_last_tick64 + 1u)) {
    time.utc_rejected++;
    return STATUS_STALE_DATA;
  }
  if (observation->pps_edge_sequence != time.pps_last_valid_sequence) {
    time.utc_rejected++;
    return STATUS_STALE_DATA;
  }
  if (time_map_internal(&time, paired_tick, &mono_at_pps) != STATUS_OK) {
    time.utc_rejected++;
    return STATUS_STALE_DATA;
  }
  (void)time_now(&time, &now_us);

  new_offset = (int64_t)observation->utc_time_ns - (int64_t)(mono_at_pps * 1000ull);
  if (time.utc_have_mapping == 0u) {
    time.utc_offset_ns = (uint64_t)new_offset;
    time.utc_mapping_epoch++;
  } else {
    diff = new_offset - (int64_t)time.utc_offset_ns;
    if (diff > 1000000000LL || diff < -1000000000LL) {
      time.utc_offset_ns = (uint64_t)new_offset;
      time.utc_mapping_epoch++;
    } else {
      time.utc_offset_ns = (uint64_t)((int64_t)time.utc_offset_ns + diff / 8);
    }
  }
  time.utc_have_mapping = 1u;
  time.utc_accuracy_ns = observation->receiver_accuracy_ns;
  time.utc_anchor_mono_us = mono_at_pps;
  time.utc_last_update_mono_us = now_us;
  time.utc_valid = 1u;
  return STATUS_OK;
}

static Status impl_time_submit_external_sync(const ExternalClockSyncObservation *observation)
{
  int64_t forward_ns;
  int64_t backward_ns;
  int64_t offset_ns;
  int64_t rtt_signed;
  uint64_t rtt_ns;

  if (observation == NULL || !time_initialized) {
    return STATUS_NOT_READY;
  }
  if (observation->source_clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC ||
      observation->source_clock_domain_id == CLOCK_DOMAIN_UTC_POSIX) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (observation->local_time_epoch != time.config.time_epoch) {
    return STATUS_STALE_DATA;
  }

  forward_ns = (int64_t)observation->source_receive_time_ns -
               (int64_t)(observation->local_send_time_us * 1000ull);
  backward_ns = (int64_t)observation->source_send_time_ns -
                (int64_t)(observation->local_receive_time_us * 1000ull);
  offset_ns = (forward_ns + backward_ns) / 2;
  rtt_signed = (int64_t)(observation->local_receive_time_us -
                         observation->local_send_time_us) *
                   1000LL -
               ((int64_t)observation->source_send_time_ns -
                (int64_t)observation->source_receive_time_ns);
  rtt_ns = (rtt_signed > 0) ? (uint64_t)rtt_signed : 0ull;
  if (observation->source_clock_domain_id != time.ext_domain_id) {
    time.ext_mapping_epoch++;
  }
  time.ext_domain_id = observation->source_clock_domain_id;
  time.ext_anchor_source_ns =
      (uint64_t)((int64_t)(observation->local_receive_time_us * 1000ull) + offset_ns);
  time.ext_anchor_local_us = observation->local_receive_time_us;
  time.ext_last_update_us = observation->local_receive_time_us;
  time.ext_uncertainty_ns = (uint64_t)observation->source_uncertainty_ns + rtt_ns / 2u;
  time.ext_rate_ppb = 0;
  time.ext_have_mapping = 1u;
  return STATUS_OK;
}

/* Optional platform serialization. Public time APIs are thread-context only.
 * A recursive priority-inheriting mutex is used on RT-Thread; internal public
 * calls may reenter these wrappers. Host/single-thread users may omit hooks. */
static TimeAccessHook access_lock, access_unlock;
static void *access_context;
void systime_set_access_hooks(TimeAccessHook lock, TimeAccessHook unlock, void *context)
{
  if ((lock == NULL) != (unlock == NULL)) return;
  access_lock = lock; access_unlock = unlock; access_context = context;
}
Status systime_init(const TimeConfig *config, TickSource tick_source)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_service_init(config, tick_source);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status systime_stop(void)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_service_stop();
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status systime_reset_epoch(uint32_t new_time_epoch)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_service_reset_epoch(new_time_epoch);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status clock_read(ClockInstant *out)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_clock_read(out);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status clock_now_us(uint64_t *mono_us)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_clock_now_us(mono_us);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status clock_ticks_to_mono_us(uint64_t tick64, uint32_t time_epoch, uint64_t *mono_us)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_clock_ticks_to_mono_us(tick64, time_epoch, mono_us);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_get_status(TimeServiceStatus *out)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_get_status(out);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_get_local_discipline_status(LocalTimebaseDisciplineStatus *out)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_get_local_discipline_status(out);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_get_mapping_status(uint32_t source_clock_domain_id, ClockDomainMappingStatus *out)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_get_mapping_status(source_clock_domain_id, out);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_submit_pps(const PpsCaptureEvent *event)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_submit_pps(event);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_get_latest_pps(PpsCaptureEvent *out)
{
  Status result=STATUS_NOT_READY;
  if(!out) return STATUS_INVALID_ARGUMENT;
  if(access_lock) access_lock(access_context);
  memset(out,0,sizeof(*out));
  if(time_initialized && time.pps_have_last && time.pps_last_valid_sequence) {
    out->capture_tick64=time.pps_last_tick64;
    out->time_epoch=time.config.time_epoch;
    out->edge_sequence=time.pps_last_valid_sequence;
    out->capture_flags=TIMER_CAPTURE_VALID;
    result=STATUS_OK;
  }
  if(access_unlock) access_unlock(access_context);
  return result;
}
Status time_submit_gnss_time(const GnssTimeObservation *observation)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_submit_gnss_time(observation);
  if (access_unlock) access_unlock(access_context);
  return result;
}
Status time_submit_external_sync(const ExternalClockSyncObservation *observation)
{
  Status result;
  if (access_lock) access_lock(access_context);
  result = impl_time_submit_external_sync(observation);
  if (access_unlock) access_unlock(access_context);
  return result;
}
