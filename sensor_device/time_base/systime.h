/*
 * PLATFORM Time Service (A-owned service core).
 *
 * Authority: spec/02 section 1.4.2 (public time types and APIs),
 *            spec/05 section 4 (local monotonic clock and PPS frequency
 *            discipline), section 6 (external mapping) and section 7
 *            (TimeService state machine).
 *
 * The core is RTOS/HAL independent.  A board layer provides the consistent
 * 64-bit tick snapshot (TIM2 UIF extension) through TickSource, and the
 * service maintains:
 *   - a piecewise continuous tick64 -> FCU monotonic mapping (Q32.32 us/tick,
 *     64 retained segments, bounded rate correction and slew);
 *   - local frequency discipline from gated 1 Hz PPS edges (8-interval lock,
 *     1000 ppm period gate, 50 us residual gate, 500 ppm rate bound,
 *     5 ppm/s slew, 2.5 s timeout -> HOLDOVER);
 *   - a separate FCU monotonic -> UTC offset mapping with growing uncertainty
 *     and the LOCKED/DEGRADED/utc_valid thresholds from spec 05.
 *
 * Published monotonic time never steps backward; rate changes only affect
 * segments created after the change and never rewrite history.
 */
#ifndef SYSTIME_H
#define SYSTIME_H

#include "base_types/common_base.h"
EXTERN_C_BEGIN

#define TIME_SEGMENT_COUNT 64u
/* Native receiver adapter pairs only with the most recent accepted edge. */
Status time_get_latest_pps(PpsCaptureEvent *out);
#define TIME_PPS_WINDOW 8u

/* Design acceptance limits (spec 05 section 8.1). */
#define TIME_PPS_PERIOD_TOLERANCE_PPM 1000
#define TIME_PPS_RESIDUAL_LIMIT_US 50
#define TIME_RATE_CORRECTION_LIMIT_PPB 500000 /* 500 ppm */
#define TIME_RATE_SLEW_PPB_PER_S 5000          /* 5 ppm/s */
#define TIME_PPS_TIMEOUT_US 2500000u
#define TIME_UTC_TIMEOUT_US 2500000u
#define TIME_UTC_LOCK_UNCERTAINTY_NS 100000u   /* 100 us */
#define TIME_UTC_MAX_UNCERTAINTY_NS 1000000000u /* 1 s */

/* GnssTimeObservation.time_scale / validity_flags (02 leaves the values to
 * the implementation; these are the first-version encodings). */
#define TIME_SCALE_UTC 0u
#define TIME_SCALE_GPS 1u
#define TIME_SCALE_OTHER 2u
#define GNSS_TIME_UTC_VALID (1u << 0)
#define GNSS_TIME_PPS_PAIRED (1u << 1)

typedef struct {
  uint64_t boot_id;
  uint32_t time_epoch;       /* nonzero; changes only on clock discontinuity */
  uint32_t tick_hz;          /* measured/derived TIM2 tick frequency */
  int32_t initial_rate_correction_ppb;
  uint32_t pps_timeout_us;
  uint32_t utc_timeout_us;
  uint32_t utc_locked_uncertainty_ns;
  uint32_t utc_max_uncertainty_ns;
  uint32_t holdover_rate_bound_ppb; /* oscillator bound used for U(t) growth */
  uint8_t gnss_pps_installed;       /* fixed 1 for Flight UM982; signal loss never changes this */
} TimeConfig;

/* Implemented by the board layer: consistent (H, CNT) snapshot.  Must be
 * callable from thread context; may briefly mask interrupts (<2 us). */
typedef Status (*TickSource)(uint64_t *tick64, uint32_t *time_epoch);

typedef struct {
  uint64_t anchor_tick64;
  uint64_t anchor_mono_us;
  uint64_t scale_q32;
  uint32_t discipline_epoch;
} TimeSegment;

typedef struct {
  TimeConfig config;
  TickSource tick_source;

  TimeSegment segments[TIME_SEGMENT_COUNT];
  uint32_t segment_first;
  uint32_t segment_count;
  uint64_t last_mono_us;
  uint64_t last_reanchor_mono_us;
  int32_t correction_ppb;        /* applied rate correction */
  int32_t target_correction_ppb; /* from PPS estimate */
  uint64_t last_slew_mono_us;
  uint32_t discipline_epoch;
  uint8_t local_valid;
  uint8_t pps_ever_locked;
  uint8_t discipline_state;      /* LocalTimebaseDisciplineState */

  uint8_t pps_have_last;
  uint64_t pps_last_tick;
  uint32_t pps_last_sequence;
  uint32_t pps_last_valid_sequence; /* accepted edge paired with pps_last_tick64 */
  uint32_t pps_good_intervals;
  uint32_t pps_rejected;
  uint64_t pps_window[TIME_PPS_WINDOW];
  uint32_t pps_window_count;
  uint64_t pps_last_valid_mono_us;
  uint64_t pps_last_tick64;

  uint8_t utc_have_mapping;
  uint8_t utc_valid;
  uint64_t utc_offset_ns;
  uint64_t utc_anchor_mono_us;
  uint64_t utc_last_update_mono_us;
  uint32_t utc_mapping_epoch;
  uint32_t utc_accuracy_ns;
  int32_t utc_rate_ppb;
  uint32_t utc_rejected;

  uint8_t ext_have_mapping;
  uint32_t ext_domain_id;
  uint64_t ext_anchor_source_ns;
  uint64_t ext_anchor_local_us;
  uint64_t ext_last_update_us;
  int32_t ext_rate_ppb;
  uint32_t ext_mapping_epoch;
  uint64_t ext_uncertainty_ns;
} TimeService;

Status systime_init(const TimeConfig *config, TickSource tick_source);
/* Set once before init. Hooks must be paired and recursively reentrant;
 * all public APIs are thread-context only while hooks are installed. */
typedef void (*TimeAccessHook)(void *context);
void systime_set_access_hooks(TimeAccessHook lock, TimeAccessHook unlock, void *context);
Status systime_stop(void);
Status systime_reset_epoch(uint32_t new_time_epoch);

/* Public Port functions (02 section 1.4.2). */
Status clock_read(ClockInstant *out);
Status clock_now_us(uint64_t *mono_us);
Status clock_ticks_to_mono_us(uint64_t tick64, uint32_t time_epoch, uint64_t *mono_us);
Status time_get_status(TimeServiceStatus *out);
Status time_get_local_discipline_status(LocalTimebaseDisciplineStatus *out);
Status time_get_mapping_status(uint32_t source_clock_domain_id, ClockDomainMappingStatus *out);
Status time_submit_pps(const PpsCaptureEvent *event);
Status time_submit_gnss_time(const GnssTimeObservation *observation);
Status time_submit_external_sync(const ExternalClockSyncObservation *observation);

/* Diagnostics/testing helpers. */
uint64_t time_map_tick(const TimeService *service, uint64_t tick64);
uint32_t time_segment_count(const TimeService *service);
int32_t time_applied_correction_ppb(const TimeService *service);

EXTERN_C_END
#endif /* SYSTIME_H */
