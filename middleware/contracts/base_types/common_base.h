/*
 * Flight public contract - common base layer.
 *
 * Authority: spec/02_接口契约与公共数据规范.md (V1.9.1-GLOBAL-AUDIT-R3).
 *
 * Layer position (spec 02 section 1.3):
 *   common_base.h  ->  product_contract.h  ->  common_messages.h  ->  port_api.h
 *
 * Contents: public status codes, ContractHeader, PLATFORM time/raw-acquisition
 * service structures, standard sensor samples, NavStateCore and the common
 * sensor diagnostics.  Nothing in this file may depend on the selected product,
 * RT-Thread, HAL, Simulink or C++ runtime types.
 *
 * All fields are normative.  Field order, width and alignment are locked by
 * tests/abi + middleware/contracts/product_contract/abi_lock.h;
 * changing any of them is an ICD change (schema Major bump + ABI hash change).
 */
#ifndef COMMON_BASE_H
#define COMMON_BASE_H

#include "base_types/contract_version.h"

EXTERN_C_BEGIN

/* ------------------------------------------------------------------------- */
/* Public status codes (spec 02 section 1.2)                                  */
/* ------------------------------------------------------------------------- */

typedef int32_t Status;

enum {
  STATUS_OK = 0,
  STATUS_INVALID_ARGUMENT = -1,
  STATUS_NOT_READY = -2,
  STATUS_OUT_OF_RANGE = -3,
  STATUS_UNSUPPORTED = -4,
  STATUS_NUMERIC_ERROR = -5,
  STATUS_DEADLINE_MISSED = -6,
  STATUS_PROFILE_MISMATCH = -7,
  STATUS_MODEL_OUT_OF_DOMAIN = -8,
  STATUS_BUFFER_FULL = -9,
  STATUS_STALE_DATA = -10,
  STATUS_NOT_PERMITTED = -11
};

/* ------------------------------------------------------------------------- */
/* ContractHeader (spec 02 section 1.4)                                       */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint16_t schema_major;          /* V1.9.1 schema major 6 / minor 0 */
  uint16_t schema_minor;
  uint16_t message_type;
  uint16_t source_id;
  uint32_t sequence;
  uint32_t profile_id;
  uint32_t frame_id;
  uint64_t boot_id;                /* unique boot session identifier */

  uint32_t clock_domain_id;       /* domain of timestamp_sample_us; FCU monotonic */
  uint32_t time_epoch;             /* local FCU monotonic continuity epoch */
  uint32_t source_clock_domain_id; /* raw source domain; 0 for native FCU-timed source */
  uint32_t source_mapping_epoch;   /* mapping used to convert source time; 0 if native */

  uint64_t timestamp_sample_us;   /* converted event/sample epoch in FCU monotonic */
  uint64_t timestamp_publish_us;  /* FCU monotonic publication time */
  uint64_t valid_until_us;        /* FCU monotonic expiry */

  uint32_t event_time_uncertainty_us;
  uint8_t  timestamp_quality;     /* TimestampQuality, not global TimeQuality */
  uint8_t  timestamp_flags;
  uint16_t reserved;
  uint32_t validity_flags;
} ContractHeader;

/* Header.validity_flags (spec 02 section 1.4.3). */
#define VALIDITY_PAYLOAD_VALID (1u << 0)
#define VALIDITY_TIME_VALID    (1u << 1)

/* Header.timestamp_flags (spec 02 section 1.4.3). */
#define TIMESTAMP_FLAG_PIPELINE_CORRECTED     (1u << 0)
#define TIMESTAMP_FLAG_EXTERNAL_CLOCK_MAPPED  (1u << 1)
#define TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED    (1u << 2)

/* Registered frame identifiers (spec 02 section 1.4.3). */
#define FRAME_UNSPECIFIED    0u
#define FRAME_LOCAL_NED      1u
#define FRAME_BODY_FRD       2u
#define FRAME_WGS84_GEODETIC 3u

/* Registered clock domains (spec 02 section 1.4.3). */
#define CLOCK_DOMAIN_FCU_MONOTONIC 0u
#define CLOCK_DOMAIN_UTC_POSIX     1u
#define CLOCK_DOMAIN_GNSS_GPS      2u
#define CLOCK_DOMAIN_COMPANION     0x1000u

/* Registered logical source identifiers (spec 02 section 1.4.3). */
#define SOURCE_RC         1u
#define SOURCE_GCS        2u
#define SOURCE_COMPANION  3u
#define SOURCE_IMU        10u
#define SOURCE_GNSS       11u
#define SOURCE_BARO       12u
#define SOURCE_MAG        13u
#define SOURCE_M3         20u
#define SOURCE_M4         21u
#define SOURCE_M5         22u
#define SOURCE_M6         23u
#define SOURCE_M7         24u
#define SOURCE_HEALTH     25u
#define SOURCE_PLATFORM   26u

/* ------------------------------------------------------------------------- */
/* PLATFORM time services (spec 02 section 1.4.1)                             */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint64_t boot_id;
  uint64_t capture_ticks64;
  uint64_t capture_time_us;
  uint32_t time_epoch;
  uint32_t discipline_epoch;
  uint32_t edge_sequence;
  uint32_t event_time_uncertainty_us;
  uint16_t timer_instance;
  uint8_t channel;
  uint8_t flags;
} TimerCaptureEvent;

/* TimerCaptureEvent.flags (spec 02 section 4, raw SPI link rules). */
#define TIMER_CAPTURE_VALID       (1u << 0)
#define TIMER_CAPTURE_OVERCAPTURE (1u << 1)
#define TIMER_CAPTURE_OVERFLOW_AMBIGUOUS (1u << 2)

typedef struct {
  uint64_t boot_id;
  uint64_t monotonic_us;
  uint32_t time_epoch;
  uint32_t reserved;
} ClockInstant;

/* ------------------------------------------------------------------------- */
/* Time quality and time-service structures (spec 02 section 1.4.2)           */
/* ------------------------------------------------------------------------- */

typedef enum {
  TIME_QUALITY_LOCAL_ONLY = 0,
  TIME_QUALITY_ACQUIRING  = 1,
  TIME_QUALITY_LOCKED     = 2,
  TIME_QUALITY_HOLDOVER   = 3,
  TIME_QUALITY_DEGRADED   = 4,
  TIME_QUALITY_INVALID    = 5
} TimeQuality;

/* Per-message timestamp provenance; distinct from global TimeQuality. */
typedef enum {
  TIMESTAMP_QUALITY_INVALID = 0,
  TIMESTAMP_QUALITY_HARDWARE_CAPTURED = 1,
  TIMESTAMP_QUALITY_SENSOR_MODEL_CORRECTED = 2,
  TIMESTAMP_QUALITY_EXTERNAL_MAPPED = 3,
  TIMESTAMP_QUALITY_RECEIVE_TIME_ESTIMATED = 4
} TimestampQuality;

typedef enum {
  TIME_REF_NONE = 0,
  TIME_REF_GNSS_PPS = 1,
  TIME_REF_RTC = 2,
  TIME_REF_COMPANION = 3,
  TIME_REF_HARDWARE_SYNC = 4
} TimeReferenceSource;

typedef enum {
  LOCAL_TIMEBASE_NOMINAL = 0,
  LOCAL_TIMEBASE_ACQUIRING = 1,
  LOCAL_TIMEBASE_LOCKED = 2,
  LOCAL_TIMEBASE_HOLDOVER = 3,
  LOCAL_TIMEBASE_DEGRADED = 4,
  LOCAL_TIMEBASE_INVALID = 5
} LocalTimebaseDisciplineState;

typedef struct {
  uint64_t capture_tick64;      /* extended raw TIM2 ticks; never reset by PPS */
  uint32_t edge_sequence;
  uint32_t time_epoch;          /* local raw-counter continuity epoch */
  uint16_t capture_flags;       /* valid edge / overcapture / polarity */
  uint16_t reserved;
} PpsCaptureEvent;

/* Frequency discipline of the FCU local tick-to-monotonic conversion.
 * This is NOT a command to reset TIM2 or step the monotonic clock. */
typedef struct {
  uint32_t time_epoch;
  uint32_t discipline_epoch;
  uint64_t anchor_tick64;
  uint64_t anchor_monotonic_us;
  int32_t  rate_correction_ppb;       /* applied gradually, bounded */
  uint32_t rate_uncertainty_ppb;      /* conservative bound */
  int64_t  last_phase_error_ns;       /* diagnostic only; no monotonic step */
  uint64_t last_valid_pps_tick64;
  uint64_t last_update_monotonic_us;
  uint16_t accepted_interval_count;
  uint8_t  discipline_state;          /* LocalTimebaseDisciplineState */
  uint8_t  valid;
} LocalTimebaseDisciplineStatus;

typedef struct {
  uint64_t utc_time_ns;         /* UTC only when validity flags confirm */
  uint64_t paired_pps_tick64;   /* raw local TIM2 tick captured for this UTC epoch */
  uint32_t pps_edge_sequence;
  uint32_t time_epoch;          /* same raw local tick epoch as paired_pps_tick64 */
  uint32_t receiver_accuracy_ns;
  uint16_t time_scale;          /* UTC/GPS/other, explicit */
  uint16_t validity_flags;
} GnssTimeObservation;

typedef struct {
  uint32_t source_clock_domain_id;
  uint32_t source_sequence;
  uint32_t local_time_epoch;
  uint32_t reserved0;
  uint64_t local_send_time_us;       /* local t1 */
  uint64_t source_receive_time_ns;   /* remote t2 */
  uint64_t source_send_time_ns;      /* remote t3 */
  uint64_t local_receive_time_us;    /* local t4 */
  uint32_t source_uncertainty_ns;
  uint32_t source_reset_counter;
  uint16_t validity_flags;
  uint16_t reserved;
} ExternalClockSyncObservation;

typedef enum {
  CLOCK_MAPPING_UNSYNCED = 0,
  CLOCK_MAPPING_ACQUIRING = 1,
  CLOCK_MAPPING_LOCKED = 2,
  CLOCK_MAPPING_HOLDOVER = 3,
  CLOCK_MAPPING_DEGRADED = 4,
  CLOCK_MAPPING_INVALID = 5
} ClockMappingState;

typedef struct {
  uint32_t source_clock_domain_id;
  uint32_t source_reset_counter;
  uint32_t mapping_epoch;
  uint32_t local_time_epoch;
  uint64_t anchor_source_time_ns;
  uint64_t anchor_local_time_us;
  int32_t  source_rate_error_ppb;  /* source clock rate relative to FCU monotonic */
  uint8_t  mapping_state;          /* ClockMappingState */
  uint8_t  reserved[3];
  uint64_t uncertainty_ns;         /* conservative bound at anchor_local_time_us */
  uint64_t last_sync_local_us;
  uint64_t valid_until_local_us;
} ClockDomainMappingStatus;

typedef struct {
  uint64_t monotonic_now_us;
  uint64_t utc_estimate_ns;       /* ignored when utc_valid == 0 */
  uint64_t utc_uncertainty_ns;
  uint64_t holdover_duration_us;
  uint64_t last_valid_pps_us;
  uint64_t last_valid_reference_us;
  int32_t  estimated_frequency_error_ppb; /* UTC/source mapping rate error, not local tick discipline */
  uint32_t clock_domain_id;       /* FCU monotonic domain */
  uint32_t time_epoch;             /* local continuity epoch */
  uint32_t utc_mapping_epoch;      /* UTC mapping epoch; 0 when unavailable */
  uint32_t status_flags;
  uint8_t  state;                  /* UTC/reference TimeQuality */
  uint8_t  reference_source;       /* TimeReferenceSource */
  uint8_t  utc_valid;
  uint8_t  local_clock_valid;      /* independent of GNSS/UTC */
  uint8_t  frequency_discipline_valid;
  uint8_t  reserved[3];
} TimeServiceStatus;

/* ------------------------------------------------------------------------- */
/* M2 standard sensor samples (spec 02 section 1.6)                           */
/* ------------------------------------------------------------------------- */

/* High-rate IMU fast path.  accel_frd_mps2 is specific force f, not
 * gravity-compensated linear acceleration. */
typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t sample_sequence;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float gyro_frd_radps[3];
  float accel_frd_mps2[3];
  float temperature_c;
} ImuSample;

/* ImuSample.quality_flags (spec 02 section 4). */
#define IMU_Q_GYRO_VALID  (1u << 0)
#define IMU_Q_ACCEL_VALID (1u << 1)
#define IMU_Q_TEMP_VALID  (1u << 2)
#define IMU_Q_CALIBRATED  (1u << 3)
#define IMU_Q_SATURATED   (1u << 4)

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t sample_sequence;
  uint32_t calibration_version;
  uint32_t quality_flags;
  double latitude_deg;
  double longitude_deg;
  float altitude_m;
  float velocity_ned_mps[3];
  float position_variance_m2[3];
  float velocity_variance_m2ps2[3];
  uint8_t fix_type;
  uint8_t satellite_count;
  uint16_t reserved;
} GnssSample;

/* GnssSample.quality_flags: independent position/velocity validity and
 * covariance usability.  A valid position does not imply valid velocity or
 * heading observability. */
#define GNSS_Q_POSITION_VALID       (1u << 0)
#define GNSS_Q_VELOCITY_VALID       (1u << 1)
#define GNSS_Q_POSITION_COV_VALID   (1u << 2)
#define GNSS_Q_VELOCITY_COV_VALID   (1u << 3)

/* GnssSample.fix_type (spec 02 section 4). */
#define GNSS_FIX_NONE           0u
#define GNSS_FIX_DEAD_RECKONING 1u
#define GNSS_FIX_SINGLE         2u
#define GNSS_FIX_DGPS           3u
#define GNSS_FIX_RTK_FLOAT      4u
#define GNSS_FIX_RTK_FIXED      5u

/* BaroSample.altitude_m is pressure altitude from pressure/temperature with a
 * configured pressure reference, positive Up; it is neither ellipsoid height
 * nor Home-relative height. */
typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float altitude_m;
  float variance_m2;
} BaroSample;

#define BARO_Q_ALTITUDE_VALID  (1u << 0)
#define BARO_Q_VARIANCE_VALID  (1u << 1)
#define BARO_Q_REFERENCE_VALID (1u << 2)

/* MagSample field and variance are in body-FRD microtesla and microtesla^2. */
typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float field_frd_ut[3];
  float variance_ut2[3];
} MagSample;

#define MAG_Q_FIELD_VALID           (1u << 0)
#define MAG_Q_CALIBRATED            (1u << 1)
#define MAG_Q_DISTURBANCE_SUSPECTED (1u << 2)

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float range_m;
  float variance_m2;
} RangeSample;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float velocity_xy_mps[2];
  float integration_time_s;
  float quality;
} FlowSample;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float wheel_speed_mps[4];
  float steering_angle_rad;
} WheelOdometrySample;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float true_airspeed_mps;
  float variance_m2ps2;
} AirspeedSample;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t calibration_version;
  uint32_t quality_flags;
  float water_speed_mps;
  float variance_m2ps2;
} WaterSpeedSample;

/* Range/Flow/Airspeed/WaterSpeed quality_flags (spec 02 section 4). */
#define SAMPLE_Q_MEASUREMENT_VALID (1u << 0)
#define SAMPLE_Q_VARIANCE_VALID    (1u << 1)
#define SAMPLE_Q_CALIBRATED        (1u << 2)

/* ------------------------------------------------------------------------- */
/* Navigation core (spec 02 section 1.7)                                      */
/* ------------------------------------------------------------------------- */

typedef struct {
  uint32_t valid_mask;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float acceleration_ned_mps2[3];
  float q_nb_wxyz[4];
  float body_rate_frd_radps[3];
  float position_std_m[3];
  float velocity_std_mps[3];
  float attitude_std_rad[3];
} NavStateCore;

/* NavStateCore.valid_mask / NavigationCapability.capability_mask bits. */
#define NAV_VALID_ATTITUDE            (1u << 0)
#define NAV_VALID_BODY_RATE           (1u << 1)
#define NAV_VALID_HEADING             (1u << 2)
#define NAV_VALID_HORIZONTAL_POSITION (1u << 3)
#define NAV_VALID_VERTICAL_POSITION   (1u << 4)
#define NAV_VALID_HORIZONTAL_VELOCITY (1u << 5)
#define NAV_VALID_VERTICAL_VELOCITY   (1u << 6)
#define NAV_VALID_ACCELERATION        (1u << 7)

/* ------------------------------------------------------------------------- */
/* Common sensor diagnostics (spec 02 section 1.6)                            */
/* ------------------------------------------------------------------------- */

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint16_t diagnostic_class;
  uint8_t severity;
  uint8_t flags;
  uint32_t reason_code;
  uint32_t occurrence_count;
  uint64_t last_good_sample_us;
} SensorDiagnostic;

typedef struct {
  ContractHeader header;
  uint32_t sensor_instance_id;
  uint32_t first_missing_sequence;
  uint32_t last_missing_sequence;
  uint32_t reason_code;
  uint64_t last_good_sample_time_us;
  uint64_t next_sample_time_us;
} SensorDataGapEvent;

/* ------------------------------------------------------------------------- */
/* Fundamental layout invariants                                              */
/* ------------------------------------------------------------------------- */

CONTRACT_STATIC_ASSERT(sizeof(Status) == 4, "Status must stay 32-bit");
CONTRACT_STATIC_ASSERT(sizeof(ContractHeader) == CONTRACT_HEADER_BYTES,
                 "ContractHeader layout changed; bump schema Major and ABI lock");
/* Natural alignment (8) is locked by tests/abi and abi_lock.h;
 * _Alignof is not available in the C99 firmware build, so it is not asserted
 * here. */
CONTRACT_STATIC_ASSERT(sizeof(float) == 4 && sizeof(double) == 8,
                 "IEEE-754 float32/double64 required");
CONTRACT_STATIC_ASSERT(sizeof(uint64_t) == 8 && sizeof(uint32_t) == 4,
                 "fixed-width integer contract broken");
CONTRACT_STATIC_ASSERT(sizeof(TimeQuality) == 4, "public enums are stored as 32-bit");

EXTERN_C_END

#endif /* COMMON_BASE_H */
