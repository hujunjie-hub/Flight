/*
 * M4-04 HomeManager, M4-05 GeofenceManager, M4-06 PreArmManager and
 * M4-07 LandDetector.
 *
 * LandDetector follows 06 §7.1: the Host/SIL fixture (explicit ground-contact
 * evidence + 2 s stable dwell) is the only source of LANDED here; the real
 * airframe has no verified contact sensor, so without the product extension
 * the state stays UNKNOWN and Disarm/AUTO_LAND completion never becomes true.
 */
#include <string.h>
#include <math.h>
#include "flight_state_machine/fms_internal.h"
#include "geofence_geometry/geofence_geometry.h"

/* The FMT plant's frame-rate force ripple is not sustained vehicle motion.
 * Estimate acceleration from the navigation velocity only in Host/SIL. */
#define FMS_MOTION_TIME_CONSTANT_US 250000u

void fms_motion_tick(FmsEngine *e, const FmsInputs *in)
{
  if (e->config.payload.output_policy_id != 2u)
    return;

  FmsMotion *motion = &e->motion;
  const uint32_t required = NAV_VALID_ACCELERATION | NAV_VALID_HORIZONTAL_VELOCITY |
                            NAV_VALID_VERTICAL_VELOCITY;
  if (!in->nav_ok || !in->cap_ok ||
      (in->nav->core.valid_mask & in->cap->capability_mask & required) != required) {
    motion->sample_us = 0u;
    return;
  }
  const float *raw = in->nav->core.acceleration_ned_mps2;
  const float *velocity = in->nav->core.velocity_ned_mps;
  if (!fms_finite3(raw) || !fms_finite3(velocity) ||
      hypotf(hypotf(raw[0], raw[1]), raw[2]) > 6.0f) {
    motion->sample_us = 0u;
    return;
  }

  const uint64_t sample_us = in->nav->header.timestamp_sample_us;
  if (motion->sample_us == 0u || sample_us < motion->sample_us ||
      sample_us - motion->sample_us > 100000u) {
    motion->since_us = sample_us;
    memcpy(motion->velocity_filtered_ned_mps, velocity, sizeof(motion->velocity_filtered_ned_mps));
  } else if (sample_us != motion->sample_us) {
    const float dt_us = (float)(sample_us - motion->sample_us);
    const float alpha = dt_us / ((float)FMS_MOTION_TIME_CONSTANT_US + dt_us);
    for (unsigned axis = 0; axis < 3u; ++axis) {
      float *filtered = &motion->velocity_filtered_ned_mps[axis];
      *filtered += alpha * (velocity[axis] - *filtered);
      motion->acceleration_ned_mps2[axis] =
          (velocity[axis] - *filtered) * (1e6f / (float)FMS_MOTION_TIME_CONSTANT_US);
    }
  }
  motion->sample_us = sample_us;
}

const float *fms_motion_acceleration(const FmsEngine *e, const FmsInputs *in)
{
  if (e->config.payload.output_policy_id != 2u)
    return in->nav->core.acceleration_ned_mps2;
  if (e->motion.sample_us == 0u || e->motion.sample_us - e->motion.since_us < FMS_MOTION_TIME_CONSTANT_US)
    return NULL;
  return e->motion.acceleration_ned_mps2;
}

/* ------------------------------------------------------------------ */
/* land detector                                                      */
/* ------------------------------------------------------------------ */

void fms_land_init(FmsEngine *e)
{
  memset(&e->land, 0, sizeof(e->land));
  e->land.state = LANDED_STATE_UNKNOWN;
  e->land.candidate = LANDED_STATE_UNKNOWN;
  e->land.revision = 1u;
}

static uint32_t fms_land_dwell_us(uint8_t state)
{
  return (state == LANDED_STATE_LANDED) ? 2000000u : 200000u;
}

void fms_land_tick(FmsEngine *e, const FmsInputs *in)
{
  uint8_t desired = LANDED_STATE_UNKNOWN;
  uint8_t evidence = 0u;
  float confidence = 0.0f;
  float vz = 0.0f;
  uint8_t vz_valid = 0u;
  uint8_t ground_stable = 0u;

  if (in->nav_ok != 0u) {
    vz = in->nav->core.velocity_ned_mps[2];
    const float *acceleration = fms_motion_acceleration(e, in);
    vz_valid = (uint8_t)((in->nav->core.valid_mask & NAV_VALID_VERTICAL_VELOCITY) != 0u &&
                         fms_finite(vz));
    const uint32_t needed = NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY |
                            NAV_VALID_ACCELERATION;
    const float *v = in->nav->core.velocity_ned_mps;
    ground_stable = (uint8_t)(vz_valid != 0u && in->cap_ok != 0u && acceleration != NULL &&
        ((in->nav->core.valid_mask & in->cap->capability_mask) & needed) == needed &&
        fms_finite(v[0]) && fms_finite(v[1]) && hypotf(v[0], v[1]) <= 0.20f &&
        fabsf(vz) <= 0.15f && fms_finite(acceleration[2]) && fabsf(acceleration[2]) <= 0.5f);
  }

  if (in->ground_contact == GROUND_CONTACT_CONTACT && ground_stable != 0u) {
    desired = LANDED_STATE_LANDED;
    evidence = LANDED_EVIDENCE_GROUND_CONTACT | LANDED_EVIDENCE_VERTICAL_STABLE;
    confidence = 0.95f;
  } else if (in->ground_contact == GROUND_CONTACT_NO_CONTACT) {
    desired = LANDED_STATE_IN_AIR; /* explicit contact evidence beats speed heuristics */
    evidence = 0x1u;
    confidence = 0.9f;
  } else if (vz_valid != 0u && fabsf(vz) > 0.3f) {
    desired = LANDED_STATE_IN_AIR;
    evidence = 0x2u;
    confidence = 0.6f;
  } else if (in->ground_contact == GROUND_CONTACT_CONTACT && in->cap_ok != 0u &&
             (in->nav->core.valid_mask & in->cap->capability_mask &
              (NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY)) ==
                 (NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY) &&
             fms_finite(in->nav->core.velocity_ned_mps[0]) &&
             fms_finite(in->nav->core.velocity_ned_mps[1]) &&
             vz_valid != 0u && fabsf(vz) <= 0.3f) {
    /* Advisory contact candidate only. LANDED and DISARM still require the
     * unchanged velocity/acceleration gates and full 2 s stable dwell. */
    evidence = LANDED_EVIDENCE_GROUND_CONTACT;
    confidence = 0.5f;
  }

  if (e->flight.flight_mode == FLIGHT_MODE_AUTO_LAND && desired == LANDED_STATE_IN_AIR &&
      vz > 0.1f) {
    desired = LANDED_STATE_LANDING;
    confidence = 0.7f;
  }

  /* Losing the evidence cancels ground authorization immediately. The
   * confirmation dwell for the next state starts with this observation. */
  if (e->land.state == LANDED_STATE_LANDED && desired != LANDED_STATE_LANDED) {
    e->land.state = LANDED_STATE_UNKNOWN;
    e->land.since_us = in->now_us;
    e->land.revision++;
    e->land.evidence_mask = 0;
    e->land.confidence = 0.0f;
    e->land.stable_ms = 0u;
  }
  if (desired == LANDED_STATE_UNKNOWN) {
    if (e->land.state != LANDED_STATE_UNKNOWN) e->land.revision++;
    e->land.state = e->land.candidate = LANDED_STATE_UNKNOWN;
    /* Preserve a fresh physical-contact hint for M5 ground-settling only.
     * UNKNOWN never authorizes ARM/DISARM or a ground-stop certificate. */
    e->land.evidence_mask = evidence;
    e->land.confidence = confidence;
    e->land.stable_ms = 0u;
    e->land.candidate_since_us = in->now_us; e->land.since_us = in->now_us;
    return;
  }
  if (desired != e->land.candidate) {
    e->land.candidate = desired;
    e->land.candidate_since_us = in->now_us;
  }

  if (desired != LANDED_STATE_UNKNOWN && desired != e->land.state) {
    if (in->now_us - e->land.candidate_since_us >= fms_land_dwell_us(desired)) {
      e->land.state = desired;
      e->land.since_us = e->land.candidate_since_us;
      e->land.revision++;
      e->land.evidence_mask = evidence;
      e->land.confidence = confidence;
      e->land.stable_ms = (uint32_t)((in->now_us - e->land.since_us) / 1000u);
    }
  } else if (desired == e->land.state) {
    e->land.stable_ms = (uint32_t)((in->now_us - e->land.since_us) / 1000u);
    e->land.evidence_mask = evidence;
    e->land.confidence = confidence;
  }
}

/* ------------------------------------------------------------------ */
/* home                                                               */
/* ------------------------------------------------------------------ */

void fms_home_capture(FmsEngine *e, const FmsInputs *in, uint8_t source, uint16_t *reason)
{
  float std_h, std_v;

  if (in->nav_ok == 0u || in->nav->origin_id == 0u) {
    if (reason != NULL) {
      *reason = 13u;
    }
    return;
  }
  std_h = in->cap_ok ? in->cap->horizontal_position_std_m
                     : in->nav->core.position_std_m[0];
  std_v = in->cap_ok ? in->cap->vertical_position_std_m
                     : in->nav->core.position_std_m[2];
  if (!fms_finite(std_h) || !fms_finite(std_v)) {
    if (reason != NULL) {
      *reason = 5u;
    }
    return;
  }

  memset(&e->home.snap, 0, sizeof(e->home.snap));
  e->home.snap.home_id = (e->permit.epoch << 8) | 1u;
  e->home.snap.home_revision = 1u;
  e->home.snap.frame_id = FRAME_LOCAL_NED;
  e->home.snap.origin_id = in->nav->origin_id;
  e->home.snap.state_epoch = in->nav->state_epoch;
  e->home.snap.validity = PRODUCT_HOME_VALID;
  e->home.snap.quality = (uint8_t)(std_h <= e->policy->home_max_position_std_m ? 0u : 1u);
  e->home.snap.capture_source = source;
  e->home.snap.captured_at_us = in->now_us;
  e->home.snap.home_expires_at_us = fms_add_us(in->now_us, e->policy->home_ttl_us);
  e->home.snap.position_ned_m[0] = in->nav->core.position_ned_m[0];
  e->home.snap.position_ned_m[1] = in->nav->core.position_ned_m[1];
  e->home.snap.position_ned_m[2] = in->nav->core.position_ned_m[2];
  e->home.snap.position_std_m[0] = in->nav->core.position_std_m[0];
  e->home.snap.position_std_m[1] = in->nav->core.position_std_m[1];
  e->home.snap.position_std_m[2] = in->nav->core.position_std_m[2];
  e->home.valid = 1u;
}

void fms_home_tick(FmsEngine *e, const FmsInputs *in)
{
  if (e->home.valid == 0u) {
    return;
  }
  if (e->home.snap.validity != PRODUCT_HOME_VALID) {
    e->home.valid = 0u;
    return;
  }
  if (in->now_us >= e->home.snap.home_expires_at_us) {
    e->home.snap.validity = PRODUCT_HOME_INVALID;
    e->home.valid = 0u;
    return;
  }
  if (in->nav_ok != 0u &&
      (in->nav->origin_id != e->home.snap.origin_id ||
       in->nav->state_epoch != e->home.snap.state_epoch)) {
    /* Origin/state reset invalidates Home; never transform an old Home silently. */
    e->home.snap.validity = PRODUCT_HOME_INVALID;
    e->home.valid = 0u;
    fms_record_decision(e, DECISION_FORCE_SAFE_ACTION, 13u, 0u);
    return;
  }
  if ((e->flight.arm_state == ARM_STATE_ARMED ||
       (e->flight.arm_state == ARM_STATE_DISARMED &&
        e->home.snap.capture_source == PRODUCT_HOME_CAPTURE_OPERATOR)) &&
      fms_capable_for_mode(e, in, FLIGHT_MODE_POS_HOLD, 0u)) {
    e->home.snap.home_expires_at_us = fms_add_us(in->now_us, e->policy->home_ttl_us);
  }
}

/* ------------------------------------------------------------------ */
/* geofence                                                           */
/* ------------------------------------------------------------------ */

void fms_geofence_init(FmsEngine *e)
{
  memset(&e->geofence, 0, sizeof(e->geofence));
  e->geofence.revision = 0u;
}

Status fms_geofence_configure(FmsEngine *e, const ProductGeofenceSnapshot *fence)
{
  ProductGeofenceSnapshot *slot;

  if (fence == NULL) {
    return STATUS_INVALID_ARGUMENT;
  }
  if (!fence_geometry_valid(fence)) return STATUS_INVALID_ARGUMENT;
  if (e->geofence.fence_count >= FMS_MAX_GEOFENCES) {
    return STATUS_BUFFER_FULL;
  }
  slot = &e->geofence.fences[e->geofence.fence_count];
  *slot = *fence;
  memset(&slot->header, 0, sizeof(slot->header));
  e->geofence.fence_count++;
  e->geofence.revision++;
  return STATUS_OK;
}

uint8_t fms_geofence_contains(const FmsGeofence *g, const float p_ned[3], float home_down_m)
{
  uint16_t i;
  for (i = 0; i < g->fence_count; ++i)
    if (g->fences[i].validity != PRODUCT_GEOFENCE_VALID ||
        !fence_point_valid(&g->fences[i], p_ned, home_down_m)) return 0u;
  return 1u;
}

/* ------------------------------------------------------------------ */
/* prearm                                                            */
/* ------------------------------------------------------------------ */

static void fms_prearm_set(FmsPrearm *check, uint8_t id, uint8_t item)
{
  if (id < FMS_PREARM_MAX_DETAIL) {
    check->item_state[id] = item;
  }
}

static uint8_t fms_std_ok(float value, float limit)
{
  return (uint8_t)(fms_finite(value) && value >= 0 && fms_finite(limit) && value <= limit);
}

static void fms_prearm_check(const FmsEngine *e, const FmsInputs *in, FmsPrearm *check)
{
  uint32_t required = e->policy->required_arm_mask;
  uint32_t failed = 0u;
  uint32_t warning = 0u;
  uint16_t reason = 0u;
  uint8_t i;

  for (i = 0u; i < FMS_PREARM_MAX_DETAIL; ++i) {
    check->item_state[i] = PREARM_ITEM_UNKNOWN;
  }

  /* 0 PLATFORM_READY */
  if (e->clock.bound != 0u && in->nav_ok != 0u && in->act_ok != 0u &&
      e->config.payload.output_policy_id != 0u && e->flight.terminal_latched == 0u &&
      (in->faults_ok == 0u || in->faults->highest_severity < 3u)) {
    fms_prearm_set(check, PREARM_ID_PLATFORM_READY, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_PLATFORM_READY, PREARM_ITEM_FAIL);
  }

  /* 1 IMU_READY: external verified evidence only (M4 has no IMU driver view). */
  fms_prearm_set(check, PREARM_ID_IMU_READY,
                (uint8_t)((e->config.payload.verified_input_mask & (1u << 1)) != 0u
                              ? PREARM_ITEM_PASS
                              : PREARM_ITEM_UNKNOWN));

  /* 2 ESTIMATOR_ATTITUDE */
  if (in->nav_ok != 0u && in->cap_ok != 0u &&
      ((in->cap->capability_mask & in->nav->core.valid_mask) & NAV_VALID_ATTITUDE) != 0u &&
      fms_std_ok(in->nav->core.attitude_std_rad[0], e->config.payload.attitude_std_limit_rad) &&
      fms_std_ok(in->nav->core.attitude_std_rad[1], e->config.payload.attitude_std_limit_rad) &&
      fms_std_ok(in->nav->core.attitude_std_rad[2], e->config.payload.attitude_std_limit_rad)) {
    fms_prearm_set(check, PREARM_ID_ESTIMATOR_ATTITUDE, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_ESTIMATOR_ATTITUDE, PREARM_ITEM_FAIL);
  }

  /* 3 ESTIMATOR_POSITION */
  if (in->cap_ok != 0u && in->nav_ok != 0u &&
      ((in->cap->capability_mask & in->nav->core.valid_mask) &
       (NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION)) ==
          (NAV_VALID_HORIZONTAL_POSITION | NAV_VALID_VERTICAL_POSITION) &&
      fms_std_ok(in->cap->horizontal_position_std_m,
                e->config.payload.horizontal_position_std_limit_m) &&
      fms_std_ok(in->cap->vertical_position_std_m,
                e->config.payload.vertical_position_std_limit_m)) {
    fms_prearm_set(check, PREARM_ID_ESTIMATOR_POSITION, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_ESTIMATOR_POSITION, PREARM_ITEM_FAIL);
  }

  /* 4 RC_INPUT */
  if (in->rc_fresh != 0u && in->op_ok != 0u) {
    fms_prearm_set(check, PREARM_ID_RC_INPUT, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_RC_INPUT, PREARM_ITEM_FAIL);
  }

  /* 5 ACTUATOR_OUTPUT_READY */
  if (in->act_ok != 0u && in->act_cap->available_mask == 0x0Fu &&
      in->act_status->fault_mask == 0u) {
    fms_prearm_set(check, PREARM_ID_ACTUATOR_OUTPUT_READY, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_ACTUATOR_OUTPUT_READY, PREARM_ITEM_FAIL);
  }

  /* 6 BATTERY_ENERGY */
  if (in->ext_ok != 0u && (in->ext->valid_mask & PRODUCT_FMS_EXT_ENERGY_STATE) != 0u &&
      (in->ext->valid_mask & PRODUCT_FMS_EXT_BATTERY_FRACTION) != 0u) {
    if (in->energy_state == ENERGY_NORMAL &&
        in->battery_fraction >= e->policy->battery_arm_min_fraction && in->battery_fraction <= 1.0f) {
      fms_prearm_set(check, PREARM_ID_BATTERY_ENERGY, PREARM_ITEM_PASS);
    } else {
      fms_prearm_set(check, PREARM_ID_BATTERY_ENERGY, PREARM_ITEM_FAIL);
    }
  } else {
    fms_prearm_set(check, PREARM_ID_BATTERY_ENERGY, PREARM_ITEM_UNKNOWN);
  }

  /* 7 BARO_HEIGHT */
  fms_prearm_set(check, PREARM_ID_BARO_HEIGHT,
                (uint8_t)((e->config.payload.verified_input_mask & (1u << 7)) != 0u
                              ? PREARM_ITEM_PASS
                              : PREARM_ITEM_UNKNOWN));

  /* 8 GNSS_HOME_ORIGIN */
  if (in->nav_ok != 0u && in->nav->origin_id != 0u &&
      (e->flight.arm_state == ARM_STATE_DISARMED ||
       e->home.snap.validity == PRODUCT_HOME_VALID)) {
    fms_prearm_set(check, PREARM_ID_GNSS_HOME_ORIGIN, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_GNSS_HOME_ORIGIN, PREARM_ITEM_FAIL);
  }

  /* 9 COMPASS_HEADING: heading capability, not "magnetometer must be present". */
  if (in->cap_ok != 0u && (in->cap->capability_mask & NAV_VALID_HEADING) != 0u &&
      fms_std_ok(in->cap->heading_std_rad, e->config.payload.heading_std_limit_rad)) {
    fms_prearm_set(check, PREARM_ID_COMPASS_HEADING, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_COMPASS_HEADING, PREARM_ITEM_FAIL);
  }

  /* 10 CONTROL_MODEL */
  if (in->control_ok != 0u && in->control->health_state == 2u) {
    fms_prearm_set(check, PREARM_ID_CONTROL_MODEL, PREARM_ITEM_PASS);
  } else if ((e->config.payload.verified_input_mask & (1u << 10)) != 0u) {
    fms_prearm_set(check, PREARM_ID_CONTROL_MODEL, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_CONTROL_MODEL, PREARM_ITEM_UNKNOWN);
  }

  /* 11 CONTROL_SOLVER */
  fms_prearm_set(check, PREARM_ID_CONTROL_SOLVER,
                (uint8_t)((e->config.payload.verified_input_mask & (1u << 11)) != 0u
                              ? PREARM_ITEM_PASS
                              : PREARM_ITEM_UNKNOWN));

  /* 12 SAFETY_ENVELOPE */
  if (fms_finite(e->policy->default_max_tilt_rad) &&
      e->policy->default_max_tilt_rad > 0.0f &&
      fms_finite(e->policy->default_max_height_above_home_m) &&
      e->policy->default_max_height_above_home_m >
          e->policy->default_min_height_above_home_m &&
      e->config.payload.output_policy_id != 0u) {
    fms_prearm_set(check, PREARM_ID_SAFETY_ENVELOPE, PREARM_ITEM_PASS);
  } else {
    fms_prearm_set(check, PREARM_ID_SAFETY_ENVELOPE, PREARM_ITEM_FAIL);
  }

  /* 13 GEOFENCE */
  if (e->config.payload.geofence_required != 0u) {
    if (e->geofence.fence_count == 0u) {
      fms_prearm_set(check, PREARM_ID_GEOFENCE, PREARM_ITEM_FAIL);
    } else if (in->nav_ok != 0u &&
               fms_geofence_contains(&e->geofence, in->nav->core.position_ned_m,
                   e->home.valid ? e->home.snap.position_ned_m[2] : 0.0f)) {
      fms_prearm_set(check, PREARM_ID_GEOFENCE, PREARM_ITEM_PASS);
    } else {
      fms_prearm_set(check, PREARM_ID_GEOFENCE, PREARM_ITEM_FAIL);
    }
  } else {
    fms_prearm_set(check, PREARM_ID_GEOFENCE, PREARM_ITEM_PASS);
  }

  /* 14 STORAGE_EVENT_LOG */
  fms_prearm_set(check, PREARM_ID_STORAGE_EVENT_LOG,
                (uint8_t)((e->config.payload.verified_input_mask & (1u << 14)) != 0u
                              ? PREARM_ITEM_PASS
                              : PREARM_ITEM_WARNING));

  for (i = 0u; i < PREARM_ID_COUNT; ++i) {
    if ((required & (1u << i)) == 0u) {
      continue;
    }
    if (check->item_state[i] == PREARM_ITEM_FAIL) {
      failed |= (1u << i);
      if (reason == 0u) {
        reason = 2u; /* NOT_READY */
      }
    } else if (check->item_state[i] == PREARM_ITEM_UNKNOWN) {
      failed |= (1u << i);
      if (reason == 0u) {
        reason = 19u; /* POLICY_UNVERIFIED */
      }
    } else if (check->item_state[i] == PREARM_ITEM_WARNING) {
      warning |= (1u << i);
    }
  }

  check->failed_mask = failed;
  check->warning_mask = warning;
  check->primary_reason = reason;
  check->prearm_state = (uint8_t)(failed != 0u ? PREARM_REJECT : PREARM_READY);
}

void fms_prearm_evaluate(FmsEngine *e, const FmsInputs *in)
{
  fms_prearm_check(e, in, &e->prearm);
  e->prearm.check_epoch++;
  if (e->prearm.check_epoch == 0u) {
    e->prearm.check_epoch = 1u;
  }
  e->prearm.checked_at_us = in->now_us;
  e->prearm.valid = 1u;
  e->prearm_evaluations++;
}

uint8_t fms_prearm_ready_for_arm(const FmsEngine *e, const FmsInputs *in)
{
  FmsPrearm current;
  if (e->prearm.valid == 0u || e->prearm.prearm_state != PREARM_READY) {
    return 0u;
  }
  if (in->now_us < e->prearm.checked_at_us) {
    return 0u;
  }
  if (in->now_us - e->prearm.checked_at_us > 1000000u) {
    return 0u; /* stale result may not unlock */
  }
  fms_prearm_check(e, in, &current);
  return (uint8_t)(current.prearm_state == PREARM_READY);
}
