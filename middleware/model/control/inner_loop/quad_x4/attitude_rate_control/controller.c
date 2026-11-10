#include "attitude_rate_control/controller_internal.h"
#define ALLOCATION_BASE_MAX_AGE_US 25000u

ControllerState m6;
static int positive(const float *x, unsigned n, int zero) {
  unsigned i;
  for (i = 0; i < n; ++i)
    if (!isfinite(x[i]) || (zero ? x[i] < 0 : x[i] <= 0))
      return 0;
  return 1;
}
Status control_backend_validate(const ProductProfile *p) {
  if (!p)
    return STATUS_INVALID_ARGUMENT;
  return p->product_profile_id == PRODUCT_PROFILE_ID &&
                 p->nav_layout_id == PRODUCT_NAV_LAYOUT_ID &&
                 p->actuator_layout_id == PRODUCT_ACTUATOR_LAYOUT_ID &&
                 p->motion_reference_layout_id ==
                     PRODUCT_MOTION_REFERENCE_LAYOUT_ID
             ? STATUS_OK
             : STATUS_PROFILE_MISMATCH;
}
static Status validate_payload(const ProductControllerConfigPayload *p) {
  if (!isfinite(p->mass_kg) || p->mass_kg <= 0 ||
      !positive(p->so3_gain_per_s, 3, 0) ||
      !positive(p->rate_kp_nm_per_radps, 3, 1) ||
      !positive(p->rate_ki_nm_per_rad, 3, 1) ||
      !positive(p->rate_kd_nm_per_radps2, 3, 1) ||
      !positive(p->rate_kaw_per_s, 3, 1) ||
      !positive(p->rate_integral_limit_nm, 3, 1) ||
      !positive(p->fallback_kp_per_s2, 3, 0) ||
      !positive(p->fallback_kd_per_s, 3, 0) || !positive(p->mpc_q_diag, 6, 1) ||
      !positive(p->mpc_qf_diag, 6, 1) || !positive(p->mpc_r_diag, 3, 0) ||
      !positive(p->mpc_s_diag, 3, 1) || !isfinite(p->rate_d_cutoff_hz) ||
      p->rate_d_cutoff_hz <= 0 || p->rate_d_cutoff_hz >= 450 ||
      !isfinite(p->manual_hover_throttle_norm) ||
      p->manual_hover_throttle_norm <= 0 ||
      p->manual_hover_throttle_norm >= 1 ||
      !isfinite(p->actuation_preview_delay_s) ||
      p->actuation_preview_delay_s < 0 || p->actuation_preview_delay_s > .04f ||
      !positive(&p->mpc_seed_position_error_limit_m, 2, 0) ||
      !finite_array(p->motor_position_frd_m[0], 12) ||
      !finite_array(p->motor_yaw_torque_ratio_m, 4) ||
      !finite_array(p->inertia_frd_kgm2, 9))
    return STATUS_INVALID_ARGUMENT;
  /* Sylvester criterion and symmetry for the physical inertia tensor. */
  {
    const float *a = p->inertia_frd_kgm2;
    float det = a[0] * (a[4] * a[8] - a[5] * a[7]) -
                a[1] * (a[3] * a[8] - a[5] * a[6]) +
                a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (fabsf(a[1] - a[3]) > 1e-6f || fabsf(a[2] - a[6]) > 1e-6f ||
        fabsf(a[5] - a[7]) > 1e-6f || a[0] <= 0 ||
        a[0] * a[4] - a[1] * a[3] <= 0 || det <= 0)
      return STATUS_INVALID_ARGUMENT;
  }
  return STATUS_OK;
}
static int derive_allocation(const ProductControllerConfigPayload *p,
                             float effectiveness[4][4], float inverse[4][4]) {
  unsigned i;
  memset(effectiveness, 0, sizeof(float) * 16);
  for (i = 0; i < 4; ++i) {
    effectiveness[0][i] = 1;
    effectiveness[1][i] = -p->motor_position_frd_m[i][1];
    effectiveness[2][i] = p->motor_position_frd_m[i][0];
    effectiveness[3][i] = p->motor_yaw_torque_ratio_m[i];
  }
  return control_inverse4(effectiveness, inverse);
}
Status control_backend_init(const ProductControllerConfig *c) {
  ControllerState next;
  const ProductControllerConfigPayload *p;
  if (!c || !c->profile_hash || !c->parameter_hash || !c->config_version)
    return STATUS_INVALID_ARGUMENT;
  p = &c->payload;
  if (validate_payload(p) != STATUS_OK)
    return STATUS_INVALID_ARGUMENT;
  memset(&next, 0, sizeof(next));
  next.config = *c;
  if (!derive_allocation(p, next.effectiveness, next.inverse))
    return STATUS_INVALID_ARGUMENT;
  next.initialized = 1;
  next.health.common.health_state = MODULE_HEALTH_INITIALIZING;
  m6 = next;
  position_control_reset();
  mpc_consumer_reset();
  mpc_reset();
  return STATUS_OK;
}
/* Apply a parameter edit while the vehicle is disarmed.  The controller reads
 * m6.config.payload every cycle, so gains and the vehicle model take effect on
 * the next control tick; integrators are deliberately not reset. */
Status control_backend_apply_payload(const ProductControllerConfigPayload *p) {
  float effectiveness[4][4], inverse[4][4];
  if (!p || !m6.initialized)
    return STATUS_NOT_READY;
  if (validate_payload(p) != STATUS_OK)
    return STATUS_INVALID_ARGUMENT;
  if (!derive_allocation(p, effectiveness, inverse))
    return STATUS_INVALID_ARGUMENT;
  m6.config.payload = *p;
  memcpy(m6.effectiveness, effectiveness, sizeof(effectiveness));
  memcpy(m6.inverse, inverse, sizeof(inverse));
  return STATUS_OK;
}
Status control_backend_stop(void) {
  m6.initialized = 0;
  m6.has_previous = 0;
  m6.health.common.health_state = MODULE_HEALTH_STOPPED;
  position_control_reset();
  return STATUS_OK;
}
Status control_backend_get_health(ControlHealth *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  *out = m6.health;
  return m6.initialized ? STATUS_OK : STATUS_NOT_READY;
}
Status controller_validate_input(const ControllerInput *in, uint32_t *epoch,
                         uint16_t *mode) {
  const ContractHeader *h;
  const ContractHeader *headers[7];
  unsigned i;
  const ActuationPermit *p;
  const NavigationSnapshot *n;
  const GuidanceStatus *g;
  if (!in || !epoch || !mode || !in->navigation || !in->directive ||
      !in->safety_envelope || !in->authority_status || !in->actuation_permit ||
      !in->actuator_status || !in->actuator_capability || !in->guidance_status)
    return STATUS_INVALID_ARGUMENT;
  n = in->navigation;
  h = &n->header;
  p = in->actuation_permit;
  g = in->guidance_status;
  if (!header_valid(h, in->evaluation_time_us) ||
      h->frame_id != FRAME_LOCAL_NED || h->source_id != SOURCE_M3 ||
      in->control_time_us != h->timestamp_sample_us ||
      in->evaluation_time_us - in->control_time_us +
              h->event_time_uncertainty_us >
          5000)
    return STATUS_STALE_DATA;
  headers[0] = &in->directive->header;
  headers[1] = &in->safety_envelope->header;
  headers[2] = &in->authority_status->header;
  headers[3] = &p->header;
  headers[4] = &in->actuator_status->header;
  headers[5] = &in->actuator_capability->header;
  headers[6] = &g->header;
  for (i = 0; i < 7; ++i)
    if (!header_valid(headers[i], in->evaluation_time_us) ||
        !same_clock(h, headers[i]))
      return STATUS_STALE_DATA;
  if (in->directive->header.source_id != SOURCE_M4 ||
      p->header.source_id != SOURCE_M4 ||
      in->safety_envelope->header.source_id != SOURCE_M4 ||
      in->authority_status->header.source_id != SOURCE_M4 ||
      g->header.source_id != SOURCE_M5)
    return STATUS_NOT_PERMITTED;
  if (p->permit_flags != PERMIT_SCOPE_CONTROL_ACTIVE ||
      p->permit_expires_at_us <= in->evaluation_time_us || !p->permit_epoch ||
      p->state_epoch != n->state_epoch ||
      p->directive_epoch != in->directive->directive_epoch ||
      p->authority_epoch != in->authority_status->authority_epoch ||
      in->directive->authority_epoch != p->authority_epoch ||
      in->directive->state_epoch != n->state_epoch ||
      in->directive->directive_expires_at_us <= in->evaluation_time_us ||
      in->directive->operation_code == PRODUCT_OP_NONE ||
      in->directive->safety_envelope_id !=
          in->safety_envelope->safety_envelope_id)
    return STATUS_NOT_PERMITTED;
  if (in->actuator_capability->actuator_layout_id !=
          PRODUCT_ACTUATOR_LAYOUT_ID ||
      in->actuator_status->actuator_layout_id != PRODUCT_ACTUATOR_LAYOUT_ID ||
      (in->actuator_capability->available_mask &
       in->actuator_status->available_mask & 15u) != 15u ||
      (in->actuator_status->fault_mask & 15u))
    return STATUS_OUT_OF_RANGE;
  if ((n->core.valid_mask & (NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE)) !=
          (NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE) ||
      !finite_array(n->core.q_nb_wxyz, 4) ||
      !finite_array(n->core.body_rate_frd_radps, 3))
    return STATUS_NOT_READY;
  {
    const ProductSafetyEnvelopePayload *e =
        &in->safety_envelope->vehicle_extension;
    if ((e->valid_mask & 63u) != 63u ||
        !positive(e->max_velocity_ned_mps, 3, 0) ||
        !positive(e->max_acceleration_ned_mps2, 3, 0) ||
        !positive(e->max_jerk_ned_mps3, 3, 0) ||
        !positive(e->max_body_rate_frd_radps, 3, 0) ||
        !isfinite(e->max_tilt_rad) || e->max_tilt_rad <= 0 ||
        e->max_tilt_rad >= 1.4f || !isfinite(e->min_total_thrust_n) ||
        !isfinite(e->max_total_thrust_n) || e->min_total_thrust_n < 0 ||
        e->max_total_thrust_n <= e->min_total_thrust_n)
      return STATUS_OUT_OF_RANGE;
  }
  *mode = g->reference_kind;
  *epoch = g->reference_epoch;
  if ((g->guidance_state != GUIDANCE_STATE_READY &&
       g->guidance_state != GUIDANCE_STATE_ACTIVE &&
       g->guidance_state != GUIDANCE_STATE_HOLDING) ||
      g->reference_expires_at_us <= in->evaluation_time_us ||
      g->state_epoch != n->state_epoch || g->origin_id != n->origin_id ||
      g->authority_epoch != p->authority_epoch ||
      g->directive_epoch != p->directive_epoch ||
      g->reference_epoch != p->reference_epoch)
    return STATUS_NOT_PERMITTED;
  if (*mode == GUIDANCE_REFERENCE_TRAJECTORY) {
    const MotionReference *r = in->reference;
    uint32_t need = NAV_VALID_HEADING | NAV_VALID_HORIZONTAL_POSITION |
                    NAV_VALID_VERTICAL_POSITION |
                    NAV_VALID_HORIZONTAL_VELOCITY | NAV_VALID_VERTICAL_VELOCITY;
    if ((n->core.valid_mask & need) != need ||
        !finite_array(n->core.position_ned_m, 6))
      return STATUS_NOT_READY;
    if (!r || !header_valid(&r->header, in->evaluation_time_us) ||
        !same_clock(h, &r->header) ||
        r->header.frame_id != FRAME_LOCAL_NED ||
        r->header.timestamp_sample_us > in->control_time_us ||
        r->header.source_id != SOURCE_M5 ||
        r->state_epoch != n->state_epoch || r->origin_id != n->origin_id ||
        r->reference_epoch != *epoch ||
        r->directive_epoch != p->directive_epoch ||
        r->authority_epoch != p->authority_epoch ||
        r->reference_id != g->reference_id ||
        r->mission_epoch != g->mission_epoch ||
        r->current_item_id != g->current_item_id ||
        r->execution_token != g->execution_token || r->reference_kind != *mode)
      return STATUS_NOT_PERMITTED;
    if (in->directive->operation_code == PRODUCT_OP_PILOT &&
        in->directive->guidance_mode_code != PRODUCT_GUIDANCE_MODE_POS_HOLD)
      return STATUS_NOT_PERMITTED;
    if (!in->control_model_view)
      return STATUS_NOT_READY;
    {
      const ControlModelView *v = in->control_model_view;
      if (!header_valid(&v->header, in->evaluation_time_us) ||
          !same_clock(h, &v->header) ||
          v->header.frame_id != FRAME_LOCAL_NED ||
          v->descriptor.model_id != 0x1501u ||
          v->descriptor.model_version != 1 ||
          v->descriptor.model_kind != CONTROL_MODEL_KIND_LTI ||
          v->descriptor.state_layout_id !=
              PRODUCT_CONTROL_MODEL_STATE_LAYOUT_ID ||
          v->descriptor.input_layout_id !=
              PRODUCT_CONTROL_MODEL_INPUT_LAYOUT_ID ||
          fabsf(v->linear_model.sample_time_s - .02f) > 1e-7f ||
          !finite_array(v->linear_model.A, 36) ||
          !finite_array(v->linear_model.B, 18) ||
          !finite_array(v->linear_model.c, 6) ||
          !finite_array(v->bounds.state_min, 21))
        return STATUS_MODEL_OUT_OF_DOMAIN;
      /* This implementation binds only the declared fixed double integrator. */
      for (i = 0; i < 36; ++i) {
        float expected =
            i / 6 == i % 6 ? 1 : ((i / 6 < 3 && i % 6 == i / 6 + 3) ? .02f : 0);
        if (fabsf(v->linear_model.A[i] - expected) > 1e-7f)
          return STATUS_PROFILE_MISMATCH;
      }
      for (i = 0; i < 18; ++i) {
        float expected = i / 3 % 3 == i % 3 ? (i / 3 < 3 ? .0002f : .02f) : 0;
        if (fabsf(v->linear_model.B[i] - expected) > 1e-7f)
          return STATUS_PROFILE_MISMATCH;
      }
      for (i = 0; i < 6; ++i)
        if (fabsf(v->linear_model.c[i]) > 1e-7f ||
            v->bounds.state_min[i] > v->bounds.state_max[i] ||
            (i < 3
                 ? n->core.position_ned_m[i]
                 : n->core.velocity_ned_mps[i - 3]) < v->bounds.state_min[i] ||
            (i < 3 ? n->core.position_ned_m[i]
                   : n->core.velocity_ned_mps[i - 3]) > v->bounds.state_max[i])
          return STATUS_MODEL_OUT_OF_DOMAIN;
      for (i = 0; i < 3; ++i)
        if (v->bounds.input_min[i] > v->bounds.input_max[i] ||
            v->bounds.input_slew_max[i] <= 0)
          return STATUS_MODEL_OUT_OF_DOMAIN;
    }
  } else {
    const PilotControlReference *r = in->pilot_control_reference;
    uint16_t kind, expected;
    if (*mode == GUIDANCE_REFERENCE_PILOT_RATE) {
      kind = PILOT_REFERENCE_RATE;
      expected = PRODUCT_GUIDANCE_MODE_RATE;
    } else if (*mode == GUIDANCE_REFERENCE_PILOT_ATTITUDE) {
      kind = PILOT_REFERENCE_ATTITUDE;
      expected = PRODUCT_GUIDANCE_MODE_ATTITUDE;
    } else if (*mode == GUIDANCE_REFERENCE_ALTITUDE_PILOT ||
               *mode == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC) {
      kind = PILOT_REFERENCE_ALTITUDE_ATTITUDE;
      expected = PRODUCT_GUIDANCE_MODE_ALT_HOLD;
    } else
      return STATUS_UNSUPPORTED;
    if ((*mode == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC
             ? ((in->directive->operation_code != PRODUCT_OP_LAND &&
                 in->directive->operation_code != PRODUCT_OP_SAFE_ACTION) ||
                in->directive->guidance_mode_code != PRODUCT_GUIDANCE_MODE_ALT_HOLD)
             : (in->directive->operation_code != PRODUCT_OP_PILOT ||
                in->directive->guidance_mode_code != expected)) || !r ||
        !header_valid(&r->header, in->evaluation_time_us) ||
        !same_clock(h, &r->header) || r->reference_kind != kind ||
        r->header.frame_id != FRAME_LOCAL_NED ||
        r->header.source_id != SOURCE_M5 ||
        r->header.timestamp_sample_us > in->control_time_us ||
        r->reference_epoch != *epoch || r->reference_id != g->reference_id ||
        r->state_epoch != n->state_epoch || r->origin_id != n->origin_id ||
        r->authority_epoch != p->authority_epoch ||
        r->directive_epoch != p->directive_epoch)
      return STATUS_NOT_PERMITTED;
  }
  return STATUS_OK;
}
static Status fail_output(ControllerOutput *out, Status s, uint16_t reason) {
  memset(out, 0, sizeof(*out));
  m6.health.common.last_error_code = (uint32_t)s;
  ++m6.health.common.consecutive_error_count;
  m6.health.common.health_state = MODULE_HEALTH_DEGRADED;
  out->health = m6.health;
  out->capability.health_state = MODULE_HEALTH_DEGRADED;
  out->capability.reason_code = reason;
  m6.has_previous = 0;
  return s;
}
Status control_backend_update(const ControllerInput *in, ControllerOutput *out) {
  const ProductControllerConfigPayload *c = &m6.config.payload;
  const ProductSafetyEnvelopePayload *e;
  uint32_t epoch;
  uint16_t mode;
  Status s;
  int i, j, reset, same_navigation, backup = 0;
  float dt, r[9], rd[9], rel[9], err[3], rate[3], a[3] = {0},
                                                  w[4], lo[4], hi[4];
  float b2[3] = {0, 1, 0}, desired_rate[3] = {0};
  ControlAllocation allocation;
  uint32_t solver_status = CONTROL_SOLVER_NOT_RUN;
  uint64_t expiry;
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  if (!m6.initialized) {
    memset(out, 0, sizeof(*out));
    return STATUS_NOT_READY;
  }
  s = controller_validate_input(in, &epoch, &mode);
  if (s == STATUS_NOT_PERMITTED && m6.has_motor &&
      in->actuation_permit->permit_flags == PERMIT_SCOPE_IDLE_ONLY &&
      in->actuation_permit->header.source_id == SOURCE_M4 &&
      header_valid(&in->actuation_permit->header, in->evaluation_time_us) &&
      same_clock(&in->actuation_permit->header, &in->navigation->header))
    m6.has_motor = 0; /* Idle output ends the previous flight's slew history. */
  if (s)
    return fail_output(out, s,
                       s == STATUS_OUT_OF_RANGE
                           ? CONTROL_REASON_ALLOCATOR_RANK_DEFICIENT
                           : CONTROL_REASON_NONE);
  e = &in->safety_envelope->vehicle_extension;
  if(mode==GUIDANCE_REFERENCE_TRAJECTORY&&
     (e->max_total_thrust_n/(1.73205080757f*c->mass_kg)<CONTROL_GRAVITY_MPS2||
      e->min_total_thrust_n>c->mass_kg*CONTROL_GRAVITY_MPS2))
    return fail_output(out,STATUS_MODEL_OUT_OF_DOMAIN,CONTROL_REASON_MPC_HOVER_OUTSIDE_APPROX_DOMAIN);
  same_navigation = m6.boot_id == in->navigation->header.boot_id &&
                    m6.time_epoch == in->navigation->header.time_epoch &&
                    m6.state_epoch == in->navigation->state_epoch &&
                    m6.origin_id == in->navigation->origin_id;
  reset = !m6.has_previous || !same_navigation ||
          m6.reference_epoch != epoch ||
          m6.authority_epoch != in->authority_status->authority_epoch ||
          m6.directive_epoch != in->directive->directive_epoch ||
          m6.mode != mode;
  if (!reset && (in->control_time_us <= m6.last_time ||
                 in->control_time_us - m6.last_time < 950 ||
                 in->control_time_us - m6.last_time > 1100))
    return fail_output(out, STATUS_STALE_DATA, 0);
  dt = (m6.has_previous && in->control_time_us > m6.last_time)
           ? (float)(in->control_time_us - m6.last_time) * 1e-6f
           : .001f;
  if (dt < .00095f || dt > .0011f)
    return fail_output(out, STATUS_STALE_DATA, 0);
  if (reset) {
    memset(m6.integral, 0, sizeof(m6.integral));
    memset(m6.derivative, 0, sizeof(m6.derivative));
    position_control_reset();
  }
  if (!attitude_quat_matrix(in->navigation->core.q_nb_wxyz, r))
    return fail_output(out, STATUS_NUMERIC_ERROR, 0);
  if (mode == GUIDANCE_REFERENCE_TRAJECTORY) {
    PositionControlOutput position;
    s = position_control_track(in, epoch, dt, &position);
    if (s)
      return fail_output(out, s, 0);
    memcpy(a, position.acceleration, sizeof(a));
    backup = position.used_fallback;
    solver_status = position.solver_status;
    if (!attitude_from_thrust(a, position.yaw_rad, b2, rd, &w[0]))
      return fail_output(out, STATUS_NUMERIC_ERROR, 0);
  } else {
    const ProductPilotControlReferencePayload *p =
        &in->pilot_control_reference->vehicle_extension;
    uint32_t mask = p->valid_mask;
    if (mode == GUIDANCE_REFERENCE_PILOT_RATE) {
      if (!(mask & PRODUCT_PILOT_REF_VALID_RATE) ||
          !finite_array(p->body_rate_frd_radps, 3))
        return fail_output(out, STATUS_INVALID_ARGUMENT, 0);
      memcpy(rate, p->body_rate_frd_radps, sizeof(rate));
    } else {
      if (!(mask & PRODUCT_PILOT_REF_VALID_ATTITUDE) ||
          !attitude_quat_matrix(p->q_nb_desired_wxyz, rd) ||
          rd[8] < cosf(e->max_tilt_rad) - 1e-6f)
        return fail_output(out, STATUS_INVALID_ARGUMENT, 0);
    }
    if (mode == GUIDANCE_REFERENCE_ALTITUDE_PILOT ||
        mode == GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC) {
      s = position_control_altitude(in, dt, &a[2]);
      if (s)
        return fail_output(out, s, 0);
      w[0] = c->mass_kg * (CONTROL_GRAVITY_MPS2 - a[2]) / rd[8];
      if (w[0] < e->min_total_thrust_n || w[0] > e->max_total_thrust_n)
        return fail_output(out, STATUS_OUT_OF_RANGE, 0);
    } else {
      float x = p->collective_throttle_norm, h = c->manual_hover_throttle_norm,
            mg = c->mass_kg * CONTROL_GRAVITY_MPS2;
      if (!(mask & PRODUCT_PILOT_REF_VALID_THROTTLE) || !isfinite(x) || x < 0 ||
          x > 1 || mg < e->min_total_thrust_n || mg > e->max_total_thrust_n)
        return fail_output(out, STATUS_INVALID_ARGUMENT, 0);
      w[0] = x <= h
                 ? e->min_total_thrust_n + (mg - e->min_total_thrust_n) * x / h
                 : mg + (e->max_total_thrust_n - mg) * (x - h) / (1 - h);
    }
  }
  if (mode != GUIDANCE_REFERENCE_PILOT_RATE) {
    attitude_relative(r, rd, rel);
    if (!attitude_so3_log(rel, m6.log_axis, err))
      return fail_output(out, STATUS_NUMERIC_ERROR, 0);
    if (!reset) {
      float axis[3] = {0}, delta[3];
      attitude_relative(rd, m6.last_desired, rel);
      if (!attitude_so3_log(rel, axis, delta))
        return fail_output(out, STATUS_NUMERIC_ERROR, 0);
      for (i = 0; i < 3; ++i)
        desired_rate[i] = -delta[i] / dt;
    }
    attitude_relative(r, rd, rel);
    for (i = 0; i < 3; ++i) {
      rate[i] = c->so3_gain_per_s[i] * err[i];
      for (j = 0; j < 3; ++j)
        rate[i] += rel[3 * i + j] * desired_rate[j];
    }
  }
  for (i = 0; i < 3; ++i) {
    float measured = in->navigation->core.body_rate_frd_radps[i];
    float alpha = expf(-6.28318530718f * c->rate_d_cutoff_hz * dt);
    rate[i] = clip(rate[i], -e->max_body_rate_frd_radps[i],
                      e->max_body_rate_frd_radps[i]);
    if (!reset)
      m6.derivative[i] = alpha * m6.derivative[i] +
                         (1 - alpha) * (measured - m6.last_rate[i]) / dt;
    w[i + 1] = c->rate_kp_nm_per_radps[i] * (rate[i] - measured) +
               m6.integral[i] - c->rate_kd_nm_per_radps2[i] * m6.derivative[i];
  }
  for (i = 0; i < 4; ++i) {
    const ProductActuatorCapability *cap =
        &in->actuator_capability->vehicle_extension;
    if (!isfinite(cap->min_thrust_n[i]) || !isfinite(cap->max_thrust_n[i]) ||
        !isfinite(cap->max_slew_nps[i]) || cap->min_thrust_n[i] < 0 ||
        cap->max_thrust_n[i] < cap->min_thrust_n[i] ||
        cap->max_slew_nps[i] <= 0)
      return fail_output(out, STATUS_OUT_OF_RANGE, 0);
    lo[i] = cap->min_thrust_n[i];
    hi[i] = cap->max_thrust_n[i];
    if (m6.has_motor) {
      lo[i] = fmaxf(lo[i], m6.previous_motor[i] - cap->max_slew_nps[i] * dt);
      hi[i] = fminf(hi[i], m6.previous_motor[i] + cap->max_slew_nps[i] * dt);
    }
  }
  s = control_allocate_limited(m6.inverse, m6.effectiveness, w, lo, hi,
                          e->min_total_thrust_n, e->max_total_thrust_n,
                          &allocation);
  if (s == STATUS_OUT_OF_RANGE && m6.has_motor && same_navigation &&
      in->control_time_us > m6.last_time &&
      in->control_time_us - m6.last_time <= ALLOCATION_BASE_MAX_AGE_US) {
    s = control_allocate_from_previous(m6.inverse, m6.effectiveness, w, m6.previous_motor,
                                       lo, hi, e->min_total_thrust_n, e->max_total_thrust_n,
                                       &allocation);
  }
  if (s) {
    ++m6.health.allocation_failure_count;
    return fail_output(out, s, CONTROL_REASON_ALLOCATOR_INFEASIBLE);
  }
  /* Optional extra command-level anti-windup, only once per matching receipt.
   */
  {
    const ActuatorCommitReceipt *receipt = in->actuator_commit_receipt;
    if (receipt && header_valid(&receipt->header, in->evaluation_time_us) &&
        same_clock(&receipt->header, &in->navigation->header) &&
        receipt->header.source_id == SOURCE_M7 &&
        receipt->result_code == ACTUATOR_RECEIPT_COMMITTED &&
        receipt->receipt_expires_at_us > in->evaluation_time_us &&
        receipt->commit_time_us <= in->evaluation_time_us &&
        receipt->applied_control_sequence != m6.last_receipt_sequence &&
        receipt->committed_demand.valid_mask == 15u &&
        finite_array(receipt->committed_demand.motor_thrust_n, 4)) {
      const ActuatorDemand *old =
          &m6.history[receipt->applied_control_sequence % 8];
      if (old->control_sequence == receipt->applied_control_sequence &&
          old->permit_epoch == receipt->permit_epoch &&
          old->state_epoch == receipt->state_epoch &&
          old->reference_epoch == receipt->reference_epoch &&
          old->directive_epoch == receipt->directive_epoch &&
          old->authority_epoch == receipt->authority_epoch) {
        for (i = 0; i < 3; ++i)
          for (j = 0; j < 4; ++j)
            allocation.residual[i + 1] +=
                m6.effectiveness[i + 1][j] *
                (receipt->committed_demand.motor_thrust_n[j] -
                 old->demand.motor_thrust_n[j]);
        m6.last_receipt_sequence = receipt->applied_control_sequence;
      }
    }
  }
  for (i = 0; i < 3; ++i) {
    float error = rate[i] - in->navigation->core.body_rate_frd_radps[i];
    float integral =
        m6.integral[i] + (c->rate_ki_nm_per_rad[i] * error +
                          c->rate_kaw_per_s[i] * allocation.residual[i + 1]) *
                             dt;
    if (!isfinite(integral))
      return fail_output(out, STATUS_NUMERIC_ERROR, 0);
    m6.integral[i] = clip(integral, -c->rate_integral_limit_nm[i],
                             c->rate_integral_limit_nm[i]);
  }
  expiry = min_time(in->evaluation_time_us + 5000,
                       in->navigation->header.valid_until_us);
  expiry = min_time(expiry, in->actuation_permit->permit_expires_at_us);
  expiry = min_time(expiry, in->actuation_permit->header.valid_until_us);
  expiry = min_time(expiry, in->directive->directive_expires_at_us);
  expiry = min_time(expiry, in->directive->header.valid_until_us);
  expiry = min_time(expiry, in->safety_envelope->header.valid_until_us);
  expiry = min_time(expiry, in->authority_status->header.valid_until_us);
  expiry = min_time(expiry, in->actuator_status->header.valid_until_us);
  expiry = min_time(expiry, in->actuator_capability->header.valid_until_us);
  if (mode == GUIDANCE_REFERENCE_TRAJECTORY)
    expiry = min_time(expiry, in->control_model_view->header.valid_until_us);
  expiry = min_time(expiry, in->guidance_status->reference_expires_at_us);
  expiry = min_time(expiry, in->guidance_status->header.valid_until_us);
  expiry = min_time(
      expiry, mode == GUIDANCE_REFERENCE_TRAJECTORY
                  ? in->reference->header.valid_until_us
                  : in->pilot_control_reference->header.valid_until_us);
  memset(out, 0, sizeof(*out));
  ++m6.sequence;
  if (!m6.sequence)
    ++m6.sequence;
  algorithm_header(&out->actuator_demand.header, &in->navigation->header,
                      MSG_ACTUATOR_DEMAND, SOURCE_M6, m6.sequence,
                      in->evaluation_time_us, expiry);
  out->actuator_demand.header.frame_id = FRAME_BODY_FRD;
  out->actuator_demand.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  out->actuator_demand.valid_channel_mask = 15;
  out->actuator_demand.demand.valid_mask = 15;
  out->actuator_demand.permit_epoch = in->actuation_permit->permit_epoch;
  out->actuator_demand.directive_epoch = in->directive->directive_epoch;
  out->actuator_demand.authority_epoch = in->authority_status->authority_epoch;
  out->actuator_demand.state_epoch = in->navigation->state_epoch;
  out->actuator_demand.reference_epoch = epoch;
  out->actuator_demand.control_sequence = m6.sequence;
  out->actuator_demand.demand_expires_at_us = expiry;
  memcpy(out->actuator_demand.demand.motor_thrust_n, allocation.motor,
         sizeof(allocation.motor));
  algorithm_header(&m6.health.common.header, &in->navigation->header,
                      MSG_CONTROL_HEALTH, SOURCE_M6, m6.sequence,
                      in->evaluation_time_us, expiry);
  m6.health.common.health_state =
      backup ? MODULE_HEALTH_DEGRADED : MODULE_HEALTH_HEALTHY;
  m6.health.common.last_error_code = 0;
  m6.health.common.consecutive_error_count =
      mode == GUIDANCE_REFERENCE_TRAJECTORY ? m6.solver_failures : 0;
  m6.health.common.last_success_time_us = in->evaluation_time_us;
  m6.health.common.last_heartbeat_time_us = in->evaluation_time_us;
  m6.health.last_valid_command_us = in->control_time_us;
  m6.health.max_input_age_us =
      (uint32_t)(in->evaluation_time_us - in->control_time_us);
  m6.health.solver_status = solver_status;
  out->health = m6.health;
  out->capability.header = m6.health.common.header;
  out->capability.header.message_type = MSG_CONTROL_CAPABILITY;
  out->capability.control_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
  out->capability.controllable_dof_mask = 15;
  out->capability.health_state = m6.health.common.health_state;
  out->capability.control_level = (uint8_t)(backup ? 1 : 2);
  if (allocation.saturated_mask) {
    out->capability.reason_code = CONTROL_REASON_ACTUATOR_LIMITED;
    out->capability.degraded_dof_mask = 15;
  }
  m6.history[m6.sequence % 8] = out->actuator_demand;
  memcpy(m6.previous_motor, allocation.motor, sizeof(allocation.motor));
  m6.has_motor = 1;
  memcpy(m6.previous_accel, a, sizeof(a));
  memcpy(m6.last_rate, in->navigation->core.body_rate_frd_radps,
         sizeof(m6.last_rate));
  if (mode != GUIDANCE_REFERENCE_PILOT_RATE)
    memcpy(m6.last_desired, rd, sizeof(rd));
  m6.state_epoch = in->navigation->state_epoch;
  m6.origin_id = in->navigation->origin_id;
  m6.reference_epoch = epoch;
  m6.authority_epoch = in->authority_status->authority_epoch;
  m6.directive_epoch = in->directive->directive_epoch;
  m6.mode = mode;
  m6.boot_id = in->navigation->header.boot_id;
  m6.time_epoch = in->navigation->header.time_epoch;
  m6.last_time = in->control_time_us;
  m6.has_previous = 1;
  mpc_seed(a, m6.sequence);
  return STATUS_OK;
}
