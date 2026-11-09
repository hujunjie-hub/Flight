#include "attitude_rate_control/controller_internal.h"

static MpcSnapshot cached_mpc;

void position_control_reset(void)
{
  memset(&cached_mpc, 0, sizeof(cached_mpc));
}

static Status control_limit_acceleration(const float req[3], const float prev[3], float dt,
                                         const SafetyEnvelope *env, float out[3])
{
  const ProductSafetyEnvelopePayload *e = &env->vehicle_extension;
  float lo[3], hi[3], tanmax, box;
  int i, j;
  if (!finite_array(req, 3) || !finite_array(prev, 3))
    return STATUS_NUMERIC_ERROR;
  box = e->max_total_thrust_n / (1.73205080757f * m6.config.payload.mass_kg);
  tanmax = tanf(e->max_tilt_rad) * .92387953251f;
  for (i = 0; i < 3; ++i) {
    lo[i] = fmaxf(-e->max_acceleration_ned_mps2[i],
                  prev[i] - e->max_jerk_ned_mps3[i] * dt);
    hi[i] = fminf(e->max_acceleration_ned_mps2[i],
                  prev[i] + e->max_jerk_ned_mps3[i] * dt);
    lo[i] = fmaxf(lo[i], i == 2 ? CONTROL_GRAVITY_MPS2 - box : -box);
    hi[i] = fminf(hi[i], i == 2 ? CONTROL_GRAVITY_MPS2 - e->min_total_thrust_n /
                                             m6.config.payload.mass_kg
                                : box);
    if (lo[i] > hi[i])
      return STATUS_OUT_OF_RANGE;
    out[i] = clip(req[i], lo[i], hi[i]);
  }
  /* Bounded alternating half-space projection, followed by independent
     feasibility validation. Failure rejects the command, never relaxes bounds.
   */
  for (j = 0; j < 12; ++j)
    for (i = 0; i < 8; ++i) {
      float nx = cosf(i * .785398163397f), ny = sinf(i * .785398163397f);
      float d = nx * out[0] + ny * out[1] + tanmax * out[2] - tanmax * CONTROL_GRAVITY_MPS2;
      if (d > 0) {
        float scale = d / (1 + tanmax * tanmax);
        out[0] -= scale * nx;
        out[1] -= scale * ny;
        out[2] -= scale * tanmax;
        out[0] = clip(out[0], lo[0], hi[0]);
        out[1] = clip(out[1], lo[1], hi[1]);
        out[2] = clip(out[2], lo[2], hi[2]);
      }
    }
  for (i = 0; i < 8; ++i)
    if (cosf(i * .785398163397f) * out[0] + sinf(i * .785398163397f) * out[1] >
        tanmax * (CONTROL_GRAVITY_MPS2 - out[2]) + 1e-5f)
      return STATUS_OUT_OF_RANGE;
  if (m6.config.payload.mass_kg * sqrtf(out[0] * out[0] + out[1] * out[1] +
                                        (CONTROL_GRAVITY_MPS2 - out[2]) * (CONTROL_GRAVITY_MPS2 - out[2])) >
      e->max_total_thrust_n + 1e-5f)
    return STATUS_OUT_OF_RANGE;
  return STATUS_OK;
}

Status position_control_track(const ControllerInput *in, uint32_t epoch,
                              float dt, PositionControlOutput *out)
{
  const ProductControllerConfigPayload *c = &m6.config.payload;
  float *a = out->acceleration;
  float requested[3];
  ProductMotionPoint ref;
  int i, backup;
  uint64_t reference_time = in->control_time_us +
      (uint64_t)(c->actuation_preview_delay_s * 1e6f);
  Status s = control_sample_reference(in->reference, reference_time, &ref);
  if (s)
    return s;
  mpc_read(&cached_mpc);
  if (cached_mpc.header.boot_id &&
      cached_mpc.header.sequence != m6.last_mpc_sequence) {
    m6.last_mpc_sequence = cached_mpc.header.sequence;
    if (cached_mpc.solver_status == CONTROL_SOLVER_SOLVED ||
        cached_mpc.solver_status == CONTROL_SOLVER_SOLVED_INACCURATE)
      m6.solver_failures = 0;
    else {
      ++m6.solver_failures;
      if (cached_mpc.solver_status == CONTROL_SOLVER_DEADLINE_EXCEEDED)
        ++m6.health.deadline_miss_count;
    }
  }
  backup =
      !(cached_mpc.expires_at_us > in->evaluation_time_us &&
        same_clock(&cached_mpc.header, &in->navigation->header) &&
        cached_mpc.state_epoch == in->navigation->state_epoch &&
        cached_mpc.origin_id == in->navigation->origin_id &&
        cached_mpc.reference_epoch == epoch &&
        cached_mpc.authority_epoch == in->authority_status->authority_epoch &&
        cached_mpc.directive_epoch == in->directive->directive_epoch &&
        cached_mpc.envelope_epoch == in->safety_envelope->envelope_epoch &&
        cached_mpc.model_epoch == in->control_model_view->model_epoch &&
        cached_mpc.safety_envelope_id ==
            in->safety_envelope->safety_envelope_id);
  for (i = 0; i < 3; ++i) {
    if (fabsf(in->navigation->core.position_ned_m[i] -
              cached_mpc.position[i]) > c->mpc_seed_position_error_limit_m ||
        fabsf(in->navigation->core.velocity_ned_mps[i] -
              cached_mpc.velocity[i]) > c->mpc_seed_velocity_error_limit_mps)
      backup = 1;
  }
  for (i = 0; i < 3; ++i)
    requested[i] = backup ? ref.acceleration_ned_mps2[i] +
                                c->fallback_kp_per_s2[i] *
                                    (ref.position_ned_m[i] -
                                     in->navigation->core.position_ned_m[i]) +
                                c->fallback_kd_per_s[i] *
                                    (ref.velocity_ned_mps[i] -
                                     in->navigation->core.velocity_ned_mps[i])
                          : cached_mpc.acceleration[i];
  s = control_limit_acceleration(requested, m6.previous_accel, dt, in->safety_envelope, a);
  if (s)
    return s;
  for (i = 0; i < 3; ++i)
    if (a[i] < in->control_model_view->bounds.input_min[i] - 1e-5f ||
        a[i] > in->control_model_view->bounds.input_max[i] + 1e-5f ||
        fabsf(a[i] - m6.previous_accel[i]) >
            in->control_model_view->bounds.input_slew_max[i] * dt + 1e-5f)
      return STATUS_MODEL_OUT_OF_DOMAIN;
  out->yaw_rad = ref.yaw_rad;
  out->used_fallback = backup;
  out->solver_status = cached_mpc.solver_status;
  return STATUS_OK;
}

Status position_control_altitude(const ControllerInput *in, float dt,
                                 float *acceleration_down)
{
  const ProductControllerConfigPayload *c = &m6.config.payload;
  const ProductSafetyEnvelopePayload *e = &in->safety_envelope->vehicle_extension;
  const ProductPilotControlReferencePayload *p = &in->pilot_control_reference->vehicle_extension;
  const uint32_t mask = p->valid_mask;
  if ((mask & (PRODUCT_PILOT_REF_VALID_ALTITUDE_UP |
               PRODUCT_PILOT_REF_VALID_VERTICAL_UP)) !=
          (PRODUCT_PILOT_REF_VALID_ALTITUDE_UP |
           PRODUCT_PILOT_REF_VALID_VERTICAL_UP) ||
      (in->navigation->core.valid_mask &
       (NAV_VALID_VERTICAL_POSITION | NAV_VALID_VERTICAL_VELOCITY)) !=
          (NAV_VALID_VERTICAL_POSITION | NAV_VALID_VERTICAL_VELOCITY) ||
      !isfinite(p->altitude_above_origin_m) ||
      !isfinite(p->vertical_velocity_up_mps) ||
      !isfinite(in->navigation->core.position_ned_m[2]) ||
      !isfinite(in->navigation->core.velocity_ned_mps[2]))
    return STATUS_NOT_READY;
  *acceleration_down =
      -c->fallback_kp_per_s2[2] * (p->altitude_above_origin_m +
                                   in->navigation->core.position_ned_m[2]) -
      c->fallback_kd_per_s[2] * (p->vertical_velocity_up_mps +
                                 in->navigation->core.velocity_ned_mps[2]);
  float acceleration_limit = e->max_acceleration_ned_mps2[2];
  float max_delta = e->max_jerk_ned_mps3[2] * dt;
  *acceleration_down = clip(*acceleration_down, -acceleration_limit, acceleration_limit);
  *acceleration_down = clip(*acceleration_down, m6.previous_accel[2] - max_delta,
                            m6.previous_accel[2] + max_delta);
  return STATUS_OK;
}
