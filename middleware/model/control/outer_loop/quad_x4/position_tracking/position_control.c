#include "attitude_rate_control/controller_internal.h"
#include "mpc_diagnostics.h"

static MpcSnapshot cached_mpc;
/* Why the 1 kHz consumer accepted or rejected the snapshot.  Diagnostics only:
 * every clause below already decided the fallback before it is counted. */
static MpcConsumerStats consumer_stats;

void mpc_copy_consumer_stats(MpcConsumerStats *out)
{
  if (out) *out = consumer_stats;
}

static void note_snapshot_life(uint64_t expires_at_us, uint64_t evaluation_us)
{
  int64_t life = (int64_t)expires_at_us - (int64_t)evaluation_us;
  int64_t index = (life - MPC_LIFE_ORIGIN_US) / MPC_LIFE_BUCKET_US;
  if (index < 0) index = 0;
  if (index >= MPC_LIFE_BUCKETS) index = MPC_LIFE_BUCKETS - 1;
  consumer_stats.life_hist[index]++;
  consumer_stats.life_total++;
  if (consumer_stats.life_total == 1 || life < consumer_stats.life_min_us)
    consumer_stats.life_min_us = (int32_t)(life < INT32_MIN ? INT32_MIN : life);
  if (consumer_stats.life_total == 1 || life > consumer_stats.life_max_us)
    consumer_stats.life_max_us = (int32_t)(life > INT32_MAX ? INT32_MAX : life);
}

void position_control_reset(void)
{
  /* Control state only.  The consumption counters describe the whole run and
   * must survive control_backend_stop(), which is what runs before the Host SIL
   * writes its evidence; clearing them here made every run report zero samples. */
  memset(&cached_mpc, 0, sizeof(cached_mpc));
}

void mpc_consumer_reset(void)
{
  memset(&consumer_stats, 0, sizeof(consumer_stats));
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
  /* Every clause is counted, so the fallback reason is a measurement instead of
   * a guess.  The fallback itself is still the conjunction of all clauses. */
  consumer_stats.samples++;
  backup = 0;
  if (!(cached_mpc.expires_at_us > in->evaluation_time_us)) {
    consumer_stats.backup_expired++;
    backup = 1;
  }
  if (!same_clock(&cached_mpc.header, &in->navigation->header)) {
    consumer_stats.backup_clock++;
    backup = 1;
  }
  if (cached_mpc.state_epoch != in->navigation->state_epoch) {
    consumer_stats.backup_state_epoch++;
    backup = 1;
  }
  if (cached_mpc.origin_id != in->navigation->origin_id) {
    consumer_stats.backup_origin++;
    backup = 1;
  }
  if (cached_mpc.reference_epoch != epoch) {
    consumer_stats.backup_reference++;
    backup = 1;
  }
  if (cached_mpc.authority_epoch != in->authority_status->authority_epoch) {
    consumer_stats.backup_authority++;
    backup = 1;
  }
  if (cached_mpc.directive_epoch != in->directive->directive_epoch) {
    consumer_stats.backup_directive++;
    backup = 1;
  }
  if (cached_mpc.envelope_epoch != in->safety_envelope->envelope_epoch) {
    consumer_stats.backup_envelope++;
    backup = 1;
  }
  if (cached_mpc.model_epoch != in->control_model_view->model_epoch) {
    consumer_stats.backup_model++;
    backup = 1;
  }
  if (cached_mpc.safety_envelope_id !=
      in->safety_envelope->safety_envelope_id) {
    consumer_stats.backup_envelope_id++;
    backup = 1;
  }
  for (i = 0; i < 3; ++i)
    if (fabsf(in->navigation->core.position_ned_m[i] -
              cached_mpc.position[i]) > c->mpc_seed_position_error_limit_m) {
      consumer_stats.backup_seed_position++;
      backup = 1;
      break;
    }
  for (i = 0; i < 3; ++i)
    if (fabsf(in->navigation->core.velocity_ned_mps[i] -
              cached_mpc.velocity[i]) > c->mpc_seed_velocity_error_limit_mps) {
      consumer_stats.backup_seed_velocity++;
      backup = 1;
      break;
    }
  if (cached_mpc.header.boot_id)
    note_snapshot_life(cached_mpc.expires_at_us, in->evaluation_time_us);
  for (i = 0; i < 3; ++i)
    requested[i] = backup ? ref.acceleration_ned_mps2[i] +
                                c->fallback_kp_per_s2[i] *
                                    (ref.position_ned_m[i] -
                                     in->navigation->core.position_ned_m[i]) +
                                c->fallback_kd_per_s[i] *
                                    (ref.velocity_ned_mps[i] -
                                     in->navigation->core.velocity_ned_mps[i])
                          : cached_mpc.acceleration[i];
  if (!backup) {
    consumer_stats.used++;
    consumer_stats.diff_samples++;
    for (i = 0; i < 3; ++i) {
      float pd = ref.acceleration_ned_mps2[i] +
                 c->fallback_kp_per_s2[i] *
                     (ref.position_ned_m[i] -
                      in->navigation->core.position_ned_m[i]) +
                 c->fallback_kd_per_s[i] *
                     (ref.velocity_ned_mps[i] -
                      in->navigation->core.velocity_ned_mps[i]);
      float difference = cached_mpc.acceleration[i] - pd;
      float magnitude = fabsf(difference);
      if (difference < consumer_stats.diff_lo[i]) consumer_stats.diff_lo[i] = difference;
      if (difference > consumer_stats.diff_hi[i]) consumer_stats.diff_hi[i] = difference;
      if (magnitude > consumer_stats.diff_max_abs[i])
        consumer_stats.diff_max_abs[i] = magnitude;
    }
  }
  s = control_limit_acceleration(requested, m6.previous_accel, dt, in->safety_envelope, a);
  if (s) {
    consumer_stats.limit_rejected++;
    return s;
  }
  for (i = 0; i < 3; ++i)
    if (a[i] < in->control_model_view->bounds.input_min[i] - 1e-5f ||
        a[i] > in->control_model_view->bounds.input_max[i] + 1e-5f ||
        fabsf(a[i] - m6.previous_accel[i]) >
            in->control_model_view->bounds.input_slew_max[i] * dt + 1e-5f) {
      consumer_stats.bounds_rejected++;
      return STATUS_MODEL_OUT_OF_DOMAIN;
    }
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
