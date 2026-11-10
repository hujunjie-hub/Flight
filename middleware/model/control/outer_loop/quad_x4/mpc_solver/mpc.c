#include "attitude_rate_control/controller_internal.h"
#include "mpc_diagnostics.h"
#include "osqp/mpc_sparsity_pattern.h"
#include "osqp.h"
#include "quad_workspace.h"

/* Solver storage belongs exclusively to the 50 Hz worker. Exchange only
 * complete snapshots/seed commands under a bounded try-lock. Fast loop never
 * touches OSQP, never waits, and retains its last local copy on contention. */
static uint32_t exchange_lock, write_index;
static MpcSnapshot snapshots[2];
static float seed_accel[3];
static uint32_t seed_sequence;
static uint32_t solve_sequence;
static uint64_t last_solve_time;
/* Acceptance accounting for the real solver: a job is counted only when the
 * outer loop actually reaches mpc_solve(), so a scenario that never enters the
 * trajectory path reports zero jobs instead of a fake success rate. */
static MpcRunStats mpc_run_stats;
static MpcSnapshot last_context;
static OSQPFloat px[45], ax[702], q[15], lower[190], upper[190];

/* Per-job evidence.  The buffer exists only in Host builds; the firmware keeps
 * the aggregate counters and reports zero records.  Recording happens after the
 * decision it documents, so no control path depends on it. */
#if defined(FLIGHT_HOST_TEST)
#define MPC_DIAG_CAPACITY 1024u
static MpcJobRecord mpc_job_records[MPC_DIAG_CAPACITY];
static uint32_t mpc_job_record_written;
#else
#define MPC_DIAG_CAPACITY 0u
#endif
static uint32_t mpc_job_total;

uint32_t mpc_job_record_count(void) { return mpc_job_total; }

uint32_t mpc_copy_job_records(uint32_t first, MpcJobRecord *out,
                              uint32_t capacity, uint32_t *total)
{
#if defined(FLIGHT_HOST_TEST)
  uint32_t available = mpc_job_record_written;
  uint32_t count = 0;
  if (total) *total = mpc_job_total;
  if (first < available) {
    count = available - first;
    if (count > capacity) count = capacity;
    if (out && count)
      memcpy(out, &mpc_job_records[first], count * sizeof(mpc_job_records[0]));
  }
  return count;
#else
  (void)first;
  (void)out;
  (void)capacity;
  if (total) *total = mpc_job_total;
  return 0u;
#endif
}

static int lock_exchange(void) {
  return !__atomic_exchange_n(&exchange_lock, 1u, __ATOMIC_ACQUIRE);
}
static void unlock_exchange(void) {
  __atomic_store_n(&exchange_lock, 0u, __ATOMIC_RELEASE);
}
void mpc_seed(const float a[3], uint32_t sequence) {
  if (lock_exchange()) {
    memcpy(seed_accel, a, sizeof(seed_accel));
    seed_sequence = sequence;
    unlock_exchange();
  }
}
void mpc_note_worker(uint64_t now_us, int ready) {
  if (ready == 2) {   /* entry marker: the task was scheduled this tick */
    mpc_run_stats.entry_calls++;
    return;
  }
  mpc_run_stats.worker_ticks++;
  mpc_run_stats.last_worker_us = now_us;
  if (!ready) mpc_run_stats.worker_skips++;
}
void mpc_get_run_stats(MpcRunStats *out) {
  if (out) *out = mpc_run_stats;
}
/* The generated workspace terminates OSQP on
 *   eps_abs + eps_rel * max(||Px||, ||A'y||, ||q||)
 * which for this QP is about 2e-3, while mpc_solve() rejects a dual residual
 * above an absolute 1e-3.  The solver was therefore free to stop inside its own
 * contract and still be rejected downstream: 468 of 488 jobs in the 26 s MIL
 * failed exactly there (median dual residual 1.75e-3).  Terminating on the
 * absolute tolerance alone (eps_abs unchanged) makes the solver honour the gate
 * that the application already applies; it does not relax that gate. */
static void mpc_use_absolute_termination(void) {
  OSQPSettings settings;
  if (!quad_solver.settings) return;
  settings = *quad_solver.settings;
  settings.eps_rel = 0;
  (void)osqp_update_settings(&quad_solver, &settings);
}

void mpc_reset(void) {
  /* Lifecycle only; both update tasks must have stopped. */
  memset(snapshots, 0, sizeof(snapshots));
  memset(&last_context, 0, sizeof(last_context));
  memset(seed_accel, 0, sizeof(seed_accel));
  seed_sequence = 0;
  solve_sequence = 0;
  last_solve_time = 0;
  exchange_lock = 0;
  write_index = 0;
#if defined(FLIGHT_HOST_TEST)
  memset(mpc_job_records, 0, sizeof(mpc_job_records));
  mpc_job_record_written = 0;
#endif
  mpc_job_total = 0;
  mpc_use_absolute_termination();
  osqp_cold_start(&quad_solver);
}
int mpc_read(MpcSnapshot *out) {
  if (!lock_exchange())
    return 0;
  *out = snapshots[write_index];
  unlock_exchange();
  return out->header.boot_id != 0;
}
static float response(int step, int move, int velocity) {
  if (step <= move)
    return 0;
  if (move == 4) {
    float t = (step - 4) * .02f;
    return velocity ? t : .5f * t * t;
  }
  return velocity ? .02f : .0004f * (step - move - .5f);
}
static int same_context(const MpcSnapshot *a, const MpcSnapshot *b) {
  return same_clock(&a->header, &b->header) &&
         a->state_epoch == b->state_epoch && a->origin_id == b->origin_id &&
         a->reference_epoch == b->reference_epoch &&
         a->authority_epoch == b->authority_epoch &&
         a->directive_epoch == b->directive_epoch &&
         a->envelope_epoch == b->envelope_epoch &&
         a->model_epoch == b->model_epoch &&
         a->safety_envelope_id == b->safety_envelope_id;
}

/* Close one job record.  Every early return of mpc_solve() goes through here so
 * the rejecting check is named rather than inferred from the shared Status. */
static Status mpc_stop(MpcJobRecord *record, const MpcSnapshot *out,
                       uint32_t outcome, Status status) {
  record->outcome = outcome;
  record->status = (uint32_t)status;
  if (out) record->solver_status = out->solver_status;
  ++mpc_job_total;
#if defined(FLIGHT_HOST_TEST)
  if (mpc_job_record_written < MPC_DIAG_CAPACITY)
    mpc_job_records[mpc_job_record_written++] = *record;
#else
  (void)record;
  (void)out;
#endif
  return status;
}

static void mpc_record_begin(MpcJobRecord *record, const ControllerInput *in,
                             const float prev[3]) {
  memset(record, 0, sizeof(*record));
  record->job = mpc_run_stats.jobs + 1u;
  record->solver_status = CONTROL_SOLVER_NOT_RUN;
  memcpy(record->seed, prev, sizeof(record->seed));
  if (in->navigation) {
    record->sample_us = in->navigation->header.timestamp_sample_us;
    record->control_us = in->control_time_us;
  }
  record->eval_us = in->evaluation_time_us;
}

#if defined(FLIGHT_HOST_TEST)
/* outcomes are patched after mpc_solve(): the deadline and the expiry are
 * decided by the caller, not by the solver. */
static void mpc_record_last_expiry(uint64_t expires_us) {
  if (mpc_job_record_written)
    mpc_job_records[mpc_job_record_written - 1u].expires_us = expires_us;
}
static void mpc_record_last_deadline(uint32_t solver_status) {
  if (!mpc_job_record_written) return;
  {
    MpcJobRecord *last = &mpc_job_records[mpc_job_record_written - 1u];
    /* A job that already failed earlier keeps its first failing stage. */
    if (last->outcome == MPC_OUTCOME_OK) last->outcome = MPC_OUTCOME_DEADLINE;
    last->solver_status = solver_status;
  }
}
#else
static void mpc_record_last_expiry(uint64_t expires_us) { (void)expires_us; }
static void mpc_record_last_deadline(uint32_t solver_status) {
  (void)solver_status;
}
#endif

Status mpc_solve(const ControllerInput *in,
                      const ProductControllerConfigPayload *cfg,
                      const float prev[3], MpcSnapshot *out) {
  const ControlModelView *model = in->control_model_view;
  const ProductSafetyEnvelopePayload *e =
      &in->safety_envelope->vehicle_extension;
  float tangent = tanf(e->max_tilt_rad) * .923879532511f;
  float box = e->max_total_thrust_n / (1.73205080757f * cfg->mass_kg);
  ProductMotionPoint reference[20], anchor;
  MpcJobRecord record;
  int col, row, i, j, k, n;
  mpc_record_begin(&record, in, prev);
  if (!model || !isfinite(box) || box < CONTROL_GRAVITY_MPS2 ||
      e->min_total_thrust_n / cfg->mass_kg > CONTROL_GRAVITY_MPS2)
    return mpc_stop(&record, out, MPC_OUTCOME_DOMAIN, STATUS_MODEL_OUT_OF_DOMAIN);
  if (control_sample_reference(in->reference,
                   in->control_time_us +
                       (uint64_t)(cfg->actuation_preview_delay_s * 1e6f),
                   &anchor))
    return mpc_stop(&record, out, MPC_OUTCOME_REFERENCE, STATUS_STALE_DATA);
  for (k = 0; k < 20; ++k) {
    uint64_t time = in->control_time_us + (uint64_t)(k + 1) * 20000u +
                    (uint64_t)(cfg->actuation_preview_delay_s * 1e6f);
    if (control_sample_reference(in->reference, time, &reference[k]))
      return mpc_stop(&record, out, MPC_OUTCOME_REFERENCE, STATUS_STALE_DATA);
  }
  memset(q, 0, sizeof(q));
  for (col = 0; col < 15; ++col) {
    int axis = col % 3, move = col / 3;
    for (n = qp_p_ptr[col]; n < qp_p_ptr[col + 1]; ++n) {
      row = qp_p_row[n];
      float h = 0;
      int other = row / 3;
      for (k = 1; k <= 20; ++k)
        for (j = 0; j < 2; ++j) {
          float weight = k == 20 ? cfg->mpc_qf_diag[axis + 3 * j]
                                 : cfg->mpc_q_diag[axis + 3 * j];
          h += weight * response(k, move, j) * response(k, other, j);
        }
      if (row == col)
        h +=
            cfg->mpc_r_diag[axis] + cfg->mpc_s_diag[axis] * (move == 4 ? 1 : 2);
      else if (move == other + 1)
        h -= cfg->mpc_s_diag[axis];
      px[n] = 2 * h;
    }
    for (k = 1; k <= 20; ++k)
      for (j = 0; j < 2; ++j) {
        float base =
            j ? in->navigation->core.velocity_ned_mps[axis]
              : in->navigation->core.position_ned_m[axis] +
                    k * .02f * in->navigation->core.velocity_ned_mps[axis];
        float target = j ? reference[k - 1].velocity_ned_mps[axis]
                         : reference[k - 1].position_ned_m[axis];
        float weight = k == 20 ? cfg->mpc_qf_diag[axis + 3 * j]
                               : cfg->mpc_q_diag[axis + 3 * j];
        q[col] += 2 * weight * response(k, move, j) * (base - target);
      }
    /* Input cost is charged only for the five moves. */
    q[col] -= 2 * cfg->mpc_r_diag[axis] *
              (move ? reference[move - 1].acceleration_ned_mps2[axis]
                    : anchor.acceleration_ned_mps2[axis]);
    if (move == 0)
      q[col] -= 2 * cfg->mpc_s_diag[axis] * prev[axis];
    for (n = qp_a_ptr[col]; n < qp_a_ptr[col + 1]; ++n) {
      row = qp_a_row[n];
      if (row < 120)
        ax[n] = response(row / 6 + 1, move, row % 6 >= 3);
      else if (row < 135)
        ax[n] = 1;
      else if (row < 150)
        ax[n] = row - 135 == col ? 1 : -1;
      else {
        int side = (row - 150) % 8;
        ax[n] = axis == 0   ? cosf(side * .785398163397f)
                : axis == 1 ? sinf(side * .785398163397f)
                            : tangent;
      }
    }
  }
  for (k = 0; k < 20; ++k)
    for (i = 0; i < 6; ++i) {
      float base =
          i < 3 ? in->navigation->core.position_ned_m[i] +
                      (k + 1) * .02f * in->navigation->core.velocity_ned_mps[i]
                : in->navigation->core.velocity_ned_mps[i - 3];
      lower[k * 6 + i] = model->bounds.state_min[i] - base;
      upper[k * 6 + i] = model->bounds.state_max[i] - base;
      if (in->navigation->core.position_ned_m[i % 3] !=
          in->navigation->core.position_ned_m[i % 3])
        return mpc_stop(&record, out, MPC_OUTCOME_BOUNDS,
                        STATUS_INVALID_ARGUMENT);
    }
  for (k = 0; k < 5; ++k)
    for (i = 0; i < 3; ++i) {
      int index = k * 3 + i;
      lower[120 + index] =
          fmaxf(model->bounds.input_min[i], i == 2 ? CONTROL_GRAVITY_MPS2 - box : -box);
      upper[120 + index] =
          fminf(model->bounds.input_max[i],
                i == 2 ? CONTROL_GRAVITY_MPS2 - e->min_total_thrust_n / cfg->mass_kg : box);
      lower[135 + index] =
          -.02f * model->bounds.input_slew_max[i] + (k == 0 ? prev[i] : 0);
      upper[135 + index] =
          .02f * model->bounds.input_slew_max[i] + (k == 0 ? prev[i] : 0);
    }
  for (k = 150; k < 190; ++k) {
    lower[k] = -OSQP_INFTY;
    upper[k] = CONTROL_GRAVITY_MPS2 * tangent;
  }
  for (i = 0; i < 190; ++i)
    if (!isfinite(lower[i]) || !isfinite(upper[i]) || lower[i] > upper[i])
      return mpc_stop(&record, out, MPC_OUTCOME_BOUNDS, STATUS_OUT_OF_RANGE);
  if (osqp_update_data_mat(&quad_solver, px, 0, 45, ax, 0, 702) ||
      osqp_update_data_vec(&quad_solver, q, lower, upper) ||
      osqp_solve(&quad_solver))
    return mpc_stop(&record, out, MPC_OUTCOME_SOLVER_CALL, STATUS_NUMERIC_ERROR);
  switch (quad_solver.info->status_val) {
  case OSQP_SOLVED:
    out->solver_status = CONTROL_SOLVER_SOLVED;
    break;
  case OSQP_SOLVED_INACCURATE:
    out->solver_status = CONTROL_SOLVER_SOLVED_INACCURATE;
    break;
  case OSQP_MAX_ITER_REACHED:
    out->solver_status = CONTROL_SOLVER_MAX_ITER;
    return mpc_stop(&record, out, MPC_OUTCOME_SOLVER_STATUS, STATUS_NOT_READY);
  case OSQP_PRIMAL_INFEASIBLE:
  case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    out->solver_status = CONTROL_SOLVER_PRIMAL_INFEASIBLE;
    return mpc_stop(&record, out, MPC_OUTCOME_SOLVER_STATUS, STATUS_OUT_OF_RANGE);
  case OSQP_DUAL_INFEASIBLE:
  case OSQP_DUAL_INFEASIBLE_INACCURATE:
    out->solver_status = CONTROL_SOLVER_DUAL_INFEASIBLE;
    return mpc_stop(&record, out, MPC_OUTCOME_SOLVER_STATUS, STATUS_OUT_OF_RANGE);
  default:
    out->solver_status = CONTROL_SOLVER_NUMERICAL_FAILURE;
    return mpc_stop(&record, out, MPC_OUTCOME_SOLVER_STATUS, STATUS_NUMERIC_ERROR);
  }
  record.prim_res = quad_solver.info->prim_res;
  record.dual_res = quad_solver.info->dual_res;
  {
    uint32_t prim = (uint32_t)(quad_solver.info->prim_res * 1e6f);
    uint32_t dual = (uint32_t)(quad_solver.info->dual_res * 1e6f);
    if (prim > mpc_run_stats.max_prim_res_x1e6) mpc_run_stats.max_prim_res_x1e6 = prim;
    if (dual > mpc_run_stats.max_dual_res_x1e6) mpc_run_stats.max_dual_res_x1e6 = dual;
  }
  if (!isfinite(quad_solver.info->prim_res) ||
      !isfinite(quad_solver.info->dual_res) ||
      quad_solver.info->prim_res > 1e-3f || quad_solver.info->dual_res > 1e-3f)
    return mpc_stop(&record, out, MPC_OUTCOME_RESIDUAL, STATUS_NUMERIC_ERROR);
  /* Independent check against the unscaled original constraints. */
  {
    float y[190] = {0};
    float violation = 0;
    const OSQPFloat *x = quad_solver.solution->x;
    if (!finite_array(x, 15))
      return mpc_stop(&record, out, MPC_OUTCOME_NON_FINITE,
                      STATUS_NUMERIC_ERROR);
    for (col = 0; col < 15; ++col)
      for (n = qp_a_ptr[col]; n < qp_a_ptr[col + 1]; ++n)
        y[qp_a_row[n]] += ax[n] * x[col];
    for (i = 0; i < 190; ++i) {
      float below = lower[i] - y[i], above = y[i] - upper[i];
      if (below > violation) violation = below;
      if (above > violation) violation = above;
    }
    record.primal_violation = violation;
    if (violation > 1e-3f)
      return mpc_stop(&record, out, MPC_OUTCOME_PRIMAL, STATUS_OUT_OF_RANGE);
    for (k = 0; k < 5; ++k) {
      float a = x[3 * k], b = x[3 * k + 1], c = CONTROL_GRAVITY_MPS2 - x[3 * k + 2];
      float magnitude = cfg->mass_kg * sqrtf(a * a + b * b + c * c);
      float excess = magnitude - e->max_total_thrust_n;
      if (excess > record.thrust_excess_n) record.thrust_excess_n = excess;
      if (magnitude > e->max_total_thrust_n + 1e-3f)
        return mpc_stop(&record, out, MPC_OUTCOME_THRUST, STATUS_OUT_OF_RANGE);
    }
    memcpy(out->acceleration, x, 3 * sizeof(float));
    memcpy(record.accel, x, 3 * sizeof(float));
  }
  return mpc_stop(&record, out, MPC_OUTCOME_OK, STATUS_OK);
}
Status control_backend_outer_update(const ControllerInput *in) {
  MpcSnapshot result;
  ClockInstant start, end;
  uint16_t mode;
  uint32_t epoch;
  float previous[3];
  Status s;
  if (!m6.initialized) {
    mpc_run_stats.blocked_uninitialized++;
    return STATUS_NOT_READY;
  }
  {
    uint32_t age = in->evaluation_time_us > in->control_time_us
                       ? (uint32_t)(in->evaluation_time_us - in->control_time_us)
                       : 0u;
    mpc_run_stats.last_eval_age_us = age;
    mpc_run_stats.last_now_us = in->evaluation_time_us;
    mpc_run_stats.last_control_time_us = in->control_time_us;
    if (age > mpc_run_stats.max_eval_age_us) mpc_run_stats.max_eval_age_us = age;
  }
  s = controller_validate_input(in, &epoch, &mode);
  if (s) {
    mpc_run_stats.blocked_validation++;
    mpc_run_stats.last_validation_status = (int32_t)s;
    return s;
  }
  if (mode != GUIDANCE_REFERENCE_TRAJECTORY) {
    mpc_run_stats.blocked_mode++;
    return STATUS_UNSUPPORTED;
  }
  if (clock_read(&start) || start.monotonic_us < in->evaluation_time_us ||
      start.boot_id != in->navigation->header.boot_id ||
      start.time_epoch != in->navigation->header.time_epoch) {
    mpc_run_stats.blocked_clock++;
    return STATUS_STALE_DATA;
  }
  if (start.monotonic_us - in->control_time_us +
          in->navigation->header.event_time_uncertainty_us >
      5000) {
    mpc_run_stats.blocked_stale++;
    return STATUS_STALE_DATA;
  }
  if (last_solve_time && start.monotonic_us - last_solve_time < 20000) {
    mpc_run_stats.blocked_throttle++;
    return STATUS_NOT_READY;
  }
  last_solve_time = start.monotonic_us;
  if (!lock_exchange()) {
    mpc_run_stats.blocked_lock++;
    return STATUS_NOT_READY;
  }
  memcpy(previous, seed_accel, sizeof(previous));
  memset(&result, 0, sizeof(result));
  result.seed_control_sequence = seed_sequence;
  unlock_exchange();
  result.header = in->navigation->header;
  result.header.sequence = ++solve_sequence;
  result.state_epoch = in->navigation->state_epoch;
  result.origin_id = in->navigation->origin_id;
  result.reference_epoch = epoch;
  result.authority_epoch = in->authority_status->authority_epoch;
  result.directive_epoch = in->directive->directive_epoch;
  result.envelope_epoch = in->safety_envelope->envelope_epoch;
  result.model_epoch = in->control_model_view->model_epoch;
  result.safety_envelope_id = in->safety_envelope->safety_envelope_id;
  if (!same_context(&result, &last_context))
    osqp_cold_start(&quad_solver);
  last_context = result;
  s = mpc_solve(in, &m6.config.payload, previous, &result);
  mpc_run_stats.jobs++;
  mpc_run_stats.last_solver_status = (int32_t)result.solver_status;
  if (s == STATUS_OK) {
    mpc_run_stats.solved++;
    mpc_run_stats.covered_us += 20000u;
  } else {
    mpc_run_stats.rejected++;
    switch (s) {
    case STATUS_NUMERIC_ERROR: mpc_run_stats.rejected_numeric++; break;
    case STATUS_OUT_OF_RANGE: mpc_run_stats.rejected_range++; break;
    case STATUS_STALE_DATA: mpc_run_stats.rejected_stale++; break;
    case STATUS_MODEL_OUT_OF_DOMAIN: mpc_run_stats.rejected_domain++; break;
    default: break;
    }
  }
  switch (result.solver_status) {
  case CONTROL_SOLVER_SOLVED_INACCURATE: mpc_run_stats.solver_inaccurate++; break;
  case CONTROL_SOLVER_MAX_ITER: mpc_run_stats.solver_max_iter++; break;
  case CONTROL_SOLVER_PRIMAL_INFEASIBLE:
  case CONTROL_SOLVER_DUAL_INFEASIBLE: mpc_run_stats.solver_infeasible++; break;
  case CONTROL_SOLVER_DEADLINE_EXCEEDED: mpc_run_stats.solver_deadline++; break;
  default: break;
  }
  if (clock_read(&end) || end.boot_id != start.boot_id ||
      end.time_epoch != start.time_epoch ||
      end.monotonic_us < start.monotonic_us ||
      end.monotonic_us - start.monotonic_us > 2000) {
    s = STATUS_DEADLINE_MISSED;
    result.solver_status = CONTROL_SOLVER_DEADLINE_EXCEEDED;
    mpc_record_last_deadline(result.solver_status);
  }
  result.expires_at_us =
      min_time(start.monotonic_us + 20000, in->control_time_us + 25000);
  result.expires_at_us =
      min_time(result.expires_at_us, in->reference->header.valid_until_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->guidance_status->reference_expires_at_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->actuation_permit->permit_expires_at_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->actuation_permit->header.valid_until_us);
  result.expires_at_us =
      min_time(result.expires_at_us, in->directive->directive_expires_at_us);
  result.expires_at_us =
      min_time(result.expires_at_us, in->directive->header.valid_until_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->safety_envelope->header.valid_until_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->control_model_view->header.valid_until_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->guidance_status->header.valid_until_us);
  result.expires_at_us = min_time(
      result.expires_at_us, in->authority_status->header.valid_until_us);
  /* The plan expires with the inputs it was built from plus its own horizon.
   * Actuator capability/status used to be in this chain through their 5 ms
   * header TTL, but neither message enters the QP: the allocation limits are
   * enforced downstream by the controller and the guard, and the plan is not a
   * function of actuator telemetry.  A 5 ms telemetry TTL therefore capped every
   * plan at one quarter of the 20 ms period in which the worker can refresh it,
   * so a solved plan expired before the next solve and the 1 kHz consumer fell
   * back to PD for 19 of every 20 ms (FMAT-0012). */
  if (s && result.solver_status == CONTROL_SOLVER_NOT_RUN)
    result.solver_status = CONTROL_SOLVER_NUMERICAL_FAILURE;
  if (s)
    result.expires_at_us = 0;
  mpc_record_last_expiry(result.expires_at_us);
  memcpy(result.position, in->navigation->core.position_ned_m,
         sizeof(result.position));
  memcpy(result.velocity, in->navigation->core.velocity_ned_mps,
         sizeof(result.velocity));
  if (lock_exchange()) {
    write_index ^= 1;
    snapshots[write_index] = result;
    unlock_exchange();
  }
  return s;
}
