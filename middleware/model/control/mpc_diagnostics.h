#ifndef MPC_DIAGNOSTICS_H
#define MPC_DIAGNOSTICS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Aggregate tallies owned by the MPC worker and the outer loop.  The definition
 * lives here instead of controller_internal.h so the Host SIL adapter that
 * serializes these numbers includes the producer's own layout: a duplicated
 * typedef silently misreports every field as soon as one side gains a member. */
typedef struct {
  uint32_t jobs, solved, rejected;
  uint64_t covered_us;
  /* Why the outer loop never reached mpc_solve(): one counter per early return
   * in control_backend_outer_update(), so a zero-job run explains itself. */
  uint32_t blocked_uninitialized, blocked_validation, blocked_mode;
  uint32_t blocked_clock, blocked_stale, blocked_throttle, blocked_lock;
  int32_t last_validation_status;
  uint32_t last_eval_age_us, max_eval_age_us;
  uint64_t last_now_us, last_control_time_us;
  uint32_t worker_ticks, worker_skips, entry_calls;
  uint32_t rejected_numeric, rejected_range, rejected_stale, rejected_domain;
  uint32_t solver_inaccurate, solver_max_iter, solver_infeasible, solver_deadline;
  int32_t last_solver_status;
  uint32_t max_prim_res_x1e6, max_dual_res_x1e6;
  uint64_t last_worker_us;
} MpcRunStats;

/* M6 MPC evidence channel.
 *
 * The acceptance gate used to see only `jobs / solved / rejected` plus a maximum
 * residual, which cannot separate "the solver did not converge" from "the
 * solution violated a bound" from "the snapshot was already stale".  This header
 * defines the records the real firmware path writes so each rejected job carries
 * the check that rejected it, the solver's own numbers, and the three timestamps
 * that decide whether a solved job can still be consumed.
 *
 * Recording never changes a control decision: every field is written after the
 * decision it describes has already been taken.
 */

/* One entry per early return of mpc_solve() plus the outer loop's own deadline
 * check.  Several checks share a Status code, so the outcome, not the Status, is
 * what identifies the rejecting stage. */
enum {
  MPC_OUTCOME_OK = 0,
  MPC_OUTCOME_DOMAIN,        /* model view / thrust box preconditions */
  MPC_OUTCOME_REFERENCE,     /* control_sample_reference() failed */
  MPC_OUTCOME_BOUNDS,        /* assembled bounds non-finite or lower > upper */
  MPC_OUTCOME_SOLVER_CALL,   /* osqp update/solve returned non-zero */
  MPC_OUTCOME_SOLVER_STATUS, /* solver did not report SOLVED(_INACCURATE) */
  MPC_OUTCOME_NON_FINITE,    /* solution vector not finite */
  MPC_OUTCOME_RESIDUAL,      /* prim/dual residual over the 1e-3 gate */
  MPC_OUTCOME_PRIMAL,        /* independent A*x outside [lower,upper] +/-1e-3 */
  MPC_OUTCOME_THRUST,        /* independent thrust magnitude check */
  MPC_OUTCOME_DEADLINE,      /* 2 ms solver deadline overrun */
  MPC_OUTCOME_COUNT
};

/* Snapshots are consumed at 1 kHz and expire within a few tens of ms, so the
 * remaining life is bucketed in milliseconds over a window that covers both a
 * negative (already expired) and a full period. */
#define MPC_LIFE_BUCKETS 40
#define MPC_LIFE_ORIGIN_US (-8000)
#define MPC_LIFE_BUCKET_US 1000

typedef struct {
  uint32_t job;               /* 1-based job index, equals mpc_run_stats.jobs */
  uint32_t outcome;           /* MPC_OUTCOME_* */
  uint32_t solver_status;     /* CONTROL_SOLVER_* */
  uint32_t status;            /* Status returned by mpc_solve() */
  float prim_res, dual_res;   /* OSQP primal/dual residual at exit */
  float primal_violation;     /* max(lower-y, y-upper, 0) over the 190 rows */
  float thrust_excess_n;      /* thrust magnitude beyond the envelope, N */
  float accel[3];             /* first control move the consumer would use */
  float seed[3];              /* acceleration the slew constraint was seeded with */
  uint64_t sample_us;         /* navigation header timestamp_sample_us */
  uint64_t control_us;        /* in->control_time_us */
  uint64_t eval_us;           /* in->evaluation_time_us */
  uint64_t expires_us;        /* snapshot expiry, 0 when the job failed */
} MpcJobRecord;

/* Why the 1 kHz consumer used the snapshot or fell back to PD.  One counter per
 * clause of the acceptance predicate in position_control_track(). */
typedef struct {
  uint32_t samples, used;
  uint32_t backup_expired, backup_clock, backup_state_epoch, backup_origin;
  uint32_t backup_reference, backup_authority, backup_directive;
  uint32_t backup_envelope, backup_model, backup_envelope_id;
  uint32_t backup_seed_position, backup_seed_velocity;
  uint32_t limit_rejected, bounds_rejected; /* control_limit_acceleration losses */
  /* Signed (MPC - PD) difference over the ticks that actually used the snapshot,
   * so the gate can tell "the solver disagrees slightly" from "the solver is
   * pulling the loop the wrong way".  Positive means MPC asks for more along
   * that NED axis than the PD fallback would have. */
  float diff_lo[3], diff_hi[3], diff_max_abs[3];
  uint64_t diff_samples;
  uint32_t life_hist[MPC_LIFE_BUCKETS];     /* remaining life at consumption */
  uint32_t life_total;
  int32_t life_min_us, life_max_us;
} MpcConsumerStats;

/* Records are Host-only: the firmware path keeps the counters and skips the
 * record buffer.  Copies at most `capacity` records starting at `first` and
 * returns how many were copied; `total`, when non-NULL, receives the number of
 * jobs recorded so far, so a serializer can page through the buffer without
 * allocating it twice. */
/* Time-resolved consumption trace.  The aggregate counters say how often the
 * snapshot was used, not when, and the ground-phase question ("did the MPC
 * acceleration lift the vehicle before TAKEOFF?") needs the ordering.  Only the
 * takeoff window is recorded, so the buffer stays small. */
#define MPC_TRACE_CAPACITY 3000
#define MPC_TRACE_FIRST_US 4000000u
#define MPC_TRACE_LAST_US  6300000u
typedef struct {
  uint32_t time_us;
  uint32_t backup;   /* 1 = PD fallback was used, 0 = the snapshot was used */
  float mpc[3];
  float pd[3];
} MpcUseTrace;

uint32_t mpc_copy_use_trace(MpcUseTrace *out, uint32_t capacity);

uint32_t mpc_copy_job_records(uint32_t first, MpcJobRecord *out,
                              uint32_t capacity, uint32_t *total);
uint32_t mpc_job_record_count(void);
void mpc_copy_consumer_stats(MpcConsumerStats *out);
/* Clears the run-wide consumption counters.  Called when a run starts, never
 * from position_control_reset(): that one also runs at control_backend_stop(),
 * which happens before the Host SIL serializes its evidence. */
void mpc_consumer_reset(void);
void mpc_get_run_stats(MpcRunStats *out);

#ifdef __cplusplus
}
#endif

#endif
