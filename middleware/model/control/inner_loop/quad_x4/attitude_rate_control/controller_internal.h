#ifndef CONTROLLER_INTERNAL_H
#define CONTROLLER_INTERNAL_H
#include "contract_checks/algorithm_checks.h"
#include "control_backend.h"
#include "mpc_diagnostics.h"
EXTERN_C_BEGIN
/* MpcRunStats and its getter are declared in mpc_diagnostics.h: the Host SIL
 * adapter that serializes the counters must include the producer's definition,
 * not a copy. */
void mpc_note_worker(uint64_t now_us, int ready);
#define CONTROL_GRAVITY_MPS2 9.80665f
#define MPC_VARIABLE_COUNT 15
#define MPC_CONSTRAINT_COUNT 190
typedef struct {
  float motor[4], achieved[4], residual[4];
  float roll_pitch_scale, thrust_scale, yaw_scale;
  uint32_t saturated_mask;
} ControlAllocation;
typedef struct {
  ContractHeader header;
  uint32_t state_epoch, origin_id, reference_epoch, authority_epoch,
      directive_epoch;
  uint32_t envelope_epoch, model_epoch, safety_envelope_id, solver_status;
  uint32_t seed_control_sequence;
  uint64_t expires_at_us;
  float acceleration[3], position[3], velocity[3];
} MpcSnapshot;
typedef struct {
  ProductControllerConfig config;
  float inverse[4][4], effectiveness[4][4];
  float integral[3], derivative[3], last_rate[3], last_desired[9], log_axis[3];
  float previous_accel[3], previous_motor[4];
  uint64_t last_time;
  uint32_t state_epoch, origin_id, reference_epoch, authority_epoch,
      directive_epoch;
  uint32_t envelope_epoch, model_epoch, mode, sequence;
  uint64_t boot_id;
  uint32_t time_epoch;
  int initialized, has_previous, has_motor;
  ControlHealth health;
  ActuatorDemand history[8];
  uint32_t last_receipt_sequence;
  uint32_t last_mpc_sequence, solver_failures;
} ControllerState;
extern ControllerState m6;
int control_inverse4(const float in[4][4], float out[4][4]);
int attitude_quat_matrix(const float q[4], float r[9]);
int attitude_so3_log(const float r[9], float previous_axis[3], float out[3]);
void attitude_relative(const float a[9], const float b[9], float out[9]);
int attitude_from_thrust(const float a[3], float yaw, float old_b2[3], float r[9],
                       float *thrust);
Status control_allocate(const float inv[4][4], const float eff[4][4],
                   const float wrench[4], const float lo[4], const float hi[4],
                   ControlAllocation *out);
Status control_allocate_limited(const float inv[4][4], const float eff[4][4],
                           const float wrench[4], const float lo[4],
                           const float hi[4], float min_thrust,
                           float max_thrust, ControlAllocation *out);
Status control_allocate_from_previous(const float inv[4][4], const float eff[4][4],
                                      const float wrench[4], const float previous[4],
                                      const float lo[4], const float hi[4],
                                      float min_thrust, float max_thrust, ControlAllocation *out);
/* Shared segment lookup: production and test hooks use this same function. */
/* Single local-time conversion (normally seconds_per_us = 1e-6f). The _ex
   variant is the same production conversion with an explicit additive time
   error used only by the M5-06 test hooks. */
float control_reference_convert_local_time(uint64_t local_us,
                                           float seconds_per_us);
float control_reference_convert_local_time_ex(uint64_t local_us,
                                              float seconds_per_us,
                                              double additive_seconds);
/* C-side gamma_2 predicate used by the M5-06 time-error test path. */
uint8_t control_reference_time_within_gamma2(float local_seconds,
                                             uint64_t local_us);
/* Shared quintic coefficient generation (production and test hooks). */
void control_reference_hermite_coefficients(const float p0[3], const float v0[3],
                                            const float a0[3], const float p1[3],
                                            const float v1[3], const float a1[3],
                                            float h, float c[3][6]);

Status control_reference_locate_segment(const MotionReference *ref, uint64_t time,
                                        const MotionReferencePoint **a,
                                        const MotionReferencePoint **b,
                                        float *local_seconds,
                                        uint32_t *step_us_out);

Status control_sample_reference(const MotionReference *ref, uint64_t time,
                    ProductMotionPoint *out);
Status controller_validate_input(const ControllerInput *in, uint32_t *ref_epoch,
                         uint16_t *mode);
typedef struct {
  float acceleration[3];
  float yaw_rad;
  int used_fallback;
  uint32_t solver_status;
} PositionControlOutput;
void position_control_reset(void);
Status position_control_track(const ControllerInput *input, uint32_t reference_epoch,
                              float dt, PositionControlOutput *out);
Status position_control_altitude(const ControllerInput *input, float dt,
                                 float *acceleration_down);
Status mpc_reset(void);
int mpc_read(MpcSnapshot *out);
void mpc_seed(const float a[3], uint32_t sequence);
Status mpc_solve(const ControllerInput *in,
                      const ProductControllerConfigPayload *cfg,
                      const float previous_accel[3], MpcSnapshot *out);
EXTERN_C_END
#endif
