#include "observation_replay/ins_replay.hpp"
#include "kfgins/quad_core.hpp"
#include "ins_interface.h"
#include "control_interface.h"
#include "quad_workspace.h"
#include "attitude_rate_control/controller_internal.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>

static ClockInstant clock_value{0x12345678, 1000000, 1, 0};
static uint64_t clock_step;
static ClockDomainMappingStatus test_mapping{};
extern "C" Status clock_read(ClockInstant *out) {
  if (!out)
    return STATUS_INVALID_ARGUMENT;
  *out = clock_value;
  clock_value.monotonic_us += clock_step;
  return STATUS_OK;
}
extern "C" Status time_get_mapping_status(uint32_t domain,
                                             ClockDomainMappingStatus *out) {
  if(!domain || domain!=test_mapping.source_clock_domain_id) return STATUS_NOT_READY;
  *out=test_mapping;return STATUS_OK;
}
static unsigned checks, failures;
#define CHECK(x)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(x)) {                                                                \
      ++failures;                                                              \
      std::printf("FAIL %d: %s\n", __LINE__, #x);                              \
    }                                                                          \
  } while (0)
#define NEAR(a, b, t) CHECK(std::fabs((a) - (b)) <= (t))
static ContractHeader header(unsigned source,
                             unsigned frame = FRAME_LOCAL_NED,
                             uint64_t sample = 0) {
  ContractHeader h{};
  h.schema_major = 6;
  h.profile_id = PRODUCT_PROFILE_ID;
  h.boot_id = clock_value.boot_id;
  h.time_epoch = clock_value.time_epoch;
  h.timestamp_sample_us = sample ? sample : clock_value.monotonic_us;
  h.timestamp_publish_us = clock_value.monotonic_us;
  h.valid_until_us = clock_value.monotonic_us + 10000000;
  h.source_id = static_cast<uint16_t>(source);
  h.frame_id = frame;
  h.validity_flags = 3;
  h.timestamp_quality = TIMESTAMP_QUALITY_HARDWARE_CAPTURED;
  h.timestamp_flags = TIMESTAMP_FLAG_UNCERTAINTY_BOUNDED;
  h.event_time_uncertainty_us = 10;
  return h;
}
struct Fixture {
  NavigationSnapshot nav{};
  MotionReference ref{};
  PilotControlReference pilot{};
  GuidanceStatus guidance{};
  VehicleDirective directive{};
  SafetyEnvelope envelope{};
  AuthorityStatus authority{};
  ActuationPermit permit{};
  ActuatorStatus actuator{};
  ActuatorCapability capability{};
  ControlModelView model{};
  ControllerInput in{};
  Fixture() {
    nav.header = header(SOURCE_M3);
    nav.state_epoch = 1;
    nav.origin_id = 1;
    nav.core.valid_mask = 255;
    nav.core.q_nb_wxyz[0] = 1;
    ref.header = header(SOURCE_M5);
    ref.state_epoch = ref.origin_id = ref.reference_id = ref.reference_epoch =
        ref.authority_epoch = ref.directive_epoch = 1;
    ref.reference_kind = GUIDANCE_REFERENCE_TRAJECTORY;
    ref.point_count = 24;
    ref.anchor.valid_mask = ref.anchor.point.valid_mask = 31;
    for (unsigned i = 0; i < 24; ++i) {
      ref.points[i].offset_us = (i + 1) * 20000;
      ref.points[i].valid_mask = ref.points[i].point.valid_mask = 31;
    }
    guidance.header = header(SOURCE_M5);
    guidance.state_epoch = guidance.origin_id = guidance.reference_id =
        guidance.reference_epoch = 1;
    guidance.authority_epoch = guidance.directive_epoch = 1;
    guidance.reference_kind = GUIDANCE_REFERENCE_TRAJECTORY;
    guidance.guidance_state = GUIDANCE_STATE_READY;
    guidance.reference_expires_at_us = clock_value.monotonic_us + 40000;
    directive.header = header(SOURCE_M4);
    directive.operation_code = PRODUCT_OP_HOLD;
    directive.guidance_mode_code = PRODUCT_GUIDANCE_MODE_POS_HOLD;
    directive.state_epoch = directive.directive_epoch =
        directive.authority_epoch = directive.safety_envelope_id = 1;
    directive.directive_expires_at_us = clock_value.monotonic_us + 50000;
    envelope.header = header(SOURCE_M4);
    envelope.safety_envelope_id = envelope.envelope_epoch = 1;
    auto &e = envelope.vehicle_extension;
    e.valid_mask = 63;
    e.max_tilt_rad = .7f;
    e.max_total_thrust_n = 60;
    for (int i = 0; i < 3; ++i) {
      e.max_velocity_ned_mps[i] = 10;
      e.max_acceleration_ned_mps2[i] = 4;
      e.max_jerk_ned_mps3[i] = 20;
      e.max_body_rate_frd_radps[i] = 5;
    }
    authority.header = header(SOURCE_M4);
    authority.authority_epoch = 1;
    permit.header = header(SOURCE_M4);
    permit.permit_epoch = permit.state_epoch = permit.authority_epoch =
        permit.directive_epoch = permit.reference_epoch = 1;
    permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
    permit.permit_expires_at_us = clock_value.monotonic_us + 25000;
    actuator.header = header(SOURCE_M7);
    actuator.available_mask = 15;
    actuator.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
    capability.header = header(SOURCE_M7);
    capability.available_mask = 15;
    capability.actuator_layout_id = PRODUCT_ACTUATOR_LAYOUT_ID;
    for (int i = 0; i < 4; ++i) {
      capability.vehicle_extension.max_thrust_n[i] = 15;
      capability.vehicle_extension.max_slew_nps[i] = 100000;
    }
    OperatingPoint op{};
    op.operating_voltage_v = 24;
    CHECK(control_model_get_view(&op, .02f, &envelope, &model) == STATUS_OK);
    in.navigation = &nav;
    in.reference = &ref;
    in.pilot_control_reference = &pilot;
    in.guidance_status = &guidance;
    in.directive = &directive;
    in.safety_envelope = &envelope;
    in.authority_status = &authority;
    in.actuation_permit = &permit;
    in.actuator_status = &actuator;
    in.actuator_capability = &capability;
    in.control_model_view = &model;
    in.control_time_us = in.evaluation_time_us = clock_value.monotonic_us;
  }
  void tick() {
    clock_value.monotonic_us += 1000;
    nav.header.timestamp_sample_us = clock_value.monotonic_us;
    nav.header.timestamp_publish_us = clock_value.monotonic_us;
    in.control_time_us = in.evaluation_time_us = clock_value.monotonic_us;
  }
  void manual(uint16_t mode) {
    directive.operation_code = PRODUCT_OP_PILOT;
    directive.guidance_mode_code = mode;
    guidance.reference_kind = mode == PRODUCT_GUIDANCE_MODE_RATE
                                  ? GUIDANCE_REFERENCE_PILOT_RATE
                              : mode == PRODUCT_GUIDANCE_MODE_ATTITUDE
                                  ? GUIDANCE_REFERENCE_PILOT_ATTITUDE
                                  : GUIDANCE_REFERENCE_ALTITUDE_PILOT;
    pilot.header = header(SOURCE_M5);
    pilot.reference_id = pilot.reference_epoch = pilot.state_epoch =
        pilot.origin_id = pilot.authority_epoch = pilot.directive_epoch = 1;
    pilot.reference_kind = mode == PRODUCT_GUIDANCE_MODE_RATE
                               ? PILOT_REFERENCE_RATE
                           : mode == PRODUCT_GUIDANCE_MODE_ATTITUDE
                               ? PILOT_REFERENCE_ATTITUDE
                               : PILOT_REFERENCE_ALTITUDE_ATTITUDE;
    pilot.vehicle_extension.valid_mask = 63;
    pilot.vehicle_extension.collective_throttle_norm = .5f;
    pilot.vehicle_extension.q_nb_desired_wxyz[0] = 1;
  }
};
static void init_control() {
  ProductControllerConfig config{};
  control_sim_config(&config);
  CHECK(controller_init(&config) == STATUS_OK);
}
static void test_control_guards() {
  init_control();
  Fixture f;
  ControllerOutput out{};
  CHECK(controller_update(&f.in, &out) == STATUS_OK);
  CHECK(out.actuator_demand.valid_channel_mask == 15);
  CHECK(out.health.common.health_state ==
        MODULE_HEALTH_DEGRADED); // PD immediately, no MPC wait
  NEAR(out.actuator_demand.demand.motor_thrust_n[0], 1.5f * 9.80665f / 4,
       1e-4f);
  CHECK(out.actuator_demand.header.timestamp_sample_us == f.in.control_time_us);
  CHECK(out.actuator_demand.demand_expires_at_us <=
        f.permit.permit_expires_at_us);
  f.tick();
  f.permit.permit_flags |= PERMIT_FLAG_REVOKED;
  CHECK(controller_update(&f.in, &out) == STATUS_NOT_PERMITTED);
  CHECK(out.actuator_demand.valid_channel_mask == 0);
  f.permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
  f.ref.origin_id = 2;
  CHECK(controller_update(&f.in, &out) == STATUS_NOT_PERMITTED);
  f.ref.origin_id = 1;
  f.nav.header.event_time_uncertainty_us = 101;
  CHECK(controller_update(&f.in, &out) != STATUS_OK);
  f.nav.header.event_time_uncertainty_us = 10;
  f.in.control_time_us--;
  CHECK(controller_update(&f.in, &out) == STATUS_STALE_DATA);
  f.in.control_time_us++;
  f.nav.core.q_nb_wxyz[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(controller_update(&f.in, &out) != STATUS_OK);
  f.nav.core.q_nb_wxyz[0] = 1;
  f.actuator.available_mask = 7;
  CHECK(controller_update(&f.in, &out) != STATUS_OK);
  f.actuator.available_mask = 15;
  f.envelope.vehicle_extension.max_total_thrust_n=20;
  CHECK(controller_update(&f.in,&out)==STATUS_MODEL_OUT_OF_DOMAIN);
  CHECK(out.capability.reason_code==CONTROL_REASON_MPC_HOVER_OUTSIDE_APPROX_DOMAIN);
  f.envelope.vehicle_extension.max_total_thrust_n=60;
  f.permit.permit_expires_at_us = f.in.evaluation_time_us;
  CHECK(controller_update(&f.in, &out) == STATUS_NOT_PERMITTED);
  CHECK(controller_stop() == STATUS_OK);
  CHECK(controller_update(&f.in, &out) == STATUS_NOT_READY);
}
static void test_rearm_clears_motor_history() {
  init_control();
  Fixture f;
  ControllerOutput out{};
  CHECK(controller_update(&f.in, &out) == STATUS_OK);
  f.tick();
  f.permit.permit_flags = PERMIT_SCOPE_IDLE_ONLY;
  CHECK(controller_update(&f.in, &out) == STATUS_NOT_PERMITTED);

  f.tick();
  ++f.permit.permit_epoch;
  f.permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
  for (unsigned motor = 0; motor < 4; ++motor) {
    f.capability.vehicle_extension.max_thrust_n[motor] = 3.5f;
    f.capability.vehicle_extension.max_slew_nps[motor] = 1.0f;
  }
  CHECK(controller_update(&f.in, &out) == STATUS_OK);
  NEAR(out.actuator_demand.demand.motor_thrust_n[0], 3.5f, 1e-4f);
}
static void test_mpc_and_modes() {
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture f;
  ControllerOutput out{};
  CHECK(control_outer_update(&f.in) == STATUS_OK);
  CHECK(controller_update(&f.in, &out) == STATUS_OK);
  CHECK(out.health.solver_status == CONTROL_SOLVER_SOLVED);
  CHECK(out.health.common.health_state == MODULE_HEALTH_HEALTHY);
  CHECK(control_outer_update(&f.in) == STATUS_NOT_READY);
  f.tick();
  f.envelope.envelope_epoch++;
  CHECK(controller_update(&f.in, &out) == STATUS_OK);
  CHECK(out.health.common.health_state == MODULE_HEALTH_DEGRADED);
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture manual;
  manual.manual(PRODUCT_GUIDANCE_MODE_RATE);
  manual.nav.core.valid_mask = NAV_VALID_ATTITUDE | NAV_VALID_BODY_RATE;
  manual.pilot.vehicle_extension.body_rate_frd_radps[0] = .5f;
  CHECK(controller_update(&manual.in, &out) == STATUS_OK);
  CHECK(out.actuator_demand.demand.motor_thrust_n[0] >
        out.actuator_demand.demand.motor_thrust_n[1]);
  manual.tick();
  manual.manual(PRODUCT_GUIDANCE_MODE_ALT_HOLD);
  manual.nav.core.valid_mask |=
      NAV_VALID_VERTICAL_POSITION | NAV_VALID_VERTICAL_VELOCITY;
  CHECK(controller_update(&manual.in, &out) ==
        STATUS_OK); // no horizontal navigation required
  manual.tick();
  manual.nav.core.valid_mask &= ~NAV_VALID_VERTICAL_POSITION;
  CHECK(controller_update(&manual.in, &out) == STATUS_NOT_READY);
  manual.tick();
  manual.nav.core.valid_mask |= NAV_VALID_VERTICAL_POSITION;
  manual.directive.operation_code=PRODUCT_OP_LAND;
  manual.guidance.reference_kind=GUIDANCE_REFERENCE_ALTITUDE_AUTOMATIC;
  manual.pilot.vehicle_extension.valid_mask=PRODUCT_PILOT_REF_VALID_ATTITUDE |
      PRODUCT_PILOT_REF_VALID_ALTITUDE_UP | PRODUCT_PILOT_REF_VALID_VERTICAL_UP;
  CHECK(controller_update(&manual.in,&out)==STATUS_OK);
  manual.tick();
  manual.directive.operation_code=PRODUCT_OP_PILOT;
  CHECK(controller_update(&manual.in,&out)==STATUS_NOT_PERMITTED);
  // Known small position step: verify QP output reduces the predicted cost and
  // respects jerk.
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture step;
  step.ref.anchor.point.position_ned_m[0] = .05f;
  for (auto &point : step.ref.points)
    point.point.position_ned_m[0] = .05f;
  Status s = control_outer_update(&step.in);
  CHECK(s == STATUS_OK);
  CHECK(controller_update(&step.in, &out) == STATUS_OK);
  CHECK(m6.previous_accel[0] > 0);
  CHECK(m6.previous_accel[0] <= .02001f); // 1 ms jerk, not the MPC's 20 ms
  // Simulate a late solver via clock hook; expired output never becomes active.
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture deadline;
  clock_step = 3000;
  CHECK(control_outer_update(&deadline.in) == STATUS_DEADLINE_MISSED);
  clock_step = 0;
}
static void check_mpc_snapshot_feasible(const Fixture &f) {
  MpcSnapshot snapshot{};
  CHECK(mpc_read(&snapshot));
  CHECK(snapshot.header.boot_id == clock_value.boot_id);
  CHECK(snapshot.solver_status == CONTROL_SOLVER_SOLVED ||
        snapshot.solver_status == CONTROL_SOLVER_SOLVED_INACCURATE);
  CHECK(quad_solver.solution != nullptr);
  if (!quad_solver.solution || !quad_solver.solution->x)
    return;

  const auto &e = f.envelope.vehicle_extension;
  const float tangent = std::tan(e.max_tilt_rad) * .92387953251f;
  float previous[3] = {0, 0, 0}; // mpc_reset() clears the first 50 Hz seed.
  for (int move = 0; move < 5; ++move) {
    float acceleration[3];
    for (int axis = 0; axis < 3; ++axis) {
      const float value =
          static_cast<float>(quad_solver.solution->x[move * 3 + axis]);
      acceleration[axis] = value;
      CHECK(std::isfinite(value));
      CHECK(value >= f.model.bounds.input_min[axis] - 1e-3f);
      CHECK(value <= f.model.bounds.input_max[axis] + 1e-3f);
      CHECK(std::fabs(value - previous[axis]) <=
            f.model.bounds.input_slew_max[axis] * .02f + 1e-3f);
    }
    for (int side = 0; side < 8; ++side) {
      const float nx = std::cos(side * .785398163397f);
      const float ny = std::sin(side * .785398163397f);
      CHECK(nx * acceleration[0] + ny * acceleration[1] <=
            tangent * (CONTROL_GRAVITY_MPS2 - acceleration[2]) + 1e-3f);
    }
    const float thrust = m6.config.payload.mass_kg *
        std::sqrt(acceleration[0] * acceleration[0] +
                  acceleration[1] * acceleration[1] +
                  (CONTROL_GRAVITY_MPS2 - acceleration[2]) *
                  (CONTROL_GRAVITY_MPS2 - acceleration[2]));
    CHECK(thrust >= e.min_total_thrust_n - 1e-3f);
    CHECK(thrust <= e.max_total_thrust_n + 1e-3f);
    for (int axis = 0; axis < 3; ++axis)
      previous[axis] = acceleration[axis];
  }
  for (int axis = 0; axis < 3; ++axis)
    CHECK(std::fabs(snapshot.acceleration[axis] -
                    static_cast<float>(quad_solver.solution->x[axis])) <= 1e-4f);
}

static void test_mpc_axis_and_failure_regressions() {
  for (int axis = 0; axis < 3; ++axis) {
    clock_value.monotonic_us += 100000;
    init_control();
    Fixture f;
    const float target = axis == 1 ? -0.1f : 0.1f;
    f.ref.anchor.point.position_ned_m[axis] = target;
    for (auto &point : f.ref.points)
      point.point.position_ned_m[axis] = target;
    CHECK(control_outer_update(&f.in) == STATUS_OK);
    check_mpc_snapshot_feasible(f);
    MpcSnapshot snapshot{};
    CHECK(mpc_read(&snapshot));
    CHECK(snapshot.acceleration[axis] * target > 0.0f);
  }

  /* The approximate thrust polytope must admit hover at its declared edge. */
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture edge;
  edge.envelope.vehicle_extension.max_total_thrust_n =
      1.73205080757f * m6.config.payload.mass_kg * CONTROL_GRAVITY_MPS2 + .01f;
  CHECK(control_outer_update(&edge.in) == STATUS_OK);
  check_mpc_snapshot_feasible(edge);

  /* Below that approximation boundary, reject before entering OSQP. */
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture low_thrust;
  low_thrust.envelope.vehicle_extension.max_total_thrust_n = 20.0f;
  CHECK(control_outer_update(&low_thrust.in) == STATUS_MODEL_OUT_OF_DOMAIN);
  MpcSnapshot failed{};
  CHECK(mpc_read(&failed));
  CHECK(failed.expires_at_us == 0);
  CHECK(failed.solver_status == CONTROL_SOLVER_NUMERICAL_FAILURE);

  /* A non-finite point anywhere in the prediction horizon must not publish
     an executable solution. */
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture invalid_reference;
  invalid_reference.ref.points[10].point.position_ned_m[0] =
      std::numeric_limits<float>::quiet_NaN();
  CHECK(control_outer_update(&invalid_reference.in) == STATUS_NUMERIC_ERROR);
    CHECK(mpc_read(&failed));
  CHECK(failed.expires_at_us == 0);
  CHECK(failed.solver_status == CONTROL_SOLVER_NUMERICAL_FAILURE);
}

static void test_mpc_multiaxis_reference_regressions() {
  const float position_targets[3][3] = {
      {0.05f, -0.04f, 0.03f},
      {-0.04f, 0.05f, -0.03f},
      {0.03f, 0.03f, 0.03f},
  };
  for (const auto &targets : position_targets) {
    clock_value.monotonic_us += 100000;
    init_control();
    Fixture f;
    for (int axis = 0; axis < 3; ++axis) {
      f.ref.anchor.point.position_ned_m[axis] = targets[axis];
      for (auto &point : f.ref.points)
        point.point.position_ned_m[axis] = targets[axis];
    }
    const Status status = control_outer_update(&f.in);
    if (status != STATUS_OK)
      std::printf("MPC multi-axis position status=%u\n",
                  static_cast<unsigned>(status));
    CHECK(status == STATUS_OK);
    check_mpc_snapshot_feasible(f);
    MpcSnapshot snapshot{};
    CHECK(mpc_read(&snapshot));
    for (int axis = 0; axis < 3; ++axis)
      CHECK(snapshot.acceleration[axis] * targets[axis] > 0.0f);
  }

  /* Constant-velocity references exercise the velocity part of the objective
     independently of a position step. The reference position is integrated
     consistently so the test does not ask for a physically contradictory path. */
  const float velocity_targets[2][3] = {
      {0.10f, -0.08f, 0.06f},
      {-0.08f, 0.10f, -0.06f},
  };
  for (const auto &targets : velocity_targets) {
    clock_value.monotonic_us += 100000;
    init_control();
    Fixture f;
    for (int axis = 0; axis < 3; ++axis) {
      f.ref.anchor.point.velocity_ned_mps[axis] = targets[axis];
      for (unsigned i = 0; i < f.ref.point_count; ++i) {
        f.ref.points[i].point.velocity_ned_mps[axis] = targets[axis];
        f.ref.points[i].point.position_ned_m[axis] =
            targets[axis] * static_cast<float>(i + 1) * 0.02f;
      }
    }
    const Status status = control_outer_update(&f.in);
    if (status != STATUS_OK)
      std::printf("MPC multi-axis velocity status=%u\n",
                  static_cast<unsigned>(status));
    CHECK(status == STATUS_OK);
    check_mpc_snapshot_feasible(f);
    MpcSnapshot snapshot{};
    CHECK(mpc_read(&snapshot));
    for (int axis = 0; axis < 3; ++axis)
      CHECK(snapshot.acceleration[axis] * targets[axis] > 0.0f);
  }
}

static void test_mpc_repeated_workspace_update() {
  clock_value.monotonic_us += 100000;
  init_control();
  Fixture f;
  f.ref.anchor.point.position_ned_m[0] = .005f;
  for (auto &point : f.ref.points)
    point.point.position_ned_m[0] = .005f;
  CHECK(control_outer_update(&f.in) == STATUS_OK);
  MpcSnapshot first{};
  CHECK(mpc_read(&first));
  CHECK(first.solver_status == CONTROL_SOLVER_SOLVED ||
        first.solver_status == CONTROL_SOLVER_SOLVED_INACCURATE);

  /* Keep the same context/epoch but advance the trajectory sample. This
     exercises repeated OSQP matrix/vector updates without a cold-start reset. */
  clock_value.monotonic_us += 20000;
  f.nav.header.timestamp_sample_us = clock_value.monotonic_us;
  f.nav.header.timestamp_publish_us = clock_value.monotonic_us;
  f.in.control_time_us = f.in.evaluation_time_us = clock_value.monotonic_us;
  f.ref.header.timestamp_sample_us = clock_value.monotonic_us;
  f.ref.header.timestamp_publish_us = clock_value.monotonic_us;
  f.ref.anchor.point.position_ned_m[0] = .25f;
  for (auto &point : f.ref.points)
    point.point.position_ned_m[0] = .25f;
  CHECK(control_outer_update(&f.in) == STATUS_OK);

  MpcSnapshot second{};
  CHECK(mpc_read(&second));
  CHECK(second.header.sequence > first.header.sequence);
  CHECK(second.solver_status == CONTROL_SOLVER_SOLVED ||
        second.solver_status == CONTROL_SOLVER_SOLVED_INACCURATE);
  CHECK(std::fabs(second.acceleration[0] - first.acceleration[0]) > 1e-4f);
  check_mpc_snapshot_feasible(f);
}

static void test_math_allocator() {
  init_control();
  const float pi = 3.14159265358979f;
  for (float degrees : {179.9f, 180.f, 180.1f, -179.9f, -180.1f}) {
    float angle = degrees * pi / 180,
          q[4] = {std::cos(angle / 2), 0, 0, std::sin(angle / 2)}, r[9], v[3],
          axis[3] = {0, 0, 1};
    CHECK(attitude_quat_matrix(q, r));
    CHECK(attitude_so3_log(r, axis, v));
    NEAR(std::fabs(v[2]), std::min(std::fabs(angle), 2 * pi - std::fabs(angle)),
         .001f);
    q[0] = -q[0];
    q[3] = -q[3];
    float neg[9];
    CHECK(attitude_quat_matrix(q, neg));
    for (int j = 0; j < 9; ++j)
      NEAR(r[j], neg[j], 1e-6f);
  }
  {
    float inv[4][4], lo[4] = {0, 0, 0, 0}, hi[4] = {8, 8, 8, 8};
    CHECK(control_inverse4(m6.effectiveness, inv));
    for (int k = 0; k < 100; ++k) {
      float w[4] = {float(5 + k % 30), float((k % 7 - 3) * .8),
                    float((k % 9 - 4) * .7), float((k % 5 - 2) * .4)};
      ControlAllocation a;
      CHECK(control_allocate(inv, m6.effectiveness, w, lo, hi, &a) == STATUS_OK);
      for (int j = 0; j < 4; ++j)
        CHECK(a.motor[j] >= -1e-5f && a.motor[j] <= 8.00001f);
      NEAR(a.achieved[1], w[1] * a.roll_pitch_scale, 1e-4f);
      NEAR(a.achieved[2], w[2] * a.roll_pitch_scale, 1e-4f);
      CHECK(a.roll_pitch_scale >= 0 && a.roll_pitch_scale <= 1.00001f);
    }
    float w[4] = {16, 1, 1, 100};
    ControlAllocation a;
    CHECK(control_allocate(inv, m6.effectiveness, w, lo, hi, &a) == STATUS_OK);
    NEAR(a.roll_pitch_scale, 1, 1e-5f);
    NEAR(a.achieved[0], 16, 1e-4f);
    CHECK(a.yaw_scale < .01f);
  }
  {
    /* FMT-SIM takeoff replay. The old command is feasible, but the requested
     * roll/pitch ray through zero never enters the current motor slew box. */
    const float previous[4] = {3.65310526f, 3.60500765f, 4.01713562f, 4.05764103f};
    const float w[4] = {15.3459091f, .240297616f, -.244539827f, .0215905942f};
    float lo[4], hi[4];
    ControlAllocation a{};
    for (unsigned i = 0; i < 4; ++i) {
      lo[i] = previous[i] - .1f;
      hi[i] = previous[i] + .1f;
    }
    CHECK(control_allocate_limited(m6.inverse, m6.effectiveness, w, lo, hi, 4, 30, &a) ==
          STATUS_OUT_OF_RANGE);
    CHECK(control_allocate_from_previous(m6.inverse, m6.effectiveness, w, previous,
                                         lo, hi, 4, 30, &a) == STATUS_OK);
    CHECK(a.roll_pitch_scale > 0 && a.roll_pitch_scale < 1);
    CHECK(a.saturated_mask != 0);
    for (unsigned axis = 0; axis < 4; ++axis) {
      CHECK(a.motor[axis] >= lo[axis] - 1e-6f && a.motor[axis] <= hi[axis] + 1e-6f);
      float achieved = 0;
      for (unsigned motor = 0; motor < 4; ++motor)
        achieved += m6.effectiveness[axis][motor] * a.motor[motor];
      NEAR(a.residual[axis], achieved - w[axis], 2e-6f);
    }
    CHECK(control_allocate_from_previous(m6.inverse, m6.effectiveness, w, previous,
                                         lo, hi, 4, 15, &a) == STATUS_OUT_OF_RANGE);
  }
  {
    Fixture f;
    ProductMotionPoint point;
    f.ref.anchor.point.velocity_ned_mps[0] = 1;
    for (auto &p : f.ref.points) {
      p.point.position_ned_m[0] = p.offset_us * 1e-6f;
      p.point.velocity_ned_mps[0] = 1;
    }
    CHECK(control_sample_reference(&f.ref, clock_value.monotonic_us + 12345, &point) ==
          STATUS_OK);
    NEAR(point.position_ned_m[0], .012345f, 1e-6f);
    NEAR(point.velocity_ned_mps[0], 1, 1e-5f);
    CHECK(control_sample_reference(&f.ref, clock_value.monotonic_us + 480001, &point) ==
          STATUS_STALE_DATA);
  }
}
static void policy(unsigned id) {
  SensorUsePolicy p{};
  p.header = header(SOURCE_HEALTH);
  p.sensor_instance_id = id;
  p.policy_epoch = 1;
  p.policy_expires_at_us = clock_value.monotonic_us + 10000000;
  p.effective_time_us = clock_value.monotonic_us;
  CHECK(estimator_set_sensor_policy(&p, clock_value.monotonic_us) == STATUS_OK);
}
static ImuSample imu(unsigned sequence) {
  ImuSample s{};
  s.header = header(SOURCE_IMU, FRAME_BODY_FRD);
  s.sensor_instance_id = 10;
  s.sample_sequence = sequence;
  s.calibration_version = 1;
  s.quality_flags = IMU_Q_GYRO_VALID | IMU_Q_ACCEL_VALID | IMU_Q_CALIBRATED;
  s.accel_frd_mps2[2] = -static_cast<float>(
      kfg::gravity(kfg::vec3(30 * kfg::kD2R, 114 * kfg::kD2R, 30)));
  return s;
}
static SensorMeasurement mag(unsigned seq, uint64_t time = 0) {
  SensorMeasurement m{};
  m.measurement_kind = PRODUCT_MEASUREMENT_MAG;
  m.sensor_instance_id = 13;
  m.header = header(SOURCE_MAG, FRAME_BODY_FRD, time);
  m.header.sequence = seq;
  auto &s = m.payload.mag;
  s.header = m.header;
  s.sensor_instance_id = 13;
  s.calibration_version = 1;
  s.quality_flags = 3;
  s.field_frd_ut[0] = 25;
  s.field_frd_ut[2] = 40;
  for (float &v : s.variance_ut2)
    v = .01f;
  return m;
}
static SensorMeasurement gnss(unsigned seq, uint64_t time = 0) {
  SensorMeasurement m{};
  m.measurement_kind = PRODUCT_MEASUREMENT_GNSS;
  m.sensor_instance_id = 11;
  m.header = header(SOURCE_GNSS, FRAME_WGS84_GEODETIC, time);
  m.header.sequence = seq;
  auto &s = m.payload.gnss;
  s.header = m.header;
  s.sensor_instance_id = 11;
  s.calibration_version = 1;
  s.quality_flags = 15;
  s.sample_sequence = seq;
  s.fix_type = GNSS_FIX_RTK_FIXED;
  s.latitude_deg = 30;
  s.longitude_deg = 114;
  s.altitude_m = 30;
  for (int j = 0; j < 3; ++j) {
    s.position_variance_m2[j] = .04f;
    s.velocity_variance_m2ps2[j] = .01f;
  }
  return m;
}
static SensorMeasurement baro(unsigned seq, uint64_t time = 0) {
  SensorMeasurement m{};
  m.measurement_kind = PRODUCT_MEASUREMENT_BARO;
  m.sensor_instance_id = 12;
  m.header = header(SOURCE_BARO, FRAME_LOCAL_NED, time);
  m.header.sequence = seq;
  auto &s = m.payload.baro;
  s.header = m.header;
  s.sensor_instance_id = 12;
  s.calibration_version = 1;
  s.quality_flags = 7;
  s.altitude_m = 100;
  s.variance_m2 = .04f;
  return m;
}
static void test_navigation() {
  clock_value.monotonic_us = 2000000;
  ProductEstimatorConfig config{};
  ins_sim_config(&config);
  CHECK(estimator_init(&config) == STATUS_OK);
  policy(10);
  policy(11);
  policy(12);
  policy(13);
  for (unsigned i = 1; i <= 1000; ++i) {
    clock_value.monotonic_us += 1000;
    auto sample = imu(i);
    const auto result = estimator_push_imu(&sample);
    CHECK(result == (i < 1000 ? STATUS_NOT_READY : STATUS_OK));
    if (i == 990) {
      auto g = gnss(i), b = baro(i), m = mag(i);
      CHECK(estimator_push_measurement(&g) == STATUS_OK);
      CHECK(estimator_push_measurement(&b) == STATUS_OK);
      CHECK(estimator_push_measurement(&m) == STATUS_OK);
    }
  }
  NavigationSnapshot snapshot{};
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_OK);
  CHECK((snapshot.core.valid_mask & 127) == 127);
  NEAR(snapshot.core.position_ned_m[2], 0, 1e-4f);
  NEAR(snapshot.core.q_nb_wxyz[0], 1, 1e-5f);
  for (unsigned i = 1001; i <= 1260; ++i) {
    clock_value.monotonic_us += 1000;
    auto sample = imu(i);
    CHECK(estimator_push_imu(&sample) == STATUS_OK);
    if (i % 20 == 0) {
      auto g = gnss(i, clock_value.monotonic_us - 5250),
           b = baro(i, clock_value.monotonic_us - 5250),
           m = mag(i, clock_value.monotonic_us - 5250);
      CHECK(estimator_push_measurement(&g) == STATUS_OK);
      CHECK(estimator_push_measurement(&b) == STATUS_OK);
      CHECK(estimator_push_measurement(&m) == STATUS_OK);
      CHECK(estimator_update(clock_value.monotonic_us) == STATUS_OK);
      CHECK(estimator_push_measurement(&g) == STATUS_STALE_DATA);
    }
  }
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_OK);
  CHECK((snapshot.core.valid_mask & 127) == 127);
  NEAR(snapshot.core.position_ned_m[0], 0, .01f);
  NEAR(snapshot.core.position_ned_m[2], 0, .01f);
  auto position_only = gnss(1261, clock_value.monotonic_us - 4000);
  position_only.payload.gnss.quality_flags =
      GNSS_Q_POSITION_VALID | GNSS_Q_POSITION_COV_VALID;
  position_only.payload.gnss.velocity_ned_mps[0] =
      std::numeric_limits<float>::quiet_NaN();
  CHECK(estimator_push_measurement(&position_only) == STATUS_OK);
  CHECK(estimator_update(clock_value.monotonic_us) == STATUS_OK);
  auto velocity_only = gnss(1262, clock_value.monotonic_us - 3000);
  velocity_only.payload.gnss.quality_flags =
      GNSS_Q_VELOCITY_VALID | GNSS_Q_VELOCITY_COV_VALID;
  velocity_only.payload.gnss.latitude_deg =
      std::numeric_limits<double>::quiet_NaN();
  CHECK(estimator_push_measurement(&velocity_only) == STATUS_OK);
  CHECK(estimator_update(clock_value.monotonic_us) == STATUS_OK);
  auto outlier = gnss(1263, clock_value.monotonic_us - 2000);
  outlier.payload.gnss.latitude_deg += 1;
  CHECK(estimator_push_measurement(&outlier) == STATUS_OK);
  CHECK(estimator_update(clock_value.monotonic_us) == STATUS_OK);
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_OK);
  NEAR(snapshot.core.position_ned_m[0], 0, .01f);
  EstimatorResetRequest provided{};
  provided.header = header(SOURCE_M4);
  provided.reset_type = RESET_TYPE_INITIALIZE;
  provided.initial_source = RESET_SOURCE_PROVIDED_STATE;
  provided.valid_initial_state_mask = snapshot.core.valid_mask;
  provided.initial_state = snapshot.core;
  provided.initial_state.position_ned_m[0] = 1;
  CHECK(estimator_reset(&provided) == STATUS_OK);
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_OK);
  NEAR(snapshot.core.position_ned_m[0], 1, 1e-5f);
  const auto last = snapshot;
  clock_value.monotonic_us += 10000;
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_STALE_DATA);
  CHECK(snapshot.core.valid_mask == 0);
  NEAR(snapshot.core.position_ned_m[0], last.core.position_ned_m[0], 1e-7f);
  auto gap = imu(1270);
  CHECK(estimator_push_imu(&gap) == STATUS_STALE_DATA);
  EstimatorResetRequest reset{};
  reset.header = header(SOURCE_M4);
  reset.reset_type = RESET_TYPE_RECOVER_AFTER_GAP;
  reset.initial_source = RESET_SOURCE_STATIC_IMU_LOCAL;
  CHECK(estimator_reset(&reset) == STATUS_OK);
  StateResetEvent event{};
  CHECK(ins_take_reset_event(&event) == STATUS_OK);
  CHECK(event.new_state_epoch == event.old_state_epoch + 1);
  CHECK(ins_take_reset_event(&event) == STATUS_NOT_READY);
  CHECK(estimator_stop() == STATUS_OK);
  // An IMU-only start never advertises absolute heading or horizontal position.
  config.payload.aiding_mask = 0;
  clock_value.monotonic_us += 100000;
  CHECK(estimator_init(&config) == STATUS_OK);
  policy(10);
  for (unsigned i = 1; i <= 1000; ++i) {
    clock_value.monotonic_us += 1000;
    auto s = imu(i);
    estimator_push_imu(&s);
  }
  CHECK(estimator_get_snapshot(&snapshot) == STATUS_OK);
  CHECK((snapshot.core.valid_mask & NAV_VALID_ATTITUDE) != 0);
  CHECK((snapshot.core.valid_mask &
         (NAV_VALID_HEADING | NAV_VALID_HORIZONTAL_POSITION |
          NAV_VALID_HORIZONTAL_VELOCITY)) == 0);
  auto bad = config;
  bad.payload.initial_covariance_upper[0] = -1;
  CHECK(estimator_init(&bad) == STATUS_INVALID_ARGUMENT);
  CHECK(estimator_get_snapshot(&snapshot) ==
        STATUS_OK); // invalid reconfiguration is atomic
}
static void test_shadow_probation() {
  clock_value.monotonic_us=10000000;
  ProductEstimatorConfig config{};ins_sim_config(&config);
  CHECK(estimator_init(&config)==STATUS_OK);policy(10);policy(12);
  unsigned sequence=1;
  for(;sequence<=1040;++sequence) {
    clock_value.monotonic_us+=1000;auto s=imu(sequence);estimator_push_imu(&s);
    if(sequence==990) { auto b=baro(sequence);CHECK(estimator_push_measurement(&b)==STATUS_OK); }
  }
  SensorUsePolicy p{};p.header=header(SOURCE_HEALTH);p.sensor_instance_id=12;
  p.policy_epoch=2;p.use_state=SENSOR_USE_REJECT;p.evaluation_flags=SENSOR_USE_EVAL_ALLOW_SHADOW;
  p.effective_time_us=clock_value.monotonic_us;p.policy_expires_at_us=clock_value.monotonic_us+1000000;
  CHECK(estimator_set_sensor_policy(&p,clock_value.monotonic_us)==STATUS_OK);
  for(unsigned i=0;i<10;++i,++sequence) {
    clock_value.monotonic_us+=1000;auto s=imu(sequence);CHECK(estimator_push_imu(&s)==STATUS_OK);
  }
  NavigationSnapshot before{},after{};SensorConsistencyEvidence e{};
  CHECK(estimator_get_snapshot(&before)==STATUS_OK);
  auto b=baro(2000,clock_value.monotonic_us-1000);
  CHECK(estimator_push_measurement(&b)==STATUS_OK);
  CHECK(estimator_update(clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_OK);
  CHECK(e.evaluation_mode==2 && e.policy_epoch==2 && e.evaluated_count==1 && e.accepted_count==1);
  CHECK((e.evidence_flags&15u)==15u);
  CHECK(estimator_get_snapshot(&after)==STATUS_OK);
  CHECK(std::memcmp(&before.core,&after.core,sizeof(before.core))==0);
  const auto proof_sequence=e.header.sequence;
  p.header=header(SOURCE_HEALTH);
  CHECK(estimator_set_sensor_policy(&p,clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_OK);
  CHECK(e.header.sequence==proof_sequence); // renewing policy does not renew evidence
  CHECK(estimator_push_measurement(&b)==STATUS_STALE_DATA);
  clock_value.monotonic_us+=1000;auto sample=imu(sequence++);CHECK(estimator_push_imu(&sample)==STATUS_OK);
  CHECK(estimator_get_snapshot(&before)==STATUS_OK);
  b=baro(2001,clock_value.monotonic_us);b.payload.baro.altitude_m=1000;
  CHECK(estimator_push_measurement(&b)==STATUS_OK);CHECK(estimator_update(clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_OK);
  CHECK(e.rejected_count==1 && e.reason_code==SENSOR_CONSISTENCY_REASON_INCONSISTENT);
  CHECK(estimator_get_snapshot(&after)==STATUS_OK);
  CHECK(std::memcmp(&before.core,&after.core,sizeof(before.core))==0);
  test_mapping.source_clock_domain_id=100;test_mapping.mapping_epoch=1;
  test_mapping.local_time_epoch=clock_value.time_epoch;test_mapping.mapping_state=CLOCK_MAPPING_LOCKED;
  test_mapping.valid_until_local_us=clock_value.monotonic_us+1000000;test_mapping.uncertainty_ns=100;
  clock_value.monotonic_us+=1000;sample=imu(sequence++);CHECK(estimator_push_imu(&sample)==STATUS_OK);
  b=baro(2002,clock_value.monotonic_us);
  b.header.source_clock_domain_id=100;b.header.source_mapping_epoch=1;b.payload.baro.header=b.header;
  CHECK(estimator_push_measurement(&b)==STATUS_OK);
  test_mapping.mapping_epoch=2; /* invalidates an admitted, not yet evaluated probe */
  CHECK(estimator_update(clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_NOT_READY);
  clock_value.monotonic_us+=1000;sample=imu(sequence++);CHECK(estimator_push_imu(&sample)==STATUS_OK);
  b=baro(2003,clock_value.monotonic_us);
  b.header.source_clock_domain_id=100;b.header.source_mapping_epoch=2;b.payload.baro.header=b.header;
  CHECK(estimator_push_measurement(&b)==STATUS_OK);CHECK(estimator_update(clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_OK);CHECK(e.accepted_count==1);
  test_mapping.mapping_state=CLOCK_MAPPING_DEGRADED;
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_NOT_READY);
  test_mapping={};
  p.policy_epoch=3;p.evaluation_flags=0;
  CHECK(estimator_set_sensor_policy(&p,clock_value.monotonic_us)==STATUS_OK);
  CHECK(estimator_get_sensor_consistency(12,&e)==STATUS_NOT_READY);
  CHECK(estimator_push_measurement(&b)==STATUS_NOT_PERMITTED);
  CHECK(estimator_stop()==STATUS_OK);
}
int main() {
  test_math_allocator();
  test_control_guards();
  test_rearm_clears_motor_history();
  test_mpc_and_modes();
  test_mpc_axis_and_failure_regressions();
  test_mpc_multiaxis_reference_regressions();
  test_mpc_repeated_workspace_update();
  test_navigation();
  test_shadow_probation();
  std::printf("controller_navigation: %u checks, %u failures\n", checks,
              failures);
  return failures ? 1 : 0;
}
