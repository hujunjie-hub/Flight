/*
 * Host tests for M5 Guidance (QUAD_X4, NONE obstacle mode).
 *
 * Scope: reference construction, anchor+24 preview, dynamics/geofence checks,
 * READY/ACTIVE lifecycle, Host/SIL M5-02 map, M5-03 A*, M5-04 time scaling,
 * M5-05 minimum-jerk optimization, and M5-06 independent validator cases.
 * Host tests are not H723 deployment evidence or a production safety release.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <fenv.h>
#if (defined(__x86_64__) || defined(__i386__)) && defined(__SSE__)
#include <xmmintrin.h>
#endif
#include "ports/port_api.h"
#include "guidance_update/guidance.h"
#include "guidance_update/guidance_internal.h"
#include "trajectory_planning/m5_local_map.h"
#include "trajectory_planning/m5_kinodynamic_astar.h"
#include "trajectory_planning/m5_path_smoother.h"
#include "trajectory_planning/m5_time_scaler.h"
#include "trajectory_planning/m5_trajectory_safety.h"
#include "m5_trajectory_safety_test_api.h"
/* The product SafetyEnvelope source is linked into the Host/SIL suite so the
   M5-06 fixtures consume the real QUAD_X4 policy instead of a second table. */
#include "fms_contract/fms_model.h"
#include "flight_policy/fms_policy.h"
/* MR-009: link the real M6 reference sampler, not a Python re-implementation. */
#include "attitude_rate_control/controller_internal.h"

#define T_BOOT 0x1122334455667788ull
#define T_EPOCH 3u
#define T_ORIGIN 42u
#define T_STATE_EPOCH 7u
#define T_AUTH_EPOCH 5u
#define T_DIR_EPOCH 9u

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                            \
  do {                                                                         \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
      g_failures++;                                                            \
    }                                                                          \
  } while (0)

typedef struct {
  uint64_t now;
  NavigationSnapshot nav;
  VehicleDirective dir;
  SafetyEnvelope env;
  AuthorityStatus auth;
  AuthorizedMissionSnapshot mission;
  ActuationPermit permit;
  ProductGeofenceSnapshot fence;
  uint16_t fence_count;
  uint8_t has_mission;
  uint8_t has_permit;
  uint8_t has_fence;
  GuidanceOutput out;
} Fx;

static uint32_t g_seq;

static void fill_header(ContractHeader *h, uint16_t type, uint16_t source, uint64_t now)
{
  memset(h, 0, sizeof(*h));
  h->schema_major = CONTRACT_SCHEMA_MAJOR;
  h->schema_minor = CONTRACT_SCHEMA_MINOR;
  h->message_type = type;
  h->source_id = source;
  h->sequence = ++g_seq;
  h->profile_id = PRODUCT_PROFILE_ID;
  h->frame_id = FRAME_LOCAL_NED;
  h->boot_id = T_BOOT;
  h->time_epoch = T_EPOCH;
  h->clock_domain_id = CLOCK_DOMAIN_FCU_MONOTONIC;
  h->timestamp_sample_us = now;
  h->timestamp_publish_us = now;
  h->valid_until_us = now + 500000u;
  h->validity_flags = VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID;
}

static void fx_defaults(Fx *fx)
{
  memset(fx, 0, sizeof(*fx));
  fx->now = 2000000u;
  fx->nav.state_epoch = T_STATE_EPOCH;
  fx->nav.origin_id = T_ORIGIN;
  fx->nav.core.valid_mask = 0xFFu;
  fx->nav.core.q_nb_wxyz[0] = 1.0f;
  fx->auth = (AuthorityStatus){0};
  fx->dir.operation_code = PRODUCT_OP_HOLD;
  fx->dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_POS_HOLD;
  fx->dir.directive_epoch = T_DIR_EPOCH;
  fx->dir.authority_epoch = T_AUTH_EPOCH;
  fx->dir.state_epoch = T_STATE_EPOCH;
  fx->dir.policy_id = 2u;
  fx->dir.safety_envelope_id = 1u;
  fx->dir.payload.home.validity = PRODUCT_HOME_VALID;
  fx->dir.payload.home.frame_id = FRAME_LOCAL_NED;
  fx->dir.payload.home.origin_id = T_ORIGIN;
  fx->dir.payload.home.state_epoch = T_STATE_EPOCH;
  fx->dir.payload.home.home_expires_at_us = fx->now + 100000000u;
  fx->dir.payload.home.position_ned_m[0] = 0.0f;
  fx->dir.payload.home.position_ned_m[1] = 0.0f;
  fx->dir.payload.home.position_ned_m[2] = 0.0f;
  fx->auth.authority_epoch = T_AUTH_EPOCH;
  fx->auth.selected_source_id = 1u;
  fx->auth.authority_owner = 0u;
  fx->auth.operation_mode = 4u;

  fx->env.safety_envelope_id = 1u;
  fx->env.max_linear_speed_mps = 4.0f;
  fx->env.max_linear_accel_mps2 = 2.5f;
  fx->env.max_angular_rate_radps = 2.0f;
  fx->env.max_yaw_rate_radps = 2.0f;
  fx->env.vehicle_extension.valid_mask = 0x7Fu;
  fx->env.vehicle_extension.max_velocity_ned_mps[0] = 4.0f;
  fx->env.vehicle_extension.max_velocity_ned_mps[1] = 4.0f;
  fx->env.vehicle_extension.max_velocity_ned_mps[2] = 1.5f;
  fx->env.vehicle_extension.max_acceleration_ned_mps2[0] = 2.5f;
  fx->env.vehicle_extension.max_acceleration_ned_mps2[1] = 2.5f;
  fx->env.vehicle_extension.max_acceleration_ned_mps2[2] = 1.0f;
  /* Geometry fixtures are not jerk tests; the product jerk limit is
     exercised by m5_safety_jerk_envelope. */
  fx->env.vehicle_extension.max_jerk_ned_mps3[0] = 3.4028235e38f;
  fx->env.vehicle_extension.max_jerk_ned_mps3[1] = 3.4028235e38f;
  fx->env.vehicle_extension.max_jerk_ned_mps3[2] = 3.4028235e38f;
  fx->env.vehicle_extension.max_tilt_rad = 0.5f;
  fx->env.vehicle_extension.max_body_rate_frd_radps[0] = 2.0f;
  fx->env.vehicle_extension.max_body_rate_frd_radps[1] = 2.0f;
  fx->env.vehicle_extension.max_body_rate_frd_radps[2] = 2.0f;
  fx->env.vehicle_extension.max_total_thrust_n = 20.0f;
  fx->env.vehicle_extension.min_total_thrust_n = 4.0f;
  fx->env.vehicle_extension.max_height_above_home_m = 50.0f;
  fx->env.vehicle_extension.min_height_above_home_m = 0.3f;
}

static void fx_mission(Fx *fx, uint32_t epoch, uint64_t token, uint16_t index)
{
  ProductMissionPayload *m = &fx->mission.mission;
  memset(&fx->mission, 0, sizeof(fx->mission));
  fx->mission.mission_epoch = epoch;
  fx->mission.authority_epoch = T_AUTH_EPOCH;
  fx->mission.directive_epoch = T_DIR_EPOCH;
  fx->mission.state_epoch = T_STATE_EPOCH;
  fx->mission.origin_id = T_ORIGIN;
  fx->mission.mission_id = 100u;
  fx->mission.current_item_index = index;
  fx->mission.current_item_id = (uint32_t)(index + 1u);
  fx->mission.execution_token = token;
  m->mission_id = 100u;
  m->waypoint_count = 2u;
  m->waypoints[0].waypoint_id = 1u;
  m->waypoints[0].position_ned_m[0] = 3.0f;
  m->waypoints[0].position_ned_m[1] = 0.0f;
  m->waypoints[0].position_ned_m[2] = -1.0f;
  m->waypoints[0].acceptance_radius_m = 1.5f;
  m->waypoints[0].speed_limit_mps = 2.0f;
  m->waypoints[1].waypoint_id = 2u;
  m->waypoints[1].position_ned_m[0] = 4.0f;
  m->waypoints[1].position_ned_m[1] = 0.0f;
  m->waypoints[1].position_ned_m[2] = -1.0f;
  m->waypoints[1].acceptance_radius_m = 1.5f;
  m->waypoints[1].speed_limit_mps = 2.0f;
  fx->has_mission = 1u;
}

static void fx_fill(Fx *fx)
{
  fx->dir.directive_expires_at_us = fx->now + 500000u;
  fx->mission.mission_expires_at_us = fx->now + 500000u;
  if (fx->has_fence) {
    fill_header(&fx->fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT, SOURCE_M4, fx->now);
    fx->fence.origin_id = T_ORIGIN; fx->fence.state_epoch = T_STATE_EPOCH;
  }
  fill_header(&fx->nav.header, MSG_NAVIGATION_SNAPSHOT, SOURCE_M3, fx->now);
  fill_header(&fx->dir.header, MSG_VEHICLE_DIRECTIVE, SOURCE_M4, fx->now);
  fill_header(&fx->env.header, MSG_SAFETY_ENVELOPE, SOURCE_M4, fx->now);
  fill_header(&fx->auth.header, MSG_AUTHORITY_STATUS, SOURCE_M4, fx->now);
  if (fx->has_mission != 0u) {
    fill_header(&fx->mission.header, MSG_AUTHORIZED_MISSION_SNAPSHOT, SOURCE_M4,
                fx->now);
  }
  if (fx->has_permit != 0u) {
    fill_header(&fx->permit.header, MSG_ACTUATION_PERMIT, SOURCE_M4, fx->now);
  }
}

static void fx_input(Fx *fx, GuidanceInput *in)
{
  memset(in, 0, sizeof(*in));
  in->navigation = &fx->nav;
  in->directive = &fx->dir;
  in->safety_envelope = &fx->env;
  in->authority_status = &fx->auth;
  in->authorized_mission = fx->has_mission ? &fx->mission : NULL;
  in->active_permit = fx->has_permit ? &fx->permit : NULL;
  if (fx->has_fence != 0u) {
    in->geofences = &fx->fence;
    in->geofence_count = fx->fence_count;
  }
  in->control_time_us = fx->nav.header.timestamp_sample_us;
  in->evaluation_time_us = fx->now;
}

static Status fx_step(Fx *fx)
{
  GuidanceInput in;
  Status st;
  fx_fill(fx);
  fx_input(fx, &in);
  st = guidance_update(&in, &fx->out);
  fx->now += 10000u;
  return st;
}

static int fx_init(Fx *fx)
{
  ProductGuidanceConfig cfg;
  ProductProfile profile;
  fx_defaults(fx);
  guidance_default_config(&cfg);
  memset(&profile, 0, sizeof(profile));
  profile.product_profile_id = PRODUCT_PROFILE_ID;
  profile.guidance_impl_id = 0x1301u;
  if (guidance_validate(&profile) != STATUS_OK) {
    return 0;
  }
  return guidance_init(&cfg) == STATUS_OK;
}

static void fx_make_permit(Fx *fx, uint32_t ref_epoch)
{
  memset(&fx->permit, 0, sizeof(fx->permit));
  fx->permit.permit_epoch = 3u;
  fx->permit.reference_epoch = ref_epoch;
  fx->permit.authority_epoch = T_AUTH_EPOCH;
  fx->permit.directive_epoch = T_DIR_EPOCH;
  fx->permit.state_epoch = T_STATE_EPOCH;
  fx->permit.permit_flags = PERMIT_SCOPE_CONTROL_ACTIVE;
  fx->permit.permit_expires_at_us = fx->now + 25000u;
  fx->permit.output_policy_id = 2u;
  fx->has_permit = 1u;
}

/* ------------------------------------------------------------------ */
/* tests                                                              */
/* ------------------------------------------------------------------ */

static void test_landing_aligns_and_limits_final_descent(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_RETURN;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx.nav.core.position_ned_m[2] = -2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  const uint32_t epoch = fx.out.guidance_status.reference_epoch;
  fx.nav.core.position_ned_m[0] = 0.6f;
  fx.nav.core.position_ned_m[2] = -0.6f;
  for (unsigned i = 0; i < 630; ++i)
    CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible);
  CHECK(fabsf(fx.out.reference.points[23].point.position_ned_m[2] + 0.6f) < 0.001f);

  fx.nav.core.position_ned_m[0] = 0.0f;
  unsigned low_points = 0;
  for (unsigned i = 0; i < 450; ++i) {
    CHECK(fx_step(&fx) == STATUS_OK);
    CHECK(fx.out.feasibility.feasible && fx.out.guidance_status.reference_epoch == epoch);
    const ProductMotionPoint *point = &fx.out.reference.points[23].point;
    if (point->position_ned_m[2] >= -0.5f) {
      ++low_points;
      CHECK(point->velocity_ned_mps[2] <= 0.35001f);
    }
  }
  CHECK(low_points > 0);
  CHECK(fabsf(fx.out.reference.anchor.point.position_ned_m[2]) < 0.001f);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_trajectory_handoff_uses_active_reference_acceleration(void)
{
  for (unsigned reset = 0; reset < 2u; ++reset) {
    Fx fx;
    CHECK(fx_init(&fx));
    fx.nav.core.position_ned_m[2] = -2.0f;
    CHECK(fx_step(&fx) == STATUS_OK);
    const uint32_t previous_epoch = fx.out.guidance_status.reference_epoch;
    fx_make_permit(&fx, previous_epoch);
    CHECK(fx_step(&fx) == STATUS_OK);
    CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);

    fx.has_permit = 0u;
    ++fx.dir.directive_epoch;
    fx.nav.core.position_ned_m[0] = 0.1f;
    fx.nav.core.velocity_ned_mps[0] = 0.05f;
    fx.nav.core.acceleration_ned_mps2[2] = 2.0f;
    if (reset) fx.dir.state_epoch = ++fx.nav.state_epoch;
    CHECK(fx_step(&fx) == STATUS_OK);
    CHECK(fx.out.guidance_status.reference_epoch != previous_epoch);
    if (reset) {
      CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
      CHECK(!fx.out.feasibility.feasible);
    } else {
      CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
      CHECK(fx.out.feasibility.feasible);
      const ProductMotionPoint *anchor = &fx.out.reference.anchor.point;
      CHECK(fabsf(anchor->position_ned_m[0] - 0.1f) < 1e-5f);
      CHECK(fabsf(anchor->velocity_ned_mps[0] - 0.05f) < 1e-5f);
      CHECK(fabsf(anchor->acceleration_ned_mps2[2]) < 1e-5f);
    }
    CHECK(guidance_stop() == STATUS_OK);
  }
}

static void test_hold_reference_shape(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(fx.out.guidance_status.reference_kind == GUIDANCE_REFERENCE_TRAJECTORY);
  CHECK(fx.out.reference.point_count == 24u);
  CHECK(fx.out.reference.anchor.offset_us == 0u);
  CHECK(fx.out.reference.points[0].offset_us == 20000u);
  CHECK(fx.out.reference.points[23].offset_us == 480000u);
  CHECK(fx.out.reference.anchor.valid_mask ==
        (PRODUCT_POINT_VALID_POSITION | PRODUCT_POINT_VALID_VELOCITY |
         PRODUCT_POINT_VALID_ACCELERATION | PRODUCT_POINT_VALID_YAW |
         PRODUCT_POINT_VALID_YAW_RATE));
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fx.out.guidance_status.reference_expires_at_us > fx.now);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_mission_reference_and_token(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx.has_mission = 1u;
  fx_mission(&fx, 4u, 0x1234u, 0u);
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(fx.out.guidance_status.execution_token == 0x1234u);
  CHECK(fx.out.guidance_status.current_item_id == 1u);
  CHECK(fx.out.guidance_status.mission_epoch == 4u);
  CHECK(fx.out.reference.execution_token == 0x1234u);
  CHECK(fx.out.reference.mission_epoch == 4u);
  CHECK(fx.out.reference.points[23].point.position_ned_m[0] > 0.0f);
  CHECK(fx.out.reference.points[23].point.position_ned_m[0] <= 10.0f);
  CHECK(fx.out.reference.points[23].point.velocity_ned_mps[0] >= 0.0f);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_mission_uses_final_time_bin_before_rejecting(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  fx.mission.mission.waypoints[0].position_ned_m[0] = 3.25f;
  fx.mission.mission.waypoints[0].position_ned_m[2] = 0.0f;
  fx.mission.mission.waypoints[0].speed_limit_mps = 1.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(fx.out.reference.header.validity_flags != 0u);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_mission_leg_beyond_time_window_rejected(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  fx.mission.mission.waypoints[0].position_ned_m[0] = 18.0f;
  fx.mission.mission.waypoints[0].speed_limit_mps = 2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 0u);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.feasibility.violated_constraint_id == GUIDANCE_PLAN_CONSTRAINT_TIME);
  CHECK(fx.out.reference.header.validity_flags == 0u);
  CHECK(guidance_stop() == STATUS_OK);
}
static void test_return_leg_beyond_time_window_rejected(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_RETURN;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx.nav.core.position_ned_m[0] = 18.0f;
  fx.nav.core.position_ned_m[2] = -2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 0u);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.reference.header.validity_flags == 0u);
  CHECK(guidance_stop() == STATUS_OK);
}
static void test_hold_at_end_of_long_leg(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.nav.core.position_ned_m[0] = -18.0f;
  fx.nav.core.position_ned_m[2] = -2.0f;
  fx.nav.core.velocity_ned_mps[0] = -0.36f;
  fx.nav.core.acceleration_ned_mps2[0] = 0.60f;
  for (unsigned step = 0; step < 200u; ++step) {
    fx.now += 10000u;
    if (fx_step(&fx) != STATUS_OK || !fx.out.feasibility.feasible) break;
  }
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fabsf(fx.out.reference.anchor.point.velocity_ned_mps[0]) < 0.01f);
  CHECK(fx.out.reference.points[0].offset_us == PRODUCT_ROUTE_PREVIEW_STEP_US);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_reference_epoch_stable_and_rotates(void)
{
  Fx fx;
  uint32_t epoch;
  uint32_t id;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx.has_mission = 1u;
  fx_mission(&fx, 1u, 10u, 0u);
  (void)fx_step(&fx);
  epoch = fx.out.guidance_status.reference_epoch;
  id = fx.out.guidance_status.reference_id;
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.reference_epoch == epoch);
  CHECK(fx.out.guidance_status.reference_id == id);
  fx_mission(&fx, 2u, 11u, 1u); /* new item/token/epoch */
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.reference_epoch > epoch);
  CHECK(fx.out.guidance_status.reference_id > id);
  CHECK(fx.out.guidance_status.current_item_id == 2u);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_permit_matching_activates(void)
{
  Fx fx;
  uint32_t epoch;
  CHECK(fx_init(&fx));
  (void)fx_step(&fx);
  epoch = fx.out.guidance_status.reference_epoch;
  fx_make_permit(&fx, epoch);
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_ACTIVE);
  /* A stale/foreign permit epoch returns to READY without changing reference. */
  fx.permit.reference_epoch = epoch + 1u;
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(fx.out.guidance_status.reference_epoch == epoch);
  fx.has_permit = 0u;
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_avoidance_mission_rejected(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  fx.mission.mission.flags = PRODUCT_MISSION_REQUIRES_OBSTACLE_AVOIDANCE;
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.guidance_status.reason_code == GUIDANCE_PLAN_REASON_OBSTACLE_UNKNOWN);
  CHECK(fx.out.feasibility.feasible == 0u);
  CHECK(fx.out.reference.header.validity_flags == 0u);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_geofence_violation_rejected(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  memset(&fx.fence, 0, sizeof(fx.fence));
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.vertex_count = 3u;
  fx.fence.vertices_ne_m[0][0] = -2.0f;
  fx.fence.vertices_ne_m[0][1] = -2.0f;
  fx.fence.vertices_ne_m[1][0] = 2.0f;
  fx.fence.vertices_ne_m[1][1] = -2.0f;
  fx.fence.vertices_ne_m[2][0] = 0.0f;
  fx.fence.vertices_ne_m[2][1] = 2.0f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 20.0f;
  fx.fence.horizontal_margin_m = 0.0f;
  fx.has_fence = 1u;
  fx.fence_count = 1u;
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.feasibility.violated_constraint_id == GUIDANCE_PLAN_CONSTRAINT_GEOFENCE);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_pilot_rate_reference(void)
{
  Fx fx;
  AuthorizedOperatorInput op;
  CHECK(fx_init(&fx));
  memset(&op, 0, sizeof(op));
  fill_header(&op.header, MSG_AUTHORIZED_OPERATOR_INPUT, SOURCE_M4, fx.now);
  op.authority_class = 1u; op.selected_source_id = 1u;
  op.authority_epoch = T_AUTH_EPOCH;
  op.input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
  op.payload.roll_norm = 0.5f;
  op.payload.pitch_norm = -0.25f;
  op.payload.yaw_norm = 0.1f;
  op.payload.throttle_norm = 0.6f;
  fx.dir.operation_code = PRODUCT_OP_PILOT;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_RATE;
  fx.auth.authority_owner = 1u;
  {
    GuidanceInput in;
    fx_fill(&fx);
    fx_input(&fx, &in);
    in.authorized_operator_input = &op;
    CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  }
  CHECK(fx.out.guidance_status.reference_kind == GUIDANCE_REFERENCE_PILOT_RATE);
  CHECK(fx.out.pilot_control_reference.reference_kind == PILOT_REFERENCE_RATE);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.valid_mask ==
        (PRODUCT_PILOT_REF_VALID_RATE | PRODUCT_PILOT_REF_VALID_THROTTLE));
  CHECK(fx.out.pilot_control_reference.vehicle_extension.body_rate_frd_radps[0] == 1.0f);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.body_rate_frd_radps[1] == -0.5f);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.collective_throttle_norm == 0.6f);
  CHECK(fx.out.reference.header.validity_flags == 0u);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_pilot_altitude_masks(void)
{
  Fx fx;
  AuthorizedOperatorInput op;
  CHECK(fx_init(&fx));
  memset(&op, 0, sizeof(op));
  fill_header(&op.header, MSG_AUTHORIZED_OPERATOR_INPUT, SOURCE_M4, fx.now);
  op.authority_class = 1u; op.selected_source_id = 1u;
  op.authority_epoch = T_AUTH_EPOCH;
  op.input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
  op.payload.roll_norm = 0.0f;
  op.payload.pitch_norm = 0.0f;
  op.payload.yaw_norm = 0.0f;
  op.payload.throttle_norm = 0.75f;
  fx.dir.operation_code = PRODUCT_OP_PILOT;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_ALT_HOLD;
  fx.auth.authority_owner = 1u;
  fx.nav.core.position_ned_m[2] = -5.0f;
  {
    GuidanceInput in;
    fx_fill(&fx);
    fx_input(&fx, &in);
    in.authorized_operator_input = &op;
    CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  }
  CHECK(fx.out.guidance_status.reference_kind == GUIDANCE_REFERENCE_ALTITUDE_PILOT);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.valid_mask ==
        (PRODUCT_PILOT_REF_VALID_ATTITUDE | PRODUCT_PILOT_REF_VALID_YAW_FF | PRODUCT_PILOT_REF_VALID_ALTITUDE_UP |
         PRODUCT_PILOT_REF_VALID_VERTICAL_UP));
  CHECK(fx.out.pilot_control_reference.vehicle_extension.altitude_above_origin_m == 5.0f);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.vertical_velocity_up_mps == 0.75f);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_stale_time_and_epochs_rejected(void)
{
  Fx fx;
  GuidanceInput in;
  CHECK(fx_init(&fx));
  fx_fill(&fx);
  fx_input(&fx, &in);
  in.control_time_us = fx.nav.header.timestamp_sample_us + 1u;
  CHECK(guidance_update(&in, &fx.out) == STATUS_INVALID_ARGUMENT);
  fx_input(&fx, &in);
  fx.dir.state_epoch = T_STATE_EPOCH + 1u;
  fx_fill(&fx);
  fx_input(&fx, &in);
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_reference_expiry_and_dynamics(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  fx_mission(&fx, 1u, 1u, 0u);
  fx.mission.mission.waypoints[0].position_ned_m[0] = 1.0e6f;
  (void)fx_step(&fx);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.feasibility.feasible == 0u);
  CHECK(fx.out.feasibility.violated_constraint_id != GUIDANCE_PLAN_CONSTRAINT_NONE);
  CHECK(guidance_stop() == STATUS_OK);
}

static void make_box(ProductGeofenceSnapshot *f, float left, float right, uint8_t kind)
{
  memset(f, 0, sizeof(*f));
  f->validity = PRODUCT_GEOFENCE_VALID; f->kind = kind; f->vertex_count = 4;
  f->frame_id = FRAME_LOCAL_NED; f->origin_id = T_ORIGIN; f->state_epoch = T_STATE_EPOCH;
  f->vertices_ne_m[0][0] = left; f->vertices_ne_m[0][1] = -1;
  f->vertices_ne_m[1][0] = right; f->vertices_ne_m[1][1] = -1;
  f->vertices_ne_m[2][0] = right; f->vertices_ne_m[2][1] = 1;
  f->vertices_ne_m[3][0] = left; f->vertices_ne_m[3][1] = 1;
  f->min_height_above_home_m = 0; f->max_height_above_home_m = 20;
}

static void test_config_all_axes_and_unsupported_sources(void)
{
  ProductGuidanceConfig cfg;
  unsigned axis;
  for (axis = 0; axis < 3; ++axis) {
    guidance_default_config(&cfg); cfg.payload.max_jerk_ned_mps3[axis] = NAN;
    CHECK(guidance_init(&cfg) == STATUS_INVALID_ARGUMENT);
    guidance_default_config(&cfg); cfg.payload.max_acceleration_ned_mps2[axis] = 0;
    CHECK(guidance_init(&cfg) == STATUS_INVALID_ARGUMENT);
  }
  guidance_default_config(&cfg); cfg.payload.mass_kg = NAN;
  CHECK(guidance_init(&cfg) == STATUS_INVALID_ARGUMENT);
  guidance_default_config(&cfg); cfg.profile_hash++;
  CHECK(guidance_init(&cfg) == STATUS_INVALID_ARGUMENT);
  for (axis = 1; axis <= 3; ++axis) {
    guidance_default_config(&cfg); cfg.payload.obstacle_source_mode = axis;
    CHECK(guidance_init(&cfg) == STATUS_UNSUPPORTED);
  }
}

static void test_input_and_authorization_fail_closed(void)
{
  Fx fx;
  GuidanceInput in;
  CHECK(fx_init(&fx)); fx_fill(&fx); fx_input(&fx, &in);
  fx.nav.header.validity_flags = 0;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  CHECK(fx.out.reference.header.validity_flags == 0);
  fx_fill(&fx); fx.nav.header.profile_id++;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  fx_fill(&fx); fx.dir.directive_expires_at_us = fx.now;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  fx_fill(&fx); fx.nav.header.timestamp_sample_us -= 6000; in.control_time_us -= 6000;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  fx.dir.operation_code = PRODUCT_OP_MISSION; fx_mission(&fx, 1, 1, 0);
  fx_fill(&fx); fx_input(&fx, &in); fx.mission.authority_epoch++;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  fx.mission.authority_epoch--; fx.mission.current_item_index = PRODUCT_MAX_MISSION_WAYPOINTS;
  CHECK(guidance_update(&in, &fx.out) == STATUS_INVALID_ARGUMENT);
  fx.mission.current_item_index = 0; fx.mission.mission_expires_at_us = fx.now;
  CHECK(guidance_update(&in, &fx.out) == STATUS_STALE_DATA);
  fx.mission.mission_expires_at_us = fx.now+100000; fx.mission.mission.waypoints[0].position_ned_m[0] = NAN;
  CHECK(guidance_update(&in, &fx.out) == STATUS_NUMERIC_ERROR);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_reference_rolls_on_one_continuous_plan(void)
{
  Fx fx;
  ProductMotionPoint expected;
  uint32_t epoch;
  unsigned i;
  CHECK(fx_init(&fx)); fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx_mission(&fx, 1, 1, 0); CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 1); epoch = fx.out.reference.reference_epoch;
  for (i = 0; i < 150; ++i) {
    expected = fx.out.reference.points[0].point;
    fx.now += PRODUCT_ROUTE_PREVIEW_STEP_US - 10000u;
    CHECK(fx_step(&fx) == STATUS_OK);
    if (!fx.out.feasibility.feasible) printf("rolling rejection at %u: reason=%u constraint=%u offset=%u\n",
        i, fx.out.feasibility.reason_code, fx.out.feasibility.violated_constraint_id,
        fx.out.feasibility.earliest_violation_offset_us);
    CHECK(fx.out.feasibility.feasible == 1);
    CHECK(fx.out.reference.reference_epoch == epoch);
    CHECK(fabsf(fx.out.reference.anchor.point.position_ned_m[0]-expected.position_ned_m[0]) < 2e-5f);
    CHECK(fabsf(fx.out.reference.anchor.point.velocity_ned_mps[0]-expected.velocity_ned_mps[0]) < 2e-5f);
    CHECK(fabsf(fx.out.reference.anchor.point.acceleration_ned_mps2[0]-expected.acceleration_ned_mps2[0]) < 2e-5f);
  }
  CHECK(fx.out.reference.anchor.point.position_ned_m[0] > 2);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_hold_latches_nonzero_and_reset_rotates(void)
{
  Fx fx;
  uint32_t epoch;
  CHECK(fx_init(&fx)); fx.nav.core.position_ned_m[0] = 2;
  CHECK(fx_step(&fx) == STATUS_OK); epoch = fx.out.reference.reference_epoch;
  fx.nav.core.position_ned_m[0] = 2.5f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.reference.reference_epoch == epoch);
  CHECK(fabsf(fx.out.reference.anchor.point.position_ned_m[0]-2) < 1e-5f);
  fx.nav.state_epoch++; fx.dir.state_epoch++;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.reference.reference_epoch > epoch);
  CHECK(fabsf(fx.out.reference.anchor.point.position_ned_m[0]-2.5f) < 1e-5f);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_permit_identity_state_policy_revocation(void)
{
  Fx fx;
  GuidanceInput in;
  uint32_t epoch;
  CHECK(fx_init(&fx)); CHECK(fx_step(&fx) == STATUS_OK);
  epoch = fx.out.reference.reference_epoch; fx_make_permit(&fx, epoch);
  fx_fill(&fx); fx_input(&fx, &in); fx.permit.header.boot_id++;
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  fx_fill(&fx); fx.permit.state_epoch++;
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  fx.permit.state_epoch--; fx.permit.output_policy_id++;
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  fx.permit.output_policy_id--; fx.permit.permit_flags |= PERMIT_FLAG_REVOKED;
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_READY);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_fence_geometry_and_home_down_sign(void)
{
  ProductGeofenceSnapshot f;
  float p[3] = {0, 0, 7};
  make_box(&f, -1, 1, PRODUCT_GEOFENCE_KEEP_IN);
  CHECK(fence_geometry_valid(&f));
  f.min_height_above_home_m = 2; f.max_height_above_home_m = 4;
  CHECK(fence_point_valid(&f, p, 10)); /* Home D=10, point D=7 -> +3 m */
  f.vertical_margin_m = 1.1f; CHECK(!fence_geometry_valid(&f));
  make_box(&f, -1, 1, PRODUCT_GEOFENCE_KEEP_IN); f.horizontal_margin_m = -1;
  CHECK(!fence_geometry_valid(&f));
  make_box(&f, -1, 1, PRODUCT_GEOFENCE_KEEP_IN);
  f.vertices_ne_m[1][0] = 1; f.vertices_ne_m[1][1] = 1;
  f.vertices_ne_m[2][0] = 1; f.vertices_ne_m[2][1] = -1;
  CHECK(!fence_geometry_valid(&f)); /* bow tie */
}

static void test_continuous_fence_crossing_between_samples(void)
{
  Fx fx;
  GuidanceInput in;
  MotionReference r;
  ProductGuidanceConfig cfg;
  uint16_t reason, constraint;
  uint32_t offset;
  unsigned i;
  CHECK(fx_init(&fx)); guidance_default_config(&cfg);
  make_box(&fx.fence, 0.0028f, 0.0032f, PRODUCT_GEOFENCE_KEEP_OUT);
  cfg.payload.vehicle_radius_m = 0.00001f; cfg.payload.clearance_margin_m = 0;
  fx.has_fence = 1; fx.fence_count = 1; fx_fill(&fx); fx_input(&fx, &in);
  memset(&r, 0, sizeof(r)); r.point_count = 24;
  for (i = 0; i <= 24; ++i) {
    MotionReferencePoint *p = i ? &r.points[i-1] : &r.anchor;
    p->valid_mask = 0x1Fu; p->offset_us = i*20000u;
    p->point.position_ned_m[0] = -0.01f+i*0.02f; p->point.position_ned_m[2] = -1;
    p->point.velocity_ned_mps[0] = 1;
    CHECK(fence_point_valid(&fx.fence, p->point.position_ned_m, 0));
  }
  CHECK(!guidance_validate_reference(&in, &cfg, &r, &reason, &constraint, &offset));
  CHECK(constraint == GUIDANCE_PLAN_CONSTRAINT_GEOFENCE);
  CHECK(offset < 20000u);
  make_box(&fx.fence, -2, 2, PRODUCT_GEOFENCE_KEEP_IN);
  guidance_default_config(&cfg);
  CHECK(guidance_validate_reference(&in, &cfg, &r, &reason, &constraint, &offset));
  fx.nav.core.position_std_m[0] = 10;
  CHECK(!guidance_validate_reference(&in, &cfg, &r, &reason, &constraint, &offset));
  fx.nav.core.position_std_m[0] = NAN;
  CHECK(!guidance_validate_reference(&in, &cfg, &r, &reason, &constraint, &offset));
  CHECK(reason == M5_VALIDATION_UNCERTAINTY_UNBOUNDED);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_consumed_curve_speed_spike_and_thrust(void)
{
  Fx fx;
  GuidanceInput in;
  MotionReference r;
  ProductGuidanceConfig cfg;
  uint16_t reason, constraint;
  uint32_t offset;
  unsigned i;
  CHECK(fx_init(&fx)); guidance_default_config(&cfg);
  fx_fill(&fx); fx_input(&fx, &in); memset(&r, 0, sizeof(r)); r.point_count = 24;
  for (i = 0; i <= 24; ++i) {
    MotionReferencePoint *p = i ? &r.points[i-1] : &r.anchor;
    p->valid_mask = 0x1Fu; p->offset_us = i*20000u;
    p->point.position_ned_m[0] = i ? 0.1f : 0;
  }
  CHECK(!guidance_validate_reference(&in, &cfg, &r, &reason, &constraint, &offset));
  CHECK(constraint != GUIDANCE_PLAN_CONSTRAINT_NONE);
  fx.env.vehicle_extension.max_total_thrust_n = 8; /* hover alone exceeds this */
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.guidance_status.guidance_state == GUIDANCE_STATE_REJECTED);
  CHECK(fx.out.feasibility.violated_constraint_id == GUIDANCE_PLAN_CONSTRAINT_THRUST);
  CHECK(fx.out.reference.header.validity_flags == 0);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_bernstein_bounds_cover_analytic_extrema(void)
{
  GuidanceQuintic q;
  unsigned trial, derivative, i;
  for (trial = 0; trial < 16; ++trial) {
    float t = 0.4f+trial*0.2f;
    guidance_quintic_boundary(-1+trial*0.1f, 0.3f, -0.2f, 2-trial*0.05f, -0.4f, 0.1f, t, &q);
    CHECK(fabsf(guidance_quintic_eval(&q, 0, 0)-(-1+trial*0.1f)) < 1e-5f);
    CHECK(fabsf(guidance_quintic_eval(&q, t, 1)+0.4f) < 2e-4f);
    for (derivative = 1; derivative <= 3; ++derivative) {
      float lo, hi;
      CHECK(guidance_quintic_bounds(&q, t*0.13f, t*0.83f, (uint8_t)derivative, &lo, &hi));
      for (i = 0; i <= 100; ++i) {
        float tm = t*(0.13f+0.7f*i/100);
        float value = derivative == 3 ? guidance_quintic_jerk(&q, tm) : guidance_quintic_eval(&q, tm, (uint8_t)derivative);
        CHECK(value >= lo-1e-4f && value <= hi+1e-4f);
      }
    }
  }
}

static void test_pilot_yaw_integrates_and_altitude_centre_holds(void)
{
  Fx fx;
  GuidanceInput in;
  AuthorizedOperatorInput op;
  float height, q0;
  CHECK(fx_init(&fx)); memset(&op, 0, sizeof(op));
  op.authority_epoch = T_AUTH_EPOCH; op.authority_class = 1; op.selected_source_id = 1;
  op.input_layout_id = PRODUCT_OPERATOR_INPUT_LAYOUT_ID;
  op.payload.throttle_norm = 0.75f; op.payload.yaw_norm = 0.5f;
  op.payload.roll_norm = 1; op.payload.pitch_norm = 1;
  fx.auth.authority_owner = 1; fx.dir.operation_code = PRODUCT_OP_PILOT;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_ALT_HOLD;
  fx.nav.core.position_ned_m[2] = -5;
  fx_fill(&fx); fx_input(&fx, &in); in.authorized_operator_input = &op;
  fill_header(&op.header, MSG_AUTHORIZED_OPERATOR_INPUT, SOURCE_M4, fx.now);
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  q0 = fx.out.pilot_control_reference.vehicle_extension.q_nb_desired_wxyz[0];
  CHECK(fabsf(2*acosf(q0)-fx.env.vehicle_extension.max_tilt_rad) < 1e-5f);
  fx.now += 100000; fx_fill(&fx); fx_input(&fx, &in); in.authorized_operator_input = &op;
  fill_header(&op.header, MSG_AUTHORIZED_OPERATOR_INPUT, SOURCE_M4, fx.now);
  op.payload.throttle_norm = 0.5f;
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  height = fx.out.pilot_control_reference.vehicle_extension.altitude_above_origin_m;
  CHECK(fabsf(height-5.075f) < 1e-5f);
  CHECK(fx.out.pilot_control_reference.vehicle_extension.q_nb_desired_wxyz[3] > 0);
  fx.now += 100000; fx.nav.core.position_ned_m[2] = -4;
  fx_fill(&fx); fx_input(&fx, &in); in.authorized_operator_input = &op;
  fill_header(&op.header, MSG_AUTHORIZED_OPERATOR_INPUT, SOURCE_M4, fx.now);
  CHECK(guidance_update(&in, &fx.out) == STATUS_OK);
  CHECK(fabsf(fx.out.pilot_control_reference.vehicle_extension.altitude_above_origin_m-height) < 1e-5f);
  CHECK(guidance_stop() == STATUS_OK);
}


static void test_minimum_jerk_single_segment_boundary(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {0.6f};
  float zero[3] = {0};
  double initial_cost = 0.0, optimized_cost = 0.0;

  positions[1][0] = 1.0f;
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 1u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.0f, 0u)) < 1e-6);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.6f, 0u) - 1.0f) < 1e-5);
  CHECK(fabs(initial_cost - optimized_cost) < 1e-7);
}

static void test_minimum_jerk_two_segment_continuity_and_cost(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {1.0f, 1.0f};
  float zero[3] = {0};
  double initial_cost = 0.0, optimized_cost = 0.0;
  float left_velocity, right_velocity, left_acceleration, right_acceleration;

  positions[1][0] = 1.0f;
  positions[2][0] = 2.0f;
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 2u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(optimized_cost < initial_cost);
  CHECK(fabs(initial_cost - 1440.0) < 1e-6);
  CHECK(fabs(optimized_cost - 90.0) < 1e-6);

  CHECK(fabs(guidance_quintic_eval(&output[0][0], 1.0f, 0u) - 1.0f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output[1][0], 0.0f, 0u) - 1.0f) < 1e-5);
  left_velocity = guidance_quintic_eval(&output[0][0], 1.0f, 1u);
  right_velocity = guidance_quintic_eval(&output[1][0], 0.0f, 1u);
  left_acceleration = guidance_quintic_eval(&output[0][0], 1.0f, 2u);
  right_acceleration = guidance_quintic_eval(&output[1][0], 0.0f, 2u);
  CHECK(fabsf(left_velocity - right_velocity) < 1e-5f);
  CHECK(fabsf(left_acceleration - right_acceleration) < 1e-4f);
  CHECK(left_velocity > 1.8f && left_velocity < 1.95f);
}

static void test_minimum_jerk_nonuniform_multiaxis(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {0.4f, 0.8f};
  float zero[3] = {0};
  double initial_cost = 0.0, optimized_cost = 0.0;

  positions[1][0] = 0.5f; positions[2][0] = 2.0f;
  positions[1][1] = 0.2f; positions[2][1] = 0.4f;
  positions[1][2] = -0.1f; positions[2][2] = -0.2f;
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 2u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(isfinite(initial_cost) && isfinite(optimized_cost));
  CHECK(optimized_cost < initial_cost);
  for (unsigned axis = 0u; axis < 3u; ++axis) {
    CHECK(fabs(guidance_quintic_eval(&output[0][axis], 0.4f, 0u) -
               positions[1][axis]) < 2e-5);
    CHECK(fabs(guidance_quintic_eval(&output[1][axis], 0.0f, 0u) -
               positions[1][axis]) < 2e-5);
    CHECK(fabs(guidance_quintic_eval(&output[0][axis], 0.4f, 1u) -
               guidance_quintic_eval(&output[1][axis], 0.0f, 1u)) < 2e-4);
    CHECK(fabs(guidance_quintic_eval(&output[0][axis], 0.4f, 2u) -
               guidance_quintic_eval(&output[1][axis], 0.0f, 2u)) < 2e-3);
  }
}

static void test_minimum_jerk_eight_segments_deterministic(void)
{
  GuidanceMinimumJerkWorkspace workspace_a, workspace_b;
  GuidanceQuintic output_a[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  GuidanceQuintic output_b[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS];
  float zero[3] = {0};
  double initial_a = 0.0, optimized_a = 0.0, initial_b = 0.0, optimized_b = 0.0;

  for (unsigned i = 0u; i < GUIDANCE_MIN_JERK_MAX_SEGMENTS; ++i) {
    durations[i] = 0.2f;
    positions[i + 1u][0] = 0.1f * (float)(i + 1u);
    positions[i + 1u][1] = 0.02f * (float)((i + 1u) % 3u);
    positions[i + 1u][2] = -0.01f * (float)(i + 1u);
  }
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 8u,
      zero, zero, zero, zero, &workspace_a, output_a, &initial_a, &optimized_a));
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 8u,
      zero, zero, zero, zero, &workspace_b, output_b, &initial_b, &optimized_b));
  CHECK(memcmp(output_a, output_b, sizeof(output_a)) == 0);
  CHECK(initial_a == initial_b && optimized_a == optimized_b);
  CHECK(optimized_a < initial_a);
}

static void test_minimum_jerk_rejects_invalid_inputs_atomically(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {0.2f, 0.2f, 0.2f, 0.2f,
      0.2f, 0.2f, 0.2f, 0.2f};
  float zero[3] = {0};
  double initial_cost = 1.0, optimized_cost = 1.0;

  memset(output, 0xA5, sizeof(output));
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 0u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(output[0][0].c[0] == 0.0 && initial_cost == 0.0 && optimized_cost == 0.0);

  durations[0] = 0.3f;
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 2u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  durations[0] = 0.20001f;
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 1u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  durations[0] = 0.2f;
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 9u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  for (unsigned i = 0u; i < 8u; ++i) durations[i] = 0.8f;
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 8u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));

  durations[0] = 0.2f;
  positions[1][0] = NAN;
  CHECK(!guidance_minimum_jerk_optimize(positions, durations, 1u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(output[0][0].c[0] == 0.0);
}


static void test_minimum_jerk_objective_sums_three_axes(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {1.0f, 1.0f};
  float zero[3] = {0};
  double initial_cost = 0.0, optimized_cost = 0.0;

  positions[1][0] = 1.0f; positions[2][0] = 2.0f;
  positions[1][1] = 0.5f; positions[2][1] = 1.0f;
  positions[1][2] = -0.25f; positions[2][2] = -0.5f;
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 2u,
      zero, zero, zero, zero, &workspace, output, &initial_cost, &optimized_cost));
  CHECK(fabs(initial_cost - 1890.0) < 1e-6);
  CHECK(fabs(optimized_cost - 118.125) < 1e-6);
}


static void test_minimum_jerk_nonzero_endpoint_boundary_states(void)
{
  GuidanceMinimumJerkWorkspace workspace;
  GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3];
  float positions[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3] = {{0}};
  float durations[GUIDANCE_MIN_JERK_MAX_SEGMENTS] = {0.4f, 0.6f};
  float start_velocity[3] = {0.2f, 0.0f, 0.0f};
  float start_acceleration[3] = {0.1f, 0.0f, 0.0f};
  float end_velocity[3] = {0.1f, 0.0f, 0.0f};
  float end_acceleration[3] = {-0.05f, 0.0f, 0.0f};
  double initial_cost = 0.0, optimized_cost = 0.0;

  positions[1][0] = 1.0f;
  positions[2][0] = 2.0f;
  CHECK(guidance_minimum_jerk_optimize(positions, durations, 2u,
      start_velocity, start_acceleration, end_velocity, end_acceleration,
      &workspace, output, &initial_cost, &optimized_cost));
  CHECK(optimized_cost < initial_cost);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.0f, 0u)) < 1e-6);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.0f, 1u) - 0.2f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.0f, 2u) - 0.1f) < 1e-4);
  CHECK(fabs(guidance_quintic_eval(&output[1][0], 0.6f, 0u) - 2.0f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output[1][0], 0.6f, 1u) - 0.1f) < 1e-4);
  CHECK(fabs(guidance_quintic_eval(&output[1][0], 0.6f, 2u) + 0.05f) < 1e-3);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.4f, 1u) -
             guidance_quintic_eval(&output[1][0], 0.0f, 1u)) < 1e-4);
  CHECK(fabs(guidance_quintic_eval(&output[0][0], 0.4f, 2u) -
             guidance_quintic_eval(&output[1][0], 0.0f, 2u)) < 1e-3);
}


static void planner_nav_init(NavigationSnapshot *nav, uint64_t now_us)
{
  unsigned axis;
  memset(nav, 0, sizeof(*nav));
  fill_header(&nav->header, MSG_NAVIGATION_SNAPSHOT, SOURCE_M3, now_us);
  nav->state_epoch = T_STATE_EPOCH;
  nav->origin_id = T_ORIGIN;
  nav->core.valid_mask = 0xFFu;
  nav->core.q_nb_wxyz[0] = 1.0f;
  for (axis = 0u; axis < 3u; ++axis) {
    nav->core.position_std_m[axis] = 0.01f;
    nav->core.velocity_std_mps[axis] = 0.01f;
  }
}

static void planner_observation_init(ProductObstacleObservation *observation,
                                     const NavigationSnapshot *nav, uint64_t now_us,
                                     uint32_t id)
{
  memset(observation, 0, sizeof(*observation));
  fill_header(&observation->header, MSG_PRODUCT_OBSTACLE_OBSERVATION,
              SOURCE_PLATFORM, now_us);
  observation->observation_id = id;
  observation->sensor_instance_id = 2u;
  observation->source_health_epoch = 1u;
  observation->origin_id = nav->origin_id;
  observation->state_epoch = nav->state_epoch;
  observation->source_health = PRODUCT_OBS_SOURCE_NORMAL;
  observation->confidence_0_to_1 = 1.0f;
}

static void test_m5_map_fixed_grid_and_bounds(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  float p[3], outside[3] = {4.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  const uint64_t now = 1000000u;
  planner_nav_init(&nav, now);
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  p[0] = p[1] = p[2] = -4.0f;
  CHECK(m5_local_map_world_to_voxel(&map, p, &voxel) && voxel == 0u);
  p[0] = p[1] = p[2] = 3.999f;
  CHECK(m5_local_map_world_to_voxel(&map, p, &voxel) && voxel == 4095u);
  CHECK(!m5_local_map_world_to_voxel(&map, outside, &voxel));
  m5_local_map_voxel_center(&map, 4095u, p);
  CHECK(fabsf(p[0] - 3.75f) < 1e-6f);
  CHECK(fabsf(p[1] - 3.75f) < 1e-6f);
  CHECK(fabsf(p[2] - 3.75f) < 1e-6f);
}

static void test_m5_map_initial_origin_is_cell_aligned(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  const uint64_t now = 1500000u;
  planner_nav_init(&nav, now);
  nav.core.position_ned_m[0] = 0.24f;
  nav.core.position_ned_m[1] = -0.26f;
  nav.core.position_ned_m[2] = 1.13f;
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  CHECK(fabsf(map.center_ned_m[0] - 0.0f) < 1e-6f);
  CHECK(fabsf(map.center_ned_m[1] + 0.5f) < 1e-6f);
  CHECK(fabsf(map.center_ned_m[2] - 1.0f) < 1e-6f);
  /* Geometry is initialized, but no observation batch has made the map ready. */
  CHECK(!m5_local_map_is_ready(&map, &nav, now));
}

static void test_m5_map_roll_preserves_world_cells(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observation;
  float old_world[3] = {0.25f, 0.0f, 0.0f};
  float cell_center[3];
  uint16_t old_voxel = 0u, new_voxel = 0u;
  uint8_t state = M5_MAP_CELL_UNKNOWN;
  const uint64_t t0 = 1500000u;

  planner_nav_init(&nav, t0);
  CHECK(m5_local_map_init(&map, &nav, t0, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  CHECK(m5_local_map_world_to_voxel(&map, old_world, &old_voxel));
  map.cells[old_voxel].state = M5_MAP_CELL_FREE_CONFIRMED;
  map.cells[old_voxel].log_odds = -4;
  map.cells[old_voxel].last_update_ms = (uint32_t)(t0 / 1000u);

  planner_nav_init(&nav, t0 + 10000u);
  nav.core.position_ned_m[0] = 0.5f;
  planner_observation_init(&observation, &nav, t0 + 10000u, 31u);
  observation.observation_kind = PRODUCT_OBS_FREE_SPACE_RAY;
  observation.ray_start_ned_m[0] = 0.5f;
  observation.ray_start_ned_m[1] = 0.0f;
  observation.ray_start_ned_m[2] = 0.0f;
  observation.ray_end_ned_m[0] = 0.75f;
  observation.ray_end_ned_m[1] = 0.0f;
  observation.ray_end_ned_m[2] = 0.0f;
  CHECK(m5_local_map_ingest(&map, &nav, &observation, 1u, t0 + 10000u) == STATUS_OK);
  CHECK(fabsf(map.center_ned_m[0] - 0.5f) < 1e-6f);
  CHECK(m5_local_map_world_to_voxel(&map, old_world, &new_voxel));
  CHECK(new_voxel == (uint16_t)(old_voxel - 1u));
  m5_local_map_voxel_center(&map, new_voxel, cell_center);
  CHECK(fabsf(cell_center[0] - old_world[0]) < 1e-6f);
  CHECK(m5_local_map_query(&map, old_world, t0 + 10000u, &state) == STATUS_OK);
  CHECK(state == M5_MAP_CELL_FREE_CONFIRMED);
}

static void test_m5_map_free_ray_log_odds_and_stale(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observations[2];
  float start[3] = {-1.75f, -1.75f, -1.75f};
  float outside[3] = {4.0f, 0.0f, 0.0f};
  uint8_t state = M5_MAP_CELL_INVALID;
  uint16_t voxel = 0u;
  const uint64_t now = 2000000u;

  planner_nav_init(&nav, now);
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  planner_observation_init(&observations[0], &nav, now, 1u);
  planner_observation_init(&observations[1], &nav, now, 2u);
  for (unsigned i = 0u; i < 2u; ++i) {
    observations[i].observation_kind = PRODUCT_OBS_FREE_SPACE_RAY;
    observations[i].ray_start_ned_m[0] = -1.75f;
    observations[i].ray_start_ned_m[1] = -1.75f;
    observations[i].ray_start_ned_m[2] = -1.75f;
    observations[i].ray_end_ned_m[0] = -1.25f;
    observations[i].ray_end_ned_m[1] = -1.75f;
    observations[i].ray_end_ned_m[2] = -1.75f;
  }
  CHECK(m5_local_map_ingest(&map, &nav, observations, 1u, now) == STATUS_OK);
  CHECK(m5_local_map_world_to_voxel(&map, start, &voxel));
  CHECK(map.cells[voxel].log_odds == -2);
  CHECK(map.cells[voxel].state == M5_MAP_CELL_UNKNOWN);
  CHECK(m5_local_map_ingest(&map, &nav, &observations[1], 1u, now) == STATUS_OK);
  CHECK(map.cells[voxel].log_odds == -4);
  CHECK(m5_local_map_is_ready(&map, &nav, now));
  CHECK(m5_local_map_query(&map, start, now, &state) == STATUS_OK);
  CHECK(state == M5_MAP_CELL_FREE_CONFIRMED);
  CHECK(m5_local_map_query(&map, outside, now, &state) == STATUS_OUT_OF_RANGE);
  CHECK(state == M5_MAP_CELL_INVALID);
  CHECK(m5_local_map_query(&map, start, now + 501000u, &state) == STATUS_OK);
  CHECK(state == M5_MAP_CELL_STALE);
}

static void test_m5_map_source_lease_refresh(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observation;
  const uint64_t t0 = 2500000u;
  float p[3] = {-1.75f, -1.75f, -1.75f};
  uint8_t state = M5_MAP_CELL_INVALID;

  planner_nav_init(&nav, t0);
  CHECK(m5_local_map_init(&map, &nav, t0, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  planner_observation_init(&observation, &nav, t0, 21u);
  observation.observation_kind = PRODUCT_OBS_FREE_SPACE_RAY;
  observation.ray_start_ned_m[0] = p[0];
  observation.ray_start_ned_m[1] = p[1];
  observation.ray_start_ned_m[2] = p[2];
  observation.ray_end_ned_m[0] = -1.25f;
  observation.ray_end_ned_m[1] = p[1];
  observation.ray_end_ned_m[2] = p[2];
  CHECK(m5_local_map_ingest(&map, &nav, &observation, 1u, t0) == STATUS_OK);
  CHECK(map.source_deadline_us == t0 + 500000u);

  planner_nav_init(&nav, t0 + 100000u);
  planner_observation_init(&observation, &nav, t0 + 100000u, 22u);
  observation.observation_kind = PRODUCT_OBS_FREE_SPACE_RAY;
  observation.ray_start_ned_m[0] = p[0];
  observation.ray_start_ned_m[1] = p[1];
  observation.ray_start_ned_m[2] = p[2];
  observation.ray_end_ned_m[0] = -1.25f;
  observation.ray_end_ned_m[1] = p[1];
  observation.ray_end_ned_m[2] = p[2];
  CHECK(m5_local_map_ingest(&map, &nav, &observation, 1u, t0 + 100000u) == STATUS_OK);
  CHECK(map.source_deadline_us == t0 + 600000u);

  planner_nav_init(&nav, t0 + 550000u);
  CHECK(m5_local_map_is_ready(&map, &nav, t0 + 550000u));
  CHECK(!m5_local_map_is_ready(&map, &nav, t0 + 600001u));
  CHECK(m5_local_map_query(&map, p, t0 + 600001u, &state) == STATUS_OK);
  CHECK(state == M5_MAP_CELL_STALE);
}

static void test_m5_map_occupied_primitive_and_dynamic_track(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observation;
  float center[3] = {-1.75f, -1.75f, -1.75f};
  uint8_t state = M5_MAP_CELL_UNKNOWN;
  const uint64_t now = 3000000u;

  planner_nav_init(&nav, now);
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  planner_observation_init(&observation, &nav, now, 7u);
  observation.sensor_instance_id = 70002u;
  observation.observation_kind = PRODUCT_OBS_OCCUPIED_PRIMITIVE;
  observation.shape_kind = PRODUCT_OBS_SHAPE_SPHERE;
  for (unsigned axis = 0u; axis < 3u; ++axis) {
    observation.obstacle_center_ned_m[axis] = center[axis];
    observation.obstacle_half_extent_m[axis] = 0.25f;
    observation.position_std_m[axis] = 0.02f;
    observation.velocity_std_mps[axis] = 0.01f;
  }
  observation.obstacle_velocity_ned_mps[0] = 0.5f;
  CHECK(m5_local_map_ingest(&map, &nav, &observation, 1u, now) == STATUS_OK);
  CHECK(m5_local_map_query(&map, center, now, &state) == STATUS_OK);
  CHECK(state == M5_MAP_CELL_OCCUPIED_CONFIRMED);
  CHECK(map.tracks[0].id == 7u);
  CHECK(map.tracks[0].source_id == 70002u);
  CHECK(fabsf(map.tracks[0].radius_m - 0.25f) < 1e-6f);
  CHECK(map.tracks[0].expires_us == observation.header.valid_until_us);
  CHECK(m5_local_map_is_ready(&map, &nav, now));
}

static void test_m5_map_epoch_reset_and_reject_source(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observation;
  uint8_t state = M5_MAP_CELL_INVALID;
  float p[3] = {-1.75f, -1.75f, -1.75f};
  const uint64_t now = 4000000u;

  planner_nav_init(&nav, now);
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  map.cells[0].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  nav.state_epoch++;
  fill_header(&nav.header, MSG_NAVIGATION_SNAPSHOT, SOURCE_M3, now + 10000u);
  CHECK(m5_local_map_ingest(&map, &nav, NULL, 0u, now + 10000u) == STATUS_STALE_DATA);
  CHECK(map.state_epoch == nav.state_epoch);
  CHECK(map.last_ingest_us == 0u);
  CHECK(map.cells[0].state == M5_MAP_CELL_UNKNOWN);
  planner_observation_init(&observation, &nav, now + 10000u, 8u);
  observation.observation_kind = PRODUCT_OBS_OCCUPIED_PRIMITIVE;
  observation.shape_kind = PRODUCT_OBS_SHAPE_POINT;
  observation.obstacle_center_ned_m[0] = p[0];
  observation.obstacle_center_ned_m[1] = p[1];
  observation.obstacle_center_ned_m[2] = p[2];
  observation.source_health = PRODUCT_OBS_SOURCE_REJECT;
  CHECK(m5_local_map_ingest(&map, &nav, &observation, 1u, now + 10000u) ==
        STATUS_NOT_PERMITTED);
  CHECK(map.valid == 0u);
  CHECK(m5_local_map_query(&map, p, now + 10000u, &state) == STATUS_NOT_PERMITTED);
  CHECK(state == M5_MAP_CELL_INVALID);
}

static void test_m5_map_batch_validation_is_fail_closed(void)
{
  M5LocalMap map;
  NavigationSnapshot nav;
  ProductObstacleObservation observations[2];
  const uint64_t now = 5000000u;
  uint16_t voxel = 0u;
  float p[3] = {-1.75f, -1.75f, -1.75f};

  planner_nav_init(&nav, now);
  CHECK(m5_local_map_init(&map, &nav, now, 0.25f, 0.30f, 2.0f,
                          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  planner_observation_init(&observations[0], &nav, now, 11u);
  planner_observation_init(&observations[1], &nav, now, 12u);
  observations[0].observation_kind = PRODUCT_OBS_FREE_SPACE_RAY;
  observations[0].ray_start_ned_m[0] = p[0];
  observations[0].ray_start_ned_m[1] = p[1];
  observations[0].ray_start_ned_m[2] = p[2];
  observations[0].ray_end_ned_m[0] = -1.25f;
  observations[0].ray_end_ned_m[1] = p[1];
  observations[0].ray_end_ned_m[2] = p[2];
  observations[1].observation_kind = PRODUCT_OBS_OCCUPIED_PRIMITIVE;
  observations[1].shape_kind = 99u;
  observations[1].obstacle_center_ned_m[0] = p[0];
  observations[1].obstacle_center_ned_m[1] = p[1];
  observations[1].obstacle_center_ned_m[2] = p[2];
  CHECK(m5_local_map_ingest(&map, &nav, observations, 2u, now) == STATUS_STALE_DATA);
  CHECK(m5_local_map_world_to_voxel(&map, p, &voxel));
  CHECK(map.cells[voxel].log_odds == 0);
  CHECK(map.valid == 0u);
}

static void test_m5_kinodynamic_key_and_primitive_manifest(void)
{
  const M5KinodynamicPrimitive *table;
  uint8_t count = 0u, velocity_bin, yaw_bin, time_bin;
  uint16_t voxel;
  uint32_t key;
  float velocity[3];

  table = m5_kinodynamic_primitive_table(&count);
  CHECK(table != NULL && count == 60u);
  CHECK(m5_kinodynamic_primitive_table_validate());
  CHECK(table[0].kind == M5_PRIMITIVE_TRANSLATE && table[0].dir == 0u &&
        table[0].speed_id == 0u && table[0].duration_us == 200000u);
  CHECK(table[31].dir == 7u && table[31].speed_id == 1u &&
        table[31].duration_us == 400000u);
  CHECK(table[32].speed_id == 2u && table[32].target_speed == 2.0f);
  CHECK(table[40].kind == M5_PRIMITIVE_VERTICAL);
  CHECK(table[48].kind == M5_PRIMITIVE_YAW && table[48].yaw_rate_hint < 0.0f);
  CHECK(table[56].kind == M5_PRIMITIVE_WAIT);
  CHECK(table[58].kind == M5_PRIMITIVE_BRAKE);
  for (unsigned i = 60u; i < 64u; ++i)
    CHECK(table[i].kind == 0u && table[i].duration_us == 0u &&
          table[i].target_speed == 0.0f && table[i].yaw_rate_hint == 0.0f);

  key = m5_kinodynamic_encode_key(4095u, 28u, 15u, 31u);
  CHECK(key != UINT32_MAX && (key >> 26) == 0u);
  CHECK(m5_kinodynamic_decode_key(key, &voxel, &velocity_bin, &yaw_bin, &time_bin));
  CHECK(voxel == 4095u && velocity_bin == 28u && yaw_bin == 15u && time_bin == 31u);
  CHECK(m5_kinodynamic_encode_key(4096u, 0u, 0u, 0u) == UINT32_MAX);
  CHECK(m5_kinodynamic_encode_key(0u, 29u, 0u, 0u) == UINT32_MAX);
  CHECK(m5_kinodynamic_encode_key(0u, 0u, 0u, 32u) == UINT32_MAX);
  CHECK(m5_kinodynamic_velocity_bin(0u, velocity));
  CHECK(velocity[0] == 0.0f && velocity[1] == 0.0f && velocity[2] == 0.0f);
  CHECK(m5_kinodynamic_velocity_bin(25u, velocity) && velocity[2] < 0.0f);
  CHECK(m5_kinodynamic_velocity_bin(28u, velocity) && velocity[2] > 0.0f);
  CHECK(!m5_kinodynamic_velocity_bin(29u, velocity));
}

static uint64_t planner_clock_read(void *context)
{
  return *(const uint64_t *)context;
}

typedef struct {
  NavigationSnapshot navigation;
  ProductGuidanceConfig config;
  SafetyEnvelope envelope;
  M5LocalMap map;
  M5AstarWorkspace workspace;
  M5AstarPath path;
  M5AstarStats stats;
  M5AstarRequest request;
  uint64_t clock_us;
} PlannerFx;

static void planner_fx_init(PlannerFx *fx, uint64_t now_us)
{
  unsigned i, axis;
  memset(fx, 0, sizeof(*fx));
  fx->clock_us = now_us;
  planner_nav_init(&fx->navigation, now_us);
  guidance_default_config(&fx->config);
  fx->config.payload.obstacle_source_mode = 1u; /* Host STATIC_MAP profile */
  fx->config.payload.max_dynamic_prediction_us = 1000000u;
  /* This deterministic one-edge fixture tests search, not the default
     conservative QUAD tuning. Keep config and envelope limits consistent. */
  for (axis = 0u; axis < 3u; ++axis) {
    fx->config.payload.max_acceleration_ned_mps2[axis] = 20.0f;
    fx->config.payload.max_jerk_ned_mps3[axis] = 1000.0f;
  }
  fill_header(&fx->envelope.header, MSG_SAFETY_ENVELOPE, SOURCE_M4, now_us);
  fx->envelope.safety_envelope_id = 1u;
  fx->envelope.max_linear_speed_mps = 4.0f;
  fx->envelope.max_linear_accel_mps2 = 20.0f;
  fx->envelope.max_angular_rate_radps = 10.0f;
  fx->envelope.max_yaw_rate_radps = 2.0f;
  fx->envelope.vehicle_extension.valid_mask = 0x7Fu;
  fx->envelope.vehicle_extension.max_tilt_rad = 1.2f;
  fx->envelope.vehicle_extension.min_total_thrust_n = 4.0f;
  fx->envelope.vehicle_extension.max_total_thrust_n = 30.0f;
  for (axis = 0u; axis < 3u; ++axis) {
    fx->envelope.vehicle_extension.max_velocity_ned_mps[axis] =
        fx->config.payload.max_velocity_ned_mps[axis];
    fx->envelope.vehicle_extension.max_acceleration_ned_mps2[axis] = 20.0f;
    fx->envelope.vehicle_extension.max_jerk_ned_mps3[axis] = 1000.0f;
    fx->envelope.vehicle_extension.max_body_rate_frd_radps[axis] = 10.0f;
  }
  CHECK(m5_local_map_init(&fx->map, &fx->navigation, now_us,
      fx->config.payload.vehicle_radius_m, fx->config.payload.clearance_margin_m,
      fx->config.payload.uncertainty_inflation_factor,
      fx->config.payload.max_dynamic_prediction_us) == STATUS_OK);
  for (i = 0u; i < M5_MAP_CELL_COUNT; ++i) {
    fx->map.cells[i].log_odds = -4;
    fx->map.cells[i].state = M5_MAP_CELL_FREE_CONFIRMED;
    fx->map.cells[i].source_mask = 1u;
    fx->map.cells[i].last_update_ms = (uint32_t)(now_us / 1000u);
  }
  fx->map.last_ingest_us = now_us;
  fx->map.source_deadline_us = now_us + 500000u;
  fx->request.map = &fx->map;
  fx->request.navigation = &fx->navigation;
  fx->request.guidance_config = &fx->config;
  fx->request.safety_envelope = &fx->envelope;
  fx->request.start_position_ned_m[0] = -1.75f;
  fx->request.start_position_ned_m[1] = -1.75f;
  fx->request.start_position_ned_m[2] = -1.75f;
  fx->request.start_velocity_ned_mps[0] = 1.0f;
  fx->request.goal_position_ned_m[0] = -1.25f;
  fx->request.goal_position_ned_m[1] = -1.75f;
  fx->request.goal_position_ned_m[2] = -1.75f;
  fx->request.goal_yaw_rad = 0.0f;
  fx->request.start_yaw_rad = 0.0f;
  fx->request.acceptance_radius_m = 0.15f;
  fx->request.terminal_speed_max_mps = 1.1f;
  fx->request.yaw_tolerance_rad = 0.1f;
  fx->request.evaluation_time_us = now_us;
  fx->request.planning_deadline_monotonic_us = now_us + 100000u;
  fx->request.monotonic_now = planner_clock_read;
  fx->request.clock_context = &fx->clock_us;
}

static void smoother_fx_path_init(PlannerFx *fx, uint64_t now_us)
{
  planner_fx_init(fx, now_us);
  fx->path.segment_count = 2u;
  fx->path.map_revision = fx->map.map_revision;
  fx->path.total_cost_s = 0.8f;
  fx->path.offset_us[0] = 0u;
  fx->path.offset_us[1] = 400000u;
  fx->path.offset_us[2] = 800000u;
  fx->path.position_ned_m[0][0] = -1.75f;
  fx->path.position_ned_m[1][0] = -1.35f;
  fx->path.position_ned_m[2][0] = -0.95f;
  for (unsigned knot = 0u; knot < 3u; ++knot) {
    fx->path.position_ned_m[knot][1] = -1.75f;
    fx->path.position_ned_m[knot][2] = -1.75f;
    fx->path.velocity_ned_mps[knot][0] = 1.0f;
    fx->path.velocity_ned_mps[knot][1] = 0.0f;
    fx->path.velocity_ned_mps[knot][2] = 0.0f;
    fx->path.acceleration_ned_mps2[knot][0] = 0.0f;
    fx->path.acceleration_ned_mps2[knot][1] = 0.0f;
    fx->path.acceleration_ned_mps2[knot][2] = 0.0f;
    fx->path.yaw_rad[knot] = 0.0f;
    fx->path.yaw_rate_radps[knot] = 0.0f;
  }
  fx->path.primitive_index[0] = 3u;
  fx->path.primitive_index[1] = 3u;
}

static uint8_t smoother_fx_time_scale(PlannerFx *fx, uint64_t now_us,
                                      M5TimeScaledPath *timed_path)
{
  return (uint8_t)(m5_time_scaler_build(&fx->path, &fx->config, &fx->envelope,
      &fx->navigation, now_us, timed_path) == M5_TIME_SCALER_OK);
}

static void test_m5_time_scaler_bounded_growth(void)
{
  PlannerFx fx;
  M5TimeScaledPath timed;
  const uint64_t now = 9400000u;
  smoother_fx_path_init(&fx, now);
  CHECK(smoother_fx_time_scale(&fx, now, &timed));
  CHECK(timed.path.segment_count == 2u);
  CHECK(timed.attempts[0] == 2u && timed.attempts[1] == 2u);
  CHECK(timed.path.offset_us[1] == 400000u);
  CHECK(timed.path.offset_us[2] == 800000u);
  CHECK(timed.path.total_cost_s == 0.8f);
}

static void test_m5_time_scaler_enforces_yaw_rate(void)
{
  PlannerFx fx;
  M5TimeScaledPath timed;
  const uint64_t now = 9475000u;
  planner_fx_init(&fx, now);
  fx.path.segment_count = 1u;
  fx.path.map_revision = fx.map.map_revision;
  fx.path.total_cost_s = 0.8f;
  fx.path.offset_us[0] = 0u;
  fx.path.offset_us[1] = 800000u;
  fx.path.position_ned_m[0][0] = fx.path.position_ned_m[1][0] = -1.75f;
  fx.path.position_ned_m[0][1] = fx.path.position_ned_m[1][1] = -1.75f;
  fx.path.position_ned_m[0][2] = fx.path.position_ned_m[1][2] = -1.75f;
  fx.path.yaw_rad[0] = 0.0f;
  fx.path.yaw_rad[1] = 0.3926990817f;
  fx.path.primitive_index[0] = 49u; /* one yaw bin, 800 ms primitive */
  fx.envelope.max_yaw_rate_radps = 0.5f;
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  CHECK(timed.attempts[0] == 8u);
  CHECK(timed.path.offset_us[1] >= 4800000u);
}

static void test_m5_time_scaler_capacity_and_envelope_fail_closed(void)
{
  PlannerFx fx;
  M5TimeScaledPath timed;
  const uint64_t now = 9450000u;
  smoother_fx_path_init(&fx, now);
  for (unsigned axis = 0u; axis < 3u; ++axis) {
    fx.config.payload.max_velocity_ned_mps[axis] = 0.01f;
    fx.config.payload.max_acceleration_ned_mps2[axis] = 0.01f;
    fx.config.payload.max_jerk_ned_mps3[axis] = 0.01f;
    fx.envelope.vehicle_extension.max_velocity_ned_mps[axis] = 0.01f;
    fx.envelope.vehicle_extension.max_acceleration_ned_mps2[axis] = 0.01f;
    fx.envelope.vehicle_extension.max_jerk_ned_mps3[axis] = 0.01f;
  }
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_CAPACITY_EXCEEDED);
  smoother_fx_path_init(&fx, now);
  fx.envelope.vehicle_extension.valid_mask &= ~PRODUCT_ENVELOPE_VALID_JERK;
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_INVALID_INPUT);
}

static void test_m5_time_scaler_enforces_global_vector_limits(void)
{
  PlannerFx fx;
  M5TimeScaledPath timed;
  const uint64_t now = 9470000u;
  planner_fx_init(&fx, now);
  memset(&fx.path, 0, sizeof(fx.path));
  fx.path.segment_count = 1u;
  fx.path.map_revision = fx.map.map_revision;
  fx.path.total_cost_s = 0.4f;
  fx.path.offset_us[0] = 0u;
  fx.path.offset_us[1] = 400000u;
  fx.path.position_ned_m[0][0] = -1.75f;
  fx.path.position_ned_m[0][1] = -1.75f;
  fx.path.position_ned_m[0][2] = -1.75f;
  fx.path.position_ned_m[1][0] = -1.35f;
  fx.path.position_ned_m[1][1] = -1.35f;
  fx.path.position_ned_m[1][2] = -1.75f;
  fx.path.primitive_index[0] = 3u;

  /* Each axis is individually below 4 m/s, but the 3-D norm is not. */
  fx.envelope.max_linear_speed_mps = 1.5f;
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  CHECK(timed.path.offset_us[1] >= 800000u);

  /* Repeat with speed relaxed and the global acceleration norm binding. */
  fx.envelope.max_linear_speed_mps = 4.0f;
  fx.envelope.max_linear_accel_mps2 = 5.0f;
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  CHECK(timed.path.offset_us[1] >= 1000000u);
}

static void test_m5_smoother_minimum_jerk_continuity(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5TimeScaledPath timed;
  M5SmoothedTrajectory output;
  const uint64_t now = 9500000u;
  smoother_fx_path_init(&fx, now);
  CHECK(smoother_fx_time_scale(&fx, now, &timed));
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 2u);
  CHECK(output.fallback_used == 0u);
  CHECK(output.cost_valid == 1u);
  CHECK(output.map_revision == fx.map.map_revision);
  CHECK(output.total_duration_us == 800000u);
  CHECK(output.segment_duration_us[0] == 400000u);
  CHECK(output.segment_duration_us[1] == 400000u);
  CHECK(output.optimized_cost <= output.initial_cost + 1e-8);
  CHECK(fabs(guidance_quintic_eval(&output.position[0][0], 0.0f, 0u) + 1.75f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[0][0], 0.4f, 0u) + 1.35f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[1][0], 0.0f, 0u) + 1.35f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[1][0], 0.4f, 0u) + 0.95f) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[0][0], 0.4f, 1u) -
             guidance_quintic_eval(&output.position[1][0], 0.0f, 1u)) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[0][0], 0.4f, 2u) -
             guidance_quintic_eval(&output.position[1][0], 0.0f, 2u)) < 1e-4);
}

static void test_m5_smoother_rejects_stale_and_invalid_path(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5TimeScaledPath timed;
  M5SmoothedTrajectory output;
  const uint64_t now = 9600000u;
  smoother_fx_path_init(&fx, now);
  CHECK(smoother_fx_time_scale(&fx, now, &timed));
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  fx.map.map_revision++;
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_INPUT_STALE);
  CHECK(output.segment_count == 0u);

  smoother_fx_path_init(&fx, now);
  CHECK(smoother_fx_time_scale(&fx, now, &timed));
  timed.path.offset_us[1] = 450000u;
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_CAPACITY_EXCEEDED);
  CHECK(output.segment_count == 0u);
}

static void test_m5_astar_finds_deterministic_one_edge_path(void)
{
  PlannerFx fx;
  M5AstarPath first;
  const uint64_t now = 6000000u;
  M5PlannerResult result;
  planner_fx_init(&fx, now);
  result = m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats);
  if (result != M5_PLANNER_OK) {
    printf("A* diagnostic result=%u expanded=%u generated=%u rejected=%u nodes=%u open=%u map_ready=%u\\n",
        (unsigned)result, (unsigned)fx.stats.expanded_nodes,
        (unsigned)fx.stats.generated_edges, (unsigned)fx.stats.rejected_edges,
        (unsigned)fx.stats.node_count, (unsigned)fx.workspace.open_count,
        (unsigned)m5_local_map_is_ready(&fx.map, &fx.navigation, now));
  }
  CHECK(result == M5_PLANNER_OK);
  CHECK(fx.path.segment_count == 1u);
  CHECK(fx.path.map_revision == fx.map.map_revision);
  CHECK(fabsf(fx.path.position_ned_m[0][0] + 1.75f) < 1e-6f);
  CHECK(fabsf(fx.path.position_ned_m[1][0] + 1.25f) < 1e-6f);
  CHECK(fabsf(fx.path.velocity_ned_mps[1][0] - 0.5f) < 1e-6f);
  CHECK(fx.path.offset_us[1] == 400000u);
  CHECK(fx.stats.expanded_nodes <= M5_ASTAR_MAX_EXPANSIONS);
  first = fx.path;
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  CHECK(memcmp(&first, &fx.path, sizeof(first)) == 0);
}

static void test_m5_astar_uses_rolling_world_coordinates(void)
{
  PlannerFx fx;
  const uint64_t now = 9000000u;
  planner_fx_init(&fx, now);
  fx.navigation.core.position_ned_m[0] = 10.0f;
  fx.map.center_ned_m[0] = 10.0f;
  fx.request.start_position_ned_m[0] = 8.25f;
  fx.request.goal_position_ned_m[0] = 8.75f;
  CHECK(m5_local_map_is_ready(&fx.map, &fx.navigation, now));
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  CHECK(fx.path.segment_count == 1u);
  CHECK(fabsf(fx.path.position_ned_m[0][0] - 8.25f) < 1e-6f);
  CHECK(fabsf(fx.path.position_ned_m[1][0] - 8.75f) < 1e-6f);
}

static void test_m5_astar_unknown_space_and_deadline_fail_closed(void)
{
  PlannerFx fx;
  uint16_t start_voxel, goal_voxel;
  uint8_t state;
  const uint64_t now = 7000000u;
  planner_fx_init(&fx, now);
  for (unsigned i = 0u; i < M5_MAP_CELL_COUNT; ++i) {
    fx.map.cells[i].state = M5_MAP_CELL_UNKNOWN;
    fx.map.cells[i].log_odds = 0;
  }
  CHECK(m5_local_map_world_to_voxel(&fx.map, fx.request.start_position_ned_m, &start_voxel));
  CHECK(m5_local_map_world_to_voxel(&fx.map, fx.request.goal_position_ned_m, &goal_voxel));
  fx.map.cells[start_voxel].state = M5_MAP_CELL_FREE_CONFIRMED;
  fx.map.cells[start_voxel].log_odds = -4;
  fx.map.cells[goal_voxel].state = M5_MAP_CELL_FREE_CONFIRMED;
  fx.map.cells[goal_voxel].log_odds = -4;
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_NO_PATH);
  CHECK(fx.path.segment_count == 0u);
  CHECK(fx.stats.rejected_edges > 0u);
  fx.clock_us = fx.request.planning_deadline_monotonic_us;
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_DEADLINE_MISSED);
  CHECK(fx.path.segment_count == 0u);
  CHECK(!m5_local_map_voxel_state(&fx.map, M5_MAP_CELL_COUNT, now, &state));
}

static void test_m5_astar_rejects_future_stale_map_cells(void)
{
  PlannerFx fx;
  const uint64_t now = 8500000u;
  planner_fx_init(&fx, now);
  fx.map.source_deadline_us = now + 300000u;
  CHECK(m5_local_map_is_ready(&fx.map, &fx.navigation, now));
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) != M5_PLANNER_OK);
  CHECK(fx.path.segment_count == 0u);
  CHECK(fx.stats.rejected_edges > 0u);
}

static void test_m5_astar_rejects_stale_map_and_obstacle_barrier(void)
{
  PlannerFx fx;
  const uint64_t now = 8000000u;
  planner_fx_init(&fx, now);
  fx.map.last_ingest_us = now - M5_MAP_STALE_US - 1u;
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_INPUT_STALE);

  planner_fx_init(&fx, now);
  for (unsigned i = 0u; i < M5_MAP_CELL_COUNT; ++i) {
    fx.map.cells[i].state = M5_MAP_CELL_UNKNOWN;
    fx.map.cells[i].log_odds = 0;
  }
  /* A narrow known corridor is cut by a full occupied voxel plane. */
  for (unsigned z = 3u; z <= 5u; ++z)
    for (unsigned y = 3u; y <= 5u; ++y)
      for (unsigned x = 3u; x <= 7u; ++x) {
        uint16_t voxel = (uint16_t)((z * 16u + y) * 16u + x);
        fx.map.cells[voxel].state = M5_MAP_CELL_FREE_CONFIRMED;
        fx.map.cells[voxel].log_odds = -4;
        fx.map.cells[voxel].last_update_ms = (uint32_t)(now / 1000u);
      }
  for (unsigned z = 3u; z <= 5u; ++z)
    for (unsigned y = 3u; y <= 5u; ++y) {
      uint16_t voxel = (uint16_t)((z * 16u + y) * 16u + 5u);
      fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
      fx.map.cells[voxel].log_odds = 4;
    }
  fx.request.goal_position_ned_m[0] = -0.75f;
  {
    M5PlannerResult result = m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
        &fx.path, &fx.stats);
    CHECK(result != M5_PLANNER_OK);
    CHECK(result == M5_PLANNER_NO_PATH || result == M5_PLANNER_CAPACITY_EXCEEDED);
  }
  CHECK(fx.path.segment_count == 0u);
}



static void test_m5_smoother_builds_single_edge_candidate(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5TimeScaledPath timed;
  M5SmoothedTrajectory output;
  const uint64_t now = 10000000u;
  planner_fx_init(&fx, now);
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 1u);
  CHECK(output.fallback_used == 0u);
  CHECK(output.cost_valid == 1u);
  CHECK(output.map_revision == fx.map.map_revision);
  CHECK(output.total_duration_us == timed.path.offset_us[1]);
  CHECK(output.segment_duration_us[0] == timed.path.offset_us[1]);
  CHECK(isfinite(output.initial_cost) && isfinite(output.optimized_cost));
  CHECK(output.optimized_cost <= output.initial_cost + 1e-6);
  CHECK(fabsf(guidance_quintic_eval(&output.position[0][0], 0.0f, 0u) -
              fx.path.position_ned_m[0][0]) < 1e-5f);
  CHECK(fabsf(guidance_quintic_eval(&output.position[0][0],
              (float)timed.path.offset_us[1] * 1e-6f, 0u) -
              fx.path.position_ned_m[1][0]) < 1e-4f);
  for (unsigned i = 0u; i < 6u; ++i) CHECK(isfinite(output.yaw[0].c[i]));
}

static void test_m5_smoother_optimizes_two_segment_path_continuously(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5TimeScaledPath timed;
  M5SmoothedTrajectory output;
  const uint64_t now = 11000000u;
  float left_velocity, right_velocity, left_acceleration, right_acceleration;
  planner_fx_init(&fx, now);
  memset(&fx.path, 0, sizeof(fx.path));
  fx.path.segment_count = 2u;
  fx.path.map_revision = fx.map.map_revision;
  fx.path.total_cost_s = 0.8f;
  fx.path.offset_us[1] = 400000u;
  fx.path.offset_us[2] = 800000u;
  fx.path.position_ned_m[0][0] = -1.75f;
  fx.path.position_ned_m[1][0] = -1.25f;
  fx.path.position_ned_m[2][0] = -0.75f;
  fx.path.velocity_ned_mps[0][0] = 1.0f;
  fx.path.velocity_ned_mps[1][0] = 0.5f;
  fx.path.velocity_ned_mps[2][0] = 0.0f;
  fx.path.primitive_index[0] = 1u;
  fx.path.primitive_index[1] = 1u;
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 2u && output.fallback_used == 0u);
  CHECK(output.cost_valid == 1u);
  CHECK(output.total_duration_us == timed.path.offset_us[2]);
  CHECK(output.segment_duration_us[0] ==
        timed.path.offset_us[1] - timed.path.offset_us[0]);
  CHECK(output.segment_duration_us[1] ==
        timed.path.offset_us[2] - timed.path.offset_us[1]);
  CHECK(output.optimized_cost < output.initial_cost);
  {
    float first_duration_s = (float)output.segment_duration_us[0] * 1e-6f;
    CHECK(fabs(guidance_quintic_eval(&output.position[0][0],
                first_duration_s, 0u) + 1.25) < 1e-5);
    CHECK(fabs(guidance_quintic_eval(&output.position[1][0], 0.0f, 0u) + 1.25) < 1e-5);
    left_velocity = guidance_quintic_eval(&output.position[0][0], first_duration_s, 1u);
    right_velocity = guidance_quintic_eval(&output.position[1][0], 0.0f, 1u);
    left_acceleration = guidance_quintic_eval(&output.position[0][0], first_duration_s, 2u);
    right_acceleration = guidance_quintic_eval(&output.position[1][0], 0.0f, 2u);
  }
  CHECK(fabsf(left_velocity - right_velocity) < 1e-4f);
  CHECK(fabsf(left_acceleration - right_acceleration) < 1e-3f);
}

static void test_m5_smoother_rejects_stale_and_mismatched_paths(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5TimeScaledPath timed;
  M5SmoothedTrajectory output;
  const uint64_t now = 12000000u;
  planner_fx_init(&fx, now);
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  CHECK(m5_time_scaler_build(&fx.path, &fx.config, &fx.envelope,
      &fx.navigation, now, &timed) == M5_TIME_SCALER_OK);
  fx.map.map_revision++;
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_INPUT_STALE);
  CHECK(output.segment_count == 0u);
  fx.map.map_revision = timed.path.map_revision;
  timed.path.offset_us[1] += 100000u;
  timed.path.total_cost_s += 0.1f;
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_CAPACITY_EXCEEDED);
  CHECK(output.segment_count == 0u);
  timed.path.segment_count = (uint8_t)(M5_SMOOTHER_MAX_SEGMENTS + 1u);
  CHECK(m5_path_smoother_build(&timed, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_CAPACITY_EXCEEDED);
  CHECK(output.segment_count == 0u);
}



typedef struct {
  Fx base;
  M5LocalMap map;
  MotionReference reference;
  ProductGeofenceSnapshot fence;
  M5TrajectorySafetyInput input;
  M5TrajectorySafetyWorkspace workspace;
  M5TrajectorySafetyResult result;
} SafetyFx;

static void safety_point_set(MotionReferencePoint *point,
                             float x, float y, float z,
                             float vx, float vy, float vz)
{
  uint32_t offset_us = point->offset_us;
  memset(point, 0, sizeof(*point));
  point->offset_us = offset_us;
  point->valid_mask = 0x1Fu;
  point->point.valid_mask = 0x1Fu;
  point->point.position_ned_m[0] = x;
  point->point.position_ned_m[1] = y;
  point->point.position_ned_m[2] = z;
  point->point.velocity_ned_mps[0] = vx;
  point->point.velocity_ned_mps[1] = vy;
  point->point.velocity_ned_mps[2] = vz;
}

static void safety_map_reset(SafetyFx *fx)
{
  unsigned i;
  CHECK(m5_local_map_init(&fx->map, &fx->base.nav, fx->base.now,
      0.10f, 0.05f, 1.0f,
      M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  for (i = 0u; i < M5_MAP_CELL_COUNT; ++i) {
    fx->map.cells[i].state = M5_MAP_CELL_FREE_CONFIRMED;
    fx->map.cells[i].log_odds = -4;
    fx->map.cells[i].source_mask = 1u;
    fx->map.cells[i].last_update_ms = (uint32_t)(fx->base.now / 1000u);
  }
  fx->map.last_ingest_us = fx->base.now;
  fx->map.source_deadline_us = fx->base.now + 1000000u;
  fx->input.map = &fx->map;
  fx->input.expected_map_revision = fx->map.map_revision;
}

static void safety_fx_init(SafetyFx *fx, uint64_t now_us)
{
  unsigned i;
  memset(fx, 0, sizeof(*fx));
  fx_defaults(&fx->base);
  fx->base.now = now_us;
  fx->base.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_fill(&fx->base);

  fill_header(&fx->reference.header, MSG_MOTION_REFERENCE, SOURCE_M5, now_us);
  fx->reference.reference_id = 1u;
  fx->reference.reference_epoch = 1u;
  fx->reference.state_epoch = T_STATE_EPOCH;
  fx->reference.origin_id = T_ORIGIN;
  fx->reference.authority_epoch = T_AUTH_EPOCH;
  fx->reference.directive_epoch = T_DIR_EPOCH;
  fx->reference.point_count = PRODUCT_PREVIEW_COUNT;
  fx->reference.reference_kind = GUIDANCE_REFERENCE_TRAJECTORY;
  safety_point_set(&fx->reference.anchor, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx->reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx->reference.points[i], 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f);
  }

  safety_map_reset(fx);
  fx->input.candidate_reference = &fx->reference;
  fx->input.navigation = &fx->base.nav;
  fx->input.directive = &fx->base.dir;
  fx->input.safety_envelope = &fx->base.env;
  fx->input.max_dynamic_obstacle_accel_mps2 = 0.5f;
  fx->input.evaluation_time_us = now_us;
}

static void test_m5_safety_reference_time_epoch_mismatch(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 56000000u);
  fx.reference.header.time_epoch++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_map_time_epoch_mismatch(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 56100000u);
  fx.map.time_epoch++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_directive_frame_mismatch(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 56200000u);
  fx.base.dir.header.frame_id = (uint8_t)(FRAME_LOCAL_NED ^ 1u);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}


static void safety_set_constant_position(SafetyFx *fx,
                                         float x, float y, float z)
{
  unsigned i;
  fx->base.nav.core.position_ned_m[0] = x;
  fx->base.nav.core.position_ned_m[1] = y;
  fx->base.nav.core.position_ned_m[2] = z;
  memset(fx->base.nav.core.velocity_ned_mps, 0,
         sizeof(fx->base.nav.core.velocity_ned_mps));
  memset(fx->base.nav.core.acceleration_ned_mps2, 0,
         sizeof(fx->base.nav.core.acceleration_ned_mps2));
  safety_point_set(&fx->reference.anchor, x, y, z, 0.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx->reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx->reference.points[i], x, y, z,
                     0.0f, 0.0f, 0.0f);
  }
}

static void safety_set_linear_reference(
    SafetyFx *fx,
    float x, float y, float z,
    float vx, float vy, float vz)
{
  unsigned i;
  fx->base.nav.core.position_ned_m[0] = x;
  fx->base.nav.core.position_ned_m[1] = y;
  fx->base.nav.core.position_ned_m[2] = z;
  fx->base.nav.core.velocity_ned_mps[0] = vx;
  fx->base.nav.core.velocity_ned_mps[1] = vy;
  fx->base.nav.core.velocity_ned_mps[2] = vz;
  memset(fx->base.nav.core.acceleration_ned_mps2, 0,
         sizeof(fx->base.nav.core.acceleration_ned_mps2));
  safety_point_set(&fx->reference.anchor, x, y, z, vx, vy, vz);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float t = (float)((double)(i + 1u) *
        (double)PRODUCT_PREVIEW_STEP_US * 1.0e-6);
    fx->reference.points[i].offset_us =
        (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx->reference.points[i],
        x + vx * t, y + vy * t, z + vz * t, vx, vy, vz);
  }
  safety_map_reset(fx);
}

static void safety_set_keep_in_fence(SafetyFx *fx,
                                     float min_height,
                                     float max_height,
                                     float vertical_margin)
{
  fill_header(&fx->fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx->base.now);
  fx->fence.fence_id = 91u;
  fx->fence.frame_id = FRAME_LOCAL_NED;
  fx->fence.origin_id = T_ORIGIN;
  fx->fence.state_epoch = T_STATE_EPOCH;
  fx->fence.validity = PRODUCT_GEOFENCE_VALID;
  fx->fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx->fence.vertex_count = 4u;
  fx->fence.horizontal_margin_m = 0.0f;
  fx->fence.vertical_margin_m = vertical_margin;
  fx->fence.min_height_above_home_m = min_height;
  fx->fence.max_height_above_home_m = max_height;
  fx->fence.vertices_ne_m[0][0] = -2.0f;
  fx->fence.vertices_ne_m[0][1] = -2.0f;
  fx->fence.vertices_ne_m[1][0] = 2.0f;
  fx->fence.vertices_ne_m[1][1] = -2.0f;
  fx->fence.vertices_ne_m[2][0] = 2.0f;
  fx->fence.vertices_ne_m[2][1] = 2.0f;
  fx->fence.vertices_ne_m[3][0] = -2.0f;
  fx->fence.vertices_ne_m[3][1] = 2.0f;
  fx->input.geofences = &fx->fence;
  fx->input.geofence_count = 1u;
}

static uint32_t safety_fuzz_next(uint32_t *state)
{
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static float safety_fuzz_signed_unit(uint32_t *state)
{
  int32_t scaled = (int32_t)(safety_fuzz_next(state) % 1001u) - 500;
  return (float)scaled * 0.0004f;
}

static float safety_fuzz_unit(uint32_t *state)
{
  return (float)(safety_fuzz_next(state) % 10001u) * 0.0001f;
}

static float safety_fuzz_between(uint32_t *state, float low, float high)
{
  return low + (high - low) * safety_fuzz_unit(state);
}

static void safety_reference_quintic(
    const float p0[3], const float v0[3], const float a0[3],
    const float p1[3], const float v1[3], const float a1[3],
    double duration, double time_s,
    double position[3], double velocity[3], double acceleration[3])
{
  double s = time_s / duration;
  double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  double h[6] = {
    1.0 - 10.0*s3 + 15.0*s4 - 6.0*s5,
    s - 6.0*s3 + 8.0*s4 - 3.0*s5,
    0.5*s2 - 1.5*s3 + 1.5*s4 - 0.5*s5,
    10.0*s3 - 15.0*s4 + 6.0*s5,
    -4.0*s3 + 7.0*s4 - 3.0*s5,
    0.5*s3 - s4 + 0.5*s5
  };
  double dh[6] = {
    -30.0*s2 + 60.0*s3 - 30.0*s4,
    1.0 - 18.0*s2 + 32.0*s3 - 15.0*s4,
    s - 4.5*s2 + 6.0*s3 - 2.5*s4,
    30.0*s2 - 60.0*s3 + 30.0*s4,
    -12.0*s2 + 28.0*s3 - 15.0*s4,
    1.5*s2 - 4.0*s3 + 2.5*s4
  };
  double ddh[6] = {
    -60.0*s + 180.0*s2 - 120.0*s3,
    -36.0*s + 96.0*s2 - 60.0*s3,
    1.0 - 9.0*s + 18.0*s2 - 10.0*s3,
    60.0*s - 180.0*s2 + 120.0*s3,
    -24.0*s + 84.0*s2 - 60.0*s3,
    3.0*s - 12.0*s2 + 10.0*s3
  };
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    double terms[6] = {
      p0[axis], duration * v0[axis],
      duration * duration * a0[axis],
      p1[axis], duration * v1[axis],
      duration * duration * a1[axis]
    };
    double p = 0.0, v = 0.0, a = 0.0;
    unsigned k;
    for (k = 0u; k < 6u; ++k) {
      p += terms[k] * h[k];
      v += terms[k] * dh[k];
      a += terms[k] * ddh[k];
    }
    position[axis] = p;
    velocity[axis] = v / duration;
    acceleration[axis] = a / (duration * duration);
  }
}

static void test_m5_safety_bernstein_bounds_randomized(void)
{
  uint32_t seed = 0xB37A1155u;
  unsigned trial;

  for (trial = 0u; trial < 256u; ++trial) {
    float p0[3], v0[3], a0[3], p1[3], v1[3], a1[3];
    float position_low[3], position_high[3];
    float speed_bound = 0.0f, acceleration_bound = 0.0f;
    float duration = safety_fuzz_between(&seed, 0.02f, 1.5f);
    float u0 = safety_fuzz_between(&seed, 0.0f, 0.75f);
    float u1 = safety_fuzz_between(&seed, u0 + 0.05f, 1.0f);
    unsigned axis, sample;

    for (axis = 0u; axis < 3u; ++axis) {
      p0[axis] = safety_fuzz_between(&seed, -2.0f, 2.0f);
      p1[axis] = safety_fuzz_between(&seed, -2.0f, 2.0f);
      v0[axis] = safety_fuzz_between(&seed, -3.0f, 3.0f);
      v1[axis] = safety_fuzz_between(&seed, -3.0f, 3.0f);
      a0[axis] = safety_fuzz_between(&seed, -8.0f, 8.0f);
      a1[axis] = safety_fuzz_between(&seed, -8.0f, 8.0f);
    }

    CHECK(m5_trajectory_safety_test_curve_bounds(
        p0, v0, a0, p1, v1, a1, duration, u0, u1,
        position_low, position_high, &speed_bound, &acceleration_bound));

    for (sample = 0u; sample <= 64u; ++sample) {
      double fraction = (double)sample / 64.0;
      double t = (double)duration *
          ((double)u0 + ((double)u1 - (double)u0) * fraction);
      double position[3], velocity[3], acceleration[3];
      double speed2 = 0.0, acceleration2 = 0.0;
      safety_reference_quintic(p0, v0, a0, p1, v1, a1,
          (double)duration, t, position, velocity, acceleration);
      for (axis = 0u; axis < 3u; ++axis) {
        CHECK(position[axis] >= (double)position_low[axis]);
        CHECK(position[axis] <= (double)position_high[axis]);
        speed2 += velocity[axis] * velocity[axis];
        acceleration2 += acceleration[axis] * acceleration[axis];
      }
      CHECK(sqrt(speed2) <= (double)speed_bound);
      CHECK(sqrt(acceleration2) <= (double)acceleration_bound);

      if ((sample & 1u) == 0u) {
        float evaluated_position[3], evaluated_velocity[3];
        float evaluated_acceleration[3];
        CHECK(m5_trajectory_safety_test_curve_eval(
            p0, v0, a0, p1, v1, a1, duration, u0, u1,
            (float)fraction, evaluated_position, evaluated_velocity,
            evaluated_acceleration));
        for (axis = 0u; axis < 3u; ++axis) {
          CHECK(fabs((double)evaluated_position[axis] - position[axis]) <= 0.0001);
          CHECK(fabs((double)evaluated_velocity[axis] - velocity[axis]) <= 0.001);
          CHECK(fabs((double)evaluated_acceleration[axis] -
                     acceleration[axis]) <= 0.01);
        }
      }
    }
  }
}

static void test_m5_safety_bernstein_zero_curve_enclosure(void)
{
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  float low[3], high[3], speed, acceleration;
  unsigned axis;

  CHECK(m5_trajectory_safety_test_curve_bounds(
      zero, zero, zero, zero, zero, zero, 0.02f, 0.0f, 1.0f,
      low, high, &speed, &acceleration));
  for (axis = 0u; axis < 3u; ++axis) {
    CHECK(low[axis] <= 0.0f);
    CHECK(high[axis] >= 0.0f);
  }
  CHECK(speed >= 0.0f);
  CHECK(acceleration >= 0.0f);
}

static void test_m5_safety_bernstein_bounds_invalid_inputs(void)
{
  const float p0[3] = {0.0f, 0.0f, 0.0f};
  const float v0[3] = {0.0f, 0.0f, 0.0f};
  const float a0[3] = {0.0f, 0.0f, 0.0f};
  float low[3], high[3], speed, acceleration;
  CHECK(!m5_trajectory_safety_test_curve_bounds(
      p0, v0, a0, p0, v0, a0, 1.0f, 0.5f, 0.5f,
      low, high, &speed, &acceleration));
  CHECK(!m5_trajectory_safety_test_curve_bounds(
      p0, v0, a0, p0, v0, a0, 0.0f, 0.0f, 1.0f,
      low, high, &speed, &acceleration));
  CHECK(!m5_trajectory_safety_test_curve_bounds(
      p0, v0, a0, p0, v0, a0, 1.0f, -0.1f, 0.5f,
      low, high, &speed, &acceleration));
  CHECK(!m5_trajectory_safety_test_curve_bounds(
      p0, v0, a0, p0, v0, a0, 1.0f, 0.0f, 1.1f,
      low, high, &speed, &acceleration));
  {
    float position[3], velocity[3], acceleration_values[3];
    CHECK(!m5_trajectory_safety_test_curve_eval(
        p0, v0, a0, p0, v0, a0, 1.0f, 0.0f, 1.0f, -0.1f,
        position, velocity, acceleration_values));
    CHECK(!m5_trajectory_safety_test_curve_eval(
        p0, v0, a0, p0, v0, a0, 1.0f, 0.0f, 1.0f, 1.1f,
        position, velocity, acceleration_values));
  }
}

static void test_m5_safety_dynamic_prediction_time_boundaries(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;

  /* A track exactly at the maximum age remains admissible if its lease
     still covers the complete preview proof horizon. */
  safety_fx_init(&fx, 57000000u);
  track = &fx.map.tracks[0];
  track->id = 51u;
  track->source_id = 2u;
  track->sample_us = fx.base.now - M5_MAP_STALE_US;
  track->expires_us = track->sample_us + M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  track->position_ned_m[0] = 3.0f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);

  /* One microsecond beyond the stale-age boundary must fail closed. */
  track->sample_us--;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);

  /* The prediction lease may end exactly at sample + configured horizon. */
  safety_fx_init(&fx, 57100000u);
  track = &fx.map.tracks[0];
  track->id = 52u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = track->sample_us +
      M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  track->position_ned_m[0] = 3.0f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  track->expires_us++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);

  /* A map that advertises less than the required prediction horizon is
     invalid even when no track currently occupies the map. */
  safety_fx_init(&fx, 57200000u);
  fx.map.max_dynamic_prediction_us =
      M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US - 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_geofence_height_margins(void)
{
  SafetyFx fx;
  M5TrajectorySafetyStatus status;

  /* Both lower and upper limits include the approved vertical margin. */
  safety_fx_init(&fx, 57300000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -0.31f);
  safety_map_reset(&fx);
  safety_set_keep_in_fence(&fx, 0.0f, 2.0f, 0.15f);
  CHECK(fence_geometry_valid(&fx.fence));
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);

  safety_fx_init(&fx, 57400000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -0.29f);
  safety_map_reset(&fx);
  safety_set_keep_in_fence(&fx, 0.0f, 2.0f, 0.15f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status != M5_SAFETY_PASS);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 57500000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -1.69f);
  safety_map_reset(&fx);
  safety_set_keep_in_fence(&fx, 0.0f, 2.0f, 0.15f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);

  safety_fx_init(&fx, 57600000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -1.71f);
  safety_map_reset(&fx);
  safety_set_keep_in_fence(&fx, 0.0f, 2.0f, 0.15f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status != M5_SAFETY_PASS);
  CHECK(fx.result.passed == 0u);

  /* Exact mathematical boundary is not assumed safe: outward rounding may
     conservatively reject it, but it must never be reported as PASS. */
  safety_fx_init(&fx, 57700000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -0.15f);
  safety_map_reset(&fx);
  safety_set_keep_in_fence(&fx, 0.0f, 2.0f, 0.15f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status != M5_SAFETY_PASS);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 57800000u);
  safety_set_keep_in_fence(&fx, 1.0f, 1.0f, 0.0f);
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_preview_time_overflow_rejected(void)
{
  SafetyFx fx;
  uint64_t reference_time = UINT64_MAX - 400000u;
  uint64_t evaluation_time = reference_time + 1000u;
  unsigned i;

  safety_fx_init(&fx, 57900000u);
  fx.base.now = evaluation_time;
  fx.input.evaluation_time_us = evaluation_time;

  fx.base.nav.header.timestamp_sample_us = reference_time;
  fx.base.nav.header.timestamp_publish_us = evaluation_time;
  fx.base.nav.header.valid_until_us = UINT64_MAX;

  fx.reference.header.timestamp_sample_us = reference_time;
  fx.reference.header.timestamp_publish_us = evaluation_time;
  fx.reference.header.valid_until_us = UINT64_MAX;

  fx.base.dir.header.timestamp_sample_us = evaluation_time;
  fx.base.dir.header.timestamp_publish_us = evaluation_time;
  fx.base.dir.header.valid_until_us = UINT64_MAX;
  fx.base.dir.directive_expires_at_us = UINT64_MAX;

  fx.base.env.header.timestamp_sample_us = evaluation_time;
  fx.base.env.header.timestamp_publish_us = evaluation_time;
  fx.base.env.header.valid_until_us = UINT64_MAX;

  fx.map.last_ingest_us = evaluation_time;
  fx.map.source_deadline_us = UINT64_MAX;
  for (i = 0u; i < M5_MAP_CELL_COUNT; ++i)
    fx.map.cells[i].last_update_ms = (uint32_t)(evaluation_time / 1000u);

  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_finite_numeric_extreme_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;

  safety_fx_init(&fx, 58000000u);
  track = &fx.map.tracks[0];
  track->id = 53u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  track->position_ned_m[0] = 3.0f;
  track->velocity_ned_mps[0] = 0.0f;
  track->position_std_m = 3.4e38f;
  track->velocity_std_mps = 3.0e38f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_NUMERIC_FAILURE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_deterministic_property_fuzz(void)
{
  SafetyFx fx;
  uint32_t seed = 0x6D350607u;
  unsigned i;

  for (i = 0u; i < 32u; ++i) {
    M5DynamicTrack *track;
    M5TrajectorySafetyStatus status;
    unsigned bad_index;
    uint64_t now = 59000000ull + (uint64_t)i * 1000000ull;
    float x = safety_fuzz_signed_unit(&seed);
    float y = safety_fuzz_signed_unit(&seed);
    float z = -(0.05f + (float)(safety_fuzz_next(&seed) % 151u) * 0.001f);

    safety_fx_init(&fx, now);
    safety_set_constant_position(&fx, x, y, z);
    track = &fx.map.tracks[0];
    track->id = 100u + i;
    track->source_id = 2u;
    track->sample_us = now;
    track->expires_us = now + M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
    track->position_ned_m[0] = 3.0f + safety_fuzz_signed_unit(&seed) * 0.2f;
    track->position_ned_m[1] = 3.0f + safety_fuzz_signed_unit(&seed) * 0.2f;
    track->position_ned_m[2] = 3.0f;
    track->velocity_ned_mps[0] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->velocity_ned_mps[1] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->velocity_ned_mps[2] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->radius_m = 0.05f;
    track->position_std_m = 0.01f;
    track->velocity_std_mps = 0.01f;

    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    CHECK(status == M5_SAFETY_PASS);
    CHECK(fx.result.passed == 1u);
    CHECK(fx.result.reason_code == M5_SAFETY_REASON_NONE);
    CHECK(fx.result.valid_until_us >= now +
        PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US);
    CHECK(fx.result.work_units <= M5_SAFETY_MAX_WORK_UNITS);
    CHECK(fx.result.cell_checks <= M5_SAFETY_MAX_CELL_CHECKS);
    CHECK(fx.result.track_checks > 0u);

    bad_index = (unsigned)(safety_fuzz_next(&seed) % PRODUCT_PREVIEW_COUNT);
    switch (i % 4u) {
      case 0u:
        track->velocity_std_mps = NAN;
        break;
      case 1u:
        fx.reference.points[bad_index].offset_us++;
        break;
      case 2u:
        fx.map.time_epoch++;
        break;
      default:
        track->sample_us = now + 1u;
        break;
    }
    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    CHECK(status != M5_SAFETY_PASS);
    CHECK(fx.result.passed == 0u);
    CHECK(fx.result.status == (uint8_t)status);
  }
}

static double safety_reference_polygon_clearance(
    const ProductGeofenceSnapshot *fence, double x, double y)
{
  double best = INFINITY;
  int winding = 0;
  unsigned i;

  for (i = 0u; i < fence->vertex_count; ++i) {
    unsigned next = (i + 1u) % fence->vertex_count;
    double ax = fence->vertices_ne_m[i][0];
    double ay = fence->vertices_ne_m[i][1];
    double bx = fence->vertices_ne_m[next][0];
    double by = fence->vertices_ne_m[next][1];
    double dx = bx - ax, dy = by - ay;
    double length2 = dx * dx + dy * dy;
    double u = length2 > 0.0 ?
        (((x - ax) * dx + (y - ay) * dy) / length2) : 0.0;
    double ex, ey, distance, side;

    u = fmax(0.0, fmin(1.0, u));
    ex = x - (ax + u * dx);
    ey = y - (ay + u * dy);
    distance = sqrt(ex * ex + ey * ey);
    if (distance < best) best = distance;

    /* Winding-number classification is independent of production ray parity. */
    side = dx * (y - ay) - dy * (x - ax);
    if (ay <= y) {
      if (by > y && side > 0.0) ++winding;
    } else if (by <= y && side < 0.0) {
      --winding;
    }
  }

  {
    double signed_clearance = (winding != 0) ? best : -best;
    if (fence->kind == PRODUCT_GEOFENCE_KEEP_OUT)
      signed_clearance = -signed_clearance;
    return signed_clearance - fence->horizontal_margin_m;
  }
}

static void safety_geometry_fixture(
    ProductGeofenceSnapshot *fence,
    const float vertices[][2],
    uint8_t vertex_count,
    uint8_t kind,
    float margin)
{
  unsigned i;
  memset(fence, 0, sizeof(*fence));
  fence->validity = PRODUCT_GEOFENCE_VALID;
  fence->frame_id = FRAME_LOCAL_NED;
  fence->kind = kind;
  fence->vertex_count = vertex_count;
  fence->horizontal_margin_m = margin;
  fence->vertical_margin_m = 0.10f;
  fence->min_height_above_home_m = 0.0f;
  fence->max_height_above_home_m = 10.0f;
  for (i = 0u; i < vertex_count; ++i) {
    fence->vertices_ne_m[i][0] = vertices[i][0];
    fence->vertices_ne_m[i][1] = vertices[i][1];
  }
}

static void safety_check_geometry_polygon(
    const float vertices[][2],
    uint8_t vertex_count,
    uint32_t *seed)
{
  ProductGeofenceSnapshot fence;
  unsigned kind, i;

  for (kind = PRODUCT_GEOFENCE_KEEP_IN;
       kind <= PRODUCT_GEOFENCE_KEEP_OUT; ++kind) {
    safety_geometry_fixture(&fence, vertices, vertex_count,
                            (uint8_t)kind, 0.12f);
    CHECK(fence_geometry_valid(&fence));

    for (i = 0u; i < 128u; ++i) {
      float x = safety_fuzz_signed_unit(seed) * 15.0f;
      float y = safety_fuzz_signed_unit(seed) * 15.0f;
      float actual = fence_clearance(&fence, x, y);
      float expected = (float)safety_reference_polygon_clearance(
          &fence, (double)x, (double)y);
      CHECK(isfinite(actual));
      CHECK(fabsf(actual - expected) <= 0.00002f);
    }

    /* Exercise every vertex, every collinear edge midpoint, and both sides
       of each edge at a small but representable distance. */
    for (i = 0u; i < vertex_count; ++i) {
      unsigned next = (i + 1u) % vertex_count;
      double ax = vertices[i][0], ay = vertices[i][1];
      double bx = vertices[next][0], by = vertices[next][1];
      double dx = bx - ax, dy = by - ay;
      double length = sqrt(dx * dx + dy * dy);
      double nx = (length > 0.0) ? -dy / length : 0.0;
      double ny = (length > 0.0) ? dx / length : 0.0;
      double samples[3][2] = {
        {ax, ay},
        {0.5 * (ax + bx), 0.5 * (ay + by)},
        {0.5 * (ax + bx) + nx * 0.0001,
         0.5 * (ay + by) + ny * 0.0001}
      };
      unsigned sample;
      for (sample = 0u; sample < 3u; ++sample) {
        float actual = fence_clearance(&fence,
            (float)samples[sample][0], (float)samples[sample][1]);
        float expected = (float)safety_reference_polygon_clearance(
            &fence, samples[sample][0], samples[sample][1]);
        CHECK(isfinite(actual));
        CHECK(fabsf(actual - expected) <= 0.00002f);
      }
    }
  }
}

static void test_m5_safety_geometry_clearance_reference(void)
{
  static const float concave_l[6][2] = {
    {-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, -1.0f},
    {0.0f, -1.0f}, {0.0f, 2.0f}, {-2.0f, 2.0f}
  };
  static const float concave_c[8][2] = {
    {-2.0f, -2.0f}, {2.0f, -2.0f}, {2.0f, -1.0f},
    {-1.0f, -1.0f}, {-1.0f, 1.0f}, {2.0f, 1.0f},
    {2.0f, 2.0f}, {-2.0f, 2.0f}
  };
  static const float collinear_square[5][2] = {
    {-2.0f, -2.0f}, {0.0f, -2.0f}, {2.0f, -2.0f},
    {2.0f, 2.0f}, {-2.0f, 2.0f}
  };
  static const float near_collinear[5][2] = {
    {-2.0f, -2.0f}, {0.0f, -2.0f}, {2.0f, -1.9999f},
    {2.0f, 2.0f}, {-2.0f, 2.0f}
  };
  static const float concave_l_clockwise[6][2] = {
    {-2.0f, 2.0f}, {0.0f, 2.0f}, {0.0f, -1.0f},
    {2.0f, -1.0f}, {2.0f, -2.0f}, {-2.0f, -2.0f}
  };
  uint32_t seed = 0x0640C1EAu;

  safety_check_geometry_polygon(concave_l, 6u, &seed);
  safety_check_geometry_polygon(concave_c, 8u, &seed);
  safety_check_geometry_polygon(collinear_square, 5u, &seed);
  safety_check_geometry_polygon(near_collinear, 5u, &seed);
  safety_check_geometry_polygon(concave_l_clockwise, 6u, &seed);
}

static void safety_build_radial_concave_polygon(
    ProductGeofenceSnapshot *fence, uint32_t *seed)
{
  unsigned i;
  const double two_pi = 6.2831853071795864769;
  fence->vertex_count = 8u;
  for (i = 0u; i < 8u; ++i) {
    float radius = ((i & 1u) != 0u ? 1.25f : 2.25f) +
        safety_fuzz_signed_unit(seed) * 0.15f;
    double angle = two_pi * (double)i / 8.0;
    fence->vertices_ne_m[i][0] = radius * (float)cos(angle);
    fence->vertices_ne_m[i][1] = radius * (float)sin(angle);
  }
}

static void test_m5_safety_geofence_adversarial_vertices_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 64500000u);
  safety_set_keep_in_fence(&fx, 0.0f, 4.0f, 0.10f);
  fx.fence.vertex_count = 5u;
  fx.fence.vertices_ne_m[0][0] = -2.0f;
  fx.fence.vertices_ne_m[0][1] = -2.0f;
  fx.fence.vertices_ne_m[1][0] = 2.0f;
  fx.fence.vertices_ne_m[1][1] = -2.0f;
  fx.fence.vertices_ne_m[2][0] = 1.0f;
  fx.fence.vertices_ne_m[2][1] = -2.0f;
  fx.fence.vertices_ne_m[3][0] = 2.0f;
  fx.fence.vertices_ne_m[3][1] = 2.0f;
  fx.fence.vertices_ne_m[4][0] = -2.0f;
  fx.fence.vertices_ne_m[4][1] = 2.0f;
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  fx.fence.vertices_ne_m[0][0] = -2.0f;
  fx.fence.vertices_ne_m[0][1] = -2.0f;
  fx.fence.vertices_ne_m[1][0] = 2.0f;
  fx.fence.vertices_ne_m[1][1] = -2.0f;
  fx.fence.vertices_ne_m[2][0] = 2.0f;
  fx.fence.vertices_ne_m[2][1] = 2.0f;
  fx.fence.vertices_ne_m[3][0] = -2.0f;
  fx.fence.vertices_ne_m[3][1] = 2.0f;
  fx.fence.vertices_ne_m[4][0] = 2.0f;
  fx.fence.vertices_ne_m[4][1] = -2.0f;
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_random_polygon_dynamic_track_fuzz(void)
{
  SafetyFx fx;
  uint32_t seed = 0x65A17F03u;
  unsigned i, sample;

  for (i = 0u; i < 16u; ++i) {
    M5DynamicTrack *track;
    M5TrajectorySafetyStatus status;
    uint64_t now = 65000000ull + (uint64_t)i * 1000000ull;

    safety_fx_init(&fx, now);
    safety_set_constant_position(&fx, 0.0f, 0.0f, -1.0f);
    safety_map_reset(&fx);
    safety_set_keep_in_fence(&fx, 0.0f, 4.0f, 0.10f);
    fx.fence.horizontal_margin_m = 0.10f;
    safety_build_radial_concave_polygon(&fx.fence, &seed);
    CHECK(fence_geometry_valid(&fx.fence));

    /* Differentially check random concave geometry before feeding it to the
       full trajectory validator. */
    for (sample = 0u; sample < 32u; ++sample) {
      float x = safety_fuzz_signed_unit(&seed) * 15.0f;
      float y = safety_fuzz_signed_unit(&seed) * 15.0f;
      float actual = fence_clearance(&fx.fence, x, y);
      float expected = (float)safety_reference_polygon_clearance(
          &fx.fence, (double)x, (double)y);
      CHECK(isfinite(actual));
      CHECK(fabsf(actual - expected) <= 0.00002f);
    }

    track = &fx.map.tracks[0];
    track->id = 200u + i;
    track->source_id = 2u;
    track->sample_us = now;
    track->expires_us = now + M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
    track->position_ned_m[0] = 3.0f + safety_fuzz_signed_unit(&seed) * 0.2f;
    track->position_ned_m[1] = 3.0f + safety_fuzz_signed_unit(&seed) * 0.2f;
    track->position_ned_m[2] = 3.0f;
    track->velocity_ned_mps[0] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->velocity_ned_mps[1] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->velocity_ned_mps[2] = safety_fuzz_signed_unit(&seed) * 0.2f;
    track->radius_m = 0.05f;
    track->position_std_m = 0.01f;
    track->velocity_std_mps = 0.01f;

    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    CHECK(status == M5_SAFETY_PASS);
    CHECK(fx.result.passed == 1u);
    CHECK(fx.result.track_checks > 0u);
    CHECK(fx.result.geofence_checks > 0u);
  }
}


static void safety_set_far_dynamic_track(
    M5DynamicTrack *track, uint32_t id, uint64_t now,
    float x, float y, float z, float vx, float vy, float vz)
{
  memset(track, 0, sizeof(*track));
  track->id = id;
  track->source_id = 2u;
  track->sample_us = now;
  track->expires_us = now + M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  track->position_ned_m[0] = x;
  track->position_ned_m[1] = y;
  track->position_ned_m[2] = z;
  track->velocity_ned_mps[0] = vx;
  track->velocity_ned_mps[1] = vy;
  track->velocity_ned_mps[2] = vz;
  track->radius_m = 0.05f;
  track->position_std_m = 0.01f;
  track->velocity_std_mps = 0.01f;
}

static void test_m5_safety_dynamic_prediction_large_epoch_invariance(void)
{
  SafetyFx low_epoch, high_epoch;
  M5DynamicTrack *low_track, *high_track;
  M5TrajectorySafetyStatus low_status, high_status;
  const uint64_t low_now = 180000000ull;
  const uint64_t high_now = UINT64_MAX - 1000000000ull;

  safety_fx_init(&low_epoch, low_now);
  safety_fx_init(&high_epoch, high_now);
  safety_set_constant_position(&low_epoch, 0.0f, 0.0f, 0.0f);
  safety_set_constant_position(&high_epoch, 0.0f, 0.0f, 0.0f);

  low_track = &low_epoch.map.tracks[0];
  high_track = &high_epoch.map.tracks[0];
  safety_set_far_dynamic_track(low_track, 1190u, low_now,
      3.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f);
  safety_set_far_dynamic_track(high_track, 1190u, high_now,
      3.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f);

  low_track->sample_us -= 100000u;
  low_track->expires_us = low_track->sample_us +
      M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  high_track->sample_us -= 100000u;
  high_track->expires_us = high_track->sample_us +
      M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US;
  low_track->position_std_m = high_track->position_std_m = 0.0f;
  low_track->velocity_std_mps = high_track->velocity_std_mps = 0.0f;

  low_status = m5_trajectory_safety_validate(
      &low_epoch.input, &low_epoch.workspace, &low_epoch.result);
  high_status = m5_trajectory_safety_validate(
      &high_epoch.input, &high_epoch.workspace, &high_epoch.result);

  CHECK(low_status == M5_SAFETY_PASS);
  CHECK(high_status == low_status);
  CHECK(low_epoch.result.track_checks == PRODUCT_PREVIEW_COUNT);
  CHECK(high_epoch.result.track_checks == low_epoch.result.track_checks);
  CHECK(high_epoch.result.passed == low_epoch.result.passed);
}


static void test_m5_safety_dynamic_collision_at_adaptive_and_preview_boundaries(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  M5TrajectorySafetyStatus status;

  safety_fx_init(&fx, 86000000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, 0.0f);
  track = &fx.map.tracks[0];
  safety_set_far_dynamic_track(track, 701u, fx.base.now,
      -0.5f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.work_units > 1u);

  safety_fx_init(&fx, 87000000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, 0.0f);
  track = &fx.map.tracks[0];
  safety_set_far_dynamic_track(track, 702u, fx.base.now,
      -2.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.work_units > 1u);
}

static void test_m5_safety_dynamic_continuous_distance_differential(void)
{
  uint32_t seed = 0xD1FF0610u;
  unsigned trial;

  for (trial = 0u; trial < 64u; ++trial) {
    SafetyFx fx;
    M5DynamicTrack *track;
    const double start_x = -0.5;
    const double vehicle_vx = 0.5;
    const double horizon_s =
        (double)PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US * 1.0e-6;
    double obstacle_vx, crossing_time, offset, obstacle_x;
    double relative_x0, relative_vx, closest_time, closest_dx;
    double oracle_min_distance, physical_contact_radius;
    M5TrajectorySafetyStatus status;
    int collision_case = (trial < 32u);

    safety_fx_init(&fx, 120000000ull + (uint64_t)trial * 1000000ull);
    safety_set_linear_reference(&fx, (float)start_x, 0.0f, 0.0f,
                                (float)vehicle_vx, 0.0f, 0.0f);
    obstacle_vx = (double)safety_fuzz_between(&seed, 1.0f, 3.0f);
    crossing_time = (double)safety_fuzz_between(&seed, 0.01f, 0.47f);
    offset = collision_case ?
        (double)safety_fuzz_signed_unit(&seed) * 0.5 :
        (double)(0.75f + fabsf(safety_fuzz_signed_unit(&seed)) * 1.0f);
    obstacle_x = start_x + (vehicle_vx - obstacle_vx) * crossing_time;

    track = &fx.map.tracks[0];
    safety_set_far_dynamic_track(track, 800u + trial, fx.base.now,
        (float)obstacle_x, (float)offset, 0.0f,
        (float)obstacle_vx, 0.0f, 0.0f);
    track->position_std_m = 0.0f;
    track->velocity_std_mps = 0.0f;

    relative_x0 = start_x - (double)track->position_ned_m[0];
    relative_vx = vehicle_vx - (double)track->velocity_ned_mps[0];
    closest_time = (fabs(relative_vx) > 1e-12) ?
        -relative_x0 / relative_vx : 0.0;
    if (closest_time < 0.0) closest_time = 0.0;
    if (closest_time > horizon_s) closest_time = horizon_s;
    closest_dx = relative_x0 + relative_vx * closest_time;
    oracle_min_distance = sqrt(closest_dx * closest_dx +
        (double)track->position_ned_m[1] * track->position_ned_m[1]);
    physical_contact_radius = (double)fx.map.vehicle_radius_m +
        fx.map.clearance_margin_m + track->radius_m;

    CHECK(closest_time > 0.0 && closest_time < horizon_s);
    if (collision_case) {
      CHECK(oracle_min_distance < (long double)physical_contact_radius);
    } else {
      CHECK(oracle_min_distance > 0.70);
    }

    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    if (collision_case) {
      CHECK(status == M5_SAFETY_DYNAMIC_COLLISION);
      CHECK(fx.result.passed == 0u);
    } else {
      CHECK(status == M5_SAFETY_PASS);
      CHECK(fx.result.passed == 1u);
    }
  }
}


static double safety_reference_min_distance_constant_accel_1d(
    double relative_position,
    double relative_velocity,
    double relative_acceleration,
    double lateral_offset,
    double horizon_s)
{
  long double candidates[5];
  unsigned count = 0u, i;
  long double r0 = (long double)relative_position;
  long double v = (long double)relative_velocity;
  long double a = (long double)relative_acceleration;
  long double y = (long double)lateral_offset;
  long double horizon = (long double)horizon_s;
  long double minimum = INFINITY;

  candidates[count++] = 0.0L;
  candidates[count++] = horizon;

  /* Stationary point of the relative x position. */
  if (a != 0.0L) {
    long double stationary = -v / a;
    if (stationary > 0.0L && stationary < horizon)
      candidates[count++] = stationary;
  }

  /* Roots of r0 + v*t + 0.5*a*t^2. At these points the 2-D
     distance reaches its absolute lower bound |lateral_offset|. */
  if (a != 0.0L) {
    long double discriminant = v * v - 2.0L * a * r0;
    if (discriminant >= 0.0L) {
      long double root = sqrtl(discriminant);
      long double t0 = (-v - root) / a;
      long double t1 = (-v + root) / a;
      if (t0 >= 0.0L && t0 <= horizon) candidates[count++] = t0;
      if (t1 >= 0.0L && t1 <= horizon) candidates[count++] = t1;
    }
  } else if (v != 0.0L) {
    long double root = -r0 / v;
    if (root >= 0.0L && root <= horizon) candidates[count++] = root;
  }

  for (i = 0u; i < count; ++i) {
    long double t = candidates[i];
    long double dx = r0 + v * t + 0.5L * a * t * t;
    long double distance = sqrtl(dx * dx + y * y);
    if (distance < minimum) minimum = distance;
  }
  return (double)minimum;
}


static void test_m5_safety_dynamic_constant_acceleration_continuous_oracle(void)
{
  uint32_t seed = 0x6D350681u;
  const double start_x = -0.5;
  const double vehicle_vx = 0.5;
  const double horizon_s =
      (double)PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US * 1.0e-6;
  unsigned trial;

  for (trial = 0u; trial < 64u; ++trial) {
    SafetyFx fx;
    M5DynamicTrack *track;
    double obstacle_vx, obstacle_ax, crossing_time, lateral_offset;
    double obstacle_x, relative_position, relative_velocity;
    double oracle_min_distance, contact_radius;
    M5TrajectorySafetyStatus status;
    int collision_case = (trial < 32u);

    safety_fx_init(&fx, 160000000ull + (uint64_t)trial * 1000000ull);
    safety_set_linear_reference(&fx, (float)start_x, 0.0f, 0.0f,
                                (float)vehicle_vx, 0.0f, 0.0f);
    obstacle_vx = (double)safety_fuzz_between(&seed, 1.0f, 3.0f);
    obstacle_ax = (double)(float)(
        (double)safety_fuzz_signed_unit(&seed) * 0.45);
    crossing_time = (double)safety_fuzz_between(&seed, 0.02f, 0.46f);
    lateral_offset = collision_case ?
        (double)safety_fuzz_signed_unit(&seed) * 0.10 :
        1.10 + fabs((double)safety_fuzz_signed_unit(&seed)) * 0.30;

    obstacle_vx = (double)(float)obstacle_vx;
    obstacle_x = start_x + (vehicle_vx - obstacle_vx) * crossing_time -
        0.5 * obstacle_ax * crossing_time * crossing_time;

    track = &fx.map.tracks[0];
    safety_set_far_dynamic_track(track, 1000u + trial, fx.base.now,
        (float)obstacle_x, (float)lateral_offset, 0.0f,
        (float)obstacle_vx, 0.0f, 0.0f);
    track->position_std_m = 0.0f;
    track->velocity_std_mps = 0.0f;
    fx.input.max_dynamic_obstacle_accel_mps2 = 0.5f;

    /* Recompute the oracle from the exact float values consumed by the
       validator. The relative acceleration is -obstacle_ax because the
       vehicle reference is constant-velocity in this fixture. */
    relative_position = start_x - (double)track->position_ned_m[0];
    relative_velocity = vehicle_vx - (double)track->velocity_ned_mps[0];
    oracle_min_distance = safety_reference_min_distance_constant_accel_1d(
        relative_position, relative_velocity, -obstacle_ax,
        (double)track->position_ned_m[1], horizon_s);
    contact_radius = (double)fx.map.vehicle_radius_m +
        (double)fx.map.clearance_margin_m + (double)track->radius_m;

    if (collision_case) {
      CHECK(oracle_min_distance < contact_radius);
    } else {
      CHECK(oracle_min_distance > 1.0L);
    }

    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    if (collision_case) {
      CHECK(status == M5_SAFETY_DYNAMIC_COLLISION);
      CHECK(fx.result.passed == 0u);
    } else {
      CHECK(status == M5_SAFETY_PASS);
      CHECK(fx.result.passed == 1u);
    }
  }
}


static long double safety_constant_accel_min_distance_1d(
    long double x0,
    long double relative_velocity,
    long double relative_acceleration,
    long double lateral_offset,
    long double horizon_s)
{
  long double candidates[5] = {0.0L, horizon_s, 0.0L, 0.0L, 0.0L};
  long double minimum = INFINITY;
  unsigned count = 2u, i;

  /* d/dt (x(t)^2 + lateral_offset^2) = 2*x(t)*x'(t).
     The exact candidate set is both endpoints, x'(t)=0, and every
     in-horizon root of x(t)=0. */
  if (relative_acceleration != 0.0L) {
    long double stationary =
        -relative_velocity / relative_acceleration;
    if (stationary >= 0.0L && stationary <= horizon_s)
      candidates[count++] = stationary;
  }

  {
    long double quadratic = 0.5L * relative_acceleration;
    if (quadratic == 0.0L) {
      if (relative_velocity != 0.0L) {
        long double root = -x0 / relative_velocity;
        if (root >= 0.0L && root <= horizon_s)
          candidates[count++] = root;
      }
    } else {
      long double discriminant =
          relative_velocity * relative_velocity -
          4.0L * quadratic * x0;
      if (discriminant >= 0.0L) {
        long double root_term = sqrtl(discriminant);
        long double root_a =
            (-relative_velocity - root_term) / (2.0L * quadratic);
        long double root_b =
            (-relative_velocity + root_term) / (2.0L * quadratic);
        if (root_a >= 0.0L && root_a <= horizon_s)
          candidates[count++] = root_a;
        if (root_b >= 0.0L && root_b <= horizon_s)
          candidates[count++] = root_b;
      }
    }
  }

  for (i = 0u; i < count; ++i) {
    long double time_s = candidates[i];
    long double x = x0 + relative_velocity * time_s +
        0.5L * relative_acceleration * time_s * time_s;
    long double distance = sqrtl(x * x +
        lateral_offset * lateral_offset);
    if (distance < minimum) minimum = distance;
  }
  return minimum;
}

static void test_m5_safety_dynamic_acceleration_envelope_differential(void)
{
  uint32_t seed = 0xACCE5076u;
  const double start_x = -0.5;
  const double vehicle_vx = 0.5;
  const double horizon_s =
      (double)PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US * 1.0e-6;
  unsigned trial;

  for (trial = 0u; trial < 64u; ++trial) {
    SafetyFx fx;
    M5DynamicTrack *track;
    double obstacle_vx, obstacle_ax, crossing_time, offset, obstacle_x;
    long double oracle_min_distance;
    double physical_contact_radius;
    M5TrajectorySafetyStatus status;
    int collision_case = (trial < 32u);

    safety_fx_init(&fx, 150000000ull + (uint64_t)trial * 1000000ull);
    safety_set_linear_reference(&fx, (float)start_x, 0.0f, 0.0f,
                                (float)vehicle_vx, 0.0f, 0.0f);
    obstacle_vx = (double)safety_fuzz_between(&seed, 1.0f, 3.0f);
    obstacle_ax = (double)safety_fuzz_signed_unit(&seed) * 0.45;
    crossing_time = (double)safety_fuzz_between(&seed, 0.01f, 0.47f);
    offset = collision_case ?
        (double)safety_fuzz_signed_unit(&seed) * 0.10 :
        (double)(1.20f + fabsf(safety_fuzz_signed_unit(&seed)) * 0.30f);
    obstacle_x = start_x + (vehicle_vx - obstacle_vx) * crossing_time -
        0.5 * obstacle_ax * crossing_time * crossing_time;

    track = &fx.map.tracks[0];
    safety_set_far_dynamic_track(track, 900u + trial, fx.base.now,
        (float)obstacle_x, (float)offset, 0.0f,
        (float)obstacle_vx, 0.0f, 0.0f);
    track->position_std_m = 0.0f;
    track->velocity_std_mps = 0.0f;
    fx.input.max_dynamic_obstacle_accel_mps2 = 0.5f;

    oracle_min_distance = safety_constant_accel_min_distance_1d(
        (long double)start_x - (long double)obstacle_x,
        (long double)vehicle_vx - (long double)obstacle_vx,
        -(long double)obstacle_ax,
        (long double)offset,
        (long double)horizon_s);

    physical_contact_radius = (double)fx.map.vehicle_radius_m +
        (double)fx.map.clearance_margin_m + (double)track->radius_m;
    if (collision_case) {
      CHECK(oracle_min_distance < physical_contact_radius);
    } else {
      CHECK(oracle_min_distance > 1.0);
    }

    status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
    if (collision_case) {
      CHECK(status == M5_SAFETY_DYNAMIC_COLLISION);
      CHECK(fx.result.passed == 0u);
    } else {
      CHECK(status == M5_SAFETY_PASS);
      CHECK(fx.result.passed == 1u);
    }
  }
}

static float safety_test_round_up_float(double value)
{
  float rounded = (float)value;
  if ((double)rounded < value) rounded = nextafterf(rounded, INFINITY);
  return rounded;
}


static void test_m5_safety_dynamic_tangency_and_preview_seam_ulp(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  float inflation, tangent_radius;
  double inflation_exact, combined_exact;
  unsigned seam_case;

  safety_fx_init(&fx, 170000000u);
  safety_set_constant_position(&fx, 0.0f, 0.0f, 0.0f);
  fx.input.response_delay_us = 0u;
  fx.input.max_dynamic_obstacle_accel_mps2 = 1.0e-6f;
  for (unsigned axis = 0u; axis < 3u; ++axis) {
    fx.base.nav.core.position_std_m[axis] = 0.0f;
    fx.base.nav.core.velocity_std_mps[axis] = 0.0f;
  }
  track = &fx.map.tracks[0];
  safety_set_far_dynamic_track(track, 1100u, fx.base.now,
      2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  track->position_std_m = 0.0f;
  track->velocity_std_mps = 0.0f;

  /* Reproduce the documented outward additions for the static hold fixture:
     no navigation uncertainty, no response delay, and zero obstacle speed. */
  inflation_exact = nextafter(
      (double)fx.map.vehicle_radius_m + (double)fx.map.clearance_margin_m,
      INFINITY);
  inflation = safety_test_round_up_float(inflation_exact);
  combined_exact = nextafter((double)track->radius_m + (double)inflation,
                             INFINITY);
  tangent_radius = safety_test_round_up_float(combined_exact);

  track->position_ned_m[0] = tangent_radius;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);

  track->position_ned_m[0] = nextafterf(tangent_radius, 0.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);

  /* One ULP outside may be conservatively rejected because the lower-distance
     proof and bounded acceleration sweep overlap; it must never PASS. */
  track->position_ned_m[0] = nextafterf(tangent_radius, INFINITY);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) != M5_SAFETY_PASS);
  CHECK(fx.result.passed == 0u);

  track->position_ned_m[0] = tangent_radius + 0.01f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);

  /* A dynamic obstacle crosses the stationary vehicle exactly at the 20ms
     preview seam, and one position ULP on either side. All three must reject. */
  for (seam_case = 0u; seam_case < 3u; ++seam_case) {
    float start_x = -2.0f;
    safety_fx_init(&fx, 171000000u + (uint64_t)seam_case * 1000000u);
    safety_set_constant_position(&fx, 0.0f, 0.0f, 0.0f);
    track = &fx.map.tracks[0];
    if (seam_case == 1u) start_x = nextafterf(-2.0f, INFINITY);
    if (seam_case == 2u) start_x = nextafterf(-2.0f, -INFINITY);
    safety_set_far_dynamic_track(track, 1110u + seam_case, fx.base.now,
        start_x, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f);
    track->position_std_m = 0.0f;
    track->velocity_std_mps = 0.0f;
    fx.input.max_dynamic_obstacle_accel_mps2 = 0.5f;
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
        &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
    CHECK(fx.result.passed == 0u);
  }
}


static void test_m5_safety_dynamic_adaptive_split_ulp(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  unsigned boundary_case;

  for (boundary_case = 0u; boundary_case < 3u; ++boundary_case) {
    float obstacle_start_x = -5.0f;
    safety_fx_init(&fx, 172000000u + (uint64_t)boundary_case * 1000000u);
    safety_set_constant_position(&fx, 0.0f, 0.0f, 0.0f);
    /* Isolate adaptive subdivision from response-delay inflation: the parent
       midpoint is far outside the collision radius but within the conservative
       sweep bound; the first child midpoint is the actual collision point. */
    fx.input.response_delay_us = 0u;
    if (boundary_case == 1u)
      obstacle_start_x = nextafterf(-5.0f, INFINITY);
    else if (boundary_case == 2u)
      obstacle_start_x = nextafterf(-5.0f, -INFINITY);

    track = &fx.map.tracks[0];
    safety_set_far_dynamic_track(track, 1120u + boundary_case, fx.base.now,
        obstacle_start_x, 0.0f, 0.0f, 1000.0f, 0.0f, 0.0f);
    track->position_std_m = 0.0f;
    track->velocity_std_mps = 0.0f;
    fx.input.max_dynamic_obstacle_accel_mps2 = 0.5f;

    /* Crossing occurs at the first child-interval midpoint (5ms), while the
       parent midpoint (10ms) is only an unresolved near approach. The exact
       and +/-1 ULP starts must all be found after adaptive subdivision. */
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
        &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
    CHECK(fx.result.passed == 0u);
    /* Initial stack is LIFO and interval zero is processed first. Two work
       items prove that the parent was subdivided and a child found collision. */
    CHECK(fx.result.work_units > 1u);
  }
}


static void test_m5_safety_multi_track_all_slots_and_fail_closed(void)
{
  SafetyFx fx;
  uint32_t seed = 0x8A41D2C7u;
  unsigned trial, i;

  for (trial = 0u; trial < 16u; ++trial) {
    unsigned track_count = 1u + (trial % M5_MAP_MAX_TRACKS);
    uint64_t now = 67000000ull + (uint64_t)trial * 1000000ull;
    safety_fx_init(&fx, now);

    for (i = 0u; i < track_count; ++i) {
      float sx = ((i & 1u) != 0u) ? -1.0f : 1.0f;
      float sy = ((i & 2u) != 0u) ? -1.0f : 1.0f;
      float x = sx * (2.8f + fabsf(safety_fuzz_signed_unit(&seed)) * 0.1f);
      float y = sy * (2.8f + fabsf(safety_fuzz_signed_unit(&seed)) * 0.1f);
      float vx = safety_fuzz_signed_unit(&seed) * 0.2f;
      float vy = safety_fuzz_signed_unit(&seed) * 0.2f;
      safety_set_far_dynamic_track(&fx.map.tracks[i],
          400u + trial * M5_MAP_MAX_TRACKS + i, now,
          x, y, 3.0f, vx, vy, 0.0f);
    }

    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
        &fx.result) == M5_SAFETY_PASS);
    CHECK(fx.result.passed == 1u);
    CHECK(fx.result.track_checks ==
        track_count * PRODUCT_PREVIEW_COUNT);

    /* A single future-dated member invalidates the complete multi-track set. */
    fx.map.tracks[track_count - 1u].sample_us = now + 1u;
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
        &fx.result) == M5_SAFETY_STALE);
    CHECK(fx.result.passed == 0u);
  }

  /* All eight slots are consumed, and duplicate identity must reject the set. */
  safety_fx_init(&fx, 84000000u);
  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i) {
    float sx = ((i & 1u) != 0u) ? -1.0f : 1.0f;
    float sy = ((i & 2u) != 0u) ? -1.0f : 1.0f;
    safety_set_far_dynamic_track(&fx.map.tracks[i], 500u + i,
        fx.base.now, sx * 3.0f, sy * 3.0f, 3.0f, 0.0f, 0.0f, 0.0f);
  }
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.track_checks ==
      M5_MAP_MAX_TRACKS * PRODUCT_PREVIEW_COUNT);
  fx.map.tracks[M5_MAP_MAX_TRACKS - 1u].id = fx.map.tracks[0].id;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  /* One colliding member among seven distant tracks must reject the path. */
  safety_fx_init(&fx, 85000000u);
  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i) {
    float sx = ((i & 1u) != 0u) ? -1.0f : 1.0f;
    float sy = ((i & 2u) != 0u) ? -1.0f : 1.0f;
    safety_set_far_dynamic_track(&fx.map.tracks[i], 600u + i,
        fx.base.now, sx * 3.0f, sy * 3.0f, 3.0f, 0.0f, 0.0f, 0.0f);
  }
  fx.map.tracks[5].position_ned_m[0] = 0.0f;
  fx.map.tracks[5].position_ned_m[1] = 0.0f;
  fx.map.tracks[5].position_ned_m[2] = 0.0f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}


static void test_m5_safety_interval_arithmetic_oracle(void)
{
  uint32_t seed = 0x1A7E2A11u;
  unsigned trial;

  for (trial = 0u; trial < 2048u; ++trial) {
    double a_low = (double)safety_fuzz_between(&seed, -1000.0f, 1000.0f);
    double a_high = a_low +
        0.01 + fabs((double)safety_fuzz_between(&seed, -2.0f, 2.0f));
    double b_low = 0.25 +
        fabs((double)safety_fuzz_between(&seed, -10.0f, 10.0f));
    double b_high = b_low +
        0.01 + fabs((double)safety_fuzz_between(&seed, -2.0f, 2.0f));
    double result_low = 0.0, result_high = 0.0;
    long double exact_low = 0.0L, exact_high = 0.0L;
    uint8_t operation = (uint8_t)(trial & 3u);
    unsigned i;
    long double values[4];

    if ((trial & 4u) != 0u) {
      double old_low = b_low;
      b_low = -b_high;
      b_high = -old_low;
    }

    CHECK(m5_trajectory_safety_test_interval_op(operation,
        a_low, a_high, b_low, b_high, &result_low, &result_high));

    if (operation == 0u) {
      exact_low = (long double)a_low + (long double)b_low;
      exact_high = (long double)a_high + (long double)b_high;
    } else if (operation == 1u) {
      exact_low = (long double)a_low - (long double)b_high;
      exact_high = (long double)a_high - (long double)b_low;
    } else if (operation == 2u) {
      values[0] = (long double)a_low * (long double)b_low;
      values[1] = (long double)a_low * (long double)b_high;
      values[2] = (long double)a_high * (long double)b_low;
      values[3] = (long double)a_high * (long double)b_high;
      exact_low = values[0];
      exact_high = values[0];
      for (i = 1u; i < 4u; ++i) {
        if (values[i] < exact_low) exact_low = values[i];
        if (values[i] > exact_high) exact_high = values[i];
      }
    } else {
      values[0] = (long double)a_low / (long double)b_low;
      values[1] = (long double)a_low / (long double)b_high;
      values[2] = (long double)a_high / (long double)b_low;
      values[3] = (long double)a_high / (long double)b_high;
      exact_low = values[0];
      exact_high = values[0];
      for (i = 1u; i < 4u; ++i) {
        if (values[i] < exact_low) exact_low = values[i];
        if (values[i] > exact_high) exact_high = values[i];
      }
    }
    CHECK((long double)result_low <= exact_low);
    CHECK((long double)result_high >= exact_high);
  }

  {
    const double tiny = nextafter(0.0, 1.0);
    double low = 0.0, high = 0.0;
    CHECK(m5_trajectory_safety_test_interval_op(
        2u, tiny, tiny, tiny, tiny, &low, &high));
    CHECK((long double)low <= (long double)tiny * (long double)tiny);
    CHECK((long double)high >= (long double)tiny * (long double)tiny);
    CHECK(!m5_trajectory_safety_test_interval_op(
        3u, 1.0, 2.0, -1.0, 1.0, &low, &high));
    CHECK(!m5_trajectory_safety_test_interval_op(
        0u, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, &low, &high));
    CHECK(!m5_trajectory_safety_test_interval_op(
        3u, 1.0, 1.0, tiny, tiny, &low, &high));
    CHECK(!m5_trajectory_safety_test_interval_op(
        4u, 0.0, 1.0, 0.0, 1.0, &low, &high));
    CHECK(!m5_trajectory_safety_test_interval_op(
        0u, 2.0, 1.0, 0.0, 1.0, &low, &high));
  }
}

static void test_m5_safety_outward_rounding_and_norm_lower(void)
{
  uint32_t seed = 0xD15A6E71u;
  unsigned i;

  for (i = 0u; i < 1024u; ++i) {
    int exponent = -151 + (int)(safety_fuzz_next(&seed) % 279u);
    double mantissa = 0.5 + 0.5 * (double)safety_fuzz_unit(&seed);
    double sign = ((safety_fuzz_next(&seed) & 1u) != 0u) ? -1.0 : 1.0;
    double value = sign * ldexp(mantissa, exponent);
    float lower = 0.0f, upper = 0.0f;
    CHECK(m5_trajectory_safety_test_rounding_bounds(value, &lower, &upper));
    CHECK((double)lower <= value);
    CHECK((double)upper >= value);
    CHECK(lower <= upper);
  }

  for (i = 0u; i < 512u; ++i) {
    double component[3];
    long double exact;
    float lower;
    unsigned axis;
    for (axis = 0u; axis < 3u; ++axis) {
      int exponent = -149 + (int)(safety_fuzz_next(&seed) % 277u);
      double mantissa = 0.5 + 0.5 * (double)safety_fuzz_unit(&seed);
      double sign = ((safety_fuzz_next(&seed) & 1u) != 0u) ? -1.0 : 1.0;
      component[axis] = sign * ldexp(mantissa, exponent);
    }
    exact = sqrtl((long double)component[0] * component[0] +
                  (long double)component[1] * component[1] +
                  (long double)component[2] * component[2]);
    lower = m5_trajectory_safety_test_norm_lower_bound(
        component[0], component[1], component[2]);
    CHECK(isfinite(lower));
    CHECK(lower >= 0.0f);
    CHECK((long double)lower <= exact);
  }

  {
    float lower = 0.0f, upper = 0.0f;
    CHECK(!m5_trajectory_safety_test_rounding_bounds(NAN, &lower, &upper));
    CHECK(!m5_trajectory_safety_test_rounding_bounds(INFINITY, &lower, &upper));
    CHECK(!m5_trajectory_safety_test_rounding_bounds(1.0e300, &lower, &upper));
    CHECK(isfinite(m5_trajectory_safety_test_norm_lower_bound(
        1.0e300, -1.0e300, 1.0e300)));
  }
}


static void test_m5_safety_norm_ulp_boundaries(void)
{
  uint32_t seed = 0x6D350680u;
  const float tiny = ldexpf(1.0f, -80);
  const float min_subnormal = nextafterf(0.0f, 1.0f);
  float upper, lower;
  unsigned trial;

  /* Exact adversarial case: sqrt(1 + 2^-160) is strictly greater than 1,
     although binary64 addition rounds 1 + 2^-160 back to 1. */
  upper = m5_trajectory_safety_test_norm_upper_bound(1.0f, tiny, 0.0f);
  CHECK(upper == nextafterf(1.0f, INFINITY));
  lower = m5_trajectory_safety_test_norm_lower_bound(1.0, (double)tiny, 0.0);
  CHECK(lower <= 1.0f);

  upper = m5_trajectory_safety_test_norm_upper_bound(
      min_subnormal, min_subnormal, 0.0f);
  CHECK(upper == 2.0f * min_subnormal);
  lower = m5_trajectory_safety_test_norm_lower_bound(
      (double)min_subnormal, (double)min_subnormal, 0.0);
  CHECK(lower <= min_subnormal);

  CHECK(m5_trajectory_safety_test_norm_upper_bound(FLT_MAX, 0.0f, 0.0f) ==
        FLT_MAX);
  CHECK(isinf(m5_trajectory_safety_test_norm_upper_bound(
      FLT_MAX, FLT_MAX, 0.0f)));
  CHECK(m5_trajectory_safety_test_norm_lower_bound(
      (double)FLT_MAX, 0.0, 0.0) <= FLT_MAX);

  for (trial = 0u; trial < 512u; ++trial) {
    float component[3];
    long double exact_norm;
    float upper_bound, lower_bound;
    unsigned axis;
    for (axis = 0u; axis < 3u; ++axis) {
      int exponent = -120 + (int)(safety_fuzz_next(&seed) % 225u);
      float magnitude = (float)ldexp(
          0.5 + 0.5 * (double)safety_fuzz_unit(&seed), exponent);
      component[axis] = (safety_fuzz_next(&seed) & 1u) != 0u
          ? -magnitude : magnitude;
    }
    exact_norm = sqrtl((long double)component[0] * component[0] +
        (long double)component[1] * component[1] +
        (long double)component[2] * component[2]);
    upper_bound = m5_trajectory_safety_test_norm_upper_bound(
        component[0], component[1], component[2]);
    lower_bound = m5_trajectory_safety_test_norm_lower_bound(
        (double)component[0], (double)component[1], (double)component[2]);
    CHECK((long double)upper_bound >= exact_norm);
    CHECK((long double)lower_bound <= exact_norm);
  }
}


static void test_m5_safety_point_aabb_distance_lower_oracle(void)
{
  uint32_t seed = 0xAABB0611u;
  unsigned i;

  for (i = 0u; i < 2048u; ++i) {
    float point[3], center[3];
    float half_extent = safety_fuzz_between(&seed, 0.0f, 0.75f);
    float rounding_error = safety_fuzz_between(&seed, 0.0f, 0.25f);
    float lower = -1.0f;
    double exact2 = 0.0;
    double exact;
    unsigned axis;

    for (axis = 0u; axis < 3u; ++axis) {
      point[axis] = safety_fuzz_between(&seed, -8.0f, 8.0f);
      center[axis] = safety_fuzz_between(&seed, -8.0f, 8.0f);
    }
    CHECK(m5_trajectory_safety_test_point_aabb_distance_lower(
        point, center, half_extent, rounding_error, &lower));
    for (axis = 0u; axis < 3u; ++axis) {
      double delta = fabs((double)point[axis] - center[axis]) -
                     (double)half_extent;
      if (delta > 0.0) exact2 += delta * delta;
    }
    exact = fmax(0.0, sqrt(exact2) - (double)rounding_error);
    CHECK((double)lower <= exact);
    CHECK(lower >= 0.0f);
  }

  {
    const float center[3] = {0.0f, 0.0f, 0.0f};
    const float face[3] = {0.25f, 0.0f, 0.0f};
    const float edge[3] = {0.25f, 0.25f, 0.0f};
    const float corner[3] = {0.25f, 0.25f, 0.25f};
    float lower;
    CHECK(m5_trajectory_safety_test_point_aabb_distance_lower(
        face, center, 0.25f, 0.0f, &lower));
    CHECK(lower == 0.0f);
    CHECK(m5_trajectory_safety_test_point_aabb_distance_lower(
        edge, center, 0.25f, 0.0f, &lower));
    CHECK(lower == 0.0f);
    CHECK(m5_trajectory_safety_test_point_aabb_distance_lower(
        corner, center, 0.25f, 0.0f, &lower));
    CHECK(lower == 0.0f);
    CHECK(!m5_trajectory_safety_test_point_aabb_distance_lower(
        face, center, -0.25f, 0.0f, &lower));
    CHECK(!m5_trajectory_safety_test_point_aabb_distance_lower(
        face, center, 0.25f, -0.01f, &lower));
  }
}

static double safety_test_norm3(const double v[3])
{
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void test_m5_safety_dynamic_relative_sweep_oracle(void)
{
  uint32_t seed = 0xD19A0612u;
  unsigned trial;

  for (trial = 0u; trial < 512u; ++trial) {
    double curve_velocity[3], obstacle_velocity[3];
    double curve_acceleration[3], obstacle_acceleration[3];
    double velocity_uncertainty[3], relative_initial[3];
    double curve_accel_norm, obstacle_accel_norm;
    double velocity_std, relative_accel_bound;
    double curve_speed_bound, obstacle_speed_bound, relative_speed_bound;
    float half_dt = safety_fuzz_between(&seed, 0.001f, 0.12f);
    float sweep_bound = 0.0f;
    unsigned axis, sample;

    for (axis = 0u; axis < 3u; ++axis) {
      curve_velocity[axis] = (double)safety_fuzz_signed_unit(&seed) * 5.0;
      obstacle_velocity[axis] = (double)safety_fuzz_signed_unit(&seed) * 4.0;
      curve_acceleration[axis] =
          (double)safety_fuzz_signed_unit(&seed) * 1.5;
      obstacle_acceleration[axis] =
          (double)safety_fuzz_signed_unit(&seed) * 1.0;
      velocity_uncertainty[axis] =
          (double)safety_fuzz_signed_unit(&seed) * 0.1;
      relative_initial[axis] =
          curve_velocity[axis] - obstacle_velocity[axis];
    }

    curve_accel_norm = safety_test_norm3(curve_acceleration);
    obstacle_accel_norm = safety_test_norm3(obstacle_acceleration);
    velocity_std = safety_test_norm3(velocity_uncertainty);
    relative_accel_bound = curve_accel_norm + obstacle_accel_norm;
    curve_speed_bound = safety_test_norm3(curve_velocity) +
                        curve_accel_norm * (double)half_dt;
    obstacle_speed_bound = safety_test_norm3(obstacle_velocity) +
                           obstacle_accel_norm * (double)half_dt;
    relative_speed_bound = safety_test_norm3(relative_initial) + velocity_std;

    CHECK(m5_trajectory_safety_test_dynamic_relative_sweep_bound(
        (float)curve_speed_bound, (float)obstacle_speed_bound,
        (float)velocity_std, (float)relative_speed_bound,
        (float)relative_accel_bound, half_dt, &sweep_bound));

    for (sample = 0u; sample <= 32u; ++sample) {
      double t = (double)half_dt * (double)sample / 32.0;
      double displacement[3];
      double distance;
      for (axis = 0u; axis < 3u; ++axis) {
        double relative_acceleration = curve_acceleration[axis] -
                                       obstacle_acceleration[axis];
        displacement[axis] =
            (relative_initial[axis] + velocity_uncertainty[axis]) * t +
            0.5 * relative_acceleration * t * t;
      }
      distance = safety_test_norm3(displacement);
      CHECK(distance <= (double)sweep_bound + 0.00001);
    }
  }

  {
    float sweep = 0.0f;
    CHECK(!m5_trajectory_safety_test_dynamic_relative_sweep_bound(
        -1.0f, 1.0f, 0.1f, 1.0f, 1.0f, 0.1f, &sweep));
    CHECK(!m5_trajectory_safety_test_dynamic_relative_sweep_bound(
        1.0f, 1.0f, 0.1f, 1.0f, 1.0f, -0.1f, &sweep));
    CHECK(!m5_trajectory_safety_test_dynamic_relative_sweep_bound(
        1.0f, 1.0f, 0.1f, 1.0f, INFINITY, 0.1f, &sweep));
  }
}

static void test_m5_safety_static_free_reference_passes(void)
{
  SafetyFx fx;
  M5TrajectorySafetyStatus status;
  safety_fx_init(&fx, 20000000u);
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);
  CHECK(fx.result.work_units == PRODUCT_PREVIEW_COUNT);
  CHECK(fx.result.valid_until_us >= fx.input.evaluation_time_us +
      PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US);
}

static void test_m5_safety_dynamic_response_delay_is_inflated(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 21000000u);
  track = &fx.map.tracks[0];
  track->id = 11u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->position_ned_m[0] = 5.08f;
  track->velocity_ned_mps[0] = -10.0f;
  track->radius_m = 0.05f;
  track->position_std_m = 0.0f;
  track->velocity_std_mps = 0.0f;

  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  fx.input.response_delay_us = 10000u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_unknown_voxel_fails_closed(void)
{
  SafetyFx fx;
  float origin[3] = {0.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  safety_fx_init(&fx, 22000000u);
  CHECK(m5_local_map_world_to_voxel(&fx.map, origin, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_UNKNOWN;
  fx.map.cells[voxel].log_odds = 0;
  fx.map.cells[voxel].source_mask = 0u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_UNKNOWN);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_nan_preview_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 23000000u);
  fx.reference.points[7].point.position_ned_m[1] = NAN;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_reference_expiring_before_proof_end_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 24000000u);
  fx.reference.header.valid_until_us = fx.base.now +
      PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US - 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_keep_out_crossing_between_preview_endpoints(void)
{
  SafetyFx fx;
  unsigned i;
  safety_fx_init(&fx, 25000000u);
  fx.base.nav.core.position_ned_m[0] = -1.0f;
  fx.base.nav.core.position_ned_m[2] = -10.0f;
  fx.base.nav.core.velocity_ned_mps[0] = 100.0f;
  safety_map_reset(&fx);
  fx.base.env.max_linear_speed_mps = 120.0f;
  fx.base.env.max_linear_accel_mps2 = 200.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 120.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 200.0f;
  safety_point_set(&fx.reference.anchor, -1.0f, 0.0f, -10.0f,
                   100.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float x = -1.0f + 100.0f * (float)(i + 1u) *
        ((float)PRODUCT_PREVIEW_STEP_US * 1e-6f);
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], x, 0.0f, -10.0f,
                     100.0f, 0.0f, 0.0f);
  }

  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 1u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_OUT;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -0.2f;
  fx.fence.vertices_ne_m[0][1] = -0.2f;
  fx.fence.vertices_ne_m[1][0] = 0.2f;
  fx.fence.vertices_ne_m[1][1] = -0.2f;
  fx.fence.vertices_ne_m[2][0] = 0.2f;
  fx.fence.vertices_ne_m[2][1] = 0.2f;
  fx.fence.vertices_ne_m[3][0] = -0.2f;
  fx.fence.vertices_ne_m[3][1] = 0.2f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_GEOFENCE_VIOLATION);
  CHECK(fx.result.passed == 0u);
}


static void test_m5_safety_occupied_static_voxel_rejected(void)
{
  SafetyFx fx;
  float origin[3] = {0.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  safety_fx_init(&fx, 26000000u);
  CHECK(m5_local_map_world_to_voxel(&fx.map, origin, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  fx.map.cells[voxel].log_odds = 4;
  fx.map.cells[voxel].source_mask = 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}


static void test_m5_safety_stale_voxel_fails_closed(void)
{
  SafetyFx fx;
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  safety_fx_init(&fx, 26000000u);
  CHECK(m5_local_map_world_to_voxel(&fx.map, origin, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_STALE;
  fx.map.cells[voxel].log_odds = -4;
  fx.map.cells[voxel].source_mask = 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_invalid_voxel_encoding_rejected(void)
{
  SafetyFx fx;
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  safety_fx_init(&fx, 27000000u);
  CHECK(m5_local_map_world_to_voxel(&fx.map, origin, &voxel));
  fx.map.cells[voxel].state = 0xFFu;
  fx.map.cells[voxel].log_odds = 0;
  fx.map.cells[voxel].source_mask = 0u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_midpoint_collision_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 28000000u);
  track = &fx.map.tracks[0];
  track->id = 21u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->position_ned_m[0] = 0.10f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_crossing_collision_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  unsigned i;
  safety_fx_init(&fx, 29000000u);
  fx.base.nav.core.position_ned_m[0] = -1.0f;
  fx.base.nav.core.position_ned_m[2] = -10.0f;
  fx.base.nav.core.velocity_ned_mps[0] = 100.0f;
  safety_map_reset(&fx);
  fx.base.env.max_linear_speed_mps = 120.0f;
  fx.base.env.max_linear_accel_mps2 = 200.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 120.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 200.0f;
  safety_point_set(&fx.reference.anchor, -1.0f, 0.0f, -10.0f,
                   100.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float x = -1.0f + 100.0f * (float)(i + 1u) *
        ((float)PRODUCT_PREVIEW_STEP_US * 1e-6f);
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], x, 0.0f, -10.0f,
                     100.0f, 0.0f, 0.0f);
  }
  track = &fx.map.tracks[0];
  track->id = 22u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->position_ned_m[1] = -1.0f;
  track->position_ned_m[2] = -10.0f;
  track->velocity_ned_mps[1] = 100.0f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_static_collision_between_preview_endpoints(void)
{
  SafetyFx fx;
  const float obstacle[3] = {0.25f, 0.0f, -10.0f};
  uint16_t voxel = 0u;
  unsigned i;
  safety_fx_init(&fx, 30000000u);
  fx.base.nav.core.position_ned_m[0] = -1.0f;
  fx.base.nav.core.position_ned_m[2] = -10.0f;
  fx.base.nav.core.velocity_ned_mps[0] = 100.0f;
  safety_map_reset(&fx);
  fx.base.env.max_linear_speed_mps = 120.0f;
  fx.base.env.max_linear_accel_mps2 = 200.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 120.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 200.0f;
  safety_point_set(&fx.reference.anchor, -1.0f, 0.0f, -10.0f,
                   100.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float x = -1.0f + 100.0f * (float)(i + 1u) *
        ((float)PRODUCT_PREVIEW_STEP_US * 1e-6f);
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], x, 0.0f, -10.0f,
                     100.0f, 0.0f, 0.0f);
  }
  CHECK(m5_local_map_world_to_voxel(&fx.map, obstacle, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  fx.map.cells[voxel].log_odds = 4;
  fx.map.cells[voxel].source_mask = 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_static_collision_at_preview_seam_adaptive_split(void)
{
  SafetyFx fx;
  const float obstacle[3] = {0.0f, 0.0f, 0.0f};
  uint16_t voxel = 0u;
  unsigned i;

  safety_fx_init(&fx, 30500000u);
  fx.base.nav.core.position_ned_m[0] = -0.48f;
  fx.base.nav.core.velocity_ned_mps[0] = 24.0f;
  safety_map_reset(&fx);

  fx.base.env.max_linear_speed_mps = 100.0f;
  fx.base.env.max_linear_accel_mps2 = 100000.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 100.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 100000.0f;
  safety_point_set(&fx.reference.anchor, -0.48f, 0.0f, 0.0f,
                   24.0f, 0.0f, 0.0f);
  fx.reference.points[0].offset_us = PRODUCT_PREVIEW_STEP_US;
  safety_point_set(&fx.reference.points[0], 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f);
  for (i = 1u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us =
        (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], 0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f);
  }
  CHECK(m5_local_map_world_to_voxel(&fx.map, obstacle, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  fx.map.cells[voxel].log_odds = 4;
  fx.map.cells[voxel].source_mask = 1u;

  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.work_units > 1u);
}


static void safety_eval_hermite_basis_axis(
    double p0, double v0, double a0,
    double p1, double v1, double a1,
    double duration, double time_s,
    double *position, double *velocity, double *acceleration)
{
  double u = time_s / duration;
  double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
  double h00 = 1.0 - 10.0*u3 + 15.0*u4 - 6.0*u5;
  double h10 = u - 6.0*u3 + 8.0*u4 - 3.0*u5;
  double h20 = 0.5 * (u2 - 3.0*u3 + 3.0*u4 - u5);
  double h01 = 10.0*u3 - 15.0*u4 + 6.0*u5;
  double h11 = -4.0*u3 + 7.0*u4 - 3.0*u5;
  double h21 = 0.5 * (u3 - 2.0*u4 + u5);
  double h00d = -30.0*u2 + 60.0*u3 - 30.0*u4;
  double h10d = 1.0 - 18.0*u2 + 32.0*u3 - 15.0*u4;
  double h20d = u - 4.5*u2 + 6.0*u3 - 2.5*u4;
  double h01d = 30.0*u2 - 60.0*u3 + 30.0*u4;
  double h11d = -12.0*u2 + 28.0*u3 - 15.0*u4;
  double h21d = 1.5*u2 - 4.0*u3 + 2.5*u4;
  double h00dd = -60.0*u + 180.0*u2 - 120.0*u3;
  double h10dd = -36.0*u + 96.0*u2 - 60.0*u3;
  double h20dd = 1.0 - 9.0*u + 18.0*u2 - 10.0*u3;
  double h01dd = 60.0*u - 180.0*u2 + 120.0*u3;
  double h11dd = -24.0*u + 84.0*u2 - 60.0*u3;
  double h21dd = 3.0*u - 12.0*u2 + 10.0*u3;
  double t2 = duration * duration;

  *position = h00*p0 + h10*duration*v0 + h20*t2*a0 +
      h01*p1 + h11*duration*v1 + h21*t2*a1;
  *velocity = (h00d*p0 + h10d*duration*v0 + h20d*t2*a0 +
      h01d*p1 + h11d*duration*v1 + h21d*t2*a1) / duration;
  *acceleration = (h00dd*p0 + h10dd*duration*v0 + h20dd*t2*a0 +
      h01dd*p1 + h11dd*duration*v1 + h21dd*t2*a1) / t2;
}

static void test_m5_safety_arbitrary_hermite_swept_volume_differential(void)
{
  uint32_t seed = 0x5EED0679u;
  const double duration = (double)PRODUCT_PREVIEW_COUNT *
      PRODUCT_PREVIEW_STEP_US * 1.0e-6;
  const double obstacle_time = 0.237;
  unsigned trial;

  for (trial = 0u; trial < 64u; ++trial) {
    SafetyFx fx;
    double p0[3], v0[3], a0[3], p1[3], v1[3], a1[3];
    double contact_position[3], contact_velocity[3], contact_acceleration[3];
    double minimum_aabb_distance = INFINITY;
    float obstacle[3], center[3];
    uint16_t voxel = 0u;
    unsigned axis, i, sample;
    int collision_case = (trial < 32u);

    safety_fx_init(&fx, 180000000ull + (uint64_t)trial * 1000000ull);
    p0[0] = -0.45 + (double)safety_fuzz_signed_unit(&seed) * 0.01;
    p1[0] =  0.45 + (double)safety_fuzz_signed_unit(&seed) * 0.01;
    v0[0] = 1.875 + (double)safety_fuzz_signed_unit(&seed) * 0.05;
    v1[0] = 1.875 + (double)safety_fuzz_signed_unit(&seed) * 0.05;
    a0[0] = (double)safety_fuzz_signed_unit(&seed) * 0.10;
    a1[0] = (double)safety_fuzz_signed_unit(&seed) * 0.10;
    p0[1] = (double)safety_fuzz_signed_unit(&seed) * 0.02;
    p1[1] = (double)safety_fuzz_signed_unit(&seed) * 0.02;
    v0[1] = (double)safety_fuzz_signed_unit(&seed) * 0.03;
    v1[1] = (double)safety_fuzz_signed_unit(&seed) * 0.03;
    a0[1] = (double)safety_fuzz_signed_unit(&seed) * 0.05;
    a1[1] = (double)safety_fuzz_signed_unit(&seed) * 0.05;
    p0[2] = p1[2] = 0.0;
    v0[2] = v1[2] = 0.0;
    a0[2] = a1[2] = 0.0;

    for (axis = 0u; axis < 3u; ++axis) {
      fx.base.nav.core.position_ned_m[axis] = (float)p0[axis];
      fx.base.nav.core.velocity_ned_mps[axis] = (float)v0[axis];
      fx.base.nav.core.acceleration_ned_mps2[axis] = (float)a0[axis];
    }
    safety_point_set(&fx.reference.anchor,
        (float)p0[0], (float)p0[1], (float)p0[2],
        (float)v0[0], (float)v0[1], (float)v0[2]);
    for (axis = 0u; axis < 3u; ++axis) {
      fx.reference.anchor.point.acceleration_ned_mps2[axis] = (float)a0[axis];
    }
    for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
      double t = (double)(i + 1u) * PRODUCT_PREVIEW_STEP_US * 1.0e-6;
      double position[3], velocity[3], acceleration[3];
      for (axis = 0u; axis < 3u; ++axis) {
        safety_eval_hermite_basis_axis(p0[axis], v0[axis], a0[axis],
            p1[axis], v1[axis], a1[axis], duration, t,
            &position[axis], &velocity[axis], &acceleration[axis]);
      }
      safety_point_set(&fx.reference.points[i],
          (float)position[0], (float)position[1], (float)position[2],
          (float)velocity[0], (float)velocity[1], (float)velocity[2]);
      fx.reference.points[i].offset_us =
          (i + 1u) * PRODUCT_PREVIEW_STEP_US;
      for (axis = 0u; axis < 3u; ++axis) {
        fx.reference.points[i].point.acceleration_ned_mps2[axis] =
            (float)acceleration[axis];
      }
    }
    fx.reference.anchor.offset_us = 0u;
    for (axis = 0u; axis < 3u; ++axis) {
      safety_eval_hermite_basis_axis(p0[axis], v0[axis], a0[axis],
          p1[axis], v1[axis], a1[axis], duration, obstacle_time,
          &contact_position[axis], &contact_velocity[axis],
          &contact_acceleration[axis]);
    }

    obstacle[0] = (float)contact_position[0];
    obstacle[1] = (float)(contact_position[1] +
        (collision_case ? 0.0 : 1.5));
    obstacle[2] = (float)contact_position[2];
    safety_map_reset(&fx);
    CHECK(m5_local_map_world_to_voxel(&fx.map, obstacle, &voxel));
    m5_local_map_voxel_center(&fx.map, voxel, center);

    for (sample = 0u; sample <= 1024u; ++sample) {
      double t = duration * (double)sample / 1024.0;
      double point[3], velocity[3], acceleration[3], distance2 = 0.0;
      for (axis = 0u; axis < 3u; ++axis) {
        double delta;
        safety_eval_hermite_basis_axis(p0[axis], v0[axis], a0[axis],
            p1[axis], v1[axis], a1[axis], duration, t,
            &point[axis], &velocity[axis], &acceleration[axis]);
        delta = fabs(point[axis] - (double)center[axis]) - 0.25;
        if (delta > 0.0) distance2 += delta * delta;
      }
      minimum_aabb_distance = fmin(minimum_aabb_distance, sqrt(distance2));
    }

    if (collision_case) {
      double contact_distance2 = 0.0;
      for (axis = 0u; axis < 3u; ++axis) {
        double delta = fabs(contact_position[axis] -
                            (double)center[axis]) - 0.25;
        if (delta > 0.0) contact_distance2 += delta * delta;
      }
      CHECK(contact_distance2 == 0.0);
    } else {
      CHECK(minimum_aabb_distance > 0.70);
    }

    fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
    fx.map.cells[voxel].log_odds = 4;
    fx.map.cells[voxel].source_mask = 1u;
    if (collision_case) {
      CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
          &fx.result) == M5_SAFETY_STATIC_COLLISION);
      CHECK(fx.result.passed == 0u);
    } else {
      CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
          &fx.result) == M5_SAFETY_PASS);
      CHECK(fx.result.passed == 1u);
    }
  }
}

static void test_m5_safety_static_voxel_sweep_differential(void)
{
  uint32_t seed = 0x5A71C061u;
  unsigned trial;

  for (trial = 0u; trial < 64u; ++trial) {
    SafetyFx fx;
    const float start_x = -0.75f;
    const float speed_x = 2.0f;
    const double horizon_s =
        (double)PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US * 1.0e-6;
    float crossing_time = safety_fuzz_between(&seed, 0.01f, 0.47f);
    float contact[3], center[3];
    double distance2 = 0.0;
    double clearance;
    uint16_t voxel = 0u;
    unsigned axis;
    int collision_case = (trial < 32u);

    safety_fx_init(&fx, 100000000ull + (uint64_t)trial * 1000000ull);
    safety_set_linear_reference(&fx, start_x, 0.0f, -10.0f,
                                speed_x, 0.0f, 0.0f);
    contact[0] = start_x + speed_x * crossing_time;
    contact[1] = collision_case ? 0.0f : 1.5f;
    contact[2] = -10.0f;
    CHECK((double)crossing_time > 0.0 &&
          (double)crossing_time < horizon_s);
    CHECK(m5_local_map_world_to_voxel(&fx.map, contact, &voxel));
    m5_local_map_voxel_center(&fx.map, voxel, center);

    for (axis = 0u; axis < 3u; ++axis) {
      double delta = fabs((double)center[axis] -
                          (axis == 0u ? (double)contact[axis] :
                           (axis == 1u ? 0.0 : (double)contact[axis]))) - 0.25;
      if (delta > 0.0) distance2 += delta * delta;
    }
    clearance = sqrt(distance2) - (double)fx.map.vehicle_radius_m -
                (double)fx.map.clearance_margin_m;
    if (collision_case) {
      CHECK(clearance <= 0.0);
    } else {
      CHECK(clearance > 0.25);
    }

    fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
    fx.map.cells[voxel].log_odds = 4;
    fx.map.cells[voxel].source_mask = 1u;
    if (collision_case) {
      CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
          &fx.result) == M5_SAFETY_STATIC_COLLISION);
      CHECK(fx.result.passed == 0u);
    } else {
      CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
          &fx.result) == M5_SAFETY_PASS);
      CHECK(fx.result.passed == 1u);
    }
  }
}

static void test_m5_safety_keep_in_curve_excursion_rejected(void)
{
  SafetyFx fx;
  unsigned i;
  safety_fx_init(&fx, 31000000u);
  fx.base.nav.core.position_ned_m[2] = -5.0f;
  fx.base.nav.core.velocity_ned_mps[1] = 5.0f;
  safety_map_reset(&fx);
  fx.base.env.max_linear_speed_mps = 10.0f;
  /* The quintic that reverses +5 -> -5 m/s in 20 ms peaks just below
     1000 m/s^2; leave headroom for the MR-009 binary32 evaluation envelope so
     this fixture exercises the geofence path rather than the dynamics gate. */
  fx.base.env.max_linear_accel_mps2 = 1100.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[1] = 10.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[1] = 1100.0f;
  safety_point_set(&fx.reference.anchor, 0.0f, 0.0f, -5.0f,
                   0.0f, 5.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], 0.0f, 0.0f, -5.0f,
                     0.0f, (i == 0u) ? -5.0f : 0.0f, 0.0f);
  }

  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 2u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -2.0f;
  fx.fence.vertices_ne_m[0][1] = -0.02f;
  fx.fence.vertices_ne_m[1][0] = 2.0f;
  fx.fence.vertices_ne_m[1][1] = -0.02f;
  fx.fence.vertices_ne_m[2][0] = 2.0f;
  fx.fence.vertices_ne_m[2][1] = 0.02f;
  fx.fence.vertices_ne_m[3][0] = -2.0f;
  fx.fence.vertices_ne_m[3][1] = 0.02f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_GEOFENCE_VIOLATION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_expiry_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 32000000u);
  track = &fx.map.tracks[0];
  track->id = 23u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now +
      (uint64_t)PRODUCT_PREVIEW_COUNT * PRODUCT_PREVIEW_STEP_US - 1u;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_duplicate_dynamic_track_rejected(void)
{
  SafetyFx fx;
  unsigned i;
  safety_fx_init(&fx, 33000000u);
  for (i = 0u; i < 2u; ++i) {
    fx.map.tracks[i].id = 24u;
    fx.map.tracks[i].source_id = 2u;
    fx.map.tracks[i].sample_us = fx.base.now;
    fx.map.tracks[i].expires_us = fx.base.now + 1000000u;
    fx.map.tracks[i].radius_m = 0.05f;
  }
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_invalid_metadata_fails_closed(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33500000u);
  track = &fx.map.tracks[0];
  track->id = 26u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->radius_m = 0.05f;
  track->flags = 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);

  track->flags = 0u;
  track->source_id = 0u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_requires_acceleration_bound(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33600000u);
  track = &fx.map.tracks[0];
  track->id = 27u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->radius_m = 0.05f;
  fx.input.max_dynamic_obstacle_accel_mps2 = 0.0f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_future_sample_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33700000u);
  track = &fx.map.tracks[0];
  track->id = 28u;
  track->source_id = 2u;
  track->sample_us = fx.base.now + 1u;
  track->expires_us = fx.base.now + 1000000u;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_overage_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33800000u);
  track = &fx.map.tracks[0];
  track->id = 29u;
  track->source_id = 2u;
  track->sample_us = fx.base.now - M5_MAP_STALE_US - 1u;
  track->expires_us = fx.base.now + 1000000u;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_nonfinite_position_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33900000u);
  track = &fx.map.tracks[0];
  track->id = 30u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->position_ned_m[1] = NAN;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_negative_radius_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33950000u);
  track = &fx.map.tracks[0];
  track->id = 31u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->radius_m = -0.01f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_dynamic_track_nonfinite_velocity_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 33970000u);
  track = &fx.map.tracks[0];
  track->id = 32u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->velocity_ned_mps[2] = INFINITY;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STALE);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_geofence_collinear_vertices_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40550000u);
  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 6u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -1.0f;
  fx.fence.vertices_ne_m[0][1] = 0.0f;
  fx.fence.vertices_ne_m[1][0] = 0.0f;
  fx.fence.vertices_ne_m[1][1] = 0.0f;
  fx.fence.vertices_ne_m[2][0] = 1.0f;
  fx.fence.vertices_ne_m[2][1] = 0.0f;
  fx.fence.vertices_ne_m[3][0] = 2.0f;
  fx.fence.vertices_ne_m[3][1] = 0.0f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_response_delay_limit_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 34000000u);
  fx.input.response_delay_us = M5_SAFETY_MAX_RESPONSE_DELAY_US + 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_preview_count_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 35000000u);
  fx.reference.point_count = PRODUCT_PREVIEW_COUNT - 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}


static void test_m5_safety_navigation_map_epoch_origin_frame_mismatch_rejected(void)
{
  SafetyFx fx;

  safety_fx_init(&fx, 45000000u);
  fx.base.nav.state_epoch++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 46000000u);
  fx.base.nav.origin_id++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 47000000u);
  fx.map.state_epoch++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 48000000u);
  fx.map.origin_id++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 49000000u);
  fx.base.nav.header.frame_id = 0u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);

  safety_fx_init(&fx, 50000000u);
  fx.reference.header.frame_id = 0u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_map_edge_fails_closed(void)
{
  SafetyFx fx;
  unsigned i;
  M5TrajectorySafetyStatus status;

  safety_fx_init(&fx, 51000000u);
  fx.base.nav.core.velocity_ned_mps[0] = 8.5f;
  fx.base.env.max_linear_speed_mps = 10.0f;
  fx.base.env.max_linear_accel_mps2 = 100.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 10.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 100.0f;
  safety_map_reset(&fx);
  safety_point_set(&fx.reference.anchor, 0.0f, 0.0f, 0.0f,
                   8.5f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], (float)(i + 1u) * 0.17f,
                     0.0f, 0.0f, 8.5f, 0.0f, 0.0f);
  }
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_UNKNOWN);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_MAP_UNKNOWN);

  safety_fx_init(&fx, 52000000u);
  fx.base.nav.core.velocity_ned_mps[0] = -8.5f;
  fx.base.env.max_linear_speed_mps = 10.0f;
  fx.base.env.max_linear_accel_mps2 = 100.0f;
  fx.base.env.vehicle_extension.max_velocity_ned_mps[0] = 10.0f;
  fx.base.env.vehicle_extension.max_acceleration_ned_mps2[0] = 100.0f;
  safety_map_reset(&fx);
  safety_point_set(&fx.reference.anchor, 0.0f, 0.0f, 0.0f,
                   -8.5f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], -(float)(i + 1u) * 0.17f,
                     0.0f, 0.0f, -8.5f, 0.0f, 0.0f);
  }
  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_UNKNOWN);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_MAP_UNKNOWN);
}

static void test_m5_safety_dfs_max_depth_stack_guard(void)
{
  SafetyFx fx;
  unsigned i, axis;
  uint16_t voxel = 0u;
  const float unknown_cell[3] = {0.25f, 0.75f, 0.25f};
  struct {
    uint32_t before;
    M5TrajectorySafetyWorkspace workspace;
    uint32_t after;
  } guarded;
  M5TrajectorySafetyStatus status;

  safety_fx_init(&fx, 44000000u);
  fx.base.nav.core.position_ned_m[1] = 0.34999f;
  fx.base.nav.core.velocity_ned_mps[0] = 0.5f;
  fx.base.nav.core.velocity_ned_mps[1] = 0.0f;
  fx.base.nav.core.velocity_ned_mps[2] = 0.0f;
  for (axis = 0u; axis < 3u; ++axis)
    fx.base.nav.core.acceleration_ned_mps2[axis] = 0.0f;
  safety_map_reset(&fx);

  CHECK(m5_local_map_world_to_voxel(&fx.map, unknown_cell, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_UNKNOWN;
  fx.map.cells[voxel].log_odds = 0;
  fx.map.cells[voxel].source_mask = 0u;

  safety_point_set(&fx.reference.anchor, 0.0f, 0.34999f, 0.0f,
                   0.5f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], (float)(i + 1u) * 0.01f,
                     0.34999f, 0.0f, 0.5f, 0.0f, 0.0f);
  }

  guarded.before = 0x4D350601u;
  guarded.after = 0xA60C5AFEul;
  CHECK(M5_SAFETY_STACK_CAPACITY ==
        M5_SAFETY_INTERVAL_COUNT + M5_SAFETY_MAX_DEPTH);
  CHECK(M5_SAFETY_STACK_CAPACITY == 29u);
  CHECK(sizeof(guarded.workspace.stack) / sizeof(guarded.workspace.stack[0]) == 29u);
  CHECK(sizeof(guarded.workspace) <= 512u);
  status = m5_trajectory_safety_validate(&fx.input, &guarded.workspace,
                                         &fx.result);
  CHECK(status == M5_SAFETY_UNKNOWN);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_MAP_UNKNOWN);
  CHECK(fx.result.work_units == M5_SAFETY_MAX_DEPTH + 1u);
  CHECK(guarded.before == 0x4D350601u);
  CHECK(guarded.after == 0xA60C5AFEul);
}

static void test_m5_safety_dfs_work_unit_budget_exhaustion(void)
{
  SafetyFx fx;
  unsigned i, axis;
  uint16_t voxel = 0u;
  const float unknown_cell[3] = {0.25f, 0.75f, 0.25f};
  M5TrajectorySafetyStatus status;
  safety_fx_init(&fx, 43000000u);

  /* Constant 0.5m/s path passes 0.2mm outside the inflated UNKNOWN voxel.
     Each 20ms interval needs depth-5 subdivision to prove clearance; repeated
     intervals exhaust the DFS work-unit budget before the voxel-check budget. */
  fx.base.nav.core.position_ned_m[0] = 0.0f;
  fx.base.nav.core.position_ned_m[1] = 0.3498f;
  fx.base.nav.core.position_ned_m[2] = 0.0f;
  fx.base.nav.core.velocity_ned_mps[0] = 0.5f;
  fx.base.nav.core.velocity_ned_mps[1] = 0.0f;
  fx.base.nav.core.velocity_ned_mps[2] = 0.0f;
  for (axis = 0u; axis < 3u; ++axis)
    fx.base.nav.core.acceleration_ned_mps2[axis] = 0.0f;
  safety_map_reset(&fx);
  CHECK(m5_local_map_world_to_voxel(&fx.map, unknown_cell, &voxel));
  fx.map.cells[voxel].state = M5_MAP_CELL_UNKNOWN;
  fx.map.cells[voxel].log_odds = 0;
  fx.map.cells[voxel].source_mask = 0u;

  safety_point_set(&fx.reference.anchor, 0.0f, 0.3498f, 0.0f,
                   0.5f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], (float)(i + 1u) * 0.01f,
                     0.3498f, 0.0f, 0.5f, 0.0f, 0.0f);
  }

  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_BUDGET_EXCEEDED);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.work_units == M5_SAFETY_MAX_WORK_UNITS + 1u);
  CHECK(fx.result.cell_checks < M5_SAFETY_MAX_CELL_CHECKS);
}

static void test_m5_safety_cell_check_budget_exhaustion(void)
{
  SafetyFx fx;
  unsigned i, axis;
  M5TrajectorySafetyStatus status;
  safety_fx_init(&fx, 42000000u);

  /* Host-only stress envelope: each 20ms segment sweeps a broad but fully
     in-map FREE volume. The test targets the voxel-check counter, not a
     collision, map-edge, or dynamics rejection. */
  for (axis = 0u; axis < 3u; ++axis) {
    fx.base.nav.core.position_ned_m[axis] = 0.0f;
    fx.base.nav.core.velocity_ned_mps[axis] = 0.0f;
    fx.base.nav.core.acceleration_ned_mps2[axis] = 0.0f;
    fx.base.env.vehicle_extension.max_velocity_ned_mps[axis] = 300.0f;
    fx.base.env.vehicle_extension.max_acceleration_ned_mps2[axis] = 60000.0f;
  }
  fx.base.env.max_linear_speed_mps = 300.0f;
  fx.base.env.max_linear_accel_mps2 = 60000.0f;
  safety_map_reset(&fx);

  safety_point_set(&fx.reference.anchor, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float x = (i % 2u == 0u) ? 0.5f : -0.5f;
    fx.reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx.reference.points[i], x, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f);
  }

  status = m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result);
  CHECK(status == M5_SAFETY_BUDGET_EXCEEDED);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.cell_checks == M5_SAFETY_MAX_CELL_CHECKS + 1u);
  CHECK(fx.result.work_units < M5_SAFETY_MAX_WORK_UNITS);
}

static void test_m5_safety_dynamic_acceleration_bound_is_inflated(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 38000000u);
  track = &fx.map.tracks[0];
  track->id = 25u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 1000000u;
  track->position_ned_m[0] = 2.4f;
  track->radius_m = 0.05f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  fx.input.max_dynamic_obstacle_accel_mps2 =
      M5_SAFETY_MAX_DYNAMIC_ACCEL_MPS2;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_navigation_missing_required_validity_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 39000000u);
  fx.base.nav.core.valid_mask &= ~NAV_VALID_VERTICAL_POSITION;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_navigation_state_epoch_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40100000u);
  fx.base.nav.state_epoch++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_navigation_origin_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40200000u);
  fx.base.nav.origin_id++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_navigation_frame_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40300000u);
  fx.base.nav.header.frame_id = (uint8_t)(FRAME_LOCAL_NED ^ 0x01u);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_geofence_origin_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40400000u);
  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 4u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN + 1u;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -1.0f;
  fx.fence.vertices_ne_m[0][1] = -1.0f;
  fx.fence.vertices_ne_m[1][0] = 1.0f;
  fx.fence.vertices_ne_m[1][1] = -1.0f;
  fx.fence.vertices_ne_m[2][0] = 1.0f;
  fx.fence.vertices_ne_m[2][1] = 1.0f;
  fx.fence.vertices_ne_m[3][0] = -1.0f;
  fx.fence.vertices_ne_m[3][1] = 1.0f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_geofence_duplicate_vertex_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40500000u);
  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 5u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -1.0f;
  fx.fence.vertices_ne_m[0][1] = -1.0f;
  fx.fence.vertices_ne_m[1][0] = 1.0f;
  fx.fence.vertices_ne_m[1][1] = -1.0f;
  fx.fence.vertices_ne_m[2][0] = 1.0f;
  fx.fence.vertices_ne_m[2][1] = -1.0f;
  fx.fence.vertices_ne_m[3][0] = -1.0f;
  fx.fence.vertices_ne_m[3][1] = 1.0f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_map_revision_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 40000000u);
  fx.input.expected_map_revision++;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_self_intersecting_geofence_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 41000000u);
  fill_header(&fx.fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx.base.now);
  fx.fence.fence_id = 3u;
  fx.fence.frame_id = FRAME_LOCAL_NED;
  fx.fence.origin_id = T_ORIGIN;
  fx.fence.state_epoch = T_STATE_EPOCH;
  fx.fence.validity = PRODUCT_GEOFENCE_VALID;
  fx.fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx.fence.vertex_count = 4u;
  fx.fence.vertices_ne_m[0][0] = -1.0f;
  fx.fence.vertices_ne_m[0][1] = -1.0f;
  fx.fence.vertices_ne_m[1][0] = 1.0f;
  fx.fence.vertices_ne_m[1][1] = 1.0f;
  fx.fence.vertices_ne_m[2][0] = -1.0f;
  fx.fence.vertices_ne_m[2][1] = 1.0f;
  fx.fence.vertices_ne_m[3][0] = 1.0f;
  fx.fence.vertices_ne_m[3][1] = -1.0f;
  fx.fence.min_height_above_home_m = 0.0f;
  fx.fence.max_height_above_home_m = 50.0f;
  fx.input.geofences = &fx.fence;
  fx.input.geofence_count = 1u;
  CHECK(!fence_geometry_valid(&fx.fence));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_infinite_preview_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 36000000u);
  fx.reference.points[11].point.velocity_ned_mps[2] = INFINITY;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_preview_offset_mismatch_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 37000000u);
  fx.reference.points[3].offset_us += 1u;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

/*
 * S06-086 (MR-009): the validator must bound the curve M6 actually evaluates.
 *
 * control_sample_reference() from the control model is linked into this binary
 * and sampled at the same absolute offset_us the consumer uses. The validator's
 * binary64 curve and its binary32 evaluation envelope come from the test API.
 * Every sample must satisfy |consumer - exact| <= envelope for position,
 * velocity and acceleration. The envelope is also checked to be non-zero, so a
 * future "fix" cannot satisfy the test by returning a zero margin.
 */
static void safety_point_set_full(MotionReferencePoint *point,
                                  const float position[3],
                                  const float velocity[3],
                                  const float acceleration[3])
{
  unsigned axis;
  safety_point_set(point, position[0], position[1], position[2],
                   velocity[0], velocity[1], velocity[2]);
  for (axis = 0u; axis < 3u; ++axis)
    point->point.acceleration_ned_mps2[axis] = acceleration[axis];
}

static void safety_reference_uniform(MotionReference *ref, uint64_t t0)
{
  unsigned i;
  memset(ref, 0, sizeof(*ref));
  ref->header.timestamp_sample_us = t0;
  ref->point_count = PRODUCT_PREVIEW_COUNT;
  ref->anchor.offset_us = 0u;
  safety_point_set(&ref->anchor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    ref->points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&ref->points[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

static float safety_fuzz_scaled(uint32_t *state, float scale)
{
  int32_t raw = (int32_t)(safety_fuzz_next(state) % 2001u) - 1000;
  return (float)raw * 0.001f * scale;
}

static void safety_check_consumer_curve(const MotionReference *ref,
                                        unsigned sample_count)
{
  const float h = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  unsigned i, k, axis;

  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    const MotionReferencePoint *a = i ? &ref->points[i - 1u] : &ref->anchor;
    const MotionReferencePoint *b = &ref->points[i];
    float position_error, velocity_error, acceleration_error;

    CHECK(m5_trajectory_safety_test_curve_envelope(
        a->point.position_ned_m, a->point.velocity_ned_mps,
        a->point.acceleration_ned_mps2, b->point.position_ned_m,
        b->point.velocity_ned_mps, b->point.acceleration_ned_mps2,
        h, 0.0f, 1.0f,
        &position_error, &velocity_error, &acceleration_error));
    CHECK(position_error > 0.0f);
    CHECK(velocity_error > 0.0f);
    CHECK(acceleration_error > 0.0f);

    for (k = 0u; k < sample_count; ++k) {
      uint32_t local_us = (uint32_t)(((uint64_t)k * PRODUCT_PREVIEW_STEP_US) /
                                     sample_count);
      uint64_t time = ref->header.timestamp_sample_us +
          (uint64_t)i * PRODUCT_PREVIEW_STEP_US + local_us;
      float local_u = (float)((double)local_us /
                             (double)PRODUCT_PREVIEW_STEP_US);
      ProductMotionPoint sampled;
      float position[3], velocity[3], acceleration[3];

      CHECK(control_sample_reference(ref, time, &sampled) == STATUS_OK);
      CHECK(m5_trajectory_safety_test_curve_eval(
          a->point.position_ned_m, a->point.velocity_ned_mps,
          a->point.acceleration_ned_mps2, b->point.position_ned_m,
          b->point.velocity_ned_mps, b->point.acceleration_ned_mps2,
          h, 0.0f, 1.0f, local_u, position, velocity, acceleration));
      for (axis = 0u; axis < 3u; ++axis) {
        CHECK(fabsf(sampled.position_ned_m[axis] -
                    position[axis]) <= position_error);
        CHECK(fabsf(sampled.velocity_ned_mps[axis] -
                    velocity[axis]) <= velocity_error);
        CHECK(fabsf(sampled.acceleration_ned_mps2[axis] -
                    acceleration[axis]) <= acceleration_error);
      }
    }
  }
}

static void test_m5_safety_consumer_curve_envelope(void)
{
  static const float p0[3] = {1.0f, -2.0f, -8.0f};
  static const float v0[3] = {3.0f, -1.0f, 0.5f};
  static const float a0[3] = {0.4f, -0.2f, 0.1f};
  static const float jerk[3] = {2.0f, -1.0f, 0.3f};
  const float h = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  MotionReference ref;
  uint32_t seed = 0x6D350686u;
  unsigned trial, i, axis;

  /* Smooth reference: bounded jerk over the full 480 ms horizon. */
  safety_reference_uniform(&ref, 7000000u);
  safety_point_set_full(&ref.anchor, p0, v0, a0);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float t = (float)(i + 1u) * h;
    float position[3], velocity[3], acceleration[3];
    for (axis = 0u; axis < 3u; ++axis) {
      position[axis] = p0[axis] + v0[axis] * t + 0.5f * a0[axis] * t * t +
                       jerk[axis] * t * t * t / 6.0f;
      velocity[axis] = v0[axis] + a0[axis] * t + 0.5f * jerk[axis] * t * t;
      acceleration[axis] = a0[axis] + jerk[axis] * t;
    }
    safety_point_set_full(&ref.points[i], position, velocity, acceleration);
  }
  safety_check_consumer_curve(&ref, 33u);

  /* Adversarial references: independent random p/v/a per knot, which is the
     worst case for coefficient cancellation in the binary32 consumer. */
  for (trial = 0u; trial < 8u; ++trial) {
    safety_reference_uniform(&ref, 8000000u + trial * 100000u);
    for (axis = 0u; axis < 3u; ++axis) {
      ref.anchor.point.position_ned_m[axis] = safety_fuzz_scaled(&seed, 5.0f);
      ref.anchor.point.velocity_ned_mps[axis] = safety_fuzz_scaled(&seed, 20.0f);
      ref.anchor.point.acceleration_ned_mps2[axis] =
          safety_fuzz_scaled(&seed, 10.0f);
    }
    for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
      for (axis = 0u; axis < 3u; ++axis) {
        ref.points[i].point.position_ned_m[axis] =
            safety_fuzz_scaled(&seed, 5.0f);
        ref.points[i].point.velocity_ned_mps[axis] =
            safety_fuzz_scaled(&seed, 20.0f);
        ref.points[i].point.acceleration_ned_mps2[axis] =
            safety_fuzz_scaled(&seed, 10.0f);
      }
    }
    safety_check_consumer_curve(&ref, 17u);
  }
}

/*
 * S06-087..091: end-to-end safety decisions for the consumed curve.
 *
 * S06-085/S06-086 prove the numeric envelope; these tests drive the real
 * m5_trajectory_safety_validate() decision. Obstacles are placed from the
 * consumer sampler or exactly on a preview knot, so accepting any of them would
 * mean the controller flies into the obstacle.
 *
 * Honest scope note: for a moving reference the DFS band
 * (speed * h / 2^5) is larger than the MR-009 envelope, so these tests prove
 * the decision path, not that the envelope alone flips it. Pointwise envelope
 * coverage is proven by S06-085 and S06-086.
 */
static void safety_zero_uncertainty(SafetyFx *fx)
{
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    fx->base.nav.core.position_std_m[axis] = 0.0f;
    fx->base.nav.core.velocity_std_mps[axis] = 0.0f;
  }
  fx->input.response_delay_us = 0u;
}

static void safety_generous_envelope(SafetyFx *fx, float speed, float accel)
{
  unsigned axis;
  fx->base.env.max_linear_speed_mps = speed;
  fx->base.env.max_linear_accel_mps2 = accel;
  for (axis = 0u; axis < 3u; ++axis) {
    fx->base.env.vehicle_extension.max_velocity_ned_mps[axis] = speed;
    fx->base.env.vehicle_extension.max_acceleration_ned_mps2[axis] = accel;
    /* These fixtures intentionally exercise kinematics, not the jerk limit. */
    fx->base.env.vehicle_extension.max_jerk_ned_mps3[axis] = 3.4028235e38f;
  }
}

static void safety_mark_occupied(SafetyFx *fx, float x, float y, float z)
{
  const float point[3] = {x, y, z};
  uint16_t voxel = 0u;
  CHECK(m5_local_map_world_to_voxel(&fx->map, point, &voxel));
  fx->map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  fx->map.cells[voxel].log_odds = 4;
  fx->map.cells[voxel].source_mask = 1u;
}

static void safety_linear_reference(SafetyFx *fx,
                                    float x0, float y0, float z0,
                                    float vx, float vy, float vz)
{
  const float step_s = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  unsigned i;
  fx->base.nav.core.position_ned_m[0] = x0;
  fx->base.nav.core.position_ned_m[1] = y0;
  fx->base.nav.core.position_ned_m[2] = z0;
  fx->base.nav.core.velocity_ned_mps[0] = vx;
  fx->base.nav.core.velocity_ned_mps[1] = vy;
  fx->base.nav.core.velocity_ned_mps[2] = vz;
  memset(fx->base.nav.core.acceleration_ned_mps2, 0,
         sizeof(fx->base.nav.core.acceleration_ned_mps2));
  safety_point_set(&fx->reference.anchor, x0, y0, z0, vx, vy, vz);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    float t = (float)(i + 1u) * step_s;
    fx->reference.points[i].offset_us = (i + 1u) * PRODUCT_PREVIEW_STEP_US;
    safety_point_set(&fx->reference.points[i],
                     x0 + vx * t, y0 + vy * t, z0 + vz * t, vx, vy, vz);
  }
  safety_map_reset(fx);
}

static void safety_keep_in_box(SafetyFx *fx, float half_width)
{
  fill_header(&fx->fence.header, MSG_PRODUCT_GEOFENCE_SNAPSHOT,
              SOURCE_M4, fx->base.now);
  fx->fence.fence_id = 91u;
  fx->fence.frame_id = FRAME_LOCAL_NED;
  fx->fence.origin_id = T_ORIGIN;
  fx->fence.state_epoch = T_STATE_EPOCH;
  fx->fence.validity = PRODUCT_GEOFENCE_VALID;
  fx->fence.kind = PRODUCT_GEOFENCE_KEEP_IN;
  fx->fence.vertex_count = 4u;
  fx->fence.horizontal_margin_m = 0.0f;
  fx->fence.vertical_margin_m = 0.0f;
  fx->fence.min_height_above_home_m = 0.0f;
  fx->fence.max_height_above_home_m = 50.0f;
  fx->fence.vertices_ne_m[0][0] = -2.0f;
  fx->fence.vertices_ne_m[0][1] = -half_width;
  fx->fence.vertices_ne_m[1][0] = 2.0f;
  fx->fence.vertices_ne_m[1][1] = -half_width;
  fx->fence.vertices_ne_m[2][0] = 2.0f;
  fx->fence.vertices_ne_m[2][1] = half_width;
  fx->fence.vertices_ne_m[3][0] = -2.0f;
  fx->fence.vertices_ne_m[3][1] = half_width;
  fx->input.geofences = &fx->fence;
  fx->input.geofence_count = 1u;
}

static void test_m5_safety_consumer_static_crossing_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 50000000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  /* v = 2 m/s passes x = 0.75 at t = 0.375 s, i.e. inside interval 18. */
  safety_linear_reference(&fx, 0.0f, 0.0f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  safety_mark_occupied(&fx, 0.75f, 0.0f, -5.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_STATIC_COLLISION);
}

static void test_m5_safety_consumer_static_clear_companion_passes(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 50500000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  safety_linear_reference(&fx, 0.0f, 0.0f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  /* Same trajectory, obstacle one cell to the side: 0.25 m AABB clearance. */
  safety_mark_occupied(&fx, 0.75f, 0.5f, -5.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);
}

static void test_m5_safety_consumer_preview_seam_rejected(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 51000000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  /* v = 2.5 m/s reaches x = 0.75 exactly at the interval-15 preview knot. */
  safety_linear_reference(&fx, 0.0f, 0.0f, -5.0f, 2.5f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  safety_mark_occupied(&fx, 0.75f, 0.0f, -5.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_consumer_adaptive_subdivision(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 51500000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  /* Line at y = -0.16; occupied cell centre y = 0.25 -> AABB distance 0.16,
     inside the 0.15 + swept(0.02) unresolved band but outside the inflated
     boundary, so the DFS must subdivide and still prove clearance. */
  safety_linear_reference(&fx, 0.0f, -0.16f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  safety_mark_occupied(&fx, 0.75f, 0.25f, -5.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);
  CHECK(fx.result.work_units > PRODUCT_PREVIEW_COUNT);

  /* 0.149 m from the inflated boundary: the same cell must now be rejected
     without relying on subdivision. */
  safety_fx_init(&fx, 51600000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  safety_linear_reference(&fx, 0.0f, -0.149f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  safety_mark_occupied(&fx, 0.75f, 0.25f, -5.0f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_consumer_dynamic_track_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  safety_fx_init(&fx, 52000000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  safety_linear_reference(&fx, 0.0f, 0.0f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  track = &fx.map.tracks[0];
  track->id = 71u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 900000u;
  track->radius_m = 0.05f;
  track->position_ned_m[0] = 0.5f;
  track->position_ned_m[1] = 0.0f;
  track->position_ned_m[2] = -5.0f;
  memset(track->velocity_ned_mps, 0, sizeof(track->velocity_ned_mps));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_DYNAMIC_COLLISION);
}

static void test_m5_safety_consumer_geofence_boundary(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 53000000u);
  safety_generous_envelope(&fx, 10.0f, 50.0f);
  safety_linear_reference(&fx, 0.0f, 0.0f, -5.0f, 2.0f, 0.0f, 0.0f);
  safety_zero_uncertainty(&fx);
  /* Inflated boundary is 0.15 m (vehicle radius 0.10 + clearance 0.05). */
  safety_keep_in_box(&fx, 0.149f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_GEOFENCE_VIOLATION);
  CHECK(fx.result.passed == 0u);
  /* Same path with 0.05 m of margin must still pass. */
  safety_keep_in_box(&fx, 0.20f);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) == M5_SAFETY_PASS);
  CHECK(fx.result.passed == 1u);
}

static void test_m5_safety_consumer_max_deviation_rejected(void)
{
  SafetyFx fx;
  M5DynamicTrack *unused = NULL;
  ProductMotionPoint sampled;
  uint32_t seed = 0x6D350091u;
  uint64_t mid_time;
  unsigned i, axis;
  (void)unused;

  safety_fx_init(&fx, 54000000u);
  safety_generous_envelope(&fx, 200.0f, 20000.0f);
  /* Independent random p/v/a on every knot: worst case for binary32
     coefficient cancellation, but bounded so the sampled curve stays in the
     local map. */
  for (axis = 0u; axis < 3u; ++axis) {
    fx.reference.anchor.point.position_ned_m[axis] =
        safety_fuzz_scaled(&seed, 0.5f);
    fx.reference.anchor.point.velocity_ned_mps[axis] =
        safety_fuzz_scaled(&seed, 20.0f);
    fx.reference.anchor.point.acceleration_ned_mps2[axis] =
        safety_fuzz_scaled(&seed, 10.0f);
  }
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
    for (axis = 0u; axis < 3u; ++axis) {
      fx.reference.points[i].point.position_ned_m[axis] =
          safety_fuzz_scaled(&seed, 0.5f);
      fx.reference.points[i].point.velocity_ned_mps[axis] =
          safety_fuzz_scaled(&seed, 20.0f);
      fx.reference.points[i].point.acceleration_ned_mps2[axis] =
          safety_fuzz_scaled(&seed, 10.0f);
    }
  }
  fx.base.nav.core.position_ned_m[0] = fx.reference.anchor.point.position_ned_m[0];
  fx.base.nav.core.position_ned_m[1] = fx.reference.anchor.point.position_ned_m[1];
  fx.base.nav.core.position_ned_m[2] = fx.reference.anchor.point.position_ned_m[2];
  fx.base.nav.core.velocity_ned_mps[0] = fx.reference.anchor.point.velocity_ned_mps[0];
  fx.base.nav.core.velocity_ned_mps[1] = fx.reference.anchor.point.velocity_ned_mps[1];
  fx.base.nav.core.velocity_ned_mps[2] = fx.reference.anchor.point.velocity_ned_mps[2];
  fx.base.nav.core.acceleration_ned_mps2[0] =
      fx.reference.anchor.point.acceleration_ned_mps2[0];
  fx.base.nav.core.acceleration_ned_mps2[1] =
      fx.reference.anchor.point.acceleration_ned_mps2[1];
  fx.base.nav.core.acceleration_ned_mps2[2] =
      fx.reference.anchor.point.acceleration_ned_mps2[2];
  safety_map_reset(&fx);
  safety_zero_uncertainty(&fx);

  /* Put the obstacle exactly on the consumed curve, mid interval 11. */
  mid_time = fx.reference.header.timestamp_sample_us +
      11u * PRODUCT_PREVIEW_STEP_US + PRODUCT_PREVIEW_STEP_US / 2u;
  CHECK(control_sample_reference(&fx.reference, mid_time, &sampled) == STATUS_OK);
  safety_mark_occupied(&fx, sampled.position_ned_m[0],
                       sampled.position_ned_m[1], sampled.position_ned_m[2]);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace,
      &fx.result) != M5_SAFETY_PASS);
  CHECK(fx.result.passed == 0u);
}

static void test_m5_safety_floating_environment_rejected(void)
{
  SafetyFx fx;
  int previous = fegetround();
  static const int modes[] = {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO};
  unsigned i;

  safety_fx_init(&fx, 55000000u);
  for (i = 0u; i < sizeof(modes) / sizeof(modes[0]); ++i) {
    CHECK(fesetround(modes[i]) == 0);
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_NUMERIC_FAILURE);
    CHECK(fx.result.passed == 0u);
    CHECK(fesetround(previous) == 0);
  }
#if (defined(__x86_64__) || defined(__i386__)) && defined(__SSE__)
  {
    unsigned original = (unsigned)_mm_getcsr();
    _mm_setcsr(original | (1u << 15)); /* FTZ */
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_NUMERIC_FAILURE);
    _mm_setcsr((original & ~(1u << 15)) | (1u << 6)); /* DAZ */
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_NUMERIC_FAILURE);
    _mm_setcsr(original);
  }
#endif
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_PASS);
}

/*
 * S06-095..097: prove the MR-009 envelope participates in the decision.
 *
 * Each case places the boundary object at `inflation + d/2` from the binary64
 * midpoint, where d is the measured |consumer - exact| at that midpoint:
 *   - the exact curve clears the boundary by d/2 (no mathematical collision);
 *   - the consumed binary32 curve is d closer, so it penetrates by d/2;
 *   - the published envelope E >= ~10 d, so the validator must reject.
 * The status must be the specific collision reason, not merely "not PASS", and
 * a companion case with a larger margin must pass.
 */
static float safety_cell_distance(const float point[3], const float center[3])
{
  double sum = 0.0;
  unsigned axis;
  for (axis = 0u; axis < 3u; ++axis) {
    double separation = fabs((double)point[axis] - (double)center[axis]) -
                        0.5 * (double)M5_MAP_CELL_SIZE_M;
    if (separation > 0.0) sum += separation * separation;
  }
  return (float)sqrt(sum);
}

/*
 * S06-095..097: prove the MR-009 envelope participates in the decision.
 *
 * Fixture: a velocity-split segment (anchor v = 0, every preview point v = 50)
 * keeps the curve inside the local map while producing large binary32
 * coefficients, so the published envelope E is far above the geometry
 * resolution. The boundary object is placed at `inflation + 0.8*E` from the
 * binary64 midpoint:
 *   - the exact curve clears the boundary by 0.8*E > 0 (no mathematical hit);
 *   - the validator subtracts E from the measured distance, so it must reject;
 *   - without the envelope the same geometry would pass.
 * That is what makes these tests a proof that the envelope participates rather
 * than a proof that some other check fires. The consumer collision margin
 * (|consumer - exact|) is at float resolution for this fixture, so pointwise
 * consumer coverage is proven separately by S06-085/S06-086/S06-094.
 */
static void safety_velocity_split_reference(SafetyFx *fx, uint64_t now_us)
{
  static const float p[3] = {0.0f, 0.0f, -5.0f};
  static const float v_anchor[3] = {0.0f, 0.0f, 0.0f};
  static const float v_point[3] = {0.0f, 50.0f, 0.0f};
  static const float zero[3] = {0.0f, 0.0f, 0.0f};
  unsigned i, axis;

  safety_fx_init(fx, now_us);
  safety_generous_envelope(fx, 1000.0f, 100000.0f);
  for (axis = 0u; axis < 3u; ++axis) {
    fx->base.nav.core.position_ned_m[axis] = p[axis];
    fx->base.nav.core.velocity_ned_mps[axis] = v_anchor[axis];
    fx->base.nav.core.acceleration_ned_mps2[axis] = zero[axis];
  }
  safety_point_set_full(&fx->reference.anchor, p, v_anchor, zero);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i)
    safety_point_set_full(&fx->reference.points[i], p, v_point, zero);
  safety_map_reset(fx);
  safety_zero_uncertainty(fx);
}

static void safety_segment_midpoint(const SafetyFx *fx, unsigned interval,
                                    float exact[3], float consumer[3])
{
  const MotionReferencePoint *a = interval ? &fx->reference.points[interval - 1u]
                                           : &fx->reference.anchor;
  const MotionReferencePoint *b = &fx->reference.points[interval];
  const float h = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  uint64_t time = fx->reference.header.timestamp_sample_us +
      (uint64_t)interval * PRODUCT_PREVIEW_STEP_US + PRODUCT_PREVIEW_STEP_US / 2u;
  ProductMotionPoint sampled;
  float velocity[3], acceleration[3];

  CHECK(m5_trajectory_safety_test_curve_eval(
      a->point.position_ned_m, a->point.velocity_ned_mps,
      a->point.acceleration_ned_mps2, b->point.position_ned_m,
      b->point.velocity_ned_mps, b->point.acceleration_ned_mps2,
      h, 0.0f, 1.0f, 0.5f, exact, velocity, acceleration));
  CHECK(control_sample_reference(&fx->reference, time, &sampled) == STATUS_OK);
  consumer[0] = sampled.position_ned_m[0];
  consumer[1] = sampled.position_ned_m[1];
  consumer[2] = sampled.position_ned_m[2];
}

static float safety_interval_envelope(SafetyFx *fx, unsigned interval)
{
  const MotionReferencePoint *a = interval ? &fx->reference.points[interval - 1u]
                                           : &fx->reference.anchor;
  const MotionReferencePoint *b = &fx->reference.points[interval];
  const float h = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  float position_error, velocity_error, acceleration_error;
  CHECK(m5_trajectory_safety_test_curve_envelope(
      a->point.position_ned_m, a->point.velocity_ned_mps,
      a->point.acceleration_ned_mps2, b->point.position_ned_m,
      b->point.velocity_ned_mps, b->point.acceleration_ned_mps2,
      h, 0.0f, 1.0f, &position_error, &velocity_error, &acceleration_error));
  CHECK(position_error > 0.0f);
  CHECK(velocity_error > 0.0f);
  CHECK(acceleration_error > 0.0f);
  return position_error;
}

static void safety_map_with_margin(SafetyFx *fx, float margin)
{
  unsigned i;
  CHECK(m5_local_map_init(&fx->map, &fx->base.nav, fx->base.now,
      0.10f, margin, 1.0f, M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) == STATUS_OK);
  for (i = 0u; i < M5_MAP_CELL_COUNT; ++i) {
    fx->map.cells[i].state = M5_MAP_CELL_FREE_CONFIRMED;
    fx->map.cells[i].log_odds = -4;
    fx->map.cells[i].source_mask = 1u;
    fx->map.cells[i].last_update_ms = (uint32_t)(fx->base.now / 1000u);
  }
  fx->map.last_ingest_us = fx->base.now;
  fx->map.source_deadline_us = fx->base.now + 1000000u;
  fx->input.map = &fx->map;
  fx->input.expected_map_revision = fx->map.map_revision;
}

static void test_m5_safety_envelope_static_participation(void)
{
  SafetyFx fx;
  float exact[3], consumer[3], center[3], probe[3];
  float envelope, distance, margin;
  uint16_t voxel = 0u;

  safety_velocity_split_reference(&fx, 60000000u);
  envelope = safety_interval_envelope(&fx, 0u);
  safety_segment_midpoint(&fx, 0u, exact, consumer);

  probe[0] = exact[0];
  probe[1] = exact[1] + 1.0f;
  probe[2] = exact[2];
  CHECK(m5_local_map_world_to_voxel(&fx.map, probe, &voxel));
  m5_local_map_voxel_center(&fx.map, voxel, center);
  distance = safety_cell_distance(exact, center);
  CHECK(distance > 0.30f);

  margin = distance - 0.8f * envelope - 0.10f;
  CHECK(margin > 0.02f);
  safety_map_with_margin(&fx, margin);
  fx.map.cells[voxel].state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  fx.map.cells[voxel].log_odds = 4;
  fx.map.cells[voxel].source_mask = 1u;

  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_STATIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_STATIC_COLLISION);
}

static void test_m5_safety_envelope_dynamic_participation(void)
{
  SafetyFx fx;
  M5DynamicTrack *track;
  float exact[3], consumer[3];
  float envelope, target;
  const float inflation = 0.15000000596f;
  const float radius = 0.05f;
  double distance2;

  safety_velocity_split_reference(&fx, 61000000u);
  envelope = safety_interval_envelope(&fx, 0u);
  safety_segment_midpoint(&fx, 0u, exact, consumer);
  target = radius + inflation + 0.8f * envelope;

  track = &fx.map.tracks[0];
  track->id = 81u;
  track->source_id = 2u;
  track->sample_us = fx.base.now;
  track->expires_us = fx.base.now + 900000u;
  track->radius_m = radius;
  track->position_ned_m[0] = exact[0];
  track->position_ned_m[1] = exact[1] + target;
  track->position_ned_m[2] = exact[2];
  memset(track->velocity_ned_mps, 0, sizeof(track->velocity_ned_mps));

  distance2 = 0.0;
  {
    unsigned axis;
    for (axis = 0u; axis < 3u; ++axis) {
      double difference = (double)exact[axis] - (double)track->position_ned_m[axis];
      distance2 += difference * difference;
    }
  }
  CHECK(sqrt(distance2) > (double)(radius + inflation));
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_DYNAMIC_COLLISION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_DYNAMIC_COLLISION);
}

static void test_m5_safety_envelope_geofence_participation(void)
{
  SafetyFx fx;
  float exact[3], consumer[3];
  float envelope, target;
  const float inflation = 0.15000000596f;

  safety_velocity_split_reference(&fx, 62000000u);
  envelope = safety_interval_envelope(&fx, 0u);
  safety_segment_midpoint(&fx, 0u, exact, consumer);
  /* Keep-in box whose +y edge is `target` above the midpoint; the curve stays
     at y = 0 for this fixture, so the clearance is exactly `target`. */
  target = inflation + 0.8f * envelope;
  safety_keep_in_box(&fx, exact[1] + target);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_GEOFENCE_VIOLATION);
  CHECK(fx.result.passed == 0u);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_GEOFENCE_VIOLATION);
}





/* S06-110: production jerk enforcement (spec/02 JERK bit). */
static void test_m5_safety_jerk_envelope(void)
{
  SafetyFx fx;
  unsigned axis;
  static const float zero[3] = {0.0f, 0.0f, 0.0f};

  for (axis = 0u; axis < 3u; ++axis) {
    float position[3] = {0.0f, 0.0f, -5.0f};
    float start_acceleration[3] = {0.0f, 0.0f, 0.0f};
    float end_acceleration[3] = {0.0f, 0.0f, 0.0f};
    unsigned i;

    safety_fx_init(&fx, 95000000u + axis);
    safety_generous_envelope(&fx, 1000.0f, 100000.0f);
    fx.base.env.vehicle_extension.max_jerk_ned_mps3[0] = 10.0f;
    fx.base.env.vehicle_extension.max_jerk_ned_mps3[1] = 10.0f;
    fx.base.env.vehicle_extension.max_jerk_ned_mps3[2] = 5.0f;
    start_acceleration[axis] = 400.0f;
    end_acceleration[axis] = -400.0f;
    for (i = 0u; i < 3u; ++i) {
      fx.base.nav.core.position_ned_m[i] = position[i];
      fx.base.nav.core.velocity_ned_mps[i] = zero[i];
      fx.base.nav.core.acceleration_ned_mps2[i] = start_acceleration[i];
    }
    safety_point_set_full(&fx.reference.anchor, position, zero,
                          start_acceleration);
    for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i)
      safety_point_set_full(&fx.reference.points[i], position, zero,
                            end_acceleration);
    safety_map_reset(&fx);
    safety_zero_uncertainty(&fx);

    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_DYNAMICS_VIOLATION);
    CHECK(fx.result.reason_code == M5_SAFETY_REASON_DYNAMICS);

    fx.base.env.vehicle_extension.max_jerk_ned_mps3[axis] = 1.0e9f;
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_PASS);

    fx.base.env.vehicle_extension.max_jerk_ned_mps3[axis] = 0.0f / 0.0f;
    CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
          == M5_SAFETY_INVALID);
  }
}

/* S06-110b: the R14 formula used 6 instead of 60 and a normalized gamma_2
   tail, which under-bounds the true third derivative by up to 10x. This
   deterministic curve has true max |jerk| = 18 m/s^3 (27 m/s^3 with the
   conservative Bernstein bound), so a 16 m/s^3 product limit must reject it;
   a permissive limit remains PASS. */
static void test_m5_safety_jerk_interval_bound(void)
{
  SafetyFx fx;
  static const float position[3] = {0.0f, 0.0f, -5.0f};
  static const float velocity[3] = {0.0f, 0.0f, 0.0f};
  static const float start_acceleration[3] = {0.03f, 0.0f, 0.0f};
  static const float end_acceleration[3] = {-0.03f, 0.0f, 0.0f};
  unsigned i;

  safety_fx_init(&fx, 95500000u);
  safety_generous_envelope(&fx, 1000.0f, 100000.0f);
  fx.base.env.vehicle_extension.max_jerk_ned_mps3[0] = 16.0f;
  fx.base.env.vehicle_extension.max_jerk_ned_mps3[1] = 1.0e9f;
  fx.base.env.vehicle_extension.max_jerk_ned_mps3[2] = 1.0e9f;
  for (i = 0u; i < 3u; ++i) {
    fx.base.nav.core.position_ned_m[i] = position[i];
    fx.base.nav.core.velocity_ned_mps[i] = velocity[i];
    fx.base.nav.core.acceleration_ned_mps2[i] = start_acceleration[i];
  }
  safety_point_set_full(&fx.reference.anchor, position, velocity,
                        start_acceleration);
  for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i)
    safety_point_set_full(&fx.reference.points[i], position, velocity,
                          end_acceleration);
  safety_map_reset(&fx);
  safety_zero_uncertainty(&fx);

  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_DYNAMICS_VIOLATION);
  CHECK(fx.result.reason_code == M5_SAFETY_REASON_DYNAMICS);

  fx.base.env.vehicle_extension.max_jerk_ned_mps3[0] = 1.0e9f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_PASS);
}

/* S06-110c: a declared JERK constraint with a zero limit is not "no limit";
   it is an invalid configuration and must fail closed. */
static void test_m5_safety_jerk_zero_limit_invalid(void)
{
  SafetyFx fx;
  safety_fx_init(&fx, 95600000u);
  safety_generous_envelope(&fx, 1000.0f, 100000.0f);
  fx.base.env.vehicle_extension.max_jerk_ned_mps3[0] = 0.0f;
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_INVALID);
  CHECK(fx.result.passed == 0u);
}

/* S06-111: the Host/SIL M5-06 suite links the real QUAD_X4 product builder.
   The test API and the validator fixtures therefore see the same SafetyEnvelope
   as the FMS, not the deleted static 4/2.5/20/10 copy. */
static void test_m5_safety_product_envelope_source(void)
{
  SafetyFx fx;
  ProductFmsConfig config;
  SafetyEnvelope envelope;
  unsigned axis;
  float speed = 0.0f, accel = 0.0f;
  float velocity[3], acceleration[3], jerk[3];

  fms_model_default_config(&config);
  CHECK(fms_product_build_safety_envelope(&fms_quad_x4_policy, &config, 1u,
                                          &envelope) == STATUS_OK);
  CHECK(envelope.safety_envelope_id == 1u);
  CHECK((envelope.vehicle_extension.valid_mask & PRODUCT_ENVELOPE_VALID_JERK) != 0u);
  CHECK(envelope.max_linear_speed_mps == fms_quad_x4_policy.default_max_speed_mps);
  CHECK(envelope.max_linear_accel_mps2 == fms_quad_x4_policy.default_max_accel_mps2);
  CHECK(envelope.vehicle_extension.max_jerk_ned_mps3[0] ==
        fms_quad_x4_policy.default_max_accel_mps2 * 4.0f);
  CHECK(envelope.vehicle_extension.max_jerk_ned_mps3[1] ==
        fms_quad_x4_policy.default_max_accel_mps2 * 4.0f);
  CHECK(envelope.vehicle_extension.max_jerk_ned_mps3[2] ==
        fms_quad_x4_policy.default_max_accel_mps2 * 2.0f);

  CHECK(m5_trajectory_safety_test_product_limits(
      &speed, &accel, velocity, acceleration, jerk));
  CHECK(speed == envelope.max_linear_speed_mps);
  CHECK(accel == envelope.max_linear_accel_mps2);
  for (axis = 0u; axis < 3u; ++axis) {
    CHECK(velocity[axis] ==
          envelope.vehicle_extension.max_velocity_ned_mps[axis]);
    CHECK(acceleration[axis] ==
          envelope.vehicle_extension.max_acceleration_ned_mps2[axis]);
    CHECK(jerk[axis] == envelope.vehicle_extension.max_jerk_ned_mps3[axis]);
  }

  /* Consume the real product envelope in the validator. */
  safety_fx_init(&fx, 95700000u);
  fx.base.env = envelope;
  fill_header(&fx.base.env.header, MSG_SAFETY_ENVELOPE, SOURCE_M4, fx.base.now);
  safety_set_constant_position(&fx, 0.0f, 0.0f, -5.0f);
  safety_map_reset(&fx);
  safety_zero_uncertainty(&fx);
  CHECK(m5_trajectory_safety_validate(&fx.input, &fx.workspace, &fx.result)
        == M5_SAFETY_PASS);
}

/* S06-108 C path: the real consumer is driven with a unit-velocity reference,
   so its x output is exactly the binary32 local time. The same gamma_2
   predicate used by the Python oracle is exercised in C, including a 4x
   gamma_2 injection that must be rejected. */
static void test_m5_safety_consumer_time_error_c_path(void)
{
  static const uint32_t steps[2] = {20000u, 40000u};
  static const uint32_t offsets[4] = {0u, 10000u, 19999u, 20000u};
  const double gamma2 = (2.0 * 0x1p-24) / (1.0 - 2.0 * 0x1p-24);
  unsigned s, k, i;

  for (s = 0u; s < 2u; ++s) {
    MotionReference ref;
    memset(&ref, 0, sizeof(ref));
    ref.header.timestamp_sample_us = 1000000u;
    ref.point_count = PRODUCT_PREVIEW_COUNT;
    ref.anchor.offset_us = 0u;
    safety_point_set(&ref.anchor, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    for (i = 0u; i < PRODUCT_PREVIEW_COUNT; ++i) {
      float t = (float)((double)(i + 1u) * (double)steps[s] * 1.0e-6);
      ref.points[i].offset_us = (i + 1u) * steps[s];
      safety_point_set(&ref.points[i], t, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    }
    for (k = 0u; k < 4u; ++k) {
      ProductMotionPoint out;
      double s_c, s_exact, bound, injected;
      uint32_t offset_us = offsets[k] <= steps[s] ? offsets[k] : steps[s];
      CHECK(control_sample_reference(&ref, ref.header.timestamp_sample_us +
                                     offset_us, &out) == STATUS_OK);
      s_c = (double)out.position_ned_m[0];
      s_exact = (double)offset_us / 1000000.0;
      bound = gamma2 * s_exact;
      CHECK(fabs(s_c - s_exact) <= bound);
      if (s_exact > 0.0) {
        injected = s_c + 4.0 * bound;
        CHECK(fabs(injected - s_exact) > bound);
      }
    }
  }
}

/* S06-110d: the continuous jerk bound must cover the analytic third
   derivative on the whole sub-interval and on the consumer gamma_2 tail. The
   states include the 6-vs-60 counterexample and near-product-limit axes. */
static void test_m5_safety_jerk_bound_covers_analytic(void)
{
  static const float states[4][6][3] = {
    {{0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, {0.03f, 0.0f, 0.0f},
     {0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, {-0.03f, 0.0f, 0.0f}},
    {{1.0f, -2.0f, -4.0f}, {2.0f, 1.0f, 0.5f}, {1.0f, 0.5f, 0.0f},
     {1.05f, -1.98f, -4.0f}, {1.5f, 0.8f, 0.4f}, {0.5f, 0.2f, 0.0f}},
    {{0.0f, 0.0f, -5.0f}, {0.0f, 0.0f, 0.0f}, {4.0f, 4.0f, 2.0f},
     {0.1f, 0.1f, -5.0f}, {0.0f, 0.0f, 0.0f}, {-4.0f, -4.0f, -2.0f}},
    {{-1.0f, 2.0f, -4.5f}, {0.0f, 3.5f, 0.0f}, {0.0f, -2.5f, 0.0f},
     {-1.0f, 2.05f, -4.5f}, {0.0f, 2.0f, 0.0f}, {0.0f, -2.0f, 0.0f}},
  };
  static const float spans[5][2] = {
    {0.0f, 1.0f}, {0.0f, 0.5f}, {0.25f, 0.75f}, {0.5f, 0.75f}, {0.75f, 1.0f}
  };
  const float h = (float)PRODUCT_PREVIEW_STEP_US * 1e-6f;
  const double gamma2 = (2.0 * 0x1p-24) / (1.0 - 2.0 * 0x1p-24);
  unsigned s, span, axis, sample;

  for (s = 0u; s < 4u; ++s) {
    float coefficients[3][6];
    control_reference_hermite_coefficients(
        states[s][0], states[s][1], states[s][2],
        states[s][3], states[s][4], states[s][5], h, coefficients);
    for (span = 0u; span < 5u; ++span) {
      float u0 = spans[span][0], u1 = spans[span][1];
      float bound[3];
      unsigned tail_samples = 4u;
      CHECK(m5_trajectory_safety_test_curve_jerk(
          states[s][0], states[s][1], states[s][2],
          states[s][3], states[s][4], states[s][5],
          h, u0, u1, bound));
      for (axis = 0u; axis < 3u; ++axis) {
        CHECK(bound[axis] >= 0.0f);
        CHECK(isfinite(bound[axis]));
      }
      for (sample = 0u; sample < 66u + tail_samples; ++sample) {
        double u;
        double t;
        if (sample < 65u) {
          u = (double)u0 + ((double)u1 - (double)u0) *
              ((double)sample / 64.0);
        } else {
          double delta_u = gamma2 * (double)u1 /
              ((double)u1 - (double)u0);
          u = (double)u1 + delta_u *
              ((double)(sample - 64u) / (double)tail_samples);
        }
        t = (double)h * u;
        for (axis = 0u; axis < 3u; ++axis) {
          double jerk = 6.0 * (double)coefficients[axis][3] +
                        24.0 * (double)coefficients[axis][4] * t +
                        60.0 * (double)coefficients[axis][5] * t * t;
          CHECK(fabs(jerk) <= (double)bound[axis] * 1.000001 + 1e-7);
        }
      }
    }
  }
}

int main(void)
{
  struct {
    const char *name;
    void (*fn)(void);
  } tests[] = {
    {"hold_reference_shape", test_hold_reference_shape},
    {"landing_alignment_and_speed", test_landing_aligns_and_limits_final_descent},
    {"trajectory_handoff_acceleration", test_trajectory_handoff_uses_active_reference_acceleration},
    {"mission_reference_token", test_mission_reference_and_token},
    {"mission_final_time_bin", test_mission_uses_final_time_bin_before_rejecting},
    {"mission_leg_time_window_reject", test_mission_leg_beyond_time_window_rejected},
    {"return_time_window_reject", test_return_leg_beyond_time_window_rejected},
    {"hold_at_end_of_long_leg", test_hold_at_end_of_long_leg},
    {"reference_epoch_rotate", test_reference_epoch_stable_and_rotates},
    {"permit_activation", test_permit_matching_activates},
    {"avoidance_rejected", test_avoidance_mission_rejected},
    {"geofence_violation", test_geofence_violation_rejected},
    {"pilot_rate", test_pilot_rate_reference},
    {"pilot_altitude", test_pilot_altitude_masks},
    {"stale_time_epochs", test_stale_time_and_epochs_rejected},
    {"extreme_target_rejected", test_reference_expiry_and_dynamics},
    {"config_all_axes_sources", test_config_all_axes_and_unsupported_sources},
    {"input_authorization_fail_closed", test_input_and_authorization_fail_closed},
    {"continuous_rolling_reference", test_reference_rolls_on_one_continuous_plan},
    {"nonzero_hold_reset", test_hold_latches_nonzero_and_reset_rotates},
    {"permit_identity_state_policy", test_permit_identity_state_policy_revocation},
    {"fence_geometry_home_sign", test_fence_geometry_and_home_down_sign},
    {"continuous_fence_crossing", test_continuous_fence_crossing_between_samples},
    {"consumed_speed_spike_thrust", test_consumed_curve_speed_spike_and_thrust},
    {"bernstein_analytic_bounds", test_bernstein_bounds_cover_analytic_extrema},
    {"pilot_yaw_altitude_hold", test_pilot_yaw_integrates_and_altitude_centre_holds},
    {"minimum_jerk_single_segment", test_minimum_jerk_single_segment_boundary},
    {"minimum_jerk_two_segment", test_minimum_jerk_two_segment_continuity_and_cost},
    {"minimum_jerk_nonuniform_multiaxis", test_minimum_jerk_nonuniform_multiaxis},
    {"minimum_jerk_eight_segments", test_minimum_jerk_eight_segments_deterministic},
    {"minimum_jerk_invalid_inputs", test_minimum_jerk_rejects_invalid_inputs_atomically},
    {"minimum_jerk_objective_three_axes", test_minimum_jerk_objective_sums_three_axes},
    {"minimum_jerk_nonzero_endpoints", test_minimum_jerk_nonzero_endpoint_boundary_states},
    {"m5_map_fixed_grid_bounds", test_m5_map_fixed_grid_and_bounds},
    {"m5_map_origin_cell_aligned", test_m5_map_initial_origin_is_cell_aligned},
    {"m5_map_roll_preserves_world_cells", test_m5_map_roll_preserves_world_cells},
    {"m5_map_free_ray_stale", test_m5_map_free_ray_log_odds_and_stale},
    {"m5_map_source_lease_refresh", test_m5_map_source_lease_refresh},
    {"m5_map_occupied_dynamic_track", test_m5_map_occupied_primitive_and_dynamic_track},
    {"m5_map_epoch_reject", test_m5_map_epoch_reset_and_reject_source},
    {"m5_map_batch_fail_closed", test_m5_map_batch_validation_is_fail_closed},
    {"m5_astar_key_primitives", test_m5_kinodynamic_key_and_primitive_manifest},
    {"m5_time_scaler_bounded_growth", test_m5_time_scaler_bounded_growth},
    {"m5_time_scaler_yaw_rate", test_m5_time_scaler_enforces_yaw_rate},
    {"m5_time_scaler_capacity_envelope", test_m5_time_scaler_capacity_and_envelope_fail_closed},
    {"m5_time_scaler_global_vector_limits", test_m5_time_scaler_enforces_global_vector_limits},
    {"m5_smoother_minimum_jerk_continuity", test_m5_smoother_minimum_jerk_continuity},
    {"m5_smoother_stale_invalid", test_m5_smoother_rejects_stale_and_invalid_path},
    {"m5_astar_one_edge_deterministic", test_m5_astar_finds_deterministic_one_edge_path},
    {"m5_astar_rolling_world_coordinates", test_m5_astar_uses_rolling_world_coordinates},
    {"m5_astar_unknown_deadline", test_m5_astar_unknown_space_and_deadline_fail_closed},
    {"m5_astar_future_stale_map", test_m5_astar_rejects_future_stale_map_cells},
    {"m5_astar_stale_obstacle_barrier", test_m5_astar_rejects_stale_map_and_obstacle_barrier},
    {"m5_smoother_single_edge", test_m5_smoother_builds_single_edge_candidate},
    {"m5_smoother_two_segment", test_m5_smoother_optimizes_two_segment_path_continuously},
    {"m5_smoother_fail_closed", test_m5_smoother_rejects_stale_and_mismatched_paths},
    {"m5_safety_static_free_pass", test_m5_safety_static_free_reference_passes},
    {"m5_safety_dynamic_delay", test_m5_safety_dynamic_response_delay_is_inflated},
    {"m5_safety_unknown_fail_closed", test_m5_safety_unknown_voxel_fails_closed},
    {"m5_safety_occupied_static_voxel", test_m5_safety_occupied_static_voxel_rejected},
    {"m5_safety_nan_reject", test_m5_safety_nan_preview_rejected},
    {"m5_safety_reference_ttl", test_m5_safety_reference_expiring_before_proof_end_rejected},
    {"m5_safety_keepout_crossing", test_m5_safety_keep_out_crossing_between_preview_endpoints},
    {"m5_safety_stale_voxel", test_m5_safety_stale_voxel_fails_closed},
    {"m5_safety_invalid_voxel", test_m5_safety_invalid_voxel_encoding_rejected},
    {"m5_safety_dynamic_midpoint", test_m5_safety_dynamic_midpoint_collision_rejected},
    {"m5_safety_dynamic_crossing", test_m5_safety_dynamic_crossing_collision_rejected},
    {"m5_safety_static_midsegment", test_m5_safety_static_collision_between_preview_endpoints},
    {"m5_safety_static_seam_adaptive_split", test_m5_safety_static_collision_at_preview_seam_adaptive_split},
    {"m5_safety_static_voxel_sweep_differential", test_m5_safety_static_voxel_sweep_differential},
    {"m5_safety_arbitrary_hermite_swept_volume_differential", test_m5_safety_arbitrary_hermite_swept_volume_differential},
    {"m5_safety_keepin_excursion", test_m5_safety_keep_in_curve_excursion_rejected},
    {"m5_safety_dynamic_expiry", test_m5_safety_dynamic_track_expiry_rejected},
    {"m5_safety_duplicate_track", test_m5_safety_duplicate_dynamic_track_rejected},
    {"m5_safety_track_metadata", test_m5_safety_dynamic_track_invalid_metadata_fails_closed},
    {"m5_safety_track_accel_bound_required", test_m5_safety_dynamic_track_requires_acceleration_bound},
    {"m5_safety_track_future_sample", test_m5_safety_dynamic_track_future_sample_rejected},
    {"m5_safety_track_overage", test_m5_safety_dynamic_track_overage_rejected},
    {"m5_safety_track_nonfinite", test_m5_safety_dynamic_track_nonfinite_position_rejected},
    {"m5_safety_track_negative_radius", test_m5_safety_dynamic_track_negative_radius_rejected},
    {"m5_safety_track_nonfinite_velocity", test_m5_safety_dynamic_track_nonfinite_velocity_rejected},
    {"m5_safety_map_edge", test_m5_safety_map_edge_fails_closed},
    {"m5_safety_fence_collinear", test_m5_safety_geofence_collinear_vertices_rejected},
    {"m5_safety_response_limit", test_m5_safety_response_delay_limit_rejected},
    {"m5_safety_preview_count", test_m5_safety_preview_count_mismatch_rejected},
    {"m5_safety_inf_reject", test_m5_safety_infinite_preview_rejected},
    {"m5_safety_offset_reject", test_m5_safety_preview_offset_mismatch_rejected},
    {"m5_safety_dynamic_accel_bound", test_m5_safety_dynamic_acceleration_bound_is_inflated},
    {"m5_safety_nav_validity", test_m5_safety_navigation_missing_required_validity_rejected},
    {"m5_safety_nav_state_epoch", test_m5_safety_navigation_state_epoch_mismatch_rejected},
    {"m5_safety_nav_origin", test_m5_safety_navigation_origin_mismatch_rejected},
    {"m5_safety_nav_frame", test_m5_safety_navigation_frame_mismatch_rejected},
    {"m5_safety_fence_origin", test_m5_safety_geofence_origin_mismatch_rejected},
    {"m5_safety_fence_duplicate_vertex", test_m5_safety_geofence_duplicate_vertex_rejected},
    {"m5_safety_map_revision", test_m5_safety_map_revision_mismatch_rejected},
    {"m5_safety_geofence_self_intersection", test_m5_safety_self_intersecting_geofence_rejected},
    {"m5_safety_cell_budget", test_m5_safety_cell_check_budget_exhaustion},
    {"m5_safety_epoch_origin_frame_mismatch", test_m5_safety_navigation_map_epoch_origin_frame_mismatch_rejected},
    {"m5_safety_dfs_max_depth_stack_guard", test_m5_safety_dfs_max_depth_stack_guard},
    {"m5_safety_dfs_work_budget", test_m5_safety_dfs_work_unit_budget_exhaustion},
    {"m5_safety_reference_time_epoch", test_m5_safety_reference_time_epoch_mismatch},
    {"m5_safety_map_time_epoch", test_m5_safety_map_time_epoch_mismatch},
    {"m5_safety_directive_frame", test_m5_safety_directive_frame_mismatch},
    {"m5_safety_dynamic_prediction_boundaries", test_m5_safety_dynamic_prediction_time_boundaries},
    {"m5_safety_geofence_height_margins", test_m5_safety_geofence_height_margins},
    {"m5_safety_preview_time_overflow", test_m5_safety_preview_time_overflow_rejected},
    {"m5_safety_numeric_finite_extreme", test_m5_safety_finite_numeric_extreme_rejected},
    {"m5_safety_property_fuzz", test_m5_safety_deterministic_property_fuzz},
    {"m5_safety_geometry_clearance_reference", test_m5_safety_geometry_clearance_reference},
    {"m5_safety_random_polygon_dynamic_track_fuzz", test_m5_safety_random_polygon_dynamic_track_fuzz},
    {"m5_safety_geofence_adversarial_vertices", test_m5_safety_geofence_adversarial_vertices_rejected},
    {"m5_safety_bernstein_bounds_randomized", test_m5_safety_bernstein_bounds_randomized},
    {"m5_safety_bernstein_zero_curve", test_m5_safety_bernstein_zero_curve_enclosure},
    {"m5_safety_bernstein_bounds_invalid", test_m5_safety_bernstein_bounds_invalid_inputs},
    {"m5_safety_multi_track_all_slots", test_m5_safety_multi_track_all_slots_and_fail_closed},
    {"m5_safety_interval_arithmetic_oracle", test_m5_safety_interval_arithmetic_oracle},
    {"m5_safety_outward_rounding_norm_lower", test_m5_safety_outward_rounding_and_norm_lower},
    {"m5_safety_norm_ulp_boundaries", test_m5_safety_norm_ulp_boundaries},
    {"m5_safety_point_aabb_distance_oracle", test_m5_safety_point_aabb_distance_lower_oracle},
    {"m5_safety_dynamic_relative_sweep_oracle", test_m5_safety_dynamic_relative_sweep_oracle},
    {"m5_safety_dynamic_prediction_large_epoch_invariance", test_m5_safety_dynamic_prediction_large_epoch_invariance},
    {"m5_safety_dynamic_adaptive_preview_boundaries", test_m5_safety_dynamic_collision_at_adaptive_and_preview_boundaries},
    {"m5_safety_dynamic_continuous_distance_differential", test_m5_safety_dynamic_continuous_distance_differential},
    {"m5_safety_dynamic_constant_acceleration_continuous_oracle", test_m5_safety_dynamic_constant_acceleration_continuous_oracle},
    {"m5_safety_dynamic_tangency_preview_seam_ulp", test_m5_safety_dynamic_tangency_and_preview_seam_ulp},
    {"m5_safety_dynamic_adaptive_split_ulp", test_m5_safety_dynamic_adaptive_split_ulp},
    {"m5_safety_dynamic_acceleration_envelope_differential", test_m5_safety_dynamic_acceleration_envelope_differential},
    {"m5_safety_consumer_curve_envelope", test_m5_safety_consumer_curve_envelope},
    {"m5_safety_consumer_static_crossing", test_m5_safety_consumer_static_crossing_rejected},
    {"m5_safety_consumer_static_clear", test_m5_safety_consumer_static_clear_companion_passes},
    {"m5_safety_consumer_preview_seam", test_m5_safety_consumer_preview_seam_rejected},
    {"m5_safety_consumer_adaptive_subdivision", test_m5_safety_consumer_adaptive_subdivision},
    {"m5_safety_consumer_dynamic_track", test_m5_safety_consumer_dynamic_track_rejected},
    {"m5_safety_consumer_geofence_boundary", test_m5_safety_consumer_geofence_boundary},
    {"m5_safety_consumer_max_deviation", test_m5_safety_consumer_max_deviation_rejected},
    {"m5_safety_floating_environment", test_m5_safety_floating_environment_rejected},
    {"m5_safety_envelope_static_participation", test_m5_safety_envelope_static_participation},
    {"m5_safety_envelope_dynamic_participation", test_m5_safety_envelope_dynamic_participation},
    {"m5_safety_envelope_geofence_participation", test_m5_safety_envelope_geofence_participation},
    {"m5_safety_jerk_envelope", test_m5_safety_jerk_envelope},
    {"m5_safety_jerk_interval_bound", test_m5_safety_jerk_interval_bound},
    {"m5_safety_jerk_bound_covers_analytic", test_m5_safety_jerk_bound_covers_analytic},
    {"m5_safety_jerk_zero_limit", test_m5_safety_jerk_zero_limit_invalid},
    {"m5_safety_product_envelope_source", test_m5_safety_product_envelope_source},
    {"m5_safety_consumer_time_error_c", test_m5_safety_consumer_time_error_c_path},
  };
  size_t i;
  for (i = 0u; i < sizeof(tests) / sizeof(tests[0]); ++i) {
    int before = g_failures;
    printf("RUN  %s\n", tests[i].name);
    tests[i].fn();
    printf("%-4s %s\n", (g_failures == before) ? "PASS" : "FAIL", tests[i].name);
  }
  printf("\nchecks=%d failures=%d\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}











