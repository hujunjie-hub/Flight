/*
 * Host tests for M5 Guidance (QUAD_X4, NONE obstacle mode).
 *
 * Scope: reference construction, anchor+24 preview, dynamics/geofence checks,
 * READY/ACTIVE lifecycle, plus Host/SIL-only M5-02 map and M5-03 A* kernels.
 * The map/A* tests do not imply H723/NONE deployment or M5-06 collision proof.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ports/port_api.h"
#include "guidance_update/guidance.h"
#include "guidance_update/guidance_internal.h"
#include "trajectory_planning/m5_local_map.h"
#include "trajectory_planning/m5_kinodynamic_astar.h"
#include "trajectory_planning/m5_path_smoother.h"
#include "trajectory_planning/m5_path_smoother.h"

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
  fx->env.vehicle_extension.max_jerk_ned_mps3[0] = 10.0f;
  fx->env.vehicle_extension.max_jerk_ned_mps3[1] = 10.0f;
  fx->env.vehicle_extension.max_jerk_ned_mps3[2] = 5.0f;
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
  fx->path.primitive_index[0] = 0u;
  fx->path.primitive_index[1] = 1u;
}

static void test_m5_smoother_minimum_jerk_continuity(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5SmoothedTrajectory output;
  const uint64_t now = 9500000u;
  smoother_fx_path_init(&fx, now);
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 2u);
  CHECK(output.fallback_used == 0u);
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
  M5SmoothedTrajectory output;
  const uint64_t now = 9600000u;
  smoother_fx_path_init(&fx, now);
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  fx.map.map_revision++;
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_INPUT_STALE);
  CHECK(output.segment_count == 0u);

  smoother_fx_path_init(&fx, now);
  fx.path.offset_us[1] = 450000u;
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
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
  M5SmoothedTrajectory output;
  const uint64_t now = 10000000u;
  planner_fx_init(&fx, now);
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 1u);
  CHECK(output.fallback_used == 0u);
  CHECK(output.map_revision == fx.map.map_revision);
  CHECK(output.total_duration_us == 400000u);
  CHECK(output.segment_duration_us[0] == 400000u);
  CHECK(isfinite(output.initial_cost) && isfinite(output.optimized_cost));
  CHECK(output.optimized_cost <= output.initial_cost + 1e-6);
  CHECK(fabsf(guidance_quintic_eval(&output.position[0][0], 0.0f, 0u) -
              fx.path.position_ned_m[0][0]) < 1e-5f);
  CHECK(fabsf(guidance_quintic_eval(&output.position[0][0], 0.4f, 0u) -
              fx.path.position_ned_m[1][0]) < 1e-4f);
  for (unsigned i = 0u; i < 6u; ++i) CHECK(isfinite(output.yaw[0].c[i]));
}

static void test_m5_smoother_optimizes_two_segment_path_continuously(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
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
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_OK);
  CHECK(output.segment_count == 2u && output.fallback_used == 0u);
  CHECK(output.total_duration_us == 800000u);
  CHECK(output.optimized_cost < output.initial_cost);
  CHECK(fabs(guidance_quintic_eval(&output.position[0][0], 0.4f, 0u) + 1.25) < 1e-5);
  CHECK(fabs(guidance_quintic_eval(&output.position[1][0], 0.0f, 0u) + 1.25) < 1e-5);
  left_velocity = guidance_quintic_eval(&output.position[0][0], 0.4f, 1u);
  right_velocity = guidance_quintic_eval(&output.position[1][0], 0.0f, 1u);
  left_acceleration = guidance_quintic_eval(&output.position[0][0], 0.4f, 2u);
  right_acceleration = guidance_quintic_eval(&output.position[1][0], 0.0f, 2u);
  CHECK(fabsf(left_velocity - right_velocity) < 1e-4f);
  CHECK(fabsf(left_acceleration - right_acceleration) < 1e-3f);
}

static void test_m5_smoother_rejects_stale_and_mismatched_paths(void)
{
  PlannerFx fx;
  GuidanceMinimumJerkWorkspace workspace;
  M5SmoothedTrajectory output;
  const uint64_t now = 12000000u;
  planner_fx_init(&fx, now);
  CHECK(m5_kinodynamic_astar_plan(&fx.request, &fx.workspace,
      &fx.path, &fx.stats) == M5_PLANNER_OK);
  fx.map.map_revision++;
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_INPUT_STALE);
  CHECK(output.segment_count == 0u);
  fx.map.map_revision = fx.path.map_revision;
  fx.path.offset_us[1] += 200000u;
  fx.path.total_cost_s += 0.2f;
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_INVALID_INPUT);
  CHECK(output.segment_count == 0u);
  fx.path.segment_count = (uint8_t)(M5_SMOOTHER_MAX_SEGMENTS + 1u);
  CHECK(m5_path_smoother_build(&fx.path, &fx.map, &fx.navigation, now,
      &workspace, &output) == M5_SMOOTHER_CAPACITY_EXCEEDED);
  CHECK(output.segment_count == 0u);
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
