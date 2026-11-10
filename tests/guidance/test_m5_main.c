/*
 * Host tests for M5 Guidance (QUAD_X4, NONE obstacle mode).
 *
 * Scope: reference construction, anchor+24 preview, dynamics/geofence checks,
 * READY/ACTIVE lifecycle and epoch behaviour. A-star/local-map/collision are not
 * implemented and are reported NOT_RUN.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ports/port_api.h"
#include "guidance_update/guidance.h"
#include "guidance_update/guidance_internal.h"

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

static void test_mission_leg_beyond_short_duration(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_MISSION;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx_mission(&fx, 1u, 1u, 0u);
  fx.mission.mission.waypoints[0].position_ned_m[0] = 18.0f;
  fx.mission.mission.waypoints[0].speed_limit_mps = 2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fx.out.reference.points[23].point.position_ned_m[0] > 0.0f);
  CHECK(fx.out.reference.points[23].point.velocity_ned_mps[0] <= 2.0f);
  for (unsigned step = 0; step < 1800u; ++step) {
    fx.now += 10000u;
    if (fx_step(&fx) != STATUS_OK || !fx.out.feasibility.feasible) break;
  }
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fx.out.reference.anchor.point.position_ned_m[0] >= 17.99f);
  CHECK(guidance_stop() == STATUS_OK);
}

static void test_return_from_long_mission_leg(void)
{
  Fx fx;
  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_RETURN;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_TRAJECTORY;
  fx.nav.core.position_ned_m[0] = 18.0f;
  fx.nav.core.position_ned_m[2] = -2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  CHECK(fx.out.feasibility.feasible == 1u);
  for (unsigned step = 0; step < 1400u; ++step) {
    fx.now += 10000u;
    if (fx_step(&fx) != STATUS_OK || !fx.out.feasibility.feasible) break;
  }
  CHECK(fx.out.feasibility.feasible == 1u);
  CHECK(fabsf(fx.out.reference.anchor.point.position_ned_m[0]) < 0.01f);
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

static void test_landing_reference_probes_below_home_plane(void)
{
  Fx fx;
  float lowest = 0.0f;

  CHECK(fx_init(&fx));
  fx.dir.operation_code = PRODUCT_OP_LAND;
  fx.dir.guidance_mode_code = PRODUCT_GUIDANCE_MODE_ALT_HOLD;
  fx.nav.core.position_ned_m[2] = -2.0f;
  CHECK(fx_step(&fx) == STATUS_OK);
  for (unsigned i = 0; i < 1500; ++i) {
    CHECK(fx_step(&fx) == STATUS_OK);
    const float above = fx.out.pilot_control_reference.vehicle_extension.altitude_above_origin_m;
    if (above < lowest) lowest = above;
  }
  /* FMAT-0008: a landing that stops exactly at Home hovers above the landing
   * surface whenever the vertical estimate drifted, so the detector can never
   * confirm LANDED. The reference must therefore descend through the Home plane
   * (home=0 here) and let the ground contact model stop the airframe. */
  CHECK(lowest < -0.5f);
  CHECK(guidance_stop() == STATUS_OK);
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
    {"mission_leg_beyond_short_duration", test_mission_leg_beyond_short_duration},
    {"return_from_long_mission_leg", test_return_from_long_mission_leg},
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
    {"landing_reference_probes_below_home", test_landing_reference_probes_below_home_plane},
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
