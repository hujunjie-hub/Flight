/*
 * M5-03 fixed-capacity Kinodynamic A* interface.
 *
 * Host/SIL only. H723/NONE must not compile, allocate or call this planner.
 */
#ifndef M5_KINODYNAMIC_ASTAR_H
#define M5_KINODYNAMIC_ASTAR_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "Kinodynamic A* is Host/SIL only; H723/NONE does not admit the planner arena"
#endif

#include "guidance_update/guidance_internal.h"
#include "trajectory_planning/m5_local_map.h"

#define M5_ASTAR_NODE_CAPACITY 512u
#define M5_ASTAR_OPEN_CAPACITY 512u
#define M5_ASTAR_HASH_CAPACITY 1024u
#define M5_ASTAR_MAX_EXPANSIONS 512u
#define M5_ASTAR_MAX_EDGE_CHECKS_PER_NODE 64u
#define M5_ASTAR_MAX_PATH_SEGMENTS 8u
#define M5_ASTAR_MAX_TIME_BINS 31u
#define M5_ASTAR_VELOCITY_BIN_COUNT 29u
#define M5_ASTAR_YAW_BIN_COUNT 16u
#define M5_ASTAR_EMPTY_NODE 0xFFFFu
#define M5_ASTAR_NO_HEAP_INDEX 0xFFFFu

typedef enum {
  M5_PLANNER_OK = 0u,
  M5_PLANNER_NO_PATH = 1u,
  M5_PLANNER_CAPACITY_EXCEEDED = 2u,
  M5_PLANNER_DEADLINE_MISSED = 3u,
  M5_PLANNER_INPUT_STALE = 4u,
  M5_PLANNER_OUT_OF_RANGE = 5u,
  M5_PLANNER_INVALID_INPUT = 6u,
  M5_PLANNER_TRAJECTORY_REJECTED = 7u
} M5PlannerResult;

typedef enum {
  M5_PRIMITIVE_TRANSLATE = 1u,
  M5_PRIMITIVE_VERTICAL = 2u,
  M5_PRIMITIVE_YAW = 3u,
  M5_PRIMITIVE_WAIT = 4u,
  M5_PRIMITIVE_BRAKE = 5u
} M5PrimitiveKind;

typedef struct {
  uint8_t kind;
  uint8_t dir;
  uint8_t speed_id;
  uint8_t duration_id;
  float target_speed;
  float yaw_rate_hint;
  uint32_t duration_us;
} M5KinodynamicPrimitive;

CONTRACT_STATIC_ASSERT(sizeof(M5KinodynamicPrimitive) == 16u,
                       "Kinodynamic primitive ABI must remain 16 bytes");

typedef struct {
  uint32_t key;
  float g;
  float f;
  uint16_t parent;
  uint16_t heap_index;
} M5AstarNode;

CONTRACT_STATIC_ASSERT(sizeof(M5AstarNode) == 16u, "Kinodynamic A* node ABI must remain 16 bytes");

typedef struct {
  M5AstarNode nodes[M5_ASTAR_NODE_CAPACITY];
  uint16_t open_heap[M5_ASTAR_OPEN_CAPACITY];
  uint16_t hash_slots[M5_ASTAR_HASH_CAPACITY];
  uint8_t closed[M5_ASTAR_NODE_CAPACITY / 8u];
  uint8_t parent_primitive[M5_ASTAR_NODE_CAPACITY];
  uint16_t node_count;
  uint16_t open_count;
} M5AstarWorkspace;

CONTRACT_STATIC_ASSERT(sizeof(M5AstarWorkspace) <= 12288u,
                       "Kinodynamic A* Host workspace exceeds fixed 12 KiB budget");

typedef uint64_t (*M5PlannerNowFn)(void *context);

typedef struct {
  const M5LocalMap *map;
  const NavigationSnapshot *navigation;
  const ProductGuidanceConfig *guidance_config;
  const SafetyEnvelope *safety_envelope;
  float start_position_ned_m[3];
  float start_velocity_ned_mps[3];
  float start_acceleration_ned_mps2[3];
  float start_yaw_rad;
  float start_yaw_rate_radps;
  float goal_position_ned_m[3];
  float goal_yaw_rad;
  float acceptance_radius_m;
  float terminal_speed_max_mps;
  float yaw_tolerance_rad;
  uint64_t evaluation_time_us;
  uint64_t planning_deadline_monotonic_us;
  M5PlannerNowFn monotonic_now;
  void *clock_context;
} M5AstarRequest;

typedef struct {
  uint8_t segment_count;
  uint8_t primitive_index[M5_ASTAR_MAX_PATH_SEGMENTS];
  uint16_t reserved0;
  uint32_t map_revision;
  float total_cost_s;
  float position_ned_m[M5_ASTAR_MAX_PATH_SEGMENTS + 1u][3];
  float velocity_ned_mps[M5_ASTAR_MAX_PATH_SEGMENTS + 1u][3];
  float acceleration_ned_mps2[M5_ASTAR_MAX_PATH_SEGMENTS + 1u][3];
  float yaw_rad[M5_ASTAR_MAX_PATH_SEGMENTS + 1u];
  float yaw_rate_radps[M5_ASTAR_MAX_PATH_SEGMENTS + 1u];
  uint32_t offset_us[M5_ASTAR_MAX_PATH_SEGMENTS + 1u];
} M5AstarPath;

typedef struct {
  uint16_t expanded_nodes;
  uint16_t generated_edges;
  uint16_t rejected_edges;
  uint16_t node_count;
  uint16_t peak_open_count;
  uint16_t goal_node_id;
  uint32_t map_revision;
  M5PlannerResult result;
} M5AstarStats;

const M5KinodynamicPrimitive *m5_kinodynamic_primitive_table(uint8_t *count);
uint8_t m5_kinodynamic_primitive_table_validate(void);
uint32_t m5_kinodynamic_encode_key(uint16_t voxel, uint8_t velocity_bin,
                                   uint8_t yaw_bin, uint8_t time_bin);
uint8_t m5_kinodynamic_decode_key(uint32_t key, uint16_t *voxel,
                                  uint8_t *velocity_bin, uint8_t *yaw_bin,
                                  uint8_t *time_bin);
uint8_t m5_kinodynamic_velocity_bin(uint8_t velocity_bin, float velocity_ned_mps[3]);
M5PlannerResult m5_kinodynamic_astar_plan(const M5AstarRequest *request,
                                          M5AstarWorkspace *workspace,
                                          M5AstarPath *path,
                                          M5AstarStats *stats);

#endif /* M5_KINODYNAMIC_ASTAR_H */
