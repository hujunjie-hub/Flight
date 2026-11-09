/*
 * M5-02 fixed local map for Host/SIL advanced planning.
 *
 * This module is deliberately excluded from the H723/NONE production profile.
 * It owns copies of observations; it never retains Port input pointers.
 */
#ifndef M5_LOCAL_MAP_H
#define M5_LOCAL_MAP_H

#if !defined(FLIGHT_M5_HOST_PLANNER)
#error "M5 local map is Host/SIL only; do not enable it in the H723/NONE profile"
#endif

#include "ports/port_api.h"

#define M5_MAP_DIM 16u
#define M5_MAP_CELL_COUNT (M5_MAP_DIM * M5_MAP_DIM * M5_MAP_DIM)
#define M5_MAP_CELL_SIZE_M 0.5f
#define M5_MAP_HALF_EXTENT_M 4.0f
#define M5_MAP_MAX_TRACKS 8u
#define M5_MAP_MAX_OBSERVATIONS PRODUCT_MAX_GUIDANCE_OBSERVATIONS_PER_UPDATE
#define M5_MAP_STALE_US 500000u
#define M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US 1000000u

typedef enum {
  M5_MAP_CELL_UNKNOWN = 0u,
  M5_MAP_CELL_FREE_CONFIRMED = 1u,
  M5_MAP_CELL_OCCUPIED_CONFIRMED = 2u,
  M5_MAP_CELL_STALE = 3u,
  M5_MAP_CELL_INVALID = 4u
} M5MapCellState;

typedef struct {
  int8_t log_odds;
  uint8_t state;
  uint16_t source_mask;
  uint32_t last_update_ms;
} M5MapCell;

typedef struct {
  uint32_t id;
  uint16_t source_id;
  uint16_t flags;
  uint64_t sample_us;
  uint64_t expires_us;
  float position_ned_m[3];
  float velocity_ned_mps[3];
  float radius_m;
  float position_std_m;
  float velocity_std_mps;
  uint32_t reserved;
} M5DynamicTrack;

typedef struct {
  uint8_t initialized;
  uint8_t valid;
  uint8_t degraded;
  uint8_t reserved0;
  uint32_t origin_id;
  uint32_t state_epoch;
  uint32_t time_epoch;
  uint32_t map_revision;
  uint64_t boot_id;
  uint64_t last_ingest_us;
  uint64_t source_deadline_us;
  uint32_t max_dynamic_prediction_us;
  float vehicle_radius_m;
  float clearance_margin_m;
  float uncertainty_inflation_factor;
  M5MapCell cells[M5_MAP_CELL_COUNT];
  M5DynamicTrack tracks[M5_MAP_MAX_TRACKS];
} M5LocalMap;

CONTRACT_STATIC_ASSERT(sizeof(M5MapCell) == 8u, "M5 MapCell ABI must remain 8 bytes");
CONTRACT_STATIC_ASSERT(sizeof(M5DynamicTrack) == 64u, "M5 dynamic track ABI must remain 64 bytes");
CONTRACT_STATIC_ASSERT(M5_MAP_CELL_COUNT == 4096u, "M5 Host map must remain 4096 cells");

Status m5_local_map_init(M5LocalMap *map, const NavigationSnapshot *navigation,
                         uint64_t now_us, float vehicle_radius_m,
                         float clearance_margin_m, float uncertainty_inflation_factor,
                         uint32_t max_dynamic_prediction_us);
Status m5_local_map_ingest(M5LocalMap *map, const NavigationSnapshot *navigation,
                           const ProductObstacleObservation *observations,
                           uint16_t observation_count, uint64_t now_us);
Status m5_local_map_query(const M5LocalMap *map, const float position_ned_m[3],
                          uint64_t now_us, uint8_t *state);
uint8_t m5_local_map_is_ready(const M5LocalMap *map, const NavigationSnapshot *navigation,
                              uint64_t now_us);
uint8_t m5_local_map_world_to_voxel(const float position_ned_m[3], uint16_t *voxel);
void m5_local_map_voxel_center(uint16_t voxel, float position_ned_m[3]);
uint8_t m5_local_map_voxel_state(const M5LocalMap *map, uint16_t voxel,
                                 uint64_t now_us, uint8_t *state);

#endif /* M5_LOCAL_MAP_H */
