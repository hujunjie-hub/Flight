/*
 * M5-02 bounded local occupancy map.
 *
 * Host/SIL only. The H723/NONE profile neither compiles nor allocates this map.
 */

#if defined(FLIGHT_M5_HOST_PLANNER)
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"
#include "trajectory_planning/m5_local_map.h"

static uint8_t finite3(const float v[3])
{
  return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]));
}

static uint8_t nav_context_valid(const NavigationSnapshot *nav, uint64_t now_us)
{
  const ContractHeader *h;
  if (nav == NULL) return 0u;
  h = &nav->header;
  return (uint8_t)(h->message_type == MSG_NAVIGATION_SNAPSHOT &&
      h->profile_id == PRODUCT_PROFILE_ID && h->schema_major == CONTRACT_SCHEMA_MAJOR &&
      h->schema_minor == CONTRACT_SCHEMA_MINOR && h->frame_id == FRAME_LOCAL_NED &&
      h->boot_id != 0u && h->time_epoch != 0u &&
      h->clock_domain_id == CLOCK_DOMAIN_FCU_MONOTONIC &&
      (h->validity_flags & (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID)) ==
          (VALIDITY_PAYLOAD_VALID | VALIDITY_TIME_VALID) &&
      h->timestamp_sample_us <= h->timestamp_publish_us &&
      h->timestamp_publish_us <= now_us && h->valid_until_us > now_us &&
      now_us - h->timestamp_sample_us <= 5000u &&
      nav->origin_id != 0u && nav->state_epoch != 0u &&
      finite3(nav->core.position_ned_m));
}

uint8_t m5_local_map_world_to_voxel(const M5LocalMap *map, const float p[3],
                                     uint16_t *voxel)
{
  float relative[3];
  int ix, iy, iz;
  unsigned axis;
  if (map == NULL || map->initialized == 0u || p == NULL || voxel == NULL ||
      !finite3(p) || !finite3(map->center_ned_m)) return 0u;
  for (axis = 0u; axis < 3u; ++axis)
    relative[axis] = p[axis] - map->center_ned_m[axis];
  if (relative[0] < -M5_MAP_HALF_EXTENT_M || relative[0] >= M5_MAP_HALF_EXTENT_M ||
      relative[1] < -M5_MAP_HALF_EXTENT_M || relative[1] >= M5_MAP_HALF_EXTENT_M ||
      relative[2] < -M5_MAP_HALF_EXTENT_M || relative[2] >= M5_MAP_HALF_EXTENT_M) return 0u;
  ix = (int)floorf((relative[0] + M5_MAP_HALF_EXTENT_M) / M5_MAP_CELL_SIZE_M);
  iy = (int)floorf((relative[1] + M5_MAP_HALF_EXTENT_M) / M5_MAP_CELL_SIZE_M);
  iz = (int)floorf((relative[2] + M5_MAP_HALF_EXTENT_M) / M5_MAP_CELL_SIZE_M);
  if (ix < 0 || iy < 0 || iz < 0 || ix >= 16 || iy >= 16 || iz >= 16) return 0u;
  *voxel = (uint16_t)((iz * 16 + iy) * 16 + ix);
  return 1u;
}

void m5_local_map_voxel_center(const M5LocalMap *map, uint16_t voxel, float p[3])
{
  uint16_t ix, iy, iz;
  if (map == NULL || p == NULL || voxel >= M5_MAP_CELL_COUNT) return;
  ix = (uint16_t)(voxel % 16u);
  iy = (uint16_t)((voxel / 16u) % 16u);
  iz = (uint16_t)(voxel / 256u);
  p[0] = map->center_ned_m[0] - 3.75f + 0.5f * (float)ix;
  p[1] = map->center_ned_m[1] - 3.75f + 0.5f * (float)iy;
  p[2] = map->center_ned_m[2] - 3.75f + 0.5f * (float)iz;
}

static void roll_map_to_position(M5LocalMap *map, const float position_ned_m[3])
{
  M5MapCell shifted[M5_MAP_CELL_COUNT];
  int shift[3];
  unsigned axis, x, y, z;
  uint8_t changed = 0u, clear_all = 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    float delta = position_ned_m[axis] - map->center_ned_m[axis];
    if (!isfinite(delta)) return;
    if (fabsf(delta) >= (2.0f * M5_MAP_HALF_EXTENT_M)) {
      clear_all = 1u;
      shift[axis] = 0;
    } else {
      shift[axis] = (int)roundf(delta / M5_MAP_CELL_SIZE_M);
      if (shift[axis] != 0) changed = 1u;
    }
  }
  if (!changed && !clear_all) return;
  if (clear_all || abs(shift[0]) >= 16 || abs(shift[1]) >= 16 || abs(shift[2]) >= 16) {
    memset(map->cells, 0, sizeof(map->cells));
    memcpy(map->center_ned_m, position_ned_m, sizeof(map->center_ned_m));
  } else {
    memset(shifted, 0, sizeof(shifted));
    for (z = 0u; z < 16u; ++z)
      for (y = 0u; y < 16u; ++y)
        for (x = 0u; x < 16u; ++x) {
          int nx = (int)x - shift[0];
          int ny = (int)y - shift[1];
          int nz = (int)z - shift[2];
          if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16 && nz >= 0 && nz < 16) {
            uint16_t old_index = (uint16_t)((z * 16u + y) * 16u + x);
            uint16_t new_index = (uint16_t)(((unsigned)nz * 16u + (unsigned)ny) * 16u +
                                             (unsigned)nx);
            shifted[new_index] = map->cells[old_index];
          }
        }
    memcpy(map->cells, shifted, sizeof(map->cells));
    for (axis = 0u; axis < 3u; ++axis)
      map->center_ned_m[axis] += (float)shift[axis] * M5_MAP_CELL_SIZE_M;
  }
  map->map_revision++;
  if (map->map_revision == 0u) map->map_revision = 1u;
}

static uint8_t context_matches(const M5LocalMap *map, const NavigationSnapshot *nav)
{
  return (uint8_t)(map != NULL && nav != NULL && map->initialized != 0u &&
      map->boot_id == nav->header.boot_id && map->time_epoch == nav->header.time_epoch &&
      map->origin_id == nav->origin_id && map->state_epoch == nav->state_epoch);
}

Status m5_local_map_init(M5LocalMap *map, const NavigationSnapshot *nav,
                         uint64_t now_us, float vehicle_radius_m,
                         float clearance_margin_m, float uncertainty_inflation_factor,
                         uint32_t max_dynamic_prediction_us)
{
  if (map == NULL || !nav_context_valid(nav, now_us) ||
      !isfinite(vehicle_radius_m) || vehicle_radius_m <= 0.0f ||
      !isfinite(clearance_margin_m) || clearance_margin_m < 0.0f ||
      !isfinite(uncertainty_inflation_factor) || uncertainty_inflation_factor < 0.0f ||
      max_dynamic_prediction_us < M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) {
    return STATUS_INVALID_ARGUMENT;
  }
  memset(map, 0, sizeof(*map));
  map->initialized = 1u;
  map->valid = 1u;
  map->origin_id = nav->origin_id;
  map->state_epoch = nav->state_epoch;
  map->time_epoch = nav->header.time_epoch;
  map->boot_id = nav->header.boot_id;
  map->map_revision = 1u;
  map->max_dynamic_prediction_us = max_dynamic_prediction_us;
  map->vehicle_radius_m = vehicle_radius_m;
  map->clearance_margin_m = clearance_margin_m;
  map->uncertainty_inflation_factor = uncertainty_inflation_factor;
  memcpy(map->center_ned_m, nav->core.position_ned_m, sizeof(map->center_ned_m));
  return STATUS_OK;
}

static uint8_t observation_valid(const M5LocalMap *map,
                                 const ProductObstacleObservation *o,
                                 const NavigationSnapshot *nav, uint64_t now_us)
{
  unsigned axis;
  if (o == NULL || !guidance_header_ok(&o->header, &nav->header,
                                       MSG_PRODUCT_OBSTACLE_OBSERVATION, now_us) ||
      o->header.timestamp_sample_us > now_us ||
      now_us - o->header.timestamp_sample_us > M5_MAP_STALE_US ||
      o->observation_id == 0u || o->sensor_instance_id == 0u ||
      o->source_health_epoch == 0u || o->origin_id != nav->origin_id ||
      o->state_epoch != nav->state_epoch ||
      o->source_health > PRODUCT_OBS_SOURCE_REJECT ||
      o->source_health == PRODUCT_OBS_SOURCE_REJECT ||
      (o->observation_kind != PRODUCT_OBS_OCCUPIED_PRIMITIVE &&
       o->observation_kind != PRODUCT_OBS_FREE_SPACE_RAY) ||
      !isfinite(o->confidence_0_to_1) || o->confidence_0_to_1 < 0.0f ||
      o->confidence_0_to_1 > 1.0f) return 0u;

  if (o->observation_kind == PRODUCT_OBS_FREE_SPACE_RAY) {
    uint16_t voxel;
    if (o->source_health != PRODUCT_OBS_SOURCE_NORMAL ||
        !m5_local_map_world_to_voxel(map, o->ray_start_ned_m, &voxel) ||
        !m5_local_map_world_to_voxel(map, o->ray_end_ned_m, &voxel)) return 0u;
  } else {
    uint16_t voxel;
    if (!m5_local_map_world_to_voxel(map, o->obstacle_center_ned_m, &voxel) ||
        (o->shape_kind != PRODUCT_OBS_SHAPE_POINT &&
         o->shape_kind != PRODUCT_OBS_SHAPE_SPHERE &&
         o->shape_kind != PRODUCT_OBS_SHAPE_AABB) ||
        !finite3(o->obstacle_half_extent_m) ||
        !finite3(o->obstacle_velocity_ned_mps) ||
        !finite3(o->position_std_m) || !finite3(o->velocity_std_mps)) return 0u;
    for (axis = 0u; axis < 3u; ++axis) {
      if (o->obstacle_half_extent_m[axis] < 0.0f ||
          o->position_std_m[axis] < 0.0f || o->velocity_std_mps[axis] < 0.0f) return 0u;
    }
  }
  return 1u;
}

static int find_track(const M5LocalMap *map, uint32_t id, uint32_t source_id)
{
  unsigned i;
  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i)
    if (map->tracks[i].id == id && map->tracks[i].source_id == source_id) return (int)i;
  return -1;
}

static int choose_track_slot(const M5LocalMap *map, uint32_t id, uint32_t source_id,
                             uint64_t now_us, const uint8_t used[M5_MAP_MAX_TRACKS])
{
  unsigned i;
  int existing = find_track(map, id, source_id);
  if (existing >= 0 && !used[existing]) return existing;
  for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i)
    if (!used[i] && (map->tracks[i].id == 0u || map->tracks[i].expires_us <= now_us))
      return (int)i;
  return -1;
}

static void update_cell(M5LocalMap *map, uint16_t voxel, int delta,
                        uint16_t source_id, uint64_t sample_us, uint64_t valid_until_us)
{
  M5MapCell *cell = &map->cells[voxel];
  int value = (int)cell->log_odds + delta;
  if (value < -32) value = -32;
  if (value > 32) value = 32;
  cell->log_odds = (int8_t)value;
  cell->source_mask = (uint16_t)(cell->source_mask | (uint16_t)(1u << (source_id & 15u)));
  {
    uint64_t freshness_anchor_us = sample_us;
    uint64_t lease_anchor_us = valid_until_us > M5_MAP_STALE_US ?
        valid_until_us - M5_MAP_STALE_US : 0u;
    if (lease_anchor_us < freshness_anchor_us) freshness_anchor_us = lease_anchor_us;
    cell->last_update_ms = (uint32_t)(freshness_anchor_us / 1000u);
  }
  if (value <= -4) cell->state = M5_MAP_CELL_FREE_CONFIRMED;
  else if (value >= 4) cell->state = M5_MAP_CELL_OCCUPIED_CONFIRMED;
  else cell->state = M5_MAP_CELL_UNKNOWN;
}

static uint8_t update_free_ray(M5LocalMap *map, const ProductObstacleObservation *o)
{
  float delta[3], length;
  unsigned axis, count, i;
  uint16_t previous_voxel = UINT16_MAX;
  for (axis = 0u; axis < 3u; ++axis)
    delta[axis] = o->ray_end_ned_m[axis] - o->ray_start_ned_m[axis];
  length = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
  if (!isfinite(length) || length <= 1e-6f) return 0u;
  count = (unsigned)ceilf(length / (M5_MAP_CELL_SIZE_M * 0.5f));
  if (count < 1u) count = 1u;
  if (count > 64u) return 0u;
  for (i = 0u; i <= count; ++i) {
    float p[3];
    uint16_t voxel;
    float u = (float)i / (float)count;
    for (axis = 0u; axis < 3u; ++axis)
      p[axis] = o->ray_start_ned_m[axis] + u * delta[axis];
    if (!m5_local_map_world_to_voxel(map, p, &voxel)) return 0u;
    /* Multiple ray samples in one cell are one observation, not independent hits. */
    if (voxel != previous_voxel) {
      update_cell(map, voxel, -2, (uint16_t)o->sensor_instance_id,
                  o->header.timestamp_sample_us, o->header.valid_until_us);
      previous_voxel = voxel;
    }
  }
  return 1u;
}

static uint8_t update_occupied(M5LocalMap *map, const ProductObstacleObservation *o)
{
  float half[3], low[3], high[3];
  int lo[3], hi[3];
  unsigned axis, ix, iy, iz;
  uint16_t center_voxel;
  if (!m5_local_map_world_to_voxel(map, o->obstacle_center_ned_m, &center_voxel)) return 0u;
  if (o->shape_kind == PRODUCT_OBS_SHAPE_POINT) {
    half[0] = half[1] = half[2] = 0.0f;
  } else if (o->shape_kind == PRODUCT_OBS_SHAPE_SPHERE) {
    float radius = fmaxf(o->obstacle_half_extent_m[0],
                   fmaxf(o->obstacle_half_extent_m[1], o->obstacle_half_extent_m[2]));
    half[0] = half[1] = half[2] = radius;
  } else {
    memcpy(half, o->obstacle_half_extent_m, sizeof(half));
  }
  for (axis = 0u; axis < 3u; ++axis) {
    float relative_low, relative_high;
    half[axis] += map->uncertainty_inflation_factor * o->position_std_m[axis];
    low[axis] = o->obstacle_center_ned_m[axis] - half[axis];
    high[axis] = o->obstacle_center_ned_m[axis] + half[axis];
    if (!isfinite(half[axis]) || !isfinite(low[axis]) || !isfinite(high[axis]))
      return 0u;
    relative_low = low[axis] - map->center_ned_m[axis];
    relative_high = high[axis] - map->center_ned_m[axis];
    if (!isfinite(relative_low) || !isfinite(relative_high)) return 0u;
    if (relative_high < -M5_MAP_HALF_EXTENT_M ||
        relative_low >= M5_MAP_HALF_EXTENT_M) return 1u;
    if (relative_low < -M5_MAP_HALF_EXTENT_M) relative_low = -M5_MAP_HALF_EXTENT_M;
    if (relative_high >= M5_MAP_HALF_EXTENT_M) relative_high = M5_MAP_HALF_EXTENT_M - 1e-6f;
    lo[axis] = (int)floorf((relative_low + M5_MAP_HALF_EXTENT_M) / M5_MAP_CELL_SIZE_M);
    hi[axis] = (int)floorf((relative_high + M5_MAP_HALF_EXTENT_M) / M5_MAP_CELL_SIZE_M);
    if (lo[axis] < 0) lo[axis] = 0;
    if (hi[axis] > 15) hi[axis] = 15;
  }
  for (iz = (unsigned)lo[2]; iz <= (unsigned)hi[2]; ++iz)
    for (iy = (unsigned)lo[1]; iy <= (unsigned)hi[1]; ++iy)
      for (ix = (unsigned)lo[0]; ix <= (unsigned)hi[0]; ++ix) {
        uint16_t voxel = (uint16_t)((iz * 16u + iy) * 16u + ix);
        update_cell(map, voxel, 4, (uint16_t)o->sensor_instance_id,
                    o->header.timestamp_sample_us, o->header.valid_until_us);
      }
  return 1u;
}

static void update_track(M5LocalMap *map, const ProductObstacleObservation *o, int slot)
{
  M5DynamicTrack *track = &map->tracks[slot];
  unsigned axis;
  float speed2 = 0.0f, radius;
  memset(track, 0, sizeof(*track));
  track->id = o->observation_id;
  track->source_id = o->sensor_instance_id;
  track->flags = (uint32_t)(o->source_health == PRODUCT_OBS_SOURCE_DEGRADED ? 1u : 0u);
  track->sample_us = o->header.timestamp_sample_us;
  track->expires_us = o->header.valid_until_us;
  if (track->expires_us - track->sample_us > map->max_dynamic_prediction_us)
    track->expires_us = track->sample_us + map->max_dynamic_prediction_us;
  memcpy(track->position_ned_m, o->obstacle_center_ned_m, sizeof(track->position_ned_m));
  memcpy(track->velocity_ned_mps, o->obstacle_velocity_ned_mps, sizeof(track->velocity_ned_mps));
  for (axis = 0u; axis < 3u; ++axis) {
    speed2 += o->obstacle_velocity_ned_mps[axis] * o->obstacle_velocity_ned_mps[axis];
  }
  if (o->shape_kind == PRODUCT_OBS_SHAPE_POINT) radius = 0.0f;
  else if (o->shape_kind == PRODUCT_OBS_SHAPE_SPHERE)
    radius = fmaxf(o->obstacle_half_extent_m[0],
        fmaxf(o->obstacle_half_extent_m[1], o->obstacle_half_extent_m[2]));
  else radius = sqrtf(o->obstacle_half_extent_m[0] * o->obstacle_half_extent_m[0] +
        o->obstacle_half_extent_m[1] * o->obstacle_half_extent_m[1] +
        o->obstacle_half_extent_m[2] * o->obstacle_half_extent_m[2]);
  track->radius_m = radius;
  track->position_std_m = sqrtf(o->position_std_m[0] * o->position_std_m[0] +
      o->position_std_m[1] * o->position_std_m[1] +
      o->position_std_m[2] * o->position_std_m[2]);
  track->velocity_std_mps = sqrtf(o->velocity_std_mps[0] * o->velocity_std_mps[0] +
      o->velocity_std_mps[1] * o->velocity_std_mps[1] +
      o->velocity_std_mps[2] * o->velocity_std_mps[2]);
  if (speed2 <= 1e-4f) memset(track, 0, sizeof(*track));
}

Status m5_local_map_ingest(M5LocalMap *map, const NavigationSnapshot *nav,
                           const ProductObstacleObservation *observations,
                           uint16_t count, uint64_t now_us)
{
  uint8_t used[M5_MAP_MAX_TRACKS] = {0};
  int8_t track_slot[M5_MAP_MAX_OBSERVATIONS];
  uint64_t earliest_deadline = UINT64_MAX;
  unsigned i, j;
  if (map == NULL || map->initialized == 0u) return STATUS_NOT_READY;
  if (map->valid == 0u) return STATUS_NOT_PERMITTED;
  if (!nav_context_valid(nav, now_us)) {
    map->valid = 0u;
    return STATUS_STALE_DATA;
  }
  if (!context_matches(map, nav)) {
    float radius = map->vehicle_radius_m;
    float margin = map->clearance_margin_m;
    float inflation = map->uncertainty_inflation_factor;
    uint32_t prediction = map->max_dynamic_prediction_us;
    (void)m5_local_map_init(map, nav, now_us, radius, margin, inflation, prediction);
    return STATUS_STALE_DATA;
  }
  if (count > M5_MAP_MAX_OBSERVATIONS || (count != 0u && observations == NULL)) {
    map->valid = 0u;
    return STATUS_BUFFER_FULL;
  }
  roll_map_to_position(map, nav->core.position_ned_m);
  if (count == 0u) return STATUS_OK;
  /* A map cannot outlive any source that contributed to its current cells.
     Once that lease expires, discard the old map before accepting a fresh batch. */
  if (map->source_deadline_us != 0u && map->source_deadline_us <= now_us) {
    memset(map->cells, 0, sizeof(map->cells));
    memset(map->tracks, 0, sizeof(map->tracks));
    map->degraded = 0u;
    map->last_ingest_us = 0u;
    map->source_deadline_us = 0u;
    map->map_revision++;
    if (map->map_revision == 0u) map->map_revision = 1u;
  }

  for (i = 0u; i < count; ++i) {
    const ProductObstacleObservation *o = &observations[i];
    float speed2 = 0.0f;
    if (!observation_valid(map, o, nav, now_us)) {
      map->valid = 0u;
      return (o->source_health == PRODUCT_OBS_SOURCE_REJECT) ?
          STATUS_NOT_PERMITTED : STATUS_STALE_DATA;
    }
    for (j = 0u; j < i; ++j)
      if (observations[j].observation_id == o->observation_id &&
          observations[j].sensor_instance_id == o->sensor_instance_id) {
        map->valid = 0u;
        return STATUS_INVALID_ARGUMENT;
      }
    if (o->header.valid_until_us < earliest_deadline) earliest_deadline = o->header.valid_until_us;
    if (o->observation_kind == PRODUCT_OBS_OCCUPIED_PRIMITIVE) {
      for (j = 0u; j < 3u; ++j)
        speed2 += o->obstacle_velocity_ned_mps[j] * o->obstacle_velocity_ned_mps[j];
      if (speed2 > 1e-4f) {
        int slot = choose_track_slot(map, o->observation_id,
            o->sensor_instance_id, now_us, used);
        if (slot < 0) {
          map->valid = 0u;
          return STATUS_BUFFER_FULL;
        }
        track_slot[i] = (int8_t)slot;
        used[slot] = 1u;
      } else {
        track_slot[i] = -1;
      }
    } else {
      track_slot[i] = -1;
    }
  }

  for (i = 0u; i < count; ++i) {
    const ProductObstacleObservation *o = &observations[i];
    if (o->source_health == PRODUCT_OBS_SOURCE_DEGRADED) map->degraded = 1u;
    if (o->observation_kind == PRODUCT_OBS_FREE_SPACE_RAY) {
      if (!update_free_ray(map, o)) {
        map->valid = 0u;
        return STATUS_OUT_OF_RANGE;
      }
    } else {
      int existing = find_track(map, o->observation_id, o->sensor_instance_id);
      if (!update_occupied(map, o)) {
        map->valid = 0u;
        return STATUS_OUT_OF_RANGE;
      }
      if (track_slot[i] >= 0) update_track(map, o, track_slot[i]);
      else if (existing >= 0) memset(&map->tracks[existing], 0, sizeof(map->tracks[existing]));
    }
  }
  map->last_ingest_us = now_us;
  /* Each non-empty batch refreshes the aggregate lease. Cells not touched by
     this batch still age independently and become STALE after 500 ms. */
  map->source_deadline_us = earliest_deadline;
  map->map_revision++;
  if (map->map_revision == 0u) map->map_revision = 1u;
  return STATUS_OK;
}

uint8_t m5_local_map_is_ready(const M5LocalMap *map, const NavigationSnapshot *nav,
                              uint64_t now_us)
{
  if (map == NULL || map->initialized == 0u || map->valid == 0u ||
      !nav_context_valid(nav, now_us) || !context_matches(map, nav) ||
      fabsf(nav->core.position_ned_m[0] - map->center_ned_m[0]) > 0.2501f ||
      fabsf(nav->core.position_ned_m[1] - map->center_ned_m[1]) > 0.2501f ||
      fabsf(nav->core.position_ned_m[2] - map->center_ned_m[2]) > 0.2501f ||
      map->last_ingest_us == 0u || now_us < map->last_ingest_us ||
      now_us - map->last_ingest_us > M5_MAP_STALE_US ||
      map->source_deadline_us <= now_us ||
      map->max_dynamic_prediction_us < M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US) return 0u;
  return 1u;
}

uint8_t m5_local_map_voxel_state(const M5LocalMap *map, uint16_t voxel,
                                 uint64_t now_us, uint8_t *state)
{
  const M5MapCell *cell;
  uint32_t now_ms, age_ms;
  if (state == NULL || map == NULL || voxel >= M5_MAP_CELL_COUNT) return 0u;
  if (map->initialized == 0u || map->valid == 0u) {
    *state = M5_MAP_CELL_INVALID;
    return 1u;
  }
  if ((map->source_deadline_us != 0u && now_us >= map->source_deadline_us) ||
      (map->last_ingest_us != 0u &&
       (now_us < map->last_ingest_us ||
        now_us - map->last_ingest_us > M5_MAP_STALE_US))) {
    *state = M5_MAP_CELL_STALE;
    return 1u;
  }
  cell = &map->cells[voxel];
  if (cell->state > M5_MAP_CELL_INVALID) {
    *state = M5_MAP_CELL_INVALID;
    return 1u;
  }
  now_ms = (uint32_t)(now_us / 1000u);
  age_ms = now_ms - cell->last_update_ms;
  if (cell->state != M5_MAP_CELL_UNKNOWN && cell->state != M5_MAP_CELL_INVALID &&
      (age_ms >= 0x80000000u || age_ms > 500u)) {
    *state = M5_MAP_CELL_STALE;
    return 1u;
  }
  *state = cell->state;
  return 1u;
}

Status m5_local_map_query(const M5LocalMap *map, const float position[3],
                          uint64_t now_us, uint8_t *state)
{
  uint16_t voxel;
  if (map == NULL || position == NULL || state == NULL) return STATUS_INVALID_ARGUMENT;
  if (!m5_local_map_world_to_voxel(map, position, &voxel)) {
    *state = M5_MAP_CELL_INVALID;
    return STATUS_OUT_OF_RANGE;
  }
  if (!m5_local_map_voxel_state(map, voxel, now_us, state)) return STATUS_INVALID_ARGUMENT;
  return (*state == M5_MAP_CELL_INVALID) ? STATUS_NOT_PERMITTED : STATUS_OK;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
