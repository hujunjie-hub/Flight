/*
 * M5-03 bounded Kinodynamic A* for Host/SIL.
 *
 * Search state, memory, primitive ordering and tie-breaking are fixed by
 * spec/04 §7. H723/NONE does not compile or allocate this planner.
 */

#if defined(FLIGHT_M5_HOST_PLANNER)
#include <math.h>
#include <string.h>
#include "guidance_update/guidance_internal.h"
#include "trajectory_planning/m5_kinodynamic_astar.h"

#define M5_ASTAR_PI 3.14159265358979323846f
#define M5_ASTAR_TWO_PI 6.28318530717958647692f
#define M5_ASTAR_CELL_CHECK_LIMIT 512u
#define M5_ASTAR_EDGE_SUBDIVISIONS 4u
#define M5_ASTAR_MIN_G_IMPROVEMENT_S 1e-6f

#include "trajectory_planning/kinodynamic_primitives_generated.h"

static uint8_t finite3(const float v[3])
{
  return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]));
}

const M5KinodynamicPrimitive *m5_kinodynamic_primitive_table(uint8_t *count)
{
  if (count != NULL) *count = (uint8_t)M5_KINODYNAMIC_PRIMITIVE_COUNT;
  return m5_kinodynamic_primitives;
}

uint32_t m5_kinodynamic_encode_key(uint16_t voxel, uint8_t velocity_bin,
                                   uint8_t yaw_bin, uint8_t time_bin)
{
  if (voxel >= M5_MAP_CELL_COUNT || velocity_bin >= M5_ASTAR_VELOCITY_BIN_COUNT ||
      yaw_bin >= M5_ASTAR_YAW_BIN_COUNT || time_bin > M5_ASTAR_MAX_TIME_BINS)
    return UINT32_MAX;
  return ((uint32_t)voxel << 14) | ((uint32_t)velocity_bin << 9) |
         ((uint32_t)yaw_bin << 5) | (uint32_t)time_bin;
}

uint8_t m5_kinodynamic_decode_key(uint32_t key, uint16_t *voxel,
                                  uint8_t *velocity_bin, uint8_t *yaw_bin,
                                  uint8_t *time_bin)
{
  if ((key >> 26) != 0u) return 0u;
  if (voxel != NULL) *voxel = (uint16_t)((key >> 14) & 0x0FFFu);
  if (velocity_bin != NULL) *velocity_bin = (uint8_t)((key >> 9) & 0x1Fu);
  if (yaw_bin != NULL) *yaw_bin = (uint8_t)((key >> 5) & 0x0Fu);
  if (time_bin != NULL) *time_bin = (uint8_t)(key & 0x1Fu);
  return (uint8_t)(((key >> 9) & 0x1Fu) < M5_ASTAR_VELOCITY_BIN_COUNT &&
                   (key & 0x1Fu) <= M5_ASTAR_MAX_TIME_BINS);
}

uint8_t m5_kinodynamic_velocity_bin(uint8_t bin, float v[3])
{
  static const float speeds[3] = {0.5f, 1.0f, 2.0f};
  if (v == NULL || bin >= M5_ASTAR_VELOCITY_BIN_COUNT) return 0u;
  v[0] = v[1] = v[2] = 0.0f;
  if (bin == 0u) return 1u;
  if (bin <= 24u) {
    uint8_t index = (uint8_t)(bin - 1u);
    uint8_t direction = (uint8_t)(index / 3u);
    uint8_t speed_id = (uint8_t)(index % 3u);
    float angle = (float)direction * (M5_ASTAR_PI / 4.0f);
    v[0] = speeds[speed_id] * cosf(angle);
    v[1] = speeds[speed_id] * sinf(angle);
    return 1u;
  }
  if (bin <= 26u) {
    v[2] = -speeds[bin - 25u];
    return 1u;
  }
  v[2] = speeds[bin - 27u];
  return 1u;
}

static uint8_t nearest_voxel(const float p[3], uint16_t *voxel)
{
  int index[3];
  unsigned axis;
  if (p == NULL || voxel == NULL || !finite3(p)) return 0u;
  for (axis = 0u; axis < 3u; ++axis) {
    float coordinate;
    if (p[axis] < -4.0f || p[axis] >= 4.0f) return 0u;
    coordinate = (p[axis] + 3.75f) / 0.5f;
    /* Exact half-cell ties choose the smaller voxel index. */
    index[axis] = (int)ceilf(coordinate - 0.5f - 1e-7f);
    if (index[axis] < 0) index[axis] = 0;
    if (index[axis] > 15) index[axis] = 15;
  }
  *voxel = (uint16_t)((index[2] * 16 + index[1]) * 16 + index[0]);
  return 1u;
}

static uint8_t nearest_velocity_bin(const float velocity[3], uint8_t *bin,
                                    uint8_t *is_lattice)
{
  float best = INFINITY;
  uint8_t best_bin = 0u, candidate;
  for (candidate = 0u; candidate < M5_ASTAR_VELOCITY_BIN_COUNT; ++candidate) {
    float v[3], dx, dy, dz, distance;
    (void)m5_kinodynamic_velocity_bin(candidate, v);
    dx = velocity[0] - v[0];
    dy = velocity[1] - v[1];
    dz = velocity[2] - v[2];
    distance = dx*dx + dy*dy + dz*dz;
    if (distance < best) {
      best = distance;
      best_bin = candidate;
    }
  }
  *bin = best_bin;
  *is_lattice = (uint8_t)(best <= 1e-6f);
  return 1u;
}

static uint8_t nearest_yaw_bin(float yaw)
{
  float wrapped = fmodf(yaw, M5_ASTAR_TWO_PI);
  int bin;
  if (wrapped < 0.0f) wrapped += M5_ASTAR_TWO_PI;
  bin = (int)floorf(wrapped * (16.0f / M5_ASTAR_TWO_PI) + 0.5f);
  return (uint8_t)(bin & 15);
}

static float yaw_bin_angle(uint8_t bin)
{
  return (float)bin * (M5_ASTAR_TWO_PI / 16.0f);
}

static uint8_t bit_is_set(const uint8_t bits[M5_ASTAR_NODE_CAPACITY / 8u], uint16_t id)
{
  return (uint8_t)((bits[id >> 3] >> (id & 7u)) & 1u);
}

static void bit_set(uint8_t bits[M5_ASTAR_NODE_CAPACITY / 8u], uint16_t id)
{
  bits[id >> 3] = (uint8_t)(bits[id >> 3] | (uint8_t)(1u << (id & 7u)));
}

static void bit_clear(uint8_t bits[M5_ASTAR_NODE_CAPACITY / 8u], uint16_t id)
{
  bits[id >> 3] = (uint8_t)(bits[id >> 3] & (uint8_t)~(1u << (id & 7u)));
}

static uint8_t node_less(const M5AstarWorkspace *w, uint16_t a, uint16_t b)
{
  const M5AstarNode *na = &w->nodes[a], *nb = &w->nodes[b];
  float ha = na->f - na->g, hb = nb->f - nb->g;
  if (na->f != nb->f) return (uint8_t)(na->f < nb->f);
  if (ha != hb) return (uint8_t)(ha < hb);
  if (na->key != nb->key) return (uint8_t)(na->key < nb->key);
  return (uint8_t)(a < b);
}

static void heap_swap(M5AstarWorkspace *w, uint16_t a, uint16_t b)
{
  uint16_t id = w->open_heap[a];
  w->open_heap[a] = w->open_heap[b];
  w->open_heap[b] = id;
  w->nodes[w->open_heap[a]].heap_index = a;
  w->nodes[w->open_heap[b]].heap_index = b;
}

static void heap_sift_up(M5AstarWorkspace *w, uint16_t index)
{
  while (index > 0u) {
    uint16_t parent = (uint16_t)((index - 1u) / 2u);
    if (!node_less(w, w->open_heap[index], w->open_heap[parent])) break;
    heap_swap(w, index, parent);
    index = parent;
  }
}

static void heap_sift_down(M5AstarWorkspace *w, uint16_t index)
{
  for (;;) {
    uint16_t left = (uint16_t)(index * 2u + 1u);
    uint16_t right = (uint16_t)(left + 1u);
    uint16_t best = index;
    if (left < w->open_count && node_less(w, w->open_heap[left], w->open_heap[best]))
      best = left;
    if (right < w->open_count && node_less(w, w->open_heap[right], w->open_heap[best]))
      best = right;
    if (best == index) break;
    heap_swap(w, index, best);
    index = best;
  }
}

static uint8_t heap_push(M5AstarWorkspace *w, uint16_t id)
{
  if (w->open_count >= M5_ASTAR_OPEN_CAPACITY) return 0u;
  w->open_heap[w->open_count] = id;
  w->nodes[id].heap_index = w->open_count;
  w->open_count++;
  heap_sift_up(w, (uint16_t)(w->open_count - 1u));
  return 1u;
}

static uint16_t heap_pop(M5AstarWorkspace *w)
{
  uint16_t result = w->open_heap[0];
  w->open_count--;
  w->nodes[result].heap_index = M5_ASTAR_NO_HEAP_INDEX;
  if (w->open_count != 0u) {
    w->open_heap[0] = w->open_heap[w->open_count];
    w->nodes[w->open_heap[0]].heap_index = 0u;
    heap_sift_down(w, 0u);
  }
  return result;
}

static uint16_t hash_slot(const M5AstarWorkspace *w, uint32_t key, uint8_t *found)
{
  uint32_t slot = (key * 2654435761u) >> 22;
  unsigned probe;
  for (probe = 0u; probe < M5_ASTAR_HASH_CAPACITY; ++probe) {
    uint16_t id = w->hash_slots[slot];
    if (id == M5_ASTAR_EMPTY_NODE) {
      *found = 0u;
      return (uint16_t)slot;
    }
    if (w->nodes[id].key == key) {
      *found = 1u;
      return (uint16_t)slot;
    }
    slot = (slot + 1u) & (M5_ASTAR_HASH_CAPACITY - 1u);
  }
  *found = 0u;
  return M5_ASTAR_EMPTY_NODE;
}

static float heuristic(const M5AstarRequest *r, const float p[3], float yaw)
{
  float dx = p[0] - r->goal_position_ned_m[0];
  float dy = p[1] - r->goal_position_ned_m[1];
  float dz = p[2] - r->goal_position_ned_m[2];
  float distance = sqrtf(dx*dx + dy*dy + dz*dz);
  float distance_time = fmaxf(0.0f, distance - r->acceptance_radius_m) /
                        r->safety_envelope->max_linear_speed_mps;
  float yaw_error = fabsf(remainderf(r->goal_yaw_rad - yaw, M5_ASTAR_TWO_PI));
  float yaw_time = fmaxf(0.0f, yaw_error - r->yaw_tolerance_rad) /
                   r->safety_envelope->max_yaw_rate_radps;
  return fmaxf(distance_time, yaw_time);
}

static uint8_t goal_reached(const M5AstarRequest *r, const float p[3],
                            const float v[3], float yaw)
{
  float dx = p[0] - r->goal_position_ned_m[0];
  float dy = p[1] - r->goal_position_ned_m[1];
  float dz = p[2] - r->goal_position_ned_m[2];
  float speed = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  float yaw_error = fabsf(remainderf(r->goal_yaw_rad - yaw, M5_ASTAR_TWO_PI));
  return (uint8_t)(sqrtf(dx*dx + dy*dy + dz*dz) <= r->acceptance_radius_m &&
                   speed <= r->terminal_speed_max_mps &&
                   yaw_error <= r->yaw_tolerance_rad);
}

static uint8_t node_state(const M5AstarRequest *r, const M5AstarNode *node,
                          uint16_t id, float p[3], float v[3], float a[3],
                          float *yaw, float *yaw_rate, uint8_t *velocity_bin,
                          uint8_t *yaw_bin, uint8_t *time_bin)
{
  uint16_t voxel;
  if (id == 0u) {
    memcpy(p, r->start_position_ned_m, sizeof(float) * 3u);
    memcpy(v, r->start_velocity_ned_mps, sizeof(float) * 3u);
    memcpy(a, r->start_acceleration_ned_mps2, sizeof(float) * 3u);
    *yaw = r->start_yaw_rad;
    *yaw_rate = r->start_yaw_rate_radps;
  }
  if (!m5_kinodynamic_decode_key(node->key, &voxel, velocity_bin, yaw_bin, time_bin))
    return 0u;
  if (id != 0u) {
    m5_local_map_voxel_center(voxel, p);
    (void)m5_kinodynamic_velocity_bin(*velocity_bin, v);
    a[0] = a[1] = a[2] = 0.0f;
    *yaw = yaw_bin_angle(*yaw_bin);
    *yaw_rate = 0.0f;
  }
  return 1u;
}

static uint8_t primitive_end_velocity(const M5KinodynamicPrimitive *primitive,
                                      const float current_velocity[3],
                                      uint8_t current_velocity_bin,
                                      uint8_t root_velocity_is_lattice,
                                      uint8_t *end_velocity_bin, float end_velocity[3])
{
  uint8_t direction;
  if (primitive->kind == M5_PRIMITIVE_TRANSLATE) {
    direction = primitive->dir;
    if (primitive->speed_id == 2u) direction = (uint8_t)(direction * 2u);
    if (direction >= 8u || primitive->speed_id >= 3u) return 0u;
    *end_velocity_bin = (uint8_t)(1u + direction * 3u + primitive->speed_id);
    return m5_kinodynamic_velocity_bin(*end_velocity_bin, end_velocity);
  }
  if (primitive->kind == M5_PRIMITIVE_VERTICAL) {
    if (primitive->dir > 1u || primitive->speed_id > 1u) return 0u;
    *end_velocity_bin = (uint8_t)((primitive->dir == 0u ? 25u : 27u) +
                                  primitive->speed_id);
    return m5_kinodynamic_velocity_bin(*end_velocity_bin, end_velocity);
  }
  if (primitive->kind == M5_PRIMITIVE_YAW) {
    if (!root_velocity_is_lattice) return 0u;
    *end_velocity_bin = current_velocity_bin;
    memcpy(end_velocity, current_velocity, sizeof(float) * 3u);
    return 1u;
  }
  if (primitive->kind == M5_PRIMITIVE_WAIT) {
    if (sqrtf(current_velocity[0]*current_velocity[0] +
              current_velocity[1]*current_velocity[1] +
              current_velocity[2]*current_velocity[2]) > 1e-3f) return 0u;
    *end_velocity_bin = 0u;
    end_velocity[0] = end_velocity[1] = end_velocity[2] = 0.0f;
    return 1u;
  }
  if (primitive->kind == M5_PRIMITIVE_BRAKE) {
    *end_velocity_bin = 0u;
    end_velocity[0] = end_velocity[1] = end_velocity[2] = 0.0f;
    return 1u;
  }
  return 0u;
}

static uint8_t make_edge(const M5AstarRequest *r, const M5AstarNode *node,
                         uint16_t node_id, const M5KinodynamicPrimitive *primitive,
                         GuidanceQuintic q[4], uint32_t *end_key,
                         float end_position[3], float end_velocity[3],
                         float *end_yaw, uint8_t *end_time_bin)
{
  float p0[3], v0[3], a0[3], yaw0, yaw_rate0, duration;
  uint8_t velocity_bin, yaw_bin, time_bin, end_velocity_bin, end_yaw_bin;
  uint8_t root_velocity_is_lattice;
  uint16_t end_voxel;
  uint32_t ticks;
  unsigned axis, subdivision;
  float velocity_lattice[3];
  uint8_t current_velocity_lattice;

  if (!node_state(r, node, node_id, p0, v0, a0, &yaw0, &yaw_rate0,
                  &velocity_bin, &yaw_bin, &time_bin)) return 0u;
  (void)nearest_velocity_bin(v0, &current_velocity_lattice, &root_velocity_is_lattice);
  duration = (float)primitive->duration_us * 1e-6f;
  if (primitive->duration_us == 0u || primitive->duration_us % 200000u != 0u)
    return 0u;
  ticks = primitive->duration_us / 200000u;
  if (time_bin + ticks > M5_ASTAR_MAX_TIME_BINS) return 0u;
  if (!primitive_end_velocity(primitive, v0, velocity_bin,
                              (uint8_t)(node_id != 0u || root_velocity_is_lattice),
                              &end_velocity_bin, end_velocity)) return 0u;
  if (!m5_kinodynamic_velocity_bin(end_velocity_bin, velocity_lattice)) return 0u;
  if (primitive->kind == M5_PRIMITIVE_TRANSLATE ||
      primitive->kind == M5_PRIMITIVE_VERTICAL) {
    float target_norm = sqrtf(velocity_lattice[0]*velocity_lattice[0] +
                              velocity_lattice[1]*velocity_lattice[1] +
                              velocity_lattice[2]*velocity_lattice[2]);
    if (!isfinite(primitive->target_speed) ||
        fabsf(target_norm - primitive->target_speed) > 1e-5f) return 0u;
  } else if (primitive->target_speed != 0.0f) {
    return 0u;
  }
  if (primitive->kind == M5_PRIMITIVE_YAW) {
    float expected_delta, hinted_delta;
    if (!isfinite(primitive->yaw_rate_hint) || primitive->dir > 1u ||
        primitive->speed_id < 1u || primitive->speed_id > 2u) return 0u;
    if (node_id != 0u) {
      expected_delta = (primitive->dir == 0u ? -1.0f : 1.0f) *
          (float)primitive->speed_id * (M5_ASTAR_TWO_PI / 16.0f);
      hinted_delta = primitive->yaw_rate_hint * duration;
      if (fabsf(expected_delta - hinted_delta) > 1e-4f) return 0u;
    }
  }
  memcpy(end_velocity, velocity_lattice, sizeof(float) * 3u);

  end_yaw_bin = yaw_bin;
  if (primitive->kind == M5_PRIMITIVE_YAW) {
    int step = (int)primitive->speed_id;
    int next = (int)yaw_bin + (primitive->dir == 0u ? -step : step);
    end_yaw_bin = (uint8_t)(next & 15);
  }
  *end_yaw = yaw_bin_angle(end_yaw_bin);

  for (axis = 0u; axis < 3u; ++axis) {
    float average_velocity = 0.5f * (v0[axis] + end_velocity[axis]);
    if (primitive->kind == M5_PRIMITIVE_WAIT) average_velocity = 0.0f;
    end_position[axis] = p0[axis] + average_velocity * duration;
  }
  if (!nearest_voxel(end_position, &end_voxel)) return 0u;
  m5_local_map_voxel_center(end_voxel, end_position);

  for (axis = 0u; axis < 3u; ++axis)
    guidance_quintic_boundary(p0[axis], v0[axis], a0[axis], end_position[axis],
                              end_velocity[axis], 0.0f, duration, &q[axis]);
  {
    float yaw_unwrapped = yaw0 + remainderf(*end_yaw - yaw0, M5_ASTAR_TWO_PI);
    guidance_quintic_boundary(yaw0, yaw_rate0, 0.0f, yaw_unwrapped, 0.0f, 0.0f,
                              duration, &q[3]);
  }
  for (subdivision = 0u; subdivision < M5_ASTAR_EDGE_SUBDIVISIONS; ++subdivision) {
    uint16_t constraint = GUIDANCE_PLAN_CONSTRAINT_NONE;
    float t0 = duration * (float)subdivision / (float)M5_ASTAR_EDGE_SUBDIVISIONS;
    float t1 = duration * (float)(subdivision + 1u) / (float)M5_ASTAR_EDGE_SUBDIVISIONS;
    if (!guidance_curve_dynamics(q, t0, t1, r->safety_envelope,
                                 r->guidance_config, &constraint)) return 0u;
  }
  *end_time_bin = (uint8_t)(time_bin + ticks);
  *end_key = m5_kinodynamic_encode_key(end_voxel, end_velocity_bin,
                                       end_yaw_bin, *end_time_bin);
  return (uint8_t)(*end_key != UINT32_MAX);
}

static uint8_t dynamic_track_clear(const M5AstarRequest *r, const M5DynamicTrack *track,
                                   const GuidanceQuintic q[4], float curve_t0,
                                   float curve_t1, float elapsed_start_s)
{
  float lo[3], hi[3], center[3], distance2 = 0.0f, speed2 = 0.0f;
  float dt_mid, half_dt, radius;
  uint64_t abs_start, abs_end, abs_mid;
  unsigned axis;
  if (track->id == 0u) return 1u;
  abs_start = r->evaluation_time_us + (uint64_t)((elapsed_start_s + curve_t0) * 1e6f);
  abs_end = r->evaluation_time_us + (uint64_t)((elapsed_start_s + curve_t1) * 1e6f);
  if (abs_end > track->expires_us ||
      abs_end < track->sample_us ||
      abs_end - track->sample_us > r->map->max_dynamic_prediction_us) return 0u;
  abs_mid = abs_start + (abs_end - abs_start) / 2u;
  dt_mid = (float)(abs_mid - track->sample_us) * 1e-6f;
  half_dt = (float)(abs_end - abs_start) * 0.5e-6f;
  radius = track->radius_m + r->map->vehicle_radius_m + r->map->clearance_margin_m +
      r->map->uncertainty_inflation_factor *
        (track->position_std_m + track->velocity_std_mps * dt_mid);
  for (axis = 0u; axis < 3u; ++axis) {
    float v = track->velocity_ned_mps[axis];
    center[axis] = track->position_ned_m[axis] + v * dt_mid;
    speed2 += v * v;
    if (!guidance_quintic_bounds(&q[axis], curve_t0, curve_t1, 0u, &lo[axis], &hi[axis]))
      return 0u;
    if (center[axis] < lo[axis]) {
      float d = lo[axis] - center[axis];
      distance2 += d * d;
    } else if (center[axis] > hi[axis]) {
      float d = center[axis] - hi[axis];
      distance2 += d * d;
    }
  }
  radius += sqrtf(speed2) * half_dt;
  return (uint8_t)(distance2 > radius * radius);
}

static int edge_map_clear(const M5AstarRequest *r, const GuidanceQuintic q[4],
                          float edge_start_s, float duration)
{
  unsigned subdivision, axis, ix, iy, iz, track_id;
  float nav_std[3], nav_vel_std[3];
  for (axis = 0u; axis < 3u; ++axis) {
    nav_std[axis] = r->navigation->core.position_std_m[axis];
    nav_vel_std[axis] = r->navigation->core.velocity_std_mps[axis];
    if (!isfinite(nav_std[axis]) || nav_std[axis] < 0.0f ||
        !isfinite(nav_vel_std[axis]) || nav_vel_std[axis] < 0.0f) return 0;
  }
  for (subdivision = 0u; subdivision < M5_ASTAR_EDGE_SUBDIVISIONS; ++subdivision) {
    float t0 = duration * (float)subdivision / (float)M5_ASTAR_EDGE_SUBDIVISIONS;
    float t1 = duration * (float)(subdivision + 1u) / (float)M5_ASTAR_EDGE_SUBDIVISIONS;
    float low[3], high[3];
    unsigned volume = 1u;
    for (axis = 0u; axis < 3u; ++axis) {
      float lo, hi;
      float delay = (float)(r->evaluation_time_us - r->navigation->header.timestamp_sample_us) * 1e-6f;
      float horizon = edge_start_s + t1;
      float inflation = r->map->vehicle_radius_m + r->map->clearance_margin_m +
          r->map->uncertainty_inflation_factor *
            (nav_std[axis] + (horizon + delay) * nav_vel_std[axis]);
      if (!guidance_quintic_bounds(&q[axis], t0, t1, 0u, &lo, &hi)) return 0;
      low[axis] = lo - inflation - 1e-5f;
      high[axis] = hi + inflation + 1e-5f;
      if (low[axis] < -4.0f || high[axis] >= 4.0f) return 0;
      {
        int first = (int)floorf((low[axis] + 4.0f) / 0.5f);
        int last = (int)floorf((high[axis] + 4.0f) / 0.5f);
        if (first < 0 || last > 15 || last < first) return 0;
        volume *= (unsigned)(last - first + 1);
        if (volume > M5_ASTAR_CELL_CHECK_LIMIT) return 0;
      }
    }
    {
      int x0 = (int)floorf((low[0] + 4.0f) / 0.5f);
      int x1 = (int)floorf((high[0] + 4.0f) / 0.5f);
      int y0 = (int)floorf((low[1] + 4.0f) / 0.5f);
      int y1 = (int)floorf((high[1] + 4.0f) / 0.5f);
      int z0 = (int)floorf((low[2] + 4.0f) / 0.5f);
      int z1 = (int)floorf((high[2] + 4.0f) / 0.5f);
      for (iz = (unsigned)z0; iz <= (unsigned)z1; ++iz)
        for (iy = (unsigned)y0; iy <= (unsigned)y1; ++iy)
          for (ix = (unsigned)x0; ix <= (unsigned)x1; ++ix) {
            uint16_t voxel = (uint16_t)((iz * 16u + iy) * 16u + ix);
            uint8_t state;
            if (!m5_local_map_voxel_state(r->map, voxel, r->evaluation_time_us, &state) ||
                state != M5_MAP_CELL_FREE_CONFIRMED) return 0;
          }
    }
    for (track_id = 0u; track_id < M5_MAP_MAX_TRACKS; ++track_id)
      if (!dynamic_track_clear(r, &r->map->tracks[track_id], q,
            t0, t1, edge_start_s)) return 0;
  }
  return 1;
}

static uint8_t check_deadline(const M5AstarRequest *r, uint64_t *last_now)
{
  uint64_t now = r->monotonic_now(r->clock_context);
  if (now < *last_now) return 0u;
  *last_now = now;
  return (uint8_t)(now < r->planning_deadline_monotonic_us);
}

static M5PlannerResult validate_request(const M5AstarRequest *r)
{
  unsigned axis;
  uint16_t start_voxel, goal_voxel;
  if (r == NULL || r->map == NULL || r->navigation == NULL ||
      r->guidance_config == NULL || r->safety_envelope == NULL ||
      r->monotonic_now == NULL || r->evaluation_time_us == 0u ||
      r->planning_deadline_monotonic_us <= r->evaluation_time_us ||
      !finite3(r->start_position_ned_m) || !finite3(r->start_velocity_ned_mps) ||
      !finite3(r->start_acceleration_ned_mps2) || !finite3(r->goal_position_ned_m) ||
      !isfinite(r->start_yaw_rad) || !isfinite(r->start_yaw_rate_radps) ||
      !isfinite(r->goal_yaw_rad) || !isfinite(r->acceptance_radius_m) ||
      r->acceptance_radius_m <= 0.0f || !isfinite(r->terminal_speed_max_mps) ||
      r->terminal_speed_max_mps < 0.0f || !isfinite(r->yaw_tolerance_rad) ||
      r->yaw_tolerance_rad < 0.0f || r->yaw_tolerance_rad > M5_ASTAR_PI)
    return M5_PLANNER_INVALID_INPUT;
  if (r->evaluation_time_us > UINT64_MAX - GUIDANCE_MAX_DURATION_US ||
      r->navigation->header.message_type != MSG_NAVIGATION_SNAPSHOT ||
      r->navigation->header.profile_id != PRODUCT_PROFILE_ID ||
      r->navigation->header.boot_id == 0u ||
      r->navigation->header.time_epoch == 0u ||
      r->evaluation_time_us < r->navigation->header.timestamp_sample_us ||
      r->evaluation_time_us - r->navigation->header.timestamp_sample_us > 5000u ||
      !m5_local_map_is_ready(r->map, r->navigation, r->evaluation_time_us))
    return M5_PLANNER_INPUT_STALE;
  if (r->guidance_config->profile_hash != PRODUCT_PROFILE_ID ||
      r->guidance_config->config_version == 0u ||
      r->guidance_config->parameter_hash == 0u ||
      (r->guidance_config->payload.obstacle_source_mode != 1u &&
       r->guidance_config->payload.obstacle_source_mode != 2u) ||
      r->guidance_config->payload.max_dynamic_prediction_us <
          M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US ||
      r->map->max_dynamic_prediction_us < M5_MAP_REQUIRED_DYNAMIC_PREDICTION_US ||
      !isfinite(r->guidance_config->payload.vehicle_radius_m) ||
      r->guidance_config->payload.vehicle_radius_m <= 0.0f ||
      !isfinite(r->guidance_config->payload.clearance_margin_m) ||
      r->guidance_config->payload.clearance_margin_m < 0.0f ||
      !isfinite(r->guidance_config->payload.uncertainty_inflation_factor) ||
      r->guidance_config->payload.uncertainty_inflation_factor < 0.0f ||
      fabsf(r->map->vehicle_radius_m - r->guidance_config->payload.vehicle_radius_m) > 1e-5f ||
      fabsf(r->map->clearance_margin_m - r->guidance_config->payload.clearance_margin_m) > 1e-5f ||
      fabsf(r->map->uncertainty_inflation_factor -
            r->guidance_config->payload.uncertainty_inflation_factor) > 1e-5f ||
      !guidance_header_ok(&r->safety_envelope->header, &r->navigation->header,
                          MSG_SAFETY_ENVELOPE, r->evaluation_time_us) ||
      !isfinite(r->safety_envelope->max_linear_speed_mps) ||
      r->safety_envelope->max_linear_speed_mps <= 0.0f ||
      !isfinite(r->safety_envelope->max_yaw_rate_radps) ||
      r->safety_envelope->max_yaw_rate_radps <= 0.0f ||
      !isfinite(r->guidance_config->payload.mass_kg) ||
      r->guidance_config->payload.mass_kg <= 0.0f ||
      !isfinite(r->guidance_config->payload.gravity_mps2) ||
      r->guidance_config->payload.gravity_mps2 <= 0.0f)
    return M5_PLANNER_INVALID_INPUT;
  for (axis = 0u; axis < 3u; ++axis) {
    if (!isfinite(r->guidance_config->payload.max_velocity_ned_mps[axis]) ||
        r->guidance_config->payload.max_velocity_ned_mps[axis] <= 0.0f ||
        !isfinite(r->guidance_config->payload.max_acceleration_ned_mps2[axis]) ||
        r->guidance_config->payload.max_acceleration_ned_mps2[axis] <= 0.0f ||
        !isfinite(r->guidance_config->payload.max_jerk_ned_mps3[axis]) ||
        r->guidance_config->payload.max_jerk_ned_mps3[axis] <= 0.0f) return M5_PLANNER_INVALID_INPUT;
  }
  if (!nearest_voxel(r->start_position_ned_m, &start_voxel) ||
      !nearest_voxel(r->goal_position_ned_m, &goal_voxel)) return M5_PLANNER_OUT_OF_RANGE;
  if (r->map->tracks[0].id != 0u || r->map->tracks[1].id != 0u ||
      r->map->tracks[2].id != 0u || r->map->tracks[3].id != 0u ||
      r->map->tracks[4].id != 0u || r->map->tracks[5].id != 0u ||
      r->map->tracks[6].id != 0u || r->map->tracks[7].id != 0u) {
    unsigned i;
    for (i = 0u; i < M5_MAP_MAX_TRACKS; ++i)
      if (r->map->tracks[i].id != 0u &&
          (r->map->tracks[i].expires_us <= r->evaluation_time_us ||
           r->map->tracks[i].sample_us > r->evaluation_time_us))
        return M5_PLANNER_INPUT_STALE;
  }
  {
    uint8_t state;
    if (!m5_local_map_voxel_state(r->map, start_voxel, r->evaluation_time_us, &state) ||
        state != M5_MAP_CELL_FREE_CONFIRMED ||
        !m5_local_map_voxel_state(r->map, goal_voxel, r->evaluation_time_us, &state) ||
        state != M5_MAP_CELL_FREE_CONFIRMED) return M5_PLANNER_NO_PATH;
  }
  return M5_PLANNER_OK;
}

static uint8_t add_or_update_node(const M5AstarRequest *r, M5AstarWorkspace *w,
                                  uint32_t key, float g, uint16_t parent,
                                  uint8_t primitive_id, uint16_t *result_id)
{
  uint8_t found;
  uint16_t slot = hash_slot(w, key, &found);
  float p[3], yaw, h;
  uint16_t voxel;
  uint8_t velocity_bin, yaw_bin, time_bin;
  if (slot == M5_ASTAR_EMPTY_NODE ||
      !m5_kinodynamic_decode_key(key, &voxel, &velocity_bin, &yaw_bin, &time_bin))
    return 0u;
  m5_local_map_voxel_center(voxel, p);
  yaw = yaw_bin_angle(yaw_bin);
  h = heuristic(r, p, yaw);
  if (!isfinite(h) || !isfinite(g)) return 0u;
  if (found) {
    uint16_t id = w->hash_slots[slot];
    M5AstarNode *node = &w->nodes[id];
    if (g + M5_ASTAR_MIN_G_IMPROVEMENT_S < node->g) {
      node->g = g;
      node->f = g + h;
      node->parent = parent;
      w->parent_primitive[id] = primitive_id;
      if (bit_is_set(w->closed, id)) {
        bit_clear(w->closed, id);
        if (!heap_push(w, id)) return 0u;
      } else if (node->heap_index != M5_ASTAR_NO_HEAP_INDEX) {
        heap_sift_up(w, node->heap_index);
      } else if (!heap_push(w, id)) {
        return 0u;
      }
    }
    *result_id = id;
    return 1u;
  }
  if (w->node_count >= M5_ASTAR_NODE_CAPACITY) return 0u;
  {
    uint16_t id = w->node_count++;
    M5AstarNode *node = &w->nodes[id];
    node->key = key;
    node->g = g;
    node->f = g + h;
    node->parent = parent;
    node->heap_index = M5_ASTAR_NO_HEAP_INDEX;
    w->parent_primitive[id] = primitive_id;
    w->hash_slots[slot] = id;
    if (!heap_push(w, id)) return 0u;
    *result_id = id;
    return 1u;
  }
}

static M5PlannerResult reconstruct_path(const M5AstarRequest *r,
                                        const M5AstarWorkspace *w,
                                        uint16_t goal_id, M5AstarPath *out)
{
  uint16_t reverse[M5_ASTAR_MAX_PATH_SEGMENTS + 2u];
  unsigned count = 0u, i;
  uint16_t id = goal_id;
  M5AstarPath candidate;
  memset(&candidate, 0, sizeof(candidate));
  while (id != M5_ASTAR_EMPTY_NODE && count < M5_ASTAR_MAX_PATH_SEGMENTS + 2u) {
    reverse[count++] = id;
    if (id == 0u) break;
    id = w->nodes[id].parent;
  }
  if (count == 0u || reverse[count - 1u] != 0u)
    return M5_PLANNER_CAPACITY_EXCEEDED;
  if (count - 1u > M5_ASTAR_MAX_PATH_SEGMENTS)
    return M5_PLANNER_CAPACITY_EXCEEDED;
  candidate.segment_count = (uint8_t)(count - 1u);
  candidate.map_revision = r->map->map_revision;
  candidate.total_cost_s = w->nodes[goal_id].g;
  memcpy(candidate.position_ned_m[0], r->start_position_ned_m, sizeof(float) * 3u);
  memcpy(candidate.velocity_ned_mps[0], r->start_velocity_ned_mps, sizeof(float) * 3u);
  memcpy(candidate.acceleration_ned_mps2[0], r->start_acceleration_ned_mps2, sizeof(float) * 3u);
  candidate.yaw_rad[0] = r->start_yaw_rad;
  candidate.yaw_rate_radps[0] = r->start_yaw_rate_radps;
  candidate.offset_us[0] = 0u;
  for (i = 1u; i < count; ++i) {
    uint16_t node_id = reverse[count - 1u - i];
    uint16_t voxel;
    uint8_t velocity_bin, yaw_bin, time_bin;
    float velocity[3];
    if (!m5_kinodynamic_decode_key(w->nodes[node_id].key, &voxel,
                                   &velocity_bin, &yaw_bin, &time_bin) ||
        !m5_kinodynamic_velocity_bin(velocity_bin, velocity))
      return M5_PLANNER_CAPACITY_EXCEEDED;
    m5_local_map_voxel_center(voxel, candidate.position_ned_m[i]);
    memcpy(candidate.velocity_ned_mps[i], velocity, sizeof(velocity));
    candidate.acceleration_ned_mps2[i][0] = 0.0f;
    candidate.acceleration_ned_mps2[i][1] = 0.0f;
    candidate.acceleration_ned_mps2[i][2] = 0.0f;
    candidate.yaw_rad[i] = yaw_bin_angle(yaw_bin);
    candidate.yaw_rate_radps[i] = 0.0f;
    candidate.offset_us[i] = (uint32_t)time_bin * 200000u;
    candidate.primitive_index[i - 1u] = w->parent_primitive[node_id];
  }
  *out = candidate;
  return M5_PLANNER_OK;
}

M5PlannerResult m5_kinodynamic_astar_plan(const M5AstarRequest *r,
                                          M5AstarWorkspace *w,
                                          M5AstarPath *path,
                                          M5AstarStats *stats)
{
  M5PlannerResult result;
  uint16_t root_voxel, root_id = 0u;
  uint8_t start_velocity_bin, velocity_lattice, yaw_bin, primitive_count;
  uint32_t root_key;
  float root_h;
  uint64_t last_now;
  const M5KinodynamicPrimitive *primitives;
  if (path != NULL) memset(path, 0, sizeof(*path));
  if (stats != NULL) {
    memset(stats, 0, sizeof(*stats));
    stats->goal_node_id = M5_ASTAR_EMPTY_NODE;
  }
  if (w == NULL || path == NULL || stats == NULL) return M5_PLANNER_INVALID_INPUT;
  result = validate_request(r);
  if (result != M5_PLANNER_OK) {
    stats->result = result;
    return result;
  }
  last_now = r->monotonic_now(r->clock_context);
  if (last_now < r->evaluation_time_us) {
    stats->result = M5_PLANNER_INPUT_STALE;
    return M5_PLANNER_INPUT_STALE;
  }
  if (last_now >= r->planning_deadline_monotonic_us) {
    stats->result = M5_PLANNER_DEADLINE_MISSED;
    return M5_PLANNER_DEADLINE_MISSED;
  }
  memset(w, 0, sizeof(*w));
  for (root_id = 0u; root_id < M5_ASTAR_HASH_CAPACITY; ++root_id)
    w->hash_slots[root_id] = M5_ASTAR_EMPTY_NODE;
  if (!nearest_voxel(r->start_position_ned_m, &root_voxel)) {
    stats->result = M5_PLANNER_OUT_OF_RANGE;
    return M5_PLANNER_OUT_OF_RANGE;
  }
  (void)nearest_velocity_bin(r->start_velocity_ned_mps,
                             &start_velocity_bin, &velocity_lattice);
  (void)velocity_lattice; /* make_edge separately enforces root lattice rules */
  yaw_bin = nearest_yaw_bin(r->start_yaw_rad);
  root_key = m5_kinodynamic_encode_key(root_voxel, (uint8_t)start_velocity_bin,
                                       yaw_bin, 0u);
  if (root_key == UINT32_MAX) {
    stats->result = M5_PLANNER_INVALID_INPUT;
    return M5_PLANNER_INVALID_INPUT;
  }
  root_h = heuristic(r, r->start_position_ned_m, r->start_yaw_rad);
  w->node_count = 1u;
  w->nodes[0].key = root_key;
  w->nodes[0].g = 0.0f;
  w->nodes[0].f = root_h;
  w->nodes[0].parent = M5_ASTAR_EMPTY_NODE;
  w->nodes[0].heap_index = M5_ASTAR_NO_HEAP_INDEX;
  w->parent_primitive[0] = 0xFFu;
  {
    uint8_t found;
    uint16_t slot = hash_slot(w, root_key, &found);
    if (slot == M5_ASTAR_EMPTY_NODE || found) {
      stats->result = M5_PLANNER_CAPACITY_EXCEEDED;
      return M5_PLANNER_CAPACITY_EXCEEDED;
    }
    w->hash_slots[slot] = 0u;
  }
  if (!heap_push(w, 0u)) {
    stats->result = M5_PLANNER_CAPACITY_EXCEEDED;
    return M5_PLANNER_CAPACITY_EXCEEDED;
  }
  primitives = m5_kinodynamic_primitive_table(&primitive_count);
  if (primitive_count != 60u) {
    stats->result = M5_PLANNER_INVALID_INPUT;
    return M5_PLANNER_INVALID_INPUT;
  }
  for (;;) {
    uint16_t current_id;
    M5AstarNode current;
    float current_p[3], current_v[3], current_a[3], current_yaw, current_yaw_rate;
    uint8_t current_velocity_bin, current_yaw_bin, current_time_bin;
    uint8_t primitive_id;
    if (w->open_count == 0u) {
      result = M5_PLANNER_NO_PATH;
      break;
    }
    if (stats->expanded_nodes >= M5_ASTAR_MAX_EXPANSIONS) {
      result = M5_PLANNER_CAPACITY_EXCEEDED;
      break;
    }
    if (!check_deadline(r, &last_now)) {
      result = (last_now >= r->planning_deadline_monotonic_us) ?
          M5_PLANNER_DEADLINE_MISSED : M5_PLANNER_INPUT_STALE;
      break;
    }
    current_id = heap_pop(w);
    current = w->nodes[current_id];
    bit_set(w->closed, current_id);
    stats->expanded_nodes++;
    if (w->open_count > stats->peak_open_count)
      stats->peak_open_count = w->open_count;
    if (!node_state(r, &current, current_id, current_p, current_v, current_a,
                    &current_yaw, &current_yaw_rate,
                    &current_velocity_bin, &current_yaw_bin, &current_time_bin)) {
      result = M5_PLANNER_INVALID_INPUT;
      break;
    }
    if (goal_reached(r, current_p, current_v, current_yaw)) {
      stats->goal_node_id = current_id;
      result = reconstruct_path(r, w, current_id, path);
      break;
    }
    for (primitive_id = 0u; primitive_id < primitive_count &&
         primitive_id < M5_ASTAR_MAX_EDGE_CHECKS_PER_NODE; ++primitive_id) {
      GuidanceQuintic q[4];
      uint32_t next_key;
      float next_position[3], next_velocity[3], next_yaw, duration;
      uint8_t next_time_bin;
      uint16_t next_id;
      uint16_t voxel;
      uint8_t velocity_bin, yaw_bin_next;
      float next_h, next_g;
      if (stats->generated_edges != UINT16_MAX) stats->generated_edges++;
      if (!make_edge(r, &current, current_id, &primitives[primitive_id],
                     q, &next_key, next_position, next_velocity,
                     &next_yaw, &next_time_bin)) {
        if (stats->rejected_edges != UINT16_MAX) stats->rejected_edges++;
        continue;
      }
      duration = (float)primitives[primitive_id].duration_us * 1e-6f;
      if (!edge_map_clear(r, q, current.g, duration)) {
        if (stats->rejected_edges != UINT16_MAX) stats->rejected_edges++;
        continue;
      }
      if (!m5_kinodynamic_decode_key(next_key, &voxel, &velocity_bin,
                                     &yaw_bin_next, &next_time_bin)) {
        if (stats->rejected_edges != UINT16_MAX) stats->rejected_edges++;
        continue;
      }
      next_h = heuristic(r, next_position, next_yaw);
      next_g = current.g + duration;
      if (!isfinite(next_h) || !isfinite(next_g) ||
          !add_or_update_node(r, w, next_key, next_g, current_id,
                              primitive_id, &next_id)) {
        result = M5_PLANNER_CAPACITY_EXCEEDED;
        goto search_done;
      }
    }
  }
search_done:
  stats->node_count = w->node_count;
  if (result != M5_PLANNER_OK) memset(path, 0, sizeof(*path));
  stats->result = result;
  return result;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
