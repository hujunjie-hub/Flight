/*
 * Fixed-time minimum-jerk quintic spline optimizer.
 *
 * This bounded math kernel is enabled for Host/SIL planner builds only.
 * H723/NONE keeps the direct single-segment quintic path and does not allocate
 * this workspace. The caller owns a fixed GuidanceMinimumJerkWorkspace.
 *
 * The objective is the exact integral of squared jerk. Internal knot positions
 * are fixed; only internal knot velocity/acceleration are optimized. The normal
 * matrix is block-tridiagonal (bandwidth 3) and solved by bounded Cholesky.
 */
#if defined(FLIGHT_M5_HOST_PLANNER)

#include <float.h>
#include <math.h>
#include <string.h>

#include "guidance_update/guidance_internal.h"

static uint8_t solve_coefficients(double p0, double v0, double a0,
                                  double p1, double v1, double a1,
                                  double duration, double c[6])
{
  double t2, t3, d, e, f;
  if (!isfinite(p0) || !isfinite(v0) || !isfinite(a0) ||
      !isfinite(p1) || !isfinite(v1) || !isfinite(a1) ||
      !isfinite(duration) || duration <= 0.0) {
    return 0u;
  }

  t2 = duration * duration;
  t3 = t2 * duration;
  d = p1 - p0 - v0 * duration - 0.5 * a0 * t2;
  e = v1 - v0 - a0 * duration;
  f = a1 - a0;

  c[0] = p0;
  c[1] = v0;
  c[2] = 0.5 * a0;
  c[3] = (10.0 * d - 4.0 * e * duration + 0.5 * f * t2) / t3;
  c[4] = (-15.0 * d + 7.0 * e * duration - f * t2) / (t3 * duration);
  c[5] = (6.0 * d - 3.0 * e * duration + 0.5 * f * t2) /
         (t3 * t2);

  return (uint8_t)(isfinite(c[0]) && isfinite(c[1]) && isfinite(c[2]) &&
                   isfinite(c[3]) && isfinite(c[4]) && isfinite(c[5]));
}

static double segment_jerk_cost(double p0, double v0, double a0,
                                double p1, double v1, double a1,
                                double duration)
{
  double c[6], t2, j0, j1, j2, integral;
  if (!solve_coefficients(p0, v0, a0, p1, v1, a1, duration, c)) {
    return NAN;
  }

  t2 = duration * duration;
  /* jerk(t) = j0 + j1*s + j2*s^2, s=t/duration. */
  j0 = 6.0 * c[3];
  j1 = 24.0 * c[4] * duration;
  j2 = 60.0 * c[5] * t2;
  integral = duration *
      (j0 * j0 + j0 * j1 +
       (j1 * j1 + 2.0 * j0 * j2) / 3.0 +
       0.5 * j1 * j2 + j2 * j2 / 5.0);
  if (!isfinite(integral) || integral < 0.0) {
    return NAN;
  }
  return integral;
}

static double local_segment_cost(const float position[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3],
                                 const float duration[GUIDANCE_MIN_JERK_MAX_SEGMENTS],
                                 uint8_t segment, uint8_t segment_count, uint8_t axis,
                                 const float start_velocity[3], const float start_acceleration[3],
                                 const float end_velocity[3], const float end_acceleration[3],
                                 double scale, const double trial[4])
{
  double inverse_scale = 1.0 / scale;
  double inverse_scale2 = inverse_scale * inverse_scale;
  double v0 = segment == 0u ? start_velocity[axis] : trial[0] * inverse_scale;
  double a0 = segment == 0u ? start_acceleration[axis] : trial[1] * inverse_scale2;
  double v1 = segment + 1u == segment_count ? end_velocity[axis] : trial[2] * inverse_scale;
  double a1 = segment + 1u == segment_count ? end_acceleration[axis] : trial[3] * inverse_scale2;

  return segment_jerk_cost(position[segment][axis], v0, a0,
                           position[segment + 1u][axis], v1, a1,
                           duration[segment]);
}

static double axis_total_cost(const float position[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3],
                              const float duration[GUIDANCE_MIN_JERK_MAX_SEGMENTS],
                              uint8_t segment_count, uint8_t axis,
                              const GuidanceMinimumJerkWorkspace *workspace)
{
  double total = 0.0;
  uint8_t segment;
  for (segment = 0u; segment < segment_count; ++segment) {
    double value = segment_jerk_cost(
        position[segment][axis], workspace->knot_velocity[segment],
        workspace->knot_acceleration[segment],
        position[segment + 1u][axis], workspace->knot_velocity[segment + 1u],
        workspace->knot_acceleration[segment + 1u], duration[segment]);
    if (!isfinite(value)) {
      return NAN;
    }
    total += value;
  }
  return isfinite(total) ? total : NAN;
}

static uint8_t add_banded_hessian(GuidanceMinimumJerkWorkspace *workspace,
                                  uint8_t row, uint8_t column, double value)
{
  uint8_t difference;
  if (row < column) {
    uint8_t temporary = row;
    row = column;
    column = temporary;
  }
  difference = (uint8_t)(row - column);
  if (difference > 3u || !isfinite(value)) {
    return 0u;
  }
  workspace->hessian[row][difference] += value;
  return (uint8_t)isfinite(workspace->hessian[row][difference]);
}

static uint8_t factor_and_solve(GuidanceMinimumJerkWorkspace *workspace, uint8_t count)
{
  double max_diagonal = 0.0;
  uint8_t i;

  for (i = 0u; i < count; ++i) {
    if (!isfinite(workspace->hessian[i][0]) ||
        !isfinite(workspace->gradient[i])) {
      return 0u;
    }
    max_diagonal = fmax(max_diagonal, workspace->hessian[i][0]);
  }
  if (!(max_diagonal > 0.0) || !isfinite(max_diagonal)) {
    return 0u;
  }

  /* In-place lower Cholesky; only the three lower diagonals are stored. */
  for (i = 0u; i < count; ++i) {
    uint8_t first = i > 3u ? (uint8_t)(i - 3u) : 0u;
    uint8_t j;
    double diagonal;

    for (j = first; j < i; ++j) {
      double sum = workspace->hessian[i][i - j];
      uint8_t k;
      for (k = first; k < j; ++k) {
        sum -= workspace->hessian[i][i - k] *
               workspace->hessian[j][j - k];
      }
      if (!isfinite(sum) || !(workspace->hessian[j][0] > 0.0)) {
        return 0u;
      }
      workspace->hessian[i][i - j] = sum / workspace->hessian[j][0];
    }

    diagonal = workspace->hessian[i][0];
    for (j = first; j < i; ++j) {
      double value = workspace->hessian[i][i - j];
      diagonal -= value * value;
    }
    if (!isfinite(diagonal) ||
        diagonal <= max_diagonal * DBL_EPSILON * 128.0) {
      return 0u;
    }
    workspace->hessian[i][0] = sqrt(diagonal);
  }

  /* Forward substitution: L*y = -gradient. */
  for (i = 0u; i < count; ++i) {
    uint8_t first = i > 3u ? (uint8_t)(i - 3u) : 0u;
    uint8_t k;
    double value = -workspace->gradient[i];
    for (k = first; k < i; ++k) {
      value -= workspace->hessian[i][i - k] * workspace->solution[k];
    }
    workspace->solution[i] = value / workspace->hessian[i][0];
    if (!isfinite(workspace->solution[i])) {
      return 0u;
    }
  }

  /* Back substitution: L^T*x = y. */
  for (i = count; i > 0u; --i) {
    uint8_t row = (uint8_t)(i - 1u);
    uint8_t last = (uint8_t)fmin((double)(count - 1u), (double)row + 3.0);
    uint8_t j;
    double value = workspace->solution[row];
    for (j = (uint8_t)(row + 1u); j <= last; ++j) {
      value -= workspace->hessian[j][j - row] * workspace->solution[j];
    }
    workspace->solution[row] = value / workspace->hessian[row][0];
    if (!isfinite(workspace->solution[row])) {
      return 0u;
    }
  }
  return 1u;
}

uint8_t guidance_minimum_jerk_optimize(
    const float knot_position[GUIDANCE_MIN_JERK_MAX_SEGMENTS + 1u][3],
    const float segment_duration_s[GUIDANCE_MIN_JERK_MAX_SEGMENTS],
    uint8_t segment_count,
    const float start_velocity[3], const float start_acceleration[3],
    const float end_velocity[3], const float end_acceleration[3],
    GuidanceMinimumJerkWorkspace *workspace,
    GuidanceQuintic output[GUIDANCE_MIN_JERK_MAX_SEGMENTS][3],
    double *initial_cost, double *optimized_cost)
{
  double total_duration, total_initial = 0.0, total_optimized = 0.0;
  double scale;
  uint16_t total_ticks = 0u;
  uint8_t axis, segment, knot, variable_count;

  if (output != NULL) {
    memset(output, 0, sizeof(GuidanceQuintic) *
                         GUIDANCE_MIN_JERK_MAX_SEGMENTS * 3u);
  }
  if (initial_cost != NULL) *initial_cost = 0.0;
  if (optimized_cost != NULL) *optimized_cost = 0.0;

  if (knot_position == NULL || segment_duration_s == NULL ||
      start_velocity == NULL || start_acceleration == NULL ||
      end_velocity == NULL || end_acceleration == NULL ||
      workspace == NULL || output == NULL ||
      initial_cost == NULL || optimized_cost == NULL ||
      segment_count == 0u || segment_count > GUIDANCE_MIN_JERK_MAX_SEGMENTS) {
    return 0u;
  }

  for (axis = 0u; axis < 3u; ++axis) {
    if (!isfinite(start_velocity[axis]) || !isfinite(start_acceleration[axis]) ||
        !isfinite(end_velocity[axis]) || !isfinite(end_acceleration[axis])) {
      return 0u;
    }
  }
  for (knot = 0u; knot <= segment_count; ++knot) {
    for (axis = 0u; axis < 3u; ++axis) {
      if (!isfinite(knot_position[knot][axis])) {
        return 0u;
      }
    }
  }
  for (segment = 0u; segment < segment_count; ++segment) {
    const double time_bin_s = (double)GUIDANCE_TIME_BIN_US * 1e-6;
    double ticks;
    double nearest_ticks;
    double duration = segment_duration_s[segment];
    if (!isfinite(duration) || duration < time_bin_s || duration > GUIDANCE_MAX_DURATION_S) {
      return 0u;
    }
    ticks = duration / time_bin_s;
    nearest_ticks = floor(ticks + 0.5);
    if (fabs(ticks - nearest_ticks) > 1e-6 ||
        nearest_ticks < 1.0 || nearest_ticks > GUIDANCE_MAX_TIME_BINS) {
      return 0u;
    }
    total_ticks = (uint16_t)(total_ticks + (uint16_t)nearest_ticks);
    if (total_ticks > GUIDANCE_MAX_TIME_BINS) {
      return 0u;
    }
  }
  total_duration = (double)total_ticks * ((double)GUIDANCE_TIME_BIN_US * 1e-6);

  scale = total_duration / segment_count;
  if (!isfinite(scale) || scale <= 0.0) {
    return 0u;
  }
  variable_count = (uint8_t)(2u * (segment_count - 1u));
  memset(workspace->candidate, 0, sizeof(workspace->candidate));

  for (axis = 0u; axis < 3u; ++axis) {
    double axis_initial = 0.0;
    double axis_optimized = 0.0;
    memset(workspace->hessian, 0, sizeof(workspace->hessian));
    memset(workspace->gradient, 0, sizeof(workspace->gradient));
    memset(workspace->solution, 0, sizeof(workspace->solution));

    /* Assemble local segment quadratics into a global block-tridiagonal system. */
    for (segment = 0u; segment < segment_count; ++segment) {
      uint8_t local_count = 0u;
      uint8_t local;
      double base_cost;

      memset(workspace->trial, 0, sizeof(workspace->trial));
      base_cost = local_segment_cost(knot_position, segment_duration_s,
          segment, segment_count, axis, start_velocity, start_acceleration,
          end_velocity, end_acceleration, scale, workspace->trial);
      if (!isfinite(base_cost)) {
        return 0u;
      }
      axis_initial += base_cost;

      if (segment > 0u) {
        workspace->active_local[local_count] = 0u;
        workspace->global_index[local_count++] = (uint8_t)(2u * (segment - 1u));
        workspace->active_local[local_count] = 1u;
        workspace->global_index[local_count++] = (uint8_t)(2u * (segment - 1u) + 1u);
      }
      if (segment + 1u < segment_count) {
        workspace->active_local[local_count] = 2u;
        workspace->global_index[local_count++] = (uint8_t)(2u * segment);
        workspace->active_local[local_count] = 3u;
        workspace->global_index[local_count++] = (uint8_t)(2u * segment + 1u);
      }

      for (local = 0u; local < local_count; ++local) {
        double positive, negative;
        memset(workspace->trial, 0, sizeof(workspace->trial));
        workspace->trial[workspace->active_local[local]] = 1.0;
        positive = local_segment_cost(knot_position, segment_duration_s,
            segment, segment_count, axis, start_velocity, start_acceleration,
            end_velocity, end_acceleration, scale, workspace->trial);
        workspace->trial[workspace->active_local[local]] = -1.0;
        negative = local_segment_cost(knot_position, segment_duration_s,
            segment, segment_count, axis, start_velocity, start_acceleration,
            end_velocity, end_acceleration, scale, workspace->trial);
        if (!isfinite(positive) || !isfinite(negative)) {
          return 0u;
        }
        workspace->plus[local] = positive;
        workspace->gradient[workspace->global_index[local]] +=
            0.5 * (positive - negative);
        if (!add_banded_hessian(workspace, workspace->global_index[local],
              workspace->global_index[local], positive + negative - 2.0 * base_cost)) {
          return 0u;
        }
      }

      for (local = 0u; local < local_count; ++local) {
        uint8_t other;
        for (other = (uint8_t)(local + 1u); other < local_count; ++other) {
          double combined;
          memset(workspace->trial, 0, sizeof(workspace->trial));
          workspace->trial[workspace->active_local[local]] = 1.0;
          workspace->trial[workspace->active_local[other]] = 1.0;
          combined = local_segment_cost(knot_position, segment_duration_s,
              segment, segment_count, axis, start_velocity, start_acceleration,
              end_velocity, end_acceleration, scale, workspace->trial);
          if (!isfinite(combined) ||
              !add_banded_hessian(workspace, workspace->global_index[local],
                  workspace->global_index[other],
                  combined - workspace->plus[local] - workspace->plus[other] + base_cost)) {
            return 0u;
          }
        }
      }
    }

    if (variable_count != 0u && !factor_and_solve(workspace, variable_count)) {
      return 0u;
    }

    workspace->knot_velocity[0] = start_velocity[axis];
    workspace->knot_acceleration[0] = start_acceleration[axis];
    workspace->knot_velocity[segment_count] = end_velocity[axis];
    workspace->knot_acceleration[segment_count] = end_acceleration[axis];
    for (knot = 1u; knot < segment_count; ++knot) {
      workspace->knot_velocity[knot] =
          workspace->solution[2u * (knot - 1u)] / scale;
      workspace->knot_acceleration[knot] =
          workspace->solution[2u * (knot - 1u) + 1u] / (scale * scale);
      if (!isfinite(workspace->knot_velocity[knot]) ||
          !isfinite(workspace->knot_acceleration[knot])) {
        return 0u;
      }
    }

    {
      double optimized_axis = axis_total_cost(knot_position, segment_duration_s,
          segment_count, axis, workspace);
      double tolerance = 1e-9 * fmax(1.0, axis_initial);
      if (!isfinite(optimized_axis) || optimized_axis > axis_initial + tolerance) {
        return 0u;
      }
      axis_optimized = optimized_axis;
    }
    for (segment = 0u; segment < segment_count; ++segment) {
      double coefficients[6];
      if (!solve_coefficients(knot_position[segment][axis],
              workspace->knot_velocity[segment], workspace->knot_acceleration[segment],
              knot_position[segment + 1u][axis],
              workspace->knot_velocity[segment + 1u],
              workspace->knot_acceleration[segment + 1u],
              segment_duration_s[segment], coefficients)) {
        return 0u;
      }
      memcpy(workspace->candidate[segment][axis].c, coefficients, sizeof(coefficients));
    }
    total_initial += axis_initial;
    total_optimized += axis_optimized;
  }

  for (segment = 0u; segment < segment_count; ++segment) {
    for (axis = 0u; axis < 3u; ++axis) {
      for (knot = 0u; knot < 6u; ++knot) {
        if (!isfinite(workspace->candidate[segment][axis].c[knot])) {
          return 0u;
        }
      }
    }
  }

  *initial_cost = total_initial;
  *optimized_cost = total_optimized;
  memcpy(output, workspace->candidate, sizeof(workspace->candidate));
  return 1u;
}

#endif /* FLIGHT_M5_HOST_PLANNER */
