/* Exact bounded vertex enumeration. Lexicographic objective: roll/pitch
 * scale, closest collective, then yaw scale. No per-motor post-clipping. */
#include "attitude_rate_control/controller_internal.h"

int control_inverse4(const float in[4][4], float out[4][4]) {
  double a[4][8];
  int i, j, k, p;
  for (i = 0; i < 4; ++i)
    for (j = 0; j < 8; ++j)
      a[i][j] = j < 4 ? in[i][j] : (j - 4 == i);
  for (i = 0; i < 4; ++i) {
    p = i;
    for (j = i + 1; j < 4; ++j)
      if (fabs(a[j][i]) > fabs(a[p][i]))
        p = j;
    if (fabs(a[p][i]) < 1e-9)
      return 0;
    for (k = 0; k < 8; ++k) {
      double t = a[p][k];
      a[p][k] = a[i][k];
      a[i][k] = t;
    }
    {
      double d = a[i][i];
      for (k = 0; k < 8; ++k)
        a[i][k] /= d;
    }
    for (j = 0; j < 4; ++j)
      if (j != i) {
        double d = a[j][i];
        for (k = 0; k < 8; ++k)
          a[j][k] -= d * a[i][k];
      }
  }
  for (i = 0; i < 4; ++i)
    for (j = 0; j < 4; ++j)
      out[i][j] = (float)a[i][j + 4];
  return 1;
}
static int solve3(const double a[3][3], const double b[3], double x[3]) {
  double m[3][4];
  int i, j, k, p;
  for (i = 0; i < 3; ++i) {
    for (j = 0; j < 3; ++j)
      m[i][j] = a[i][j];
    m[i][3] = b[i];
  }
  for (i = 0; i < 3; ++i) {
    p = i;
    for (j = i + 1; j < 3; ++j)
      if (fabs(m[j][i]) > fabs(m[p][i]))
        p = j;
    if (fabs(m[p][i]) < 1e-12)
      return 0;
    for (k = i; k < 4; ++k) {
      double t = m[i][k];
      m[i][k] = m[p][k];
      m[p][k] = t;
    }
    {
      double d = m[i][i];
      for (k = i; k < 4; ++k)
        m[i][k] /= d;
    }
    for (j = 0; j < 3; ++j)
      if (j != i) {
        double d = m[j][i];
        for (k = i; k < 4; ++k)
          m[j][k] -= d * m[i][k];
      }
  }
  for (i = 0; i < 3; ++i)
    x[i] = m[i][3];
  return 1;
}
static int feasible(const double a[14][3], const double b[14],
                    const double x[3]) {
  int i;
  for (i = 0; i < 14; ++i)
    if (a[i][0] * x[0] + a[i][1] * x[1] + a[i][2] * x[2] > b[i] + 1e-7)
      return 0;
  return 1;
}
Status control_allocate_limited(const float inv[4][4], const float eff[4][4],
                           const float w[4], const float lo[4],
                           const float hi[4], float min_thrust,
                           float max_thrust, ControlAllocation *out) {
  double a[14][3] = {{0}}, b[14] = {0}, best = -1, tmin = 1e100, tmax = -1e100;
  double x[3], aa[3][3], bb[3], collective, ylo = 0, yhi = 1;
  int i, j, k, l;
  if (!out || !finite_array(w, 4) || !isfinite(min_thrust) ||
      !isfinite(max_thrust) || min_thrust > max_thrust)
    return STATUS_INVALID_ARGUMENT;
  for (i = 0; i < 4; ++i) {
    if (!isfinite(lo[i]) || !isfinite(hi[i]) || lo[i] > hi[i])
      return STATUS_OUT_OF_RANGE;
    a[2 * i][0] = inv[i][1] * w[1] + inv[i][2] * w[2];
    a[2 * i][1] = inv[i][0];
    a[2 * i][2] = inv[i][3] * w[3];
    for (j = 0; j < 3; ++j)
      a[2 * i + 1][j] = -a[2 * i][j];
    b[2 * i] = hi[i];
    b[2 * i + 1] = -lo[i];
  }
  a[8][0] = 1;
  b[8] = 1;
  a[9][0] = -1;
  a[10][2] = 1;
  b[10] = 1;
  a[11][2] = -1;
  a[12][1] = 1;
  b[12] = max_thrust;
  a[13][1] = -1;
  b[13] = -min_thrust;
  for (i = 0; i < 12; ++i)
    for (j = i + 1; j < 13; ++j)
      for (k = j + 1; k < 14; ++k) {
        for (l = 0; l < 3; ++l) {
          aa[0][l] = a[i][l];
          aa[1][l] = a[j][l];
          aa[2][l] = a[k][l];
        }
        bb[0] = b[i];
        bb[1] = b[j];
        bb[2] = b[k];
        if (solve3(aa, bb, x) && feasible(a, b, x) && x[0] > best)
          best = x[0];
      }
  if (best < 0)
    return STATUS_OUT_OF_RANGE;
  best = fmin(1, fmax(0, best));
  /* Slice at maximal RP; enumerate the T/yaw polygon vertices. */
  for (i = 0; i < 14; ++i)
    for (j = i + 1; j < 14; ++j) {
      double d = a[i][1] * a[j][2] - a[i][2] * a[j][1];
      if (fabs(d) < 1e-12)
        continue;
      double bi = b[i] - a[i][0] * best, bj = b[j] - a[j][0] * best;
      x[0] = best;
      x[1] = (bi * a[j][2] - bj * a[i][2]) / d;
      x[2] = (a[i][1] * bj - a[j][1] * bi) / d;
      if (feasible(a, b, x)) {
        tmin = fmin(tmin, x[1]);
        tmax = fmax(tmax, x[1]);
      }
    }
  if (tmin > tmax)
    return STATUS_NUMERIC_ERROR;
  collective = fmin(tmax, fmax(tmin, w[0]));
  for (i = 0; i < 8; ++i) {
    double rhs = b[i] - a[i][0] * best - a[i][1] * collective;
    if (a[i][2] > 1e-12)
      yhi = fmin(yhi, rhs / a[i][2]);
    else if (a[i][2] < -1e-12)
      ylo = fmax(ylo, rhs / a[i][2]);
    else if (rhs < -1e-6)
      return STATUS_NUMERIC_ERROR;
  }
  if (ylo > yhi + 1e-6)
    return STATUS_NUMERIC_ERROR;
  x[0] = best;
  x[1] = collective;
  x[2] = fmin(1, fmax(0, yhi));
  if (!feasible(a, b, x))
    return STATUS_NUMERIC_ERROR;
  memset(out, 0, sizeof(*out));
  out->roll_pitch_scale = (float)best;
  out->yaw_scale = (float)x[2];
  out->thrust_scale = fabsf(w[0]) > 1e-9f ? (float)(collective / w[0]) : 1;
  for (i = 0; i < 4; ++i) {
    out->motor[i] = (float)(a[2 * i][0] * best + a[2 * i][1] * collective +
                            a[2 * i][2] * x[2]);
    if (out->motor[i] < lo[i] - 1e-5f || out->motor[i] > hi[i] + 1e-5f)
      return STATUS_NUMERIC_ERROR;
    if (fabsf(out->motor[i] - lo[i]) < 1e-5f ||
        fabsf(out->motor[i] - hi[i]) < 1e-5f)
      out->saturated_mask |= 1u << i;
  }
  for (i = 0; i < 4; ++i) {
    for (j = 0; j < 4; ++j)
      out->achieved[i] += eff[i][j] * out->motor[j];
    out->residual[i] = out->achieved[i] - w[i];
  }
  return STATUS_OK;
}
Status control_allocate_from_previous(const float inv[4][4], const float eff[4][4],
                                      const float wrench[4], const float previous[4],
                                      const float lo[4], const float hi[4],
                                      float min_thrust, float max_thrust, ControlAllocation *out)
{
  float base[4] = {0}, delta[4], delta_lo[4], delta_hi[4];
  ControlAllocation result;
  for (unsigned motor = 0; motor < 4; ++motor) {
    if (!isfinite(previous[motor]) || previous[motor] < lo[motor] ||
        previous[motor] > hi[motor])
      return STATUS_OUT_OF_RANGE;
    delta_lo[motor] = lo[motor] - previous[motor];
    delta_hi[motor] = hi[motor] - previous[motor];
    for (unsigned axis = 0; axis < 4; ++axis)
      base[axis] += eff[axis][motor] * previous[motor];
  }
  if (base[0] < min_thrust || base[0] > max_thrust)
    return STATUS_OUT_OF_RANGE;
  for (unsigned axis = 0; axis < 4; ++axis)
    delta[axis] = wrench[axis] - base[axis];

  /* In a narrow slew box, scaling torques about zero can exclude all reachable
   * commands. The previous feasible command is the origin for this retry. */
  Status status = control_allocate_limited(inv, eff, delta, delta_lo, delta_hi,
                                           min_thrust - base[0], max_thrust - base[0], &result);
  if (status != STATUS_OK)
    return status;
  for (unsigned i = 0; i < 4; ++i) {
    result.motor[i] += previous[i];
    result.achieved[i] += base[i];
    result.residual[i] = result.achieved[i] - wrench[i];
  }
  result.thrust_scale = fabsf(wrench[0]) > 1e-9f ? result.achieved[0] / wrench[0] : 1.0f;
  *out = result;
  return STATUS_OK;
}

Status control_allocate(const float inv[4][4], const float eff[4][4],
                   const float wrench[4], const float lo[4], const float hi[4],
                   ControlAllocation *out) {
  float lower = 0, upper = 0;
  int i;
  for (i = 0; i < 4; ++i) {
    lower += lo[i];
    upper += hi[i];
  }
  return control_allocate_limited(inv, eff, wrench, lo, hi, lower, upper, out);
}
