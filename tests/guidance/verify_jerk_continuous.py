"""S06-112 (R18): independent continuous jerk proof with local sub-interval
Bernstein controls.

For every (state, step, u0, u1) the C API returns the production binary32
coefficients and the separate Bernstein/tail/consumer jerk bounds. The exact
maximum is computed with Fractions at the endpoints and the interior stationary
point. The negative control rebuilds the *local* sub-interval Bernstein controls
from the power-basis coefficients and evaluates the old
"Bernstein controls used as power coefficients" formula over the whole
[1, u_tail] interval, including its stationary point.
"""
import ctypes
import json
import struct
import sys
from fractions import Fraction
from math import comb

sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
from m5_product_envelope import exact_coefficients

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
STEPS = (20000, 40000)
U0S = (0.0, 0.25, 0.5, 0.75)
U1S = (0.25, 0.5, 0.75, 1.0)

ZERO = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
COUNTEREXAMPLE = ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [0.03, 0.0, 0.0],
                  [0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [-0.03, 0.0, 0.0])
ORDINARY = ([1.0, -2.0, -4.0], [2.0, 1.0, 0.5], [1.0, 0.5, 0.0],
            [1.05, -1.98, -4.0], [1.5, 0.8, 0.4], [0.5, 0.2, 0.0])
NEAR_LIMIT = ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [4.0, 4.0, 2.0],
              [0.1, 0.1, -5.0], [0.0, 0.0, 0.0], [-4.0, -4.0, -2.0])
TINY = ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.0e-18, -1.0e-18, 2.0e-18],
        [1.0e-20, -1.0e-20, -5.0], [0.0, 0.0, 0.0],
        [-1.0e-18, 1.0e-18, -2.0e-18])
HUGE = ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.0e15, -1.0e15, 5.0e14],
        [1.0e13, -1.0e13, -5.0], [0.0, 0.0, 0.0],
        [-1.0e15, 1.0e15, -5.0e14])
SUBNORMAL = ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0],
             [1.17549435e-38, 1.40129846e-45, 0.0],
             [1.0e-40, 0.0, -5.0], [0.0, 0.0, 0.0],
             [-1.17549435e-38, -1.40129846e-45, 0.0])


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def constant_acceleration_state(h):
    p0 = [0.0, 0.0, -5.0]
    v0 = [0.1, -0.2, 0.3]
    a0 = [1.0, -0.5, 0.25]
    p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h) for i in range(3)]
    v1 = [f32(v0[i] + a0[i] * h) for i in range(3)]
    return (p0, v0, a0, p1, v1, list(a0))


def polynomial_max_abs(coefficients, x0, x1):
    """max |c0+c1*x+c2*x^2| on [x0,x1], including stationary point."""
    values = []
    for x in (x0, x1):
        values.append(abs(sum(coefficients[k] * x ** k
                              for k in range(len(coefficients)))))
    if len(coefficients) == 3 and coefficients[2] != 0:
        x_star = -coefficients[1] / (2 * coefficients[2])
        if x0 <= x_star <= x1:
            values.append(abs(sum(coefficients[k] * x_star ** k
                                  for k in range(3))))
    return max(values)


def local_jerk_controls(coefficients, h, u0, u1):
    """Local sub-interval Bernstein jerk controls (J0..J2)."""
    c = [Fraction(value) for value in coefficients]
    hh = Fraction(h)
    t0 = hh * Fraction(u0)
    dt = hh * (Fraction(u1) - Fraction(u0))
    q = [Fraction(0)] * 6
    for k in range(6):
        q[k] = sum(c[i] * comb(i, k) * t0 ** (i - k) * dt ** k
                   for i in range(k, 6))
    b = [Fraction(0)] * 6
    for j in range(6):
        b[j] = sum(q[k] * comb(j, k) / comb(5, k) for k in range(j + 1))
    return [Fraction(60) * (b[k + 3] - 3 * b[k + 2] + 3 * b[k + 1] - b[k])
            / dt ** 3 for k in range(3)]


def exact_max(coefficients, s0, s1):
    c = [Fraction(value) for value in coefficients]
    values = []
    for s in (s0, s1):
        values.append(abs(6 * c[3] + 24 * c[4] * s + 60 * c[5] * s * s))
    if c[5] != 0:
        s_star = -c[4] / (5 * c[5])
        if s0 <= s_star <= s1:
            values.append(abs(6 * c[3] + 24 * c[4] * s_star +
                              60 * c[5] * s_star * s_star))
    return max(values)


class Probes:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        pointer = ctypes.POINTER(ctypes.c_float)
        self.coeff = library.m5_trajectory_safety_test_consumer_coefficients
        self.coeff.argtypes = [ctypes.c_uint32, ctypes.c_uint32] + [pointer] * 6 + [pointer]
        self.coeff.restype = ctypes.c_uint8
        self.components = library.m5_trajectory_safety_test_curve_jerk_components
        self.components.argtypes = [pointer] * 6 + [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            pointer, pointer, pointer, pointer]
        self.components.restype = ctypes.c_uint8


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_jerk_continuous.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    candidate_cases = 0
    legal_cases = 0
    fail_closed_cases = 0
    axis_cases_checked = 0
    bernstein_checks = 0
    tail_checks = 0
    consumer_checks = 0
    total_checks = 0
    wrong_tail_checked = 0
    wrong_tail_underbound = 0
    coverage = {name: {'min_slack': None}
                for name in ('bernstein', 'tail', 'consumer', 'total')}
    failures = []
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        states = [
            ('r14b_counterexample', COUNTEREXAMPLE),
            ('true_zero', ZERO),
            ('constant_acceleration', constant_acceleration_state(h)),
            ('ordinary', ORDINARY),
            ('near_limit', NEAR_LIMIT),
            ('tiny', TINY),
            ('huge', HUGE),
            ('subnormal', SUBNORMAL),
        ]
        for state_name, state in states:
            arrays = [(ctypes.c_float * 3)(*values) for values in state]
            coefficients = (ctypes.c_float * 18)()
            if not probes.coeff(step_us, 0, *arrays, coefficients):
                failures.append(f'coefficient API failed {state_name}')
                continue
            for u0 in U0S:
                for u1 in U1S:
                    if u1 <= u0:
                        continue
                    candidate_cases += 1
                    bernstein = (ctypes.c_float * 3)()
                    tail_bound = (ctypes.c_float * 3)()
                    consumer = (ctypes.c_float * 3)()
                    total = (ctypes.c_float * 3)()
                    if not probes.components(*arrays, ctypes.c_float(h),
                                             ctypes.c_float(u0), ctypes.c_float(u1),
                                             bernstein, tail_bound, consumer, total):
                        fail_closed_cases += 1
                        continue
                    legal_cases += 1
                    duration = Fraction(f32(h)) * (Fraction(f32(u1)) -
                                                   Fraction(f32(u0)))
                    s0 = Fraction(f32(h)) * Fraction(f32(u0))
                    s1 = Fraction(f32(h)) * Fraction(f32(u1))
                    tail = s1 + GAMMA2 * s1
                    for axis in range(3):
                        axis_cases_checked += 1
                        c_consumer = [Fraction(f32(coefficients[axis * 6 + k]))
                                      for k in range(6)]
                        c_math = exact_coefficients(
                            state[0][axis], state[1][axis], state[2][axis],
                            state[3][axis], state[4][axis], state[5][axis], h)
                        exact_body = exact_max(c_math, s0, s1)
                        exact_tail = exact_max(c_math, s1, tail)
                        exact_consumer = exact_max(c_consumer, s0, tail)
                        exact_total = max(exact_body, exact_tail, exact_consumer)
                        bernstein_checks += 1
                        tail_checks += 1
                        consumer_checks += 1
                        total_checks += 1
                        for name, value, target in (
                                ('bernstein', bernstein[axis], exact_body),
                                ('tail', tail_bound[axis], exact_tail),
                                ('consumer', consumer[axis], exact_consumer),
                                ('total', total[axis], exact_total)):
                            bound = Fraction(f32(value))
                            if bound < target:
                                failures.append(
                                    f'{name}<exact {state_name} {step_us} '
                                    f'u=({u0},{u1}) axis={axis} '
                                    f'bound={value} exact={float(target)}')
                            slack = bound - target
                            if coverage[name]['min_slack'] is None or \
                                    slack < coverage[name]['min_slack']:
                                coverage[name]['min_slack'] = slack
                        # Negative control: local sub-interval Bernstein controls
                        # used as power-basis coefficients over [1,u_tail].
                        jc = local_jerk_controls(c_math, f32(h), u0, u1)
                        u_tail = Fraction(1) + GAMMA2 * (s1 / duration)
                        wrong_max = polynomial_max_abs(jc, Fraction(1), u_tail)
                        true_tail = exact_tail
                        wrong_tail_checked += 1
                        if wrong_max < true_tail:
                            wrong_tail_underbound += 1
    if fail_closed_cases == 0:
        failures.append('subnormal fixture did not fail closed')
    if wrong_tail_underbound == 0:
        failures.append('old local Bernstein-as-power tail was never under-bound')
    report = {
        'test': 'S06-112',
        'seed': 'deterministic',
        'steps': list(STEPS),
        'u0s': list(U0S),
        'u1s': list(U1S),
        'states': 8,
        'candidate_cases': candidate_cases,
        'legal_cases': legal_cases,
        'fail_closed_cases': fail_closed_cases,
        'axis_cases_checked': axis_cases_checked,
        'bernstein_checks': bernstein_checks,
        'tail_checks': tail_checks,
        'consumer_checks': consumer_checks,
        'total_checks': total_checks,
        'wrong_tail_checked': wrong_tail_checked,
        'wrong_tail_underbound': wrong_tail_underbound,
        'coverage_slack': {name: {'min_slack': float(value['min_slack'])
                                  if value['min_slack'] is not None else None}
                           for name, value in coverage.items()},
        'failures': failures[:10],
    }
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print('M5_S06_112_CONTINUOUS_JERK_PASS '
          '(local sub-interval Bernstein, three separate bounds, exact Fraction '
          'endpoints+stationary, old formula whole tail including stationary)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
