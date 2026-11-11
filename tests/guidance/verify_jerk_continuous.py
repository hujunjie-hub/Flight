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
import hashlib
import json
import os
import platform
import struct
import sys
from pathlib import Path
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
            pointer, pointer, pointer, pointer, ctypes.POINTER(ctypes.c_uint8)]
        self.components.restype = ctypes.c_uint8
        self.status_values = library.m5_trajectory_safety_test_jerk_status_values
        self.status_values.argtypes = [ctypes.POINTER(ctypes.c_uint8)] * 6
        self.status_values.restype = None


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_jerk_continuous.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    raw = [ctypes.c_uint8() for _ in range(6)]
    probes.status_values(*[ctypes.byref(v) for v in raw])
    status_ok = raw[0].value
    status_invalid = raw[1].value
    status_build = raw[2].value
    status_bounds = raw[3].value
    production_invalid = raw[4].value
    production_numeric = raw[5].value
    invalid_status = ctypes.c_uint8(0)
    invalid_arrays = [(ctypes.c_float * 3)(*values) for values in COUNTEREXAMPLE]
    invalid_ok = probes.components(
        *invalid_arrays, ctypes.c_float(f32(20000 * f32(1e-6))),
        ctypes.c_float(0.5), ctypes.c_float(0.5),
        (ctypes.c_float * 3)(), (ctypes.c_float * 3)(),
        (ctypes.c_float * 3)(), (ctypes.c_float * 3)(),
        ctypes.byref(invalid_status))
    candidate_cases = 0
    legal_cases = 0
    fail_closed_cases = 0
    fail_closed_statuses = {}
    axis_cases_checked = 0
    bernstein_checks = 0
    tail_checks = 0
    consumer_checks = 0
    total_checks = 0
    wrong_tail_checked = 0
    wrong_tail_underbound = 0
    r14b_details = []
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
                    status = ctypes.c_uint8(0)
                    if not probes.components(*arrays, ctypes.c_float(h),
                                             ctypes.c_float(u0), ctypes.c_float(u1),
                                             bernstein, tail_bound, consumer, total,
                                             ctypes.byref(status)):
                        if state_name == 'subnormal' and status.value in (status_build, status_bounds):
                            fail_closed_cases += 1
                            fail_closed_statuses[status.value] =                                 fail_closed_statuses.get(status.value, 0) + 1
                            continue
                        failures.append(
                            f'unexpected C reject {state_name} {step_us} '
                            f'u=({u0},{u1}) status={status.value}')
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
                        under = wrong_max < true_tail
                        if under:
                            wrong_tail_underbound += 1
                        if state_name == 'r14b_counterexample':
                            endpoint0 = abs(sum(jc[k] for k in range(3)))
                            endpoint1 = abs(sum(jc[k] * u_tail ** k
                                               for k in range(3)))
                            stationary = None
                            if jc[2] != 0:
                                u_star = -jc[1] / (2 * jc[2])
                                if Fraction(1) <= u_star <= u_tail:
                                    stationary = abs(sum(jc[k] * u_star ** k
                                                         for k in range(3)))
                            r14b_details.append({
                                'step_us': step_us, 'u0': u0, 'u1': u1,
                                'axis': axis, 'wrong_max': float(wrong_max),
                                'wrong_max_fraction': str(wrong_max),
                                'true_tail': float(true_tail),
                                'true_tail_fraction': str(true_tail),
                                'endpoint_u1': float(endpoint0),
                                'endpoint_u_tail': float(endpoint1),
                                'stationary': (float(stationary)
                                               if stationary is not None else None),
                                'underbound': under})
    expected = {
        'test_ok': 0, 'test_invalid': 1, 'test_build': 2, 'test_bounds': 3,
        'production_invalid': 3, 'production_numeric': 11,
        'candidate_cases': 160, 'legal_cases': 140, 'fail_closed_cases': 20,
        'axis_cases_checked': 420, 'bernstein_checks': 420, 'tail_checks': 420,
        'consumer_checks': 420, 'total_checks': 420,
        'wrong_tail_checked': 420,
    }
    actual = {
        'test_ok': status_ok, 'test_invalid': status_invalid,
        'test_build': status_build, 'test_bounds': status_bounds,
        'production_invalid': production_invalid,
        'production_numeric': production_numeric,
        'candidate_cases': candidate_cases, 'legal_cases': legal_cases,
        'fail_closed_cases': fail_closed_cases,
        'axis_cases_checked': axis_cases_checked,
        'bernstein_checks': bernstein_checks, 'tail_checks': tail_checks,
        'consumer_checks': consumer_checks, 'total_checks': total_checks,
        'wrong_tail_checked': wrong_tail_checked,
    }
    for key, expected_value in expected.items():
        if actual[key] != expected_value:
            failures.append(
                f'matrix {key}: expected {expected_value} got {actual[key]}')
    if fail_closed_statuses != {status_build: 20}:
        failures.append(
            f'subnormal status matrix: expected {{{status_build}: 20}} '
            f'got {fail_closed_statuses}')
    if invalid_ok != 0 or invalid_status.value != status_invalid:
        failures.append(
            f'invalid input status mismatch ok={invalid_ok} '
            f'status={invalid_status.value}')
    if not any(item['underbound'] for item in r14b_details):
        failures.append('R14b counterexample has no under-bound axis')
    report = {
        'test': 'S06-112',
        'run_id': 'M5-06-R21-20261111',
        'code_commit': os.environ.get('FLIGHT_CODE_COMMIT', 'UNKNOWN'),
        'library_sha256': hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest(),
        'environment': {
            'python': platform.python_version(),
            'platform': platform.platform(),
            'compiler': os.environ.get('CC', 'host gcc'),
        },
        'seed': 'deterministic',
        'steps': list(STEPS),
        'u0s': list(U0S),
        'u1s': list(U1S),
        'states': 8,
        'candidate_cases': candidate_cases,
        'legal_cases': legal_cases,
        'fail_closed_cases': fail_closed_cases,
        'fail_closed_statuses': {str(k): v for k, v in fail_closed_statuses.items()},
        'invalid_input_status': invalid_status.value,
        'axis_cases_checked': axis_cases_checked,
        'bernstein_checks': bernstein_checks,
        'tail_checks': tail_checks,
        'consumer_checks': consumer_checks,
        'total_checks': total_checks,
        'wrong_tail_checked': wrong_tail_checked,
        'wrong_tail_underbound': wrong_tail_underbound,
        'r14b_axis_checks': len(r14b_details),
        'r14b_axis_underbound': sum(1 for item in r14b_details if item['underbound']),
        'r14b_details': r14b_details,
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
