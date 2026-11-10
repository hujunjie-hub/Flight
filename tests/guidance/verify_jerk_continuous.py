"""S06-112: independent continuous jerk bound from real C coefficients.

The C consumer returns its actual binary32 coefficients. For each axis the jerk
polynomial is

    j(s) = 6*c3 + 24*c4*s + 60*c5*s^2.

All coefficients are converted to exact Fractions. The continuous maximum over
[s0, s_tail] is the maximum of the two endpoints and, when it lies inside, the
stationary point s_star = -c4/(5*c5). The production C bound
m5_trajectory_safety_test_curve_jerk() must be >= that exact maximum with no
tolerance.

This test also constructs the Bernstein jerk controls independently and proves
that the old "Bernstein control points used as power coefficients" tail formula
under-bounds at least one case; that old formula is therefore rejected.
"""
import ctypes
import json
import struct
import sys
from fractions import Fraction
from math import comb

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
STEPS = (20000, 40000)
U0S = (0.0, 0.25, 0.5, 0.75)
U1S = (0.25, 0.5, 0.75, 1.0)

STATES = (
    # R14b counterexample: true max |jerk| = 18 m/s^3 at 20 ms.
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [0.03, 0.0, 0.0],
     [0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [-0.03, 0.0, 0.0]),
    # zero jerk: constant acceleration along the segment.
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.0, 0.5, 0.0],
     [0.02, 0.01, -5.0], [2.0, 1.0, 0.0], [1.0, 0.5, 0.0]),
    # ordinary product-like state.
    ([1.0, -2.0, -4.0], [2.0, 1.0, 0.5], [1.0, 0.5, 0.0],
     [1.05, -1.98, -4.0], [1.5, 0.8, 0.4], [0.5, 0.2, 0.0]),
    # near product acceleration limits.
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [4.0, 4.0, 2.0],
     [0.1, 0.1, -5.0], [0.0, 0.0, 0.0], [-4.0, -4.0, -2.0]),
    # tiny coefficients (normal range).
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.0e-18, -1.0e-18, 2.0e-18],
     [1.0e-20, -1.0e-20, -5.0], [0.0, 0.0, 0.0], [-1.0e-18, 1.0e-18, -2.0e-18]),
    # large coefficients, still finite in binary64 and binary32 bound.
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.0e15, -1.0e15, 5.0e14],
     [1.0e13, -1.0e13, -5.0], [0.0, 0.0, 0.0], [-1.0e15, 1.0e15, -5.0e14]),
    # subnormal boundary: binary32 minimum normal and one subnormal ULP.
    ([0.0, 0.0, -5.0], [0.0, 0.0, 0.0], [1.17549435e-38, 1.40129846e-45, 0.0],
     [1.0e-40, 0.0, -5.0], [0.0, 0.0, 0.0], [-1.17549435e-38, -1.40129846e-45, 0.0]),
)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def bernstein_controls(coefficients, h):
    """Exact quintic Bernstein control points for p(s), s = h*u."""
    degree = 5
    scaled = [Fraction(coefficients[k]) * Fraction(h) ** k
              for k in range(degree + 1)]
    return [sum(scaled[k] * comb(j, k) / comb(degree, k)
                for k in range(j + 1)) for j in range(degree + 1)]


def jerk_bernstein_controls(coefficients, h):
    b = bernstein_controls(coefficients, h)
    hh = Fraction(h)
    return [Fraction(60) * (b[k + 3] - 3 * b[k + 2] + 3 * b[k + 1] - b[k])
            / hh ** 3 for k in range(3)]


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
        self.jerk = library.m5_trajectory_safety_test_curve_jerk
        self.jerk.argtypes = [pointer] * 6 + [ctypes.c_float, ctypes.c_float,
                                              ctypes.c_float, pointer]
        self.jerk.restype = ctypes.c_uint8


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_jerk_continuous.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    checked = 0
    fail_closed_cases = 0
    tail_cases = 0
    failures = []
    wrong_underbound = 0
    wrong_checked = 0
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        for state_index, state in enumerate(STATES):
            arrays = [(ctypes.c_float * 3)(*values) for values in state]
            coefficients = (ctypes.c_float * 18)()
            if not probes.coeff(step_us, 0, *arrays, coefficients):
                failures.append(f'coefficient API failed step={step_us}')
                continue
            for u0 in U0S:
                for u1 in U1S:
                    if u1 <= u0:
                        continue
                    bound = (ctypes.c_float * 3)()
                    if not probes.jerk(*arrays, ctypes.c_float(h),
                                       ctypes.c_float(u0), ctypes.c_float(u1),
                                       bound):
                        # The subnormal fixture is intentionally outside the
                        # normal-range model; the validator must fail closed.
                        if state_index == len(STATES) - 1:
                            fail_closed_cases += 1
                            continue
                        failures.append(
                            f'jerk API failed state={state_index} step={step_us} u=({u0},{u1})')
                        continue
                    duration = Fraction(f32(h)) * (Fraction(f32(u1)) -
                                                   Fraction(f32(u0)))
                    s0 = Fraction(f32(h)) * Fraction(f32(u0))
                    s1 = Fraction(f32(h)) * Fraction(f32(u1))
                    tail = s1 + GAMMA2 * s1
                    for axis in range(3):
                        c = [Fraction(f32(coefficients[axis * 6 + k]))
                             for k in range(6)]
                        exact_body = exact_max(c, s0, s1)
                        exact_tail = exact_max(c, s1, tail)
                        exact = max(exact_body, exact_tail)
                        checked += 1
                        tail_cases += 1
                        if not (Fraction(f32(bound[axis])) >= exact_body):
                            failures.append(
                                f'bound<body step={step_us} u=({u0},{u1}) '
                                f'axis={axis} bound={bound[axis]} '
                                f'exact={float(exact_body)}')
                        if not (Fraction(f32(bound[axis])) >= exact_tail):
                            failures.append(
                                f'bound<tail step={step_us} u=({u0},{u1}) '
                                f'axis={axis} bound={bound[axis]} '
                                f'exact={float(exact_tail)}')
                        # Negative control: the old tail conversion treats the
                        # Bernstein jerk controls as power coefficients.
                        jc = jerk_bernstein_controls(
                            [Fraction(f32(coefficients[axis * 6 + k]))
                             for k in range(6)], f32(h))
                        ratio = s1 / duration
                        u_tail = Fraction(1) + GAMMA2 * ratio
                        if jc[2] != 0:
                            p0 = jc[0]
                            p1 = 2 * (jc[1] - jc[0])
                            p2 = jc[0] - 2 * jc[1] + jc[2]
                            correct_tail = p0 + p1 * u_tail + p2 * u_tail * u_tail
                            wrong_tail = ((jc[2] * u_tail + jc[1]) * u_tail +
                                          jc[0])
                            exact_tail = exact_max(c, s1, tail)
                            wrong_checked += 1
                            if abs(wrong_tail) < exact_tail:
                                wrong_underbound += 1
                            if abs(correct_tail) < exact_tail:
                                failures.append(
                                    f'power conversion tail under-bound '
                                    f'step={step_us} u=({u0},{u1}) axis={axis}')
    if wrong_underbound == 0:
        failures.append('old Bernstein-as-power tail was never under-bound')
    report = {
        'test': 'S06-112',
        'seed': 'deterministic',
        'steps': list(STEPS),
        'u0s': list(U0S),
        'u1s': list(U1S),
        'states': len(STATES),
        'checked': checked,
        'fail_closed_cases': fail_closed_cases,
        'tail_cases': tail_cases,
        'wrong_tail_checked': wrong_checked,
        'wrong_tail_underbound': wrong_underbound,
        'failures': failures[:10],
    }
    print(json.dumps(report, indent=2))
    if fail_closed_cases == 0:
        failures.append('subnormal fixture did not fail closed')
    if failures:
        return 1
    print('M5_S06_112_CONTINUOUS_JERK_PASS '
          '(real C coefficients, exact Fraction endpoints+stationary point, '
          'correct power tail, old Bernstein-as-power tail rejected)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
