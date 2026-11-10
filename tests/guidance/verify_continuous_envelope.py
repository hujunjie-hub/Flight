"""S06-094: continuous-time check of the MR-009 envelope (exact recomputation).

For a segment and a sub-interval [u0, u1] the validator claims

    |consumer_float(s) - exact_real(s_exact)| <= E    for every consumer time s

with s in [0, s_eval_max], s_eval_max = s_max + gamma_2*s_max,
s_max = h*u1 and gamma_2 = 2u/(1-2u). This verifier recomputes every term of E
independently with exact rational arithmetic over the whole enlarged interval:

* coefficient difference bounded by the Bernstein control points of the
  difference polynomial (convex hull, continuous — not a finite sample set);
* Horner rounding gamma_n * sum |cf_i| * s_eval_max^i, gamma_n rounded up;
* time quantisation delta_s * |d^k p/ds^k| bound at s_eval_max;
* subnormal amplification n * 2^-149 * max(1, s_eval_max)^n.

The independent sum must not exceed the published E. Both product preview step
constants and several adaptive sub-intervals are covered.
"""
from fractions import Fraction
import ctypes
import random
import struct
import sys

SEED = 0x6D350694
STEPS = (20000, 40000)
SMOOTH_SEGMENTS = 6
ADVERSARIAL_SEGMENTS = 6
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
ROUNDINGS = (10, 12, 10)
SUBINTERVALS = ((0.0, 1.0), (0.0, 0.5), (0.5, 1.0), (0.25, 0.75), (0.75, 1.0))


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def consumer_coefficients(p0, v0, a0, p1, v1, a1, h):
    p0, v0, a0 = f32(p0), f32(v0), f32(a0)
    p1, v1, a1 = f32(p1), f32(v1), f32(a1)
    h = f32(h)
    h2 = f32(h * h)
    h3 = f32(h2 * h)
    h4 = f32(h3 * h)
    h5 = f32(h4 * h)
    d = f32(f32(p1 - p0) - f32(h * v0))
    d = f32(d - f32(f32(f32(0.5 * h) * h) * a0))
    dv = f32(f32(v1 - v0) - f32(h * a0))
    da = f32(a1 - a0)
    return [
        p0,
        v0,
        f32(0.5 * a0),
        f32(f32(f32(f32(10.0 * d) / h3) - f32(f32(4.0 * dv) / h2))
            + f32(da / f32(2.0 * h))),
        f32(f32(f32(f32(-15.0 * d) / h4) + f32(f32(7.0 * dv) / h3)) - f32(da / h2)),
        f32(f32(f32(f32(6.0 * d) / h5) - f32(f32(3.0 * dv) / h4))
            + f32(da / f32(2.0 * h3))),
    ]


def exact_coefficients(p0, v0, a0, p1, v1, a1, h):
    p0, v0, a0 = Fraction(f32(p0)), Fraction(f32(v0)), Fraction(f32(a0))
    p1, v1, a1 = Fraction(f32(p1)), Fraction(f32(v1)), Fraction(f32(a1))
    h = Fraction(f32(h))
    d = (p1 - p0) - h * v0 - Fraction(1, 2) * h * h * a0
    dv = (v1 - v0) - h * a0
    da = a1 - a0
    return [
        p0,
        v0,
        Fraction(1, 2) * a0,
        10 * d / h ** 3 - 4 * dv / h ** 2 + da / (2 * h),
        -15 * d / h ** 4 + 7 * dv / h ** 3 - da / h ** 2,
        6 * d / h ** 5 - 3 * dv / h ** 4 + da / (2 * h ** 3),
    ]


def bernstein_max(coefficients, s_max):
    degree = len(coefficients) - 1
    scaled = [coefficients[k] * s_max ** k for k in range(degree + 1)]
    control = []
    for j in range(degree + 1):
        value = Fraction(0)
        for k in range(j + 1):
            value += _binomial(j, k) * scaled[k] / _binomial(degree, k)
        control.append(value)
    return max(abs(value) for value in control) if control else Fraction(0)


_BINOMIAL = {}


def _binomial(n, k):
    key = (n, k)
    if key not in _BINOMIAL:
        from math import comb
        _BINOMIAL[key] = comb(n, k)
    return _BINOMIAL[key]


def gamma_up(n):
    nu = Fraction(n) * U
    gamma = nu / (1 - nu)
    return gamma * (1 + Fraction(1, 1 << 52)) * (1 + Fraction(1, 1 << 52))


def derivative_bound(coefficients, order, s_max):
    total = Fraction(0)
    for i in range(order, len(coefficients)):
        factor = Fraction(1)
        for k in range(order):
            factor *= (i - k)
        total += abs(factor * coefficients[i]) * s_max ** (i - order)
    return total


class Envelope:
    def __init__(self, library_path):
        self.lib = ctypes.CDLL(library_path)
        pointer = ctypes.POINTER(ctypes.c_float)
        function = self.lib.m5_trajectory_safety_test_curve_envelope
        function.argtypes = [pointer] * 6 + [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float)]
        function.restype = ctypes.c_uint8
        self.function = function

    def __call__(self, segment, h, u0, u1):
        arrays = [(ctypes.c_float * 3)(*values) for values in segment]
        out = [ctypes.c_float() for _ in range(3)]
        ok = self.function(*arrays, ctypes.c_float(h), ctypes.c_float(u0),
                           ctypes.c_float(u1), *[ctypes.byref(v) for v in out])
        return ok, [Fraction(v.value) for v in out]


def smooth_segment(rnd, h):
    scale = rnd.choice([1.0, 10.0, 100.0])
    p0 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    v0 = [f32(rnd.uniform(-10.0, 10.0)) for _ in range(3)]
    a0 = [f32(rnd.uniform(-5.0, 5.0)) for _ in range(3)]
    jerk = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h + jerk[i] * h ** 3 / 6.0)
          for i in range(3)]
    v1 = [f32(v0[i] + a0[i] * h + 0.5 * jerk[i] * h * h) for i in range(3)]
    a1 = [f32(a0[i] + jerk[i] * h) for i in range(3)]
    return p0, v0, a0, p1, v1, a1


def adversarial_segment(rnd, h):
    scale = rnd.choice([0.5, 5.0, 50.0])
    p0 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    p1 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    v0 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    v1 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    a0 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    a1 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    return p0, v0, a0, p1, v1, a1


def check_segment(segment, h, u0, u1, envelope):
    """Return (failures, worst_ratio, min_slack) for one segment/sub-interval."""
    ok, limits = envelope(segment, h, u0, u1)
    if not ok:
        return ['production envelope rejected a valid sub-interval'], 0.0, None
    s_max = Fraction(f32(h)) * Fraction(u0 if False else 0)  # placeholder replaced below
    s_max = Fraction(f32(h)) * Fraction(u1).limit_denominator(10 ** 9)
    delta_s = GAMMA2 * s_max
    s_eval_max = s_max + delta_s
    failures = []
    worst = 0.0
    min_slack = None
    for axis in range(3):
        values = tuple(value[axis] for value in segment)
        consumer = [Fraction(f32(value)) for value in
                    consumer_coefficients(*values, h)]
        exact = exact_coefficients(*values, h)
        consumer_velocity = [Fraction(k) * consumer[k] for k in range(1, 6)]
        exact_velocity = [Fraction(k) * exact[k] for k in range(1, 6)]
        consumer_acceleration = [Fraction(k * (k - 1)) * consumer[k]
                                 for k in range(2, 6)]
        exact_acceleration = [Fraction(k * (k - 1)) * exact[k]
                              for k in range(2, 6)]
        for order, (consumer_coeffs, exact_coeffs, limit) in enumerate((
                (consumer, exact, limits[0]),
                (consumer_velocity, exact_velocity, limits[1]),
                (consumer_acceleration, exact_acceleration, limits[2]))):
            difference = [consumer_coeffs[k] - exact_coeffs[k]
                          for k in range(len(consumer_coeffs))]
            required = bernstein_max(difference, s_eval_max)
            magnitude = sum(abs(consumer_coeffs[k]) * s_eval_max ** k
                            for k in range(len(consumer_coeffs)))
            required += gamma_up(ROUNDINGS[order]) * magnitude
            required += delta_s * derivative_bound(consumer, order + 1, s_eval_max)
            floor_scale = max(Fraction(1), s_eval_max)
            required += Fraction(ROUNDINGS[order], 1 << 149) * floor_scale ** ROUNDINGS[order]
            slack = limit - required
            if slack < 0:
                failures.append(
                    f'axis{axis} order{order}: required {float(required):.6e} '
                    f'> envelope {float(limit):.6e}')
            if limit > 0:
                worst = max(worst, float(required / limit))
            min_slack = slack if min_slack is None else min(min_slack, slack)
    return failures, worst, min_slack


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_continuous_envelope.py <libm5_safety_oracle.so>')
    envelope = Envelope(sys.argv[1])
    rnd = random.Random(SEED)
    offenders = 0
    worst_ratio = 0.0
    min_slack = None
    checked = 0
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        for name, generator, count in (
                ('smooth', smooth_segment, SMOOTH_SEGMENTS),
                ('adversarial', adversarial_segment, ADVERSARIAL_SEGMENTS)):
            for _ in range(count):
                segment = generator(rnd, h)
                for u0, u1 in SUBINTERVALS:
                    failures, ratio, slack = check_segment(
                        segment, h, u0, u1, envelope)
                    checked += 1
                    worst_ratio = max(worst_ratio, ratio)
                    if slack is not None:
                        min_slack = slack if min_slack is None else min(min_slack, slack)
                    offenders += len(failures)
                    for failure in failures[:2]:
                        print(f'FAIL step={step_us} {name} [{u0},{u1}] {failure}')
    print(f"M5_S06_094_CONTINUOUS_ENVELOPE_REPORT seed=0x{SEED:X} steps={STEPS} "
          f"subintervals={len(SUBINTERVALS)} checks={checked} "
          f"max_required_over_envelope={worst_ratio:.6f} "
          f"min_slack={float(min_slack) if min_slack is not None else 0.0:.6e} "
          f"failures={offenders}")
    if offenders:
        return 1
    print("M5_S06_094_CONTINUOUS_ENVELOPE_PASS "
          "(exact rational Bernstein/mesh bounds on the enlarged time interval)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
