"""S06-094: continuous-time check of the MR-009 envelope (exact recomputation).

The envelope E returned by the production C API is claimed to bound

    |consumer_float(s) - exact_real(s)|   for every s in [0, s_max]

S06-085/086 sample finitely many times. This verifier recomputes every term of
that bound independently with exact rational arithmetic over the whole interval:

* coefficient difference  max_{s in [0,s_max]} |sum_i (cf_i - c_i) s^i|
  is bounded by the Bernstein control points of the difference polynomial
  (convex hull property), not by a finite set of samples;
* Horner rounding          gamma_n * sum_i |cf_i| s_max^i, with
  gamma_n = n*u/(1-n*u) rounded upward and n = 10/12/10;
* time quantisation        2u*s_max * |d^k p/ds^k|_bound;
* subnormal floor          n * 2^-149.

The independently computed sum must not exceed the published E. A shortfall
means the production envelope is optimistic and the verifier FAILs; a surplus
is reported as slack. Rational arithmetic means no floating-point oracle is
trusted.
"""
from fractions import Fraction
import ctypes
import random
import struct
import sys

SEED = 0x6D350694
STEP_US = 20000
SMOOTH_SEGMENTS = 8
ADVERSARIAL_SEGMENTS = 8
U = Fraction(1, 1 << 24)
ROUNDINGS = (10, 12, 10)  # position, velocity, acceleration


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


def scaled(values, factor):
    return [Fraction(factor) * value for value in values]


def bernstein_max(coefficients, s_max):
    """max |sum a_i s^i| over [0, s_max] via Bernstein control points."""
    degree = len(coefficients) - 1
    scaled_coefficients = [coefficients[k] * s_max ** k for k in range(degree + 1)]
    control = []
    for j in range(degree + 1):
        value = Fraction(0)
        for k in range(j + 1):
            value += _binomial(j, k) * scaled_coefficients[k] / _binomial(degree, k)
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
    # emulate two upward nextafter steps on a binary64 by adding one ulp-ish
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

    def __call__(self, segment, h):
        arrays = [(ctypes.c_float * 3)(*values) for values in segment]
        out = [ctypes.c_float() for _ in range(3)]
        ok = self.function(*arrays, ctypes.c_float(h), ctypes.c_float(0.0),
                           ctypes.c_float(1.0), *[ctypes.byref(v) for v in out])
        return ok, [Fraction(v.value) for v in out]


def smooth_segment(rnd):
    scale = rnd.choice([1.0, 10.0, 100.0])
    p0 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    v0 = [f32(rnd.uniform(-10.0, 10.0)) for _ in range(3)]
    a0 = [f32(rnd.uniform(-5.0, 5.0)) for _ in range(3)]
    jerk = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    h = f32(STEP_US * f32(1e-6))
    p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h + jerk[i] * h ** 3 / 6.0)
          for i in range(3)]
    v1 = [f32(v0[i] + a0[i] * h + 0.5 * jerk[i] * h * h) for i in range(3)]
    a1 = [f32(a0[i] + jerk[i] * h) for i in range(3)]
    return p0, v0, a0, p1, v1, a1


def adversarial_segment(rnd):
    scale = rnd.choice([0.5, 5.0, 50.0])
    p0 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    p1 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    v0 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    v1 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    a0 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    a1 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    return p0, v0, a0, p1, v1, a1


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_continuous_envelope.py <libm5_safety_oracle.so>')
    envelope = Envelope(sys.argv[1])
    h = f32(STEP_US * f32(1e-6))
    s_max = Fraction(h)
    time_scale = 2 * U * s_max
    worst_ratio = 0.0
    min_slack = None
    segments = 0
    failures = []

    regimes = (
        ('smooth', smooth_segment, random.Random(SEED), SMOOTH_SEGMENTS),
        ('adversarial', adversarial_segment,
         random.Random(SEED ^ 0x5A5A5A5A), ADVERSARIAL_SEGMENTS),
    )
    for name, generator, rnd, count in regimes:
        for _ in range(count):
            segment = generator(rnd)
            ok, limits = envelope(segment, h)
            if not ok:
                failures.append('production envelope rejected a valid segment')
                continue
            segments += 1
            for axis in range(3):
                values = tuple(value[axis] for value in segment)
                consumer = [Fraction(f32(value)) for value in
                            consumer_coefficients(*values, h)]
                exact = exact_coefficients(*values, h)
                difference = [consumer[k] - exact[k] for k in range(6)]

                consumer_velocity = [Fraction(k) * consumer[k] for k in range(1, 6)]
                exact_velocity = [Fraction(k) * exact[k] for k in range(1, 6)]
                difference_velocity = [consumer_velocity[k] - exact_velocity[k]
                                       for k in range(4)]
                consumer_acceleration = [Fraction(k * (k - 1)) * consumer[k]
                                         for k in range(2, 6)]
                exact_acceleration = [Fraction(k * (k - 1)) * exact[k]
                                      for k in range(2, 6)]
                difference_acceleration = [
                    consumer_acceleration[k] - exact_acceleration[k]
                    for k in range(4)]

                for order, (consumer_coeffs, difference_coeffs, limit) in enumerate((
                        (consumer, difference, limits[0]),
                        (consumer_velocity, difference_velocity, limits[1]),
                        (consumer_acceleration, difference_acceleration, limits[2]))):
                    required = bernstein_max(difference_coeffs, s_max)
                    magnitude = sum(abs(consumer_coeffs[k]) * s_max ** k
                                    for k in range(len(consumer_coeffs)))
                    required += gamma_up(ROUNDINGS[order]) * magnitude
                    required += time_scale * derivative_bound(
                        consumer, order + 1, s_max)
                    required += Fraction(ROUNDINGS[order], 1 << 149)
                    slack = limit - required
                    if slack < 0:
                        failures.append(
                            f'{name} axis{axis} order{order}: required {float(required):.6e} '
                            f'> envelope {float(limit):.6e}')
                    if limit > 0:
                        worst_ratio = max(worst_ratio, float(required / limit))
                    min_slack = slack if min_slack is None else min(min_slack, slack)

    print(f"M5_S06_094_CONTINUOUS_ENVELOPE_REPORT seed=0x{SEED:X} "
          f"segments={segments} axes=3 quantities=3 "
          f"max_required_over_envelope={worst_ratio:.6f} "
          f"min_slack={float(min_slack) if min_slack is not None else 'n/a':.6e} "
          f"failures={len(failures)}")
    if failures:
        for failure in failures[:5]:
            print('FAIL:', failure)
        return 1
    print("M5_S06_094_CONTINUOUS_ENVELOPE_PASS "
          "(exact rational Bernstein/mesh bounds over the whole interval, no sampling)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
