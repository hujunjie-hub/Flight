"""S06-104 (R8): quantitative feasibility of a product-envelope differential case.

The differential tests S06-100..102 need a boundary strictly between the
binary64 reference curve and the binary32 consumer curve while every input obeys
the product SafetyEnvelope (|v| <= 4 m/s, |a| <= 2.5 m/s2, 20 ms preview).

Two independent quantities decide feasibility:

 1. numerical separation: max |consumer - exact| over the interval;
 2. placement resolution: the binary32 ulp of the quantities that encode the
    boundary (clearance margin near 0.15 m, obstacle coordinate near 0.5 m).

A robust test needs the separation to be several times the placement ulp;
otherwise the "straddling" boundary is inside the rounding noise of the
placement itself and the outcome is decided by float bookkeeping, not by the
consumer error. This script reports both and states the verdict.
"""
import ctypes
import random
import struct
import sys
from fractions import Fraction

SEED = 0x6D350104
STEP_US = 20000
SEGMENTS = 2000
SAMPLES = 129
V_MAX = 4.0
A_MAX = 2.5
ROBUST_ULPS = 8.0


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def ulp(value):
    value = abs(value)
    if value == 0.0:
        return 2.0 ** -149
    exponent = struct.unpack('I', struct.pack('f', value))[0] >> 23 & 0xFF
    return 2.0 ** (exponent - 150)


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


def consumer_at(coefficients, s):
    result = f32(coefficients[5])
    for coefficient in reversed(coefficients[:5]):
        result = f32(f32(result * s) + f32(coefficient))
    return result


def exact_at(coefficients, s):
    total = Fraction(0)
    power = Fraction(1)
    for coefficient in coefficients:
        total += coefficient * power
        power *= s
    return total


def main():
    rnd = random.Random(SEED)
    h = f32(STEP_US * f32(1e-6))
    worst = 0.0
    worst_ulps = 0.0
    placement_ulps = 0.0
    worst_case = None
    for _ in range(SEGMENTS):
        p0 = [f32(rnd.uniform(-2.0, 2.0)) for _ in range(3)]
        v0 = [f32(rnd.uniform(-V_MAX, V_MAX)) for _ in range(3)]
        a0 = [f32(rnd.uniform(-A_MAX, A_MAX)) for _ in range(3)]
        p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h) for i in range(3)]
        v1 = [f32(v0[i] + a0[i] * h) for i in range(3)]
        a1 = [f32(a0[i]) for i in range(3)]
        for axis in range(3):
            consumer = consumer_coefficients(p0[axis], v0[axis], a0[axis],
                                             p1[axis], v1[axis], a1[axis], h)
            exact = exact_coefficients(p0[axis], v0[axis], a0[axis],
                                       p1[axis], v1[axis], a1[axis], h)
            for sample in range(SAMPLES):
                offset_us = (sample * STEP_US) // (SAMPLES - 1)
                s = f32(f32(offset_us) * f32(1e-6))
                value = consumer_at(consumer, s)
                reference = exact_at(exact, Fraction(s))
                deviation = abs(Fraction(value) - reference)
                spacing = ulp(float(reference))
                if float(deviation) > worst:
                    worst = float(deviation)
                    worst_case = (axis, sample)
                if spacing:
                    worst_ulps = max(worst_ulps, float(deviation) / spacing)
                # Placement quantities: clearance margin near 0.15 m and the
                # obstacle coordinate near 0.5 m (voxel centre grid).
                placement_ulps = max(placement_ulps,
                                     ulp(0.15), ulp(0.5), ulp(0.25))
    ratio = worst / placement_ulps if placement_ulps else 0.0
    print(f"M5_S06_104_DIFFERENTIAL_FEASIBILITY seed=0x{SEED:X} "
          f"segments={SEGMENTS} samples={SAMPLES} v_max={V_MAX} a_max={A_MAX} "
          f"max_deviation_m={worst:.6e} "
          f"max_deviation_over_position_ulp={worst_ulps:.1f} "
          f"placement_ulp_m={placement_ulps:.3e} "
          f"deviation_over_placement_ulp={ratio:.3f} "
          f"required_ulps={ROBUST_ULPS}")
    robust = ratio >= ROBUST_ULPS
    print(f"M5_S06_104_RESULT={'FEASIBLE' if robust else 'INFEASIBLE'} "
          f"(a robust boundary needs >= {ROBUST_ULPS} placement ulps of separation; "
          f"obtained {ratio:.3f})")
    if not robust:
        print("The product envelope caps the consumer deviation below the "
              "placement resolution of the clearance margin and the voxel grid; "
              "S06-100..102 remain NOT_IMPLEMENTED instead of being forced with "
              "non-product envelopes.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
