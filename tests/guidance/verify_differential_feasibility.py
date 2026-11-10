"""S06-104 (R9): continuous feasibility bound with the real C consumer.

For product-legal segments this computes a *continuous* upper bound on
|consumer(s) - exact(s)| over the whole evaluation interval using Bernstein
control points of the difference polynomial (exact rational arithmetic), and
compares it with the placement resolution of each geometry channel:

  static   : float point coordinate + float AABB distance
  dynamic  : float track centre + radius + clearance margin
  geofence : float polygon vertex + clearance margin

The C consumer is used to confirm that the float-coefficient model reproduces
the real binary32 evaluation bit-exactly at the sampled offsets; the continuous
bound itself is mathematical, not a sampled maximum.

Verdict per channel: a boundary can be placed strictly between the two curves
only if the bound exceeds the channel resolution by the documented factor
(boundary offset + margin + two rounding steps = 4x).
"""
import ctypes
import json
import random
import struct
import sys
from fractions import Fraction
from math import comb
from pathlib import Path

SEED = 0x6D350104
STEPS = (20000, 40000)
RANDOM_CASES = 400
V_MAX = 4.0
A_MAX = 2.5
P_MAX = 2.0
FACTOR = 4


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def ulp(value):
    value = abs(value)
    if value == 0.0:
        return 2.0 ** -149
    exponent = struct.unpack('I', struct.pack('f', value))[0] >> 23 & 0xFF
    return 2.0 ** (exponent - 150)


def float_coefficients(p0, v0, a0, p1, v1, a1, h):
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
        p0, v0, f32(0.5 * a0),
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
        p0, v0, Fraction(1, 2) * a0,
        10 * d / h ** 3 - 4 * dv / h ** 2 + da / (2 * h),
        -15 * d / h ** 4 + 7 * dv / h ** 3 - da / h ** 2,
        6 * d / h ** 5 - 3 * dv / h ** 4 + da / (2 * h ** 3),
    ]


def bernstein_bound(coefficients, s_max):
    """max |p(s)| over [0, s_max] via the convex hull of Bernstein control points."""
    degree = len(coefficients) - 1
    scaled = [coefficients[k] * s_max ** k for k in range(degree + 1)]
    control = []
    for j in range(degree + 1):
        value = Fraction(0)
        for k in range(j + 1):
            value += comb(j, k) * scaled[k] / comb(degree, k)
        control.append(value)
    return max(abs(value) for value in control)




def envelope_legal(start_p, start_v, start_a, end_p, end_v, end_a, h):
    """Continuous check that the curve itself stays inside the product envelope."""
    for axis in range(3):
        exact = exact_coefficients(start_p[axis], start_v[axis], start_a[axis],
                                   end_p[axis], end_v[axis], end_a[axis], h)
        velocity = [Fraction(k) * exact[k] for k in range(1, 6)]
        acceleration = [Fraction(k * (k - 1)) * exact[k] for k in range(2, 6)]
        if bernstein_bound(velocity, Fraction(f32(h))) > Fraction(f32(V_MAX)):
            return False
        if bernstein_bound(acceleration, Fraction(f32(h))) > Fraction(f32(A_MAX)):
            return False
    return True

def continuous_bound(start_p, start_v, start_a, end_p, end_v, end_a, h):
    worst = Fraction(0)
    for axis in range(3):
        consumer = [Fraction(f32(value)) for value in float_coefficients(
            start_p[axis], start_v[axis], start_a[axis],
            end_p[axis], end_v[axis], end_a[axis], h)]
        exact = exact_coefficients(start_p[axis], start_v[axis], start_a[axis],
                                   end_p[axis], end_v[axis], end_a[axis], h)
        difference = [consumer[k] - exact[k] for k in range(6)]
        worst = max(worst, bernstein_bound(difference, Fraction(f32(h))))
    return worst


class Probes:
    def __init__(self, library_path):
        library = ctypes.CDLL(library_path)
        pointer = ctypes.POINTER(ctypes.c_float)
        self.sample = library.m5_trajectory_safety_test_consumer_interval_sample
        self.sample.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, ctypes.POINTER(ctypes.c_float)]
        self.sample.restype = ctypes.c_uint8


def verify_model(probes, step_us, state):
    """The C consumer must reproduce the float-coefficient model bit-exactly."""
    arrays = [(ctypes.c_float * 3)(*values) for values in state]
    position = (ctypes.c_float * 3)()
    seconds = ctypes.c_float()
    if not probes.sample(step_us, 0, step_us // 2, 1_000_000, *arrays,
                         position, ctypes.byref(seconds)):
        return False
    h = f32(step_us * f32(1e-6))
    coefficients = [float_coefficients(state[0][i], state[1][i], state[2][i],
                                       state[3][i], state[4][i], state[5][i], h)
                    for i in range(3)]
    s = seconds.value
    for axis in range(3):
        value = f32(coefficients[axis][5])
        for coefficient in reversed(coefficients[axis][:5]):
            value = f32(f32(value * s) + f32(coefficient))
        if value != position[axis]:
            return False
    return True


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_differential_feasibility.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    rnd = random.Random(SEED)
    worst = Fraction(0)
    worst_case = None
    model_ok = True
    cases = 0
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        states = []

        def consistent(p0, v0, a0, jerk):
            p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h
                      + jerk[i] * h ** 3 / 6.0) for i in range(3)]
            v1 = [f32(v0[i] + a0[i] * h + 0.5 * jerk[i] * h * h) for i in range(3)]
            a1 = [f32(a0[i] + jerk[i] * h) for i in range(3)]
            return (p0, v0, a0, p1, v1, a1)

        # deterministic corners: full speed/acceleration, zero jerk (legal)
        for sign in (1.0, -1.0):
            states.append(consistent(
                [sign * P_MAX, -sign * P_MAX, sign * P_MAX],
                [sign * V_MAX, sign * V_MAX, -sign * V_MAX],
                [sign * A_MAX, -sign * A_MAX, sign * A_MAX],
                [0.0, 0.0, 0.0]))
        # deterministic curvature: legal jerk inside the interval
        for sign in (1.0, -1.0):
            states.append(consistent(
                [sign * 1.0, -sign * 1.0, sign * 1.0],
                [sign * 2.0, -sign * 2.0, sign * 2.0],
                [0.0, 0.0, 0.0],
                [sign * 20.0, -sign * 20.0, sign * 20.0]))
        for _ in range(RANDOM_CASES):
            states.append(consistent(
                [f32(rnd.uniform(-P_MAX, P_MAX)) for _ in range(3)],
                [f32(rnd.uniform(-V_MAX, V_MAX)) for _ in range(3)],
                [f32(rnd.uniform(-A_MAX, A_MAX)) for _ in range(3)],
                [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]))
        for state in states:
            if not envelope_legal(*state, h):
                continue
            cases += 1
            if cases % 37 == 0 and not verify_model(probes, step_us, state):
                model_ok = False
            bound = continuous_bound(*state, h)
            if bound > worst:
                worst = bound
                worst_case = {"step_us": step_us,
                              "state": [[float(value) for value in group]
                                        for group in state],
                              "bound": float(bound)}
    point_ulp = max(ulp(P_MAX), ulp(1.0))
    radius_ulp = max(ulp(0.05), ulp(0.5))
    margin_ulp = ulp(0.15)
    channels = {
        "static": {"resolution_m": point_ulp, "note": "float point + AABB distance"},
        "dynamic": {"resolution_m": max(radius_ulp, margin_ulp),
                    "note": "float centre/radius + margin"},
        "geofence": {"resolution_m": max(ulp(1.0), margin_ulp),
                     "note": "float polygon vertex + margin"},
    }
    for name, channel in channels.items():
        ratio = float(worst) / channel["resolution_m"]
        channel["bound_over_resolution"] = ratio
        channel["constructible"] = ratio >= FACTOR
    report = {
        "test": "S06-104",
        "seed": f"0x{SEED:X}",
        "steps": list(STEPS),
        "cases": cases,
        "random_cases_per_step": RANDOM_CASES,
        "model_matches_c_consumer": model_ok,
        "continuous_bound_m": float(worst),
        "worst_case": worst_case,
        "factor_required": FACTOR,
        "channels": channels,
    }
    print(json.dumps(report, indent=2))
    if not model_ok:
        print("FAIL: the C consumer does not match the float-coefficient model")
        return 1
    print("M5_S06_104_CONTINUOUS_BOUND_PASS "
          f"(bound={float(worst):.3e} m; constructible channels: "
          f"{[name for name, channel in channels.items() if channel['constructible']]})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
