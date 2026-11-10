"""S06-107: position/velocity/acceleration error budgets from real C coefficients.

The C consumer returns its actual binary32 coefficients, its position/velocity/
acceleration and the local time it used. The script builds the exact rational
reference and splits the error into

    E_coefficient : |float-coefficient polynomial - exact polynomial| at s_c
    E_horner      : |C output - float-coefficient polynomial at s_c|
    E_time        : |exact(s_c) - exact(s_exact)|,  s_exact = local_us / 1e6

and checks the triangle inequality against the measured total. Continuous
budgets per component use Bernstein control points on [0, s_eval] with
s_eval = max(t_max, h) + gamma_2*t_max, and are combined as a 3-D norm with an
outward square root that never relies on a binary64 sqrt approximation.

Finite sampling only confirms the implementation; the budget itself is
continuous.
"""
import ctypes
import hashlib
import json
import random
import struct
import sys
from fractions import Fraction
from math import comb
from pathlib import Path

SEED = 0x6D350107
STEPS = (20000, 40000)
CASES = 150
V_MAX = 4.0
A_MAX = 2.5
P_MAX = 2.0
ROUNDINGS = (10, 12, 10)
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def float_up(value):
    bits = struct.unpack('I', struct.pack('f', value))[0]
    return struct.unpack('f', struct.pack('I', bits + 1))[0]


def sqrt_up(value):
    """Outward binary32 square root: candidate^2 >= value, checked exactly."""
    if value <= 0:
        return Fraction(0)
    candidate = f32(float(value) ** 0.5)
    while Fraction(candidate) ** 2 < value:
        candidate = float_up(candidate)
    return Fraction(candidate)


def gamma_up(n):
    nu = Fraction(n) * U
    return nu / (1 - nu) * (1 + Fraction(1, 1 << 52)) ** 2


def bernstein(coefficients, s_max):
    degree = len(coefficients) - 1
    scaled = [coefficients[k] * s_max ** k for k in range(degree + 1)]
    control = []
    for j in range(degree + 1):
        value = Fraction(0)
        for k in range(j + 1):
            value += comb(j, k) * scaled[k] / comb(degree, k)
        control.append(value)
    return max(abs(value) for value in control)


def exact_coefficients(p0, v0, a0, p1, v1, a1, h):
    p0, v0, a0 = Fraction(f32(p0)), Fraction(f32(v0)), Fraction(f32(a0))
    p1, v1, a1 = Fraction(f32(p1)), Fraction(f32(v1)), Fraction(f32(a1))
    h = Fraction(f32(h))
    d = (p1 - p0) - h * v0 - Fraction(1, 2) * h * h * a0
    dv = (v1 - v0) - h * a0
    da = a1 - a0
    return [p0, v0, Fraction(1, 2) * a0,
            10 * d / h ** 3 - 4 * dv / h ** 2 + da / (2 * h),
            -15 * d / h ** 4 + 7 * dv / h ** 3 - da / h ** 2,
            6 * d / h ** 5 - 3 * dv / h ** 4 + da / (2 * h ** 3)]


def poly(coefficients, s):
    total = Fraction(0)
    power = Fraction(1)
    for coefficient in coefficients:
        total += coefficient * power
        power *= s
    return total


def derivative(coefficients, order, s):
    total = Fraction(0)
    for i in range(order, len(coefficients)):
        factor = Fraction(1)
        for k in range(order):
            factor *= (i - k)
        total += factor * coefficients[i] * s ** (i - order)
    return total


def series(coefficients, order):
    """Coefficient list of the order-th derivative polynomial."""
    out = []
    for i in range(order, len(coefficients)):
        factor = Fraction(1)
        for k in range(order):
            factor *= (i - k)
        out.append(factor * coefficients[i])
    return out


class Probes:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        pointer = ctypes.POINTER(ctypes.c_float)
        self.full = library.m5_trajectory_safety_test_consumer_full_sample
        self.full.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, pointer, ctypes.POINTER(ctypes.c_float)]
        self.full.restype = ctypes.c_uint8
        self.limits = library.m5_trajectory_safety_test_product_limits
        self.limits.argtypes = [ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float)]
        self.limits.restype = ctypes.c_uint8
        self.coeff = library.m5_trajectory_safety_test_consumer_coefficients
        self.coeff.argtypes = [ctypes.c_uint32, ctypes.c_uint32] + [pointer] * 6 + [
            pointer]
        self.coeff.restype = ctypes.c_uint8



def read_limits(probes):
    speed = ctypes.c_float()
    accel = ctypes.c_float()
    velocity = (ctypes.c_float * 3)()
    acceleration = (ctypes.c_float * 3)()
    jerk = (ctypes.c_float * 3)()
    if not probes.limits(ctypes.byref(speed), ctypes.byref(accel), velocity,
                         acceleration, jerk):
        raise SystemExit('product limit API failed')
    return (float(speed.value), float(accel.value),
            [float(velocity[i]) for i in range(3)],
            [float(acceleration[i]) for i in range(3)],
            [float(jerk[i]) for i in range(3)])


def envelope_legal(state, h, step_us, v_max, a_max, per_axis_v, per_axis_a,
                   per_axis_j):
    """Per-axis, 3-D norm and jerk limits over the whole evaluation interval."""
    t_max = Fraction(step_us, 1000000)
    s_eval = max(t_max, Fraction(f32(h))) + GAMMA2 * t_max
    speed_sq = Fraction(0)
    accel_sq = Fraction(0)
    for axis in range(3):
        exact = exact_coefficients(state[0][axis], state[1][axis], state[2][axis],
                                   state[3][axis], state[4][axis], state[5][axis], h)
        velocity = [Fraction(k) * exact[k] for k in range(1, 6)]
        acceleration = [Fraction(k * (k - 1)) * exact[k] for k in range(2, 6)]
        jerk = [Fraction(k * (k - 1) * (k - 2)) * exact[k] for k in range(3, 6)]
        v_bound = bernstein(velocity, s_eval)
        a_bound = bernstein(acceleration, s_eval)
        j_bound = bernstein(jerk, s_eval)
        if v_bound > Fraction(f32(per_axis_v[axis])):
            return False
        if a_bound > Fraction(f32(per_axis_a[axis])):
            return False
        if j_bound > Fraction(f32(per_axis_j[axis])):
            return False
        speed_sq += v_bound * v_bound
        accel_sq += a_bound * a_bound
    return (speed_sq <= Fraction(f32(v_max)) ** 2 and
            accel_sq <= Fraction(f32(a_max)) ** 2)

def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_consumer_error_budget.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    v_max, a_max, per_axis_v, per_axis_a, per_axis_j = read_limits(probes)
    rnd = random.Random(SEED)
    checked = 0
    legal_cases = 0
    stress_cases = 0
    failures = []
    worst = {"ratio": 0.0}
    component_max = [{"coefficient": 0.0, "horner": 0.0, "time": 0.0} for _ in range(3)]
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        t_max = Fraction(step_us, 1000000)
        s_eval = max(t_max, Fraction(h)) + GAMMA2 * t_max
        for _ in range(CASES):
            p0 = [f32(rnd.uniform(-P_MAX, P_MAX)) for _ in range(3)]
            v0 = [f32(rnd.uniform(-V_MAX, V_MAX)) for _ in range(3)]
            a0 = [f32(rnd.uniform(-A_MAX, A_MAX)) for _ in range(3)]
            jerk = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
            p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h
                      + jerk[i] * h ** 3 / 6.0) for i in range(3)]
            v1 = [f32(v0[i] + a0[i] * h + 0.5 * jerk[i] * h * h) for i in range(3)]
            a1 = [f32(a0[i] + jerk[i] * h) for i in range(3)]
            state = (p0, v0, a0, p1, v1, a1)
            if not envelope_legal(state, h, step_us, v_max, a_max, per_axis_v,
                                  per_axis_a, per_axis_j):
                stress_cases += 1
                continue
            legal_cases += 1
            arrays = [(ctypes.c_float * 3)(*values)
                      for values in (p0, v0, a0, p1, v1, a1)]
            raw = (ctypes.c_float * 18)()
            if not probes.coeff(step_us, 0, *arrays, raw):
                failures.append("coefficient API failed")
                continue
            coefficients = [[Fraction(f32(raw[axis * 6 + k])) for k in range(6)]
                            for axis in range(3)]
            exact = [exact_coefficients(p0[axis], v0[axis], a0[axis],
                                        p1[axis], v1[axis], a1[axis], h)
                     for axis in range(3)]
            budgets = []
            for order in range(3):
                squares = [Fraction(0)]
                for axis in range(3):
                    difference = [coefficients[axis][k] - exact[axis][k]
                                  for k in range(6)]
                    if order:
                        difference = series(difference, order)
                        c_series = series(coefficients[axis], order)
                    else:
                        c_series = coefficients[axis]
                    coefficient = bernstein(difference, s_eval)
                    magnitude = sum(abs(c_series[k]) * s_eval ** k
                                    for k in range(len(c_series)))
                    horner = gamma_up(ROUNDINGS[order]) * magnitude
                    subnormal = (Fraction(ROUNDINGS[order], 1 << 149) *
                                 max(Fraction(1), s_eval) ** ROUNDINGS[order])
                    time = GAMMA2 * t_max * bernstein(
                        series(coefficients[axis], order + 1), s_eval)
                    component_max[order]["coefficient"] = max(
                        component_max[order]["coefficient"], float(coefficient))
                    component_max[order]["horner"] = max(
                        component_max[order]["horner"], float(horner))
                    component_max[order]["time"] = max(
                        component_max[order]["time"], float(time))
                    axis_total = coefficient + horner + subnormal + time
                    squares[0] += axis_total * axis_total
                budgets.append(sqrt_up(squares[0]))
            for offset_us in (0, step_us // 3, step_us // 2, 2 * step_us // 3, step_us):
                position = (ctypes.c_float * 3)()
                velocity = (ctypes.c_float * 3)()
                acceleration = (ctypes.c_float * 3)()
                seconds = ctypes.c_float()
                if not probes.full(step_us, 0, offset_us, 1_000_000, *arrays,
                                   position, velocity, acceleration,
                                   ctypes.byref(seconds)):
                    failures.append(f"full sample failed {step_us}/{offset_us}")
                    continue
                s_c = Fraction(seconds.value)
                s_exact = Fraction(offset_us, 1000000)
                measured = [position, velocity, acceleration]
                for order in range(3):
                    error_sq = Fraction(0)
                    for axis in range(3):
                        reference = derivative(exact[axis], order, s_exact)
                        error_sq += (Fraction(f32(measured[order][axis])) -
                                     reference) ** 2
                    error = sqrt_up(error_sq)
                    total = budgets[order]
                    ratio = float(error / total) if total else 0.0
                    checked += 1
                    if ratio > 1.0:
                        failures.append(
                            f"order{order} {step_us}/{offset_us}: {float(error):.3e} "
                            f"> {float(total):.3e}")
                    if ratio > worst["ratio"]:
                        worst = {"ratio": ratio, "order": order,
                                 "step_us": step_us, "offset_us": offset_us,
                                 "error_m": float(error), "budget_m": float(total)}
    report = {
        "test": "S06-107",
        "seed": f"0x{SEED:X}",
        "steps": list(STEPS),
        "cases": CASES,
        "legal_cases": legal_cases,
        "stress_cases": stress_cases,
        "checked": checked,
        "component_max": component_max,
        "worst": worst,
        "library_sha256": hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest(),
        "failures": failures[:10],
    }
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print("M5_S06_107_CONSUMER_BUDGET_PASS "
          "(position/velocity/acceleration, real C coefficients, 3-D outward norms)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
