"""S06-105: component error budget and real-C verification.

E_total = E_coefficient + E_horner + E_subnormal + E_time, computed per axis with
continuous Bernstein bounds on [0, s_eval_max], then combined into a 3-D norm
outward. The real C consumer is called at several offsets of every case and the
measured error must stay below the proven budget; the worst case is checked on a
dense grid.

Finite sampling is used only to confirm that the implementation matches the
budget - the budget itself is continuous.
"""
import ctypes
import hashlib
import json
import random
import struct
import sys
from fractions import Fraction
from math import comb, sqrt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from m5_product_envelope import read_product_envelope
from m5_product_envelope import envelope_legal as product_legal

SEED = 0x6D350105
STEPS = (20000, 40000)
CASES_PER_STEP = 120
P_MAX = 2.0
ROUNDINGS = (10, 12, 10)
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def up(value):
    """Next binary32 value above a real (outward rounding)."""
    result = f32(float(value))
    if Fraction(result) < value:
        result = struct.unpack('f', struct.pack('I',
                  struct.unpack('I', struct.pack('f', result))[0] + 1))[0]
    return Fraction(result)


def sqrt_up(value):
    """Outward binary32 square root checked in exact arithmetic."""
    if value <= 0:
        return Fraction(0)
    candidate = f32(float(value) ** 0.5)
    while Fraction(candidate) ** 2 < value:
        candidate = struct.unpack('f', struct.pack(
            'I', struct.unpack('I', struct.pack('f', candidate))[0] + 1))[0]
    return Fraction(candidate)


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


def gamma_up(n):
    nu = Fraction(n) * U
    return nu / (1 - nu) * (1 + Fraction(1, 1 << 52)) ** 2


def budget(start_p, start_v, start_a, end_p, end_v, end_a, h, step_us,
           c_series):
    """Return per-component 3-D budgets (E_coeff, E_horner, E_subnormal, E_time)."""
    t_max = Fraction(step_us, 1000000)
    delta_s = GAMMA2 * t_max
    s_eval = max(t_max, Fraction(f32(h))) + delta_s
    components = [Fraction(0), Fraction(0), Fraction(0), Fraction(0)]
    for axis in range(3):
        consumer = c_series[axis]
        exact = exact_coefficients(start_p[axis], start_v[axis], start_a[axis],
                                   end_p[axis], end_v[axis], end_a[axis], h)
        difference = [consumer[k] - exact[k] for k in range(6)]
        coefficient = bernstein(difference, s_eval)
        magnitude = sum(abs(consumer[k]) * s_eval ** k for k in range(6))
        horner = gamma_up(ROUNDINGS[0]) * magnitude
        subnormal = Fraction(ROUNDINGS[0], 1 << 149) * max(Fraction(1), s_eval) ** ROUNDINGS[0]
        derivative = sum(Fraction(k) * abs(consumer[k]) * s_eval ** (k - 1)
                         for k in range(1, 6))
        time = delta_s * derivative
        for index, value in enumerate((coefficient, horner, subnormal, time)):
            components[index] += value * value
    return tuple(sqrt_up(value) for value in components)


class Probes:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        self.library = library
        pointer = ctypes.POINTER(ctypes.c_float)
        self.sample = library.m5_trajectory_safety_test_consumer_interval_sample
        self.sample.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, ctypes.POINTER(ctypes.c_float)]
        self.sample.restype = ctypes.c_uint8
        self.active = library.m5_trajectory_safety_test_active_segment
        self.active.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_uint32)]
        self.active.restype = ctypes.c_uint8
        self.local = library.m5_trajectory_safety_test_consumer_local_seconds
        self.local.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                               ctypes.c_uint64, ctypes.POINTER(ctypes.c_float)]
        self.local.restype = ctypes.c_uint8

    def coefficients(self, step_us, interval, offset_us, state):
        arrays = [(ctypes.c_float * 3)(*values) for values in state]
        endpoints = (ctypes.c_float * 18)()
        coefficients = (ctypes.c_float * 18)()
        seconds = ctypes.c_float()
        active = ctypes.c_uint32()
        if not self.active(step_us, interval, offset_us, 1_000_000, *arrays,
                           endpoints, coefficients, ctypes.byref(seconds),
                           ctypes.byref(active)):
            return None
        return ([Fraction(f32(coefficients[axis * 6 + k])) for k in range(6)]
                for axis in range(3))

    def measure(self, step_us, offset_us, state):
        arrays = [(ctypes.c_float * 3)(*values) for values in state]
        position = (ctypes.c_float * 3)()
        seconds = ctypes.c_float()
        if not self.sample(step_us, 0, offset_us, 1_000_000, *arrays,
                           position, ctypes.byref(seconds)):
            return None
        return [Fraction(f32(position[axis])) for axis in range(3)], Fraction(seconds.value)


def exact_at(state, h, s):
    return exact_coefficients(state[0][0], state[1][0], state[2][0],
                              state[3][0], state[4][0], state[5][0], h)


def exact_vector(state, h, s):
    out = []
    for axis in range(3):
        coefficients = exact_coefficients(state[0][axis], state[1][axis],
                                          state[2][axis], state[3][axis],
                                          state[4][axis], state[5][axis], h)
        total = Fraction(0)
        power = Fraction(1)
        for coefficient in coefficients:
            total += coefficient * power
            power *= s
        out.append(total)
    return out


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_continuous_error_budget.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    limits = read_product_envelope(probes.library)
    rnd = random.Random(SEED)
    worst_ratio = 0.0
    worst_case = None
    checked = 0
    candidates = 0
    legal_cases = 0
    rejected = {'speed': 0, 'acceleration': 0, 'jerk': 0,
                'speed_norm': 0, 'acceleration_norm': 0}
    component_max = [0.0, 0.0, 0.0, 0.0]
    failures = []
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        for _ in range(CASES_PER_STEP):
            p0 = [f32(rnd.uniform(-P_MAX, P_MAX)) for _ in range(3)]
            v0 = [f32(rnd.uniform(-limits.velocity[axis], limits.velocity[axis]))
                  for axis in range(3)]
            a0 = [f32(rnd.uniform(-limits.acceleration[axis],
                                  limits.acceleration[axis])) for axis in range(3)]
            jerk = [f32(rnd.uniform(-limits.jerk[axis], limits.jerk[axis]))
                    for axis in range(3)]
            p1 = [f32(p0[i] + v0[i] * h + 0.5 * a0[i] * h * h
                      + jerk[i] * h ** 3 / 6.0) for i in range(3)]
            v1 = [f32(v0[i] + a0[i] * h + 0.5 * jerk[i] * h * h) for i in range(3)]
            a1 = [f32(a0[i] + jerk[i] * h) for i in range(3)]
            state = (p0, v0, a0, p1, v1, a1)
            candidates += 1
            legal, reason = product_legal(state, h, step_us, limits)
            if not legal:
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            legal_cases += 1
            c_series = probes.coefficients(step_us, 0, 0, state)
            if c_series is None:
                failures.append("coefficient API failed")
                continue
            c_series = [list(series) for series in c_series]
            components = budget(*state, h, step_us, c_series)
            total = sum(components)
            for index, value in enumerate(components):
                component_max[index] = max(component_max[index], float(value))
            for offset_us in (0, step_us // 3, step_us // 2, 2 * step_us // 3, step_us):
                measured = probes.measure(step_us, offset_us, state)
                if measured is None:
                    failures.append(f"probe failed {step_us}/{offset_us}")
                    continue
                position, seconds = measured
                local = ctypes.c_float()
                if not probes.local(step_us, 0, offset_us, 1_000_000,
                                    ctypes.byref(local)):
                    failures.append(f"local-time hook failed {step_us}/{offset_us}")
                    continue
                s_measured = Fraction(local.value)
                s_exact = Fraction(offset_us, 1000000)
                reference = exact_vector(state, h, s_exact)
                error = sqrt(sum(float((position[axis] - reference[axis]) ** 2)
                                 for axis in range(3)))
                ratio = error / float(total) if total else 0.0
                checked += 1
                if ratio > 1.0:
                    failures.append(
                        f"{step_us}/{offset_us}: measured {error:.3e} > budget {float(total):.3e}")
                if ratio > worst_ratio:
                    worst_ratio = ratio
                    worst_case = {"step_us": step_us, "offset_us": offset_us,
                                  "measured_m": error, "budget_m": float(total)}
    report = {
        "test": "S06-105",
        "seed": f"0x{SEED:X}",
        "steps": list(STEPS),
        "cases_per_step": CASES_PER_STEP,
        "candidate_cases": candidates,
        "legal_cases": legal_cases,
        "stress_cases": candidates - legal_cases,
        "jerk_rejected_cases": rejected["jerk"],
        "speed_rejected_cases": rejected["speed"] + rejected["speed_norm"],
        "acceleration_rejected_cases": (rejected["acceleration"] +
                                        rejected["acceleration_norm"]),
        "product_envelope": {
            "max_linear_speed_mps": limits.speed,
            "max_linear_accel_mps2": limits.accel,
            "max_velocity_ned_mps": limits.velocity,
            "max_acceleration_ned_mps2": limits.acceleration,
            "max_jerk_ned_mps3": limits.jerk,
            "valid_mask": limits.valid_mask,
            "config_version": limits.config_version,
            "parameter_hash": limits.parameter_hash,
            "profile_hash": limits.profile_hash,
        },
        "checked_samples": checked,
        "component_max_m": {
            "coefficient": component_max[0],
            "horner": component_max[1],
            "subnormal": component_max[2],
            "time": component_max[3],
        },
        "worst_ratio": worst_ratio,
        "worst_case": worst_case,
        "library_sha256": hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest(),
        "failures": failures[:10],
    }
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print("M5_S06_105_ERROR_BUDGET_PASS "
          "(E_coefficient + E_horner + E_subnormal + E_time, 3-D norm, real C)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
