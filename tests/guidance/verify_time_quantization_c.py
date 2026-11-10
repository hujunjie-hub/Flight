"""S06-103: end-to-end consumer checks through the real C function.

Phase A (time only): a unit-velocity linear reference makes the returned x
position exactly the binary32 time s_c; every integer microsecond offset of both
product step constants is compared with the exact rational reference
s_exact = h*(offset/step). No Python reimplementation of the C expression is
involved.

Phase B (time and curve separated): for arbitrary intervals, base times and
non-zero multi-axis endpoint states, the same C call returns the position and a
companion measurement of s_c. The time error is checked against gamma_2 and the
measured total position error is checked against the continuous component
budget from S06-105 (metres against metres), so a time ratio is never reported
with the wrong dimension.
"""
import ctypes
import hashlib
import json
import struct
import sys
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_continuous_error_budget as budget_model

STEPS = (20000, 40000)
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
INTERVALS = (0, 11, 23)
BASES = (1_000_000, 123_456_789)
STATES = (
    # start_p, start_v, start_a, end_p, end_v, end_a  (product envelope)
    ([0.5, 0.0, -5.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0],
     [0.52, 0.0, -5.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ([-1.0, 2.0, -4.5], [0.0, 3.5, 0.0], [0.0, -2.5, 0.0],
     [-1.0, 2.05, -4.5], [0.0, 2.0, 0.0], [0.0, -2.0, 0.0]),
    ([1.5, -1.0, -6.0], [2.0, 1.0, 0.5], [1.0, 0.5, 0.0],
     [1.55, -0.99, -6.0], [2.0, 1.0, 0.5], [0.5, 0.2, 0.0]),
)



def consumer_coefficients(p0, v0, a0, p1, v1, a1, h):
    """binary32 coefficient construction (mirrors control_reference.c)."""
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


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def rn32(q):
    return Fraction(f32(float(q)))


def exact_hermite(start_p, start_v, start_a, end_p, end_v, end_a, h, s):
    out = []
    for axis in range(3):
        p0 = Fraction(f32(start_p[axis]))
        v0 = Fraction(f32(start_v[axis]))
        a0 = Fraction(f32(start_a[axis]))
        p1 = Fraction(f32(end_p[axis]))
        v1 = Fraction(f32(end_v[axis]))
        a1 = Fraction(f32(end_a[axis]))
        hh = Fraction(f32(h))
        d = (p1 - p0) - hh * v0 - Fraction(1, 2) * hh * hh * a0
        dv = (v1 - v0) - hh * a0
        da = a1 - a0
        c = [p0, v0, Fraction(1, 2) * a0,
             10 * d / hh ** 3 - 4 * dv / hh ** 2 + da / (2 * hh),
             -15 * d / hh ** 4 + 7 * dv / hh ** 3 - da / hh ** 2,
             6 * d / hh ** 5 - 3 * dv / hh ** 4 + da / (2 * hh ** 3)]
        total = Fraction(0)
        power = Fraction(1)
        for coefficient in c:
            total += coefficient * power
            power *= s
        out.append(total)
    return out


class Probes:
    def __init__(self, library_path):
        self.lib = ctypes.CDLL(library_path)
        self.time = self.lib.m5_trajectory_safety_test_consumer_time
        self.time.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.POINTER(ctypes.c_float)]
        self.time.restype = ctypes.c_uint8
        pointer = ctypes.POINTER(ctypes.c_float)
        self.full = self.lib.m5_trajectory_safety_test_consumer_full_sample
        self.full.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, pointer, ctypes.POINTER(ctypes.c_float)]
        self.full.restype = ctypes.c_uint8
        self.active = self.lib.m5_trajectory_safety_test_active_segment
        self.active.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_uint32)]
        self.active.restype = ctypes.c_uint8
        self.local = self.lib.m5_trajectory_safety_test_consumer_local_seconds
        self.local.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                               ctypes.c_uint64, ctypes.POINTER(ctypes.c_float)]
        self.local.restype = ctypes.c_uint8
        self.sample = self.lib.m5_trajectory_safety_test_consumer_interval_sample
        self.sample.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, ctypes.POINTER(ctypes.c_float)]
        self.sample.restype = ctypes.c_uint8
        envelope = self.lib.m5_trajectory_safety_test_curve_envelope
        envelope.argtypes = [pointer] * 6 + [ctypes.c_float, ctypes.c_float,
                                             ctypes.c_float, pointer, pointer,
                                             pointer]
        envelope.restype = ctypes.c_uint8
        self.envelope = envelope


def phase_a(probes, report):
    checked = 0
    max_ratio = Fraction(0)
    failures = []
    for step_us in STEPS:
        h = Fraction(f32(f32(step_us) * f32(1e-6)))
        t_max = Fraction(step_us, 1000000)
        delta_s = GAMMA2 * t_max
        s_eval_max = max(t_max, h) + delta_s
        for offset_us in range(step_us + 1):
            value = ctypes.c_float()
            if not probes.time(step_us, offset_us, ctypes.byref(value)):
                failures.append(f"time probe failed step={step_us} offset={offset_us}")
                continue
            s_c = Fraction(value.value)
            s_exact = Fraction(offset_us, 1000000)
            checked += 1
            if abs(s_c - s_exact) > GAMMA2 * s_exact:
                failures.append(f"step={step_us} offset={offset_us}: time > gamma_2")
            if s_c < max(Fraction(0), s_exact - delta_s) or s_c > s_eval_max:
                failures.append(f"step={step_us} offset={offset_us}: outside interval")
            max_ratio = max(max_ratio, abs(s_c - s_exact) / t_max)
    report["phase_a"] = {"checked": checked, "max_ratio": float(max_ratio),
                         "failures": failures[:5]}
    return failures


def phase_b(probes, report):
    """Active-segment decomposition: C coefficients, C output and exact reference
    all come from the segment the production locator selected."""
    failures = []
    checked = 0
    max_time_ratio = Fraction(0)
    max_total_error = Fraction(0)
    max_total_ratio = Fraction(0)
    max_budget_components = [Fraction(0), Fraction(0), Fraction(0), Fraction(0)]
    decomposition = {"coefficient": 0.0, "horner": 0.0, "time": 0.0}
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        t_max = Fraction(step_us, 1000000)
        for interval in INTERVALS:
            for base in BASES:
                for offset_us in (0, step_us // 2, step_us - 1, step_us):
                    for state in STATES:
                        arrays = [(ctypes.c_float * 3)(*values) for values in state]
                        endpoints = (ctypes.c_float * 18)()
                        coefficients = (ctypes.c_float * 18)()
                        seconds = ctypes.c_float()
                        active = ctypes.c_uint32()
                        if not probes.active(step_us, interval, offset_us, base,
                                             *arrays, endpoints, coefficients,
                                             ctypes.byref(seconds),
                                             ctypes.byref(active)):
                            failures.append(
                                f"active-segment API failed {step_us}/{interval}/{offset_us}")
                            continue
                        position = (ctypes.c_float * 3)()
                        velocity = (ctypes.c_float * 3)()
                        acceleration = (ctypes.c_float * 3)()
                        if not probes.full(step_us, interval, offset_us, base,
                                           *arrays, position, velocity,
                                           acceleration, ctypes.byref(seconds)):
                            failures.append(
                                f"full sample failed {step_us}/{interval}/{offset_us}")
                            continue
                        s_c = Fraction(seconds.value)
                        # Physical time is local to the ACTIVE segment: at a seam
                        # the production locator switches to the next segment, so
                        # the elapsed microseconds must be measured from that
                        # segment's start.
                        global_us = interval * step_us + offset_us
                        active_us = active.value * step_us
                        s_exact = Fraction(global_us - active_us, 1000000)
                        # Exact coefficients of the SAME active segment.
                        exact = [exact_coefficients(
                            endpoints[axis * 6 + 0], endpoints[axis * 6 + 1],
                            endpoints[axis * 6 + 2], endpoints[axis * 6 + 3],
                            endpoints[axis * 6 + 4], endpoints[axis * 6 + 5], h)
                            for axis in range(3)]
                        c_series = [[Fraction(f32(coefficients[axis * 6 + k]))
                                     for k in range(6)] for axis in range(3)]
                        budget_components = budget_model.budget(
                            *state, h, step_us, c_series)
                        budget_total = sum(budget_components)
                        for index, value in enumerate(budget_components):
                            max_budget_components[index] = max(
                                max_budget_components[index], value)
                        checked += 1
                        if abs(s_c - s_exact) > GAMMA2 * s_exact:
                            failures.append(
                                f"time {step_us}/{interval}/{offset_us}: > gamma_2")
                        max_time_ratio = max(max_time_ratio,
                                             abs(s_c - s_exact) / t_max)
                        for axis in range(3):
                            model = poly_value(c_series[axis], s_c)
                            reference = poly_value(exact[axis], s_c)
                            physical = poly_value(exact[axis], s_exact)
                            measured = Fraction(f32(position[axis]))
                            e_coefficient = abs(model - reference)
                            e_horner = abs(measured - model)
                            e_time = abs(reference - physical)
                            total = abs(measured - physical)
                            if total > e_coefficient + e_horner + e_time:
                                failures.append(
                                    f"triangle {step_us}/{interval}/{offset_us} axis{axis}")
                            if total > budget_total:
                                failures.append(
                                    f"total>budget {step_us}/{interval}/{offset_us} "
                                    f"axis{axis}")
                            decomposition["coefficient"] = max(
                                decomposition["coefficient"], float(e_coefficient))
                            decomposition["horner"] = max(
                                decomposition["horner"], float(e_horner))
                            decomposition["time"] = max(
                                decomposition["time"], float(e_time))
                            max_total_error = max(max_total_error, total)
                            if budget_total > 0:
                                max_total_ratio = max(
                                    max_total_ratio, total / budget_total)
    report["phase_b"] = {"checked": checked,
                         "decomposition": decomposition,
                         "max_time_ratio": float(max_time_ratio),
                         "max_time_error_ratio": float(max_time_ratio) / float(GAMMA2) if GAMMA2 else 0.0,
                         "max_total_error_m": float(max_total_error),
                         "max_total_error_over_budget": float(max_total_ratio),
                         "budget_components_m": [float(value) for value in
                                                 max_budget_components],
                         "failures": failures[:10]}
    return failures



def exact_coefficients(p0, v0, a0, p1, v1, a1, h):
    """Exact rational quintic coefficients for the same endpoints."""
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


def poly_value(coefficients, s):
    total = Fraction(0)
    power = Fraction(1)
    for coefficient in coefficients:
        total += coefficient * power
        power *= s
    return total


def negative_self_test():
    """A time error above gamma_2 must be detected by the same comparison."""
    h = Fraction(f32(f32(20000) * f32(1e-6)))
    s_exact = h * Fraction(1, 2)
    bad = s_exact + GAMMA2 * s_exact * 4          # four times the bound
    good = s_exact + GAMMA2 * s_exact * Fraction(1, 2)
    if abs(bad - s_exact) <= GAMMA2 * s_exact:
        return False
    if abs(good - s_exact) > GAMMA2 * s_exact:
        return False
    return True


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_time_quantization_c.py <libm5_safety_oracle.so>')
    library_path = sys.argv[1]
    probes = Probes(library_path)
    report = {
        "test": "S06-103",
        "seed": "deterministic (exhaustive offsets + fixed state table)",
        "library": library_path,
        "library_sha256": hashlib.sha256(Path(library_path).read_bytes()).hexdigest(),
        "steps": list(STEPS),
        "gamma_2": float(GAMMA2),
    }
    failures = phase_a(probes, report) + phase_b(probes, report)
    report["negative_self_test"] = ("PASS" if negative_self_test() else "FAIL")
    if report["negative_self_test"] != "PASS":
        failures.append("negative self-test: gamma_2 check did not reject an excess error")
    report["failures"] = failures[:10]
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print("M5_S06_103_C_CONSUMER_TIME_PASS "
          "(real control_sample_reference; time and curve errors checked separately)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
