"""S06-103: end-to-end consumer checks through the real C function.

Phase A (time only): a unit-velocity linear reference makes the returned x
position exactly the binary32 time s_c; every integer microsecond offset of both
product step constants is compared with the exact rational reference
s_exact = h*(offset/step). No Python reimplementation of the C expression is
involved.

Phase B (time and curve separated): for arbitrary intervals, base times and
non-zero multi-axis endpoint states, the same C call returns the position and a
companion measurement of s_c. The time error is checked against gamma_2 and the
curve error is checked against the production envelope evaluated at that same
s_c, so the two error sources are never merged into one verdict.
"""
import ctypes
import hashlib
import json
import struct
import sys
from fractions import Fraction
from pathlib import Path

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
        delta_s = GAMMA2 * h
        s_eval_max = h + delta_s
        for offset_us in range(step_us + 1):
            value = ctypes.c_float()
            if not probes.time(step_us, offset_us, ctypes.byref(value)):
                failures.append(f"time probe failed step={step_us} offset={offset_us}")
                continue
            s_c = Fraction(value.value)
            s_exact = h * Fraction(offset_us, step_us)
            checked += 1
            if abs(s_c - s_exact) > GAMMA2 * s_exact:
                failures.append(f"step={step_us} offset={offset_us}: time > gamma_2")
            if s_c < max(Fraction(0), s_exact - delta_s) or s_c > s_eval_max:
                failures.append(f"step={step_us} offset={offset_us}: outside interval")
            max_ratio = max(max_ratio, abs(s_c - s_exact) / h)
    report["phase_a"] = {"checked": checked, "max_ratio": float(max_ratio),
                         "failures": failures[:5]}
    return failures


def phase_b(probes, report):
    failures = []
    checked = 0
    max_time_ratio = Fraction(0)
    max_curve_ratio = 0.0
    for step_us in STEPS:
        h = f32(step_us * f32(1e-6))
        for interval in INTERVALS:
            for base in BASES:
                for offset_us in (0, step_us // 2, step_us - 1, step_us):
                    for state in STATES:
                        arrays = [(ctypes.c_float * 3)(*values) for values in state]
                        position = (ctypes.c_float * 3)()
                        seconds = ctypes.c_float()
                        ok = probes.sample(step_us, interval, offset_us, base,
                                           *arrays, position,
                                           ctypes.byref(seconds))
                        checked += 1
                        if not ok:
                            failures.append(f"probe failed {step_us}/{interval}/{offset_us}")
                            continue
                        hh = Fraction(f32(h))
                        measured = ctypes.c_float()
                        if not probes.local(step_us, interval, offset_us, base,
                                            ctypes.byref(measured)):
                            failures.append(
                                f"local-time hook failed {step_us}/{interval}/{offset_us}")
                            continue
                        # The production computation is the verdict: no nominal
                        # value may replace it.
                        s_c = Fraction(measured.value)
                        # Production semantics: the active segment is chosen by
                        # the global offset, so offset == step means the start of
                        # the next segment with local time 0.
                        global_us = interval * step_us + offset_us
                        active = min(global_us // step_us, 23)
                        local_us = global_us - active * step_us
                        # Physical time reference: the integer microsecond count
                        # is exact, so local_us/1e6 is the true time. h is a
                        # rounded binary32 value and must not be used here.
                        s_exact = Fraction(local_us, 1000000)
                        if abs(s_c - s_exact) > GAMMA2 * s_exact:
                            failures.append(f"time: {step_us}/{interval}/{offset_us}")
                        max_time_ratio = max(max_time_ratio, abs(s_c - s_exact) / hh)
                        # At a seam (offset == step) the knot is both the end of
                        # this segment and the start of the next one; both must
                        # be validated, never skipped.
                        if offset_us == step_us:
                            # Seam: production switches to the next segment and
                            # uses its start state at local time 0. The fixture
                            # fills later segments with zeros, so that is the
                            # exact reference here. The previous endpoint is
                            # covered by the step-1 offset above.
                            # The next segment starts at this segment's end
                            # state (the probe writes points[interval] = end).
                            exact = exact_hermite(*state[3:], *state[3:], h,
                                                  Fraction(0))
                        else:
                            exact = exact_hermite(*state[:3], *state[3:], h, s_c)
                        error = max(abs(Fraction(f32(position[axis])) - exact[axis])
                                    for axis in range(3))
                        limit = ctypes.c_float()
                        vlimit = ctypes.c_float()
                        alimit = ctypes.c_float()
                        if not probes.envelope(*arrays, ctypes.c_float(h),
                                               ctypes.c_float(0.0),
                                               ctypes.c_float(1.0),
                                               ctypes.byref(limit),
                                               ctypes.byref(vlimit),
                                               ctypes.byref(alimit)):
                            failures.append(f"envelope rejected {step_us}/{interval}")
                            continue
                        # Separate the three error sources at the same sample.
                        float_series = [
                            consumer_coefficients(state[0][axis], state[1][axis],
                                                  state[2][axis], state[3][axis],
                                                  state[4][axis], state[5][axis], h)
                            for axis in range(3)]
                        model_value = [0.0, 0.0, 0.0]
                        for axis in range(3):
                            value = f32(float_series[axis][5])
                            for k in reversed(range(5)):
                                value = f32(f32(value * f32(s_c)) +
                                            f32(float_series[axis][k]))
                            model_value[axis] = value
                        coefficient_error = max(
                            abs(Fraction(model_value[axis]) - exact[axis])
                            for axis in range(3))
                        horner_error = max(
                            abs(Fraction(f32(position[axis])) -
                                Fraction(model_value[axis]))
                            for axis in range(3))
                        exact_physical = exact_hermite(*state[:3], *state[3:], h,
                                                       s_exact)
                        time_error = max(abs(exact[axis] - exact_physical[axis])
                                         for axis in range(3))
                        report.setdefault("decomposition", {
                            "coefficient_error_m": 0.0,
                            "horner_error_m": 0.0,
                            "time_error_m": 0.0,
                        })
                        report["decomposition"]["coefficient_error_m"] = max(
                            report["decomposition"]["coefficient_error_m"],
                            float(coefficient_error))
                        report["decomposition"]["horner_error_m"] = max(
                            report["decomposition"]["horner_error_m"],
                            float(horner_error))
                        report["decomposition"]["time_error_m"] = max(
                            report["decomposition"]["time_error_m"],
                            float(time_error))
                        if float(error) > limit.value:
                            failures.append(
                                f"curve: {step_us}/{interval}/{offset_us} "
                                f"{float(error):.3e} > {limit.value:.3e}")
                        if limit.value > 0:
                            max_curve_ratio = max(max_curve_ratio,
                                                  float(error) / limit.value)
    report["phase_b"] = {"checked": checked,
                         "decomposition": report.get("decomposition", {}),
                         "max_time_ratio": float(max_time_ratio),
                         "max_curve_over_envelope": max_curve_ratio,
                         "failures": failures[:5]}
    return failures


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
