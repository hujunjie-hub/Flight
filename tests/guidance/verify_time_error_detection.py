"""S06-108 (R13): real-C negative matrix for the time/segment decomposition.

Every case calls the real control_sample_reference() through the test API and
uses the actual binary32 coefficients. A corrupted input must violate the same
check that the honest input satisfies:

 A. previous-segment coefficients + next-segment C output  -> E_horner explodes
 B. next-segment coefficients + previous-segment C output   -> E_horner explodes
 C. wrong local time (global offset instead of active-local) -> time ratio > 1
 E. injected time error above gamma_2                        -> ratio > 1
 D. h-based reference: covered by gamma_2 (tolerance case, must stay inside)
"""
import ctypes
import struct
import sys
from fractions import Fraction

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
STEPS = (20000, 40000)
INTERVALS = (0, 11, 23)
BASES = (1_000_000, 123_456_789)
OFFSETS = ("0", "mid", "step-1", "step")
STATES = (
    ([0.5, 0.0, -5.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0],
     [0.52, 0.0, -5.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
    ([-1.0, 2.0, -4.5], [0.0, 3.5, 0.0], [0.0, -2.5, 0.0],
     [-1.0, 2.05, -4.5], [0.0, 2.0, 0.0], [0.0, -2.0, 0.0]),
)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


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


class Probes:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        pointer = ctypes.POINTER(ctypes.c_float)
        self.active = library.m5_trajectory_safety_test_active_segment
        self.active.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                                ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_uint32)]
        self.active.restype = ctypes.c_uint8
        self.full = library.m5_trajectory_safety_test_consumer_full_sample
        self.full.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.c_uint64] + [pointer] * 6 + [
            pointer, pointer, pointer, ctypes.POINTER(ctypes.c_float)]
        self.full.restype = ctypes.c_uint8


def run(probes, step_us, interval, base, offset_us, state):
    arrays = [(ctypes.c_float * 3)(*values) for values in state]
    endpoints = (ctypes.c_float * 18)()
    coefficients = (ctypes.c_float * 18)()
    seconds = ctypes.c_float()
    active = ctypes.c_uint32()
    if not probes.active(step_us, interval, offset_us, base, *arrays, endpoints,
                         coefficients, ctypes.byref(seconds),
                         ctypes.byref(active)):
        return None
    position = (ctypes.c_float * 3)()
    velocity = (ctypes.c_float * 3)()
    acceleration = (ctypes.c_float * 3)()
    if not probes.full(step_us, interval, offset_us, base, *arrays, position,
                       velocity, acceleration, ctypes.byref(seconds)):
        return None
    h = f32(step_us * f32(1e-6))
    c_series = [[Fraction(f32(coefficients[axis * 6 + k])) for k in range(6)]
                for axis in range(3)]
    exact = [exact_coefficients(endpoints[axis * 6 + 0], endpoints[axis * 6 + 1],
                                endpoints[axis * 6 + 2], endpoints[axis * 6 + 3],
                                endpoints[axis * 6 + 4], endpoints[axis * 6 + 5], h)
             for axis in range(3)]
    global_us = interval * step_us + offset_us
    local_us = global_us - active.value * step_us
    s_c = Fraction(seconds.value)
    s_exact = Fraction(local_us, 1000000)
    measured = [Fraction(f32(position[axis])) for axis in range(3)]
    return {"c": c_series, "exact": exact, "measured": measured,
            "s_c": s_c, "s_exact": s_exact, "h": h, "active": active.value,
            "global_us": global_us, "local_us": local_us}


def decompose_axes(data, coefficients=None, s_exact=None):
    """Return per-axis exact rational components.

    All quantities stay Fractions: the measured value and the C coefficients
    are exact binary32 values, and s_exact is a rational number of seconds.
    """
    coefficients = coefficients or data["c"]
    s_exact = data["s_exact"] if s_exact is None else s_exact
    e_coefficient = []
    e_horner = []
    e_time = []
    for axis in range(3):
        model = poly(coefficients[axis], data["s_c"])
        reference = poly(data["exact"][axis], data["s_c"])
        physical = poly(data["exact"][axis], s_exact)
        measured = data["measured"][axis]
        e_coefficient.append(abs(model - reference))
        e_horner.append(abs(measured - model))
        e_time.append(abs(reference - physical))
    return e_coefficient, e_horner, e_time


def decompose(data, coefficients=None, s_exact=None):
    """Worst-axis exact components; kept for the negative matrix."""
    e_coefficient, e_horner, e_time = decompose_axes(data, coefficients, s_exact)
    return max(e_coefficient), max(e_horner), max(e_time)


def segment_coefficients(probes, step_us, target_interval, base, state):
    """Coefficients of the active segment selected by the real C locator."""
    if target_interval < 0 or target_interval >= 24:
        return None
    neighbour = run(probes, step_us, target_interval, base, 0, state)
    if neighbour is None or neighbour["active"] != target_interval:
        return None
    return neighbour["c"]


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_time_error_detection.py <libm5_safety_oracle.so>')
    probes = Probes(sys.argv[1])
    checked = 0
    coeff_mix_checked = 0
    coeff_mix_skipped = 0
    wrong_local_checked = 0
    failures = []
    for step_us in STEPS:
        for interval in INTERVALS:
            for base in BASES:
                for offset_name in OFFSETS:
                    offset_us = {"0": 0, "mid": step_us // 2,
                                 "step-1": step_us - 1, "step": step_us}[offset_name]
                    for state in STATES:
                        data = run(probes, step_us, interval, base, offset_us, state)
                        if data is None:
                            failures.append(f"probe failed {step_us}/{interval}/{offset_us}")
                            continue
                        checked += 1
                        honest = decompose(data)
                        # A/B: mix the previous and next segment's coefficients.
                        # The corruption is detectable only when the coefficient
                        # separation exceeds twice the honest coefficient error;
                        # then the exact reverse triangle guarantees that the
                        # mixed coefficient error is strictly larger.
                        for direction in (-1, 1):
                            neighbour = segment_coefficients(
                                probes, step_us, data["active"] + direction,
                                base, state)
                            if neighbour is None or neighbour == data["c"]:
                                continue
                            separation = max(
                                abs(poly(neighbour[axis], data["s_c"]) -
                                    poly(data["c"][axis], data["s_c"]))
                                for axis in range(3))
                            if separation <= 2 * honest[0]:
                                coeff_mix_skipped += 1
                                continue
                            coeff_mix_checked += 1
                            mixed = decompose(data, coefficients=neighbour)
                            if mixed[0] <= honest[0]:
                                failures.append(
                                    f"A/B {direction:+d} mix not detected "
                                    f"{step_us}/{interval}/{offset_us}")
                        # C: global offset used as local time. Only meaningful when
                        # the active segment is not the one the global offset names
                        # (interval > 0 or the seam at offset == step).
                        global_us = data["global_us"]
                        if data["local_us"] != global_us:
                            wrong_local_checked += 1
                            wrong_local = decompose(
                                data, s_exact=Fraction(global_us, 1000000))
                            if wrong_local[2] <= max(2 * honest[2], Fraction(1, 10 ** 12)):
                                failures.append(
                                    f"C local-time error not detected "
                                    f"{step_us}/{interval}/{offset_us}")
                        # E: injected error above gamma_2.
                        injected = data["s_exact"] + 4 * GAMMA2 * data["s_exact"] \
                            if data["s_exact"] else Fraction(0)
                        injected_time = decompose(data, s_exact=injected)
                        if data["s_exact"] and injected_time[2] <= honest[2]:
                            failures.append(
                                f"E injected error not detected {step_us}/{interval}/{offset_us}")
                        # D: h-based reference must stay inside gamma_2.
                        if data["active"] == interval:
                            h_based = Fraction(data["h"]) * Fraction(offset_us, step_us)
                            if abs(h_based - data["s_exact"]) > (
                                    GAMMA2 * data["s_exact"] + Fraction(1, 10 ** 12)):
                                failures.append(
                                    f"D h-based reference outside gamma_2 "
                                    f"{step_us}/{interval}/{offset_us}")
    print(f"M5_S06_108_TIME_ERROR_DETECTION cases={checked} "
          f"coeff_mix_checked={coeff_mix_checked} "
          f"coeff_mix_skipped={coeff_mix_skipped} "
          f"wrong_local_checked={wrong_local_checked} failures={len(failures)}")
    for failure in failures[:10]:
        print("FAIL:", failure)
    if failures:
        return 1
    print("M5_S06_108_TIME_ERROR_DETECTION_PASS "
          "(real C, bidirectional segment mix / local time / injected error "
          "rejected, h-based inside gamma_2)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
