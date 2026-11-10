"""S06-085: independent exact-arithmetic check of the MR-009 envelope.

M5-06 proves the binary64 quintic Hermite; M6 consumes the binary32 Horner
evaluation in control_sample_reference(). The validator now adds a bound for
|consumer - exact| (MR-009). `tests/guidance/test_m5_main.c` S06-086 checks that
bound against the real C consumer; this script is the independent counterpart:

* both paths start from the *same* binary32 endpoint values (P0 item 1);
* the sample time uses the consumer's actual offset_us -> seconds conversion
  (P0 item 2);
* the reference value is computed with exact rational arithmetic (Fraction),
  so no floating-point oracle is trusted (P0 item 3);
* the envelope is read from the production C API through ctypes, not
  re-implemented here (P0 item 6).

A failure means the production envelope does not cover the exact binary32
evaluation error and MR-009 is not fixed.
"""
import ctypes
import struct
import sys
from fractions import Fraction

SEED = 0x6D350685
STEP_US = 20000
SAMPLES = 33
SMOOTH_SEGMENTS = 64
ADVERSARIAL_SEGMENTS = 64


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def f32_bits(value):
    return struct.unpack('I', struct.pack('f', value))[0]


def consumer_sample(p0, v0, a0, p1, v1, a1, h, offset_us):
    """binary32 Horner evaluation mirroring control_reference.c.

    s = (float)(offset - a->offset_us) * 1e-6f; every operation is binary32.
    """
    p0, v0, a0 = f32(p0), f32(v0), f32(a0)
    p1, v1, a1 = f32(p1), f32(v1), f32(a1)
    h = f32(h)
    s = f32(f32(offset_us) * f32(1e-6))

    def op(value):
        return f32(value)

    d = op(op(op(p1 - p0) - op(h * v0)) - op(op(op(0.5 * h) * h) * a0))
    dv = op(op(v1 - v0) - op(h * a0))
    da = op(a1 - a0)
    h2 = op(h * h)
    h3 = op(h2 * h)
    h4 = op(h3 * h)
    h5 = op(h4 * h)
    c0 = p0
    c1 = v0
    c2 = op(0.5 * a0)
    c3 = op(op(op(op(10.0 * d) / h3) - op(op(4.0 * dv) / h2)) + op(da / op(2.0 * h)))
    c4 = op(op(op(op(-15.0 * d) / h4) + op(op(7.0 * dv) / h3)) - op(da / h2))
    c5 = op(op(op(op(6.0 * d) / h5) - op(op(3.0 * dv) / h4)) + op(da / op(2.0 * h3)))

    position = op(op(op(op(op(op(c5 * s) + c4) * s + c3) * s + c2) * s + c1) * s + c0)
    velocity = op(op(op(op(op(5.0 * c5 * s + 4.0 * c4) * s + 3.0 * c3) * s
                      + 2.0 * c2) * s + c1))
    acceleration = op(op(op(op(20.0 * c5 * s + 12.0 * c4) * s + 6.0 * c3) * s
                         + 2.0 * c2))
    return position, velocity, acceleration


def exact_coefficients(p0, v0, a0, p1, v1, a1, h):
    """Exact rational coefficients of the same quintic."""
    p0, v0, a0 = Fraction(f32(p0)), Fraction(f32(v0)), Fraction(f32(a0))
    p1, v1, a1 = Fraction(f32(p1)), Fraction(f32(v1)), Fraction(f32(a1))
    h = Fraction(f32(h))
    d = (p1 - p0) - h * v0 - Fraction(1, 2) * h * h * a0
    dv = (v1 - v0) - h * a0
    da = a1 - a0
    return (
        p0,
        v0,
        Fraction(1, 2) * a0,
        10 * d / h ** 3 - 4 * dv / h ** 2 + da / (2 * h),
        -15 * d / h ** 4 + 7 * dv / h ** 3 - da / h ** 2,
        6 * d / h ** 5 - 3 * dv / h ** 4 + da / (2 * h ** 3),
    )


def exact_sample(coefficients, offset_us):
    """Exact value and derivatives at the true time offset_us / 1e6."""
    s = Fraction(offset_us, 1_000_000)
    position = velocity = acceleration = Fraction(0)
    for index, coefficient in enumerate(coefficients):
        position += coefficient * s ** index
        if index >= 1:
            velocity += index * coefficient * s ** (index - 1)
        if index >= 2:
            acceleration += index * (index - 1) * coefficient * s ** (index - 2)
    return position, velocity, acceleration


class Envelope:
    def __init__(self, library_path):
        self.lib = ctypes.CDLL(library_path)
        function = self.lib.m5_trajectory_safety_test_curve_envelope
        pointer = ctypes.POINTER(ctypes.c_float)
        function.argtypes = [pointer] * 6 + [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float)]
        function.restype = ctypes.c_uint8
        self.function = function

    def __call__(self, p0, v0, a0, p1, v1, a1, h, u0, u1):
        arrays = [(ctypes.c_float * 3)(*values) for values in (p0, v0, a0, p1, v1, a1)]
        position = ctypes.c_float()
        velocity = ctypes.c_float()
        acceleration = ctypes.c_float()
        ok = self.function(*arrays, ctypes.c_float(h), ctypes.c_float(u0),
                           ctypes.c_float(u1), ctypes.byref(position),
                           ctypes.byref(velocity), ctypes.byref(acceleration))
        return ok, position.value, velocity.value, acceleration.value


def smooth_segment(rnd):
    scale = rnd.choice([1.0, 10.0, 100.0, 500.0])
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
    scale = rnd.choice([1.0, 10.0, 100.0, 500.0])
    p0 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    p1 = [f32(rnd.uniform(-scale, scale)) for _ in range(3)]
    v0 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    v1 = [f32(rnd.uniform(-40.0, 40.0)) for _ in range(3)]
    a0 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    a1 = [f32(rnd.uniform(-20.0, 20.0)) for _ in range(3)]
    return p0, v0, a0, p1, v1, a1


def measure(generator, rnd, segments, envelope, h):
    worst = {'position': 0.0, 'velocity': 0.0, 'acceleration': 0.0}
    worst_ratio = {'position': 0.0, 'velocity': 0.0, 'acceleration': 0.0}
    uncovered = 0
    for _ in range(segments):
        segment = generator(rnd)
        ok, position_error, velocity_error, acceleration_error = envelope(
            *segment, h, 0.0, 1.0)
        if not ok:
            raise SystemExit('production envelope rejected a valid segment')
        coefficients = [
            exact_coefficients(*(segment[component][axis] for component in range(6)), h)
            for axis in range(3)
        ]
        for sample in range(SAMPLES):
            offset_us = (sample * STEP_US) // SAMPLES
            for axis in range(3):
                position, velocity, acceleration = consumer_sample(
                    *(value[axis] for value in segment), h, offset_us)
                exact_p, exact_v, exact_a = exact_sample(coefficients[axis], offset_us)
                gaps = {
                    'position': abs(Fraction(position) - exact_p),
                    'velocity': abs(Fraction(velocity) - exact_v),
                    'acceleration': abs(Fraction(acceleration) - exact_a),
                }
                limits = {
                    'position': Fraction(f32(position_error)),
                    'velocity': Fraction(f32(velocity_error)),
                    'acceleration': Fraction(f32(acceleration_error)),
                }
                for kind in gaps:
                    gap = float(gaps[kind])
                    limit = float(limits[kind])
                    worst[kind] = max(worst[kind], gap)
                    if limit > 0.0:
                        worst_ratio[kind] = max(worst_ratio[kind], gap / limit)
                    if gap > limit:
                        uncovered += 1
    return worst, worst_ratio, uncovered


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_curve_eval_gap.py <libm5_safety_oracle.so>')
    envelope = Envelope(sys.argv[1])
    h = f32(STEP_US * f32(1e-6))
    import random
    smooth_rnd = random.Random(SEED)
    adversarial_rnd = random.Random(SEED ^ 0x5A5A5A5A)
    smooth, smooth_ratio, smooth_bad = measure(
        smooth_segment, smooth_rnd, SMOOTH_SEGMENTS, envelope, h)
    adversarial, adversarial_ratio, adversarial_bad = measure(
        adversarial_segment, adversarial_rnd, ADVERSARIAL_SEGMENTS, envelope, h)
    print(f"M5_S06_085_CURVE_EVAL_GAP_REPORT seed=0x{SEED:X} "
          f"smooth_segments={SMOOTH_SEGMENTS} adversarial_segments={ADVERSARIAL_SEGMENTS} "
          f"samples={SAMPLES} "
          f"smooth_max_m={smooth['position']:.6e} "
          f"smooth_vel_max={smooth['velocity']:.6e} "
          f"smooth_acc_max={smooth['acceleration']:.6e} "
          f"adversarial_max_m={adversarial['position']:.6e} "
          f"adversarial_vel_max={adversarial['velocity']:.6e} "
          f"adversarial_acc_max={adversarial['acceleration']:.6e} "
          f"smooth_max_ratio={max(smooth_ratio.values()):.3f} "
          f"adversarial_max_ratio={max(adversarial_ratio.values()):.3f} "
          f"uncovered={smooth_bad + adversarial_bad}")
    failures = []
    if smooth_bad or adversarial_bad:
        failures.append(f"{smooth_bad + adversarial_bad} samples exceed the envelope")
    if max(smooth_ratio.values()) > 1.0 or max(adversarial_ratio.values()) > 1.0:
        failures.append('gap exceeds the production envelope')
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("M5_S06_085_EXACT_ENVELOPE_COVERAGE_PASS "
          "(exact Fraction reference, consumer offset_us timing)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
