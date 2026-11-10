"""S06-099: exhaustive time-quantisation check for both preview step constants.

The consumer computes

    h   = RN(step_us   * 1e-6f)
    s_c = RN(offset_us * 1e-6f)

with integer microsecond counts that are exact in binary32. The validator uses

    s_exact = h * (offset_us / step_us)

as the mathematical reference time and claims |s_c - s_exact| <= gamma_2*s_max
with gamma_2 = 2u/(1-2u), s_max = h*u1 (u1 = 1 for a whole segment).

This script checks the claim exhaustively for every integer offset of both
product step constants, using exact rational arithmetic for the reference and
binary32 rounding for the consumer, and also checks that s_c never leaves the
enlarged evaluation interval [0, s_max + gamma_2*s_max]. It does not reuse the
production 2u approximation: the only shared input is the definition of the
consumer conversion, which is re-implemented from the C source.
"""
import struct
from fractions import Fraction

STEPS = (20000, 40000)          # PRODUCT_PREVIEW_STEP_US, PRODUCT_ROUTE_PREVIEW_STEP_US
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def rn32(q):
    """Round a rational to binary32, round-to-nearest even."""
    return Fraction(f32(float(q)))


def consumer_seconds(microseconds):
    """s = RN(microseconds * RN(1e-6)) in binary32."""
    micro = Fraction(microseconds)          # exact: microseconds < 2^24
    c = Fraction(f32(1e-6))
    return rn32(micro * c)


def main():
    worst_ratio = 0.0
    worst_step = None
    worst_offset = None
    outside = 0
    checked = 0

    for step_us in STEPS:
        h = consumer_seconds(step_us)       # same construction as the consumer
        s_max = h                            # u1 = 1
        delta_s = GAMMA2 * s_max
        s_eval_max = s_max + delta_s
        for offset_us in range(step_us + 1):
            s_c = consumer_seconds(offset_us)
            s_exact = h * Fraction(offset_us, step_us)
            error = abs(s_c - s_exact)
            ratio = float(error / s_max)
            checked += 1
            if ratio > worst_ratio:
                worst_ratio = ratio
                worst_step = step_us
                worst_offset = offset_us
            if s_c > s_eval_max:
                outside += 1

    gamma2_float = float(GAMMA2)
    print(f"M5_S06_099_TIME_QUANTIZATION_REPORT steps={STEPS} checked_offsets={checked} "
          f"gamma2={gamma2_float:.6e} max_ratio={worst_ratio:.6e} "
          f"worst_step={worst_step} worst_offset={worst_offset} "
          f"outside_eval_interval={outside}")
    failures = []
    if worst_ratio > gamma2_float:
        failures.append(f"max |s_c-s_exact|/s_max = {worst_ratio:.6e} > gamma_2")
    if outside:
        failures.append(f"{outside} consumer times exceed s_max + gamma_2*s_max")
    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1
    print("M5_S06_099_TIME_QUANTIZATION_PASS "
          "(exhaustive integer microseconds, exact rational reference)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
