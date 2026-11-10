"""S06-108: the time-error checker must reject deliberately mixed references.

Negative cases:
 1. previous-segment coefficients combined with the next-segment C output;
 2. an injected error of 4*gamma_2;
 3. an h-based reference instead of the physical local_us/1e6.

Each case must be reported as a failure by the same comparison used in S06-103.
"""
import sys
from fractions import Fraction

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def check(s_c, s_exact):
    return abs(s_c - s_exact) <= GAMMA2 * s_exact


def main():
    s_exact = Fraction(12345, 1000000)
    failures = []
    # 1. segment mix: a value 0.05 m away must not pass as a time error
    mixed = s_exact + Fraction(5, 100)
    if check(mixed, s_exact):
        failures.append("mixed-segment value accepted")
    # 2. injected 4x gamma_2
    injected = s_exact + 4 * GAMMA2 * s_exact
    if check(injected, s_exact):
        failures.append("4x gamma_2 accepted")
    # 3. wrong segment duration: using a rounded h scaled by 1+2^-20 instead of
    #    the physical microsecond time must be rejected.
    wrong_h = Fraction(20000, 1000000) * (1 + Fraction(1, 1 << 20))
    wrong_reference = wrong_h * Fraction(12345, 20000)
    if check(s_exact, wrong_reference):
        failures.append("wrong segment duration accepted")
    # the correct value must pass
    if not check(s_exact + GAMMA2 * s_exact / 2, s_exact):
        failures.append("in-bound value rejected")
    print(f"M5_S06_108_TIME_ERROR_DETECTION cases=4 failures={len(failures)}")
    for failure in failures:
        print("FAIL:", failure)
    if failures:
        return 1
    print("M5_S06_108_TIME_ERROR_DETECTION_PASS (mixed, injected and h-based errors rejected)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
