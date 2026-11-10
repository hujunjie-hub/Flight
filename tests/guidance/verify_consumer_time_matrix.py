"""S06-109: positive real-C end-to-end matrix for the consumer decomposition.

For every combination of step (20/40 ms), interval (0/11/23), base timestamp and
offset (0, mid, step-1, step) with non-zero p/v/a endpoints, the honest
decomposition must satisfy the triangle inequality and the gamma_2 time bound
using exact rational arithmetic. The C consumer and its actual binary32
coefficients are the only measured inputs; no epsilon is used.
"""
import ctypes
import struct
import sys
from fractions import Fraction

sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
import verify_time_error_detection as base

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_consumer_time_matrix.py <libm5_safety_oracle.so>')
    probes = base.Probes(sys.argv[1])
    checked = 0
    failures = []
    for step_us in base.STEPS:
        for interval in base.INTERVALS:
            for base_us in base.BASES:
                for offset_name in base.OFFSETS:
                    offset_us = {"0": 0, "mid": step_us // 2,
                                 "step-1": step_us - 1, "step": step_us}[offset_name]
                    for state in base.STATES:
                        data = base.run(probes, step_us, interval, base_us,
                                        offset_us, state)
                        if data is None:
                            failures.append(
                                f"probe failed {step_us}/{interval}/{offset_us}")
                            continue
                        checked += 1
                        e_coefficient, e_horner, e_time = base.decompose_axes(data)
                        for axis in range(3):
                            reference = base.poly(data["exact"][axis], data["s_exact"])
                            total = abs(data["measured"][axis] - reference)
                            bound = (e_coefficient[axis] + e_horner[axis] +
                                     e_time[axis])
                            if total > bound:
                                failures.append(
                                    f"triangle axis{axis} "
                                    f"{step_us}/{interval}/{offset_us}")
                        if abs(data["s_c"] - data["s_exact"]) > GAMMA2 * data["s_exact"]:
                            failures.append(
                                f"gamma_2 {step_us}/{interval}/{offset_us}")
    print(f"M5_S06_109_CONSUMER_TIME_MATRIX cases={checked} failures={len(failures)}")
    for failure in failures[:10]:
        print("FAIL:", failure)
    if failures:
        return 1
    print("M5_S06_109_CONSUMER_TIME_MATRIX_PASS "
          "(real C, 2 steps x 3 intervals x 2 bases x 4 offsets x 2 states)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
