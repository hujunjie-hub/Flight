"""S06-113: C-side time-conversion error injection.

The production local-time conversion and the C gamma_2 predicate are called
through one test API. Python does not reproduce either formula.

mode 0 = normal, 1 = global time used as local, 2 = +4*gamma_2, 3 = wrong
segment duration. Mode 1 is skipped only when global and local time are equal.
"""
import ctypes
import json
import sys
from fractions import Fraction

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)

STEPS = (20000, 40000)
INTERVALS = (0, 11, 23)


class Probe:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        self.call = library.m5_trajectory_safety_test_time_conversion
        self.call.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.c_uint32, ctypes.c_uint8,
                              ctypes.POINTER(ctypes.c_float),
                              ctypes.POINTER(ctypes.c_uint8),
                              ctypes.POINTER(ctypes.c_double),
                              ctypes.POINTER(ctypes.c_double)]
        self.call.restype = ctypes.c_uint8


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_time_conversion_injection.py <lib.so>')
    probe = Probe(sys.argv[1])
    checked = 0
    actual_error_checks = 0
    mode2_ratio_min = None
    mode2_ratio_max = None
    failures = []
    for step_us in STEPS:
        offsets = (0, step_us // 2, step_us - 1, step_us)
        for interval in INTERVALS:
            for offset_us in offsets:
                for mode, expected in ((0, 1), (2, 0), (3, 0)):
                    if mode in (2, 3) and offset_us == 0:
                        continue
                    seconds = ctypes.c_float()
                    within = ctypes.c_uint8()
                    true_seconds = ctypes.c_double()
                    error = ctypes.c_double()
                    if not probe.call(step_us, interval, offset_us, mode,
                                      ctypes.byref(seconds), ctypes.byref(within),
                                      ctypes.byref(true_seconds),
                                      ctypes.byref(error)):
                        failures.append(
                            f'API failed step={step_us} interval={interval} '
                            f'offset={offset_us} mode={mode}')
                        continue
                    checked += 1
                    if within.value != expected:
                        failures.append(
                            f'mode={mode} expected={expected} got={within.value} '
                            f'step={step_us} interval={interval} offset={offset_us}')
                    bound = float(GAMMA2 * Fraction(true_seconds.value))
                    if mode == 0 and error.value > bound:
                        failures.append(f'normal error > gamma_2 mode=0')
                    if mode == 2:
                        actual_error_checks += 1
                        ratio = error.value / bound if bound else 0.0
                        mode2_ratio_min = (ratio if mode2_ratio_min is None
                                           else min(mode2_ratio_min, ratio))
                        mode2_ratio_max = (ratio if mode2_ratio_max is None
                                           else max(mode2_ratio_max, ratio))
                        if not (3.0 * bound < error.value < 5.0 * bound):
                            failures.append(
                                f'mode=2 error={error.value} is not ~4*gamma_2 '
                                f'bound={bound}')
                    if mode in (1, 3) and error.value <= bound:
                        failures.append(
                            f'mode={mode} error={error.value} <= gamma_2 {bound}')
                global_us = interval * step_us + offset_us
                if global_us != offset_us:
                    seconds = ctypes.c_float()
                    within = ctypes.c_uint8()
                    true_seconds = ctypes.c_double()
                    error = ctypes.c_double()
                    if not probe.call(step_us, interval, offset_us, 1,
                                      ctypes.byref(seconds), ctypes.byref(within),
                                      ctypes.byref(true_seconds),
                                      ctypes.byref(error)):
                        failures.append(
                            f'API failed mode=1 step={step_us} interval={interval} '
                            f'offset={offset_us}')
                    else:
                        checked += 1
                        if within.value != 0:
                            failures.append(
                                f'mode=1 expected=0 got={within.value} '
                                f'step={step_us} interval={interval} offset={offset_us}')
                        bound = float(GAMMA2 * Fraction(true_seconds.value))
                        if error.value <= bound:
                            failures.append(f'mode=1 error <= gamma_2')
    report = {'test': 'S06-113', 'seed': 'NOT_APPLICABLE',
              'steps': list(STEPS), 'intervals': list(INTERVALS),
              'checked': checked, 'actual_error_checks': actual_error_checks,
              'mode2_error_over_gamma2_min': mode2_ratio_min,
              'mode2_error_over_gamma2_max': mode2_ratio_max,
              'failures': failures[:10]}
    print(json.dumps(report, indent=2))
    print(f"S06_113_TIME_CONVERSION_INJECTION checked={checked} failures={len(failures)}")
    if failures:
        return 1
    print('M5_S06_113_TIME_CONVERSION_INJECTION_PASS '
          '(production C conversion + C gamma_2 predicate)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
