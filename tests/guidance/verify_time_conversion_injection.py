"""S06-113: C-side time-conversion error injection.

The production local-time conversion and the C gamma_2 predicate are called
through one test API. Python does not reproduce either formula.

mode 0 = normal, 1 = global time used as local, 2 = +4*gamma_2, 3 = wrong
segment duration. Mode 1 is skipped only when global and local time are equal.
"""
import ctypes
import json
import sys

STEPS = (20000, 40000)
INTERVALS = (0, 11, 23)


class Probe:
    def __init__(self, path):
        library = ctypes.CDLL(path)
        self.call = library.m5_trajectory_safety_test_time_conversion
        self.call.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
                              ctypes.c_uint32, ctypes.c_uint8,
                              ctypes.POINTER(ctypes.c_float),
                              ctypes.POINTER(ctypes.c_uint8)]
        self.call.restype = ctypes.c_uint8


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_time_conversion_injection.py <lib.so>')
    probe = Probe(sys.argv[1])
    checked = 0
    failures = []
    for step_us in STEPS:
        offsets = (0, step_us // 2, step_us - 1, step_us)
        for interval in INTERVALS:
            for offset_us in offsets:
                for mode, expected in ((0, 1), (2, 0), (3, 0)):
                    if mode == 3 and offset_us == 0:
                        continue
                    seconds = ctypes.c_float()
                    within = ctypes.c_uint8()
                    if not probe.call(step_us, interval, offset_us, mode,
                                      ctypes.byref(seconds), ctypes.byref(within)):
                        failures.append(
                            f'API failed step={step_us} interval={interval} '
                            f'offset={offset_us} mode={mode}')
                        continue
                    checked += 1
                    if within.value != expected:
                        failures.append(
                            f'mode={mode} expected={expected} got={within.value} '
                            f'step={step_us} interval={interval} offset={offset_us}')
                global_us = interval * step_us + offset_us
                if global_us != offset_us:
                    seconds = ctypes.c_float()
                    within = ctypes.c_uint8()
                    if not probe.call(step_us, interval, offset_us, 1,
                                      ctypes.byref(seconds), ctypes.byref(within)):
                        failures.append(
                            f'API failed mode=1 step={step_us} interval={interval} '
                            f'offset={offset_us}')
                    else:
                        checked += 1
                        if within.value != 0:
                            failures.append(
                                f'mode=1 expected=0 got={within.value} '
                                f'step={step_us} interval={interval} offset={offset_us}')
    report = {'test': 'S06-113', 'seed': 'deterministic',
              'steps': list(STEPS), 'intervals': list(INTERVALS),
              'checked': checked, 'failures': failures[:10]}
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print('M5_S06_113_TIME_CONVERSION_INJECTION_PASS '
          '(production C conversion + C gamma_2 predicate)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
