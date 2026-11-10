"""S06-103: end-to-end time check through the real C consumer.

`m5_trajectory_safety_test_consumer_time()` builds a unit-velocity linear
reference and calls the real `control_sample_reference()`. For that reference
the polynomial is c0 = 0, c1 = 1, c2..c5 = 0, so the returned x position is
exactly the binary32 time s_c the consumer used - no Python reimplementation of
the C expression participates.

Every integer microsecond offset of both product step constants is checked
against the exact rational reference s_exact = h*(offset/step) and the gamma_2
envelope.
"""
import ctypes
import hashlib
import json
import struct
import sys
from fractions import Fraction
from pathlib import Path

SEED = "deterministic"          # exhaustive, no random input
STEPS = (20000, 40000)
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'SConstruct').is_file())


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: verify_time_quantization_c.py <libm5_safety_oracle.so>')
    library_path = sys.argv[1]
    library = ctypes.CDLL(library_path)
    probe = library.m5_trajectory_safety_test_consumer_time
    probe.argtypes = [ctypes.c_uint32, ctypes.c_uint32,
                      ctypes.POINTER(ctypes.c_float)]
    probe.restype = ctypes.c_uint8

    checked = 0
    failures = []
    worst = Fraction(0)
    worst_at = None
    for step_us in STEPS:
        h = Fraction(f32(f32(step_us) * f32(1e-6)))
        delta_s = GAMMA2 * h
        s_eval_max = h + delta_s
        for offset_us in range(step_us + 1):
            value = ctypes.c_float()
            ok = probe(step_us, offset_us, ctypes.byref(value))
            if not ok:
                failures.append(f"probe failed at step={step_us} offset={offset_us}")
                continue
            s_c = Fraction(value.value)
            s_exact = h * Fraction(offset_us, step_us)
            checked += 1
            if abs(s_c - s_exact) > GAMMA2 * s_exact:
                failures.append(f"step={step_us} offset={offset_us}: error > gamma_2*s_exact")
            if s_c < max(Fraction(0), s_exact - delta_s):
                failures.append(f"step={step_us} offset={offset_us}: s_c below interval")
            if s_c > s_eval_max:
                failures.append(f"step={step_us} offset={offset_us}: s_c > s_eval_max")
            ratio = abs(s_c - s_exact) / h
            if ratio > worst:
                worst = ratio
                worst_at = (step_us, offset_us)

    digest = hashlib.sha256(Path(library_path).read_bytes()).hexdigest()
    report = {
        "test": "S06-103",
        "seed": SEED,
        "library": library_path,
        "library_sha256": digest,
        "steps": list(STEPS),
        "checked_offsets": checked,
        "max_error_over_s_max": float(worst),
        "worst_step_us": worst_at[0] if worst_at else None,
        "worst_offset_us": worst_at[1] if worst_at else None,
        "gamma_2": float(GAMMA2),
        "failures": failures[:10],
    }
    print(json.dumps(report, indent=2))
    if failures:
        return 1
    print("M5_S06_103_C_CONSUMER_TIME_PASS "
          "(real control_sample_reference, exact rational reference)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
