"""S06-099: exhaustive time-quantisation check, exact rational comparisons only.

Consumer:
    h   = RN(step_us   * 1e-6f)
    s_c = RN(offset_us * 1e-6f)
Validator reference:
    s_exact = h * (offset_us / step_us)
Claim:
    |s_c - s_exact| <= gamma_2 * s_exact,   gamma_2 = 2u/(1-2u)
    max(0, s_exact - gamma_2*s_max) <= s_c <= s_max + gamma_2*s_max
with s_max = h (u1 = 1). Every comparison is done on exact Fractions; no Python
float participates in the verdict.
"""
import struct
from fractions import Fraction

STEPS = (20000, 40000)          # PRODUCT_PREVIEW_STEP_US, PRODUCT_ROUTE_PREVIEW_STEP_US
U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def rn32(q):
    return Fraction(f32(float(q)))


def consumer_seconds(microseconds):
    return rn32(Fraction(microseconds) * Fraction(f32(1e-6)))


def main():
    checked = 0
    failures = []
    worst = Fraction(0)
    worst_at = None
    model_worst = Fraction(0)
    model_worst_at = None
    model_checked = 0
    outside = 0

    for step_us in STEPS:
        # Physical time reference: the microsecond count is exact, so
        # step_us/1e6 is the true segment duration. h is only the binary32 value
        # the consumer computes and must not be used as the reference.
        h = consumer_seconds(step_us)
        s_max = Fraction(step_us, 1000000)
        delta_s = GAMMA2 * s_max
        s_eval_max = max(s_max, Fraction(h)) + delta_s
        for offset_us in range(step_us + 1):
            s_c = consumer_seconds(offset_us)
            s_exact = Fraction(offset_us, 1000000)
            error = abs(s_c - s_exact)
            checked += 1
            if error > GAMMA2 * s_exact:
                failures.append(f"step={step_us} offset={offset_us}: "
                                f"|s_c-s_exact| > gamma_2*s_exact")
            if s_c < max(Fraction(0), s_exact - delta_s):
                failures.append(f"step={step_us} offset={offset_us}: s_c below the physical-time interval")
            if s_c > s_eval_max:
                outside += 1
                failures.append(f"step={step_us} offset={offset_us}: s_c > physical-time s_eval_max")

            # Exact production proof domain: h is the binary32 segment duration,
            # while the ideal local parameter is h*(offset/step). This is the
            # quantity bounded by gamma_2*h*u1 in build_curve().
            s_model = Fraction(h) * Fraction(offset_us, step_us)
            model_error = abs(s_c - s_model)
            model_bound = GAMMA2 * s_model
            model_checked += 1
            if model_error > model_bound:
                failures.append(f"step={step_us} offset={offset_us}: "
                                "|s_c-h*(offset/step)| > gamma_2*h*(offset/step)")
            model_eval_max = s_model + model_bound
            if s_c > model_eval_max:
                failures.append(f"step={step_us} offset={offset_us}: "
                                "consumer time exceeds production s_eval_max")
            model_ratio = model_error / model_bound if model_bound else Fraction(0)
            if model_ratio > model_worst:
                model_worst = model_ratio
                model_worst_at = (step_us, offset_us)

            ratio = error / s_max if s_max else Fraction(0)
            if ratio > worst:
                worst = ratio
                worst_at = (step_us, offset_us)

    print(f"M5_S06_099_TIME_QUANTIZATION_REPORT steps={STEPS} checked_offsets={checked} "
          f"gamma2={float(GAMMA2):.6e} physical_max_ratio={float(worst):.6e} "
          f"physical_worst_step={worst_at[0] if worst_at else -1} "
          f"physical_worst_offset={worst_at[1] if worst_at else -1} "
          f"production_model_checked={model_checked} "
          f"production_model_max_ratio={float(model_worst):.6e} "
          f"model_worst_step={model_worst_at[0] if model_worst_at else -1} "
          f"model_worst_offset={model_worst_at[1] if model_worst_at else -1} "
          f"outside={outside} failures={len(failures)}")
    if failures:
        for failure in failures[:5]:
            print("FAIL:", failure)
        return 1
    if worst > GAMMA2:
        print(f"FAIL: max ratio {float(worst):.6e} exceeds gamma_2")
        return 1
    print("M5_S06_099_TIME_QUANTIZATION_PASS "
          "(physical-time and exact production h*offset/step bounds; "
          "60,002 integer offsets, exact Fraction comparisons)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
