#!/usr/bin/env python3
"""Exact rational oracle for the M5-06 binary32 norm bounds.

The production helpers return binary32 bounds. Every finite binary32 input is
exactly representable as Python binary64, and Fraction.from_float reconstructs
that value exactly. Squaring and summing as Fractions avoids using sqrt/libm
as the test oracle: compare squared bounds directly with the exact squared norm.
"""
from __future__ import annotations

import ctypes
from fractions import Fraction
import random
import struct
import sys


def float_from_bits(bits: int) -> float:
    return struct.unpack("!f", struct.pack("!I", bits))[0]


def double_from_bits(bits: int) -> float:
    return struct.unpack("!d", struct.pack("!Q", bits))[0]


def exact_square(values: tuple[float, float, float]) -> Fraction:
    terms = (Fraction.from_float(float(v)) for v in values)
    return sum((v * v for v in terms), Fraction(0))


def verify(lib_path: str) -> None:
    lib = ctypes.CDLL(lib_path)
    upper_fn = lib.m5_trajectory_safety_test_norm_upper_bound
    upper_fn.argtypes = (ctypes.c_float, ctypes.c_float, ctypes.c_float)
    upper_fn.restype = ctypes.c_float
    lower_fn = lib.m5_trajectory_safety_test_norm_lower_bound
    lower_fn.argtypes = (ctypes.c_double, ctypes.c_double, ctypes.c_double)
    lower_fn.restype = ctypes.c_float

    tiny = float_from_bits(0x2F800000)  # 2^-80
    min_subnormal = float_from_bits(0x00000001)
    max_finite = float_from_bits(0x7F7FFFFF)
    cases = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, tiny, 0.0),  # 1 + 2^-160 rounds back to 1 in binary64
        (min_subnormal, min_subnormal, 0.0),
        (max_finite, 0.0, 0.0),
        (max_finite, max_finite, 0.0),
        (float_from_bits(0x00800000), min_subnormal, -min_subnormal),
    ]

    rng = random.Random(0x6D350680)
    # Exercise arbitrary finite binary32 encodings, including signs,
    # subnormals, widely separated exponents, and values near FLT_MAX.
    for _ in range(10000):
        vector = []
        for _axis in range(3):
            bits = rng.getrandbits(32)
            if ((bits >> 23) & 0xFF) == 0xFF:
                bits &= ~(0xFF << 23)
            vector.append(float_from_bits(bits))
        cases.append(tuple(vector))

    for index, vector in enumerate(cases):
        exact = exact_square(vector)
        upper = float(upper_fn(*vector))
        lower = float(lower_fn(*(float(v) for v in vector)))
        if upper < 0.0:
            raise AssertionError(f"case {index}: negative upper bound {upper!r}")
        if lower < 0.0:
            raise AssertionError(f"case {index}: negative lower bound {lower!r}")
        if upper != float("inf") and Fraction.from_float(upper) ** 2 < exact:
            raise AssertionError(
                f"case {index}: upper bound underestimates; vector={vector!r}, "
                f"upper={upper!r}"
            )
        if lower == float("inf") or Fraction.from_float(lower) ** 2 > exact:
            raise AssertionError(
                f"case {index}: lower bound overestimates; vector={vector!r}, "
                f"lower={lower!r}"
            )

    # The lower-bound helper accepts binary64 components in production.
    # Exercise its full finite input domain as well; binary32-only vectors
    # above do not cover values created by the validator's interval math.
    double_cases = [
        (0.0, 0.0, 0.0),
        (1.0, 2.0**-80, 0.0),
        (double_from_bits(0x0000000000000001), 0.0, 0.0),
        (double_from_bits(0x0010000000000000), 0.0, 0.0),
        (sys.float_info.max, 0.0, 0.0),
        (sys.float_info.max, sys.float_info.max, 0.0),
        (1.0, double_from_bits(0x0000000000000001), 0.0),
    ]
    double_rng = random.Random(0x6D350680)
    for _ in range(10000):
        vector = []
        for _axis in range(3):
            bits = double_rng.getrandbits(64)
            if ((bits >> 52) & 0x7FF) == 0x7FF:
                bits &= ~(0x7FF << 52)
            vector.append(double_from_bits(bits))
        double_cases.append(tuple(vector))

    for index, vector in enumerate(double_cases):
        exact = exact_square(vector)
        lower = float(lower_fn(*vector))
        if lower < 0.0:
            raise AssertionError(
                f"binary64 case {index}: negative lower bound {lower!r}"
            )
        if lower == float("inf") or Fraction.from_float(lower) ** 2 > exact:
            raise AssertionError(
                f"binary64 case {index}: lower bound overestimates; "
                f"vector={vector!r}, lower={lower!r}"
            )

    print(
        f"M5_S06_080_EXACT_FRACTION_ORACLE_PASS "
        f"binary32_upper_cases={len(cases)} "
        f"binary64_lower_cases={len(double_cases)} seed=0x6D350680"
    )


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: verify_norm_bounds.py <libm5_safety_oracle.so>")
    verify(sys.argv[1])
