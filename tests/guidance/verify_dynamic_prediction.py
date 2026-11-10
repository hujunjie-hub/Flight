#!/usr/bin/env python3
"""Fraction oracle for dynamic-track time and predicted-position enclosures."""
from __future__ import annotations

import ctypes
from fractions import Fraction
import random
import struct
import sys


def float_from_bits(bits: int) -> float:
    return struct.unpack("!f", struct.pack("!I", bits))[0]


def exact_position(position: tuple[float, ...],
                   velocity: tuple[float, ...],
                   local_mid_offset_us: float,
                   sample_offset_us: int) -> tuple[Fraction, ...]:
    dt = (Fraction.from_float(local_mid_offset_us) -
          Fraction(sample_offset_us, 1)) / 1_000_000
    return tuple(
        Fraction.from_float(p) + Fraction.from_float(v) * dt
        for p, v in zip(position, velocity)
    )


def verify(lib_path: str) -> None:
    lib = ctypes.CDLL(lib_path)
    fn = lib.m5_trajectory_safety_test_predict_dynamic_position
    float3 = ctypes.c_float * 3
    fn.argtypes = (
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_double,
        ctypes.c_int64,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    )
    fn.restype = ctypes.c_uint8

    tiny = float_from_bits(0x00000001)
    max_finite = float_from_bits(0x7F7FFFFF)
    cases = [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10000.0, 0),
        ((1.0, 0.0, 0.0), (float_from_bits(0x3F800001), 0.0, 0.0),
         10000.25, -500000),
        ((1.0, tiny, 0.0), (float_from_bits(0x2F800000), 0.0, 0.0),
         20000.5, 20000),
        ((tiny, -tiny, tiny), (tiny, tiny, -tiny), 0.5, -1),
        ((max_finite, 0.0, 0.0), (0.0, 0.0, 0.0), 480000.0, 0),
        ((float_from_bits(0x00800000), tiny, 0.0),
         (float_from_bits(0x7E800000), -float_from_bits(0x7E800000), 0.0),
         479999.75, -500000),
        ((-1.0, 2.0, -3.0), (0.125, -0.25, 0.5), 12345.625, 5000),
    ]

    rng = random.Random(0x6D350684)
    for _ in range(10000):
        position = []
        velocity = []
        for _axis in range(3):
            sign_p = rng.getrandbits(1)
            sign_v = rng.getrandbits(1)
            exp_p = rng.randrange(0, 201)
            exp_v = rng.randrange(0, 201)
            mant_p = rng.getrandbits(23)
            mant_v = rng.getrandbits(23)
            position.append(float_from_bits(
                (sign_p << 31) | (exp_p << 23) | mant_p))
            velocity.append(float_from_bits(
                (sign_v << 31) | (exp_v << 23) | mant_v))
        local_mid = float(rng.randrange(0, 480001)) + rng.random()
        sample_offset = rng.randrange(-500000, 5001)
        cases.append((tuple(position), tuple(velocity),
                      local_mid, sample_offset))

    for index, (position, velocity, local_mid, sample_offset) in enumerate(cases):
        p_in, v_in = float3(*position), float3(*velocity)
        p_out = float3()
        error = ctypes.c_float()
        ok = fn(p_in, v_in, local_mid, sample_offset,
                p_out, ctypes.byref(error))
        if not ok:
            raise AssertionError(f"case {index}: valid finite case rejected")
        if not (error.value >= 0.0 and error.value != float("inf")):
            raise AssertionError(
                f"case {index}: invalid position error {error.value!r}"
            )

        exact = exact_position(position, velocity, local_mid, sample_offset)
        error2 = sum(
            ((Fraction.from_float(float(actual)) - expected) ** 2
             for actual, expected in zip(p_out, exact)),
            Fraction(0),
        )
        bound = Fraction.from_float(float(error.value))
        if error2 > bound * bound:
            raise AssertionError(
                f"case {index}: position error exceeds bound; "
                f"position={position!r}, velocity={velocity!r}, "
                f"local_mid={local_mid!r}, sample_offset={sample_offset}, "
                f"output={tuple(p_out)!r}, bound={error.value!r}"
            )

    # A prediction that cannot be represented as a finite float must fail closed.
    p_in, v_in = float3(max_finite, 0.0, 0.0), float3(max_finite, 0.0, 0.0)
    p_out, error = float3(), ctypes.c_float()
    if fn(p_in, v_in, 480000.0, -500000, p_out, ctypes.byref(error)):
        raise AssertionError("overflowing predicted position was accepted")

    print(
        "M5_S06_084_FRACTION_PREDICTION_ORACLE_PASS "
        f"cases={len(cases)} seed=0x6D350684"
    )


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(
            "usage: verify_dynamic_prediction.py <libm5_safety_oracle.so>"
        )
    verify(sys.argv[1])
