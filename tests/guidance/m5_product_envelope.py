"""Shared product-envelope reader and exact legality oracle for M5-06 tests.

The limits come from the real C product builder through the test API; no Python
copy of max speed/acceleration/jerk is maintained here.
"""
import ctypes
import struct
from fractions import Fraction
from math import comb

U = Fraction(1, 1 << 24)
GAMMA2 = (2 * U) / (1 - 2 * U)


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


class ProductEnvelope:
    def __init__(self, speed, accel, velocity, acceleration, jerk,
                 valid_mask, envelope_id, config_version, parameter_hash,
                 profile_hash, profile_id):
        self.speed = speed
        self.accel = accel
        self.velocity = velocity
        self.acceleration = acceleration
        self.jerk = jerk
        self.valid_mask = valid_mask
        self.envelope_id = envelope_id
        self.config_version = config_version
        self.parameter_hash = parameter_hash
        self.profile_hash = profile_hash
        self.profile_id = profile_id


def read_product_envelope(library):
    function = library.m5_trajectory_safety_test_product_envelope
    pointer = ctypes.POINTER(ctypes.c_float)
    function.argtypes = [pointer, pointer, pointer, pointer, pointer,
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32),
                         ctypes.POINTER(ctypes.c_uint32)]
    function.restype = ctypes.c_uint8
    speed = ctypes.c_float()
    accel = ctypes.c_float()
    velocity = (ctypes.c_float * 3)()
    acceleration = (ctypes.c_float * 3)()
    jerk = (ctypes.c_float * 3)()
    valid_mask = ctypes.c_uint32()
    envelope_id = ctypes.c_uint32()
    config_version = ctypes.c_uint32()
    parameter_hash = ctypes.c_uint32()
    profile_hash = ctypes.c_uint32()
    profile_id = ctypes.c_uint32()
    if not function(ctypes.byref(speed), ctypes.byref(accel), velocity,
                    acceleration, jerk, ctypes.byref(valid_mask),
                    ctypes.byref(envelope_id), ctypes.byref(config_version),
                    ctypes.byref(parameter_hash), ctypes.byref(profile_hash),
                    ctypes.byref(profile_id)):
        raise SystemExit('product envelope API failed')
    if valid_mask.value != 0x7F or envelope_id.value == 0:
        raise SystemExit('product envelope identity/mask invalid')
    if config_version.value == 0 or parameter_hash.value == 0 or profile_hash.value == 0:
        raise SystemExit('product config identity invalid')
    return ProductEnvelope(
        float(speed.value), float(accel.value),
        [float(velocity[i]) for i in range(3)],
        [float(acceleration[i]) for i in range(3)],
        [float(jerk[i]) for i in range(3)],
        int(valid_mask.value), int(envelope_id.value),
        int(config_version.value), int(parameter_hash.value),
        int(profile_hash.value), int(profile_id.value))


def exact_coefficients(p0, v0, a0, p1, v1, a1, h):
    p0, v0, a0 = Fraction(f32(p0)), Fraction(f32(v0)), Fraction(f32(a0))
    p1, v1, a1 = Fraction(f32(p1)), Fraction(f32(v1)), Fraction(f32(a1))
    h = Fraction(f32(h))
    d = (p1 - p0) - h * v0 - Fraction(1, 2) * h * h * a0
    dv = (v1 - v0) - h * a0
    da = a1 - a0
    return [p0, v0, Fraction(1, 2) * a0,
            10 * d / h ** 3 - 4 * dv / h ** 2 + da / (2 * h),
            -15 * d / h ** 4 + 7 * dv / h ** 3 - da / h ** 2,
            6 * d / h ** 5 - 3 * dv / h ** 4 + da / (2 * h ** 3)]


def bernstein_bound(coefficients, s_max):
    degree = len(coefficients) - 1
    scaled = [coefficients[k] * s_max ** k for k in range(degree + 1)]
    control = []
    for j in range(degree + 1):
        value = Fraction(0)
        for k in range(j + 1):
            value += comb(j, k) * scaled[k] / comb(degree, k)
        control.append(value)
    return max(abs(value) for value in control)


def polynomial_max(coefficients, s0, s1):
    """max |sum c_k s^k| on [s0,s1], including the interior stationary point."""
    values = []
    for s in (s0, s1):
        total = Fraction(0)
        power = Fraction(1)
        for coefficient in coefficients:
            total += coefficient * power
            power *= s
        values.append(abs(total))
    if len(coefficients) == 3 and coefficients[2] != 0:
        s_star = -coefficients[1] / (2 * coefficients[2])
        if s0 <= s_star <= s1:
            total = coefficients[0] + coefficients[1] * s_star + \
                coefficients[2] * s_star * s_star
            values.append(abs(total))
    return max(values)


def envelope_legal(state, h, step_us, limits):
    """Per-axis + 3-D speed/accel + continuous extended-interval jerk."""
    t_max = Fraction(step_us, 1000000)
    s_eval = max(t_max, Fraction(f32(h))) + GAMMA2 * t_max
    speed_sq = Fraction(0)
    accel_sq = Fraction(0)
    for axis in range(3):
        exact = exact_coefficients(state[0][axis], state[1][axis],
                                   state[2][axis], state[3][axis],
                                   state[4][axis], state[5][axis], h)
        velocity = [Fraction(k) * exact[k] for k in range(1, 6)]
        acceleration = [Fraction(k * (k - 1)) * exact[k] for k in range(2, 6)]
        jerk = [Fraction(k * (k - 1) * (k - 2)) * exact[k] for k in range(3, 6)]
        v_bound = bernstein_bound(velocity, s_eval)
        a_bound = bernstein_bound(acceleration, s_eval)
        j_bound = polynomial_max(jerk, Fraction(0), s_eval)
        if v_bound > Fraction(f32(limits.velocity[axis])):
            return False, 'speed'
        if a_bound > Fraction(f32(limits.acceleration[axis])):
            return False, 'acceleration'
        if j_bound > Fraction(f32(limits.jerk[axis])):
            return False, 'jerk'
        speed_sq += v_bound * v_bound
        accel_sq += a_bound * a_bound
    if speed_sq > Fraction(f32(limits.speed)) ** 2:
        return False, 'speed_norm'
    if accel_sq > Fraction(f32(limits.accel)) ** 2:
        return False, 'acceleration_norm'
    return True, ''
