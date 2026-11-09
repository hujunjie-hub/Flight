#!/usr/bin/env python3
"""Independent MAVLink v2 golden frames for the Flight HIL link test.

This generator deliberately shares no code with applications/tasks/communication/task_sim.c:
it packs the payload fields itself (struct, little endian) and computes the
X.25 / CRC-16-MCRF4XX checksum with its own table-free loop. The produced
header is committed, so a change to the C codec that the dialect does not
authorize fails the host test instead of silently matching a modified
encoder.

Authoritative constants (generated common dialect, FMT-Firmware baseline):
    HIL_SENSOR               id 107, payload 64, CRC_EXTRA 108
    HIL_GPS                  id 113, payload 36 (+1 ext), CRC_EXTRA 124
    HIL_ACTUATOR_CONTROLS    id  93, payload 81, CRC_EXTRA 47
    HEARTBEAT                id   0, payload  9, CRC_EXTRA 50
    FMT_EXTERNAL_STATE      id 12001, payload 88, CRC_EXTRA 168

MAVLink v2 drops trailing zero payload bytes, so the length written on the
wire is the truncated length; the receiver zero-fills the rest.

Usage:  python tests/simulation/golden/generate_golden.py [--check]
        --check only verifies that the committed header is up to date.
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = 0xFD
CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    93: 47,   # HIL_ACTUATOR_CONTROLS
    107: 108, # HIL_SENSOR
    113: 124, # HIL_GPS
    12001: 168, # FMT_EXTERNAL_STATE
}

SENSOR = dict(
    time_usec=1_234_567,
    accel=(0.25, -0.5, 9.79),
    gyro=(0.01, -0.02, 0.03),
    mag=(0.21, -0.12, 0.43),
    abs_pressure=900.0,
    diff_pressure=0.0,
    pressure_alt=1000.0,
    temperature=25.5,
    fields_updated=0x1FFF,
)

GPS = dict(
    time_usec=2_000_000,
    lat=473977420,      # 47.3977420 deg
    lon=85455940,       # 8.5455940 deg
    alt=488000,         # mm
    eph=120,            # cm
    epv=200,            # cm
    vel=300,            # cm/s
    vn=150, ve=-150, vd=25,
    cog=9000,
    fix_type=3,
    satellites=14,
    id=1,  # non-zero extension keeps the 37-byte form distinct from the 36-byte one
)

ACTUATOR = dict(time_usec=3_000_000, controls=(0.1, 0.2, 0.3, 0.4), mode=0x80)

HEARTBEAT = dict(custom_mode=0, type=2, autopilot=0, base_mode=0, system_status=3,
                 mavlink_version=3)


def crc_byte(crc: int, byte: int) -> int:
    """CRC-16/MCRF4XX, the MAVLink checksum."""
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def frame(message_id: int, payload: bytes, sequence: int = 1) -> bytes:
    body = bytearray(payload)
    while len(body) > 1 and body[-1] == 0:
        body.pop()
    header = bytes([MAGIC, len(body), 0, 0, sequence, 1, 1,
                    message_id & 0xFF, (message_id >> 8) & 0xFF, (message_id >> 16) & 0xFF])
    crc = 0xFFFF
    for byte in header[1:] + bytes(body):
        crc = crc_byte(crc, byte)
    # CRC_EXTRA is a single byte (MAVLink v2 section 5.3).
    crc = crc_byte(crc, CRC_EXTRA[message_id] & 0xFF)
    return header + bytes(body) + struct.pack("<H", crc)


def sensor_payload() -> bytes:
    floats = (*SENSOR["accel"], *SENSOR["gyro"], *SENSOR["mag"], SENSOR["abs_pressure"],
              SENSOR["diff_pressure"], SENSOR["pressure_alt"], SENSOR["temperature"])
    assert len(floats) == 13
    packed = struct.pack("<Q", SENSOR["time_usec"])
    packed += struct.pack("<13f", *floats)
    packed += struct.pack("<I", SENSOR["fields_updated"])
    assert len(packed) == 64, len(packed)
    return packed


def gps_payload(include_extension: bool = True) -> bytes:
    packed = struct.pack("<QiiiHHHhhhHBB", GPS["time_usec"], GPS["lat"], GPS["lon"],
                         GPS["alt"], GPS["eph"], GPS["epv"], GPS["vel"], GPS["vn"],
                         GPS["ve"], GPS["vd"], GPS["cog"], GPS["fix_type"],
                         GPS["satellites"])
    assert len(packed) == 36, len(packed)
    if include_extension:
        packed += struct.pack("<B", GPS["id"])
    return packed


def actuator_payload() -> bytes:
    controls = list(ACTUATOR["controls"]) + [0.0] * 12
    packed = struct.pack("<QQ16f", ACTUATOR["time_usec"], 0, *controls)
    packed += struct.pack("<B", ACTUATOR["mode"])
    assert len(packed) == 81, len(packed)
    return packed


def heartbeat_payload() -> bytes:
    return struct.pack("<IBBBBB", HEARTBEAT["custom_mode"], HEARTBEAT["type"],
                       HEARTBEAT["autopilot"], HEARTBEAT["base_mode"],
                       HEARTBEAT["system_status"], HEARTBEAT["mavlink_version"])


def render() -> str:
    frames = {
        "HIL_SENSOR_FRAME": frame(107, sensor_payload()),
        "HIL_GPS_FRAME": frame(113, gps_payload()),
        "HIL_GPS_TRUNCATED_FRAME": frame(113, gps_payload(include_extension=False)),
        "HIL_ACTUATOR_FRAME": frame(93, actuator_payload()),
        "HEARTBEAT_FRAME": frame(0, heartbeat_payload()),
        "FMT_EXTERNAL_STATE_FRAME": frame(
            12001, struct.pack("<I4f3f3f3f3f3ifI", 0,
                               1.0, 0.0, 0.0, 0.0,
                               0.01, -0.02, 0.03,
                               0.2, -0.4, -9.8,
                               1.5, -1.5, 0.25,
                               10.0, -20.0, -2.0,
                               376278579, -1223895167, 4488, -1.0, 0xFC)),
    }
    corrupted = bytearray(frames["HIL_SENSOR_FRAME"])
    corrupted[-1] ^= 0xFF
    frames["HIL_SENSOR_BAD_CRC_FRAME"] = bytes(corrupted)

    lines = [
        "/* Generated by tests/simulation/golden/generate_golden.py; do not edit.",
        " * Independent MAVLink v2 frames for the HIL link host test. */",
        "#ifndef HIL_GOLDEN_H",
        "#define HIL_GOLDEN_H",
        "",
    ]
    for name, data in frames.items():
        lines.append(f"static const unsigned char {name}[] = {{")
        for offset in range(0, len(data), 12):
            chunk = ", ".join(f"0x{byte:02x}" for byte in data[offset:offset + 12])
            lines.append(f"    {chunk},")
        lines.append("};")
        lines.append("")
    lines.append("#endif /* HIL_GOLDEN_H */")
    lines.append("")
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true",
                        help="fail if the committed golden header is stale")
    args = parser.parse_args(argv)

    target = Path(__file__).with_name("hil_golden.h")
    text = render()
    if args.check:
        current = target.read_text(encoding="utf-8") if target.exists() else ""
        if current != text:
            print(f"stale: {target}", file=sys.stderr)
            return 1
        print(f"up to date: {target}")
        return 0
    target.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {target} ({len(text)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
