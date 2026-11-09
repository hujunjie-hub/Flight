#!/usr/bin/env python3
"""Fly one open-loop hop in FMT-SIM and record what the simulated plant does.

The flight controller is not involved: this drives the simulator with the same
MAVLink frames the firmware emits (`HEARTBEAT` + `HIL_ACTUATOR_CONTROLS`, one
normalized 0..1 value per motor), so it verifies the *simulator half* of the
HIL link: that FMT-SIM accepts our actuator frame, applies it to the plant, and
reports the motion back through `HIL_SENSOR`/`HIL_GPS`.

Profile (all timings in seconds, 50 Hz commands):

    idle 1.0 | ramp 2.0 to THROTTLE | hold HOLD | cut 0.5 | idle 2.0

PASS criteria, printed at the end:

    * HIL_SENSOR keeps arriving at >= 50 Hz during the whole run;
    * pressure altitude rises by at least --min-climb metres while thrust is on;
    * altitude comes back down after the cut.

What this does NOT prove: the firmware's estimator/controller/guard chain. That
needs the board on the HIL UART via the bridge in `middleware/communication/sim`.

Usage:
    python tools/fmt_sim_fly.py                   # 0.65 throttle, 6 s hold
    python tools/fmt_sim_fly.py --throttle 0.6 --hold 4 --csv out.csv
Exit codes: 0 PASS, 2 no HIL stream, 3 flight criteria not met.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests" / "simulation" / "golden"))

from generate_golden import CRC_EXTRA, frame  # noqa: E402

MAGIC = 0xFD
HEADER = 10
ARMED_MODE = 0x80


def crc_byte(crc: int, byte: int) -> int:
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def heartbeat() -> bytes:
    return frame(0, struct.pack("<IBBBBB", 0, 2, 0, 0, 3, 3))


def actuator(now_us: int, throttle: float, mode: int) -> bytes:
    values = [throttle] * 4 + [0.0] * 12
    payload = struct.pack("<QQ16f", now_us, 0, *values) + struct.pack("<B", mode)
    return frame(93, payload)


def parse(stream: bytearray):
    while len(stream) >= HEADER + 2:
        if stream[0] != MAGIC:
            del stream[0]
            continue
        length = stream[1]
        total = HEADER + length + 2
        if total > 280:
            del stream[0]
            continue
        if len(stream) < total:
            return
        message_id = stream[7] | (stream[8] << 8) | (stream[9] << 16)
        extra = CRC_EXTRA.get(message_id)
        if extra is None:
            del stream[0]
            continue
        crc = 0xFFFF
        for byte in stream[1:HEADER + length]:
            crc = crc_byte(crc, byte)
        crc = crc_byte(crc, extra)
        expected = stream[HEADER + length] | (stream[HEADER + length + 1] << 8)
        payload = bytes(stream[HEADER:HEADER + length])
        del stream[:total]
        if crc == expected:
            yield message_id, payload


def sensor_values(payload: bytes) -> tuple[float, float, float, float]:
    """(pressure_alt_m, az_mps2, gyro_magnitude, temperature_c)."""
    padded = payload + b"\x00" * (64 - len(payload))
    accel = struct.unpack_from("<3f", padded, 8)
    gyro = struct.unpack_from("<3f", padded, 20)
    pressure_alt = struct.unpack_from("<f", padded, 52)[0]
    temperature = struct.unpack_from("<f", padded, 56)[0]
    gyro_mag = (gyro[0] ** 2 + gyro[1] ** 2 + gyro[2] ** 2) ** 0.5
    return pressure_alt, accel[2], gyro_mag, temperature


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Fly one open-loop hop in FMT-SIM")
    parser.add_argument("--peer", default="127.0.0.1:14553")
    parser.add_argument("--throttle", type=float, default=0.65,
                        help="normalized motor command during the hold, 0..1")
    parser.add_argument("--ramp", type=float, default=2.0)
    parser.add_argument("--hold", type=float, default=6.0)
    parser.add_argument("--min-climb", type=float, default=2.0,
                        help="metres of pressure altitude that count as a climb")
    parser.add_argument("--csv", default="build/validation/sim/flight.csv")
    args = parser.parse_args(argv)

    host, _, port = args.peer.rpartition(":")
    peer = (host, int(port))
    sock = None
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.05)

    # 1. wait for the simulator to answer at all
    print(f"waiting for FMT-SIM on {peer} ...")
    deadline = time.monotonic() + 10.0
    stream = bytearray()
    while time.monotonic() < deadline:
        sock.sendto(heartbeat() + actuator(0, 0.0, 0), peer)
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        stream.extend(data)
        if any(True for _ in parse(stream)):
            print("simulator is streaming")
            break
    else:
        print("no HIL stream: start the SIH vehicle in FMT-SIM first", file=sys.stderr)
        return 2

    # 2. fly the profile, sampling at 50 Hz
    start = time.monotonic()
    idle, ramp, hold, cut, settle = 1.0, args.ramp, args.hold, 0.5, 2.0
    total = idle + ramp + hold + cut + settle
    next_cmd = start
    next_heartbeat = start
    rows: list[tuple[float, float, float, float, float, float]] = []
    sensor_hz_window: list[float] = []
    stream.clear()
    last_sensor = start

    while True:
        now = time.monotonic()
        t = now - start
        if t >= total:
            break
        if t < idle:
            throttle, mode = 0.0, 0
        elif t < idle + ramp:
            throttle = args.throttle * (t - idle) / ramp
            mode = ARMED_MODE
        elif t < idle + ramp + hold:
            throttle, mode = args.throttle, ARMED_MODE
        elif t < idle + ramp + hold + cut:
            throttle, mode = 0.0, ARMED_MODE
        else:
            throttle, mode = 0.0, 0

        if now >= next_cmd:
            sock.sendto(actuator(int(t * 1e6), throttle, mode), peer)
            next_cmd += 0.02
        if now >= next_heartbeat:
            sock.sendto(heartbeat(), peer)
            next_heartbeat += 1.0

        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        stream.extend(data)
        for message_id, payload in parse(stream):
            if message_id != 107:
                continue
            press_alt, az, gyro_mag, temp = sensor_values(payload)
            rows.append((t, throttle, press_alt, az, gyro_mag, temp))
            if last_sensor:
                sensor_hz_window.append(now - last_sensor)
            last_sensor = now

    # 3. analyse
    out = ROOT / args.csv
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        handle.write("t_s,throttle,pressure_alt_m,accel_z_mps2,gyro_mag_radps,temp_c\n")
        for row in rows:
            handle.write(",".join(f"{value:.4f}" for value in row) + "\n")

    if not rows:
        print("no HIL_SENSOR frames during the flight", file=sys.stderr)
        return 2

    base = [r[2] for r in rows if r[0] < idle]
    during = [r[2] for r in rows if idle + ramp <= r[0] < idle + ramp + hold]
    after = [r[2] for r in rows if r[0] > idle + ramp + hold + cut]
    if not base or not during:
        print("not enough samples to judge the flight", file=sys.stderr)
        return 3

    base_alt = sum(base) / len(base)
    peak_alt = max(during)
    end_alt = sum(after) / len(after) if after else peak_alt
    az_peak = max((r[3] for r in rows), key=abs)
    gyro_peak = max(r[4] for r in rows)
    rate_hz = 1.0 / (sum(sensor_hz_window) / len(sensor_hz_window)) if sensor_hz_window else 0.0

    print(f"samples {len(rows)} ({rate_hz:.0f} Hz HIL_SENSOR), csv {out}")
    print(f"altitude: base {base_alt:.2f} m, peak {peak_alt:.2f} m, after cut {end_alt:.2f} m")
    print(f"peak accel_z {az_peak:+.2f} m/s2, peak |gyro| {gyro_peak:.2f} rad/s, "
          f"max throttle {max(r[1] for r in rows):.2f}")

    climbed = peak_alt - base_alt
    descended = peak_alt - end_alt
    ok = climbed >= args.min_climb and descended > 0.2 and rate_hz >= 50.0
    print(f"climb {climbed:+.2f} m, descent after cut {descended:+.2f} m -> "
          f"{'PASS' if ok else 'FAIL'}")
    if not ok:
        print("  criteria: climb >= %.1f m, descent > 0.2 m, >= 50 Hz HIL_SENSOR"
              % args.min_climb, file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
