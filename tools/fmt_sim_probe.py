#!/usr/bin/env python3
"""Talk to a running FMT-SIM over the real HIL link (no bridge, no FCU needed).

What it does

1. sends one MAVLink v2 `HEARTBEAT` and one `HIL_ACTUATOR_CONTROLS` to the
   simulator's `MavproxyFCS` endpoint (default `127.0.0.1:14553`), which is
   what `Autobound: true` waits for;
2. captures every datagram the simulator sends back for a few seconds;
3. validates each frame with CRC_EXTRA and reports per-message counts, rates
   and one decoded `HIL_SENSOR`/`HIL_GPS` sample;
4. saves the raw capture so the *firmware* decoder can be run on exactly the
   same bytes (`tests/simulation/decode_capture.c`).

This is the simulator half of the bench link. The flight controller half needs
the board on a COM port and the UDP bridge in `middleware/communication/sim`.

Usage:
    python tools/fmt_sim_probe.py                # 8 s, saves capture
    python tools/fmt_sim_probe.py --seconds 20 --save build/validation/sim/run1.bin
Exit codes: 0 stream observed, 2 no reply (simulator not running / other port).
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests" / "simulation" / "golden"))

from generate_golden import CRC_EXTRA, frame  # noqa: E402  (independent encoder)

MAGIC = 0xFD
HEADER = 10
IDS = {0: "HEARTBEAT", 93: "HIL_ACTUATOR_CONTROLS", 107: "HIL_SENSOR", 113: "HIL_GPS"}


def crc_byte(crc: int, byte: int) -> int:
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def heartbeat() -> bytes:
    payload = struct.pack("<IBBBBB", 0, 2, 0, 0, 3, 3)
    return frame(0, payload)


def actuator(now_us: int, controls: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0),
             mode: int = 0) -> bytes:
    values = list(controls) + [0.0] * 12
    payload = struct.pack("<QQ16f", now_us, 0, *values) + struct.pack("<B", mode)
    return frame(93, payload)


def parse(stream: bytearray):
    """Yield (id, payload) for every CRC-valid frame; drop leading garbage."""
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


def decode_sensor(payload: bytes) -> str:
    padded = payload + b"\x00" * (64 - len(payload))
    time_usec = struct.unpack_from("<Q", padded, 0)[0]
    accel = struct.unpack_from("<3f", padded, 8)
    gyro = struct.unpack_from("<3f", padded, 20)
    pressure = struct.unpack_from("<f", padded, 44)[0]
    temperature = struct.unpack_from("<f", padded, 56)[0]
    return (f"t={time_usec}us acc=({accel[0]:+.2f},{accel[1]:+.2f},{accel[2]:+.2f}) "
            f"gyro=({gyro[0]:+.3f},{gyro[1]:+.3f},{gyro[2]:+.3f})rad/s "
            f"p={pressure:.1f}hPa T={temperature:.1f}C")


def decode_gps(payload: bytes) -> str:
    padded = payload + b"\x00" * (37 - len(payload))
    time_usec, lat, lon, alt, eph, epv, vel, vn, ve, vd = struct.unpack_from("<QiiiHHHhhh", padded, 0)
    fix = padded[34]
    sats = padded[35]
    return (f"t={time_usec}us lat={lat * 1e-7:.7f} lon={lon * 1e-7:.7f} alt={alt / 1000.0:.1f}m "
            f"vel=({vn / 100:.2f},{ve / 100:.2f},{vd / 100:.2f})m/s fix={fix} sats={sats} "
            f"eph={eph}cm epv={epv}cm")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Probe a running FMT-SIM HIL endpoint")
    parser.add_argument("--peer", default="127.0.0.1:14553")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--save", default="build/validation/sim/sim_capture.bin")
    parser.add_argument("--actuator-hz", type=float, default=50.0,
                        help="re-send HIL_ACTUATOR_CONTROLS at this rate, 0 = once")
    args = parser.parse_args(argv)

    host, _, port = args.peer.rpartition(":")
    peer = (host, int(port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.2)
    local = sock.getsockname()

    hello = heartbeat() + actuator(0)
    sock.sendto(hello, peer)
    print(f"sent {len(hello)} B (HEARTBEAT + HIL_ACTUATOR_CONTROLS) to {peer} from {local[0]}:{local[1]}")

    deadline = time.monotonic() + args.seconds
    next_actuator = time.monotonic()
    raw = bytearray()
    stream = bytearray()
    counts = {name: 0 for name in IDS.values()}
    first_sensor = ""
    first_gps = ""
    started = time.monotonic()

    while time.monotonic() < deadline:
        if args.actuator_hz > 0 and time.monotonic() >= next_actuator:
            sock.sendto(actuator(int((time.monotonic() - started) * 1e6)), peer)
            next_actuator += 1.0 / args.actuator_hz
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        raw.extend(data)
        stream.extend(data)
        for message_id, payload in parse(stream):
            name = IDS.get(message_id)
            if not name:
                continue
            counts[name] += 1
            if message_id == 107 and not first_sensor:
                first_sensor = decode_sensor(payload)
            elif message_id == 113 and not first_gps:
                first_gps = decode_gps(payload)

    elapsed = time.monotonic() - started
    save = ROOT / args.save
    save.parent.mkdir(parents=True, exist_ok=True)
    save.write_bytes(raw)

    print(f"captured {len(raw)} B in {elapsed:.1f} s -> {save}")
    for name, count in counts.items():
        print(f"  {name:24s} {count:6d}  {count / elapsed:6.1f} Hz")
    if first_sensor:
        print("  first HIL_SENSOR:", first_sensor)
    if first_gps:
        print("  first HIL_GPS   :", first_gps)

    if counts["HIL_SENSOR"] == 0 and counts["HIL_GPS"] == 0:
        print("no HIL stream: is FMT_SIM running and in SIH mode on this port?", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
