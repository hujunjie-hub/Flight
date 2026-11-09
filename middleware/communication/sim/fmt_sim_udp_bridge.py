#!/usr/bin/env python3
"""Transparent serial <-> UDP bridge for simulation traffic.

The Flight UART HIL task expects a peer that sends standard HIL_SENSOR and
HIL_GPS. The installed FMT-SIM sends FMT_EXTERNAL_STATE in HIL mode, which
uses the Host SIL connector. With --sim-root this bridge reads the SIH
endpoint for an upstream that emits HIL_STATE poses.

Quick start

    # Standard UART HIL peer (needs pyserial, default uart2 @ 921600):
    python middleware/communication/sim/fmt_sim_udp_bridge.py --port COM7 --udp-peer 127.0.0.1:14560

    # FMT-SIM SIH display (serial upstream provides HIL_STATE):
    python middleware/communication/sim/fmt_sim_udp_bridge.py --port COM7 --sim-root D:/FMT-SIM --vehicle-id 1

Exit codes: 0 clean stop (Ctrl+C), 1 bad arguments or serial open failure.
Requires: pyserial (`pip install pyserial`).
"""
from __future__ import annotations

import argparse
from ipaddress import IPv4Address
import json
from pathlib import Path
import socket
import sys
import threading
import time

MAVLINK_V2_MAGIC = 0xFD
HEADER_LEN = 10
CRC_LEN = 2


class Counters:
    """Plain byte counters plus an independent MAVLink v2 frame count."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.serial_to_udp_bytes = 0
        self.udp_to_serial_bytes = 0
        self.serial_to_udp_frames = 0
        self.udp_to_serial_frames = 0
        self.udp_datagrams = 0
        self.serial_errors = 0
        self.udp_errors = 0

    def bump(self, **kwargs: int) -> None:
        with self.lock:
            for name, value in kwargs.items():
                setattr(self, name, getattr(self, name) + value)

    def snapshot(self) -> dict[str, int]:
        with self.lock:
            return dict(self.__dict__)


class FrameCounter:
    """Counts complete MAVLink v2 frames in a byte stream (diagnostics only)."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> int:
        self.buffer.extend(data)
        frames = 0
        while len(self.buffer) >= HEADER_LEN + CRC_LEN:
            if self.buffer[0] != MAVLINK_V2_MAGIC:
                del self.buffer[0]
                continue
            payload_len = self.buffer[1]
            total = HEADER_LEN + payload_len + CRC_LEN
            if payload_len > 255 or total > 280:
                del self.buffer[0]
                continue
            if len(self.buffer) < total:
                break
            del self.buffer[:total]
            frames += 1
        if len(self.buffer) > 4096:  # a stalled stream must not grow forever
            self.buffer.clear()
        return frames


def parse_endpoint(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit():
        raise argparse.ArgumentTypeError(f"expected HOST:PORT, got {text!r}")
    return host, int(port)


def fmt_sim_endpoint(root: Path, vehicle_id: int) -> tuple[str, int]:
    settings = root / "FMT_SIM" / "Content" / "Settings" / "settings.jason"
    document = json.loads(settings.read_text(encoding="utf-8-sig"))
    vehicles = document.get("Vehicles") if isinstance(document, dict) else None
    if not isinstance(vehicles, list):
        raise ValueError(f"{settings}: expected a Vehicles list")

    matches = [vehicle for vehicle in vehicles if isinstance(vehicle, dict) and
               vehicle.get("ID") == vehicle_id]
    if len(matches) != 1:
        raise ValueError(f"{settings}: expected exactly one vehicle with ID {vehicle_id}")

    vehicle = matches[0]
    fcs = vehicle.get("MavproxyFCS")
    if (vehicle.get("SimMode") != "SIH" or not isinstance(fcs, dict) or
            fcs.get("Interface") != "UDP" or fcs.get("Autobound") is not True):
        raise ValueError(f"{settings}: vehicle {vehicle_id} requires SIH and UDP MavproxyFCS with Autobound")

    host, port = fcs.get("LocalIP"), fcs.get("LocalPort")
    if not isinstance(host, str) or type(port) is not int or not 1 <= port <= 65535:
        raise ValueError(f"{settings}: vehicle {vehicle_id} has an invalid LocalIP or LocalPort")
    try:
        address = IPv4Address(host)
    except ValueError as error:
        raise ValueError(f"{settings}: vehicle {vehicle_id} has an invalid LocalIP {host!r}") from error
    return ("127.0.0.1" if address.is_unspecified else str(address)), port


def open_serial(port: str, baud: int):
    try:
        import serial  # noqa: PLC0415 - optional dependency, clear error below
    except ImportError:
        raise SystemExit("pyserial is required: pip install pyserial") from None
    try:
        return serial.Serial(port, baud, timeout=0.05, write_timeout=0.05)
    except Exception as error:  # serial.SerialException and friends
        raise SystemExit(f"cannot open {port} @ {baud}: {error}") from None


def serial_to_udp(link, udp, peer, counters: Counters, stop: threading.Event,
                  hexdump: int) -> None:
    frames = FrameCounter()
    while not stop.is_set():
        try:
            chunk = link.read(512)
        except Exception:
            counters.bump(serial_errors=1)
            stop.set()
            return
        if not chunk:
            continue
        counters.bump(serial_to_udp_bytes=len(chunk),
                      serial_to_udp_frames=frames.feed(chunk))
        try:
            udp.sendto(chunk, peer)
        except OSError:
            counters.bump(udp_errors=1)
        if hexdump:
            print(f"FCU->SIM {len(chunk):4d} B {chunk[:hexdump].hex()}")


def udp_to_serial(udp, link, counters: Counters, stop: threading.Event,
                  hexdump: int) -> None:
    frames = FrameCounter()
    while not stop.is_set():
        try:
            data, _source = udp.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            counters.bump(udp_errors=1)
            stop.set()
            return
        counters.bump(udp_to_serial_bytes=len(data), udp_datagrams=1,
                      udp_to_serial_frames=frames.feed(data))
        try:
            link.write(data)
        except Exception:
            counters.bump(serial_errors=1)
            stop.set()
            return
        if hexdump:
            print(f"SIM->FCU {len(data):4d} B {data[:hexdump].hex()}")


def report(counters: Counters, interval: float, stop: threading.Event) -> None:
    previous = counters.snapshot()
    while not stop.wait(interval):
        now = counters.snapshot()
        delta_s2u = now["serial_to_udp_bytes"] - previous["serial_to_udp_bytes"]
        delta_u2s = now["udp_to_serial_bytes"] - previous["udp_to_serial_bytes"]
        print(
            f"FCU->SIM {now['serial_to_udp_frames']:7d} frames "
            f"({delta_s2u / interval:7.0f} B/s)  "
            f"SIM->FCU {now['udp_to_serial_frames']:7d} frames "
            f"({delta_u2s / interval:7.0f} B/s)  "
            f"dg={now['udp_datagrams']} err={now['serial_errors'] + now['udp_errors']}",
            flush=True,
        )
        previous = now


def self_test() -> int:
    """Checks the frame counter and endpoint parser without opening hardware."""
    stream = bytes([0xFD, 9, 0, 0, 1, 1, 1, 0, 0, 0]) + b"\x00" * 9 + b"\x00\x00"
    stream += b"garbage" + stream
    counter = FrameCounter()
    assert counter.feed(stream[:5]) == 0
    assert counter.feed(stream[5:]) == 2, "expected two complete frames"
    assert parse_endpoint("127.0.0.1:14553") == ("127.0.0.1", 14553)
    print("bridge self-test OK")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="FMT-SIM HIL bridge: FCU serial <-> simulator UDP")
    parser.add_argument("--port", help="flight controller HIL serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=921600,
                        help="must match FLIGHT_SIM_LINK_BAUD (default 921600)")
    peer_options = parser.add_mutually_exclusive_group()
    peer_options.add_argument("--udp-peer", type=parse_endpoint, metavar="HOST:PORT",
                              help="FMT-SIM MavproxyFCS endpoint (default 127.0.0.1:14553)")
    peer_options.add_argument("--sim-root", type=Path,
                              help="read the SIH endpoint from a local FMT-SIM installation")
    parser.add_argument("--vehicle-id", type=int, help="vehicle in --sim-root (default 1)")
    parser.add_argument("--udp-bind", type=parse_endpoint, default="127.0.0.1:0",
                        metavar="HOST:PORT",
                        help="local UDP address; port 0 picks an ephemeral one")
    parser.add_argument("--udp-peer-port", type=int, default=None,
                        help="override the peer port without repeating the host")
    parser.add_argument("--stats", type=float, default=5.0,
                        help="seconds between statistics lines, 0 disables them")
    parser.add_argument("--hexdump", type=int, default=0, metavar="N",
                        help="print the first N bytes of every chunk")
    parser.add_argument("--self-test", action="store_true",
                        help="run the offline checks and exit")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.port:
        parser.error("--port is required unless --self-test is used")

    if args.vehicle_id is not None and args.sim_root is None:
        parser.error("--vehicle-id requires --sim-root")

    peer = args.udp_peer or ("127.0.0.1", 14553)
    if args.sim_root is not None:
        try:
            vehicle_id = 1 if args.vehicle_id is None else args.vehicle_id
            peer = fmt_sim_endpoint(args.sim_root, vehicle_id)
        except (OSError, ValueError) as error:
            parser.error(str(error))
    if args.udp_peer_port is not None:
        peer = (peer[0], args.udp_peer_port)

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(args.udp_bind)
    udp.settimeout(0.05)
    link = open_serial(args.port, args.baud)

    counters = Counters()
    stop = threading.Event()
    threads = [
        threading.Thread(target=serial_to_udp,
                         args=(link, udp, peer, counters, stop, args.hexdump),
                         name="serial->udp", daemon=True),
        threading.Thread(target=udp_to_serial,
                         args=(udp, link, counters, stop, args.hexdump),
                         name="udp->serial", daemon=True),
    ]
    for thread in threads:
        thread.start()
    if args.stats > 0:
        threading.Thread(target=report, args=(counters, args.stats, stop),
                         name="stats", daemon=True).start()

    print(f"bridging {args.port} @ {args.baud} <-> UDP {peer[0]}:{peer[1]} "
          f"(local {args.udp_bind[0]}:{udp.getsockname()[1]})", flush=True)
    try:
        while not stop.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            link.close()
        finally:
            udp.close()
    final = counters.snapshot()
    print(f"stopped: FCU->SIM {final['serial_to_udp_bytes']} B in "
          f"{final['serial_to_udp_frames']} frames, SIM->FCU "
          f"{final['udp_to_serial_bytes']} B in {final['udp_to_serial_frames']} frames, "
          f"errors {final['serial_errors'] + final['udp_errors']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
