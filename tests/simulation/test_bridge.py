#!/usr/bin/env python3
"""End-to-end test of the UDP bridge without hardware.

The bridge in `middleware/communication/sim` forwards bytes, so a fake serial link
plus two real UDP sockets exercise the same threads the field setup uses:

    FakeLink  --serial_to_udp-->  UDP peer  (the simulator side)
    UDP peer  --udp_to_serial-->  FakeLink  (the flight controller side)

What this proves: bytes are not re-framed, split or merged on the way through,
the counters agree with the traffic, and malformed arguments are rejected.
What it cannot prove: the driver/baud/timing of a real UART (NOT_RUN).

Run: python tests/simulation/test_bridge.py
"""
from __future__ import annotations

import argparse
import importlib.util
import socket
import threading
import time
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[2] / "middleware/communication/sim/fmt_sim_udp_bridge.py"


def load_bridge():
    spec = importlib.util.spec_from_file_location("fmt_sim_udp_bridge", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load_bridge()


class FakeLink:
    """Minimal pyserial stand-in: byte queue + write capture."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.incoming = bytearray()
        self.written = bytearray()
        self.closed = False

    def queue(self, data: bytes) -> None:
        with self.lock:
            self.incoming.extend(data)

    def read(self, size: int) -> bytes:
        with self.lock:
            chunk = bytes(self.incoming[:size])
            del self.incoming[:size]
        if not chunk:
            time.sleep(0.005)  # behave like a 5 ms serial read timeout
        return chunk

    def write(self, data: bytes) -> int:
        with self.lock:
            self.written.extend(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


def make_frame(payload: bytes) -> bytes:
    """A MAVLink v2 shaped frame; the bridge must not care about contents."""
    body = bytes([0xFD, len(payload), 0, 0, 1, 1, 1, 107, 0, 0]) + payload + b"\x00\x00"
    return body


class BridgeEndToEndTest(unittest.TestCase):
    def setUp(self) -> None:
        self.link = FakeLink()
        self.bridge_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bridge_socket.bind(("127.0.0.1", 0))
        self.bridge_socket.settimeout(0.05)
        self.peer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer_socket.bind(("127.0.0.1", 0))
        self.peer_socket.settimeout(0.05)
        self.peer = self.peer_socket.getsockname()
        self.counters = bridge.Counters()
        self.stop = threading.Event()

    def tearDown(self) -> None:
        self.stop.set()
        self.bridge_socket.close()
        self.peer_socket.close()

    def _start(self):
        to_udp = threading.Thread(
            target=bridge.serial_to_udp,
            args=(self.link, self.bridge_socket, self.peer, self.counters, self.stop, 0),
            daemon=True)
        to_serial = threading.Thread(
            target=bridge.udp_to_serial,
            args=(self.bridge_socket, self.link, self.counters, self.stop, 0),
            daemon=True)
        to_udp.start()
        to_serial.start()
        return to_udp, to_serial

    def _receive_from_bridge(self, expected: int, timeout: float = 3.0) -> bytes:
        deadline = time.monotonic() + timeout
        received = bytearray()
        while len(received) < expected and time.monotonic() < deadline:
            try:
                data, _ = self.peer_socket.recvfrom(4096)
            except socket.timeout:
                continue
            received.extend(data)
        return bytes(received)

    def test_serial_bytes_reach_udp_unchanged(self) -> None:
        self._start()
        stream = make_frame(bytes(range(64))) + make_frame(b"\x01\x02\x03")
        self.link.queue(stream)
        self.assertEqual(self._receive_from_bridge(len(stream)), stream)

    def test_udp_datagram_reaches_serial_unchanged(self) -> None:
        self._start()
        stream = make_frame(b"\x10\x20\x30\x40")
        for offset in range(0, len(stream), 17):  # several datagrams, no framing
            self.peer_socket.sendto(stream[offset:offset + 17], self.bridge_socket.getsockname())
        deadline = time.monotonic() + 3.0
        while len(self.link.written) < len(stream) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(bytes(self.link.written), stream)

    def test_counters_track_frames_and_bytes(self) -> None:
        self._start()
        stream = make_frame(b"\x00" * 8) + make_frame(b"\x00" * 8)
        self.link.queue(stream)
        self._receive_from_bridge(len(stream))
        deadline = time.monotonic() + 3.0
        while self.counters.snapshot()["serial_to_udp_frames"] < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        snapshot = self.counters.snapshot()
        self.assertEqual(snapshot["serial_to_udp_bytes"], len(stream))
        self.assertEqual(snapshot["serial_to_udp_frames"], 2)


class FrameCounterTest(unittest.TestCase):
    def test_counts_frames_and_resyncs(self) -> None:
        frame = make_frame(b"\x01" * 4)
        counter = bridge.FrameCounter()
        self.assertEqual(counter.feed(b"noise" + frame[:5]), 0)
        self.assertEqual(counter.feed(frame[5:] + frame), 2)

    def test_truncated_frame_waits_for_completion(self) -> None:
        frame = make_frame(b"\x02" * 4)
        counter = bridge.FrameCounter()
        self.assertEqual(counter.feed(frame[:-2]), 0)
        self.assertGreater(len(counter.buffer), 0)
        self.assertEqual(counter.feed(frame[-2:]), 1)
        self.assertEqual(len(counter.buffer), 0)


class ArgumentTest(unittest.TestCase):
    def test_peer_parser(self) -> None:
        self.assertEqual(bridge.parse_endpoint("127.0.0.1:14553"), ("127.0.0.1", 14553))
        self.assertEqual(bridge.parse_endpoint("10.0.0.2:0"), ("10.0.0.2", 0))
        with self.assertRaises(argparse.ArgumentTypeError):
            bridge.parse_endpoint("127.0.0.1")
        with self.assertRaises(argparse.ArgumentTypeError):
            bridge.parse_endpoint(":14553")


if __name__ == "__main__":
    unittest.main(verbosity=2)
