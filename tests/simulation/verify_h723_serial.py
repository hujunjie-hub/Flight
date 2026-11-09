"""Read-only MAVLink acceptance for the Flight H723 USART2 bench link."""

import argparse
import importlib.util
from pathlib import Path
import sys
import time

import serial
from serial.tools import list_ports


def verify(port, dialect, timeout):
    spec = importlib.util.spec_from_file_location("flight_h723_dialect", dialect)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    receive = module.MAVLink(None)
    receive.robust_parsing = True
    send = module.MAVLink(None, srcSystem=255, srcComponent=190)
    required = {"FLT_PROFILE", "FLT_CONTRACT", "FLT_BENCH"}
    parameters = {}
    seen = set()
    requested = False
    deadline = time.monotonic() + timeout
    with serial.Serial(port, baudrate=115200, timeout=0.2) as link:
        while time.monotonic() < deadline:
            packet = link.read(link.in_waiting or 1)
            for message in receive.parse_buffer(packet) or ():
                if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                name = message.get_type()
                seen.add(name)
                if name == "HEARTBEAT":
                    if message.base_mode & module.MAV_MODE_FLAG_SAFETY_ARMED:
                        raise RuntimeError("Flight reported an armed state on the bench link")
                    if not requested:
                        send.file = link
                        send.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
                        send.param_request_list_send(1, 1)
                        requested = True
                elif name == "AUTOPILOT_VERSION":
                    if not message.capabilities & module.MAV_PROTOCOL_CAPABILITY_MAVLINK2:
                        raise RuntimeError("Flight did not report the MAVLink 2 capability")
                elif name == "PARAM_VALUE" and message.param_id in required:
                    parameters[message.param_id] = message.param_value
                if required <= parameters.keys() and {"HEARTBEAT", "SYS_STATUS", "AUTOPILOT_VERSION"} <= seen:
                    if parameters["FLT_BENCH"] != 1.0:
                        raise RuntimeError("Flight did not report the bench inhibit profile")
                    print(f"PASS {port} at 115200: MAVLink 2, unarmed heartbeat, SYS_STATUS and {parameters}")
                    return
    raise TimeoutError(f"{port}: observed {sorted(seen)}, missing parameters {sorted(required - parameters.keys())}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="H723 USART2 adapter, for example COM7")
    parser.add_argument("--list", action="store_true", help="list ports without opening a device")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--dialect", type=Path, default=Path("build/firmware_integration/wire/fcu.py"))
    arguments = parser.parse_args()
    if arguments.list:
        ports = sorted(list_ports.comports())
        for port in ports:
            print(f"{port.device}  {port.description}")
        if not ports:
            print("No serial ports detected.")
        return
    if not arguments.port or arguments.timeout <= 0.0:
        parser.error("specify --port and a positive --timeout, or use --list")
    verify(arguments.port, arguments.dialect, arguments.timeout)


if __name__ == "__main__":
    main()
