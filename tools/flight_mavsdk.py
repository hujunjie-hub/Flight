# /// script
# requires-python = ">=3.10"
# dependencies = ["mavsdk==4.1.0"]
# ///
"""Send Flight SIL actions over the local MAVLink route."""

import argparse
import sys

from mavsdk import Configuration, ConnectionResult, Mavsdk
from mavsdk.plugins.action import Action, ActionError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("commands", nargs="+", choices=["status", "arm", "disarm", "takeoff", "land", "rtl"])
    parser.add_argument("--address", default="udpin://127.0.0.1:14540")
    parser.add_argument("--altitude", type=float, default=2.0, help="takeoff height above home in metres")
    parser.add_argument("--connect-timeout", type=float, default=15.0)
    args = parser.parse_args()
    if not 0 < args.altitude <= 10:
        parser.error("--altitude must be in (0, 10] metres")
    if not 0 < args.connect_timeout <= 120:
        parser.error("--connect-timeout must be in (0, 120] seconds")

    config = Configuration.create_manual(245, 190, True)
    try:
        with Mavsdk(config) as sdk:
            connection = sdk.add_any_connection(args.address)
            if connection != ConnectionResult.SUCCESS:
                raise RuntimeError(f"connection: {connection.name}")
            system = sdk.first_autopilot(args.connect_timeout)
            if system is None:
                raise RuntimeError("no Flight heartbeat")
            if system.get_system_id() != 1 or 1 not in system.component_ids():
                raise RuntimeError(f"expected Flight system/component 1/1, found system {system.get_system_id()}")
            action = Action(system)
            for command in args.commands:
                if command == "status":
                    print("Flight 1/1 connected", flush=True)
                    continue
                if command == "takeoff":
                    action.set_takeoff_altitude(args.altitude)
                method = "return_to_launch" if command == "rtl" else command
                result = getattr(action, method)()
                print(f"{command}: {result.name}", flush=True)
    except (ActionError, RuntimeError) as error:
        print(error, file=sys.stderr)
        return 1
    finally:
        config.destroy()
    return 0


if __name__ == "__main__":
    sys.exit(main())
