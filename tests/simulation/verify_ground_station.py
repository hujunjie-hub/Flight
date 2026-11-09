"""Drive a running Flight/FMT-SIM session through its ground-station UDP port."""

import argparse
import math
import os
import importlib.util
import json
import socket
import struct
import sys
import time
from pathlib import Path


def read_qgc_plan(path):
    items = []
    for item in json.loads(path.read_text(encoding="utf-8-sig"))["mission"]["items"]:
        if item["type"] == "SimpleItem":
            items.append(item)
        else:
            transects = item["TransectStyleComplexItem"]
            assert transects.get("routeOnly"), "Use a Flight route-only pattern or a SIL readback plan"
            items.extend(transects["Items"])
    commands = [item["command"] for item in items]
    assert len(items) >= 3 and commands[0] == 22 and commands[-1] in (20, 21)
    assert all(command == 16 for command in commands[1:-1]), commands
    print(f"{path.name}: {len(items) - 2} waypoints", flush=True)
    return [(item["command"], {2: 2, 3: 6}[item["frame"]],
             round(item["params"][4] * 1e7), round(item["params"][5] * 1e7), item["params"][6])
            for item in items]


def verify(port, dialect, return_home, bench, mission, abort, hold_before_mission, no_pause, auto_plan, logs,
           auxiliary_plans=False, home_clear=False, resume_after_rtl=False, offset_home=False, qgc_plan=None):
    spec = importlib.util.spec_from_file_location("flight_test_dialect", dialect)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(("127.0.0.1", port))
        udp.settimeout(0.25)
        mav = module.MAVLink(None, srcSystem=255, srcComponent=190)
        receive = module.MAVLink(None)
        peer = None
        last_heartbeat = None
        reached = set()
        saw_airborne = saw_touchdown = False
        monitor_plan_failsafe = False

        def wait_for(predicate, timeout=10):
            nonlocal peer, last_heartbeat, saw_airborne, saw_touchdown
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    packet, sender = udp.recvfrom(4096)
                except TimeoutError:
                    if last_heartbeat is not None and time.monotonic() - last_heartbeat > 8.0:
                        raise RuntimeError("Flight heartbeat lost during the scenario")
                    continue
                message = receive.decode(bytearray(packet))
                if message and message.get_type() == "HEARTBEAT":
                    last_heartbeat = time.monotonic()
                    if monitor_plan_failsafe and message.system_status >= module.MAV_STATE_CRITICAL:
                        raise RuntimeError(f"Flight entered failsafe after mission items {sorted(reached)}")
                if message and message.get_type() == "MISSION_ITEM_REACHED":
                    if qgc_plan and message.seq not in reached:
                        print(f"mission item {message.seq} reached", flush=True)
                    reached.add(message.seq)
                if message and message.get_type() == "EXTENDED_SYS_STATE":
                    saw_airborne |= message.landed_state in (2, 3, 4)
                    saw_touchdown |= saw_airborne and message.landed_state == 1
                if message and predicate(message):
                    peer = sender
                    return message
            raise TimeoutError("timed out waiting for Flight")

        def send(message, legacy=False):
            udp.sendto(message.pack(mav, force_mavlink1=legacy), peer)
            mav.seq = (mav.seq + 1) % 256

        def command(code, *params, timeout=240, expected=0, wait_to_retry=None):
            params = (*params, *((0.0,) * (7 - len(params))))
            for attempt in range(6):
                send(mav.command_long_encode(1, 1, code, 0, *params))
                reply = wait_for(lambda message: message.get_type() == "COMMAND_ACK"
                                 and message.command == code and message.result != 5, timeout)
                if reply.result != 1 or wait_to_retry is None or attempt == 5:
                    break
                wait_to_retry()
            assert reply.result == expected, (code, reply.result, reply.result_param2)
            print(f"command {code} returned {expected}", flush=True)

        heartbeat = wait_for(lambda msg: msg.get_type() == "HEARTBEAT", timeout=90)
        assert heartbeat.base_mode & 1
        gyro_and_accelerometer = module.MAV_SYS_STATUS_SENSOR_3D_GYRO | module.MAV_SYS_STATUS_SENSOR_3D_ACCEL
        status = wait_for(lambda msg: msg.get_type() == "SYS_STATUS" and
                          msg.onboard_control_sensors_health & gyro_and_accelerometer == gyro_and_accelerometer)
        assert status.onboard_control_sensors_present & gyro_and_accelerometer == gyro_and_accelerometer
        assert status.onboard_control_sensors_enabled & gyro_and_accelerometer == gyro_and_accelerometer
        receiver_bit = 1 << 16
        assert status.onboard_control_sensors_present & receiver_bit
        assert status.onboard_control_sensors_enabled & receiver_bit
        assert status.onboard_control_sensors_health & receiver_bit
        rc = wait_for(lambda msg: msg.get_type() == "RC_CHANNELS")
        assert (rc.chancount, rc.chan3_raw, rc.chan17_raw, rc.chan18_raw) == (16, 1000, 65535, 65535), rc
        battery = wait_for(lambda msg: msg.get_type() == "BATTERY_STATUS")
        assert battery.id == 0 and battery.temperature == 2500 and battery.current_battery == 500, battery
        assert battery.voltages == [4200] * 4 + [65535] * 6, battery
        assert battery.battery_remaining == 85 and battery.charge_state == 1 and battery.current_consumed == -1, battery
        hud = wait_for(lambda msg: msg.get_type() == "VFR_HUD" and math.isfinite(msg.groundspeed), timeout=30)
        assert math.isnan(hud.airspeed) and math.isfinite(hud.climb), hud
        if logs:
            wait_for(lambda msg: msg.get_type() == "SYS_STATUS")
            send(mav.log_request_list_encode(1, 1, 0, 65535))
            entry = wait_for(lambda msg: msg.get_type() == "LOG_ENTRY")
            assert entry.num_logs == 1 and entry.size > 20 and entry.time_utc > 0, entry
            send(mav.log_request_data_encode(1, 1, entry.id, 0, entry.size))
            stream = bytearray()
            while len(stream) < entry.size:
                block = wait_for(lambda msg: msg.get_type() == "LOG_DATA" and msg.ofs == len(stream))
                assert block.id == entry.id and 0 < block.count <= 90, block
                stream.extend(block.data[:block.count])
            assert len(stream) == entry.size
            send(mav.log_request_data_encode(1, 1, entry.id, 5, 11))
            block = wait_for(lambda msg: msg.get_type() == "LOG_DATA" and msg.ofs == 5)
            assert bytes(block.data[:block.count]) == stream[5:16]
            send(mav.log_request_end_encode(1, 1))
            types = set()
            offset = 0
            while offset < len(stream):
                timestamp, = struct.unpack_from(">Q", stream, offset)
                assert timestamp // 1000000 >= entry.time_utc
                offset += 8
                assert stream[offset] == 0xfd
                size = stream[offset + 1] + 12
                types.add(receive.decode(stream[offset:offset + size]).get_type())
                offset += size
            assert offset == len(stream) and "HEARTBEAT" in types
            logs.write_bytes(stream)
            send(mav.log_erase_encode(1, 1))
            send(mav.log_request_list_encode(1, 1, 0, 65535))
            assert wait_for(lambda msg: msg.get_type() == "LOG_ENTRY").num_logs == 0
            print(f"logs passed: listing, {len(stream)} bytes, range retry, replay, RAM erase", flush=True)
            return
        params = {}

        def received_all(message):
            if message.get_type() != "PARAM_VALUE":
                return False
            params[message.param_index] = message
            return len(params) == message.param_count

        for attempt in range(3):
            send(mav.param_request_list_encode(1, 0), legacy=True)
            try:
                wait_for(received_all)
                break
            except TimeoutError:
                if attempt == 2:
                    raise
        first = params[0]

        def set_parameter(name, value):
            send(mav.param_set_encode(1, 1, name.encode(), value, 9))
            return wait_for(lambda message: message.get_type() == "PARAM_VALUE"
                            and message.param_id == name)

        touched = set_parameter("CTL_ACT_DLY", 0.02)
        assert abs(touched.param_value - 0.02) < 1e-6, touched
        rejected = set_parameter("CTL_ACT_DLY", 1.0)
        assert abs(rejected.param_value - 0.02) < 1e-6, rejected
        restored = set_parameter("CTL_ACT_DLY", 0.0)
        assert restored.param_value == 0.0, restored
        assert not any(p.param_id == "VEH_IXY" for p in params.values())
        print(f"read {len(params)} parameters over MAVLink 1 request; "
              f"session write, range rejection and hidden geometry checks passed", flush=True)
        if any(p.param_id == "SIM_TLM_HZ" for p in params.values()):
            for requested, expected in ((7.0, 7.0), (11.0, 7.0), (5.0, 5.0)):
                send(mav.param_set_encode(1, 1, b"SIM_TLM_HZ", requested, 9))
                rate = wait_for(lambda msg: msg.get_type() == "PARAM_VALUE"
                                and msg.param_id == "SIM_TLM_HZ")
                assert rate.param_value == expected, (requested, rate)
            print("simulation parameter read/write and range rejection passed", flush=True)

        if bench:
            for name, requested, expected in (
                ("RC_MODE_CH", 5, 5), ("RC_MODE_CH", 4, 5), ("RC_MODE_CH", 5.5, 5),
                ("RC_MODE_4", 4, 4), ("RC_MODE_4", 5, 4), ("RC_MODE_4", 0, 0),
                ("RC_MODE_CH", 0, 0),
            ):
                reply = set_parameter(name, requested)
                assert reply.param_value == expected, reply
            print("RC mode channel and slot configuration write, validation and readback passed", flush=True)
            send(mav.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
            version = wait_for(lambda msg: msg.get_type() == "AUTOPILOT_VERSION")
            assert version.capabilities & module.MAV_PROTOCOL_CAPABILITY_MAVLINK2
            for mission_type in (0, 1, 2):
                send(mav.mission_request_list_encode(1, 1, mission_type))
                count = wait_for(lambda msg: msg.get_type() == "MISSION_COUNT")
                assert count.count == 0 and count.mission_type == mission_type
            wait_for(lambda msg: msg.get_type() == "ATTITUDE")
            wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT")
            def calibration_status(kind, state):
                return wait_for(lambda msg: msg.get_type() == "STATUSTEXT" and
                                msg.text.startswith(f"FCAL {kind} {state} "), timeout=45)

            full_calibration = bool(os.environ.get("FLIGHT_SIL_CAL_FIXTURE"))
            if not full_calibration:
                for kind, params in ((1, (1,)), (4, (0, 0, 0, 0, 2)), (5, (0, 0, 1))):
                    command(241, *params, timeout=5)
                    calibration_status(kind, 2)
            for kind, params in ((2, (0, 0, 0, 0, 1)), (3, (0, 1)), (6, (0, 0, 0, 1))):
                command(241, *params, timeout=5)
                calibration_status(kind, 1)
                command(31011, 0, 2, timeout=5)
                calibration_status(kind, 3)
            command(31011, 0, 4, timeout=5, expected=2)  # No completed RC travel calibration.
            print("RC trim copy correctly rejected without calibrated travel", flush=True)
            command(31011, 0, 5, timeout=5)
            print("CRSF bind command accepted by the mock UART with exact frame and both CRCs", flush=True)
            for value, expected in ((4, 4), (4.5, 4), (0, 0)):
                assert set_parameter("RC_AUX_6", value).param_value == expected
            print("Auxiliary switch write, validation and readback passed", flush=True)
            if full_calibration:
                command(241, 1, timeout=5)
                calibration_status(1, 2)
                command(241, 0, 0, 0, 0, 1, timeout=5)
                calibration_status(2, 1)
                for face in range(6):
                    for _ in range(2):
                        wait_for(lambda msg: msg.get_type() == "HEARTBEAT")
                    command(31011, face, 1, timeout=5)
                    if face == 0:
                        landed = wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE")
                        assert landed.landed_state == 1, landed
                calibration_status(2, 2)
                command(241, 0, 0, 0, 0, 2, timeout=5)
                calibration_status(4, 2)
                command(241, 0, 1, timeout=5)
                calibration_status(3, 2)
                command(241, 0, 0, 1, timeout=5)
                calibration_status(5, 2)
                print("All five sensor procedures completed via MAVLink in configuration order", flush=True)
                return  # The bench fixture deliberately rotates simulated sensor inputs.
            # With the normal stationary SIL input, an upright sample cannot
            # satisfy the inverted face. It must remain retryable on face 2.
            command(241, 0, 0, 0, 0, 1, timeout=5)
            calibration_status(2, 1)
            for _ in range(2):
                wait_for(lambda msg: msg.get_type() == "HEARTBEAT")
            command(31011, 0, 1, timeout=5)
            for _ in range(2):
                wait_for(lambda msg: msg.get_type() == "HEARTBEAT")
            command(31011, 1, 1, timeout=5, expected=2)
            retry_status = calibration_status(2, 1)
            assert retry_status.text.split()[3] == "1", retry_status
            command(31011, 0, 2, timeout=5)
            calibration_status(2, 3)
            print("Stationary SIL rejects the inverted face and preserves the first capture", flush=True)
            command(246, 1, timeout=5)
            print("Configuration protocol passed: sensor health, RC monitor, calibration start/completion/cancel and reboot ACK", flush=True)
            # The stationary SIL fixture acknowledges arm, then terminates the
            # scenario without a motion plant. The physical board is checked
            # separately for the BENCH_INHIBIT rejection in the runtime test.
            command(400, 1.0, timeout=5)
            print("Flight/QGroundControl bench protocol passed", flush=True)
            return

        send(mav.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
        version = wait_for(lambda msg: msg.get_type() == "AUTOPILOT_VERSION")
        assert bytes(version.flight_custom_version) == b"FLTSIL01"
        print("Flight SIL identity received by the ground station", flush=True)
        wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE" and msg.landed_state == 1,
                 timeout=90)
        position = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT"
                            and msg.time_boot_ms >= 6000, timeout=150)
        if offset_home:
            send(mav.command_int_encode(1, 1, 0, 179, 0, 0, 0, 0, 0, 0,
                                        position.lat, position.lon, -1.0))
            reply = wait_for(lambda msg: msg.get_type() == "COMMAND_ACK" and msg.command == 179 and msg.result != 5)
            assert reply.result == 0, reply
            wait_for(lambda msg: msg.get_type() == "HOME_POSITION" and msg.altitude == -1000)
        else:
            command(179, 1.0)  # Capture Home at the current estimator position while disarmed.
        if home_clear:
            def clear_empty(expected):
                send(mav.mission_clear_all_encode(1, 1, 0))
                reply = wait_for(lambda msg: msg.get_type() == "MISSION_ACK" and msg.mission_type == 0)
                assert reply.type == expected, reply

            def set_home(frame, lat, lon, expected):
                send(mav.command_int_encode(1, 1, frame, 179, 0, 0, 0, 0, 0, 0, lat, lon, 0))
                reply = wait_for(lambda msg: msg.get_type() == "COMMAND_ACK"
                                 and msg.command == 179 and msg.result != 5)
                assert reply.result == expected, reply

            def check_home(lat, lon):
                send(mav.command_long_encode(1, 1, 512, 0, 242, 0, 0, 0, 0, 0, 0))
                home = wait_for(lambda msg: msg.get_type() == "HOME_POSITION")
                assert (home.latitude, home.longitude, home.altitude) == (lat, lon, 0), home

            lat, lon = position.lat + 45, position.lon - 50
            set_home(6, lat, lon, 2)
            set_home(0, lat + 100000, lon, 2)
            set_home(0, lat, lon, 0)
            check_home(lat, lon)
            clear_empty(0)
            clear_empty(0)
            command(400, 1.0)
            check_home(lat, lon)
            clear_empty(14)
            command(400, 0.0)
            clear_empty(0)
            print("explicit Home and idempotent disarmed mission clear passed", flush=True)
            if not auxiliary_plans:
                return
        if auxiliary_plans:
            def count(plan_type, expected):
                send(mav.mission_request_list_encode(1, 1, plan_type))
                reply = wait_for(lambda msg: msg.get_type() == "MISSION_COUNT" and msg.mission_type == plan_type)
                assert reply.count == expected, reply

            def upload_auxiliary(plan_type, items, result=0):
                send(mav.mission_count_encode(1, 1, len(items), plan_type))
                for index, (operation, frame, param1, lat, lon, alt) in enumerate(items):
                    wait_for(lambda msg: msg.get_type() == "MISSION_REQUEST_INT" and msg.seq == index
                             and msg.mission_type == plan_type)
                    send(mav.mission_item_int_encode(1, 1, index, frame, operation, 0, int(plan_type == 0),
                                                     param1, 0, 0, 0, lat, lon, alt, plan_type))
                reply = wait_for(lambda msg: msg.get_type() == "MISSION_ACK" and msg.mission_type == plan_type)
                assert reply.type == result, reply

            def roundtrip(plan_type, items):
                count(plan_type, len(items))
                for index, (operation, frame, param1, lat, lon, alt) in enumerate(items):
                    send(mav.mission_request_int_encode(1, 1, index, plan_type))
                    item = wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_INT"
                                    and msg.mission_type == plan_type and msg.seq == index)
                    assert (item.command, item.frame, item.param1, item.x, item.y, item.z) == (
                        operation, frame, param1, lat, lon, alt), item

            lat, lon = position.lat, position.lon
            fence = [(5001, 0, 4, lat + dn, lon + de, 0) for dn, de in
                     [(-500, -500), (-500, 500), (500, 500), (500, -500)]]
            fence.append((5004, 0, 0.75, lat, lon + 300, 0))
            rally = [(5100, 3, 0, lat + 200, lon, 2)]
            assert version.capabilities & module.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE
            assert version.capabilities & module.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY
            upload_auxiliary(1, fence)
            upload_auxiliary(2, rally)
            upload_auxiliary(1, [fence[i] for i in (0, 2, 1, 3)], result=5)
            roundtrip(1, fence)
            roundtrip(2, rally)
            upload_auxiliary(0, [(16, 6, 0, lat + 900, lon, 2)], result=14)
            upload_auxiliary(0, [(16, 6, 0, lat, lon + 300, 2)], result=14)
            count(0, 0)
            upload_auxiliary(0, [(16, 6, 0, lat + 200, lon, 2)])
            count(0, 1)
            for plan_type in (0, 1, 2):
                send(mav.mission_clear_all_encode(1, 1, plan_type))
                ack = wait_for(lambda msg: msg.get_type() == "MISSION_ACK" and msg.mission_type == plan_type)
                assert ack.type == 0, ack
                count(plan_type, 0)
            print("Fence and rally upload, readback, independent clear and invalid fence rollback passed.")
            print("FMS rejected waypoints outside the inclusion polygon and inside the exclusion circle.")
            print("FMS committed an allowed waypoint.")
            return
        if mission:
            items = [(16, 6, position.lat + 117, position.lon, 2.0),
                     (16, 6, position.lat + 117, position.lon + 137, 2.0)]
            if auto_plan:
                items.insert(0, (22, 6, position.lat, position.lon, 2.0))
                items.append((20, 2, 0, 0, 0.0) if return_home
                             else (21, 6, items[-1][2], items[-1][3], 0.0))
            if qgc_plan:
                items = read_qgc_plan(qgc_plan)

            def mission_count(expected):
                send(mav.mission_request_list_encode(1, 1, 0))
                count = wait_for(lambda msg: msg.get_type() == "MISSION_COUNT"
                                 and msg.mission_type == 0)
                assert count.count == expected, count

            def upload_plan(plan, expected=0):
                send(mav.mission_count_encode(1, 1, len(plan), 0))
                for index, (operation, frame, lat, lon, altitude) in enumerate(plan):
                    request = wait_for(lambda msg: msg.get_type() == "MISSION_REQUEST_INT"
                                       and msg.seq == index)
                    assert request.target_system == 255 and request.target_component == 190
                    send(mav.mission_item_int_encode(1, 1, index, frame, operation, 0, 1,
                                                     0, .35 if operation == 16 else 0,
                                                     0, 0, lat, lon, altitude, 0))
                ack = wait_for(lambda msg: msg.get_type() == "MISSION_ACK")
                assert ack.type == expected, ack

            mission_count(0)
            upload_plan(items)
            upload_plan([(16, 6, position.lat + 50000, position.lon, 2.0)], expected=5)
            mission_count(len(items))
            for index, (operation, frame, lat, lon, altitude) in enumerate(items):
                send(mav.mission_request_int_encode(1, 1, index, 0))
                item = wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_INT"
                                and msg.seq == index)
                assert (item.command, item.frame, item.x, item.y, item.z) == (
                    operation, frame, lat, lon, altitude), item
            print("mission uploaded, invalid replacement rejected, mission downloaded intact", flush=True)
            if auto_plan:
                monitor_plan_failsafe = qgc_plan is not None
                command(300, 0.0, float(len(items) - 1))
                if offset_home:
                    wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq == 1, timeout=120)
                    reached_position = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT")
                    assert abs(reached_position.relative_alt / 1000.0 - 2.0) <= 0.4, reached_position
                    print("waypoint altitude is relative to the offset Home", flush=True)
                wait_for(lambda msg: msg.get_type() == "MISSION_CURRENT"
                         and msg.mission_state == 5 and msg.seq == len(items) - 1, timeout=600 if qgc_plan else 300)
                assert reached == set(range(len(items))), reached
                if not saw_touchdown:
                    wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE" and msg.landed_state == 1)
                wait_for(lambda msg: msg.get_type() == "HEARTBEAT" and not msg.base_mode & 128)
                send(mav.mission_clear_all_encode(1, 1, 0))
                assert wait_for(lambda msg: msg.get_type() == "MISSION_ACK").type == 0
                mission_count(0)
                print("one-command plan passed: takeoff, all waypoints, terminal action and disarm", flush=True)
                return
            if resume_after_rtl:
                takeoff = (22, 6, position.lat, position.lon, 2.0)
                items.append((16, 6, position.lat + 235, position.lon + 137, 2.0))
                upload_plan([takeoff, *items])
                command(300)
                wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq == 1, timeout=120)
                command(20)
                wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE" and msg.landed_state == 1)
                command(400, 0.0)
                wait_for(lambda msg: msg.get_type() == "HEARTBEAT" and not msg.base_mode & 128)
                landed = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT")
                resume_takeoff = (22, 6, landed.lat, landed.lon, 2.0)
                upload_plan([resume_takeoff, *items[1:]])
                reached.clear()
                command(300)
                wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq == 2, timeout=120)
                assert reached == {0, 1, 2}, reached
                wait_for(lambda msg: msg.get_type() == "MISSION_CURRENT" and msg.mission_state == 5)

                def wait_for_hold():
                    stamp = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT").time_boot_ms
                    wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT"
                             and msg.time_boot_ms >= stamp + 400)

                command(21, wait_to_retry=wait_for_hold)
                wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE" and msg.landed_state == 1)
                command(400, 0.0)
                print("resume plan passed: interrupt with RTL, disarm, upload remainder and relaunch", flush=True)
                return
        command(400, 1.0)
        send(mav.command_long_encode(1, 1, 512, 0, 242, 0, 0, 0, 0, 0, 0))
        home = wait_for(lambda msg: msg.get_type() == "HOME_POSITION")
        assert home.latitude != 0 and home.longitude != 0
        print("received FMS home position", flush=True)
        command(22, 0, 0, 0, 0, 0, 0, 2.0)
        position = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT" and msg.relative_alt >= 1700,
                            timeout=30)
        print(f"estimated altitude {position.relative_alt / 1000:.2f} m", flush=True)
        if mission:
            def wait_for_settle(duration_ms=400):
                first = wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT").time_boot_ms
                wait_for(lambda msg: msg.get_type() == "GLOBAL_POSITION_INT"
                         and msg.time_boot_ms >= first + duration_ms, timeout=90)

            wait_for_settle(max(400, round(hold_before_mission * 1000)))
            command(300, 0.0, float(len(items) - 1))
            if abort:
                wait_for_settle()
                command(31010)
                wait_for_settle()
                print("mission aborted; ready for direct landing", flush=True)
            else:
                if not no_pause:
                    wait_for_settle()
                    command(193, 0.0)
                    wait_for_settle(2200)
                    command(300, wait_to_retry=wait_for_settle)  # Native Continue Mission.
                wait_for(lambda msg: msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq == len(items) - 1,
                         timeout=240)
                print("final waypoint completion confirmed" if no_pause else
                      "mission pause, resume and final waypoint completion confirmed", flush=True)
                if not return_home:
                    wait_for_settle()
        command(20 if return_home else 21)
        wait_for(lambda msg: msg.get_type() == "EXTENDED_SYS_STATE" and msg.landed_state == 1)
        command(400, 0.0)
        wait_for(lambda msg: msg.get_type() == "HEARTBEAT" and not msg.base_mode & 128)
        if mission:
            send(mav.mission_clear_all_encode(1, 1, 0))
            ack = wait_for(lambda msg: msg.get_type() == "MISSION_ACK")
            assert ack.type == 0, ack
            mission_count(0)
            print("mission cleared after landing", flush=True)
        print("Flight/FMT-SIM basic ground-station loop passed", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--dialect", type=Path, required=True)
    parser.add_argument("--return-home", action="store_true", help="use return-to-home instead of direct landing")
    parser.add_argument("--bench", action="store_true", help="verify the stationary ground link")
    parser.add_argument("--mission", action="store_true", help="verify mission upload, execution and clear")
    parser.add_argument("--abort", action="store_true", help="abort the mission and test direct landing")
    parser.add_argument("--hold-before-mission", type=float, default=0.0, metavar="SECONDS")
    parser.add_argument("--no-pause", action="store_true", help="fly the mission without pausing")
    parser.add_argument("--auto-plan", action="store_true", help="execute takeoff through disarm with one mission start")
    parser.add_argument("--aux-plans", action="store_true", help="verify fence and rally transfer and FMS waypoint checks")
    parser.add_argument("--home-clear", action="store_true", help="verify explicit Home and empty mission clear")
    parser.add_argument("--resume-after-rtl", action="store_true", help="upload and fly the remaining mission after RTL")
    parser.add_argument("--offset-home", action="store_true", help="verify waypoint altitude with Home 1 m below the launch point")
    parser.add_argument("--qgc-plan", type=Path, help="execute a Flight route-only QGC plan or a flattened SIL readback")
    parser.add_argument("--logs", type=Path, help="test log download and save a replayable .tlog")
    arguments = parser.parse_args()
    if arguments.abort and (not arguments.mission or arguments.return_home or arguments.bench):
        parser.error("--abort requires --mission and a direct landing")
    if not 0.0 <= arguments.hold_before_mission <= 10.0:
        parser.error("--hold-before-mission must be between 0 and 10 seconds")
    if (arguments.no_pause or arguments.hold_before_mission) and not arguments.mission:
        parser.error("--no-pause and --hold-before-mission require --mission")
    if arguments.auto_plan and (not arguments.mission or arguments.bench or arguments.abort
                                or arguments.no_pause or arguments.hold_before_mission):
        parser.error("--auto-plan requires --mission without manual phase options")
    if arguments.offset_home and (not arguments.auto_plan or arguments.home_clear or arguments.aux_plans):
        parser.error("--offset-home requires an automatic mission without auxiliary tests")
    if arguments.qgc_plan and (not arguments.auto_plan or arguments.aux_plans or arguments.home_clear):
        parser.error("--qgc-plan requires --mission --auto-plan without auxiliary tests")
    if arguments.logs and not arguments.bench:
        parser.error("--logs requires --bench so the test erases only a disposable log")
    if arguments.resume_after_rtl and (not arguments.mission or arguments.auto_plan or arguments.aux_plans
                                       or arguments.bench or arguments.abort or arguments.home_clear):
        parser.error("--resume-after-rtl requires --mission without other scenarios")
    verify(arguments.port, arguments.dialect, arguments.return_home, arguments.bench,
           arguments.mission, arguments.abort, arguments.hold_before_mission, arguments.no_pause,
           arguments.auto_plan, arguments.logs, arguments.aux_plans, arguments.home_clear,
           arguments.resume_after_rtl, arguments.offset_home, arguments.qgc_plan)
