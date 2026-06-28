#!/usr/bin/env python3
"""motor_tool: one CLI for every direct-to-motor task (not a ROS node).

Run in WSL with the U2D2 attached and 12 V on:

    python3 tools/motor_tool.py scan                 # find motors across baud rates
    python3 tools/motor_tool.py read                 # pos / goal / torque / error per motor
    python3 tools/motor_tool.py zero                 # drive all to center (2048) and hold
    python3 tools/motor_tool.py home-offset 1 2048   # calibrate id 1 so its pose reads 2048
    python3 tools/motor_tool.py find-limits          # back-drive by hand; capture min/max ticks
    python3 tools/motor_tool.py set-limits           # burn config lo/hi into motor EEPROM

Commissioning order: zero -> (home-offset if a horn is clocked) -> find-limits ->
put the numbers in config/arm_params.yaml -> set-limits.
"""
import argparse
import os
import time

from dynamixel_sdk import PortHandler, PacketHandler

DEVICE, BAUD = "/dev/ttyUSB0", 57600
IDS = [1, 2, 3]                      # gripper, wrist, elbow
NAMES = {1: "gripper", 2: "wrist", 3: "elbow"}
CENTER = 2048
CONFIG = os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "dyn_arm",
                      "config", "arm_params.yaml")

# control table
ADDR_HOMING_OFFSET = 20
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_CURRENT_UNUSED = 102
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_HW_ERROR = 70
POSITION_MODE = 3


def open_port(baud=BAUD):
    port = PortHandler(DEVICE)
    packet = PacketHandler(2.0)
    if not port.openPort():
        raise SystemExit(f"cannot open {DEVICE} (U2D2 attached, 12 V on, in 'dialout'?)")
    port.setBaudRate(baud)
    return port, packet


def to_signed32(v):
    return v - 0x100000000 if v >= 0x80000000 else v


def cmd_scan(_):
    port, packet = open_port()
    found = []
    for baud in (57600, 1000000, 115200, 2000000, 3000000, 9600):
        if not port.setBaudRate(baud):
            continue
        for i in range(0, 21):
            model, comm, err = packet.ping(port, i)
            if comm == 0 and err == 0:
                print(f"  FOUND id={i} model={model} baud={baud}")
                found.append(i)
    port.closePort()
    print(f"\n{len(found)} motor(s) found." if found else "No motors found. Check wiring/power/attach.")


def cmd_read(_):
    port, packet = open_port()
    for i in IDS:
        pos, c, _ = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)
        goal, _, _ = packet.read4ByteTxRx(port, i, ADDR_GOAL_POSITION)
        tq, _, _ = packet.read1ByteTxRx(port, i, ADDR_TORQUE_ENABLE)
        hw, _, _ = packet.read1ByteTxRx(port, i, ADDR_HW_ERROR)
        if c == 0:
            print(f"id{i} {NAMES[i]:<8} pos={pos:>4} goal={goal:>4} torque={tq} hw_err={hw}")
        else:
            print(f"id{i} {NAMES[i]:<8} NO RESPONSE (comm={c})")
    port.closePort()


def cmd_zero(_):
    port, packet = open_port()
    for i in IDS:
        packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)       # off to change mode
        packet.write1ByteTxRx(port, i, ADDR_OPERATING_MODE, POSITION_MODE)
        packet.write4ByteTxRx(port, i, ADDR_PROFILE_VELOCITY, 50)   # slow
        packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 1)       # torque ON before goal (no jump)
        packet.write4ByteTxRx(port, i, ADDR_GOAL_POSITION, CENTER)
    time.sleep(2.0)
    print("Centered and HOLDING (torque stays on while 12 V is up). Bolt parts on now.")
    for i in IDS:
        pos, _, _ = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)
        print(f"  id{i} {NAMES[i]}: {pos}")
    port.closePort()


def cmd_home_offset(args):
    port, packet = open_port()
    i, target = args.id, args.target
    packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)           # EEPROM write needs torque off
    old, c, _ = packet.read4ByteTxRx(port, i, ADDR_HOMING_OFFSET)
    if c != 0:
        raise SystemExit(f"id{i}: no response")
    raw = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)[0] - to_signed32(old)
    new = target - raw
    packet.write4ByteTxRx(port, i, ADDR_HOMING_OFFSET, new & 0xFFFFFFFF)
    after = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)[0]
    print(f"id{i}: homing_offset={new}, pose now reads {after} (target {target}). Saved to EEPROM.")
    port.closePort()


def cmd_find_limits(_):
    port, packet = open_port()
    for i in IDS:
        packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)       # free to move by hand
    mins = {i: None for i in IDS}
    maxs = {i: None for i in IDS}
    print("Torque OFF. Sweep each joint slowly to both limits. Ctrl+C to finish.\n", flush=True)
    try:
        last = 0.0
        while True:
            for i in IDS:
                v, c, e = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)
                if c == 0 and e == 0:
                    mins[i] = v if mins[i] is None else min(mins[i], v)
                    maxs[i] = v if maxs[i] is None else max(maxs[i], v)
            if time.time() - last >= 1.0:
                last = time.time()
                print("  " + "  ".join(f"{NAMES[i]} {mins[i]}..{maxs[i]}" for i in IDS), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        port.closePort()
        print("\nFINAL (put these, with a small margin, into config/arm_params.yaml):")
        print("  " + "  ".join(f"{NAMES[i]} {mins[i]}..{maxs[i]}" for i in IDS))


def _load_config_limits():
    import yaml
    with open(CONFIG) as f:
        p = yaml.safe_load(f)["arm_node"]["ros__parameters"]
    return p["ids"], p["lo"], p["hi"]


def cmd_set_limits(_):
    ids, lo, hi = _load_config_limits()
    port, packet = open_port()
    for n, i in enumerate(ids):
        packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)       # EEPROM write needs torque off
        packet.write4ByteTxRx(port, i, ADDR_MIN_POSITION_LIMIT, lo[n])
        packet.write4ByteTxRx(port, i, ADDR_MAX_POSITION_LIMIT, hi[n])
        rlo = packet.read4ByteTxRx(port, i, ADDR_MIN_POSITION_LIMIT)[0]
        rhi = packet.read4ByteTxRx(port, i, ADDR_MAX_POSITION_LIMIT)[0]
        print(f"id{i} {NAMES.get(i,'')}: min={rlo} max={rhi}")
    port.closePort()
    print("Firmware now clamps to these ranges (the hard backstop).")


def cmd_teach(_):
    """Lead-through teaching: torque off, pose the arm by hand, record each pose.
    Prints ready-to-paste lines for the POSES list in demo_node.py."""
    # These three mirror kinematics.CALIB for the gripper; keep them in sync.
    GRIP_LO, GRIP_HI, GRIP_SCALE = -0.20, 0.425, 0.00158

    port, packet = open_port()
    for i in IDS:
        packet.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)   # free to move by hand
    print("Torque OFF - support the arm! Pose it, then type a name and press Enter")
    print("to record. Press Enter on an empty line to finish.\n")

    lines = []
    try:
        while True:
            name = input("pose name (blank = done): ").strip()
            if not name:
                break
            ticks = {}
            for i in IDS:
                v, c, e = packet.read4ByteTxRx(port, i, ADDR_PRESENT_POSITION)
                if c != 0 or e != 0:
                    print(f"  id{i} read failed, try again"); break
                ticks[i] = v
            else:
                e_deg = (ticks[3] - CENTER) * 360.0 / 4096.0          # elbow
                w_deg = (ticks[2] - CENTER) * 360.0 / 4096.0          # wrist
                g_rad = (ticks[1] - CENTER) * GRIP_SCALE              # gripper
                grip = (g_rad - GRIP_LO) / (GRIP_HI - GRIP_LO)        # -> openness 0..1
                line = f'    ("{name}", {e_deg:6.1f}, {w_deg:6.1f}, {max(0.0, min(1.0, grip)):.2f}),'
                lines.append(line)
                print("  recorded:" + line)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        port.closePort()

    if lines:
        print("\nPaste these into POSES in ros2_ws/src/dyn_arm/dyn_arm/demo_node.py:\n")
        print("\n".join(lines))
        print("\nThen: cd ros2_ws && colcon build && relaunch demo.launch.py")


def main():
    ap = argparse.ArgumentParser(description="Direct-to-motor utilities for the arm.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan", help="find motors across baud rates").set_defaults(fn=cmd_scan)
    sub.add_parser("read", help="print pos/goal/torque/error per motor").set_defaults(fn=cmd_read)
    sub.add_parser("zero", help="center all motors (2048) and hold").set_defaults(fn=cmd_zero)
    ho = sub.add_parser("home-offset", help="calibrate a joint zero in firmware")
    ho.add_argument("id", type=int)
    ho.add_argument("target", type=int, nargs="?", default=2048)
    ho.set_defaults(fn=cmd_home_offset)
    sub.add_parser("find-limits", help="back-drive by hand, capture min/max ticks").set_defaults(fn=cmd_find_limits)
    sub.add_parser("teach", help="lead-through teaching: pose by hand, record demo poses").set_defaults(fn=cmd_teach)
    sub.add_parser("set-limits", help="burn config lo/hi into motor EEPROM").set_defaults(fn=cmd_set_limits)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
