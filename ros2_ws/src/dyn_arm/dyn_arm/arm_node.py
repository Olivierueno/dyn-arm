"""arm_node: the driver for the whole 3-motor bus.

Owns /dev/ttyUSB0 and exposes:
  sub: /goal_positions     std_msgs/Int32MultiArray [gripper, wrist, elbow] ticks (0..4095)
  pub: /present_positions  std_msgs/Int32MultiArray [gripper, wrist, elbow] ticks @ 20 Hz

Safety, in layers (each catches what the one before cannot):
  1. Per-joint soft limits  - clamp every goal to that joint's safe tick range.
  2. Virtual floor (FK)      - reject any elbow+wrist combo that would drive the tip
                               through the table; hold the last safe pose instead.
  3. Current-limited holding - mode 5 caps torque, so contact is a soft stall that still
                               holds the arm up (never goes limp, never crushes).
  4. Watchdog                - if targets stop arriving, ease back to the home pose.

Every tunable lives in config/arm_params.yaml (loaded by the launch file), so the
behaviour can be changed without editing this code.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

from dyn_arm.kinematics import tip_height

# --- X-series control table (same for XM430 and XM540) ---
ADDR_OPERATING_MODE       = 11    # 3 = position, 5 = current-based position
ADDR_TORQUE_ENABLE        = 64
ADDR_POSITION_D_GAIN      = 80    # damping (quiets idle hunting)
ADDR_POSITION_P_GAIN      = 84    # stiffness
ADDR_GOAL_CURRENT         = 102   # torque cap in mode 5 (units ~2.69 mA)
ADDR_PROFILE_ACCELERATION = 108   # accel ramp (0 = instant jerk)
ADDR_PROFILE_VELOCITY     = 112   # speed cap
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132
ADDR_PRESENT_CURRENT      = 126   # signed; load/torque proxy (units ~2.69 mA)
ADDR_PRESENT_TEMPERATURE  = 146   # deg C
CURRENT_POSITION_MODE     = 5
CURRENT_UNIT_MA           = 2.69  # mA per Present-Current count


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")

        # ---- parameters (defaults here; overridden by config/arm_params.yaml) ----
        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baud", 57600)
        self.declare_parameter("profile_velocity", 50)
        self.declare_parameter("profile_acceleration", 10)
        self.declare_parameter("watchdog_timeout", 1.5)
        # floor geometry (meters / ticks)
        self.declare_parameter("elbow_axis_height", 0.04475)
        self.declare_parameter("link1", 0.140)
        self.declare_parameter("link2", 0.192)
        self.declare_parameter("floor_margin", 0.015)
        self.declare_parameter("zero_tick", 2048)
        # per-joint arrays, all in order [gripper, wrist, elbow]
        self.declare_parameter("ids", [1, 2, 3])
        self.declare_parameter("names", ["gripper", "wrist", "elbow"])
        self.declare_parameter("lo", [1903, 456, 920])
        self.declare_parameter("hi", [2316, 3639, 3174])
        self.declare_parameter("goal_current", [200, 300, 500])
        self.declare_parameter("p_gain", [400, 800, 800])
        self.declare_parameter("d_gain", [4500, 1200, 1200])
        self.declare_parameter("deadband", [12, 8, 8])
        self.declare_parameter("home", [2048, 2048, 2048])

        gp = lambda n: self.get_parameter(n).value
        self.device = gp("device")
        self.profile_velocity = gp("profile_velocity")
        self.profile_acceleration = gp("profile_acceleration")
        self.watchdog_timeout = gp("watchdog_timeout")
        self.floor_margin = gp("floor_margin")
        self.geom = dict(elbow_axis_h=gp("elbow_axis_height"), link1=gp("link1"),
                         link2=gp("link2"), zero_tick=gp("zero_tick"))

        ids, names = gp("ids"), gp("names")
        lo, hi, gc = gp("lo"), gp("hi"), gp("goal_current")
        pg, dg, db = gp("p_gain"), gp("d_gain"), gp("deadband")
        self.motors = [dict(id=ids[i], name=names[i], lo=lo[i], hi=hi[i],
                            goal_current=gc[i], p_gain=pg[i], d_gain=dg[i], deadband=db[i])
                       for i in range(len(ids))]
        self.home = {ids[i]: gp("home")[i] for i in range(len(ids))}
        self.elbow_id = ids[names.index("elbow")]
        self.wrist_id = ids[names.index("wrist")]

        # ---- open the bus and set every motor up ----
        self.port = PortHandler(self.device)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f"Failed to open {self.device}")
        self.port.setBaudRate(gp("baud"))
        for m in self.motors:
            self._setup_motor(m)

        # ---- runtime state ----
        self.safe_elbow, _, _ = self.packet.read4ByteTxRx(self.port, self.elbow_id, ADDR_PRESENT_POSITION)
        self.safe_wrist, _, _ = self.packet.read4ByteTxRx(self.port, self.wrist_id, ADDR_PRESENT_POSITION)
        self.written = {m["id"]: None for m in self.motors}
        self.last_goal_time = time.time()
        self.homed = False

        self.pub = self.create_publisher(Int32MultiArray, "present_positions", 10)
        self.create_subscription(Int32MultiArray, "goal_positions", self.on_goal, 10)
        self.create_timer(0.05, self.on_timer)    # 20 Hz state feedback + watchdog
        self.create_timer(0.33, self.on_status)   # ~3 Hz load + temperature telemetry
        names_str = ", ".join(f'{m["name"]}(id{m["id"]})' for m in self.motors)
        self.get_logger().info(f"arm_node ready. Motors: {names_str}")

    def _setup_motor(self, m):
        i = m["id"]
        self.packet.write1ByteTxRx(self.port, i, ADDR_TORQUE_ENABLE, 0)             # off to change mode
        self.packet.write1ByteTxRx(self.port, i, ADDR_OPERATING_MODE, CURRENT_POSITION_MODE)
        self.packet.write2ByteTxRx(self.port, i, ADDR_POSITION_P_GAIN, m["p_gain"])
        self.packet.write2ByteTxRx(self.port, i, ADDR_POSITION_D_GAIN, m["d_gain"])
        self.packet.write4ByteTxRx(self.port, i, ADDR_PROFILE_ACCELERATION, self.profile_acceleration)
        self.packet.write4ByteTxRx(self.port, i, ADDR_PROFILE_VELOCITY, self.profile_velocity)
        present, comm, err = self.packet.read4ByteTxRx(self.port, i, ADDR_PRESENT_POSITION)
        if comm == 0 and err == 0:
            self.packet.write4ByteTxRx(self.port, i, ADDR_GOAL_POSITION, present)   # no-jump: goal=present
        self.packet.write1ByteTxRx(self.port, i, ADDR_TORQUE_ENABLE, 1)
        self.packet.write2ByteTxRx(self.port, i, ADDR_GOAL_CURRENT, m["goal_current"])
        self.get_logger().info(
            f"id {i}: current-pos mode, cap={m['goal_current']}, torque on, holding {present}")

    def on_goal(self, msg):
        self.last_goal_time = time.time()   # fresh target: pet the watchdog
        self.homed = False

        # 1) per-joint soft-limit clamp
        goals = {}
        for m, pos in zip(self.motors, msg.data):
            goals[m["id"]] = max(m["lo"], min(m["hi"], int(pos)))

        # 2) virtual floor: refuse elbow+wrist combos that would hit the table
        if tip_height(goals[self.elbow_id], goals[self.wrist_id], **self.geom) < self.floor_margin:
            goals[self.elbow_id] = self.safe_elbow
            goals[self.wrist_id] = self.safe_wrist
            self.get_logger().warn("virtual floor: goal would hit the table, holding",
                                   throttle_duration_sec=1.0)
        else:
            self.safe_elbow, self.safe_wrist = goals[self.elbow_id], goals[self.wrist_id]

        # 3) deadband: ignore sub-threshold goal wobble (command noise)
        for m in self.motors:
            g = goals[m["id"]]
            prev = self.written[m["id"]]
            if prev is None or abs(g - prev) >= m["deadband"]:
                self.packet.write4ByteTxRx(self.port, m["id"], ADDR_GOAL_POSITION, g)
                self.written[m["id"]] = g

    def on_timer(self):
        # watchdog: no fresh target for a while -> ease home once
        if not self.homed and time.time() - self.last_goal_time > self.watchdog_timeout:
            self.homed = True
            self.get_logger().warn("watchdog: no targets, easing to home pose")
            for m in self.motors:
                self.packet.write4ByteTxRx(self.port, m["id"], ADDR_GOAL_POSITION, self.home[m["id"]])
                self.written[m["id"]] = self.home[m["id"]]
            self.safe_elbow, self.safe_wrist = self.home[self.elbow_id], self.home[self.wrist_id]

        out = []
        for m in self.motors:
            val, comm, err = self.packet.read4ByteTxRx(self.port, m["id"], ADDR_PRESENT_POSITION)
            out.append(int(val) if comm == 0 and err == 0 else -1)
        self.pub.publish(Int32MultiArray(data=out))

    def on_status(self):
        # Load (present current, signed mA) + temperature per motor. Current is a direct
        # proxy for torque/strain; % is against that joint's goal_current cap.
        parts = []
        for m in self.motors:
            raw, c, e = self.packet.read2ByteTxRx(self.port, m["id"], ADDR_PRESENT_CURRENT)
            signed = (raw - 65536 if raw >= 32768 else raw) if (c == 0 and e == 0) else 0
            ma = abs(int(signed * CURRENT_UNIT_MA))
            temp, tc, te = self.packet.read1ByteTxRx(self.port, m["id"], ADDR_PRESENT_TEMPERATURE)
            pct = int(100 * ma / (m["goal_current"] * CURRENT_UNIT_MA))
            parts.append(f"{m['name']} {ma:>4}mA {pct:>3}% {temp}C")
        self.get_logger().info("load: " + " | ".join(parts), throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for m in node.motors:
            node.packet.write1ByteTxRx(node.port, m["id"], ADDR_TORQUE_ENABLE, 0)
        node.port.closePort()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
