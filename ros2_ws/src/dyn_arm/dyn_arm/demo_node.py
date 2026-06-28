"""demo_node: hands-free choreography, the light way.

It is just predefined poses, so it sends each pose to /goal_positions ONCE as a
target and lets the Dynamixel's own trapezoidal motion profile (profile_velocity /
profile_acceleration in arm_params.yaml) do the accelerate-cruise-decelerate
smoothing in hardware. No streamed trajectory, no interpolation - so the bus stays
nearly idle and the heavy joints move smoothly under their own profile.

  demo_node --/goal_positions--> arm_node --> motors            (smooth via motor profile)
  arm_node  --/present_positions--> twin_echo_node --/joint_states--> RViz (twin follows the REAL arm)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from dyn_arm.kinematics import CALIB, rad_to_ticks

# Choreography: (name, elbow deg, wrist deg, gripper openness 0=closed..1=open).
# Angles are from the straight-up zero, same convention as kinematics.py.
POSES = [
    ("Start",    0.0, 0.2, 0.00),
    ("Collect",   20, 121, 0.00),
    ("Pinch",   20, 121, 1.00),
    ("Pivot",    0.7, -91.8, 1.00),
    ("Drop",  -50, -70, 1.00),
    ("Let Go",  -50, -70, 0.00),
]


class Demo(Node):
    def __init__(self):
        super().__init__("demo")

        # ---- tunables (defaults here; overridden by config/arm_params.yaml) ----
        self.declare_parameter("step_period", 3.0)   # seconds per pose (motor moves, then holds)
        self.declare_parameter("heartbeat", 5.0)     # Hz to re-send the held target (feeds the watchdog)
        self.declare_parameter("loop", True)         # repeat forever, or run once and hold the last pose
        gp = lambda n: self.get_parameter(n).value
        self.step_period = max(0.1, float(gp("step_period")))
        self.loop = bool(gp("loop"))

        # Resolve the choreography to motor-tick triples once: [gripper, wrist, elbow].
        self.seq = []
        for name, e_deg, w_deg, grip in POSES:
            ticks = [0, 0, 0]
            ticks[CALIB["elbow"]["idx"]] = rad_to_ticks("elbow", math.radians(e_deg))
            ticks[CALIB["wrist"]["idx"]] = rad_to_ticks("wrist", math.radians(w_deg))
            ticks[CALIB["gripper"]["idx"]] = rad_to_ticks("gripper", self._grip_rad(grip))
            self.seq.append((name, ticks))

        self.pub = self.create_publisher(Int32MultiArray, "goal_positions", 10)
        self.t0 = self.get_clock().now()
        self.idx = -1
        self.create_timer(1.0 / max(1.0, float(gp("heartbeat"))), self.on_timer)
        self.get_logger().info(
            f"demo: {len(self.seq)} poses, {self.step_period}s each, loop={self.loop} "
            f"(speed set by profile_velocity in the yaml)")

    @staticmethod
    def _grip_rad(openness):
        """Gripper openness 0..1 -> radians across its calibrated travel."""
        c = CALIB["gripper"]
        return c["lo"] + max(0.0, min(1.0, openness)) * (c["hi"] - c["lo"])

    def on_timer(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        i = int(elapsed // self.step_period)
        if i >= len(self.seq):
            i = i % len(self.seq) if self.loop else len(self.seq) - 1

        name, ticks = self.seq[i]
        if i != self.idx:
            self.idx = i
            self.get_logger().info(f"-> {name}")
        self.pub.publish(Int32MultiArray(data=ticks))   # heartbeat: same target = no motor write


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
