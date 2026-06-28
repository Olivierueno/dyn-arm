"""twin_echo_node: drive the RViz twin from the REAL arm's feedback.

When the demo commands the motors directly (one goal per pose) the smoothing happens
inside the motors, so there is no streamed trajectory for the twin to follow. This
node closes that gap by echoing the arm's measured state to RViz:

  arm_node --/present_positions (ticks)--> twin_echo_node --/joint_states (rad)--> robot_state_publisher --> RViz

So the twin mirrors what the arm is actually doing, not what it was told to do.
Same tick<->radian calibration as everything else (kinematics.CALIB).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

from dyn_arm.kinematics import CALIB, ticks_to_rad


class TwinEcho(Node):
    def __init__(self):
        super().__init__("twin_echo")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Int32MultiArray, "present_positions", self.on_present, 10)
        self.get_logger().info("twin_echo: /present_positions (ticks) -> /joint_states (rad)")

    def on_present(self, msg):
        if not msg.data or -1 in msg.data:   # a read failed this cycle; skip rather than show a glitch
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for name, c in CALIB.items():
            js.name.append(name)
            js.position.append(ticks_to_rad(name, msg.data[c["idx"]]))
        self.pub.publish(js)


def main():
    rclpy.init()
    node = TwinEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
