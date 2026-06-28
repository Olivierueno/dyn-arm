"""joint_command_node: /joint_states (radians) -> /goal_positions (ticks).

The standard joint_state_publisher_gui publishes joint angles in radians (the sliders).
This node converts them to motor ticks for arm_node, so dragging a slider moves the real
arm and the RViz twin together. It is the inverse of the tick->radian conversion that
feeds RViz, both share the calibration in kinematics.py.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

from dyn_arm.kinematics import CALIB, rad_to_ticks


class JointCommand(Node):
    def __init__(self):
        super().__init__("joint_command")
        self.pub = self.create_publisher(Int32MultiArray, "goal_positions", 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self.get_logger().info("joint_command: /joint_states (rad) -> /goal_positions (ticks)")

    def on_joint_states(self, msg):
        # default every joint to its home tick, then fill in whatever the message carries
        ticks = [CALIB[n]["zero"] for n in ("gripper", "wrist", "elbow")]
        for name, pos in zip(msg.name, msg.position):
            if name in CALIB:
                ticks[CALIB[name]["idx"]] = rad_to_ticks(name, pos)
        self.pub.publish(Int32MultiArray(data=ticks))


def main():
    rclpy.init()
    node = JointCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
