from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from linker_hand_ros2_sdk.LinkerHand.utils.mapping import l20_l_min, l20_l_max, l20_l_derict, l20_r_min, l20_r_max, l20_r_derict 


class JointAngleEncoder(Node):
    """Convert radian-based hand references to byte-oriented control commands."""

    def __init__(self) -> None:
        super().__init__('joint_angle_encoder')
        self.get_logger().info('JointAngleEncoder node started.')

        self.enable_right_hand = self.declare_parameter('enable_right_hand', True).value
        self.enable_left_hand = self.declare_parameter('enable_left_hand', True).value

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        self.right_ref_sub = None
        self.left_ref_sub = None
        self.right_cmd_pub = None
        self.left_cmd_pub = None

        if self.enable_right_hand:
            self.right_ref_sub = self.create_subscription(
                JointState,
                '/linkerhand_right/reference',
                self._handle_right_ref,
                best_effort_qos,
            )
            self.right_cmd_pub = self.create_publisher(
                JointState,
                '/cb_right_hand_control_cmd',
                best_effort_qos,
            )
            self.get_logger().info('Right hand interface enabled.')
        else:
            self.get_logger().info('Right hand interface disabled by parameter.')

        if self.enable_left_hand:
            self.left_ref_sub = self.create_subscription(
                JointState,
                '/linkerhand_left/reference',
                self._handle_left_ref,
                best_effort_qos,
            )
            self.left_cmd_pub = self.create_publisher(
                JointState,
                '/cb_left_hand_control_cmd',
                best_effort_qos,
            )
            self.get_logger().info('Left hand interface enabled.')
        else:
            self.get_logger().info('Left hand interface disabled by parameter.')

    def _handle_right_ref(self, msg: JointState) -> None:
        if not self.enable_right_hand or self.right_cmd_pub is None:
            return
        cmd = self._build_command(msg, "right")
        self.right_cmd_pub.publish(cmd)

    def _handle_left_ref(self, msg: JointState) -> None:
        if not self.enable_left_hand or self.left_cmd_pub is None:
            return
        cmd = self._build_command(msg, "left")
        self.left_cmd_pub.publish(cmd)

    def _build_command(self, ref: JointState, hand_type: str) -> JointState:
        byte_cmd = JointState()
        byte_cmd.header = ref.header
        byte_cmd.position = [0.0] * len(ref.position)
        byte_cmd.velocity = []
        byte_cmd.effort = []

        if hand_type == "left":
            angle_min = l20_l_min
            angle_max = l20_l_max
            derict = l20_l_derict
        elif hand_type == "right":
            angle_min = l20_r_min
            angle_max = l20_r_max
            derict = l20_r_derict
        else:
            raise ValueError(f'Unsupported hand_type: {hand_type}')

        for i in range(len(ref.position)):
            if 11 <= i <= 14:
                continue
            byte_val = self._is_within_range(ref.position[i], angle_min[i], angle_max[i])
            if derict[i] == -1:
                byte_cmd.position[i] = self._scale_value(byte_val, angle_min[i], angle_max[i], 255, 0)
            else:
                byte_cmd.position[i] = self._scale_value(byte_val, angle_min[i], angle_max[i], 0, 255)

        return byte_cmd
    
    def _is_within_range(self, value, min_value, max_value):
        return min(max_value, max(min_value, value))

    def _scale_value(self, original_value, a_min, a_max, b_min, b_max):
        return (original_value - a_min) * (b_max - b_min) / (a_max - a_min) + b_min


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JointAngleEncoder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
