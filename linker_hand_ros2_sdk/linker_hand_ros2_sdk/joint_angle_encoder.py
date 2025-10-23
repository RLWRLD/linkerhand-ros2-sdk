import json
import struct
from typing import Iterable, List, Optional

try:
    import zmq
except:
    pass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from linker_hand_ros2_sdk.LinkerHand.utils.mapping import (
    l20_l_min,
    l20_l_max,
    l20_l_derict,
    l20_r_min,
    l20_r_max,
    l20_r_derict,
)


class JointAngleEncoder(Node):
    """Convert radian-based hand references to byte-oriented control commands."""

    def __init__(self) -> None:
        super().__init__("joint_angle_encoder")
        self.get_logger().info("JointAngleEncoder node started.")

        self._enable_right_hand = self.declare_parameter("enable_right_hand", True).value
        self._enable_left_hand = self.declare_parameter("enable_left_hand", True).value
        self._source = self.declare_parameter("source", "ros").value.lower()

        if self._source not in {"ros", "zmq"}:
            self.get_logger().warning(
                "Unsupported source '%s'. Falling back to ROS 2 subscription.",
                self._source,
            )
            self._source = "ros"

        if self._enable_right_hand:
            self._right_cmd_pub = self.create_publisher(
                JointState,
                "/cb_right_hand_control_cmd",
                10,
            )
        if self._enable_left_hand:
            self._left_cmd_pub = self.create_publisher(
                JointState,
                "/cb_left_hand_control_cmd",
                10,
            )

        if self._source == "ros":
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                depth=1,
            )
            if self._enable_right_hand:
                self._right_ref_sub = self.create_subscription(
                    JointState,
                    "/linkerhand_right/reference",
                    self._handle_right_ref,
                    qos,
                )
            if self._enable_left_hand:
                self._left_ref_sub = self.create_subscription(
                    JointState,
                    "/linkerhand_left/reference",
                    self._handle_left_ref,
                    qos,
                )
            self.get_logger().info("Listening for joint references via ROS 2 topics.")
        elif self._source == "zmq":
            self._setup_zmq_interface()

    def _handle_right_ref(self, msg: JointState) -> None:
        if not self._enable_right_hand or self._right_cmd_pub is None:
            return
        cmd = self._build_command(msg, "right")
        self._right_cmd_pub.publish(cmd)

    def _handle_left_ref(self, msg: JointState) -> None:
        if not self._enable_left_hand or self._left_cmd_pub is None:
            return
        cmd = self._build_command(msg, "left")
        self._left_cmd_pub.publish(cmd)

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
            raise ValueError(f"Unsupported hand_type: {hand_type}")

        for i in range(len(ref.position)):
            if 11 <= i <= 14:
                continue
            byte_val = self._is_within_range(
                ref.position[i], angle_min[i], angle_max[i]
            )
            if derict[i] == -1:
                byte_cmd.position[i] = self._scale_value(
                    byte_val, angle_min[i], angle_max[i], 255, 0
                )
            else:
                byte_cmd.position[i] = self._scale_value(
                    byte_val, angle_min[i], angle_max[i], 0, 255
                )

        return byte_cmd

    def _is_within_range(self, value, min_value, max_value):
        return min(max_value, max(min_value, value))

    def _scale_value(self, original_value, a_min, a_max, b_min, b_max):
        return (original_value - a_min) * (b_max - b_min) / (a_max - a_min) + b_min

    def _setup_zmq_interface(self) -> None:
        self._zmq_endpoint = self.declare_parameter(
            "zmq_endpoint", "ipc:///tmp/linkerhand_reference"
        ).value
        poll_interval_param = self.declare_parameter("zmq_poll_interval_sec", 0.01).value

        try:
            poll_interval = float(poll_interval_param)
        except (TypeError, ValueError):
            poll_interval = 0.01
        if poll_interval <= 0:
            poll_interval = 0.01

        self._zmq_context = zmq.Context()
        self._zmq_sock = self._zmq_context.socket(zmq.SUB)
        self._zmq_sock.setsockopt_string(zmq.SUBSCRIBE, "left")
        self._zmq_sock.setsockopt_string(zmq.SUBSCRIBE, "right")
        self._zmq_sock.connect(self._zmq_endpoint)

        self._zmq_poller = zmq.Poller()
        self._zmq_poller.register(self._zmq_sock, zmq.POLLIN)
        self._zmq_timer = self.create_timer(poll_interval, self._poll_zmq)

        self.get_logger().info(
                f"Listening for joint references via ZMQ SUB on {self._zmq_endpoint}")

    def _poll_zmq(self) -> None:
        if self._zmq_sock is None or self._zmq_poller is None:
            return

        try:
            events = dict(self._zmq_poller.poll(0))
        except zmq.ZMQError as exc:
            self.get_logger().error("ZMQ poll error: %s", exc)
            return

        if self._zmq_sock not in events:
            return

        try:
            frames = self._zmq_sock.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        except zmq.ZMQError as exc:
            self.get_logger().error("Failed to receive ZMQ message: %s", exc)
            return

        if not frames:
            self.get_logger().warning("ZMQ message was empty.")
            return

        try:
            hand = frames[0].decode("utf-8").strip()
        except UnicodeDecodeError:
            self.get_logger().warning("ZMQ message topic is not UTF-8 encoded.")
            return

        if hand not in {"left", "right"}:
            self.get_logger().warning("Unsupported ZMQ topic '%s'.", hand)
            return

        if len(frames) < 2:
            self.get_logger().warning(
                "ZMQ joint reference on %s is missing payload frame.", hand
            )
            return

        payload_bytes = frames[1]
        positions: Optional[List[float]] = None

        if len(payload_bytes) % 4 == 0 and payload_bytes:
            count = len(payload_bytes) // 4
            try:
                positions = list(struct.unpack(f"<{count}f", payload_bytes))
            except struct.error:
                positions = None

        if positions is None:
            try:
                payload_text = payload_bytes.decode("utf-8").strip()
            except UnicodeDecodeError:
                payload_text = ""

            if payload_text:
                try:
                    decoded = json.loads(payload_text)
                except json.JSONDecodeError:
                    decoded = None

                if isinstance(decoded, list):
                    try:
                        positions = [float(value) for value in decoded]
                    except (TypeError, ValueError):
                        positions = None

                if positions is None:
                    try:
                        positions = [float(item) for item in payload_text.split()]
                    except ValueError:
                        positions = None

        if positions is None:
            self.get_logger().warning(
                "Unable to parse ZMQ joint reference payload for %s hand.", hand
            )
            return

        expected_len = len(l20_l_min) if hand == "left" else len(l20_r_min)
        if len(positions) != expected_len:
            self.get_logger().warning(
                f"ZMQ joint reference for {hand} hand expected {expected_len} positions but received {len(positions)}."
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = positions

        if hand == "left":
            self._handle_left_ref(msg)
        else:
            self._handle_right_ref(msg)

    def destroy_node(self) -> bool:
        if self._source == "zmq":
            if self._zmq_timer is not None:
                self._zmq_timer.cancel()
            if self._zmq_poller is not None and self._zmq_sock is not None:
                try:
                    self._zmq_poller.unregister(self._zmq_sock)
                except KeyError:
                    pass
            if self._zmq_sock is not None:
                self._zmq_sock.close(0)
            if self._zmq_context is not None:
                self._zmq_context.term()

        return super().destroy_node()


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JointAngleEncoder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
