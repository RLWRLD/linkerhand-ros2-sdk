import math
import struct
import time
from typing import List

import rclpy
from rclpy.node import Node

try:
    import zmq
except ImportError:  # pragma: no cover - optional dependency at runtime
    zmq = None


class TestZmqPublisher(Node):
    """Publish dummy joint references over ZMQ for integration testing."""

    def __init__(self) -> None:
        super().__init__("test_zmq_publisher")

        if zmq is None:
            raise RuntimeError(
                "pyzmq is required for the ZMQ reference publisher node."
            )

        endpoint = self.declare_parameter(
            "zmq_endpoint", "ipc:///tmp/linkerhand_reference"
        ).value
        publish_interval = float(
            self.declare_parameter("publish_interval_sec", 0.01).value
        )
        self._publish_format = self.declare_parameter(
            "payload_format", "binary"
        ).value.lower()

        # TODO: support json and text format
        if self._publish_format not in {"binary"}:
            self.get_logger().warning(
                f"Unsupported payload_format {self._publish_format}. Please set to 'binary'.",
            )
            raise NotImplementedError()

        self._zmq_context = zmq.Context()
        self._zmq_socket = self._zmq_context.socket(zmq.PUB)
        self._zmq_socket.setsockopt(zmq.SNDHWM, 10)
        self._zmq_socket.setsockopt(zmq.LINGER, 0)
        self._zmq_socket.setsockopt(zmq.IMMEDIATE, 1)
        self._zmq_socket.bind(endpoint)
        time.sleep(0.1)

        self.get_logger().info(
            f"Publishing ZMQ joint references on {endpoint} with {self._publish_format} payloads."
        )

        self._timer = self.create_timer(0.01, self._publish)
        self._cnt = 0.0

    def _publish(self) -> None:
        base_value = math.sin(self._cnt / 30) * 0.5 + 0.5
        print(f"{base_value:3f}", flush=True)
        self._cnt += 1
        right_positions = self._make_positions(base_value)
        left_positions = self._make_positions(base_value)
        # print(f"{base_value:.3f}", flush=True)
        for i in range(len(left_positions)):
            if i != 1:
                left_positions[i] = 0.0

        self._send_frame("right", right_positions)
        self._send_frame("left", left_positions)

    def _make_positions(self, offset: float) -> List[float]:
        return [offset + 0.01 * index for index in range(20)]

    def _send_frame(self, hand: str, data: List[float]) -> None:
        payload = struct.pack("<20f", *data)

        try:
            self._zmq_socket.send_multipart(
                [hand.encode("utf-8"), payload], flags=zmq.NOBLOCK
            )
        except zmq.Again:
            self.get_logger().warning(f"ZMQ publisher backpressure for {hand} hand.")

    def destroy_node(self) -> bool:
        if self._timer is not None:
            self._timer.cancel()

        if zmq is not None:
            if getattr(self, "_zmq_socket", None) is not None:
                self._zmq_socket.close(0)
            if getattr(self, "_zmq_context", None) is not None:
                self._zmq_context.term()

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TestZmqPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
