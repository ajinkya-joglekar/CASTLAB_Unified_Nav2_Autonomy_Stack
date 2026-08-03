"""Small typed topic relay used for hardware topic adaptation.

The base stack keeps Nav2/localization-facing topics canonical.  When a
hardware platform publishes or consumes platform-specific topic names, this
node relays between the platform topic and the canonical base-stack topic
using the message type selected by the launch file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def _reliability(value: str) -> ReliabilityPolicy:
    normalized = value.strip().lower()
    if normalized in ("best_effort", "besteffort", "best-effort"):
        return ReliabilityPolicy.BEST_EFFORT
    if normalized in ("reliable", ""):
        return ReliabilityPolicy.RELIABLE
    raise ValueError(f"unsupported reliability '{value}'")


class TopicRelay(Node):
    def __init__(self):
        super().__init__("topic_relay")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("message_type", "")
        self.declare_parameter("input_reliability", "best_effort")
        self.declare_parameter("output_reliability", "reliable")
        self.declare_parameter("depth", 10)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        message_type_name = self.get_parameter("message_type").value
        depth = int(self.get_parameter("depth").value)

        if not input_topic or not output_topic or not message_type_name:
            raise ValueError("input_topic, output_topic, and message_type are required")

        message_type = get_message(message_type_name)
        input_qos = QoSProfile(
            depth=depth,
            reliability=_reliability(self.get_parameter("input_reliability").value),
        )
        output_qos = QoSProfile(
            depth=depth,
            reliability=_reliability(self.get_parameter("output_reliability").value),
        )

        self.publisher = self.create_publisher(message_type, output_topic, output_qos)
        self.subscription = self.create_subscription(
            message_type,
            input_topic,
            self.publisher.publish,
            input_qos,
        )
        self.get_logger().info(
            f"Relaying {message_type_name}: {input_topic} -> {output_topic}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
