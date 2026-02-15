#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')

        # Declare parameters
        self.declare_parameter('topic_name', 'chatter')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('message', 'Hello from dummy_publisher')

        # Get parameters
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.message = self.get_parameter('message').get_parameter_value().string_value

        # Set up publisher and timer
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = self.message
        self.get_logger().info(f"Publishing: {msg.data}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
