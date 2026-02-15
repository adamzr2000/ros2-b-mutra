#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DummySubscriber(Node):
    def __init__(self):
        super().__init__('dummy_subscriber')

        # Declare the parameter for the topic name
        self.declare_parameter('topic_name', 'chatter')

        # Get the topic name from the parameter
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Create the subscription
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = DummySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
