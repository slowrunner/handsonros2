#!/usr/bin/env python3

# FILE: topic_sub_node.py

# Pythonic Migration of Chapter3_ROS_basics/topic_subscriber.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class TopicSubscriberNode(Node):

    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('counter topic_subscriber node created')

    def listener_callback(self, msg):
        self.get_logger().info('I heard {}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)

    topic_sub_node = TopicSubscriberNode()

    try:
      rclpy.spin(topic_sub_node)
    except KeyboardInterrupt:
      topic_sub_node.get_logger().info('control-c: shutting down')
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      topic_sub_node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
