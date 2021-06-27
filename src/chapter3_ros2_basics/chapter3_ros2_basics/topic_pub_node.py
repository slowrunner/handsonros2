#!/bin/env python3

# File: topic_pub_node.py

# Pythonic Migration of Chapter3_ROS_basics/topic_publisher.py

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class TopicPublisherNode(Node):

    def __init__(self):
        super().__init__('topic_publisher')
        self.pub = self.create_publisher(Int32, 'counter', qos_profile=10)
        timer_period = 0.5  # 0.5 seconds = 2Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info('Created topic_publisher node')

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.pub.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    topic_pub_node = TopicPublisherNode()

    try:
      rclpy.spin(topic_pub_node)
    except KeyboardInterrupt:
      print('\ncontrol-c: topic_pub_node shutting down')
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      topic_pub_node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
