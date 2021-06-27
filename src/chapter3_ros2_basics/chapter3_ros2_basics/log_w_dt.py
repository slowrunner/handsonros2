#!/usr/bin/env python3

# File: topic_pub_node.py

# Pythonic Migration of Chapter3_ROS_basics/topic_publisher.py
# Demonstrates adding datetime to log strings
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
import datetime as dt

# add_dt(str) - prepends datetime "[YYYY-MM-DD HH:MM:SS]" to str
def add_dt(str):
  new_str = dt.datetime.now().strftime('[%Y-%m-%d %H:%M:%S] ') + str
  return new_str


class TopicPublisherNode(Node):

    def __init__(self):
        super().__init__('log_w_dt')
        self.pub = self.create_publisher(Int32, 'counter', qos_profile=10)
        timer_period = 0.5  # 0.5 seconds = 2Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info(add_dt('Created log_w_dt node'))

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.pub.publish(msg)
        self.get_logger().info(add_dt('Publishing: {}'.format(msg.data)))
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    topic_pub_node = TopicPublisherNode()

    try:
      rclpy.spin(topic_pub_node)
    except KeyboardInterrupt:
      topic_pub_node.get_logger().info(add_dt('control-c: log_w_dt shutting down'))
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      topic_pub_node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
