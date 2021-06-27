#!/usr/bin/env python

# FILE: chapter3_ros2_basics/doubler.py

# Pythonic Migration of Chapter3_ROS_basics/doubler.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class DoublerNode(Node):

  def __init__(self):
    super().__init__('doubler')
    self.pub = self.create_publisher(Int32, 'doubled', qos_profile=10)
    self.sub = self.create_subscription(
      Int32,
      'number',
      self.sub_callback,
      10)
    self.sub  # prevent unused var warning
    self.get_logger().info('number topic subscriber created')
    self.get_logger().info('doubled topic publisher created')


  def sub_callback(self,number_msg):
    doubled_msg = Int32()
    doubled_msg.data = number_msg.data * 2
    self.pub.publish(doubled_msg)


def main(args=None):

  rclpy.init(args=args)
  doubler_node = DoublerNode()
  try:
    rclpy.spin(doubler_node)
  except KeyboardInterrupt:
    print('\ncontrol-c: doubler_node shutting down')
  finally:
    doubler_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
  main()


