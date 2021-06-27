#!/usr/bin/env python
# BEGIN ALL


# import rospy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys

# BEGIN CALLBACK
def callback(msg):
    global node

    print(msg.data)
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg.data)
    node.get_logger().info('I heard: {}'.format(msg.data))
# END CALLBACK

def main():
    # Don't know correct way to get the node into the callback, so made it a global
    global node
    # rospy.init_node('topic_subscriber')
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('topic_subscriber')


    # BEGIN SUBSCRIBER
    # sub = rospy.Subscriber('counter', Int32, callback)
    sub = node.create_subscription(Int32, 'counter', callback, qos_profile=1)
    # END SUBSCRIBER

    # rospy.spin()
    rclpy.spin(node)
# END ALL

if __name__ == '__main__':
    main()
