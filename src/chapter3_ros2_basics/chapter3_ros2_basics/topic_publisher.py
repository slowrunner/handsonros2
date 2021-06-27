#!/usr/bin/env python

# Note: The "ROS2 Way" (and "Pythonic") is in publisher_member_function.py
#       This file is an attempt to directly migrate the book's example to ROS2
#       ROS2 Execution is different - unless you "spin", the rate.sleep() will never return

# BEGIN IMPORT
# import rospy
import rclpy  # ROS Client Library For Python
from rclpy.node import Node
import sys
import threading
# END IMPORT

# BEGIN STD_MSGS
from std_msgs.msg import Int32
# END STD_MSGS

# NODE INITIALIZATION
# rospy.init_node('topic_publisher')

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('topic_publisher')

    node.get_logger().info('Created topic_publisher node')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # BEGIN PUB
    # pub = rospy.Publisher('counter', Int32, queue_size=10)
    pub = node.create_publisher(Int32, 'counter',qos_profile=10)

    # END PUB

    # BEGIN LOOP
    # rate = rospy.Rate(2) # If set to 1 Hz it will be a counter of seconds
    rate = node.create_rate(2)
    count = 0
    # while not rospy.is_shutdown():
    # there is no rclpy.is_shutdown()
    try:
        while rclpy.ok():
            msg = Int32()
            msg.data = count
            pub.publish(msg)
            node.get_logger().info('Published Count: {}'.format(count))
            count += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
    # END LOOP
    # END ALL - end of main()

if __name__ == '__main__':
    main()
