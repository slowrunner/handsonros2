# ROS2 versions of "Hands-On-ROS-for-Robotics-Programming

Originals: https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming/tree/master/Chapter3_ROS_basics

NOTE: "ROS2 Migration" 
- is one-to-one rospy-to-rclpy conversion
  with addition of a thread to execute rclpy.spin()  
  which is needed in ROS2 to make callbacks and rate.sleep() happen
  
"Pythonic ROS2 Migration"
- involves reorganization into a Class with main()
  of the one-to-one rospy-to-rclpy conversions

FILES:
- doubler.py: "Pythonic" ROS2 version of original
- __init__.py: ROS2 ament_python auto generated file
- publisher_member_function.py:  
  From ROS2 Tutorial: https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- subscriber_member_function.py
- topic_publisher.py: "ROS2 Migration" of original
- topic_pub_node.py: "Pythonic" ROS2 version of original
- topic_sub_node.py: "Pythonic" ROS2 version of original
- topic_subscriber.py: "ROS2 Migration" of original
