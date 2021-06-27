# handsonros2
Desktop code - ROS2 version of "Hands-On-ROS-For-Robotics-Programming"

Originals: https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming


NOTE: "ROS2 Migration" 
- is one-to-one rospy-to-rclpy conversion
  with addition of a thread to execute rclpy.spin()  
  which is needed in ROS2 to make callbacks and rate.sleep() happen
  
"Pythonic ROS2 Migration"
- involves reorganization into a Class with main()
  of the one-to-one rospy-to-rclpy conversions
