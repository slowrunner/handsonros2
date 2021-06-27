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
- log_w_dt.py: Demonstrates adding datetime to log msgs  
```
$ ros2 run chapter3_ros2_basics log_w_dt 
[INFO] [1624812324.155095487] [log_w_dt]: [2021-06-27 12:45:24] Created log_w_dt node
[INFO] [1624812324.646024785] [log_w_dt]: [2021-06-27 12:45:24] Publishing: 0
[INFO] [1624812325.146772983] [log_w_dt]: [2021-06-27 12:45:25] Publishing: 1
^C[INFO] [1624812327.262942489] [log_w_dt]: [2021-06-27 12:45:27] control-c: log_w_dt shutting down
```  
