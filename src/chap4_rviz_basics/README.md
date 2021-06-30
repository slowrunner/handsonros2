# Creating the virtual two wheeled ROS robot

The files in this folder provide the code samples for "Chapter 4: Creating the virtual two wheeled ROS robot" of the book "Hands-on ROS for Robotics Programming"

ROS2 Versions of Chapter 4 of Hands-On-ROS-For-Robotics-Programming

To build rviz2_basics package:
$ rosdep install -i --from-path src
$ colcon build --packages-selelect rviz2_basics
$ . install/setup.bash

To run robot_state_publisher and joint_state_publisher:
T1:ros2 launch rviz2_basics ros2_gpgMin_rviz2_simple.launch.py
T2:rviz2

or to additionally launch rviz2

ros2 launch rviz2_basics ros2_gpgMin_rviz2.launch.py

(P.S: I haven't the slightest idea what I'm doing - so this gets close)
