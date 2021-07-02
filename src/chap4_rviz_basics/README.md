# Creating the virtual two wheeled ROS robot

The files in this folder provide the code samples for "Chapter 4: Creating the virtual two wheeled ROS robot" of the book "Hands-on ROS for Robotics Programming"

ROS2 Versions of Chapter 4 of Hands-On-ROS-For-Robotics-Programming

To build rviz2_basics package:
$ rosdep install -i --from-path src    (installs robot_state_publisher, joint_state_publisher, joint_state_publisher_gui)
$ colcon build --packages-selelect rviz2_basics
$ . install/setup.bash

To run robot_state_publisher and joint_state_publisher:
T1:ros2 launch rviz2_basics ros2_gpgMin_rviz2_simple.launch.py
T2:rviz2

or to additionally launch rviz2

ros2 launch rviz2_basics ros2_gpgMin_rviz2.launch.py

STATUS: I haven't the slightest idea what I'm doing - but this gets close
- TF is complaining base_link cannot be passed as target frame? 


Basis:
- urdf/gpgMin.urdf   is my creation of a 220mm long, 104mm wide, 70mm high "chassis" box,
                     with 66mm dia by 25mm wide wheels, placed 20mm forward of the chassis center
                     and with a 117mm wheel base.  A 3/4" castor holds the robot level
                     rviz2 is comp 
- rviz/gpgMin.rviz2  contains the config for viewing the gpgMin.urdf model


