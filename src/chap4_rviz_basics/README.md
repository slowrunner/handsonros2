# Creating the virtual two wheeled ROS robot

The files in this folder provide the code samples for "Chapter 4: Creating the virtual two wheeled ROS robot" of the book "Hands-on ROS for Robotics Programming"

ROS2 Versions of Chapter 4 of Hands-On-ROS-For-Robotics-Programming
plus a gpgMin.urdf that more closely matches the body, castor, and wheel sizes and positions

To build rviz2_basics package:
$ rosdep install -i --from-path src    (installs robot_state_publisher, joint_state_publisher, joint_state_publisher_gui)
$ colcon build --packages-selelect rviz2_basics
$ source install/setup.bash

To run robot_state_publisher and joint_state_publisher:
T1: ros2 launch rviz2_basics ros2_state_and_joint.launch.py
T2: rviz2 -d ~/handsonros2/src/chap4_rviz_basics/rviz/gpgMin.rviz2.rviz

or to additionally launch rviz2

ros2 launch rviz2_basics ros2_gpgMin_rviz2_simple.launch.py
or
ros2 launch rviz2_basics ros2_gopigoMinimal_rviz2_simple.launch.py


Basis:
- urdf/gpgMin.urdf   is my creation of a 220mm long, 104mm wide, 70mm high "chassis" box,
                     with 66mm dia by 25mm wide wheels, placed 20mm forward of the chassis center
                     and with a 117mm wheel base.  A 3/4" castor holds the robot level
                     
- urdf/gopigoMinimal.urdf  book's overly simplified model of a GoPiGo3

- rviz/gpgMin.rviz2.rviz  contains the config for viewing the gpgMin.urdf model

- launch/os2_gopigoMinimal_rviz2_simple.launch.py:    use book's gopigoMinimal.urdf and launch everything (w/joint gui)
- launch/ros2_gpgMin_rviz2_simple.launch.py:          use my urdf and launch everything (w/joint gui)
- launch/ros2_gpgMin_rviz2.launch.py:                 failed attempt passing args and use_gui parm for joint_state_publisher
- launch/ros2_state_and_joint.launch.py:              only launch state and joint publishers, no rviz2

![gpgMin.urdf in rviz2](Chap4_rviz2_basics_gpgMin.urdf.jpg?raw=true)
