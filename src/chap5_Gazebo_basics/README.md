# ROS2 Gazebo Basics

The files in this folder provide the code samples for "Chapter 5: Gazebo Basics" of the book "Hands-on ROS for Robotics Programming"

ROS2 Versions of Chapter 5 of Hands-On-ROS-For-Robotics-Programming
using a gpgMin.urdf that more closely matches the body, castor, and wheel sizes and positions

To build gazebo_basics package:  
$ rosdep install -i --from-path src      
$ colcon build --packages-selelect gazebo_basics  
$ source install/setup.bash  

To launch everything  

ros2 launch gazebo_basics ros2_gpgMin_gazebo.launch.py  

or  

ros2 launch gazebo_basics ros2_gopigoMinimal_gazebo.launch.py  


Basis:
- urdf/gpgMin.urdf   is my creation of a 220mm long, 104mm wide, 70mm high "chassis" box,
                     with 66mm dia by 25mm wide wheels, placed 20mm forward of the chassis center
                     and with a 117mm wheel base.  A 3/4" castor holds the robot level
                     
- urdf/gopigoMinimal.urdf  book's overly simplified model of a GoPiGo3

- launch/ros2_gopigoMinimal_gazebo.launch.py:         use book's gopigoMinimal.urdf and launch everything 
- launch/ros2_gpgMin_gazebo.launch.py:          use my urdf and launch everything 

![gpgMin.urdf in rviz2](Chap5_gazebo_basics.jpg?raw=true)
