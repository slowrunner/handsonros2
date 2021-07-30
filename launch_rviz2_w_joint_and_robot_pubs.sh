#!/bin/bash
# FILE: launch_rviz2_w_joint_and_robot_pubs.sh
#
# PURPOSE: Start rviz2 
#          (rviz2 will not display the wheels moving with the bot without
#          the joint_state_publisher and robot_state_publisher running)
#
# Note: killing rviz2 does not kill the state_pubs, so cntrl-c when done.

cd ~/handsonros2
source ~/handsonros2/install/setup.bash
ros2 launch rviz2_basics ros2_gpgMin_rviz2.launch.py
