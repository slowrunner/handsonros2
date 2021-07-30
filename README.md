# handsonros2 on Desktop

Desktop code - ROS2 version of "Hands-On-ROS-For-Robotics-Programming"

Originals: https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming


NOTE: "ROS2 Migration" 
- is one-to-one rospy-to-rclpy conversion
  with addition of a thread to execute rclpy.spin()  
  which is needed in ROS2 to make callbacks and rate.sleep() happen
  
- or preferably "Pythonic ROS2 Migration"  
  involves reorganization into a Class with main()
  of the one-to-one rospy-to-rclpy conversions


# WATCH GoPiGo3 ROSbot Dave in Rviz2

- On ROSbot Dave (launch ROS2 gopigo3_node):
```
cd ~/rosbot-on-gopigo3/handsonros2
./runit.sh

and when done:
./killit.sh
```

- On Desktop PC (launch Rviz2 and needed publishers):  
```
T1:  
cd ~/handsonros2  
./launch_rviz2_w_joint_and_robot_pubs.sh  
```

# Drive ROSbot Dave

Two methods (rqt->Robot_Steering plugin, or teleop_keyboard_twist node):
- Using rqt 
```
On Desktop PC T2:
rqt
   plugins->Robot Steering
```

- Using teleop_twist_keyboard (installed on ROSbot Dave):
```
From 2nd ssh session to ROSbot Dave:

cd ~/rosbot-on-gopigo3/handsonros2
source install/setup.bash
ros2 run teleop_twist_keybard teleop_twist_keyboard

press x repeatedly unil speed is around 0.1 m/s

i         - fwd
k         - stop
j, l      - spin left/right
comma/"<" - bwd

ctrl-c    - quit
```

