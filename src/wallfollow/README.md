bug2 wallfollow and obstacle avoidance in ROS2

REF:  https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/



On a high level, the Bug2 algorithm has two main modes:  

Go to Goal Mode: Move from the current location towards the goal (x,y) coordinate.  
Wall Following Mode: Move along a wall.  
Here is pseudocode for the algorithm:  

1.      Calculate a start-goal line. The start-goal line is an imaginary line that connects the starting position to the goal position.  

2.      While Not at the Goal  

Move towards the goal along the start-goal line.  
If a wall is encountered:  
Remember the location where the wall was first encountered. This is the “hit point.”  
Follow the wall until you encounter the start-goal line. This point is known as the “leave point.”  
 If the leave point is closer to the goal than the hit point, leave the wall, and move towards the goal again.  
Otherwise, continue following the wall.  


Can also set to 
- obstacle avoidance only
- wall follow only

Sensor: 180 degree laser scan (0 right, 180 left)


