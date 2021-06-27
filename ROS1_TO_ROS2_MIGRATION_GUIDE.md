# ROS1 to ROS2 Migration Guide

- Based on https://docs.ros.org/en/rolling/Contributing/Migration-Guide-Python.html
- Spinning based on https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/?answer=358386#post-id-358386



# BASIC ROS1 EXAMPLE IN ROS2

ROS1 SIMPLE_NODE:

```
#! /usr/bin/env python

# Endless loop at 2 Hz (Twice per second = 0.5 sec period)
# Ctrl-c to end

import rospy

rospy.init_node("simple_node")

rate = rospy.Rate(2) # We create a Rate object of 2Hz

while not rospy.is_shutdown(): # Endless loop until Ctrl + C
    print("simple_node is alive")
    rate.sleep()   # sleep to maintain rate set above
```


MIGRATION (brute force): ROS2 SIMPLE_NODE ~/ros2ws/src/simple_node_pkg/simple_node_pkg/simple_node.py:
```
#!/usr/bin/env python3

# add entry point to simple_node_pkg/setup.py
#     'console_scripts': [
#            'simple_node = simple_node_pkg.simple_node:main',

# ros2 run simple_node_pkg simple_node


import threading  # Needed to call "spin" 
                  # which manages/executes ROS2 discovery, callbacks, time updates, and more 

import rclpy      # ROS2 Client Library For Python

def main():       # wrapped everything into main for convenient entry point on command line
  rclpy.init()      

  node = rclpy.create_node('simple_node')

  # Spin in a separate thread
  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()

  rate = node.create_rate(2)  # create a rate object with 2Hz freq (0.5 sec period)

  try:
    while rclpy.ok():
      print('simple_node is alive')
      rate.sleep()    # spin above ensures time advances and rate callback occurs
  except KeyboardInterrupt:
    pass

  # Clean-up
  rclpy.shutdown()
  thread.join()

if __name__ == '__main__':
  main()

```

ROS1 to ROS2 MIGRATION ("Pythonic" ~/ros2ws/src/simple_node_pkg/simple_node_pkg/simple_node.py ):
```
#!/usr/bin/env python3

# First create Python ROS2 package:
# cd ~/ros2_ws/src        
# ros2 pkg create --build-type ament_python [--node-name simple_node] simple_node_pkg
# edit this file into ~/ros2ws/src/simple_node_pkg/simple_node_pkg/simple_node.py
# add entry point to ~/ros2ws/src/simple_node_pkg/setup.py
#     'console_scripts': [
#            'simple_node = simple_node_pkg.simple_node:main',

# source install/setup.bash
# ros2 run simple_node_pkg simple_node

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Simple_Node(Node):

    def __init__(self):
        super().__init__('simple_node')
        
        # Create a timer that will gate the node actions twice a second
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)
        self.get_logger().info('simple_node and node_callback created')
    # 
    def node_callback(self):
        self.get_logger().info('simple_node is alive')
    


def main(args=None):
    rclpy.init(args=args)

    my_node = Simple_Node()   # instantiate my simple_node
    try:
        rclpy.spin(my_node)       # execute simple_node 
    except KeyboardInterrupt:
        my_node.get_logger().info('\ncontrol-c: shutting down')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        my_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

===
# MIGRATE ROS Parameters

In ROS 1:
```
 port = rospy.get_param('port', '/dev/ttyUSB0')
 assert isinstance(port, str), 'port parameter must be a str'

 buadrate = rospy.get_param('baudrate', 115200)
 assert isinstance(port, int), 'port parameter must be an integer'

rospy.logwarn('port: ' + port)
```

In ROS 2:
```
port = node.declare_parameter('port', '/dev/ttyUSB0').value
assert isinstance(port, str), 'port parameter must be a str'

baudrate = node.declare_parameter('baudrate', 115200).value
assert isinstance(port, int), 'port parameter must be an integer'

node.get_logger().warn('port: ' + port)
```

# MIGRATING A PUBLISHER

In ROS 1:
```
pub = rospy.Publisher('chatter', String)
```

In ROS 2:
```
pub = node.create_publisher(String, 'chatter')
```

# Creating a Subscriber

In ROS 1:
```
sub = rospy.Subscriber('chatter', String, callback)
```
In ROS 2:
```
sub = node.create_subscription(String, 'chatter', callback)
```

# MIGRATING A SERVICE 

In ROS 1:
```
srv = rospy.Service('add_two_ints', AddTwoInts, add_two_ints_callback)
```
In ROS 2:
```
srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)
```


# MIGRATING A SERVICE CLIENT

In ROS 1:
```
rospy.wait_for_service('add_two_ints')
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
resp = add_two_ints(req)

```
In ROS 2:
```
add_two_ints = node.create_client(AddTwoInts, 'add_two_ints')
while not add_two_ints.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
resp = add_two_ints.call_async(req)
rclpy.spin_until_future_complete(node, resp)
```

# ADDING DATETIME TO LOG MESSAGES

```
import datetime as dt

# add_dt(str) - prepends datetime "[YYYY-MM-DD HH:MM:SS]" to str
def add_dt(str):
  new_str = dt.datetime.now().strftime('[%Y-%m-%d %H:%M:%S] ') + str
  return new_str``` 
```
