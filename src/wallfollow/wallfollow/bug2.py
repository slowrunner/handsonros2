# Author: Addison Sears-Collins
# Date: December 17, 2020
# ROS Version: ROS 2 Foxy Fitzroy
# REF: https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/



############## IMPORT LIBRARIES #################
# Python math library
import math

# ROS client library for Python
import rclpy

# Enables pauses in the execution of code
from time import sleep

# Used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)
from sensor_msgs.msg import LaserScan

# Handle Pose messages
from geometry_msgs.msg import Pose

# Handle float64 arrays
from std_msgs.msg import Float64MultiArray

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data

# Scientific computing library
import numpy as np

class PlaceholderController(Node):
    """
    Create a Placeholder Controller class, which is a subclass of the Node
    class for ROS2.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        ####### INITIALIZE ROS PUBLISHERS AND SUBSCRIBERS##############
        # Initiate the Node class's constructor and give it a name
        super().__init__('PlaceholderController')

        # Create a subscriber
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /en613/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/en613/state_est',
            self.state_estimate_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a subscriber
        # This node subscribes to messages of type
        # sensor_msgs/LaserScan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/en613/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        # Create a subscriber
        # This node subscribes to messages of type geometry_msgs/Pose
        # over a topic named: /en613/goal
        # The message represents the the goal position.
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription_goal_pose = self.create_subscription(
            Pose,
            '/en613/goal',
            self.pose_received,
            10)

        # Create a publisher
        # This node publishes the desired linear and angular velocity
        # of the robot (in the robot chassis coordinate frame) to the
        # /en613/cmd_vel topic. Using the diff_drive
        # plugin enables the basic_robot model to read this
        # /end613/cmd_vel topic and execute the motion accordingly.
        self.publisher_ = self.create_publisher(
            Twist,
            '/en613/cmd_vel',
            10)

        # Initialize the LaserScan sensor readings to some large value
        # Values are in meters.
        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right

        ################### ROBOT CONTROL PARAMETERS ##################

        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed = 0.035

        # Current position and orientation of the robot in the global
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # By changing the value of self.robot_mode, you can alter what
        # the robot will do when the program is launched.
        #   "obstacle avoidance mode": Robot will avoid obstacles
        #   "go to goal mode": Robot will head to an x,y coordinate
        #   "wall following mode": Robot will follow a wall
        self.robot_mode = "go to goal mode"

        ############# OBSTACLE AVOIDANCE MODE PARAMETERS ##############

        # Obstacle detection distance threshold
        self.dist_thresh_obs = 0.25 # in meters

        # Maximum left-turning speed
        self.turning_speed = 0.25 # rad/s

        ############# GO TO GOAL MODE PARAMETERS ######################
        # Finite states for the go to goal mode
        #   "adjust heading": Orient towards a goal x, y coordinate
        #   "go straight": Go straight towards goal x, y coordinate
        #   "goal achieved": Reached goal x, y coordinate
        self.go_to_goal_state = "adjust heading"

        # List the goal destinations
        # We create a list of the (x,y) coordinate goals
        self.goal_x_coordinates = False # [ 0.0, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
        self.goal_y_coordinates = False # [-4.0, 1.0, 1.5,  1.0, -3.0, -4.0, 0.0]

        # Keep track of which goal we're headed towards
        self.goal_idx = 0

        # Keep track of when we've reached the end of the goal list
        self.goal_max_idx =  None # len(self.goal_x_coordinates) - 1

        # +/- 2.0 degrees of precision
        self.yaw_precision = 2.0 * (math.pi / 180)

        # How quickly we need to turn when we need to make a heading
        # adjustment (rad/s)
        self.turning_speed_yaw_adjustment = 0.0625

        # Need to get within +/- 0.2 meter (20 cm) of (x,y) goal
        self.dist_precision = 0.2

        ############# WALL FOLLOWING MODE PARAMETERS ##################
        # Finite states for the wall following mode
        #   "turn left": Robot turns towards the left
        #   "search for wall": Robot tries to locate the wall
        #   "follow wall": Robot moves parallel to the wall
        self.wall_following_state = "turn left"

        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0  # Fast turn
        self.turning_speed_wf_slow = 0.125 # Slow turn

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.45 # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.15 # in meters

        ################### BUG2 PARAMETERS ###########################

        # Bug2 Algorithm Switch
        # Can turn "ON" or "OFF" depending on if you want to run Bug2
        # Motion Planning Algorithm
        self.bug2_switch = "ON"

        # Start-Goal Line Calculated?
        self.start_goal_line_calculated = False

        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0

        # Anything less than this distance means we have encountered
        # a wall. Value determined through trial and error.
        self.dist_thresh_bug2 = 0.15

        # Leave point must be within +/- 0.1m of the start-goal line
        # in order to go from wall following mode to go to goal mode
        self.distance_to_start_goal_line_precision = 0.1

        # Used to record the (x,y) coordinate where the robot hit
        # a wall.
        self.hit_point_x = 0
        self.hit_point_y = 0

        # Distance between the hit point and the goal in meters
        self.distance_to_goal_from_hit_point = 0.0

        # Used to record the (x,y) coordinate where the robot left
        # a wall.
        self.leave_point_x = 0
        self.leave_point_y = 0

        # Distance between the leave point and the goal in meters
        self.distance_to_goal_from_leave_point = 0.0

        # The hit point and leave point must be far enough
        # apart to change state from wall following to go to goal
        # This value helps prevent the robot from getting stuck and
        # rotating in endless circles.
        # This distance was determined through trial and error.
        self.leave_point_to_hit_point_diff = 0.25 # in meters

    def pose_received(self,msg):
        """
        Populate the pose.
        """
        self.goal_x_coordinates = [msg.position.x]
        self.goal_y_coordinates = [msg.position.y]
        self.goal_max_idx = len(self.goal_x_coordinates) - 1

    def scan_callback(self, msg):
        """
        This method gets called every time a LaserScan message is 
        received on the /en613/scan ROS topic
        """
        # Read the laser scan data that indicates distances
        # to obstacles (e.g. wall) in meters and extract
        # 5 distinct laser readings to work with.
        # Each reading is separated by 45 degrees.
        # Assumes 181 laser readings, separated by 1 degree.
        # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
        self.left_dist = msg.ranges[180]
        self.leftfront_dist = msg.ranges[135]
        self.front_dist = msg.ranges[90]
        self.rightfront_dist = msg.ranges[45]
        self.right_dist = msg.ranges[0]

        # The total number of laser rays. Used for testing.
        #number_of_laser_rays = str(len(msg.ranges))

        # Print the distance values (in meters) for testing
        #self.get_logger().info('L:%f LF:%f F:%f RF:%f R:%f' % (
        #   self.left_dist,
        #   self.leftfront_dist,
        #   self.front_dist,
        #   self.rightfront_dist,
        #   self.right_dist))

        if self.robot_mode == "obstacle avoidance mode":
            self.avoid_obstacles()

    def state_estimate_callback(self, msg):
        """
        Extract the position and orientation data.
        This callback is called each time
        a new message is received on the '/en613/state_est' topic
        """
        # Update the current estimated state in the global reference frame
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

        # Wait until we have received some goal destinations.
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return

        # Print the pose of the robot
        # Used for testing
        #self.get_logger().info('X:%f Y:%f YAW:%f' % (
        #   self.current_x,
        #   self.current_y,
        #   np.rad2deg(self.current_yaw)))  # Goes from -pi to pi

        # See if the Bug2 algorithm is activated. If yes, call bug2()
        if self.bug2_switch == "ON":
            self.bug2()
        else:

            if self.robot_mode == "go to goal mode":
                self.go_to_goal()
            elif self.robot_mode == "wall following mode":
                self.follow_wall()
            else:
                pass # Do nothing

    def avoid_obstacles(self):
        """
        Wander around the maze and avoid obstacles.
        """
        # Create a Twist message and initialize all the values
        # for the linear and angular velocities
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Logic for avoiding obstacles (e.g. walls)
        # >d means no obstacle detected by that laser beam
        # <d means an obstacle was detected by that laser beam
        d = self.dist_thresh_obs
        if   self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed # Go straight forward
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            msg.angular.z = self.turning_speed  # Turn left
        elif self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            msg.angular.z = -self.turning_speed # Turn right
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            msg.angular.z = -self.turning_speed
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            msg.linear.x = self.forward_speed
        else:
            pass

        # Send the velocity commands to the robot by publishing
        # to the topic
        self.publisher_.publish(msg)

    def go_to_goal(self):
        """
        This code drives the robot towards to the goal destination
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # If Bug2 algorithm is activated
        if self.bug2_switch == "ON":

            # If the wall is in the way
            d = self.dist_thresh_bug2
            if (    self.leftfront_dist < d or
                self.front_dist < d or
                self.rightfront_dist < d):

                # Change the mode to wall following mode.
                self.robot_mode = "wall following mode"

                # Record the hit point
                self.hit_point_x = self.current_x
                self.hit_point_y = self.current_y

                # Record the distance to the goal from the
                # hit point
                self.distance_to_goal_from_hit_point = (
                    math.sqrt((
                    pow(self.goal_x_coordinates[self.goal_idx] - self.hit_point_x, 2)) + (
                    pow(self.goal_y_coordinates[self.goal_idx] - self.hit_point_y, 2))))

                # Make a hard left to begin following wall
                msg.angular.z = self.turning_speed_wf_fast

                # Send command to the robot
                self.publisher_.publish(msg)

                # Exit this function
                return

        # Fix the heading
        if (self.go_to_goal_state == "adjust heading"):

            # Calculate the desired heading based on the current position
            # and the desired position
            desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)

            # How far off is the current heading in radians?
            yaw_error = desired_yaw - self.current_yaw

            # Adjust heading if heading is not good enough
            if math.fabs(yaw_error) > self.yaw_precision:

                if yaw_error > 0:
                    # Turn left (counterclockwise)
                    msg.angular.z = self.turning_speed_yaw_adjustment
                else:
                    # Turn right (clockwise)
                    msg.angular.z = -self.turning_speed_yaw_adjustment

                # Command the robot to adjust the heading
                self.publisher_.publish(msg)

            # Change the state if the heading is good enough
            else:
                # Change the state
                self.go_to_goal_state = "go straight"

                # Command the robot to stop turning
                self.publisher_.publish(msg)

        # Go straight
        elif (self.go_to_goal_state == "go straight"):

            position_error = math.sqrt(
                        pow(
                        self.goal_x_coordinates[self.goal_idx] - self.current_x, 2)
                        + pow(
                        self.goal_y_coordinates[self.goal_idx] - self.current_y, 2))


            # If we are still too far away from the goal
            if position_error > self.dist_precision:

                # Move straight ahead
                msg.linear.x = self.forward_speed

                # Command the robot to move
                self.publisher_.publish(msg)

                # Check our heading
                desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)

                # How far off is the heading?
                yaw_error = desired_yaw - self.current_yaw

                # Check the heading and change the state if there is too much heading error
                if math.fabs(yaw_error) > self.yaw_precision:

                    # Change the state
                    self.go_to_goal_state = "adjust heading"

            # We reached our goal. Change the state.
            else:
                # Change the state
                self.go_to_goal_state = "goal achieved"

                # Command the robot to stop
                self.publisher_.publish(msg)

        # Goal achieved
        elif (self.go_to_goal_state == "goal achieved"):

            self.get_logger().info('Goal achieved! X:%f Y:%f' % (
                self.goal_x_coordinates[self.goal_idx],
                self.goal_y_coordinates[self.goal_idx]))

            # Get the next goal
            self.goal_idx = self.goal_idx + 1

            # Do we have any more goals left?
            # If we have no more goals left, just stop
            if (self.goal_idx > self.goal_max_idx):
                self.get_logger().info('Congratulations! All goals have been achieved.')
                while True:
                    pass

            # Let's achieve our next goal
            else:
                # Change the state
                self.go_to_goal_state = "adjust heading"

            # We need to recalculate the start-goal line if Bug2 is running
            self.start_goal_line_calculated = False

        else:
            pass

    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall.
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Special code if Bug2 algorithm is activated
        if self.bug2_switch == "ON":

            # Calculate the point on the start-goal
            # line that is closest to the current position
            x_start_goal_line = self.current_x
            y_start_goal_line = (
                self.start_goal_line_slope_m * (
                x_start_goal_line)) + (
                self.start_goal_line_y_intercept)

            # Calculate the distance between current position
            # and the start-goal line
            distance_to_start_goal_line = math.sqrt(pow(
                        x_start_goal_line - self.current_x, 2) + pow(
                        y_start_goal_line - self.current_y, 2))

            # If we hit the start-goal line again
            if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:

                # Determine if we need to leave the wall and change the mode
                # to 'go to goal'
                # Let this point be the leave point
                self.leave_point_x = self.current_x
                self.leave_point_y = self.current_y

                # Record the distance to the goal from the leave point
                self.distance_to_goal_from_leave_point = math.sqrt(
                    pow(self.goal_x_coordinates[self.goal_idx]
                    - self.leave_point_x, 2)
                    + pow(self.goal_y_coordinates[self.goal_idx]
                    - self.leave_point_y, 2))

                # Is the leave point closer to the goal than the hit point?
                # If yes, go to goal.
                diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
                if diff > self.leave_point_to_hit_point_diff:

                    # Change the mode. Go to goal.
                    self.robot_mode = "go to goal mode"


                # Exit this function
                return

        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means an wall was detected by that laser beam
        d = self.dist_thresh_wf

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast


        elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                # Getting too close to the wall
                self.wall_following_state = "turn left"
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast
            else:
                # Go straight ahead
                self.wall_following_state = "follow wall"
                msg.linear.x = self.forward_speed

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall

        else:
            pass

        # Send velocity command to the robot
        self.publisher_.publish(msg)

    def bug2(self):

        # Each time we start towards a new goal, we need to calculate the start-goal line
        if self.start_goal_line_calculated == False:

            # Make sure go to goal mode is set.
            self.robot_mode = "go to goal mode"

            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x_coordinates[self.goal_idx]
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y_coordinates[self.goal_idx]

            # Calculate the slope of the start-goal line m
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / (
                self.start_goal_line_xgoal - self.start_goal_line_xstart))

            # Solve for the intercept b
            self.start_goal_line_y_intercept = self.start_goal_line_ygoal - (
                    self.start_goal_line_slope_m * self.start_goal_line_xgoal) 

            # We have successfully calculated the start-goal line
            self.start_goal_line_calculated = True

        if self.robot_mode == "go to goal mode":
            self.go_to_goal()
        elif self.robot_mode == "wall following mode":
            self.follow_wall()
def main(args=None):

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = PlaceholderController()

    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
