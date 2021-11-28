# State Estimator
# Author: Addison Sears-Collins
# Date: December 8, 2020
# ROS Version: ROS 2 Foxy Fitzroy
# REF: https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/



# Python math library
import math

# ROS client library for Python
import rclpy

# Used to create nodes
from rclpy.node import Node

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry

# Handles laser distance scan to detect obstacles
from sensor_msgs.msg import LaserScan

# Used for laser scan
from rclpy.qos import qos_profile_sensor_data

# Enable use of std_msgs/Float64MultiArray message
from std_msgs.msg import Float64MultiArray 

# Scientific computing library for Python
import numpy as np

class PlaceholderEstimator(Node):
    """
    Class constructor to set up the node
    """
    def __init__(self):
        ############## INITIALIZE ROS PUBLISHERS AND SUBSCRIBERS ######


        # Initiate the Node class's constructor and give it a name
        super().__init__('PlaceholderEstimator')

        # Create a subscriber
        # This node subscribes to messages of type
        # geometry_msgs/Twist.msg
        # The maximum number of queued messages is 10.
        self.velocity_subscriber = self.create_subscription(
            Twist,
            '/en613/cmd_vel',
            self.velocity_callback,
            10)

        # Create a subscriber
        # This node subscribes to messages of type
        # nav_msgs/Odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/en613/odom',
            self.odom_callback,
            10)

        # Create a publisher
        # This node publishes the estimated position (x, y, yaw) 
        # The type of message is std_msgs/Float64MultiArray
        self.publisher_state_est = self.create_publisher(
            Float64MultiArray, 
            '/en613/state_est', 
            10)

        ############# STATE TRANSITION MODEL PARAMETERS ############### 

        # Time step from one time step t-1 to the next time step t
        self.delta_t = 0.002 # seconds

        # Keep track of the estimate of the yaw angle
        # for control input vector calculation.
        self.est_yaw = 0.0

        # A matrix
        # 3x3 matrix -> number of states x number of states matrix
        # Expresses how the state of the system [x,y,yaw] changes
        # from t-1 to t when no control command is executed. Typically
        # a robot on wheels only drives when the wheels are commanded
        # to turn.
        # For this case, A is the identity matrix.
        # A is sometimes F in the literature.
        self.A_t_minus_1 = np.array([   [1.0,  0,   0],
                        [  0,1.0,   0],
                        [  0,  0, 1.0]])

        # The estimated state vector at time t-1 in the global
        # reference frame
        # [x_t_minus_1, y_t_minus_1, yaw_t_minus_1]
        # [meters, meters, radians]
        self.state_vector_t_minus_1 = np.array([0.0,0.0,0.0])

        # The control input vector at time t-1 in the global
        # reference frame
        # [v,v,yaw_rate]
        # [meters/second, meters/second, radians/second]
        # In the literature, this is commonly u.
        self.control_vector_t_minus_1 = np.array([0.001,0.001,0.001])

        # Noise applied to the forward kinematics (calculation
        # of the estimated state at time t from the state transition
        # model of the mobile robot. This is a vector with the
        # number of elements equal to the number of states.
        self.process_noise_v_t_minus_1 = np.array([0.096,0.096,0.032])

        ############# MEASUREMENT MODEL PARAMETERS ####################

        # Measurement matrix H_t
        # Used to convert the predicted state estimate at time t=1
        # into predicted sensor measurements at time t=1.
        # In this case, H will be the identity matrix since the
        # estimated state maps directly to state measurements from the
        # odometry data [x, y, yaw]
        # H has the same number of rows as sensor measurements
        # and same number of columns as states.
        self.H_t = np.array([   [1.0,  0,   0],
                    [  0,1.0,   0],
                    [  0,  0, 1.0]])

        # Sensor noise. This is a vector with the
        # number of elements as the number of sensor measurements.
        self.sensor_noise_w_t = np.array([0.07,0.07,0.04])

        ############# EXTENDED KALMAN FILTER PARAMETERS ###############

        # State covariance matrix P_t_minus_1
        # This matrix has the same number of rows (and columns) as the
        # number of states (i.e. 3x3 matrix). P is sometimes referred
        # to as Sigma in the literature. It represents an estimate of
        # the accuracy of the state estimate at t=1 made using the
        # state transition matrix. We start off with guessed values.
        self.P_t_minus_1 = np.array([   [0.1,  0,   0],
                        [  0,0.1,   0],
                        [  0,  0, 0.1]])

        # State model noise covariance matrix Q_t
        # When Q is large, the Kalman Filter tracks large changes in
        # the sensor measurements more closely than for smaller Q.
        # Q is a square matrix that has the same number of rows as
        # states.
        self.Q_t = np.array([       [1.0,   0,   0],
                        [  0, 1.0,   0],
                        [  0,   0, 1.0]])

        # Sensor measurement noise covariance matrix R_t
        # Has the same number of rows and columns as sensor measurements.
        # If we are sure about the measurements, R will be near zero.
        self.R_t = np.array([       [1.0,   0,    0],
                        [  0, 1.0,    0],
                        [  0,    0, 1.0]])

    def velocity_callback(self, msg):
        """
        Listen to the velocity commands (linear forward velocity
        in the x direction in the robot's reference frame and
        angular velocity (yaw rate) around the robot's z-axis.
        Convert those velocity commands into a 3-element control
        input vector ...
        [v,v,yaw_rate]
        [meters/second, meters/second, radians/second]
        """
        # Forward velocity in the robot's reference frame
        v = msg.linear.x

        # Angular velocity around the robot's z axis
        yaw_rate = msg.angular.z

        # [v,v,yaw_rate]
        self.control_vector_t_minus_1[0] = v
        self.control_vector_t_minus_1[1] = v
        self.control_vector_t_minus_1[2] = yaw_rate

    def odom_callback(self, msg):
        """
        Receive the odometry information containing the position and orientation
        of the robot in the global reference frame. 
        The position is x, y, z.
        The orientation is a x,y,z,w quaternion. 
        """
        roll, pitch, yaw = self.euler_from_quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)

        obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]

        # These are the measurements taken by the odometry in Gazebo
        z_t_observation_vector =  np.array([obs_state_vector_x_y_yaw[0],
                        obs_state_vector_x_y_yaw[1],
                        obs_state_vector_x_y_yaw[2]])

        # Apply the Extended Kalman Filter
        # This is the updated state estimate after taking the latest
        # sensor (odometry) measurements into account.
        updated_state_estimate_t = self.ekf(z_t_observation_vector)

        # Publish the estimate state
        self.publish_estimated_state(updated_state_estimate_t)

    def publish_estimated_state(self, state_vector_x_y_yaw):
        """
        Publish the estimated pose (position and orientation) of the
        robot to the '/en613/state_est' topic.
        :param: state_vector_x_y_yaw [x, y, yaw]
            x is in meters, y is in meters, yaw is in radians
        """
        msg = Float64MultiArray()
        msg.data = state_vector_x_y_yaw
        self.publisher_state_est.publish(msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

    def getB(self,yaw,dt):
        """
        Calculates and returns the B matrix

        3x3 matix -> number of states x number of control inputs
        The control inputs are the forward speed and the rotation
        rate around the z axis from the x-axis in the
        counterclockwise direction.
        [v,v,yaw_rate]
        Expresses how the state of the system [x,y,yaw] changes
        from t-1 to t
        due to the control commands (i.e. inputs).
        :param yaw: The yaw (rotation angle around the z axis) in rad
        :param dt: The change in time from time step t-1 to t in sec
        """
        B = np.array([  [np.cos(yaw) * dt,  0,   0],
                [  0, np.sin(yaw) * dt,   0],
                [  0,  0, dt]])
        return B

    def ekf(self,z_t_observation_vector):
        """
        Extended Kalman Filter. Fuses noisy sensor measurement to
        create an optimal estimate of the state of the robotic system.

        INPUT
        :param z_t_observation_vector The observation from the Odometry
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].

        OUTPUT
        :return state_estimate_t optimal state estimate at time t
            [x,y,yaw]....3x1 list --->
            [meters,meters,radians]

        """
        ######################### Predict #############################
        # Predict the state estimate at time t based on the state
        # estimate at time t-1 and the control input applied at time
        # t-1.
        state_estimate_t = self.A_t_minus_1 @ (
            self.state_vector_t_minus_1) + (
            self.getB(self.est_yaw,self.delta_t)) @ (
            self.control_vector_t_minus_1) + (
            self.process_noise_v_t_minus_1)

        # Predict the state covariance estimate based on the previous
        # covariance and some noise
        P_t = self.A_t_minus_1 @ self.P_t_minus_1 @ self.A_t_minus_1.T + (
            self.Q_t)

        ################### Update (Correct) ##########################
        # Calculate the difference between the actual sensor measurements
        # at time t minus what the measurement model predicted
        # the sensor measurements would be for the current timestep t.
        measurement_residual_y_t = z_t_observation_vector - (
            (self.H_t @ state_estimate_t) + (
            self.sensor_noise_w_t))

        # Calculate the measurement residual covariance
        S_t = self.H_t @ P_t @ self.H_t.T + self.R_t

        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be
        # non-square or singular.
        K_t = P_t @ self.H_t.T @ np.linalg.pinv(S_t)

        # Calculate an updated state estimate for time t
        state_estimate_t = state_estimate_t + (K_t @ measurement_residual_y_t)

        # Update the state covariance estimate for time t
        P_t = P_t - (K_t @ self.H_t @ P_t)

        #### Update global variables for the next iteration of EKF ####
        # Update the estimated yaw
        self.est_yaw = state_estimate_t[2]

        # Update the state vector for t-1
        self.state_vector_t_minus_1 = state_estimate_t

        # Update the state covariance matrix
        self.P_t_minus_1 = P_t

        ######### Return the updated state estimate ###################
        state_estimate_t = state_estimate_t.tolist()
        return state_estimate_t


def main(args=None):
    """
    Entry point for the program.
    """

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    estimator = PlaceholderEstimator()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    # Publish any pending messages to the topics.
    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
