#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FILE:  safe_imu_sensor.py

# DOC:  Mutex Protected, Exception Tolerant Interface for the DI BNO055 IMU

"""
The BNO055 IMU requires I2C clock-stretching, which is available on the
GoPiGo3 robot by software I2C through the AD1 and AD2 ports.

di_sensors provides an I2C mutex which must be used when more than one
I2C client is active, such as a GoPiGo3 with the BNO055 IMU and 
a DI V53L0X Distance Sensor.

The DI clock-stretching software I2C sometimes throws an exception
which must be handled in the BNO055 driver to continue access to the IMU.

This ROS node was originally created by Bernardo Japon for the
Hands On ROS for Robotics Programming book,
and is modified to use the imu4gopigo3ros "Safe IMU For GoPiGo3" PyPi package

This node assumes (defaults): 
  - The DI BNO055 IMU is connected to the GoPiGo3 AD1 port
  - NDOF mode is desired which uses accelerometers, gyros, and magnetometers.

The node assumes the IMU is oriented:
  - Chip side up, arrow head pointing to left side of bot (viewed from rear)
  - X is forward
  - Y is toward left side
  - Z is up

For Python2 install the imu4gopigo3ros package:
$ sudo pip install imu4gopigo3ros

For Python3 install the imu4gopigo3ros2 package:
$ sudo pip install imu4gopigo3ros2

To test IMU access:
$ startIMU
$ readIMU

To remove the Python2 imu4gopigo3ros package:
$ sudo pip uninstall imu4gopigo3ros

To remove the Python3 imu4gopigo3ros2 package:
$ sudo pip3 uninstall imu4gopigo3ros2

Publishes:
   /imu/data
   /imu/mag  magnetic_field in unit of full Tesla 
   /temp
TODO:
   /imu/status
   /imu/raw

ROS Parameters:  
  - port: default "AD1" or "AD2"
  - mode: default "NDOF" or "IMUPLUS" (accelerometers and gyros, no mags)

"""
import rospy
# from di_sensors.inertial_measurement_unit import InertialMeasurementUnit
from ros_safe_inertial_measurement_unit import SafeIMUSensor
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Header
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from datetime import datetime as dt
# TODO: from diagnostic_msgs import DiagnosticStatus, DiagnosticArray, KeyValue


# Set to True for debug prints to stdout
VERBOSITY = False

def main():
    rospy.init_node("imu")

    # port = rospy.get_param("~port", "GPG3_AD1")
    # Changed allowable port parameter to AD1 or AD2
    parm_port = rospy.get_param("~port", "AD1")

    # Default to NDOF mode but allow for parameter "IMUPLUS"
    parm_mode = rospy.get_param("~mode", "NDOF")

    if (parm_mode == "IMUPLUS"):
        imu_mode = rosBNO055.OPERATION_MODE_IMUPLUS
    else:
        imu_mode = rosBNO055.OPERATION_MODE_NDOF

    # Keep track of soft IMU exceptions
    imu_exceptionCnt = 0

    # sensor = InertialMeasurementUnit(bus=port)
    sensor = SafeIMUSensor(use_mutex=True, port=parm_port, mode=parm_mode, verbose=VERBOSITY, init=True)

    pub_imu = rospy.Publisher("~imu", Imu, queue_size=10)
    pub_temp = rospy.Publisher("~temp", Temperature, queue_size=10)
    pub_magn = rospy.Publisher("~magnetometer", MagneticField, queue_size=10)
    # TODO: pub_diag = rospy.Publisher("~diagnostics", DiagnosticStatus, queue_size=10)

    br = TransformBroadcaster()

    msg_imu = Imu()
    msg_temp = Temperature()
    msg_magn = MagneticField()
    # TODO: msg_diag = DiagnosticStatus()

    hdr = Header(stamp=rospy.Time.now(), frame_id="IMU")

    rate = rospy.Rate(rospy.get_param('~hz', 30))
    while not rospy.is_shutdown():
        q = sensor.safe_read_quaternion()        # x,y,z,w
        mag = sensor.safe_read_magnetometer()    # micro Tesla (µT)
        gyro = sensor.safe_read_gyroscope()      # deg/second
        accel = sensor.safe_read_accelerometer() # m/s²
        temp = sensor.safe_read_temperature()    # °C
        # TODO: cal = sensor.safe_sgam_calibration_status()  # sysCal, gyroCal, accCal, magCal none:0 - fully:3

        msg_imu.header = hdr

        hdr.stamp = rospy.Time.now()

        msg_temp.header = hdr
        msg_temp.temperature = temp
        # pub_temp.publish(msg_temp)  # moved to exception check

        msg_imu.header = hdr
        msg_imu.linear_acceleration.x = accel[0]
        msg_imu.linear_acceleration.y = accel[1]
        msg_imu.linear_acceleration.z = accel[2]
        msg_imu.angular_velocity.x = math.radians(gyro[0])
        msg_imu.angular_velocity.y = math.radians(gyro[1])
        msg_imu.angular_velocity.z = math.radians(gyro[2])
        msg_imu.orientation.w = q[3]
        msg_imu.orientation.x = q[0]
        msg_imu.orientation.y = q[1]
        msg_imu.orientation.z = q[2]
        # pub_imu.publish(msg_imu)

        msg_magn.header = hdr
        msg_magn.magnetic_field.x = mag[0]*1e-6
        msg_magn.magnetic_field.y = mag[1]*1e-6
        msg_magn.magnetic_field.z = mag[2]*1e-6
        # pub_magn.publish(msg_magn)

        # TODO: build put calibration info in msg_diag

        transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="world"), child_frame_id="IMU")
        transform.transform.rotation = msg_imu.orientation
        br.sendTransformMessage(transform)

        # track soft I2C exceptions, do not publish if occurred
        current_exCnt = sensor.getExceptionCount()
        if (current_exCnt != imu_exceptionCnt):
            imu_exceptionCnt = current_exCnt
            print("\n{}: IMU Exception Count: {}".format(dt.now(),imu_exceptionCnt))
        else:
            # no exception so publish data
            pub_temp.publish(msg_temp)
            pub_imu.publish(msg_imu)
            pub_magn.publish(msg_magn)
            # TODO: pub_diag.publish(msg_diag)

            transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="world"), child_frame_id="IMU")
            transform.transform.rotation = msg_imu.orientation
            br.sendTransformMessage(transform)



        rate.sleep()


if __name__ == '__main__':
    main()
