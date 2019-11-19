#!/usr/bin/env python
'''
This is a test script for rotation calibration for IMU on Heron
(c) Guoyao Shen
'''

import rospy
import sys
import time

import numpy as np
from numpy import sin, cos, pi, arctan2, arcsin, arccos

from geometry_msgs.msg import Vector3Stamped

# initiate calibrated imu rpy data
rpy_c = Vector3Stamped()

def cb_imu_calibrate(rpy_data):
    '''
    Callback function to calibrate "/imu/rpy" topic data
    '''
    global rpy_c

    x = rpy_data.vector.x
    y = rpy_data.vector.y
    z = rpy_data.vector.z

    # get rotation matrix of Euler roll-pitch-yaw
    R = np.array(
        [[cos(z) * cos(y), cos(z) * sin(y) * sin(x) - sin(z) * cos(x), sin(z) * sin(x) + cos(z) * sin(y) * cos(x)],
         [sin(z) * cos(y), sin(z) * sin(y) * sin(x) + cos(z) * cos(x), -cos(z) * sin(x) + sin(z) * sin(y) * cos(x)],
         [-sin(y), cos(y) * sin(x), cos(y) * cos(x)]])

    # define the matrix for calibration:
    # rotate around inertial frame y for 180 degree, pre-multiply
    R_filter = np.array([[cos(pi), 0, sin(pi)],
                         [0, 1, 0],
                         [-sin(pi), 0, cos(pi)]])

    R_calibrated = R_filter.dot(R)

    # get new roll-pitch-yaw angle from calibrated matrix
    y_c = arcsin(-R_calibrated[2, 0]) + 1e-5

    # z_c = arcsin(R_calibrated[1, 0] / cos(y_c))
    # z_c2 = arccos(R_calibrated[0, 0] / cos(y_c))
    z_c = arctan2(R_calibrated[1, 0], R_calibrated[0, 0])

    # x_c = arcsin(R_calibrated[2, 1] / cos(y_c))
    # x_c2 = arccos(R_calibrated[2, 2] / cos(y_c))
    x_c = arctan2(R_calibrated[2, 1], R_calibrated[2, 2])

    # publish calibrated imu rpy data
    imu_calibrated_publ = rospy.Publisher('/imu/calibrated_rpy', Vector3Stamped, queue_size=100)
    rpy_c.vector.x = x_c
    rpy_c.vector.y = y_c
    rpy_c.vector.z = z_c
    imu_calibrated_publ.publish(rpy_c)
    # print('x_calibrated', x_c)
    # print('y_calibrated', y_c)
    # print('z_calibrated', z_c)


def imu_calibration_publisher():
    rospy.init_node('imu_calibrated_publisher', anonymous=True)
    rospy.Subscriber("/imu/rpy", Vector3Stamped, cb_imu_calibrate)


if __name__ == '__main__':
    imu_calibration_publisher()