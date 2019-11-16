'''
This is a test script for rotation calibration for IMU on Heron
(c) Guoyao Shen
'''

import numpy as np
from numpy import sin, cos, pi, arctan2, arcsin, arccos

x = 1e-6
y = 1e-6
z = 1e-6
# x = 0
# y = 0
# z = 0
# get rotation matrix of Euler roll-pitch-yaw
R = np.array([[cos(z)*cos(y), cos(z)*sin(y)*sin(x)-sin(z)*cos(x), sin(z)*sin(x)+cos(z)*sin(y)*cos(x)],
              [sin(z)*cos(y), sin(z)*sin(y)*sin(x)+cos(z)*cos(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)],
              [-sin(y), cos(y)*sin(x), cos(y)*cos(x)]])

# get rotation matrix of Euler yaw-pitch-roll
R2 = np.array([[cos(y)*cos(z), -cos(y)*sin(z), sin(y)],
              [cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x)],
              [sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y)]])

# ================================================================ roll-pitch-yaw
# define the matrix for calibration:
# rotate around inertial frame y for 180 degree, pre-multiply
R_filter = np.array([[cos(pi), 0, sin(pi)],
                     [0, 1, 0],
                     [-sin(pi), 0, cos(pi)]])

R_calibrated = R_filter.dot(R)
print(R_calibrated)

# get new roll-pitch-yaw angle from calibrated matrix
y_c = arcsin(-R_calibrated[2,0])+1e-5

z_c = arcsin(R_calibrated[1,0]/cos(y_c))
z_c2 = arccos(R_calibrated[0,0]/cos(y_c))
z_c3 = arctan2(R_calibrated[1,0], R_calibrated[0,0])

x_c = arcsin(R_calibrated[2,1]/cos(y_c))
x_c2 = arccos(R_calibrated[2,2]/cos(y_c))
x_c3 = arctan2(R_calibrated[2,1], R_calibrated[2,2])
# print(R_calibrated[2,1]/cos(y_c))
# print(R_calibrated[2,2]/cos(y_c))

# ================================================================ yaw-pitch-roll
# define the matrix for calibration:
# rotate around inertial frame y for 180 degree, pre-multiply
R2_filter = np.array([[cos(pi), 0, sin(pi)],
                     [0, 1, 0],
                     [-sin(pi), 0, cos(pi)]])

R2_calibrated = R2_filter.dot(R2)
print(R2_calibrated)

# get new roll-pitch-yaw angle from calibrated matrix
y2_c = arcsin(R2_calibrated[0,2])+1e-5

z2_c = arcsin(-R2_calibrated[0,1]/cos(y2_c))
z2_c2 = arccos(R2_calibrated[0,0]/cos(y2_c))
z2_c3 = arctan2(-R2_calibrated[0,1], R2_calibrated[0,0])

x2_c = arcsin(-R2_calibrated[1,2]/cos(y2_c))
x2_c2 = arccos(R2_calibrated[2,2]/cos(y2_c))
x2_c3 = arctan2(-R2_calibrated[1,2], R2_calibrated[2,2])
# print(R2_calibrated[2,1]/cos(y2_c))
# print(R2_calibrated[2,2]/cos(y2_c))

if __name__ == '__main__':
    # Vin = np.array([[1],[0],[0]])
    # Vout_orig = R.dot(Vin)
    # Vout_calibrated = R_calibrated.dot(Vin)
    # print('Vout_orig', Vout_orig)
    # print('Vout_calibrated', Vout_calibrated)

    print('y_c', y_c, -R_calibrated[2,0])

    print('x_c arcsin', x_c, ' | ', R_calibrated[2,1]/cos(y_c))
    print('x_c2 arccos', x_c2, ' | ', R_calibrated[2,2]/cos(y_c))
    print('x_c3 arctan2', x_c3, ' | ', R_calibrated[2,1]/R_calibrated[2,2])

    print('z_c arcsin', z_c, ' | ', R_calibrated[1,0]/cos(y_c))
    print('z_c2 arccos', z_c2, ' | ', R_calibrated[0,0]/cos(y_c))
    print('z_c3 arctan2', z_c3, ' | ', R_calibrated[1,0], R_calibrated[0,0])

    print('============================')

    print('y2_c', y2_c, -R2_calibrated[2, 0])

    print('x2_c arcsin', x2_c, ' | ', -R2_calibrated[1, 2] / cos(y2_c))
    print('x2_c2 arccos', x2_c2, ' | ', R2_calibrated[2, 2] / cos(y2_c))
    print('x2_c3 arctan2', x2_c3, ' | ', -R2_calibrated[1, 2] / R2_calibrated[2, 2])

    print('z2_c arcsin', z2_c, ' | ', -R2_calibrated[0, 1] / cos(y2_c))
    print('z2_c2 arccos', z2_c2, ' | ', R2_calibrated[0, 0] / cos(y2_c))
    print('z2_c3 arctan2', z2_c3, ' | ', -R2_calibrated[0, 1], R2_calibrated[0, 0])