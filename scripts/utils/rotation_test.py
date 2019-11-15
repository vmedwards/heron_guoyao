'''
This is a test script for rotation calibration for IMU on Heron
(c) Guoyao Shen
'''

import numpy as np
from numpy import sin, cos, pi

x = 0
y = 0
z = pi/2
# get rotation matrix of Euler roll-pitch-yaw
R = np.array([[cos(z)*cos(y), cos(z)*sin(y)*sin(x)-sin(z)*cos(x), sin(z)*sin(x)+cos(z)*sin(y)*cos(x)],
              [sin(z)*cos(y), sin(z)*sin(y)*sin(x)+cos(z)*cos(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)],
              [-sin(y), cos(y)*sin(x), cos(y)*cos(x)]])

# define the matrix for calibration:
# rotate around inertial frame y for 180 degree, pre-multiply
R_filter = np.array([[cos(pi), 0, sin(pi)],
                     [0, 1, 0],
                     [-sin(pi), 0, cos(pi)]])

R_calibrated = R_filter.dot(R)

Vin = np.array([[1],[0],[0]])
Vout_orig = R.dot(Vin)
Vout_calibrated = R_calibrated.dot(Vin)
print('Vout_orig', Vout_orig)
print('Vout_calibrated', Vout_calibrated)