import math

import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing
from forwardkinematics import d_h_table_0_1, d_h_table_1_2, d_h_table_2_3, d_h_table_3_4, d_h_table_4_5, d_h_table_5_6, end_effector_position
from forwardkinematics import transform_0_1, transform_1_2, transform_2_3, transform_3_4, transform_4_5, transform_5_6, transform_0_6


#desired end effector (target) position
x = 400
y = 200
z = 0

#calculating angle of second joint
theta1 = np.arctan(y/x)
print(f'theta 1 = {theta1} radians')

#desired orientation of end effector relative to base frame
#desired orientation
#rotation matrix of frame 6 relative to frame 0
#INPUT THIS DIRECTLY FROM FORWARD KINEMATICS
rotation_0_6 = transform_0_6[0:3, 0:3]
print(rotation_0_6)

#rotation matrix of frame 3 relative to frame 0
rotation_0_3 = transform_2_3[0:3,0:3]
print(f'rotation_0_3 = \n {rotation_0_3}')

#calculate inverse rotation matrix
#rotation_0_3 = rotation_0_3.astype('int16')
#int_array = np.array([rotation_0_3], dtype='int16')
int_array = np.array(rotation_0_3).astype(int)
#int.format(rotation_0_3)
inverse_rotation_0_3 = np.linalg.inv(int_array)
print(f'inverse_rotation_0_3 = \n {inverse_rotation_0_3}')

#calculate the rotation matrix of frame 6 relative to frame 3
rotation_3_6 = inverse_rotation_0_3 @ rotation_0_6
print(f'rotation_3_6 = \n {rotation_3_6}')

#extracting theta 5 from rotation matrix 3-6 (starting with this as it is the simplest value)
#it is in the 3rd column 3rd row
#theta_5 = np.arccos(rotation_3_6[2,2])
#print(f'\n theta 5 = {theta_5} radians')

#finding theta 6
#theta_6 = np.arccos(rotation_3_6[2,0] / -np.sin(theta_5))
#print(f'\n theta 6 = {theta_6} radians')

