import numpy as np
from sympy import S, erf, log, sqrt, pi, sin, cos, tan




#desired end effector orientation
x = 1510
y = 0
z = 2290

theta0 = 0
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
theta6 = 0

theta0 = np.deg2rad(theta0)
theta1 = np.deg2rad(theta1)
theta2 = np.deg2rad(theta2)
theta3 = np.deg2rad(theta3)
theta4 = np.deg2rad(theta4)
theta5 = np.deg2rad(theta5)
theta6 = np.deg2rad(theta6)

#these are the rotation matrices for the workspace diagram configuration, with the theta's being for the 'original' position
rot_mat_0_1 = np.array([[np.cos(theta0), 0, np.sin(theta0)],
                       [np.sin(theta0), 0, - np.cos(theta0)],
                       [0, 1, 0]])

rot_mat_1_2 = np.array([[- np.sin(theta1), np.cos(theta1), 0],
                       [np.cos(theta1), np.sin(theta1), 0],
                       [0, 0, 1]])

rot_mat_2_3 = np.array([[np.cos(theta2), 0, - np.sin(theta2)],
                       [np.sin(theta2), 0, np.cos(theta2)],
                       [0, 1, 0]])

rot_mat_3_4 = np.array([[np.cos(theta3), 0, - np.sin(theta3)],
                       [np.sin(theta3), 0, np.cos(theta3)],
                       [0, 1, 0]])

rot_mat_4_5 = np.array([[np.cos(theta4), 0,  - np.sin(theta4)],
                        [np.sin(theta4), 0, np.cos(theta4)],
                        [0, 1, 0]])

rot_mat_5_6 = np.array([[np.cos(theta5), - np.sin(theta5), 0],
                       [np.sin(theta5), np.cos(theta5), 0],
                       [0, 0, 1]])

rot_mat_0_6 = rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @ rot_mat_3_4 @ rot_mat_4_5 @ rot_mat_5_6

#can verify this is as expected
print(rot_mat_0_6)





#new_theta2 = np.arctan2(y,x)
new_theta2 = 1.57


rot_mat_0_3 = np.array([[0, np.sin(new_theta2), np.cos(new_theta2)],
                        [0, - np.cos(new_theta2), np.sin(new_theta2)],
                        [1, 0, 0]])

inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6
rot_mat_3_6 = rot_mat_3_6.round()
print(rot_mat_3_6)

#solving for theta5
#third row third column
new_theta5 = np.arccos(rot_mat_3_6[2,2])
print(f'theta 5 = {new_theta5} radians')

#solving for theta6
#third row, first column
#sin(theta5)cos(theta6) = rot_mat_3_6[2,0]
#rot_mat_3_6[2,0]/sin(theta5) = cos(theta6)
new_theta6 = np.arccos(rot_mat_3_6[2,0]/np.sin(new_theta5))
print(f'theta 6 = {new_theta6} radians')

#solving for theta4
#second row, third column
#-sin(theta5)sin(theta4) = rot_mat_3_6[1,2]
#rot_mat_3_6[1,2]/-sin(theta5) = sin(theta4)
new_theta4 = np.arccos(rot_mat_3_6[1,2]/-np.sin(new_theta5))
print(f'theta 4 = {new_theta4} radians')

#checking that the angles we calculated result in a valid rotation matrix
r11 = np.cos(new_theta5) * np.cos(new_theta6) * np.cos(new_theta4) - np.sin(new_theta6) * np.sin(new_theta4)
r12 = - np.cos(new_theta5) * np.sin(new_theta6) * np.cos(new_theta4) - np.cos(new_theta6) * np.sin(new_theta4)
r13 = - np.sin(new_theta5) * np.cos(new_theta4)
r21 = np.cos(new_theta5) * np.cos(new_theta6) * np.sin(new_theta4) + np.sin(new_theta6) * np.cos(new_theta4)
r22 = np.cos(new_theta6) * np.cos(new_theta4) - np.cos(new_theta5) * np.sin(new_theta6) * np.sin(new_theta4)
r23 = - np.sin(new_theta5) * np.sin(new_theta4)
r31 = np.sin(new_theta5) * np.cos(new_theta6)
r32 = - np.sin(new_theta5) * np.sin(new_theta6)
r33 = np.cos(new_theta5)

check_rot_mat_3_6 = np.array([[r11, r12, r13],
                              [r21, r22, r23],
                              [r31, r32, r33]])
check_rot_mat_3_6 = check_rot_mat_3_6.round()
print(check_rot_mat_3_6)

#original matrix
print(f'\nrot_mat_3_6 = {rot_mat_3_6}')

#check matrix
print(f'\ncheck_rot_mat_3_6 = {check_rot_mat_3_6}')

#return if original matrix == check matrix
rot_minus_check_3_6 = rot_mat_3_6.round() - check_rot_mat_3_6.round()
print(rot_minus_check_3_6)
zero_matrix = np.array([[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])
matrices_are_equal = np.array_equal(rot_minus_check_3_6, zero_matrix)

#determine if the solution is valid or not
#if the solution is valid, it means that the end effector can reach the target location
if(matrices_are_equal):
    valid_matrix = "Yes"
else:
    valid_matrix = "No"
print(f'Is solution valid?\n{valid_matrix}')




