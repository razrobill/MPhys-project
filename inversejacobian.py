import matplotlib.pyplot as plt
import numpy
import math
import matplotlib.pyplot
import numpy as np
import scipy
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, diff, det
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

plt.rcParams["font.family"] = 'Times New Roman'
plt.rcParams["figure.autolayout"] = True

#d-h parameters in mm
a1 = 500
a2 = 1300
a3 = -55
a4 = 0
a5 = 0
a6 = 0

d1 = 1045
d2 = 0
d3 = 0
d4 = 1025
d5 = 0
d6 = 290

#defining variables to be used as symbols
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
theta = Matrix([theta1, theta2, theta3, theta4, theta5, theta6])
init_printing()

alpha1 = 90*(pi/180)
alpha2 = 0*(pi/180)
alpha3 = 90*(pi/180)
alpha4 = -90*(pi/180)
alpha5 = 90*(pi/180)
alpha6 = 0*(pi/180)

#d-h parameters table

p0 = Matrix([0, 0, 0, 1])
p0x = (p0[0])
p0y = (p0[1])
p0z = (p0[2])
#print(p0)


d_h_table_0_1 = Matrix([[cos(theta1), -sin(theta1)*cos(alpha1), sin(theta1)*sin(alpha1), a1*cos(theta1)],
                        [sin(theta1), cos(theta1)*cos(alpha1), -cos(theta1)*sin(alpha1), a1*sin(theta1)],
                        [0, sin(alpha1), cos(alpha1), d1],
                        [0, 0, 0, 1]])

transform_0_1 = d_h_table_0_1
last_column1 = transform_0_1[:, 3]
deletion1 = np.delete(last_column1, 3, 0)
p1 = Matrix(deletion1)
print('p1 equals: ')
print(p1)
p1x = np.unique(p1[0])
p1y = np.unique(p1[1])
p1z = np.unique(p1[2])




d_h_table_1_2 = np.array([[cos(theta2), -sin(theta2)*cos(alpha2), sin(theta2)*sin(alpha2), a2*cos(theta2)],
                        [sin(theta2), cos(theta2)*cos(alpha2), -cos(theta2)*sin(alpha2), a2*sin(theta2)],
                        [0, sin(alpha2), cos(alpha2), d2],
                        [0, 0, 0, 1]])

transform_1_2 = transform_0_1 @ d_h_table_1_2
last_column2 = transform_1_2[:, 3]
deletionp2 = np.delete(last_column2, 3, 0)
p2 = Matrix(deletionp2)
print(p2)
p2x = np.unique(p2[0])
p2y = np.unique(p2[1])
p2z = np.unique(p2[2])



d_h_table_2_3 = np.array([[cos(theta3), -sin(theta3)*cos(alpha3), sin(theta3)*sin(alpha3), a3*cos(theta3)],
                          [sin(theta3), cos(theta3)*cos(alpha3), -cos(theta3)*sin(alpha3), a3*sin(theta3)],
                          [0, sin(alpha3), cos(alpha3), d3],
                          [0, 0, 0, 1]])

transform_2_3 = transform_1_2 @ d_h_table_2_3
last_column3 = transform_2_3[:, 3]
p3 = np.delete(last_column3, 3, 0)
#print(p3)
p3 = Matrix(p3)
p3x = np.unique(p3[0])
p3y = np.unique(p3[1])
p3z = np.unique(p3[2])


d_h_table_3_4 = np.array([[cos(theta4), -sin(theta4)*cos(alpha4), sin(theta4)*sin(alpha4), a4*cos(theta4)],
                          [sin(theta4), cos(theta4)*cos(alpha4), -cos(theta4)*sin(alpha4), a4*sin(theta4)],
                          [0, sin(alpha4), cos(alpha4), d4],
                          [0, 0, 0, 1]])

transform_3_4 = transform_2_3 @ d_h_table_3_4
last_column4 = transform_3_4[:, 3]
p4 = np.delete(last_column4, 3, 0)
#print(p4)
p4 = Matrix(p4)
p4x = np.unique(p4[0])
p4y = np.unique(p4[1])
p4z = np.unique(p4[2])

d_h_table_4_5 = np.array([[cos(theta5), -sin(theta5)*cos(alpha5), sin(theta5)*sin(alpha5), a5*cos(theta5)],
                          [sin(theta5), cos(theta5)*cos(alpha5), -cos(theta5)*sin(alpha5), a5*sin(theta5)],
                          [0, sin(alpha5), cos(alpha5), d5],
                          [0, 0, 0, 1]])

transform_4_5 = transform_3_4 @ d_h_table_4_5
last_column5 = transform_4_5[:, 3]
p5 = np.delete(last_column5, 3, 0)
#print(p5)
p5 = Matrix(p5)
p5x = np.unique(p5[0])
p5y = np.unique(p5[1])
p5z = np.unique(p5[2])

d_h_table_5_6 = np.array([[cos(theta6), -sin(theta6)*cos(alpha6), sin(theta6)*sin(alpha6), a6*cos(theta6)],
                          [sin(theta6), cos(theta6)*cos(alpha6), -cos(theta6)*sin(alpha6), a6*sin(theta6)],
                          [0, sin(alpha6), cos(alpha6), d6],
                          [0, 0, 0, 1]])

transform_5_6 = transform_4_5 @ d_h_table_5_6
last_column6 = transform_5_6[:, 3]
p6 = np.delete(last_column6, 3, 0)
p6 = Matrix(p6)
p6x = np.unique(p6[0])
p6y = np.unique(p6[1])
p6z = np.unique(p6[2])


#multiplying to find transformation from frame 0 to 6
#(@ symbol used for matrix multiplication)

transform_0_6 = d_h_table_0_1 @ d_h_table_1_2 @ d_h_table_2_3 @ d_h_table_3_4 @ d_h_table_4_5 @ d_h_table_5_6

init_printing()

print('T1=',transform_0_1,'\n\nT2=',transform_1_2,'\n\nT3=',transform_2_3,'\n\nT4=',transform_3_4,'\n\nT5=',transform_4_5,'\n\nT6=',transform_5_6)
print('p1=',p1,'\n\np2=',p2,'\n\np3=',p3,'\n\np4=',p4,'\n\np5=',p5,'\n\np6=',p6)

#coordinates of arm tip
#
p = Matrix([p6[0], p6[1], p6[2]])

j11 = diff(p[0], theta1) # differentiate px with theta_1
j12 = diff(p[0], theta2) # differentiate px with theta_2
j13 = diff(p[0], theta3) # differentiate px with theta_3
j14 = diff(p[0], theta4) # differentiate px with theta_4
j15 = diff(p[0], theta5) # differentiate px with theta_5
j16 = diff(p[0], theta6) # differentiate px with theta_6

j21 = diff(p[1], theta1) # differentiate py with theta_1
j22 = diff(p[1], theta2) # differentiate py with theta_2
j23 = diff(p[1], theta3) # differentiate py with theta_3
j24 = diff(p[1], theta4) # differentiate py with theta_4
j25 = diff(p[1], theta5) # differentiate py with theta_5
j26 = diff(p[1], theta6) # differentiate py with theta_6

j31 = diff(p[2], theta1) # differentiate pz with theta_1
j32 = diff(p[2], theta2) # differentiate pz with theta_2
j33 = diff(p[2], theta3) # differentiate pz with theta_3
j34 = diff(p[2], theta4) # differentiate pz with theta_4
j35 = diff(p[2], theta5) # differentiate pz with theta_5
j36 = diff(p[2], theta6) # differentiate pz with theta_6

J = Matrix([[j11, j12, j13, j14, j15, j16], [j21, j22, j23, j24, j25, j26], [j31, j32, j33, j34, j35, j36]]) # assemble into matrix form

print(J.shape)
print(J)

#initial theta values, perhaps set these as for forward kinematics
theta_i = Matrix([0,pi/2,0,0,0,0])

p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()

#p_i is the initial position of the end effector (same as p6)
print(p_i)

#final (target) point of the end effector, defined as a relative movement from the initial position, for example moving
#the arm down in the z-axis by 5cm
p_f = p_i + Matrix([0, 0, 500])

dp = p_f - p_i

dp_threshold = 5
dp_step = 1
theta_max_step = 0.5
j = 0
max_steps = 500

while dp.norm() > dp_threshold and j < max_steps:
    print(f'step{j}: θ[{theta_i}, P[{p_i}]')
    #reduce the dp 3-element dp vector by some scaling factor
    #dp represents the distance between where the end effector is now and our goal position
    v_p = dp * dp_step / dp.norm()
    J = p.jacobian(theta)
    J_i = J.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                  theta6: theta_i[5]}).evalf()
    J_inv = J_i.pinv()
    dtheta = J_inv * v_p
    theta_i = theta_i + np.clip(dtheta, -1 * theta_max_step, theta_max_step)
    p_i = p.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                  theta6: theta_i[5]}).evalf()
    dp = p_f - p_i
    j = j + 1

    p0sub = p0.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p1sub = p1.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p2sub = p2.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p3sub = p3.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p4sub = p4.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p5sub = p5.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    p6sub = p6.subs({theta1: theta_i[0], theta2: theta_i[1], theta3: theta_i[2], theta4: theta_i[3], theta5: theta_i[4],
                     theta6: theta_i[5]}).evalf()
    soa = np.array([p0sub, p1sub, p2sub, p3sub, p4sub, p5sub, p6sub], dtype=object)
    soa2 =np.array([p6sub])
    X, Y, Z, = zip(*soa)
    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)
    X = np.ndarray.flatten(X)
    Y = np.ndarray.flatten(Y)
    Z = np.ndarray.flatten(Z)
    X2, Y2, Z2, = zip(*soa2)
    X2 = np.array(X2)
    Y2 = np.array(Y2)
    Z2 = np.array(Z2)
    X2 = np.ndarray.flatten(X2)
    Y2 = np.ndarray.flatten(Y2)
    Z2 = np.ndarray.flatten(Z2)
    fig = matplotlib.pyplot.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    matplotlib.pyplot.xticks(fontsize=9)
    ax.set_xlim([-2000, 2000])
    ax.set_ylim([0, 2000])
    ax.set_zlim([0, 3000])
    #ax.view_init(elev=30, azim=-60)
    #ax.view_init(elev=45, azim=45)
    ax.view_init(elev=10, azim=90)
    ax.plot3D(X2, Y2, Z2, 'red', marker="^", zorder=4)
    ax.scatter(X2, Y2, Z2, color='red', marker='^', label="End effector position", zorder=1)
    ax.plot3D(X, Y, Z, 'darkorange', marker="o", zorder=3)
    ax.scatter(X, Y, Z, color='darkorange', marker="o", label="Joint positions", zorder=2)
    plt.title("Initial Configuration of KUKA KR 360, with End Effector "
              "\n moved 500mm upwards through Inverse Kinematics")
    plt.legend(bbox_to_anchor=(1.05, 1), loc="upper right")
    plt.savefig('inverse_plot3.png')
    matplotlib.pyplot.draw()
    matplotlib.pyplot.show()
    matplotlib.pyplot.pause(0.1)



theta__2 = int(theta_i.evalf()[1])
rot_mat_0_3 = np.array([[0, np.sin(theta__2), np.cos(theta__2)],
                        [0, - np.cos(theta__2), np.sin(theta__2)],
                        [1, 0, 0]])
#print('rotmat_0_3: ', rot_mat_0_3)
inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)

theta__0 = 0
theta__1 = int(theta_i.evalf()[0])
theta__3 = (theta_i.evalf()[2])
#print('theta__3: ', theta__3)
theta__4 = int(theta_i.evalf()[3])
theta__5 = int(theta_i.evalf()[4])
theta__6 = int(theta_i.evalf()[5])

rot_mat_0_1 = np.array([[np.cos(theta__0), 0, np.sin(theta__0)],
                       [np.sin(theta__0), 0, - np.cos(theta__0)],
                       [0, 1, 0]])

rot_mat_1_2 = np.array([[- np.sin(theta__1), np.cos(theta__1), 0],
                       [np.cos(theta__1), np.sin(theta__1), 0],
                       [0, 0, 1]])

rot_mat_2_3 = np.array([[np.cos(theta__2), 0, - np.sin(theta__2)],
                       [np.sin(theta__2), 0, np.cos(theta__2)],
                       [0, 1, 0]])

rot_mat_3_4 = np.array([[math.cos(theta__3), 0, - math.sin(theta__3)],
                       [math.sin(theta__3), 0, math.cos(theta__3)],
                       [0, 1, 0]])

rot_mat_4_5 = np.array([[np.cos(theta__4), 0,  - np.sin(theta__4)],
                        [np.sin(theta__4), 0, np.cos(theta__4)],
                        [0, 1, 0]])

rot_mat_5_6 = np.array([[np.cos(theta__5), - np.sin(theta__5), 0],
                       [np.sin(theta__5), np.cos(theta__5), 0],
                       [0, 0, 1]])

#rot_mat_0_6 = rot_mat_0_1 @ rot_mat_1_2 @ rot_mat_2_3 @ rot_mat_3_4 @ rot_mat_4_5 @ rot_mat_5_6


#this defines the orientation when the joint angles above are used
print("rot_mat_0_6: ")
#print(rot_mat_0_6)

#to define the orientation of the end effector, rot_mat_0_6 can be changed
#for example:

#end effector pointed upwards towards sky
#rot_mat_0_6 = np.array([[-1, 0, 0],
                        #[0, -1, 0],
                        #[0, 0, 1]])


#end effector pointed downwards?? this needs verifying
rot_mat_0_6 = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, -1]])


rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6
rot_mat_3_6 = rot_mat_3_6.round()
#print(rot_mat_3_6)

new_theta5 = np.arccos(rot_mat_3_6[2,2])
#print(f'theta 5 = {new_theta5} radians')

new_theta6 = np.arccos(rot_mat_3_6[2,0]/np.sin(new_theta5))
#print(f'theta 6 = {new_theta6} radians')

new_theta4 = np.arccos(rot_mat_3_6[1,2]/-np.sin(new_theta5))
#print(f'theta 4 = {new_theta4} radians')

first_three_angles = (theta_i.evalf()[:3])

first_angle = (theta_i.evalf()[0])
second_angle = (theta_i.evalf()[1])
third_angle = (theta_i.evalf()[2])

fourth_angle = new_theta4
fifth_angle = new_theta5
sixth_angle = new_theta6

print(first_angle)
print(second_angle)
print(third_angle)
print(fourth_angle)
print(fifth_angle)
print(sixth_angle)


print('\n\nFinal Joint Angles in Radians:\n', theta_i.evalf())
