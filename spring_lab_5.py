import numpy as np
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, asin, acos, atan2, init_printing

def T(x, y, z):
   T_xyz = Matrix([[1,         0,          0,          x],
                   [0,         1,          0,          y],
                   [0,         0,          1,          z],
                   [0,         0,          0,          1]])
   return T_xyz

def Rx(roll):
   R_x = Matrix([[1,         0,          0, 0],
                 [0, cos(roll), -sin(roll), 0],
                 [0, sin(roll),  cos(roll), 0],
                 [0,         0,          0, 1]])
   return R_x

def Ry(pitch):
   R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
                 [          0, 1,          0, 0],
                 [-sin(pitch), 0, cos(pitch), 0],
                 [          0, 0,          0, 1]])
   return R_y

def Rz(yaw):
   R_z = Matrix([[cos(yaw),-sin(yaw), 0, 0],
                 [sin(yaw), cos(yaw), 0, 0],
                 [       0,        0, 1, 0],
                 [       0,        0, 0, 1]])
   return R_z

#theta1=-pi/2
#theta2=-pi/4
#theta3=3*pi/4
#theta4=0
#theta5=pi/2
#theta6=0

theta1,theta2,theta3,theta4,theta5,theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])



# Define transforms to each joint
T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
T6 = T5 * T(0.104, 0, 0) * Rx(theta6)


#T1 prints out a matrix with coordinates to be extracted for the position of each point
print(T1)

# Find joint positions in space
p0 = Matrix([0,0,0,1])
p1 = T1 * p0
p2 = T2 * p0
p3 = T3 * p0
p4 = T4 * p0
p5 = T5 * p0
p6 = T6 * p0


# Not necessary but gives nice-looking latex output
# More info at: http://docs.sympy.org/latest/tutorial/printing.html
init_printing()

print('T1=',T1,'\n\nT2=',T2,'\n\nT3=',T3,'\n\nT4=',T4,'\n\nT5=',T5,'\n\nT6=',T6)
print('p1=',p1,'\n\np2=',p2,'\n\np3=',p3,'\n\np4=',p4,'\n\np5=',p5,'\n\np6=',p6)

p = Matrix([p6[0], p6[1], p6[2]]) # coordinates of arm tip

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

theta_i = Matrix([0,0,0,0,0,0])

p_i = p.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
#p_i is the initial position of the end effector (same as p6)
print(p_i)

#final (target) point of the end effector, defined as a relative movement from the initial position, for example moving
#the arm down in the z-axis by 1cm
p_f = p_i + Matrix([0, 0, -0.1])

dp = p_f - p_i

dp_threshold = 0.001
dp_step = 0.0005
theta_max_step = 0.2
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

    #print("step “,step,”:\n θ[", theta_i, "]\n p[", p_i, "]")






pp0sub = p0.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p1sub = p1.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p2sub = p2.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p3sub = p3.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p4sub = p4.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p5sub = p5.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
p6sub = p6.subs({theta1:theta_i[0], theta2:theta_i[1], theta3:theta_i[2], theta4:theta_i[3], theta5:theta_i[4], theta6:theta_i[5]}).evalf()
soa = numpy.array([p0sub,p1sub,p2sub,p3sub,p4sub,p5sub,p6sub])
X, Y, Z, W = zip(*soa)
X = numpy.array(X)
Y = numpy.array(Y)
Z = numpy.array(Z)
W = numpy.array(W)
X = numpy.ndarray.flatten(X)
Y = numpy.ndarray.flatten(Y)
Z = numpy.ndarray.flatten(Z)
W = numpy.ndarray.flatten(W)
fig = matplotlib.pyplot.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([0, 1])
ax.view_init(elev=45, azim=45)
ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.draw()
matplotlib.pyplot.show()
matplotlib.pyplot.pause(0.1)




