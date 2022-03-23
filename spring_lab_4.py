from sympy import Matrix, Symbol, symbols, solveset, solve, simplify, S, diff, det, erf, log, sqrt, pi, sin, cos, tan, asin, acos, atan2, init_printing, pretty_print
import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


#trying to adapt this for my kuka arm
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

# These numeric angles will be replaced with symbolic variables in this lab
theta1=-pi/2
theta2=-pi/4
theta3=3*pi/4
theta4=0
theta5=pi/2
theta6=0

# Define transforms to each joint
T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
T2 = T1 * T(0.096, 0, 0) * Rz(theta2)
T3 = T2 * T(0.205, 0, 0) * Rz(theta3)
T4 = T3 * T(0.124, 0, 0) * Rx(theta4)
T5 = T4 * T(0.167, 0, 0) * Rz(theta5)
T6 = T5 * T(0.104, 0, 0) * Rx(theta6)
print(T1)

# Find joint positions in space
p0 = Matrix([0,0,0,1])
p1 = T1 * p0
p2 = T2 * p0
p3 = T3 * p0
p4 = T4 * p0
p5 = T5 * p0
p6 = T6 * p0

theta1,theta2,theta3,theta4,theta5,theta6 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6')
theta = Matrix([theta1,theta2,theta3,theta4,theta5,theta6])

# Not necessary but gives nice-looking latex output
# More info at: http://docs.sympy.org/latest/tutorial/printing.html
init_printing()

print('T1=',T1,'\n\nT2=',T2,'\n\nT3=',T3,'\n\nT4=',T4,'\n\nT5=',T5,'\n\nT6=',T6)
print('p1=',p1,'\n\np2=',p2,'\n\np3=',p3,'\n\np4=',p4,'\n\np5=',p5,'\n\np6=',p6)

p06t = Matrix([0.45, 0, 0.308, 1])

def R(roll, pitch, yaw):
  R_x = Matrix([[1, 0, 0, 0],
              [0, cos(roll), -sin(roll), 0],
              [0, sin(roll),  cos(roll), 0],
              [0, 0, 0, 1]])
  R_y = Matrix([[ cos(pitch), 0, sin(pitch), 0],
              [0, 1, 0, 0],
              [-sin(pitch), 0, cos(pitch), 0],
              [0, 0, 0, 1]])
  R_z = Matrix([[cos(yaw),-sin(yaw), 0, 0],
              [sin(yaw), cos(yaw), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
  return R_z*R_y*R_x

R06t = R(0, 0, 0)
print(p06t)
print(R06t)

R03 = Rz(theta1+pi/2)*Rx(pi/2)
R36 = Rz(theta4)*Rx(theta5)*Rz(theta6)
#R03sub = R03.subs({theta1:theta_1})
#R36t = R03sub.inv() * R06t


p0 = Matrix([0,0,0,1])
p1 = T1 * p0
p2 = T2 * p0
p3 = T3 * p0
p4 = T4 * p0
p5 = T5 * p0
p6 = T6 * p0

theta1=-pi/2
theta2=-pi/4
theta3=3*pi/4
theta4=0
theta5=pi/2
theta6=0


soa = numpy.array([p0,p1,p2,p3,p4,p5,p6])
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
ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.show()
ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.draw()
matplotlib.pyplot.show()
matplotlib.pyplot.pause(1)


