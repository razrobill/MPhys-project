import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

#kinematics simulator

#p0 to p6 are points in 3D space defined as symbolic matrices that can be transformed mathematically
#note that they are 4 dimensional so they can be used with homogenous transforms with the last 'W' coordinate always 1
#(to facilitate translation as well as rotation)
p0 = Matrix([0, 0, 0, 1])
p1 = Matrix([0, 0, 0.187, 1])
p2 = Matrix([0, 0, 0.187+0.096, 1])
p3 = Matrix([0.205, 0, 0.187+0.096+0.205, 1])
p4 = Matrix([0.205+0.124, 0, 0.187+0.096+0.205, 1])
p5 = Matrix([0.205+0.124+0.167, 0, 0.187+0.096+0.205, 1])
p6 = Matrix([0.205+0.124+0.167, 0, 0.187+0.096+0.205-0.215, 1])

#concatenate these points into a column
soa = numpy.array([p0, p1, p2, p3, p4, p5, p6])
print(soa)

#extract columns from soa and make them column vectors of X,Y and Z components (W unused for 3D projection = 1)
X, Y, Z, W = zip(*soa)

#ensure a flat set of vectors result
X = numpy.array(X)
Y = numpy.array(Y)
Z = numpy.array(Z)
W = numpy.array(W)
X = numpy.ndarray.flatten(X)
Y = numpy.ndarray.flatten(Y)
Z = numpy.ndarray.flatten(Z)
W = numpy.ndarray.flatten(W)

#creating axis
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')

#add axis labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

#add axis limits
ax.set_xlim([-1, 2])
ax.set_ylim([-1, 2])
ax.set_zlim([0, 2])

#plots lines connecting the X, Y, Z specified points
#uses "o" for circles at joints
ax.plot3D(X, Y, Z, 'blue', marker="o")

#showing plot
matplotlib.pyplot.show()

#defining forward kinematics

def T(x, y, z):
    T_xyz = Matrix([1,    0,    0,    x]
                   [0,    1,    0,    y]
                   [0,    0,    1,    z]
                   [0,    0,    0,    1])
    return T_xyz

def R(roll, pitch, yaw):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(roll), -sin(roll)],
                  [0, sin(roll), cos(roll)]])
    R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                  [          0, 1,          0],
                  [-sin(pitch), 0, cos(pitch)]])
    R_z = Matrix([[cos(yaw),-sin(yaw), 0],
                  [sin(yaw), cos(yaw), 0],
                  [       0,        0, 1]])
    return R_z*R_y*R_x

#defining translations

T1 = Ry(-pi/2) * T(0.187, 0, 0) * Rx(theta1)
