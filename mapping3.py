import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

#kinematics simulator

#p1 to p6 are points in 3D space defined as symbolic matrices that can be transformed mathematically
#note that they are 4 dimensional so they can be used with homogenous transforms with the last 'W' coordinate always 1
#(to facilitate translation as well as rotation)
p1 = Matrix([0, 0, 0, 1])
p2 = Matrix([0.5, 0, 1.045, 1])
p3 = Matrix([0.5, 0, 1.045+1.3, 1])
p4 = Matrix([1.025+0.5*cos(pi/4), 0, 1.045+1.3*sin(pi/4), 1])
p5 = Matrix([0.29+1.025+0.5*cos(pi/4), 0, 1.045+1.3*sin(pi/4), 1])
p6 = Matrix([0.3+0.29+1.025+0.5*cos(pi/4), 0, 1.045+1.3*sin(pi/4), 1])


#concatenate these points into a column
soa = numpy.array([p1, p2, p3, p4, p5, p6])


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


#d-h parameters
a1 = 500
a2 = 1300
a3 = 55
a4 = 0
a5 = 0
a6 = 0

d1 = 1045
d2 = 0
d3 = 0
d4 = 1525
d5 = 290
d6 = 300

theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
theta6 = 0

alpha1 = -90*(pi/180)
alpha2 = 0*(pi/180)
alpha3 = 90*(pi/180)
alpha4 = -90*(pi/180)
alpha5 = 90*(pi/180)
alpha6 = 0*(pi/180)

d_h_table = np.array([[np.deg2rad(theta1), np.deg2rad(-90), 0, a1],
                      [np.deg2rad(theta2), 0, a2, 0]
                      [np.deg2rad(theta3), 0, a3, 0],
                      [np.deg2rad(theta4 + 90), np.deg2rad(90)]
                      ])