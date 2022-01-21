import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
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

#starting forward kinematics



#rot = Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha)],
              #[sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha)]
              #[0, sin(alpha), cos(alpha)]])

#print(rot)

#trans = Matrix([a*cos(theta), a*sin(theta), d])

#last_row = Matrix([[0, 0, 0, 1]])

#t = Matrix.vstack(Matrix.hstack(rot, trans), last_row)

#t01 = t.subs


