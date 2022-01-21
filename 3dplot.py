import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing

a1 = 0.5
a2 = 1.300
a3 = 0.055
a4 = 0
a5 = 0
a6 = 0

d1 = 1.045
d2 = 0
d3 = 0
d4 = 1.525
d5 = 0.290
d6 = 0.300

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



p1 = Matrix([0, 0, 0, 1])
p2 = Matrix([a1, 0, d1, 1])
p3 = Matrix([a1+a2, 0, d1+d2, 1])
p4 = Matrix([a1+a2+a3*cos(pi/4), 0, d1+d2+d3*sin(pi/4), 1])
p5 = Matrix([a1+a2+a3+a4*cos(pi/4), 0, d1+d2+d3+d4*sin(pi/4), 1])
p6 = Matrix([a1+a2+a3+a4+a5*cos(pi/4), 0, d1+d2+d3+d4+d5*sin(pi/4), 1])
p7 = Matrix([a1+a2+a3+a4+a5+a6*cos(pi/4), 0, d1+d2+d3+d4+d5+d6*sin(pi/4), 1])

soa = numpy.array([p1, p2, p3, p4, p5, p6, p7])
print(soa)

X, Y, Z, W = zip(*soa)

X = numpy.array(X)
Y = numpy.array(Y)
Z = numpy.array(Z)
W = numpy.array(W)
X = numpy.ndarray.flatten(X)
Y = numpy.ndarray.flatten(Y)
Z = numpy.ndarray.flatten(Z)
W = numpy.ndarray.flatten(W)

fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

ax.plot3D(X, Y, Z, 'blue', marker="o")

matplotlib.pyplot.show()
