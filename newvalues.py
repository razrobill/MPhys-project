import numpy
import matplotlib.pyplot
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan


p0 = Matrix([0, 0, 0, 1])
p1 = Matrix([500, 0, 1.045, 1])
p2 = Matrix([0, 0, 1.045+1.3, 1])
p3 = Matrix([1.025*cos(pi/4), 0, 1.045+1.3+1.025*sin(pi/4), 1])
p4 = Matrix([1.025*cos(pi/4)+0.29, 0, 1.045+1.3+1.025*sin(pi/4), 1])
p5 = Matrix([1.025*cos(pi/4)+0.29+0.3, 0, 1.045+1.3+1.025*sin(pi/4), 1])
p6 = Matrix([1.025*cos(pi/4)+0.29+0.3, 0, 1.045+1.3+1.025*sin(pi/4), 1])

soa = numpy.array([p0,p1,p2,p3,p4,p5,p6])
X, Y, Z, W = zip(*soa)
X = numpy.array(X)
Y = numpy.array(Y)
Z = numpy.array(Z)
W = numpy.array((W))
X = numpy.ndarray.flatten(X)
Y = numpy.ndarray.flatten(Y)
Z = numpy.ndarray.flatten(Z)
W = numpy.ndarray.flatten(W)
fig = matplotlib.pyplot.figure()

ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim([-10,10])
ax.set_ylim([-10,10])
ax.set_zlim([0,10])

ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.show()
ax.plot3D(X,Y,Z, 'blue', marker="o")
matplotlib.pyplot.show()