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



def DH_table(theta, alpha, r, d):
    theta_ = math.radians(theta)
    alpha_ = math.radians(alpha)

    a = np.matrix([[math.cos(theta_), -1 * math.sin(theta_) * math.cos(alpha_), math.sin(theta_) * math.sin(alpha_),
                    r * math.cos(theta_)],
                   [math.sin(theta_), math.cos(theta_) * math.cos(alpha_), -1 * math.cos(theta_) * math.sin(alpha_),
                    r * math.sin(theta_)],
                   [o, math.sin(alpha_), math.cos(alpha_), d],
                   [0, 0, 0, 1]])
    return a

endEffectorvalue = np.array([[0, 0, 1, 1510],
                        [-1, 0, 0, 0],
                        [0, -1, 0, 2290],
                        [0, 0, 0, 1]])

def magnitude_vector(x, y, z):

    return pow(x * x + y * y + z * z, 0.5)

def getHypotenuse(a, b):

    return pow(a * a + b * b, 0.5)

def get_cosine_angle(a, b, c):

    cos1 = (a * a + b * b - c * c) / (2 * a * b)

    return math.degrees(math.acos(cos1))


class kuka:

    def __init__(self):

        self.a1 = 500
        self.d1 = 1045
        self.alpha1 = 90

        self.a2 = 1300
        self.d2 = 0
        self.alpha2 = 0

        self.a3 = -55
        self.d3 = 0
        self.alpha3 = 90

        self.a4 = 0
        self.d4 = 720
        self.alpha4 = -90

        self.a5 = 0
        self.d5 = 0
        self.alpha5 = 90

        self.a6 = 0
        self.d6 = 290
        self.alpha6 = 0



    def getWristPosition(endEffector):
        dg = abs(self.d6)
        nx, ny, nz = endEffector[0, 2], endEffector[1, 2], endEffector[2, 3]
        xWrist = endEffector[0, 3] - nx * dg
        yWrist = endEffector[1, 3] - ny * dg
        zWrist = endEffector[2, 3] - nz * dg

        return [xWrist, yWrist, zWrist]

    getWristPosition(endEffectorvalue)
