import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing
import math
import sys

#d-h parameters matrix

def dh_matrix(theta, alpha, a, d):
    theta_ = math.radians(theta)
    alpha_ = math.radians(alpha)

    dhmatrix = np.matrix([[cos(theta_), -1 * sin(theta_) * cos(alpha_), sin(theta_) * sin(alpha_),
                       a * cos(theta_)],
                      [sin(theta_), cos(theta_) * cos(alpha_), -1 * cos(theta_) * sin(alpha_),
                       a * sin(theta_)],
                      [0, sin(alpha_), cos(alpha_), d],
                      [0, 0, 0, 1]])
    return dhmatrix

class kr360:

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

    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):

        a1 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a2 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        a3 = dh_matrix(theta3, self.alpha3, self.a3, self.d3)
        a4 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        a5 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        a6 = dh_matrix(theta6, self.alpha6, self.a6, self.d6)
        a_final = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(a1, a2), a3), a4), a5), a6)
        return a_final

if __name__ == '__main__':

    pass

    kr360 = kr360()

    endeffector = kr360.forward_kinematics(0, 90, 0, 0, 0, 0)
    print('end effector position: ')
    print(endeffector)



