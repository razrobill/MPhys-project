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

    dhmatrix = np.array([[cos(theta_), -1 * sin(theta_) * cos(alpha_), sin(theta_) * sin(alpha_),
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
        a_final = a1 @ a2 @ a3 @ a4 @ a5 @ a6
        return a_final


    def find_joint_positions(self, theta1, theta2, theta3, theta4, theta5, theta6):
        a1 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a0 = np.array([0, 0, 0, 1])
        mat_0_1 = a1
        p0 = [0, 0, 0]
        last_column1 = mat_0_1[:, 3]
        p1 = np.delete(last_column1, 3, 0)
        p1x = np.unique(p1[0])
        p1y = np.unique(p1[1])
        p1z = np.unique(p1[2])

        a2 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        mat_1_2 = a1 @ a2
        last_column2 = mat_1_2[:, 3]
        p2 = np.delete(last_column2, 3, 0)
        p2x = np.unique(p2[0])
        p2y = np.unique(p2[1])
        p2z = np.unique(p2[2])

        a3 = dh_matrix(theta3, self.alpha3, self.a3, self.d3)
        mat_2_3 = a2 @ a3
        last_column3 = mat_2_3[:, 3]
        p3 = np.delete(last_column3, 3, 0)
        p3x = np.unique(p3[0])
        p3y = np.unique(p3[1])
        p3z = np.unique(p3[2])

        a4 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        mat_3_4 = a3 @ a4
        last_column4 = mat_3_4[:, 3]
        p4 = np.delete(last_column4, 3, 0)
        p4x = np.unique(p4[0])
        p4y = np.unique(p4[1])
        p4z = np.unique(p4[2])

        a5 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        mat_4_5 = a4 @ a5
        last_column5 = mat_4_5[:, 3]
        p5 = np.delete(last_column5, 3, 0)
        p5x = np.unique(p5[0])
        p5y = np.unique(p5[1])
        p5z = np.unique(p5[2])

        a6 = dh_matrix(theta6, self.alpha6, self.a6, self.d6)
        mat_5_6 = a5 @ a6
        last_column6 = mat_5_6[:, 3]
        p6 = np.delete(last_column6, 3, 0)
        p6x = np.unique(p6[0])
        p6y = np.unique(p6[1])
        p6z = np.unique(p6[2])

        #jointcoordinates = repr([p0, p1, p2, p3, p4, p5, p6])

        return p6

    #if returning multiple points doesn't work, define a list with the points and return that?
    #(have them separated by a new line or something)








if __name__ == '__main__':

    pass

    kr360 = kr360()

    endeffector = kr360.forward_kinematics(0, 90, 0, 0, 0, 0)
    #print('end effector position: ')
    #print(endeffector)
    firstjoint = kr360.find_joint_positions(0, 90, 0, 0, 0, 0)
    print(repr(firstjoint))



