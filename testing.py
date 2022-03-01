import matplotlib.pyplot as plt
import numpy
import matplotlib.pyplot
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sympy as sp
from sympy import Matrix, Symbol, symbols, solveset
from sympy import S, erf, log, sqrt, pi, sin, cos, tan
from sympy import init_printing
from scipy.spatial.transform import Rotation
import math
import sys



#kinematics simulator

#d-h parameters in mm


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

    def forward_kinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):

        a1 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a2 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        a3 = dh_matrix(theta3, self.alpha3, self.a3, self.d3)
        a4 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        a5 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        a6 = dh_matrix(theta6, self.alpha6, self.a6, self.d6)
        a_final = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(a1, a2), a3), a4), a5), a6)
        return a_final









    #defines the wrist position for a given end effector position
    def wrist_position(self, endeffector):
        end5_6 = abs(self.d6)
        nx, ny, nz = endeffector[0, 2], endeffector[1, 2], endeffector[2, 2]
        xwrist = endeffector[0, 3] - nx * end5_6
        ywrist = endeffector[1, 3] = ny * end5_6
        zwrist = endeffector[2, 3] = nz * end5_6

        return [xwrist, ywrist, zwrist]

    # still need to add valid speeds to the configurations
    def valid_theta_configurations(self, joint, theta, constraint_type="angle"):

        if joint == 1:
            if constraint_type == 'angle':
                if abs(theta) <= 185:
                    return True
                else:
                    return False

        elif joint == 2:
            if constraint_type == 'angle':
                if theta >= -130 and theta <= 20:
                    return True
                else:
                    return False

        elif joint == 3:
            if constraint_type == 'angle':
                if theta >= -100 and theta <= 144:
                    return True
                else:
                    return False

        elif joint == 4:
            if constraint_type == 'angle':
                if abs(theta) <= 350:
                    return True
                else:
                    return False

        elif joint == 5:
            if constraint_type == 'angle':
                if abs(theta) <= 120:
                    return True
                else:
                    return False

        elif joint == 6:
            if constraint_type == 'angle':
                if abs(theta) <= 350:
                    return True
                else:
                    return False


    def theta1_configurations(self, xwrist, ywrist, zwrist):
        theta1 = math.degrees(math.atan2(ywrist, xwrist))



    ##def endorientation(self, endeffector):
        #outputconfigurations = []
        #endeffector_rotationmatrix = endeffector[0:3, 0:3]

        #for configuration in configurations:
            #theta1, theta2, theta3 = configuration[0], configuration[1], configuration[2]
        #transform = self.forward_kinematics(theta1, theta2, heta3, theta4, theta5, theta6)
        #rotation_matrix = np.linalg.inv(transform[0:3, 0:3])
        #arm_endeffector = np.matmul(rotation_matrix, endeffector_rotationmatrix)

        #R02 = arm_endeffector[0, 2]
        #R12 = arm_endeffector[1, 2]
        #R20 = arm_endeffector[2, 0]
        #R21 = arm_endeffector[2, 1]
        #R22 = arm_endeffector[2, 2]

        #theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))

        #finalwristorientation = []

        #if theta5 < 0.0001:
            #finalwristorientation = [[0, 0, 0], [0, 0, 180], [180, 0, 0], [180, 0, 180]]
        #return finalwristorientation

    #def inverse_kinematics(self, endtransformationmatrix):














#testing values
if __name__ == '__main__':

    pass

    kuka = kuka()

    endeffector = kuka.forward_kinematics(0, 90, 0, 0, 0, 0)
    print('end effector position: ')
    print(endeffector)
    wristposition = kuka.wrist_position(endeffector)
    print('wrist position: ')
    print(wristposition)
    #orient = kuka.endorientation(endeffector)
    #print(orient)

































#showing plot
matplotlib.pyplot.show()