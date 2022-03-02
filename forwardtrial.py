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

def cosine_angle(a, b, c):
    cos1 = (a * a + b * b - c * c) / (2 * a * b)
    return math.degrees(math.acos(cos1))

class kr360:

    def __init__(self):

        self.a0 = 0
        self.d0 = 0
        self.alpha0 = 0

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

        a0 = dh_matrix(0, self.alpha0, self.a0, self.d0)
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

        if ywrist >= 0 and xwrist < 0:
            return [180 - theta1, (180 - theta1) + 180, (180 - theta1) - 180]
        elif ywrist >= 0 and xwrist > 0:
            return [-theta1, -theta1 - 180, 180 - theta1]
        elif ywrist <= 0 and xwrist < 0:
            return [-(180 + theta1), -theta1, -180 - (180 + theta1)]
        elif ywrist <= 0 and xwrist > 0:
            return [-theta1, -(theta1 - 180), -(180 + theta1)]
        elif xwrist == 0:
            return [90, -90]

        #if i'm having problems, change this function around it might be a problem with how im defining a0
    def link2_position(self, theta1):
        a0 = dh_matrix(0, self.alpha0, self.a0, self.d0)
        a1 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        return np.matmul(a0, a1)

    def link4_position(self, theta1, theta2, theta3):

        a0 = dh_matrix(0, 0, 0, 1)
        a1 = dh_matrix(theta1, 90, 500, 1045)
        a2 = dh_matrix(theta2, 0, 1300, 0)
        a3 = dh_matrix(theta3, 90, -55, 0)

        return np.matmul(np.matmul(np.matmul(a0, a1), a2), a3)

    def wrist_configurations(self, theta4, theta5, theta6):
        output_configurations = []
        output_configurations.append([theta4, theta5, theta6])

        if theta4 > 0:
            if theta6 > 0:
                output_configurations.append([theta4 - 360, theta5, theta6])
                output_configurations.append([theta4 - 360, theta5, theta6 - 360])
                output_configurations.append([theta4, theta5, theta6 - 360])
            else:
                output_configurations.append([theta4 - 360, theta5, theta6])
                output_configurations.append([theta4 - 360, theta5, theta6 + 360])
                output_configurations.append([theta4, theta5, theta6 + 360])
        else:
            if theta6 > 0:
                output_configurations.append([theta4 + 360, theta5, theta6])
                output_configurations.append([theta4 + 360, theta5, theta6 + 360])
                output_configurations.append([theta4, theta5, theta6 + 360])
            else:
                output_configurations.append([theta4 + 360, theta5, theta6])
                output_configurations.append([theta4 + 360, theta5, theta6 + 360])
                output_configurations.append([theta4, theta5, theta6 + 360])
        return output_configurations

    def end_orientation(self, configurations, endeffector):

        output_configurations = []
        endeffector_rotationmatrix = endeffector[0:3, 0:3]

        for configuration in configurations:

            theta1, theta2, theta3 = configuration[0], configuration[1], configuration[2]

            transform_matrix = self.link4_position(theta1, theta2, theta3)

            rotationmatrix = np.linalg.inv(transform_matrix[0:3, 0:3])

            arm_endeffector = np.matmul(rotationmatrix, endeffector_rotationmatrix)

            R02 = arm_endeffector[0, 2]
            R12 = arm_endeffector[1, 2]
            R20 = arm_endeffector[2, 0]
            R21 = arm_endeffector[2, 1]
            R22 = arm_endeffector[2, 2]

            theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))

            final_wristorientation = []

            if theta5 < 0.0001:
                final_wristorientation = [[0, 0, 0], [0, 0, 180], [180, 0, 0], [180, 0, 180]]
            else:
                theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(-R12, -R02))
                theta6 = math.degrees(math.atan2(R21, R20))
                print("--------")
                print(self.wrist_configurations(theta4, theta5, theta6))
                final_wristorientation += self.wrist_configurations(theta4, theta5, theta6)
                #there are 2 configurations for link 5
                theta5 = math.degrees(math.atan2(-pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(R12, R02))
                theta6 = math.degrees(math.atan2(-R21, -R20))
                final_wristorientation += self.wrist_configurations(theta4, theta5, theta6)
                print(final_wristorientation)

            for wrist_configuration in final_wristorientation:
                newconfiguration = configuration + wrist_configuration
                output_configurations.append(newconfiguration)

        return output_configurations

    def theta2_configurations(self, theta1_configurations, xwrist, ywrist, zwrist):

        output_configurations = []
        wrist_coordinates = np.expand_dims(np.array([xwrist, ywrist, zwrist, 1]), axis=1)
        ld = pow(self.d4 * self.d4 + self.a3 * self.a3, 0.5)
        l2 = abs(self.a2)

        for _, theta1 in enumerate(theta1_configurations):

            link2_transformmatrix = self.link2_position(theta1)
            print(link2_transformmatrix.dtype)
            link2_transformmatrix2 = np.linalg.inv(link2_transformmatrix)

            #coordinates of wrist point wrt link 2
            wrist_coordinateslink2 = np.matmul(link2_transformmatrix2, wrist_coordinates)
            #distance of wrist point from link 2
            hypotenuse_length = pow(wrist_coordinateslink2[0].item() * wrist_coordinateslink2[0].item() + wrist_coordinateslink2[1].item() * wrist_coordinateslink2[1].item(), 0.5)
            beta1 = math.degrees(math.atan2(wrist_coordinateslink2[1], wrist_coordinateslink2[0]))

            #not every configuration is possible for theta2
            try:
                gamma1 = cosine_angle(l2, hypotenuse_length, ld)
                elbowup = beta1 + gamma1
                elbowdown = beta1 - gamma1

                if -elbowdown - 180 > 0:
                    elbowdown += 360

                if elbowup - 180 > 0:
                    elbowup -= 360

                output_configurations.append([theta1, elbowup, 1])
                output_configurations.append([theta1, elbowdown, 0])

            except:
                pass

        return output_configurations


    def theta3_configurations(self, theta2_configurations, xwrist, ywrist, zwrist):

        output_configurations = []

        wrist_coordinates = np.expand_dims(np.array([xwrist, ywrist, zwrist, 1]), axis=1)

        #shoulder offset
        phi = math.degrees(math.atan2(self.a3, abs(self.d4)))

        l2 = abs(self.a2)
        ld = pow(self.d4 * self.d4 + self.a3 * self.a3, 0.5)

        for _, thetaconfiguration in enumerate(theta2_configurations):
            theta1 = thetaconfiguration[0]
            theta2 = thetaconfiguration[1]
            elbow = thetaconfiguration[2]

            link2_transformationmatrix = self.link2_position(theta1)
            link2_transformationmatrix = np.linalg.inv(link2_transformationmatrix)

            wrist_coordinateslink2 = np.matmul(link2_transformationmatrix, wrist_coordinates)
            hypotenuse_length = pow(wrist_coordinateslink2[0].item() * wrist_coordinateslink2[0].item() + wrist_coordinateslink2[1].item() * wrist_coordinateslink2[1].item(), 0.5)

            try:
                eta = cosine_angle(l2, ld, hypotenuse_length)
                if elbow == 1:
                    output_configurations.append([theta1, theta2, -180 + phi + eta])
                else:
                    theta3 = 18- + phi - eta
                    if theta3 > 180:
                        theta3 = phi - eta - 180
                    output_configurations.append(theta1, theta2, theta3)
            except:
                pass
            return output_configurations

    def inverse_kinematics(self, endeffector_transformmatrix):

        xwrist, ywrist, zwrist = self.wrist_position(endeffector_transformmatrix)
        theta1_configurations = self.theta1_configurations(xwrist, ywrist, zwrist)
        theta2_configurations = self.theta2_configurations(theta1_configurations, xwrist, ywrist, zwrist)
        theta3_configurations = self.theta3_configurations(theta2_configurations, xwrist, ywrist, zwrist)
        allconfigurations = self.end_orientation(theta3_configurations, endeffector_transformmatrix)

        return allconfigurations




#testing values
if __name__ == '__main__':

    pass

    kr360 = kr360()

    endeffector = kr360.forward_kinematics( 0, 90, 0, 0, 0, 0)
    print('end effector position: ')
    print(endeffector)
    print(type(endeffector))
    wristposition = kr360.wrist_position(endeffector)
    print('wrist position: ')
    print(wristposition)






    #printing out end orientation is not working
    #orient = kr360.end_orientation(configurations,endeffector)
    #print(orient)

































#showing plot
matplotlib.pyplot.show()