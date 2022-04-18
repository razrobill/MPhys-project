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

    dhmatrix = np.array([[(sp.cos(theta_)), (-1 * sp.sin(theta_)) * (sp.cos(alpha_)), (sp.sin(theta_)) * (sp.sin(alpha_)),
                       a * (sp.cos(theta_))],
                      [(sp.sin(theta_)), (sp.cos(theta_)) * (sp.cos(alpha_)), (-1 * sp.cos(theta_)) * (sp.sin(alpha_)),
                       a * (sp.sin(theta_))],
                      [0, (sp.sin(alpha_)), (sp.cos(alpha_)), d],
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
        self.d4 = 1025
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

    def valid_theta_configurations(self, joint, theta, constraint_type="angle"):

        if constraint_type not in ['angle']:
            raise Exception("enter valid constraint for joint {}".format(joint))
        if joint not in [1, 2, 3, 4, 5, 6]:
            raise Exception("please enter a valid joint number. Given {}".format(joint))

        if joint == 1:
            if constraint_type == "angle":
                if abs(theta) <= 185:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta1')
                    return False

        elif joint == 2:
            if constraint_type == "angle":
                if theta >= -130 and theta <= 20:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta2')
                    return False

        elif joint == 3:
            if constraint_type == "angle":
                if theta >= -100 and theta <= 144:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta3')
                    return False

        elif joint == 4:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta4')
                    return False

        elif joint == 5:
            if constraint_type == "angle":
                if abs(theta) <= 120:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta5')
                    return False

        elif joint == 6:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    #valid = print('valid')
                    return True
                else:
                    invalid = print('invalid theta6')
                    return False

    def check_configuration(self, all_theta_configurations):

        output_configurations = []

        #for configuration in all_theta_configurations:
        t1 = self.valid_theta_configurations(1, all_theta_configurations[0], constraint_type="angle")
        t2 = self.valid_theta_configurations(2, all_theta_configurations[1], constraint_type="angle")
        t3 = self.valid_theta_configurations(3, all_theta_configurations[2], constraint_type="angle")
        t4 = self.valid_theta_configurations(4, all_theta_configurations[3], constraint_type="angle")
        t5 = self.valid_theta_configurations(5, all_theta_configurations[4], constraint_type="angle")
        t6 = self.valid_theta_configurations(6, all_theta_configurations[5], constraint_type="angle")

        if t1 and t2 and t3 and t4 and t5 and t6:
            print('valid configuration')
        else:
            print('invalid configuration')


        return







    #it is this function that contains the plotting for now
    def find_joint_positions(self, theta1, theta2, theta3, theta4, theta5, theta6):
        a1 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a0 = np.array([0, 0, 0, 1])
        mat_0_1 = a1
        p0 = [0, 0, 0]
        p0x = int(p0[0])
        p0y = int(p0[1])
        p0z = int(p0[2])
        last_column1 = mat_0_1[:, 3]
        p1 = np.delete(last_column1, 3, 0)
        p1x = int(p1[0])
        p1y = int(p1[1])
        p1z = int(p1[2])

        a2 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        mat_1_2 = mat_0_1 @ a2
        last_column2 = mat_1_2[:, 3]
        p2 = np.delete(last_column2, 3, 0)
        p2x = int(p2[0])
        p2y = int(p2[1])
        p2z = int(p2[2])

        a3 = dh_matrix(theta3, self.alpha3, self.a3, self.d3)
        mat_2_3 = mat_1_2 @ a3
        last_column3 = mat_2_3[:, 3]
        p3 = np.delete(last_column3, 3, 0)
        p3x = int(p3[0])
        p3y = int(p3[1])
        p3z = int(p3[2])

        a4 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        mat_3_4 = mat_2_3 @ a4
        last_column4 = mat_3_4[:, 3]
        p4 = np.delete(last_column4, 3, 0)
        p4x = int(p4[0])
        p4y = int(p4[1])
        p4z = int(p4[2])

        a5 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        mat_4_5 = mat_3_4 @ a5
        last_column5 = mat_4_5[:, 3]
        p5 = np.delete(last_column5, 3, 0)
        p5x = int(p5[0])
        p5y = int(p5[1])
        p5z = int(p5[2])

        a6 = dh_matrix(theta6, self.alpha6, self.a6, self.d6)
        mat_5_6 = mat_4_5 @ a6
        last_column6 = mat_5_6[:, 3]
        p6 = np.delete(last_column6, 3, 0)
        print(p6)
        p6x = int(p6[0])
        #print(p6x)
        p6y = int(p6[1])
        #print(p6y)
        p6z = int(p6[2])
        #print(p6z)

        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.set_xlim([-2000, 2000])
        ax.set_ylim([-1500, 2000])
        ax.set_zlim([-1500, 2500])

        ax.plot3D(p0x, p0y, p0z, 'blue', marker="o")
        ax.plot3D(p1x, p1y, p1z, 'blue', marker="o")
        ax.plot3D(p2x, p2y, p2z, 'blue', marker="o")
        ax.plot3D(p3x, p3y, p3z, 'blue', marker="o")
        ax.plot3D(p4x, p4y, p4z, 'blue', marker="o")
        ax.plot3D(p5x, p5y, p5z, 'blue', marker="o")
        ax.plot3D(p6x, p6y, p6z, 'red', marker="^")



        jointcoordinates = [p0, p1, p2, p3, p4, p5, p6]
        jointp0 = jointcoordinates[0]
        jointp1 = jointcoordinates[1]
        jointcoordinates = np.array([p0, p1, p2, p3, p4, p5, p6])
        jointcoordinates = jointcoordinates.tolist()
        return jointcoordinates







if __name__ == '__main__':

    pass

    kr360 = kr360()

    endeffector = kr360.forward_kinematics(0, 90, 1, 0, 0, 0)
    print('end effector position: ')
    print(endeffector)
    last_column = endeffector[:, 3]
    end_effector_position = np.delete(last_column, 3, 0)
    print(end_effector_position)
    jointcoordinates = kr360.find_joint_positions(0, 90, 1, 0, 0, 0)
    #validity2 = kr360.valid_theta_configurations(2, 200, 'angle')
    all_theta_configurations = [0, 90, 1, 0, 0, 0]
    check = kr360.check_configuration(all_theta_configurations)
    #print(check)

    open_file = open('forward-coordinates(maximum reach).txt', 'w')
    sys.stdout = open_file

    ux = int(end_effector_position[0])
    uy = int(end_effector_position[1])
    uz = int(end_effector_position[2])
    #ax.plot3D(ux, uy, uz, 'red', marker="^")
    matplotlib.pyplot.show()



    for (i, item) in enumerate(jointcoordinates):
        jointcoordinates = (str(item))
        print(jointcoordinates)

    open_file.close()















