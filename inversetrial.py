import sys

import numpy as np
import math


def dh_matrix(theta, alpha, r, d):
    theta_ = math.radians(theta)
    alpha_ = math.radians(alpha)

    dhmatrix = np.matrix([[math.cos(theta_), -1 * math.sin(theta_) * math.cos(alpha_), math.sin(theta_) * math.sin(alpha_),
                    r * math.cos(theta_)],
                   [math.sin(theta_), math.cos(theta_) * math.cos(alpha_), -1 * math.cos(theta_) * math.sin(alpha_),
                    r * math.sin(theta_)],
                   [0, math.sin(alpha_), math.cos(alpha_), d],
                   [0, 0, 0, 1]])

    return dhmatrix




def getHypotenuse(a, b):

    return pow(a * a + b * b, 0.5)


def getCosineAngle(a, b, c):

    cos1 = (a * a + b * b - c * c) / (2 * a * b)

    return math.degrees(math.acos(cos1))


class kuka:


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

    def forwardKinematics(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """
        All the angles should be given in degrees.
        This method calculates final position and orientation of the end flange using
        the theta1, theta2, theta3, theta4, theta5, theta6.
        :param theta1: rotation around the 1st angle in degrees.
        :param theta2: rotation around the 2nd angle in degrees.
        :param theta3: rotation around the 3rd angle in degrees.
        :param theta4: rotation around the 4th angle in degrees.
        :param theta5: rotation around the 5th angle in degrees.
        :param theta6: rotation around the 6th angle in degrees.
        :return: np array of 4X4 transformation matrix.
        """
        a1 = dh_matrix(0, self.alpha0, self.a0, self.d0)
        a2 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)
        a3 = dh_matrix(theta2, self.alpha2, self.a2, self.d2)
        a4 = dh_matrix(theta3 - 90, self.alpha3, self.a3, self.d3)
        a5 = dh_matrix(theta4, self.alpha4, self.a4, self.d4)
        a6 = dh_matrix(theta5, self.alpha5, self.a5, self.d5)
        a7 = dh_matrix(theta6 - 180, self.alpha6, self.a6, self.d6)
        a_final = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(a1, a2), a3), a4), a5), a6), a7)

        return a_final

    def getWristPosition(self, endEffector):
        """
        This function returns the position of the wrist transformation matrix for a given EndEffector.
        :param endEffector: 4X4 transformation matrix of the end effector.
        :return: coordinates of the end effector.
        """
        dg = abs(self.d6)
        nx, ny, nz = endEffector[0, 2], endEffector[1, 2], endEffector[2, 2]

        xWrist = endEffector[0, 3] - nx * dg
        yWrist = endEffector[1, 3] - ny * dg
        zWrist = endEffector[2, 3] - nz * dg

        return [xWrist, yWrist, zWrist]

    def thetaValidConfiguration(self, joint, theta, constraint_type = "angle"):
        """
        This method is used to check if a theta configuration is valid or not. These constraints are given in the
        datasheet and are used in lineMovement method.
        :param joint: joint for which the constraints have to be checked.
        :param theta: angle of joint in degrees.
        :param constraint_type: "angle" for range of motion possible for the joint1 and "speed" if the given speed is
        possible for the joint to obtain.
        :return: boolean. true if the configuration is valid otherwise false.
        """

        if constraint_type not in ['angle', 'speed']:
            raise Exception("please enter valid constraint type, (angle or speed) for joint {}".format(joint))

        if joint not in [1, 2, 3, 4, 5, 6]:
            raise Exception("please enter a valid link number. Given: {}".format(joint))

        if joint == 1:
            if constraint_type == "angle":
                if abs(theta) <= 185:
                    return True
                else:
                    return False
        elif joint == 2:
            if constraint_type == "angle":
                if theta >= -130 and theta <= 20:
                    return True
                else:
                    return False
        elif joint == 3:
            if constraint_type == "angle":
                if theta >= -100 and theta <= 144:
                    return True
                else:
                    return False
        elif joint == 4:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    return True
                else:
                    return False
        elif joint == 5:
            if constraint_type == "angle":
                if abs(theta) <= 120:
                    return True
                else:
                    return False
        elif joint == 6:
            if constraint_type == "angle":
                if abs(theta) <= 350:
                    return True
                else:
                    return False

    def __theta1Configurations(self, xWrist, yWrist, zWrist):
        """
        This method returns 3 configurations for the first joint.
        :param flange: final 4 X 4 matrix of the end effector
        :return: [theta1.1, theta1.2, theta1.3] angles in degrees.
        """

        theta1 = math.degrees(math.atan2(yWrist, xWrist))

        if yWrist >= 0 and xWrist < 0:
            return [180 - theta1, (180 - theta1) + 180, (180 - theta1) - 180]
        elif yWrist >= 0 and xWrist > 0:
            return [-theta1, -theta1 - 180, 180 - theta1]
        elif yWrist <= 0 and xWrist < 0:
            return [-(180 + theta1), -theta1, -180 - (180 + theta1)]
        elif yWrist <= 0 and xWrist > 0:
            return [-theta1, -(theta1 - 180), -(180 + theta1)]
        elif xWrist == 0:
            return [90, -90]

    def __getLink2Position(self, theta1):
        """
        This method is used to find the position and orientation for the 2nd link using forward kinematics.
        This is important in order to calculate 3rd and 2nd configurations.
        :param theta1: theta1 angle
        :return: 4X4 transformation matrix in form of numpy array.
        """

        a1 = dh_matrix(0, self.alpha0, self.a0, self.d0)
        a2 = dh_matrix(theta1, self.alpha1, self.a1, self.d1)

        return np.matmul(a1, a2)

    def __getLink5Position(self, theta1, theta2, theta3):
        """
        This method is used to find the end position and orientation of the link5 using forward kinematics
        :param theta1: angle of link 1.
        :param theta2: angle of link 2.
        :param theta3: angle of link 3.
        :return: 4X4 transformation matrix in form of numpy array.
        """

        a1 = dh_matrix(0, -180, 0, 645)
        a2 = dh_matrix(theta1, 90, 330, 0)
        a3 = dh_matrix(theta2, 0, 1150, 0)
        a4 = dh_matrix(theta3 - 90, 90, 115, 0)

        return np.matmul(np.matmul(np.matmul(a1, a2), a3), a4)

    def __getwristConfigurations(self, theta4, theta5, theta6):
        """
        This method is used to find all the configurations possible.
        :param theta4: angle of link 4.
        :param theta5: angle of link 5.
        :param theta6: angle of link 6.
        :return: [[theta4.1, theta5.1, theta6.1], [theta4.1, theta5.2, theta6.2], ...]
        """
        outputConfig = []

        outputConfig.append([theta4, theta5, theta6])

        if theta4 > 0:
            if theta6 > 0:
                outputConfig.append([theta4 - 360, theta5, theta6])
                outputConfig.append([theta4 - 360, theta5, theta6 - 360])
                outputConfig.append([theta4, theta5, theta6 - 360])
            else:
                outputConfig.append([theta4 - 360, theta5, theta6])
                outputConfig.append([theta4 - 360, theta5, theta6 + 360])
                outputConfig.append([theta4, theta5, theta6 + 360])
        else:
            if theta6 > 0:
                outputConfig.append([theta4 + 360, theta5, theta6])
                outputConfig.append([theta4 + 360, theta5, theta6 - 360])
                outputConfig.append([theta4, theta5, theta6 - 360])
            else:
                outputConfig.append([theta4 + 360, theta5, theta6])
                outputConfig.append([theta4 + 360, theta5, theta6 + 360])
                outputConfig.append([theta4, theta5, theta6 + 360])

        return outputConfig

    def __endOrientation(self, configurations, endEffector):
        """
        This method returns all 48 configurations (at max).
        :param configurations: configurations calculated previously
        [[theta1.1, theta2.1, theta3.1], [theta1.1, theta2.2, theta3.2], ...]
        :param endEffector: transformation matrix of the end effector (not the wrist position)
        :return: list of 48 configurations at max.
        [[theta1.1, theta2.1, theta3.1, theta4.1, theta5.1, theta6.1],
        [theta1.1, theta2.1, theta3.1, theta4.1, theta5.2, theta6.2],...]
        """
        outputConfigurations = []
        endEffectorRotationMatrix = endEffector[0:3, 0:3]

        for configuration in configurations:

            theta1, theta2, theta3 = configuration[0], configuration[1], configuration[2]

            transformationMatrix = self.__getLink5Position(theta1, theta2, theta3)

            rotationMatrix = np.linalg.inv(transformationMatrix[0:3, 0:3])

            armEndEff = np.matmul(rotationMatrix, endEffectorRotationMatrix)

            R02 = armEndEff[0, 2]
            R12 = armEndEff[1, 2]
            R20 = armEndEff[2, 0]
            R21 = armEndEff[2, 1]
            R22 = armEndEff[2, 2]

            theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))

            # final wrist orientation

            finalwristOrientation = []
            # in order to handle singularity, as the solutions can be infinite.
            if theta5 < 0.0001 or abs(abs(theta5) - 180) < 0.0001:
                finalwristOrientation = [[0, 0, 0], [0, 0, 180], [180, 0, 0], [180, 0, 180]]
            else:
                # since there are 2 configurations for the link 5.

                theta5 = math.degrees(math.atan2(pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(-R12, -R02))
                theta6 = math.degrees(math.atan2(R21, R20))
                # print("---")
                # print(self.__getwristConfigurations(theta4, theta5, theta6))
                finalwristOrientation += self.__getwristConfigurations(theta4, theta5, theta6)

                theta5 = math.degrees(math.atan2(-pow(R02 * R02 + R12 * R12, 0.5), -R22))
                theta4 = math.degrees(math.atan2(R12, R02))
                theta6 = math.degrees(math.atan2(-R21, -R20))

                finalwristOrientation += self.__getwristConfigurations(theta4, theta5, theta6)

            for wristConfiguration in finalwristOrientation:
                tempConfig = configuration + wristConfiguration
                outputConfigurations.append(tempConfig)

        return outputConfigurations

    def __theta2Configurations(self, theta1Configurations, xWrist, yWrist, zWrist):
        """
        This method returns the 6 (at max) configurations for elbow up and elbow down.
        :param theta1Configurations: list of theta1 configurations.
        :param xWrist: x coordinate of the wrist.
        :param yWrist: y coordinate of the wrist.
        :param zWrist: z coordinate of the wrist.
        :return: [[theta1.1, theta2.1, mark], [theta1.1, theta2.2, mark], ...]
        mark = 1 for elbow up configuration.
        mark = 0 for elbow down configuration.
        This mark is used to distinguish the 2 configurations.
        Refer to inverse kinematics for 2 links for additional info.
        """
        outputConfigurations = []

        wristCoordinates = np.expand_dims(np.array([xWrist, yWrist, zWrist, 1]), axis=1)

        ld = getHypotenuse(self.d4, self.a3)
        l2 = abs(self.a2)

        for _, theta1 in enumerate(theta1Configurations):

            link2TransformationMatrix = self.__getLink2Position(theta1)

            link2TranforsmationMatrix = np.linalg.inv(link2TransformationMatrix)

            # get coordinates of the wrist point w.r.t to link2.
            wristCoordinatesLink2 = np.matmul(link2TranforsmationMatrix, wristCoordinates)

            # distance of wrist point from link2.
            hypotenuseLength = getHypotenuse(wristCoordinatesLink2[0].item(), wristCoordinatesLink2[1].item())

            beta1 = math.degrees(math.atan2(wristCoordinatesLink2[1], wristCoordinatesLink2[0]))

            # since not every configuration is possible for theta2.
            try:
                gamma1 = getCosineAngle(l2, hypotenuseLength, ld)

                elbowUp = beta1 + gamma1
                elbowDown = beta1 - gamma1

                # this is just angle adjustment according to the project's desired configuration.
                # found this with experimentation.

                if -elbowDown - 180 > 0:
                    elbowDown += 360
                if elbowUp - 180 > 0:
                    elbowUp -= 360

                outputConfigurations.append([theta1, elbowUp, 1])
                outputConfigurations.append([theta1, elbowDown, 0])

            except:
                pass

        return outputConfigurations

    def __theta3Configurations(self, theta2Configurations, xWrist, yWrist, zWrist):
        """
        This method is used to find the third configurations.
        :param theta2Configurations: list of configurations found in method __theta2Configurations.
        :param xWrist: x coordinate of the wrist.
        :param yWrist: y coordinate of the wrist.
        :param zWrist: z coordinate of the wrist.
        :return: [[theta1.1, theta2.1, theta3.1], [theta1.1, theta2.2, theta3.2], ....]
        """

        outputConfigurations = []

        wristCoordinates = np.expand_dims(np.array([xWrist, yWrist, zWrist, 1]), axis=1)

        # shoulder offset.
        phi = math.degrees(math.atan2(self.a3, abs(self.d4)))

        l2 = abs(self.a2)
        ld = getHypotenuse(self.d4, self.a3)

        for _, thetaConfiguration in enumerate(theta2Configurations):
            theta1 = thetaConfiguration[0]
            theta2 = thetaConfiguration[1]
            elbow = thetaConfiguration[2]

            link2TransformationMatrix = self.__getLink2Position(theta1)
            link2TransformationMatrix = np.linalg.inv(link2TransformationMatrix)

            # get coordinates of the wrist point w.r.t to link2.
            wristCoordinatesLink2 = np.matmul(link2TransformationMatrix, wristCoordinates)
            hypotenuseLength = getHypotenuse(wristCoordinatesLink2[0].item(), wristCoordinatesLink2[1].item())

            try:
                eta = getCosineAngle(l2, ld, hypotenuseLength)

                if elbow == 1:

                    outputConfigurations.append([theta1, theta2, -180 + phi + eta])
                else:
                    theta3 = 180 + phi - eta

                    if theta3 > 180:
                        theta3 = phi - eta - 180

                    outputConfigurations.append([theta1, theta2, theta3])
            except:
                pass

        return outputConfigurations

    def inverseKinematicsAllConfig(self, endTransformationMatrix):
        """
        This method is used for the inverse kinematics.
        This calculates all the configurations for a given end effector position and orientation
        using geometry.
        Note that these might be all the valid possible configurations for the robot
        because of physical constraints.
        Additionally, this method returns configurations according to the direction of the robot
        defined in the datasheet.
        :param endTransformationMatrix: 4X4 end transformation matrix as a form of numpy array.
        :return: [[theta1, theta2,...theta6], [theta1, theta2,...theta6]...]
        """
        xWrist, yWrist, zWrist = self.getWristPosition(endTransformationMatrix)

        theta1Configurations = self.__theta1Configurations(xWrist, yWrist, zWrist)

        theta2Configurations = self.__theta2Configurations(theta1Configurations, xWrist, yWrist, zWrist)

        theta3Configurations = self.__theta3Configurations(theta2Configurations, xWrist, yWrist, zWrist)

        allConfigurations = self.__endOrientation(theta3Configurations, endTransformationMatrix)

        return allConfigurations

if __name__ == '__main__':

    pass

    kukaBot = kuka()

    endEffector = [[ -8.85562315e-01,   2.69213051e-02,  -4.63739830e-01,   9.16922244e+02],
                    [ -4.47193236e-01,  -3.19530365e-01,   8.35415199e-01,   1.56699319e+02],
                   [ -1.25688490e-01,   9.47193533e-01,   2.95003075e-01,   2.66453955e+03],
                   [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,  1.00000000e+00]]

    endEffector = np.array(endEffector)
    temp = np.array(kukaBot.inverseKinematicsAllConfig(endEffector))
    count = 0
    for i in temp:
        print(i)

