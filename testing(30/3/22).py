import numpy as np


def axis_angle_rot_matrix(k, q):
    """
    Creates a 3x3 rotation matrix in 3D space from an axis and an angle.

    Input
    :param k: A 3 element array containing the unit axis to rotate around (kx,ky,kz)
    :param q: The angle (in radians) to rotate by

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """

    # 15 pts
    c_theta = np.cos(q)
    s_theta = np.sin(q)
    v_theta = 1 - np.cos(q)
    kx = k[0]
    ky = k[1]
    kz = k[2]

    # First row of the rotation matrix
    r00 = kx * kx * v_theta + c_theta
    r01 = kx * ky * v_theta - kz * s_theta
    r02 = kx * kz * v_theta + ky * s_theta

    # Second row of the rotation matrix
    r10 = kx * ky * v_theta + kz * s_theta
    r11 = ky * ky * v_theta + c_theta
    r12 = ky * kz * v_theta - kx * s_theta

    # Third row of the rotation matrix
    r20 = kx * kz * v_theta - ky * s_theta
    r21 = ky * kz * v_theta + kx * s_theta
    r22 = kz * kz * v_theta + c_theta

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def hr_matrix(k, t, q):
    '''
    Create the Homogenous Representation matrix that transforms a point from Frame B to Frame A.
    Using the axis-angle representation
    Input
    :param k: A 3 element array containing the unit axis to rotate around (kx,ky,kz)
    :param t: The translation from the current frame (e.g. Frame A) to the next frame (e.g. Frame B)
    :param q: The rotation angle (i.e. joint angle)

    Output
    :return: A 4x4 Homogenous representation matrix
    '''
    # Calculate the rotation matrix (angle-axis representation)
    rot_matrix_A_B = axis_angle_rot_matrix(k, q)

    # Store the translation vector t
    translation_vec_A_B = t

    # Convert to a 2D matrix
    t0 = translation_vec_A_B[0]
    t1 = translation_vec_A_B[1]
    t2 = translation_vec_A_B[2]
    translation_vec_A_B = np.array([[t0],
                                    [t1],
                                    [t2]])

    # Create the homogeneous transformation matrix
    homgen_mat = np.concatenate((rot_matrix_A_B, translation_vec_A_B), axis=1)  # side by side

    # Row vector for bottom of homogeneous transformation matrix
    extra_row_homgen = np.array([[0, 0, 0, 1]])

    # Add extra row to homogeneous transformation matrix
    homgen_mat = np.concatenate((homgen_mat, extra_row_homgen), axis=0)  # one above the other

    return homgen_mat

class RoboticArm:
    def __init__(self, k_arm, t_arm):
        '''
        Creates a robotic arm class for computing position and velocity.

        Input
        :param k_arm: A 2D array that lists the different axes of rotation (rows) for each joint.
        :param t_arm: A 2D array that lists the translations from the previous joint to the current joint
	                  The first translation is from the global (base) frame to joint 1 (which is often equal to the global frame)
	                  The second translation is from joint 1 to joint 2, etc.
        '''
        self.k = np.array(k_arm)
        self.t = np.array(t_arm)
        assert k_arm.shape == t_arm.shape, 'Warning! Improper definition of rotation axes and translations'
        self.N_joints = k_arm.shape[0]

    def position(self, Q, index=-1, p_i=[0, 0, 0]):
        '''
        Compute the position in the global (base) frame of a point given in a joint frame
        (default values will assume the input position vector is in the frame of the last joint)
        Input
        :param p_i: A 3 element vector containing a position in the frame of the index joint
        :param index: The index of the joint frame being converted from (first joint is 0, the last joint is N_joints - 1)

        Output
        :return: A 3 element vector containing the new position with respect to the global (base) frame
        '''
        # The position of this joint described by the index
        p_i_x = p_i[0]
        p_i_y = p_i[1]
        p_i_z = p_i[2]
        this_joint_position = np.array([[p_i_x],
                                        [p_i_y],
                                        [p_i_z],
                                        [1]])

        # End effector joint
        if (index == -1):
            index = self.N_joints - 1

        # Store the original index of this joint
        orig_joint_index = index

        # Store the result of matrix multiplication
        running_multiplication = None

        # Start from the index of this joint and work backwards to index 0
        while (index >= 0):

            # If we are at the original joint index
            if (index == orig_joint_index):
                running_multiplication = hr_matrix(self.k[index], self.t[index], Q[index]) @ this_joint_position
            # If we are not at the original joint index
            else:
                running_multiplication = hr_matrix(self.k[index], self.t[index], Q[index]) @ running_multiplication

            index = index - 1

        # extract the points
        px = running_multiplication[0][0]
        py = running_multiplication[1][0]
        pz = running_multiplication[2][0]

        position_global_frame = np.array([px, py, pz])

        return position_global_frame

def pseudo_inverse(self, theta_start, p_eff_N, goal_position, max_steps=np.inf):
    v_step_size = 0.05
    theta_max_step = 0.2
    Q_j = theta_start
    p_end = np.array([goal_position[0], goal_position[1], goal_position[2]])
    p_j = self.position(Q_j, p_i=p_eff_N)
    delta_p = p_end - p_j
    j = 0
    while np.linalg.norm(delta_p) > 0.01 and j < max_steps:
        print(f'j{j}: Q[{Q_j}] , P[{p_j}]')
        v_p = delta_p * v_step_size / np.linalg.norm(delta_p)
        J_j = self.jacobian(Q_j, p_eff_N)
        J_invj = np.linalg.pinv(J_j)
        v_Q = np.matmul(J_invj, v_p)
        Q_j = Q_j + np.clip(v_Q, -1 * theta_max_step, theta_max_step)
        p_j = self.position(Q_j, p_i=p_eff_N)
        j = j + 1
        delta_p = p_end - p_j
    return Q_j

def main():
    #a 2D array that lists the different axes of rotation (rows) for each joint
    #assuming two joints, but I need to add the others
    k = np.array([[0, 0, 1], [0, 0, 1]])

    #a 2D array that lists the translations from the previous joint to the current joint
    #the first translation is from the base frame to joint 1 (ehich is equal to the base frame)
    #the second translation is from joint 1 to joint 2
    #t = tx, ty, tz
    a1 = 4.7
    a2 = 5.9
    a3 = 5.4
    a4 = 6.0
    t = np.array([[0, 0, 0], [a2, 0, a1]])

    #position of end effector in joint 2 (the last joint) frame
    p_eff_2 = [a4, 0, a3]
    k_c = RoboticArm(k, t)

    #starting joint angles in radians (joint 1, joint 2)
    q_0 = np.array([0, 0])

    #desired end effector position with respect to the base frame of the robotic arm
    endeffector_goalposition = np.array([4.0, 10.0, a1+a4])

    # Display the starting position of each joint in the global frame
    for i in np.arange(0, k_c.N_joints):
        print(f'joint {i} position = {k_c.position(q_0, index=i)}')

    print(f'end_effector = {k_c.position(q_0, index=-1, p_i=p_eff_2)}')
    print(f'goal = {endeffector_goal_position}')

    # Return joint angles that result in the end effector reaching endeffector_goal_position
    final_q = k_c.pseudo_inverse(q_0, p_eff_N=p_eff_2, goal_position=endeffector_goal_position, max_steps=500)

    # Final Joint Angles in degrees
    print('\n\nFinal Joint Angles in Degrees')
    print(f'Joint 1: {np.degrees(final_q[0])} , Joint 2: {np.degrees(final_q[1])}')


if __name__ == '__main__':
    main()




