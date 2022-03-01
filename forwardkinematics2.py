import numpy as np
import math
from forwardkinematics import d_h_table_0_1, d_h_table_1_2, d_h_table_2_3, d_h_table_3_4, d_h_table_4_5, d_h_table_5_6, end_effector_position
from forwardkinematics import transform_0_1, transform_1_2, transform_2_3, transform_3_4, transform_4_5, transform_5_6, transform_0_6

class robot:

    def __init__(self, **kwargs):
        self.xroot = kwargs.get('xroot', 0)
        self.yroot = kwargs.get('yroot', 0)
        self.thetas = np.array([[]], dtype=np.float)
        self.joints = np.array([[self.xroot, self.yroot, 0, 1]], dtype=np.float).T
        self.lengths = []

    def get_transform_matrix(self):

        end_effector_position = np.array(end_effector_position)

        return end_effector_position

    def update_joint_coordinates(self):

        T= self.get_transform_matrix(self.thetas[0])