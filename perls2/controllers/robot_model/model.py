import numpy as np
import perls2.controllers.utils.transform_utils as T
import scipy


class Model:

    def __init__(self):

        # robot states
        self.ee_pos = None
        self.ee_ori_mat = None
        self.ee_pos_vel = None
        self.ee_ori_vel = None
        self.joint_pos = None
        self.joint_vel = None
        self.joint_tau = None
        self.joint_dim = None

        # dynamics and kinematics
        self.J_pos = None
        self.J_ori = None
        self.J_full = None
        self.mass_matrix = None

        self.torque_compensation = None
        self.nullspace = None


    def update_states(self,
                      ee_pos,
                      ee_ori,
                      ee_pos_vel,
                      ee_ori_vel,
                      joint_pos,
                      joint_vel,
                      joint_tau,
                      joint_dim=None):

        self.ee_pos = ee_pos

        if ee_ori.shape == (3, 3):
            self.ee_ori_mat = ee_ori
        elif ee_ori.shape[0] == 4:
            self.ee_ori_mat = T.quat2mat(ee_ori)
        else:
            raise ValueError("orientation is not quaternion or matrix")

        self.ee_pos_vel = ee_pos_vel
        self.ee_ori_vel = ee_ori_vel

        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
        self.joint_tau = joint_tau

        # Only update the joint_dim and torque_compensation attributes if it hasn't been updated in the past
        if not self.joint_dim:
            if joint_dim is not None:
                # User has specified explicit joint dimension
                self.joint_dim = joint_dim
            else:
                # Default to joint_pos length
                self.joint_dim = len(joint_pos)
            # Update torque_compensation accordingly
            self.torque_compensation = np.zeros(self.joint_dim)

    def update_model(self,
                     J_pos,
                     J_ori,
                     mass_matrix):
        self.mass_matrix = mass_matrix
        self.J_full = np.concatenate((J_pos, J_ori))
        self.J_pos = J_pos
        self.J_ori = J_ori

    def update(self):
      pass