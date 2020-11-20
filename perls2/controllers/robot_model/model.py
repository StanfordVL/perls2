import numpy as np
import perls2.controllers.utils.transform_utils as T
import scipy


class Model:
    """Robot state and dynamics model.

    Attributes:
        ee_pos (list): 3f xyz position (m) of end-effector in world frame.
        ee_ori_quat (list): 4f orientation of end-effector as quaternion
          in world frame.
        ee_ori_mat (list): (3f, 3f) orientation of end-effector in world frame
          as a rotation matrix.
        ee_pos_vel (list): 3f xyz velocity of end-effector.
        ee_ori_vel (list): 3f angular velocity of end-effector about
          world frame axes.
        joint_pos (list): 7f joint positions ordered from base to ee(radians)
        joint_vel (list): 7f joint velocity ordered from base to ee(rad/s)
        joint_torque (list): 7f joint torques ordered from base to ee (Nm)
        J_pos (list): (3, 7) Jacobian mapping ee linear velocity to joint velocity.
        J_ori (list): (3, 7) Jacobian mapping ee angular velocity to joint velocity.
        J_ful (list): (6, 7) Jacobian mapping ee twist to joint velocity.
        mass_marix (list): (7,7) Joint space inertia matrix
        off_set_mass_matrix (bool): flag to offset mass_matrix at last 3 joints.
        mass_matrix_offset_val (list): 3f list of offsets to add to the mass matrix
          diagonal's last three elements. Used for real robots to adjust for high
          friction at end joints.
        torque_compensation (list): Additional compensation torques, usually used for
          gravity.
        nullspace (list): List of nullspace constrained torques for the osc task.
    """
    def __init__(self, offset_mass_matrix=True):
        """Initialize the robot model.

        Args:
          offset_mass_matrix (bool): flag to turn on mass_matrix offset.
        """

        # robot states
        self.ee_pos = None
        self.ee_ori_quat = None
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
        self.offset_mass_matrix = offset_mass_matrix
        self.mass_matrix_offset_val = [0.1, 0.1, 0.1]
        self.torque_compensation = None
        self.nullspace = None
        self._compile_jit_functions()

    def _compile_jit_functions(self):
        dummy_mat = np.eye(3)
        dummy_quat = np.zeros(4)
        dummy_quat[-1] = 1.
        T.mat2quat(dummy_mat)
        T.quat2mat(dummy_quat)
        dummy_J = np.zeros((6,7))
        dummy_dq = np.zeros(7)
        T.calc_twist(dummy_J, dummy_dq)

        

    def update_states(self,
                      ee_pos,
                      ee_ori,
                      ee_pos_vel,
                      ee_ori_vel,
                      joint_pos,
                      joint_vel,
                      joint_tau,
                      joint_dim=None,
                      torque_compensation=None):

        self.ee_pos = ee_pos
        if ee_ori.shape == (3, 3):
            self.ee_ori_mat = ee_ori
            self.ee_ori_quat = T.mat2quat(ee_ori)
        elif ee_ori.shape[0] == 4:
            self.ee_ori_quat = ee_ori
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
            #self.torque_compensation = np.zeros(self.joint_dim)
        if torque_compensation is None:
          self.torque_compensation = np.zeros(7)
        else:  
          self.torque_compensation = np.asarray(torque_compensation)

    def update_model(self,
                     J_pos,
                     J_ori,
                     mass_matrix):

        self.mass_matrix = mass_matrix
        if self.offset_mass_matrix:
          mm_weight_indices = [(4,4), (5,5), (6,6)]
          for i in range(3):
            self.mass_matrix[mm_weight_indices[i]] += self.mass_matrix_offset_val[i]

        self.J_full = np.concatenate((J_pos, J_ori))
        self.J_pos = J_pos
        self.J_ori = J_ori

    def update(self):
      pass

    def update_state_model(self,
                      ee_pos,
                      ee_ori,
                      joint_pos,
                      joint_vel,
                      joint_tau,
                      J_full, 
                      mass_matrix,
                      joint_dim=None,
                      torque_compensation=None, 
                      ):
      """Update state and model together. 

      Helpful if you need to calculate the ee twist using the jacobian.
      """

      ee_twist = T.calc_twist(J_full, joint_vel)
      ee_pos_vel = ee_twist[0:3]
      ee_ori_vel = ee_twist[3:]
      self.update_states(ee_pos=ee_pos, 
        ee_ori=ee_ori,
        ee_pos_vel=ee_pos_vel, 
        ee_ori_vel=ee_ori_vel, 
        joint_vel=joint_vel,
        joint_pos=joint_pos,
        joint_tau=joint_tau)
      J_pos = J_full[0:3,:]
      J_ori = J_full[3:, :]
      self.update_model(J_pos=J_pos, J_ori=J_ori,
        mass_matrix=mass_matrix)