from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np
import time

class EEPostureController(EEImpController):
    """ Class definition for end effector impedance controller with posture control. 

    End effector uses pd control, with a desired joint position in null space.
    
    Attributes:
        posture (list): 7f joint positions to maintain within nullspace of task. 
        posture_gain (float): constant gain to control posture. 

    """
    def __init__(self, 
                 robot_model: Model,
                 input_max=1,
                 input_min=-1,
                 output_max=1.0,
                 output_min=-1.0,
                 kp=50,
                 damping=1,
                 posture_gain=0,
                 control_freq=20,
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=True, ):
        """ Initialize controller.
        """
        self.posture_gain = posture_gain
        super(EEPostureController, self).__init__(
            robot_model=robot_model, 
            input_max=input_max,
            input_min=input_min,
            output_max=output_max,
            output_min=output_min,
            kp=kp,
            damping=damping,
            control_freq=control_freq,
            position_limits=position_limits,
            orientation_limits=orientation_limits,
            interpolator_pos=interpolator_pos,
            interpolator_ori=interpolator_ori,
            uncouple_pos_ori=uncouple_pos_ori)


    def set_goal(self, delta,  
        set_pos=None, set_ori=None, posture=[0,-1.18,0.00,2.18,0.00,0.57,3.3161], **kwargs):

        super().set_goal(delta, set_pos, set_ori)
        self.goal_posture = posture


    def run_controller(self):
        # TODO: check if goal has been set.
        desired_vel_pos = 0.0
        desired_acc_pos = 0.0
        desired_vel_ori = 0.0
        desired_acc_ori = 0.0

        if self.interpolator_pos is not None:
            if self.interpolator_pos.order == 4:
                interpolated_results = self.interpolator_pos.get_interpolated_goal(self.model.ee_pos)
                desired_pos = interpolated_results[0:3]
                desired_vel_pos = interpolated_results[3:6]
                desired_acc_pos = interpolated_results[6:]
            else:
                desired_pos = self.interpolator_pos.get_interpolated_goal(self.model.ee_pos)
        else:
            desired_pos = np.array(self.goal_pos)

        if self.interpolator_ori is not None:
            #relative orientation based on difference between current ori and ref
            self.relative_ori = orientation_error(self.model.ee_ori_mat, self.ori_ref)

            interpolated_results = self.interpolator_ori.get_interpolated_goal(self.relative_ori)
            ori_error = interpolated_results[0:3]

            if self.interpolator_ori.order == 4:
                desired_vel_ori = interpolated_results[3:6]
                desired_acc_ori = interpolated_results[6:]
                self.ori_interpolate_started = True

        else:
            desired_ori = np.array(self.goal_ori)
            ori_error = orientation_error(desired_ori, self.model.ee_ori_mat)

        position_error = desired_pos - self.model.ee_pos
        vel_pos_error = desired_vel_pos - self.model.ee_pos_vel
        desired_force = (np.multiply(np.array(position_error), np.array(self.kp[0:3]))
                         + np.multiply(vel_pos_error, self.kv[0:3])) + desired_acc_pos

        vel_ori_error = desired_vel_ori - self.model.ee_ori_vel
        desired_torque = (np.multiply(np.array(ori_error), np.array(self.kp[3:6]))
                          + np.multiply(vel_ori_error, self.kv[3:6])) + desired_acc_ori
        
        lambda_full, lambda_pos, lambda_ori, nullspace_matrix = opspace_matrices(self.model.mass_matrix,
                                                                                 self.model.J_full,
                                                                                 self.model.J_pos,
                                                                                 self.model.J_ori)
        if self.uncoupling:
            decoupled_force = np.dot(lambda_pos, desired_force)
            decoupled_torque = np.dot(lambda_ori, desired_torque)
            decoupled_wrench = np.concatenate([decoupled_force, decoupled_torque])
        else:
            desired_wrench = np.concatenate([desired_force, desired_torque])
            decoupled_wrench = np.dot(lambda_full, desired_wrench)


        self.torques = np.dot(self.model.J_full.T, decoupled_wrench) + self.model.torque_compensation
        
        posture_error = np.array(self.model.joint_pos - self.goal_posture) 
        pgain_vec = np.array([-self.posture_gain] * 7)
        posture_control = np.multiply(pgain_vec, posture_error)

        self.torques = self.torques + np.dot(nullspace_matrix.T, posture_control)
        # todo: null space! (as a wrapper)

        return self.torques
