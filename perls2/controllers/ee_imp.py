from perls2.controllers.base_controller import Controller
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np
import time

class EEImpController(Controller):
    """ Class definition for End-effector Impedance Controller. 

    End effector impedance uses PD control to reach desired end-effector 
    position and orientation. 

    Attributes: 
        input_max (float or list of float): Maximum above which an inputted action will be clipped. Can be either be
            a scalar (same value for all action dimensions), or a list (specific values for each dimension). If the
            latter, dimension should be the same as the control dimension for this controller
        
        input_min (float or list of float): Minimum below which an inputted action will be clipped. Can be either be
            a scalar (same value for all action dimensions), or a list (specific values for each dimension). If the
            latter, dimension should be the same as the control dimension for this controller
        
        output_max (float or list of float): Maximum which defines upper end of scaling range when scaling an input
            action. Can be either be a scalar (same value for all action dimensions), or a list (specific values for
            each dimension). If the latter, dimension should be the same as the control dimension for this controller
        
        output_min (float or list of float): Minimum which defines upper end of scaling range when scaling an input
            action. Can be either be a scalar (same value for all action dimensions), or a list (specific values for
            each dimension). If the latter, dimension should be the same as the control dimension for this controller
        
        kp (float or list of float): positional gain for determining desired torques based upon the pos / ori errors.
            Can be either be a scalar (same value for all action dims), or a list (specific values for each dim)
        
        damping (float or list of float): used in conjunction with kp to determine the velocity gain for determining
            desired torques based upon the pos / ori errors. Can be either be a scalar (same value for all action dims),
            or a list (specific values for each dim)
        policy_freq (int): Frequency at which actions from the robot policy are fed into this controller
        
        position_limits (2-list of float or 2-list of list of floats): Limits (m) below and above which the magnitude
            of a calculated goal eef position will be clipped. Can be either be a 2-list (same min/max value for all
            cartesian dims), or a 2-list of list (specific min/max values for each dim)
        
        orientation_limits (2-list of float or 2-list of list of floats): Limits (rad) below and above which the
            magnitude of a calculated goal eef orientation will be clipped. Can be either be a 2-list
            (same min/max value for all joint dims), or a 2-list of list (specific min/mx values for each dim)
        
        interpolator_pos (Interpolator): Interpolator object to be used for interpolating from the current position to
            the goal position during each timestep between inputted actions
        
        interpolator_ori (Interpolator): Interpolator object to be used for interpolating from the current orientation
            to the goal orientation during each timestep between inputted actions
        
        control_ori (bool): Whether inputted actions will control both pos and ori or exclusively pos
        
        uncouple_pos_ori (bool): Whether to decouple torques meant to control pos and torques meant to control ori
        
        **kwargs: Does nothing; placeholder to "sink" any additional arguments so that instantiating this controller
            via an argument dict that has additional extraneous arguments won't raise an error

    """

    def __init__(self,
                 robot_model,
                 input_max=1,
                 input_min=-1,
                 output_max=1.0,
                 output_min=-1.0,
                 kp=50,
                 damping=1,
                 control_freq=20,
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=True,
                 **kwargs):

        super(EEImpController, self).__init__()
        # input and output max and min
        self.input_max = input_max
        self.input_min = input_min
        self.output_max = output_max
        self.output_min = output_min

        # limits
        self.position_limits = position_limits
        self.orientation_limits = orientation_limits

        # kp kv
        if kp is list:
            self.kp = kp
        else:
            self.kp = np.ones(6) * kp
        self.kv = np.ones(6) * 2 * np.sqrt(self.kp) * damping

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator_pos = interpolator_pos
        self.interpolator_ori = interpolator_ori
        # self.interpolator_ori = None
        # todo: orientation interpolators change to relative! refactor!

        # whether or not pos and ori want to be uncoupled
        self.uncoupling = uncouple_pos_ori

        # initialize
        self.goal_ori = None
        self.goal_pos = None

        self.relative_ori = np.zeros(3)

        self.set_goal(np.zeros(6))

    def set_goal(self, delta, set_pos=None, set_ori=None, **kwargs):
        self.model.update()

        if delta is not None:
            if (len(delta) <6):
              raise ValueError("incorrect delta dimension")

            scaled_delta = self.scale_action(delta)
        else:
            scaled_delta = None

        self.goal_ori = set_goal_orientation(scaled_delta[3:],
                                             self.model.ee_ori_mat,
                                             orientation_limit=self.orientation_limits,
                                             set_ori=set_ori)
        self.goal_pos = set_goal_position(scaled_delta[:3],
                                          self.model.ee_pos,
                                          position_limit=self.position_limits,
                                          set_pos=set_pos)

        if self.interpolator_pos is not None:
            self.interpolator_pos.set_goal(self.goal_pos)

        if self.interpolator_ori is not None:
            self.ori_ref = np.array(self.model.ee_ori_mat) #reference is the current orientation at start
            self.interpolator_ori.set_goal(orientation_error(self.goal_ori, self.ori_ref)) #goal is the total orientation error
            self.relative_ori = np.zeros(3) #relative orientation always starts at 0
    
    
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
        # todo: null space! (as a wrapper)
        return self.torques
