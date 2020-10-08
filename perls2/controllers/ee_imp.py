from perls2.controllers.base_controller import Controller
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np
import time
import math


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

        orientation_limits (2-list of float or 2-list of list of floats): [Lower_bounds, upper_bounds]. Limits (rad) below and above which the
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
                 kv=None,
                 damping=1,
                 control_freq=20,
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=False,
                 ):

        super(EEImpController, self).__init__()
        # Logging
        self.intera_ee_pos_log = []
        self.intera_ee_ori_log = []
        self.intera_ee_v_log = []
        self.intera_ee_w_log = []
        self.ori_error_log = []
        self.pos_error_log = []
        self.des_pos_log = []
        self.des_ori_log =[]
        self.interp_pos_log = []
        self.interp_ori_log = []

        # input and output max and min
        self.input_max = np.array(input_max)
        self.input_min = np.array(input_min)
        self.output_max = np.array(output_max)
        self.output_min = np.array(output_min)

        # limits
        self.position_limits = position_limits
        self.orientation_limits = orientation_limits

        # kp kv
        if kp is list:
            self.kp = kp
        else:
            self.kp = np.ones(6) * kp

        # Set kv using damping if kv not explicitly set.
        if kv is not None:
            self.kv = kv
        else:
            self.kv = np.ones(6) * 2 * np.sqrt(self.kp) * damping

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator_pos = interpolator_pos
        self.interpolator_ori = interpolator_ori

        # whether or not pos and ori want to be uncoupled
        self.uncoupling = uncouple_pos_ori

        # initialize
        self.goal_ori = None
        self.goal_pos = None

        self.prev_goal_ori = None
        self.prev_goal_pos = None

        self.relative_ori = np.zeros(3)

        self.set_goal(np.zeros(6))

        self._compile_jit_functions()

    def _compile_jit_functions(self):
        """
        Helper function to incur the cost of compiling jit functions used by this class
        and robosuite upfront.
        """
        dummy_mat = np.eye(3)
        dummy_quat = np.zeros(4)
        dummy_quat[-1] = 1.
        T.mat2quat(dummy_mat)
        T.quat2mat(dummy_quat)

        #dummy_nullspace_matrix = np.zeros((7, 7))
        _, _, _, dummy_nullspace_matrix =opspace_matrices(
            mass_matrix=self.model.mass_matrix,
            J_full=self.model.J_full,
            J_pos=self.model.J_pos,
            J_ori=self.model.J_ori,
        )
        orientation_error(dummy_mat, dummy_mat)

    def set_goal(self, delta, set_pos=None, set_ori=None, **kwargs):
        if not (isinstance(set_pos, np.ndarray)) and set_pos is not None:
            set_pos = np.array(set_pos)
        if not (isinstance(set_ori, np.ndarray)) and set_ori is not None:
            if len(set_ori) != 4:
                raise ValueError("invalid ori dimensions, should be quaternion.")
            else:
                set_ori = T.quat2mat(np.array(set_ori))
                print("set_ori\t{}".format(set_ori))

        self.model.update()

        if delta is not None:
            if (len(delta) <6):
              raise ValueError("incorrect delta dimension")

            scaled_delta = self.scale_action(delta)

            self.goal_ori = set_goal_orientation(scaled_delta[3:],
                                                     self.model.ee_ori_mat,
                                                     orientation_limit=self.orientation_limits,
                                                     set_ori=set_ori,
                                                     axis_angle=True)

            self.goal_pos = set_goal_position(scaled_delta[:3],
                                              self.model.ee_pos,
                                              position_limit=self.position_limits,
                                              set_pos=set_pos)
        else:


            scaled_delta = None

            self.goal_ori = set_goal_orientation(None,
                                                 self.model.ee_ori_mat,
                                                 orientation_limit=self.orientation_limits,
                                                 set_ori=set_ori,
                                                 axis_angle=True)

            self.goal_pos = set_goal_position(None,
                                              self.model.ee_pos,
                                              position_limit=self.position_limits,
                                              set_pos=set_pos)


        if self.interpolator_pos is not None:
            self.interpolator_pos.set_goal(self.goal_pos)

        if self.interpolator_ori is not None:

            self.ori_ref = np.array(self.model.ee_ori_mat) #reference is the current orientation at start
            self.interpolator_ori.set_goal(T.mat2quat(self.goal_ori)) # goal is the clipped orientation.
            self.relative_ori = np.zeros(3) #relative orientation always starts at 0

    def run_controller(self):

            # TODO: check if goal has been set.
        desired_vel_pos = np.asarray([0.0, 0.0, 0.0])
        desired_acc_pos = np.asarray([0.0, 0.0, 0.0])
        desired_vel_ori = np.asarray([0.0, 0.0, 0.0])
        desired_acc_ori = np.asarray([0.0, 0.0, 0.0])

        if self.interpolator_pos is not None:
            desired_pos = self.interpolator_pos.get_interpolated_goal()
            self.interp_pos_log.append(desired_pos)
        else:
            desired_pos = np.array(self.goal_pos)

        if self.interpolator_ori is not None:

            desired_ori = T.quat2mat(self.interpolator_ori.get_interpolated_goal())
            ori_error = orientation_error(desired_ori, self.model.ee_ori_mat)
            publish_pose((desired_pos, T.mat2quat(desired_ori)), 'interp_pose')
        else:
            desired_ori = np.array(self.goal_ori)
            ori_error = orientation_error(desired_ori, self.model.ee_ori_mat)

        # publish_pose((self.goal_pos, T.mat2quat(self.goal_ori)), 'goal_pose')

        position_error = desired_pos - self.model.ee_pos
        vel_pos_error = desired_vel_pos - self.model.ee_pos_vel
        desired_force = (np.multiply(np.array(position_error), np.array(self.kp[0:3]))
                         + np.multiply(vel_pos_error, self.kv[0:3])) + desired_acc_pos

        vel_ori_error = desired_vel_ori - self.model.ee_ori_vel
        desired_torque = (np.multiply(np.array(ori_error), np.array(self.kp[3:]))
                          + np.multiply(vel_ori_error, self.kv[3:])) + desired_acc_ori

        lambda_full, lambda_pos, lambda_ori, nullspace_matrix = opspace_matrices(self.model.mass_matrix,
                                                                                 self.model.J_full,
                                                                                 self.model.J_pos,
                                                                                 self.model.J_ori)
        self.nullspace_matrix = nullspace_matrix
        if self.uncoupling:
            decoupled_force = np.dot(lambda_pos, desired_force)
            decoupled_torque = np.dot(lambda_ori, desired_torque)
            decoupled_wrench = np.concatenate([decoupled_force, decoupled_torque])
        else:
            desired_wrench = np.concatenate([desired_force, desired_torque])
            decoupled_wrench = np.dot(lambda_full, desired_wrench)

        self.torques = np.dot(self.model.J_full.T, decoupled_wrench) + self.model.torque_compensation

        self.des_pos_log.append(self.goal_pos)
        self.des_ori_log.append(self.goal_ori)
        self.intera_ee_pos_log.append(self.model.ee_pos)
        self.intera_ee_ori_log.append(self.model.ee_ori_mat)
        self.intera_ee_v_log.append(self.model.ee_pos_vel)
        self.intera_ee_w_log.append(self.model.ee_ori_vel)
        self.pos_error_log.append(position_error)
        self.ori_error_log.append(ori_error)

        return self.torques

    def reset_goal(self):
        self.set_goal(np.zeros(6))
