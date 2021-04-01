"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc   # For abstract class definitions
import six   # For abstract class definitions
import redis  # For communicating with RobotCtrlInterfaces
from scipy.spatial.transform import Rotation as R
from perls2.robots.robot_interface import RobotInterface
import numpy as np
import logging
import json
import time
from perls2.redis_interfaces.redis_keys import *
from perls2.redis_interfaces.redis_values import *
import perls2.controllers.utils.transform_utils as T
from perls2.redis_interfaces.redis_interface import RobotRedisInterface
AVAILABLE_CONTROLLERS = [EE_IMPEDANCE,
                         EE_POSTURE,
                         JOINT_VELOCITY,
                         JOINT_IMPEDANCE,
                         JOINT_TORQUE]


class RealRobotInterface(RobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self, config, controlType=None):
        """ Initialize the Real Robot Interface.

            Args:
                config (dict) : parsed YAML dictionary with config information
                controlType (str): string identifying controller for robot. Must be in
                    AVAILABLE CONTROLLERS

        """
        super().__init__(config=config, controlType=controlType )
        self.config = config
        self.robot_cfg  = self.config[self.config['world']['robot']]

    def create(config, controlType):
        """Factory for creating robot interfaces based on config type

            Args:
                config (dict): dictionary containing robot information
                controlType (str): string id for type of controller. Must be in
                    AVAILABLE CONTROLLERS.

            returns:
                RobotInterface

        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.real_sawyer_interface import RealSawyerInterface
            return RealSawyerInterface(
                config=config, controlType=controlType)
        if (config['world']['robot'] == 'panda'):
            from perls2.robots.real_panda_interface import RealPandaInterface
            return RealPandaInterface(
                config=config, controlType=controlType)
        else:
            raise ValueError("invalid robot interface type. choose 'sawyer'")

    def connect(self):
        self.redisClient.set(ROBOT_ENV_CONN_KEY, 'True')

    def disconnect(self):
        self.redisClient.set(ROBOT_ENV_CONN_KEY, 'False')

    def step(self):
        """Compatability only

            Actions are executed by CtrlInterface, so here do nothing.
        """
        pass

    def update_model(self):
        """Compatability only

            Model for Controller updated by Controller in CtrlInterface. do nothing.
        """
        pass

    def set_control_params(self):
        """Set the control parameters key on redis
        """
        control_config = self.controller_cfg['Real'][self.control_type]
        if self.use_safenet:
            control_config['position_limits'] =  np.array([self.safenet_ee_pos_lower, self.safenet_ee_pos_upper])
        self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(control_config))

    def reset(self):
        """ Reset robot to neutral positions.
        """
        raise NotImplementedError

    def set_safenet_boundaries(self, lower, upper):
        """Set position limits for existing controller.

        Controller will clip all end-effector goals to these limits.

        Args:
            lower (list): 3f lower boundary position limits for EE
            upper (list): 3f upper boundary position limits for EE

        Return: 
            None
        """
        assert (np.all(lower <= upper))
        self.safenet_ee_pos_upper = upper
        self.safenet_ee_pos_lower = lower
        self.use_safenet = True
        if self.controller is not None:
            self.set_controller_params_from_config()

    def set_controller_params_from_config(self):
        """Set controller parameters from config file to redis.
        """
        selected_type = self.config['world']['controlType']
        self.control_config = self.controller_cfg['Real'][selected_type]
        if self.use_safenet:
            self.control_config['position_limits'] =  [self.safenet_ee_pos_lower, self.safenet_ee_pos_upper]
        self.redisClient.mset({CONTROLLER_CONTROL_PARAMS_KEY: json.dumps(self.control_config),
                               CONTROLLER_CONTROL_TYPE_KEY: selected_type})

        cmd_type = CHANGE_CONTROLLER
        control_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(), ROBOT_CMD_TYPE_KEY: cmd_type}

        self.redisClient.mset(control_cmd)

        logging.debug("{} Control parameters set to redis: {}".format(selected_type, self.control_config))

    def change_controller(self, next_type):
        """Change to a different controller type.
        Args:
            next_type (str): keyword for desired control type.
                Choose from:
                    -'EEImpedance'
                    -'JointVelocity'
                    -'JointImpedance'
                    -'JointTorque'
        """
        if next_type in AVAILABLE_CONTROLLERS:
            self.controlType = next_type
            self.redisClient.set(CONTROLLER_CONTROL_TYPE_KEY, next_type)
            control_config = self.controller_cfg['Real'][self.controlType]
            self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(control_config))
            logging.debug("Changing controller to {} with params: {}".format(self.controlType, control_config))

            # Command to send
            cmd_type = CHANGE_CONTROLLER
            control_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(), ROBOT_CMD_TYPE_KEY: cmd_type}
            self.redisClient.mset(control_cmd)
            return self.controlType
        else:
            raise ValueError("Invalid control type " + "\nChoose from EEImpedance, JointVelocity, JointImpedance, JointTorque")

    def set_joint_positions(self, pose, **kwargs):
        """ Set goal as joint positions for robot by sending robot command.

            Args:
                pose (7f): list of joint positions to set as controller goal.

            Returns: None

            Example:

                env.robot_interface.set_joint_positions([0,-1.18,0.00,2.18,0.00,0.57,3.3161])

            Notes:
                Does not check if desired joint position within robot limits.
                TODO: add this check.
                Does not check if desired joint position appropriate dims.
                TODO: add this check.
        """
        self._check_controller(JOINT_IMPEDANCE)
        kwargs = {'cmd_type': SET_JOINT_POSITIONS, 'delta': None, 'set_qpos': pose}
        self.set_controller_goal(**kwargs)

    def set_joint_delta(self, delta, **kwargs):
        """ Set goal as delta from current joint position by sending robot command.

            Args
                delta (7f): list of delta from current joint position to set as goal.

            Returns: None

            Example:
                env.robot_interface.set_joint_delta([0.1, 0.2, 0.1, 0.05, -0.05, 0, 0.0])

            Notes:
                Does not check if desired joint position within robot limits.
                TODO: add this check.
                Does not check if desired joint position appropriate dims.
                TODO: add this check.
        """
        self._check_controller(JOINT_IMPEDANCE)
        if delta is not None:
            if isinstance(delta, np.ndarray):
                delta = delta.tolist()
        self._check_controller(JOINT_IMPEDANCE)

        kwargs['delta'] = delta
        kwargs['cmd_type'] = SET_JOINT_DELTA

        self.set_controller_goal(**kwargs)

    def move_ee_delta(self, delta, set_pos=None, set_ori=None):
        """ Use controller to move end effector by some delta.

        Args:
            delta (6f): delta position (dx, dy, dz) concatenated with delta orientation.
                Orientation change is specified as an Euler angle body XYZ rotation about the
                end effector link.
            set_pos (3f): end effector position to maintain while changing orientation.
                [x, y, z]. If not None, the delta for position is ignored.
            set_ori (4f): end effector orientation to maintain while changing orientation
                as a quaternion [qx, qy, qz, w]. If not None, any delta for orientation is ignored.

        Note: only for use with EE impedance controller
        Note: to fix position or orientation, it is better to specify using the kwargs than
            to use a 0 for the corresponding delta. This prevents any error from accumulating in
            that dimension.

        """
        self._check_controller([EE_IMPEDANCE, EE_POSTURE])
        if set_ori is not None:
            if len(set_ori) != 4:
                raise ValueError('set_ori incorrect dimensions, should be quaternion length 4')
            if isinstance(set_ori, np.ndarray):
                set_ori = set_ori.tolist()
        if set_pos is not None:
            if len(set_pos) != 3:
                raise ValueError('set_pos incorrect dimensions, should be length 3')
            if isinstance(set_pos, np.ndarray):
                set_pos = set_pos.tolist()

        if delta is not None:
            if isinstance(delta, np.ndarray):
                delta = delta.tolist()
        else:
            raise ValueError("delta cannot be none.")

        kwargs = {'cmd_type': MOVE_EE_DELTA, 'delta': delta, 'set_pos': set_pos, 'set_ori': set_ori}

        self.set_controller_goal(**kwargs)

    def set_ee_pose(self, set_pos, set_ori, **kwargs):
        """ Use controller to set end effector pose.

        Args: des pose (7f): [x, y , z, qx, qy, qz, w]. end effector pose as position + quaternion orientation

        """
        self._check_controller([EE_IMPEDANCE, EE_POSTURE])
        if not isinstance(set_pos, list):
            set_pos = set_pos.tolist()
        if not isinstance(set_ori, list):
            set_ori = set_ori.tolist()
        kwargs = {'cmd_type': SET_EE_POSE, 'delta': None, 'set_pos': set_pos, 'set_ori': set_ori}
        self.set_controller_goal(**kwargs)

    def set_controller_goal(self, cmd_type, **kwargs):
        """ Set the appropriate redis keys for the robot command. Internal use only.

            Sets the command timestamp, command type and controller goal.
            This version is specific for real robots.

            Args:
                cmd_type (str): string identifier for fn to execute. Must exactly match
                    function name for CtrlInterface.

                kwargs (dict): dictionary containing keys specific to robot command.

            Returns: None

            Example:
                        delta = [0., 0., 0.1, 0., 0., 0., 0.1]
                        kwargs['delta'] = delta
                        kwargs['cmd_type'] = SET_JOINT_DELTA
                        self.set_controller_goal(**kwargs)

        """
        control_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(),
                       ROBOT_CMD_TYPE_KEY: cmd_type,
                       CONTROLLER_GOAL_KEY: json.dumps(kwargs)}
        self.redisClient.mset(control_cmd)
        self.action_set = True

    def set_joint_torques(self, torques, **kwargs):
        """Use the controller to set joint torques.

        Args:
            des_torques (7f): torques to command.
        """
        if isinstance(torques, np.ndarray):
            torques = torques.tolist()

        kwargs = {'cmd_type': SET_JOINT_TORQUES, 'torques': torques}
        self.set_controller_goal(**kwargs)

    def set_joint_velocities(self, velocities, **kwargs):
        """Use the controller to set joint velocities.

        Args:
            dq (list): 7f list of desired joint velocities.
        """
        if isinstance(velocities, np.ndarray):
            velocities = velocities.tolist()
        kwargs = {'cmd_type': SET_JOINT_VELOCITIES, "velocities": velocities}
        self.set_controller_goal(**kwargs)

    @property
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    @property
    def name(self):
        """str of the name of the robot
        """
        raise NotImplementedError

    @property
    def ee_position(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.
        """
        raise NotImplementedError

    @property
    def ee_orientation(self):
        """list of four floats [qx, qy, qz, qw] of the orientation
        quaternion of the end-effector.
        """
        raise NotImplementedError

    @property
    def ee_pose(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    def ee_v(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    def ee_w(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    def ee_twist(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    def q(self):
        """List of 7f describing joint positions (rad) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def dq(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def tau(self):
        """List of 7f describing joint torques (Nm)

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def linear_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def angular_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    def mass_matrix(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    def open_gripper(self):
        """ Open robot gripper.
        """
        raise NotImplementedError

    def close_gripper(self):
        """ Close robot gripper
        """
        raise NotImplementedError

    def set_gripper_to_value(self, value):
        """ Set gripper to desired open/close value
        """
        raise NotImplementedError
