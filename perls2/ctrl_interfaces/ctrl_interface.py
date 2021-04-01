"""Abstract Control Interface for Robots.
"""

import time
import six
import abc
import numpy as np
import logging
from perls2.utils.yaml_config import YamlConfig

import perls2.redis_interfaces.redis_keys as R 
# Robot Interface and Controller Imports
from perls2.robots.robot_interface import RobotInterface
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.joint_torque import JointTorqueController
from perls2.controllers.joint_vel import JointVelController
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.interpolator.linear_ori_interpolator import LinearOriInterpolator
from perls2.controllers.robot_model.model import Model


LOOP_LATENCY = 0.000
LOOP_TIME = (1.0 / 500.0) - LOOP_LATENCY


@six.add_metaclass(abc.ABCMeta)
class CtrlInterface(RobotInterface):
    """ Abstract class definition for Control Interface.

    Interface creates and monitors RedisServer for new commands from RobotInterface.
    Commands are interpreted and converted into set of joint torques sent to robot.

    Attributes:
        config (YamlConfig): YamlConfig expressing default sawyer_ctrl_parameters and redis connection info.

    """
    def __init__(self,
                 config,
                 controlType):
        """
        Initialize robot for control.
        """
        try:
            self.config = YamlConfig(config)
        except TypeError:
            self.config = config

        # Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False

        # Control Init
        self.controlType = controlType
        self.model = Model(offset_mass_matrix=True)

    @property
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    @property
    def name(self):
        """
        Get the name of the robot
        :return: 'Sawyer'
        """
        return self._params.get_robot_name()

    @property
    def ee_position(self):
        """
        Get the position of the end-effector.
        :return: a list of floats for the position [x, y, z]
        """
        raise NotImplementedError

    @property
    def ee_orientation(self):
        """
        Get the orientation of the end-effector.
        :return: a list of floats for the orientation quaternion [qx,
        qy, qz, qw]
        """
        raise NotImplementedError

    @property
    def ee_pose(self):
        """
        Get the pose of the end effector.
        :return: a list of floats for the position and orientation [x,
        y, z, qx, qy, qz, qw]
        """
        raise NotImplementedError

    @property
    def ee_v(self):
        """
        Get the current linear velocity of end effector in the eef
        frame.
        :return: a list of floats for the linear velocity m/s [vx, vy,
        vz]
        """
        raise NotImplementedError

    def get_ee_v_world(self):
        """
        Returns ee_v in world frame, after applying transformations from eef ori.
        """
        raise NotImplementedError

    @property
    def ee_omega(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        raise NotImplementedError

    @property
    def ee_w(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        raise NotImplementedError


    def get_ee_omega_world(self):
        """
        Returns ee_v in world frame, after applying transformations from eef ori.
        """
        return np.dot(np.array(self.ee_orientation), np.transpose(np.array(self.ee_omega)))

    @property
    def ee_twist(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return list(self.ee_v + self.ee_omega)

    @property
    def ee_force(self):
        """
        Get the force (fx, fy, fz) on end effector
        :return: a list of floats for force N [fx, fy, fz]
        """
        raise NotImplementedError

    @property
    def ee_torque(self):
        """
        Get the torque (mx, my, mz) on end effector
        :return: a list of floats for torque N*m [mx, my, mz]
        """
        raise NotImplementedError

    @property
    def ee_wrench(self):
        """
        Get the wrench on end effector
        :return: a list of floats for the wrench [fx, fy, fz, mx, my,
        mz]
        """
        raise NotImplementedError

    @property
    def q(self):
        """
        Get the joint configuration of the robot arm.
        :return: a list of joint configurations in radian ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        raise NotImplementedError

    @property
    def dq(self):
        """
        Get the joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        raise NotImplementedError

    @property
    def tau(self):
        """
        Get the joint torques of the robot arm.
        :return: a list of joint torques in N*m ordered by indices from
        small to large.
        Typically the order goes from base to end effector.
        """
        raise NotImplementedError

    @property
    def num_joints(self):
        """ Number of joints according to pybullet.
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

############################################################################
    # Properties for perls2 RobotInterface
    @property
    def env_connected(self):
        """ Boolean flag indicating if environment and RobotInterface are connected.
        """
        return (self.redisClient.get(R.ROBOT_ENV_CONN_KEY) == b'True')

    @property
    def cmd_type(self):
        """ Redis key identifying the type of command robot should execute
        """
        return self.redisClient.get(R.ROBOT_CMD_TYPE_KEY)

    @property
    def des_gripper_state(self):
        return float(self.redisClient.get(R.ROBOT_SET_GRIPPER_CMD_KEY))

    @property
    def controller_goal(self):
        return self.redisClient.get(R.CONTROLLER_GOAL_KEY)

    @property
    def controlType(self):
        return self._controlType

    @controlType.setter
    def controlType(self, new_type):
        self._controlType = new_type

    def get_gripper_cmd_tstamp(self):
        return self.redisClient.get(R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY)

    def get_cmd_tstamp(self):
        return self.redisClient.get(R.ROBOT_CMD_TSTAMP_KEY)

    def check_for_new_cmd(self):
        cmd_tstamp = self.get_cmd_tstamp()
        if self.last_cmd_tstamp is None or (self.last_cmd_tstamp != cmd_tstamp):
            self.last_cmd_tstamp = cmd_tstamp
            return True
        else:
            return False

    def process_cmd(self, cmd_type):
        """ process command from redis

        Args:
            cmd_type (str): byte-array string from redis cmd key
        """
        if (cmd_type.decode() == R.SET_EE_POSE):
            self.set_ee_pose(**self.controller_goal)
        elif (cmd_type.decode() == R.MOVE_EE_DELTA):
            self.move_ee_delta(**self.controller_goal)
        elif(cmd_type.decode() == R.SET_JOINT_DELTA):
            self.set_joint_delta(**self.controller_goal)
        elif (cmd_type.decode() == R.SET_JOINT_POSITIONS):
            self.set_joint_positions(**self.controller_goal)
        elif (cmd_type.decode() == R.SET_JOINT_TORQUES):
            self.set_joint_torques(**self.controller_goal)
        elif (cmd_type.decode() == R.SET_JOINT_VELOCITIES):
            self.set_joint_velocities(**self.controller_goal)
        elif(cmd_type.decode() == R.RESET):
            logging.info("RESET Command received.")
            self.redisClient.set(R.ROBOT_RESET_COMPL_KEY, 'False')
            self.reset_to_neutral()
            self.action_set = False
        elif (cmd_type.decode() == R.CHANGE_CONTROLLER):
            logging.info("CHANGE CONTROLLER COMMAND RECEIVED")
            self.controller = self.make_controller_from_redis(
                self.get_control_type(),
                self.get_controller_params())
        else:
            logging.warn("Unknown command: {}".format(cmd_type))

    def make_controller_from_redis(self, control_type, controller_dict):
        """ Make controller given args obtained from redis. 

        Args: 
            control_type (str): type of controller. Must match redis string literal.
            controller_dict (dict): kwargs for constructing the controller. 

        Returns:
            controller (Controller): appropriate perls2.controllers Controller given
                the kwargs.
                
        """
        print("Making controller {} with params: {}".format(control_type, controller_dict))
        if control_type == R.EE_IMPEDANCE:
            interp_kwargs = {'max_dx': 0.005,
                             'ndim': 3,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            self.interpolator_ori = LinearOriInterpolator(**interp_kwargs)
            controller = EEImpController(
                self.model,
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type == R.EE_POSTURE:
            interp_kwargs = {'max_dx': 0.005,
                             'ndim': 3,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            self.interpolator_ori = LinearOriInterpolator(**interp_kwargs)
            controller = EEPostureController(
                self.model,
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type == R.JOINT_IMPEDANCE:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            controller =  JointImpController(
                self.model,
                interpolator_qpos=self.interpolator_pos,
                **controller_dict)
        elif control_type == R.JOINT_TORQUE:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            controller =  JointTorqueController(
                self.model,
                interpolator=self.interpolator_pos,
                **controller_dict)
        elif control_type == R.JOINT_VELOCITY:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            controller =  JointVelController(
                self.model,
                interpolator=self.interpolator_pos,
                **controller_dict)
        else:
            raise ValueError("Invalid control type.")
        self.controlType = control_type
        return controller

    def get_controller_params(self):
        return self.redisClient.get(R.CONTROLLER_CONTROL_PARAMS_KEY)

    def get_control_type(self):
        return self.redisClient.get(R.CONTROLLER_CONTROL_TYPE_KEY).decode()

########################################################################
# Step and Loop Functions
    def step(self, start):
        """Update the robot state and model, set torques from controller

        Note this is different from other robot interfaces, as the Sawyer
        low-level controller takes gravity compensation into account.
        """
        self.update_model()

        if self.action_set:
            torques = self.controller.run_controller()
            torques = np.clip(torques, self.CLIP_CMD_TORQUES[0], self.CLIP_CMD_TORQUES[1] )
            self.set_torques(torques)

            while (time.time() - start < LOOP_TIME):
                pass

        else:
            pass

    def run(self):
        if (self.env_connected == b'True'):
            self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())

        while True:
            start = time.time()
            self.log_start_times.append(start)
            if (self.env_connected == b'True'):
                if self.check_for_new_cmd():
                    self.process_cmd(self.cmd_type)
                self.step(start)
            else:
                break