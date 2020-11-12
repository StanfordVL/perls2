"""Abstract Control Interface for Robots.
"""

import time
import numpy as np
import logging
from perls2.utils.yaml_config import YamlConfig

from perls2.ros_interfaces.redis_interface import RobotRedisInterface as RobotRedis
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
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


import perls2.controllers.utils.transform_utils as T

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
        self.config = YamlConfig(config)

        # Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False

        # Control Init
        self.controlType = controlType
        self.model = Model()

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

############################################################################
    # Properties for perls2 RobotInterface
    @property
    def env_connected(self):
        """ Flag indicating if environment and RobotInterface are connected.
        """
        return self.redisClient.get(ROBOT_ENV_CONN_KEY)

    @property
    def cmd_type(self):
        """ Redis key identifying the type of command robot should execute
        """
        return self.redisClient.get(ROBOT_CMD_TYPE_KEY)

    @property
    def des_gripper_state(self):
        return float(self.redisClient.get(ROBOT_SET_GRIPPER_CMD_KEY))

    @property
    def controller_goal(self):
        return self.redisClient.get(CONTROLLER_GOAL_KEY)

    @property
    def controlType(self):
        return self._controlType

    @controlType.setter
    def controlType(self, new_type):
        self._controlType = new_type

    def get_gripper_cmd_tstamp(self):
        return self.redisClient.get(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY)

    def get_cmd_tstamp(self):
        return self.redisClient.get(ROBOT_CMD_TSTAMP_KEY)

    def check_for_new_cmd(self):
        cmd_tstamp = self.get_cmd_tstamp()
        if self.last_cmd_tstamp is None or (self.last_cmd_tstamp != cmd_tstamp):
            self.last_cmd_tstamp = cmd_tstamp
            return True
        else:
            return False

    def run(self):
        if (self.env_connected == b'True'):
            self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())

        while True:
            start = time.time()
            self.log_start_times.append(start)
            if (self.env_connected == b'True'):
                if self.check_for_new_cmd():
                    self.process_cmd(self.cmd_type)
                if self.check_for_new_gripper_cmd():
                    self.process_gripper_cmd()
                self.step(start)
            else:
                break