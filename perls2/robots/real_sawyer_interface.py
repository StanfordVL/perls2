"""
Class defining the interface to the Sawyer Robotsrobots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import numpy as np
import logging
import time
from perls2.robots.real_robot_interface import RealRobotInterface
from perls2.redis_interfaces.redis_interface import RobotRedisInterface
from perls2.redis_interfaces.redis_keys import *
from perls2.redis_interfaces.redis_values import *
# For dumping config dict to redis
import json


def bstr_to_ndarray(array_bstr):
    """Convert bytestring array to 1d array
    """
    return np.fromstring(array_bstr[1:-1], dtype=np.float, sep=',')


class RealSawyerInterface(RealRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 use_safenet=True,
                 use_moveit=True,
                 controlType='EEImpedance',
                 node_name='sawyer_interface'):
        """
        Initialize variables and wrappers
        """
        super().__init__(controlType=controlType, config=config)
        self.redisClient = RobotRedisInterface(**self.config['redis'])
        #logging.info("warming up redis connection - sleep for 10s")
        #time.sleep(10.0)
        self.update_model()

        logging.debug("Real Sawyer Interface created")

        # Check if redis connection already exists, if not
        # setup a new one.
        self.neutral_joint_angles = self.robot_cfg['neutral_joint_angles']
        self.RESET_TIMEOUT = 15       # Wait 3 seconds for reset to complete.
        self.GRIPPER_MAX_CLOSED = 1.0
        self.set_controller_params_from_config()
        self.connect()

    def connect(self):
        self.redisClient.set(ROBOT_ENV_CONN_KEY, 'True')

    def disconnect(self):
        self.redisClient.set(ROBOT_ENV_CONN_KEY, 'False')

    def reset(self):
        """ Reset arm to neutral configuration. Blocking call
        waits on redis flag to be cleared by the Control Interface

        TODO: This doesn't actually work, it just waits for the timeout.
        """
        logging.debug("Resetting robot")
        reset_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(),
                     ROBOT_CMD_TYPE_KEY: RESET}
        self.redisClient.mset(reset_cmd)
        # Wait for reset to be read by contrl interface.
        time.sleep(5)
        start = time.time()

        while (self.redisClient.get(ROBOT_RESET_COMPL_KEY) != b'True' and (time.time() - start < self.RESET_TIMEOUT)):
            time.sleep(0.01)

        if (self.redisClient.get(ROBOT_RESET_COMPL_KEY) == b'True'):
            print("reset successful")
        else:
            print("reset failed")

    @property
    def version(self):
        """
        List current versions of robot SDK, gripper, and robot
        :return: dictionary of version strings
        """
        pass

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
        # Get position as bytes string
        return self.redisClient.get(ROBOT_STATE_EE_POS_KEY)

    @property
    def ee_orientation(self):
        """
        Get the orientation of the end-effector.
        :return: a list of floats for the orientation quaternion [qx,
        qy, qz, qw]
        """
        return self.redisClient.get(ROBOT_STATE_EE_ORN_KEY)

    @property
    def ee_pose(self):
        """
        Get the pose of the end effector.
        :return: a list of floats for the position and orientation [x,
        y, z, qx, qy, qz, qw]
        """
        return self.redisClient.get(ROBOT_STATE_EE_POSE_KEY)

    @property
    def ee_v(self):
        """
        Get the current linear velocity of end effector in the eef
        frame.
        :return: a list of floats for the linear velocity m/s [vx, vy,
        vz]
        """
        return self.redisClient.get(ROBOT_STATE_EE_V_KEY)

    @property
    def ee_pose_euler(self):
        euler_orn = pb.getEulerFromQuaternion(self.ee_orientation)
        return list(self.ee_position) + list(euler_orn)

    @property
    def ee_w(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return self.redisClient.get(ROBOT_STATE_EE_OMEGA_KEY)

    @property
    def ee_twist(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return list(self.ee_v + self.ee_w)

    @property
    def ee_force(self):
        """
        Get the force (fx, fy, fz) on end effector
        :return: a list of floats for force N [fx, fy, fz]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_force'))

    @property
    def ee_torque(self):
        """
        Get the torque (mx, my, mz) on end effector
        :return: a list of floats for torque N*m [mx, my, mz]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_torque'))

    @property
    def ee_wrench(self):
        """
        Get the wrench on end effector
        :return: a list of floats for the wrench [fx, fy, fz, mx, my,
        mz]
        """
        return list(self.ee_force + self.ee_torque)

    @property
    def q(self):
        """
        Get the joint configuration of the robot arm.
        :return: a list of joint configurations in radian ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        return self.redisClient.get(ROBOT_STATE_Q_KEY)

    @q.setter
    def q(self, qd):
        self.redisClient.set(ROBOT_CMD_TYPE_KEY, 'joint_position')
        self.redisClient.set('robot::qd', str(qd))

    @property
    def dq(self):
        """
        Get the joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        dq_dict = self.redisClient.get_dict(ROBOT_STATE_DQ_KEY)
        dq = []
        if isinstance(dq_dict, dict):
            for limb_name in self.config['sawyer']['limb_joint_names']:
                dq.append(dq_dict[limb_name])
        dq = dq_dict
        return dq

    @property
    def ddq(self):
        """
        Get the joint accelerations of the robot arm.
        :return: a list of joint accelerations in radian/s^2, which
        is ordered by indices from small to large.
        Typically the order goes from base to end effector.
        """
        return NotImplemented

    @property
    def tau(self):
        """
        Get the joint torques of the robot arm.
        :return: a list of joint torques in N*m ordered by indices from
        small to large.
        Typically the order goes from base to end effector.
        """
        tau_dict = self.redisClient.get_dict(ROBOT_STATE_TAU_KEY)
        tau = []

        for limb_name in self.config['sawyer']['limb_joint_names']:
            tau.append(tau_dict[limb_name])

        return tau

    @property
    def jacobian(self):
        return self.redisClient.get(ROBOT_MODEL_JACOBIAN_KEY)

    @property
    def linear_jacobian(self):
        return self.redisClient.get(ROBOT_MODEL_L_JACOBIAN_KEY)

    @property
    def angular_jacobian(self):
        return self.redisClient.get(ROBOT_MODEL_A_JACOBIAN_KEY)

    @property
    def mass_matrix(self):
        return self.redisClient.get(ROBOT_MODEL_MASS_MATRIX_KEY)

    @property
    def N_q(self):
        return bstr_to_ndarray(self.redisClient.get('robot::N_q'))

    @property
    def gravity_vector(self):
        return np.zeros(7)


    # def set_controller_params_from_config(self):
    #     """Set controller parameters from config file to redis.
    #     """
    #     selected_type = self.config['sawyer_controller']['selected_type']
    #     self.control_config = self.config['controller']['Real'][selected_type]

    #     self.redisClient.mset({CONTROLLER_CONTROL_PARAMS_KEY: json.dumps(self.control_config),
    #                            CONTROLLER_CONTROL_TYPE_KEY: selected_type})

    #     cmd_type = CHANGE_CONTROLLER
    #     control_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(), ROBOT_CMD_TYPE_KEY: cmd_type}

    #     self.redisClient.mset(control_cmd)

    #     logging.debug("{} Control parameters set to redis: {}".format(selected_type, self.control_config))

    def set_goal_state(self, goal):
        self.redisClient.set("robot::desired_state", str(goal))

    def open_gripper(self):
        """Open Robot gripper
        """
        self.set_gripper_to_value(0.99)

    def close_gripper(self):
        """ Close robot gripper.
        """
        self.set_gripper_to_value(0.1)

    def set_gripper_to_value(self, value):
        """Set gripper to desired value
        """
        if (value > 1.0 or value < 0):
            raise ValueError("Invalid gripper value must be fraction between 0 and 1")

        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_KEY, (value * self.GRIPPER_MAX_CLOSED))
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY, time.time())