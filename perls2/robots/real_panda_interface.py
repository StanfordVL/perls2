"""Class defining interface to Real Panda Robots.

RealPandaInterfaces provide access to robot states, and the ability to send
commands to real Franka Panda arms.

Operation:

RealPandaInterfaces connect to a redis-server, and send commands to robots by
setting keys on the redis db. These actions are interpreted by the PandaCtrlInterface
running on the NUC, which converts these actions along with the robot state into torque
commands. RealPandaInterfaces obtain robot proprioception by querying the redis-db.

"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
import time
import numpy as np
from perls2.robots.real_robot_interface import RealRobotInterface
from perls2.redis_interfaces.redis_interface import PandaRedisInterface
from perls2.redis_interfaces.redis_keys import *
from perls2.redis_interfaces.redis_values import *
import perls2.redis_interfaces.panda_redis_keys as P
import perls2.controllers.utils.transform_utils as T
import logging
logger = logging.getLogger(__name__)


class RealPandaInterface(RealRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 controlType='EEImpedance'):
        """Initialize the real panda interface.
        """
        super().__init__(controlType=controlType, config=config)

        # Create redis interface specific to panda.
        redis_config = self.config['redis']
        self.redisClient = PandaRedisInterface(**self.config['redis'])
        self.last_redis_update = None
        self._get_state_from_redis()
        self.RESET_TIMEOUT = 10
        self.GRIPPER_MAX_VALUE = 0.0857
        self.MAX_REDIS_STALE_TIME = 0.005 # time in s (20Hz) after which to update redis states.
        self.neutral_joint_angles = np.array(self.config['panda']['neutral_joint_angles'])
        self.redisClient.set_reset_q(ROBOT_RESET_Q_KEY, self.neutral_joint_angles)
        self.set_controller_params_from_config()
        self.connect()


    def step(self):
        self._get_state_from_redis()

    def reset(self):
        logger.info("Resetting Panda Robot. Confirm on driver.")
        self.redisClient.set_reset_q(ROBOT_RESET_Q_KEY, self.neutral_joint_angles)        
        reset_cmd = {ROBOT_CMD_TSTAMP_KEY: time.time(),
                     ROBOT_CMD_TYPE_KEY: RESET}

        self.redisClient.mset(reset_cmd)
        # Wait for reset to be read by contrl interface.
        time.sleep(self.RESET_TIMEOUT)
        start = time.time()

        while (self.redisClient.get(ROBOT_RESET_COMPL_KEY) != b'True' and (time.time() - start < self.RESET_TIMEOUT)):
            time.sleep(0.01)

        if (self.redisClient.get(ROBOT_RESET_COMPL_KEY) == b'True'):
            logger.info("reset successful")
        else:
            logger.info("reset failed")
        self._get_state_from_redis()

    def _get_state_from_redis(self):
        """ Get states from redis and update all relevant attributes.

        """
        if self.last_redis_update is None or (time.time() - self.last_redis_update > self.REDIS_STALE_TIME):
            driver_states = self.redisClient.get_driver_state_model()
            self._ee_position, self._ee_orientation = T.mat2pose(driver_states[P.ROBOT_STATE_EE_POSE_KEY])
            self._ee_pose = np.hstack((self._ee_position, self._ee_orientation))
            self._ee_twist = T.calc_twist(driver_states[P.ROBOT_MODEL_JACOBIAN_KEY], driver_states[P.ROBOT_STATE_DQ_KEY])
            self._ee_v = self._ee_twist[:3]
            self._ee_w = self._ee_twist[3:]
            self._q = driver_states[P.ROBOT_STATE_Q_KEY]
            self._dq = driver_states[P.ROBOT_STATE_DQ_KEY]
            self._tau = driver_states[P.ROBOT_STATE_TAU_KEY]
            self._jacobian = driver_states[P.ROBOT_MODEL_JACOBIAN_KEY]
            self._linear_jacobian = self._jacobian[:3, :]
            self._angular_jacobian = self._jacobian[3:, :]
            self._mass_matrix = driver_states[P.ROBOT_MODEL_MASS_MATRIX_KEY]
            self._gravity = driver_states[P.ROBOT_MODEL_GRAVITY_KEY]
        else:
            pass

    @property
    def ee_position(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.
        """
        self._get_state_from_redis()
        return self._ee_position

    @property
    def ee_orientation(self):
        """list of four floats [qx, qy, qz, qw] of the orientation
        quaternion of the end-effector.
        """
        self._get_state_from_redis()
        return self._ee_orientation

    @property
    def ee_pose(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        self._get_state_from_redis()
        return self._ee_pose

    @property
    def ee_v(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        return self._ee_v

    @property
    def ee_w(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        return self._ee_w

    @property
    def ee_twist(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        return self._ee_twist

    @property
    def q(self):
        """List of 7f describing joint positions (rad) of the robot arm.

        Ordered from base to end_effector
        """
        self._get_state_from_redis()
        return self._q

    @property
    def dq(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        self._get_state_from_redis()
        return self._dq

    @property
    def tau(self):
        """List of 7f describing joint torques (Nm)

        Ordered from base to end_effector
        """
        return self._tau

    @property
    def jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        return self._jacobian

    @property
    def linear_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        return self._linear_jacobian

    @property
    def angular_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        return self._angular_jacobian

    @property
    def mass_matrix(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        return self._mass_matrix

    @property
    def gravity_vector(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.
        """
        self._get_state_from_redis()
        return self._gravity

    def open_gripper(self):
        """ Open robot gripper.
        """
        raise NotImplementedError

    def close_gripper(self):
        """ Close robot gripper
        """
        raise NotImplementedError
    def set_gripper_to_value(self, value):
        """Set gripper to desired value
        """
        if (value > 1.0 or value < 0):
            raise ValueError("Invalid gripper value must be fraction between 0 and 1")

        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_KEY, value*self.GRIPPER_MAX_VALUE)
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY, time.time())