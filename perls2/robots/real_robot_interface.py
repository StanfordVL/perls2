"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc   # For abstract class definitions
import six   # For abstract class definitions
import redis  # For communicating with RobotCtrlInterfaces

from perls2.robots.robot_interface import RobotInterface


class RealRobotInterface(RobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 pose=None,
                 controlType=None):

        self.config = config
        self.robot_cfg  = self.config[self.config['world']['robot']]

        # Sets environment connected flag for control interface
        self.redisClient.set('robot::env_connected', 'True')
        self.neutral_joint_angles = self.robot_cfg['neutral_joint_angles']
        self.RESET_TIMEOUT = 10       # Wait 3 seconds for reset to complete.

    def create(config, physics_id, arm_id):
        """Factory for creating robot interfaces based on config type
        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.real_sawyer_interface import RealSawyerInterface
            return RealSawyerInterface(
                config=config, physics_id=physics_id, arm_id=arm_id)
        else:
            raise ValueError("invalid robot interface type. choose 'sawyer'")


    def disconnect(self):
        self.redisClient.set('robot::env_connected', 'False')

    def reset(self):
        """ Reset arm to neutral configuration. Blocking call
        waits on redis flag to be cleared by the Control Interface

        TODO: This doesn't actually work, it just waits for the timeout.
        """
        logging.debug("Resetting robot")
        self.redisClient.set('robot::cmd_type', 'reset_to_neutral')
        start = time.time()
        while (self.redisClient.get('robot::reset_complete') != b'True' and
               (time.time() - start < self.RESET_TIMEOUT)):
            time.sleep(0.1)

        if (self.redisClient.get('robot::reset_complete') == b'True'):
            logging.debug("reset successful")
        else:
            logging.debug("reset failed")

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
        ee_pos_bstr = self.redisClient.get('robot::ee_position')
        return bstr_to_ndarray(ee_pos_bstr)

    @ee_position.setter
    def ee_position(self, position):
        self.redisClient.set('robot::desired_ee_position', str(position))

    @property
    def ee_orientation(self):
        """
        Get the orientation of the end-effector.
        :return: a list of floats for the orientation quaternion [qx,
        qy, qz, qw]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_orientation'))

    def goto_ee_pose(self, ee_pose):
        return self.ee_pose = ee_pose
    @property
    def ee_pose(self):
        """
        Get the pose of the end effector.
        :return: a list of floats for the position and orientation [x,
        y, z, qx, qy, qz, qw]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_pose'))

    @ee_pose.setter
    def ee_pose(self, pose):
        """
        Get the pose of the end effector.
        :return: a list of floats for the position and orientation [x,
        y, z, qx, qy, qz, qw]
        """
        self.redisClient.set('robot::cmd_type', 'ee_pose')
        self.redisClient.set('robot::desired_ee_pose', str(pose))

    @property
    def ee_v(self):
        """
        Get the current linear velocity of end effector in the eef
        frame.
        :return: a list of floats for the linear velocity m/s [vx, vy,
        vz]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_v'))

    @property
    def ee_omega(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return bstr_to_ndarray(self.redisClient.get('robot::ee_omega'))

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
        return bstr_to_ndarray(self.redisClient.get('robot::q'))

    @q.setter
    def q(self, qd):
        self.redisClient.set('robot::cmd_type', 'joint_position')
        self.redisClient.set('robot::qd', str(qd))

    @property
    def dq(self):
        """
        Get the joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        return bstr_to_ndarray(self.redisClient.get('robot::dq'))

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
        return bstr_to_ndarray(self.redisClient.get('robot::tau'))
