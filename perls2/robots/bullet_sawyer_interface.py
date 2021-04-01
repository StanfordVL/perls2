"""
Class defining the interface to the Sawyer Robots in PyBullet.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import pybullet
from perls2.robots.bullet_robot_interface import BulletRobotInterface
import logging


class BulletSawyerInterface(BulletRobotInterface):
    """ Class for Sawyer Robot Interface in Pybullet. This class provides the
    functionsfor information about the state of the robot as well as sending
    commands.

    """

    def __init__(self,
                 physics_id,
                 arm_id,
                 config,
                 controlType='JointVelocity'):
        """
        Initialize variables

        Parameters
        ----------
            pose: list of seven floats (optional)
            Pose of robot base in world frame
            x y z qx qy qz qw
        """
        self.data_dir = config['data_dir']

        super().__init__(physics_id, arm_id, config, controlType)
        self._ee_index = self.get_link_id_from_name('right_hand')

        # Neutral positions and configuration specifics
        self.limb_neutral_positions = self.config['sawyer']['neutral_joint_angles']
        self._name = "Rethink Bullet Sawyer"
        logging.debug("BulletSawyerInterface created")
        self._default_force = 100
        self._default_position_gain = 0.1
        self._default_velocity_gain = 2.5

    def start(self):
        """Start the robot
        """
        raise NotImplementedError

    @property
    def ee_index(self):
        return self.get_link_id_from_name("right_hand")

    @property
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        return {'robot': '5.0.4',
                'gripper': 'rethink_ee',
                'robotSDK': '5.0.4'}

    @property
    def q(self):
        """
        Get the joint configuration of the robot arm.
        Args:
            None
        Returns:
            list of joint positions in radians ordered by
            from small to large.
        """
        return super().q

    @q.setter
    def q(self, qd):
        """
        Get the joint configuration of the robot arm.
        Args:
            qd: list
                list of desired joint position
        """
        pybullet.setJointMotorControlArray(
            bodyIndex=self._arm_id,
            jointIndices=range(0, self._num_joints),
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=qd,
            forces=[250] * self._num_joints,
            positionGains=[0.1] * self._num_joints,
            velocityGains=[1.2] * self._num_joints)

    def disconnect(self):
        pass