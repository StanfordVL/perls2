"""
Class defining the interface to the Panda Robot in Bullet
"""

import pybullet
import numpy as np
from perls2.robots.bullet_robot_interface import BulletRobotInterface


class BulletPandaInterface(BulletRobotInterface):
    """ Class for Panda Robot Interface in Pybullet. This class provides the
    functions for information about the state of the robot as well as sending
    commands.

    Attributes:
        physics_id (int): unique identifer for pybullet sim.
        arm_id (int) : unique identifier produced by pybullet to id robot.
        config (dict) : dictionary with configuration params for robot
        controlType (str): id for controlType ('osc', 'joint_space')
        limb_neutral_positiosn (list): list of joint angles for default

    """
    def __init__(self,
                 physics_id,
                 arm_id,
                 config,
                 controlType='EEImp'):

        self.data_dir = config['data_dir']

        super().__init__(physics_id, arm_id, config, controlType)

        #self._ee_index = self.get_link_id_from_name('panda_grasptarget')
        self._ee_index = self.get_link_id_from_name('panda_hand')
        # Neutral positions
        self.limb_neutral_positions = self.robot_cfg['neutral_joint_angles']

        self._name = "Franka Panda"
        self._default_force = 100
        self._default_position_gain = 0.1
        self._default_velocity_gain = 2.5

    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    def disconnect(self):
        pass

    @property
    def grasp_target_pos(self):
        """ Special getter for panda grasp target link position.

        Useful for grasping
        """
        return pybullet.getLinkState(self._arm_id, self.get_link_id_from_name('panda_grasptarget'))[0]

    def open_gripper(self):
        """Open the gripper of the robot
        """
        if 'gripper' in self.config:
            self.set_gripper_to_value(self.config['gripper']['open_value'])
        else:
            self.set_gripper_to_value(0.99)

    def close_gripper(self):
        """Open the gripper of the robot
        """
        if 'gripper' in self.config:
            self.set_gripper_to_value(self.config['gripper']['close_value'])
        else:
            self.set_gripper_to_value(0.1)

    def set_gripper_to_value(self, value):
        """Open/Close the gripper of the robot to fractional value.

        Args:
            value (float): gripper "open" fraction. 0.0 = Closed, 1.0 = Open.
        """

        # Get the joint limits for the right and left joint from config file
        l_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['l_finger_name']))

        r_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['r_finger_name']))

        # Get the range based on these joint limits
        l_finger_joint_range = (
            l_finger_joint_limits.get('upper') - l_finger_joint_limits.get('lower'))

        r_finger_joint_range = (
            r_finger_joint_limits.get('upper') - r_finger_joint_limits.get('lower'))

        # Determine the joint position by clipping to upper limit.
        # This part is different from the sawyer and bullet_robot_interface implementation.
        l_finger_position = (
            l_finger_joint_limits.get('lower') + value * l_finger_joint_range)

        r_finger_position = (
            r_finger_joint_limits.get('lower') + value * r_finger_joint_range)

        # Set the joint angles all at once
        l_finger_index = self.get_link_id_from_name(
            self.robot_cfg['l_finger_name'])
        r_finger_index = self.get_link_id_from_name(
            self.robot_cfg['r_finger_name'])


        gripper_des_q = [l_finger_position, r_finger_position]
        gripper_indices = [l_finger_index, r_finger_index]

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=gripper_indices,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=gripper_des_q,
            physicsClientId=self._physics_id)
