"""
Class defining the interface to the Panda Robot in Bullet
"""

import pybullet
import numpy as np
import rbdl 
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
                 config=None,
                 controlType='EEImp'):

        self.data_dir = config['data_dir']
        import os
        model_path = os.path.join(self.data_dir, 'robot/franka_panda/panda.urdf')
        self.rbdl_model = rbdl.loadModel(bytes(model_path, 'utf-8'))
        super().__init__(physics_id, arm_id, config, controlType)

        self._ee_index = self.get_link_id_from_name('panda_link7')

        # Neutral positions
        self.limb_neutral_positions =  self.robot_cfg['neutral_joint_angles']

        self._name = "Franka Panda"
        self._default_force = 100
        self._default_position_gain = 0.1
        self._default_velocity_gain = 2.5
        #self.num_joints = 9

    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    def disconnect(self):
        pass

    def set_gripper_to_value(self, value):
        """Close the gripper of the robot
        """
        # Get the joint limits for the right and left joint from config file
        l_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['l_finger_name']))

        r_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['r_finger_name']))

        # Get the range based on these joint limits
        l_finger_joint_range = (
            l_finger_joint_limits.get('upper') -
            l_finger_joint_limits.get('lower'))

        r_finger_joint_range = (
            r_finger_joint_limits.get('upper') -
            r_finger_joint_limits.get('lower'))

        # Determine the joint position by clipping to upper limit.
        l_finger_position = (
            l_finger_joint_limits.get('lower') +
            value * l_finger_joint_range)

        r_finger_position = (
            r_finger_joint_limits.get('lower') +
            value * r_finger_joint_range)

        # Set the joint angles all at once
        l_finger_index = self.get_link_id_from_name(
            self.robot_cfg['l_finger_name'])
        r_finger_index = self.get_link_id_from_name(
            self.robot_cfg['r_finger_name'])

        gripper_q = self.q
        gripper_q[l_finger_index] = l_finger_position
        gripper_q[r_finger_index] = r_finger_position
        
        gripper_des_q = [gripper_q[l_finger_index], 
                          gripper_q[r_finger_index]]
        gripper_indices = [l_finger_index, r_finger_index]

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,

            jointIndices=gripper_indices,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=gripper_des_q)
