""" Class for Pybullet Sawyer environments performing a reach task.
"""
from __future__ import division

import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig
from perls2.envs.env import Env

import gym.spaces as spaces

class DemoControlEnv(Env):
    """The class for Pybullet Sawyer Robot environments performing a reaching task.
    """

    def get_observation(self):
        robot_state = {'q': self.robot_interface.motor_joint_positions,
                      'dq': self.robot_interface.motor_joint_velocities}
        return robot_state
    
    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        if self.robot_interface.controlType == 'EEImpedance':
            self.robot_interface.move_ee_delta(action)
        elif self.robot_interface.controlType == 'JointVelocity':
            self.robot_interface.set_dq(action)
        elif self.robot_interface.controlType == 'JointImpedance':
            self.robot_interface.set_joint_delta(action)
        elif self.robot_interface.controlType == 'JointTorque':
            self.robot_interface.set_joint_torque(action)
        self.robot_interface.action_set = True

    def rewardFunction(self):
        return None