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

class JVEnv(Env):
    """The class for Pybullet Sawyer Robot environments performing a reaching task.
    """

    def get_observation(self):
        robot_state = {'q': self.robot_interface.motor_joint_positions,
                      'dq': self.robot_interface.motor_joint_velocities}
        return robot_state
    
    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        self.robot_interface.action_set = True
        #self.robot_interface.set_dq(action)

    def rewardFunction(self):
        return None