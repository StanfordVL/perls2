""" Class for Pybullet Sawyer environments performing a reach task.
"""
from __future__ import division

import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig
from examples.simple_reach.simple_reach_env import SimpleReachEnv

import gym.spaces as spaces

class SimpleReachEnvTq(SimpleReachEnv):
    """The class for Pybullet Sawyer Robot environments performing a reaching task.
    """

    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        action = np.hstack((action, [0, 0, 0]))
        self.robot_interface.move_ee_delta(action)