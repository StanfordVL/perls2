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

class MassMatrixEnv(Env):
    """The class for Pybullet Sawyer Robot environments performing a reaching task.
    """    
    def get_observation(self):
        return None
    
    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        pass

    def rewardFunction(self):
        return None

if __name__ == '__main__':
    env = MassMatrixEnv('dev/demo_control/mass_matrix.yaml', True, 'MassMatrixEnv')
    env.reset()
    mass_matrix = env.robot_interface.mass_matrix
    joint_q = env.robot_interface.q 

    print("Joint positions: ")
    print(joint_q)

    print("Mass matrix: " )
    print(mass_matrix)
    input("Continue?")