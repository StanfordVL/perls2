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

class PBControlEnv(Env):
    """The class for Pybullet Sawyer Robot environments performing a reaching task.
    """
    def get_observation(self):
        pass

    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        self.robot_interface.move_ee_delta(action)

    def rewardFunction(self):
        return None

if __name__ == '__main__':
    env = PBControlEnv('dev/tester/pb_control/pb_control.yaml', True, 'PB Control Env')
    for step in range(5):
        initial_position = env.robot_interface.ee_position
        
        env.step([0, 0, 0.1, 0, 0, 0])
        final_position = env.robot_interface.ee_position
        
        delta = np.subtract(final_position, initial_position)

        print("*** " + str(delta) + " ***")

