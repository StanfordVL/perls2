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

    def reset(self):
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        #self.sensor_interface.reset()
        
        observation = self.get_observation()

    def get_observation(self):
        obs  = {}
        obs['ee_pose'] = self.robot_interface.ee_pose
        obs['q'] = self.robot_interface.q
        obs['dq'] = self.robot_interface.dq

        return obs
    
    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        if (self.robot_interface.controlType == 'EEImpedance' or 
           (self.robot_interface.controlType == 'EEPosture')):
            #self.robot_interface.move_ee_delta(action)
            self.robot_interface.set_ee_pose(action)
        elif self.robot_interface.controlType == 'JointVelocity':
            self.robot_interface.set_joint_velocity(action)
        elif self.robot_interface.controlType == 'JointImpedance':
            self.robot_interface.set_joint_delta(action)
            #self.robot_interface.set_joint_positions(action)
        elif self.robot_interface.controlType == 'JointTorque':
            self.robot_interface.set_joint_torque(action)
        self.robot_interface.action_set = True

    def rewardFunction(self):
        return None

