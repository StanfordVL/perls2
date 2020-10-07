"""Test script for BulletRobots

This script tests the types and dimensions of the BulletRobotInterface
states and methods. For testing controllers see test_controllers.py
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.envs.env import Env
import numpy as np


def get_robot_states(env):
    assert len(env.robot_interface.q) == 7
    assert len(env.robot_interface.dq) == 7
    assert len(env.robot_interface.tau) == 7
    assert len(env.robot_interface.ee_position) == 3
    assert len(env.robot_interface.ee_orientation) == 4
    assert len(env.robot_interface.ee_pose) == 7
    assert len(env.robot_interface.ee_pose_euler) == 6
    assert len(env.robot_interface.ee_v) == 3
    assert len(env.robot_interface.ee_w) == 3
    assert len(env.robot_interface.ee_twist) == 6
    assert np.shape(env.robot_interface.rotation_matrix) == (9, )
    assert np.shape(env.robot_interface.jacobian) == (6, 7)
    assert np.shape(env.robot_interface.linear_jacobian) == (3, 7)
    assert np.shape(env.robot_interface.angular_jacobian) == (3, 7)
    assert np.shape(env.robot_interface.mass_matrix) == (7, 7)
    assert len(env.robot_interface.N_q) == 7


env = Env(config='dev/test/test_bullet_cfg.yaml',
          use_visualizer=False,
          name='BulletSawyerTestEnv')

get_robot_states(env)

# Change config to panda.
test_config = YamlConfig('dev/test/test_bullet_cfg.yaml')
test_config['world']['robot'] = 'panda'

env = Env(config=test_config, use_visualizer=False, name='BulletPandaTestEnv')
get_robot_states(env)
