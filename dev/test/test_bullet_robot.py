"""Test script for BulletRobots
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.envs.env import Env
#test_config = YamlConfig('dev/test/test_bullet_cfg.yaml')

env = Env(config='dev/test/test_bullet_cfg.yaml',
          use_visualizer=True,
          name='BulletTestEnv')
