"""Test script for BulletRobots gripper / pybullet control functions.


This script performs a visual test for Bullet gripper functions.
"""
# Change config to panda.
test_config = YamlConfig('dev/test/test_bullet_cfg.yaml')
test_config['world']['robot'] = 'panda'

env = Env(config=test_config, use_visualizer=True, name='BulletPandaTestEnv')
get_robot_states(env)

env.robot_interface.open_gripper()
for _ in range(10):
    env.world.step()
    input("check if gripper open")

env.robot_interface.set_gripper_to_value(0.2)
for _ in range(10):
    env.world.step()
    input("check if gripper closed")

env.robot_interface.set_gripper_to_value(0.8)
for _ in range(10):
    env.world.step()
    input("check if gripper closed")

env.robot_interface.close_gripper()
for _ in range(10):
    env.world.step()
    input("check if gripper closed")