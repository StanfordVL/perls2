"""Test script for BulletWorlds.

Tests:
    BulletWorld:
        - Loading objects from config
        - Loading objects from explicit paths.
        - Removing object from simulations.
    BulletObjectInterface:
        - Object state properties are accurate and correct dim.
        - Placing object works.


    -Loading objects from function calls.
    -
"""
from perls2.envs.env import Env
import perls2.controllers.utils.transform_utils as T
import pybullet as pb
import numpy as np
import gym.spaces as spaces
import pytest

env = Env(config='dev/test/test_bullet_cfg.yaml',
          use_visualizer=True,
          name='BulletTestEnv')

# Check Env properties

# Test World properties
# Check is_sim flag
assert env.world.is_sim
# Check observation space type
assert isinstance(env.observation_space, spaces.Box)
# Check action space type
assert isinstance(env.action_space, spaces.Box)
# Check use visualizer flag set.
assert env.world.use_visualizer

# Pybullet specific checks
# Check pybullet connection.
assert isinstance(env._physics_id, int)
assert pb.getConnectionInfo(env._physics_id) == {'connectionMethod': 1, 'isConnected': 1}

# Check object loading
assert env.has_objects
assert isinstance(env.object_interfaces, dict)
assert '013_apple' in env.object_interfaces

# BulletObjectInterface Tests
# Attributes
assert isinstance(env.object_interfaces['013_apple'].obj_id, int)
assert env.object_interfaces['013_apple'].physics_id == env._physics_id

# Object States
assert len(env.object_interfaces['013_apple'].pose) == 7
assert len(env.object_interfaces['013_apple'].position) == 3
assert len(env.object_interfaces['013_apple'].orientation) == 4
assert len(env.object_interfaces['013_apple'].linear_velocity == 3)
assert len(env.object_interfaces['013_apple'].angular_velocity == 3)

# BulletObjectInterface.place()
env.object_interfaces['013_apple'].place(
    new_object_pos=[2.0, 2.0, 2.0],
    new_object_orn=[0, 0, 0, 1])
assert env.object_interfaces['013_apple'].pose == [2.0, 2.0, 2.0, 0, 0, 0, 1]
env.object_interfaces['013_apple'].place(
    new_object_pos=[2.1, 2.1, 2.1],
    new_object_orn=None)
assert env.object_interfaces['013_apple'].pose == [2.1, 2.1, 2.1, 0, 0, 0, 1]
new_quat = T.random_quat().tolist()
new_pose = [2.1, 2.1, 2.1] + new_quat
env.object_interfaces['013_apple'].place(
    new_object_pos=None,
    new_object_orn=new_quat)

assert env.object_interfaces['013_apple'].position.tolist() == new_pose[:3]
assert np.isclose(np.abs(env.object_interfaces['013_apple'].orientation),
                  np.abs(new_pose[3:])).all()

# BulletWorld Tests
# Loading Objects
env.world.add_object(
    path='objects/ycb/013_apple/google_16k/textured.urdf',
    name='apple2',
    pose=[3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 1.0],
    scale=1.0,
    is_static=False)

assert 'apple2' in env.object_interfaces
assert np.isclose(env.object_interfaces['apple2'].position, [3.0, 3.0, 3.0], rtol=5e-02).all()
assert np.isclose(np.abs(env.object_interfaces['apple2'].orientation),
                  np.abs([0.0, 0.0, 0.0, 1.0]), rtol=1e-03).all()

# Step world forward to make sure object falls.
env.world.step()
assert env.world.object_interfaces['apple2'].position[2] < 3.0
# Removing objects
pytest.raises(KeyError, env.world.remove_object, 'apple0')
env.world.remove_object('apple2')
assert 'apple2' not in env.object_interfaces
