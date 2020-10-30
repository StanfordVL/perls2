"""Test script for BulletArenas.

Tests:
    Cameras:
        -Loading camera intrinsics, extrinsics params from configs.
        -Randomizing camera parameters within bounds set in config.
    Objects:
        -Loading objects from configs.
        -Randomizing object placement within bounds set in config.
        -Waiting for object stability after placement.
"""
from perls2.envs.env import Env
import pybullet as pb
import numpy as np
import gym.spaces as spaces
import pytest

env = Env(config='dev/test/test_bullet_cfg.yaml',
          use_visualizer=True,
          name='BulletTestEnv')

# Check env properties (to make sure config hasn't been changed. )
# Check world is loaded and is simulation
assert env.world.is_sim
# Check pybullet GUI connected.
assert pb.getConnectionInfo(env._physics_id) ==  {'connectionMethod': 1, 'isConnected': 1}

arena = env.arena
# Check world has camera.
if 'sensor' in env.config.keys():
    assert env.arena.has_camera
    # Check camera extrinsics match config
    assert len(arena.camera_eye_pos) == 3
    assert arena.camera_eye_pos == env.config['sensor']['camera']['extrinsics']['eye_position']
    assert arena.camera_target_pos == env.config['sensor']['camera']['extrinsics']['target_position']
    assert arena.camera_up_vector == env.config['sensor']['camera']['extrinsics']['up_vector']
    # Check camera intrinsics match config
    assert arena.fov = env.config['sensor']['camera']['intrinsics']['fov']
    assert arena.near_plane = env.config['sensor']['camera']['intrinsics']['near_plane']
    assert arena.far_plane = env.config['sensor']['camera']['intrinsics']['far_plane']
    # Check image dimensions match config
    assert arena.image_height = env.config['sensor']['camera']['image']['height']
    assert arena.image_width = env.config['sensor']['camera']['image']['width']
    # Check arena attributes match BulletCameraInterface attributes.
