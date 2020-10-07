"""Test script for BulletWorlds.

Tests:
    BulletCameraInterface:
        - Get rgb, depth and segmask frames.


    -Loading objects from function calls.
    -
"""
from perls2.envs.env import Env
import perls2.controllers.utils.transform_utils as T
import pybullet as pb
import numpy as np
import gym.spaces as spaces
import pytest
import matplotlib.pyplot as plt
env = Env(config='dev/test/test_bullet_cfg.yaml',
          use_visualizer=True,
          name='BulletTestEnv')

# Get RGBA, Depth, Segmask
frames = env.camera_interface.frames()
assert np.shape(frames['rgb']) == (224, 224, 3)
assert np.shape(frames['rgba']) == (224, 224, 4)
assert np.shape(frames['depth']) == (224, 224)
# plt.imshow(frames['rgba'])
# plt.show()

frames_rgb = env.camera_interface.frames_rgb()
assert np.shape(frames_rgb['rgb'] == (224, 224, 3))

# Move camera
env.camera_interface.place([3.6, 0.0, 1.0])
assert env.camera_interface.cameraEyePosition == [3.6, 0.0, 1.0]
frames_rgb = env.camera_interface.frames_rgb()
# plt.imshow(frames_rgb['rgb'])
# plt.show()

#