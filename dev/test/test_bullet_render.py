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
          use_visualizer=False,
          name='BulletTestEnv')

# # Get RGBA, Depth, Segmask
# frames = env.camera_interface.frames()
# assert np.shape(frames['rgb']) == (224, 224, 3)
# assert np.shape(frames['rgba']) == (224, 224, 4)
# assert np.shape(frames['depth']) == (224, 224)
# # plt.imshow(frames['rgba'])
# # plt.show()

# frames_rgb = env.camera_interface.frames_rgb()
# assert np.shape(frames_rgb['rgb'] == (224, 224, 3))

# # # Move camera
# env.camera_interface.place([3.6, 0.0, 1.0])
# assert env.camera_interface.cameraEyePosition == [3.6, 0.0, 1.0]
# frames_rgb = env.camera_interface.frames_rgb()

env.camera_interface.place([0.6, 0.0, 1.0])

depth_frame = env.camera_interface.frames()['depth']
depth_clip = np.clip(depth_frame, 0.0, 2.0)
# plt.imshow(depth_clip,cmap='cool')
# plt.show()
depth_world = []
for row_i in range(224):
    for col_i in range(224):
        point_xyz = env.camera_interface.deproject(
            pixel=(row_i, col_i),
            depth=depth_clip[row_i, col_i])
        depth_world.append(point_xyz)

ax = plt.axes(projection='3d')
depth_world = np.asarray(depth_world)
ax.scatter3D(depth_world[:,0], depth_world[:,1], depth_world[:,2])

#plt.show()

# plt.imshow(frames_rgb['rgb'])
plt.show()

env.camera_interface.place([0.6, 0.4, 1.0])
new_frames = env.camera_interface.frames()
plt.imshow(new_frames['rgba'])
plt.show()

# plt.imshow(new_frames['depth'])
# plt.show()

depth_clip = np.clip(new_frames['depth'], 0.0, 2.0)
depth_world = []
for row_i in range(224):
    for col_i in range(224):
        point_xyz = env.camera_interface.deproject(
            pixel=(row_i, col_i),
            depth=depth_clip[row_i, col_i])
        depth_world.append(point_xyz)

ax = plt.axes(projection='3d')
depth_world = np.asarray(depth_world)
ax.scatter3D(depth_world[:,0], depth_world[:,1], depth_world[:,2])
plt.show()