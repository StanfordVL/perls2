""" Script for tuning gains on Bullet Sawyer Controller
"""
import pybullet
from perls2.robots.bullet_sawyer_interface import BulletSawyerInterface
from perls2.utils.yaml_config import YamlConfig
import os
import numpy as np
import time

import matplotlib.pyplot as plt
#################### SETUP ################################
# TODO: Change this to its own file in the tester folder
# Create a pybullet simulation in isolation
physics_id = pybullet.connect(pybullet.GUI)
dir_path = os.path.dirname(os.path.realpath(__file__))
yaml_dir = os.path.join(dir_path, 'tester_config.yaml')
config = YamlConfig(yaml_dir)
data_dir = config['data_dir']

# Load URDFs
import pybullet_data
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground plane

plane_path = os.path.join(data_dir, config['ground']['path'])
plane_id = pybullet.loadURDF(
    fileName=plane_path,
    basePosition=config['ground']['pose'][0],
    baseOrientation=pybullet.getQuaternionFromEuler(config['ground']['pose'][1]),
    globalScaling=1.0,
    useFixedBase=config['ground']['is_static'],
    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    physicsClientId=physics_id )

# Load arm
arm_id = pybullet.loadURDF(
    fileName=config['robot']['arm']['path'],
    basePosition=config['robot']['arm']['pose'],
    baseOrientation=pybullet.getQuaternionFromEuler(
                            config['robot']['arm']['orn']),
    globalScaling=1.0,
    useFixedBase=config['robot']['arm']['is_static'],
    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    physicsClientId=physics_id)

# Load Base
base_id = pybullet.loadURDF(
    fileName=config['robot']['base']['path'],
    basePosition=config['robot']['base']['pose'],
    baseOrientation=pybullet.getQuaternionFromEuler(
                            config['robot']['base']['orn']),
    globalScaling=1.0,
    useFixedBase=config['robot']['base']['is_static'],
    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    physicsClientId=physics_id)

# Set simulation parameters
time_step = config['sim_params']['time_step']

pybullet.setGravity(0,0,-10, physicsClientId=physics_id)
pybullet.setTimeStep(time_step, physicsClientId=physics_id)

# Create Bullet Sawyer Interface

bullet_sawyer = BulletSawyerInterface(
    physics_id=physics_id,
    arm_id=arm_id,
    config=config,
    controlType='osc')

bullet_sawyer.set_joints_to_neutral_positions()

##########################################################
# Set position control to see how much torque is applied
curr_ee_pose = bullet_sawyer.ee_pose
curr_ee_position = curr_ee_pose[:3]
curr_ee_orn = np.asarray(curr_ee_pose[3:])

# add displacement to current position
# keep orn the same
des_ee_position = np.asarray(curr_ee_pose[:3]) + np.asarray([0, 0, 0.1])
des_ee_pose = np.hstack((des_ee_position, curr_ee_orn))

max_steps = 5000
ee_pose_list = []
torque_list = []

steps=0
bullet_sawyer.ee_pose = des_ee_pose
while (steps < max_steps):
    bullet_sawyer.ee_pose = des_ee_pose
    pybullet.stepSimulation()
    ee_pose_list.append(bullet_sawyer.ee_pose)
    torque_list.append(bullet_sawyer.last_torques)
    steps+=1
pose_error = []
torque_joints = []
for index in range(6):
    pose_error.append([(step[index] - des_ee_pose[index]) for step in ee_pose_list])

for index in range(7):
    torque_joints.append([step_torque[index] for step_torque in torque_list])

for graph_num in range(6):

    plt.subplot(2,7,1+ graph_num)
    plt.xlabel('num_steps')
    plt.ylabel('position_error')
    plt.ylim(-0.12, 0.12)
    plt.plot(
        range(steps),
        pose_error[graph_num],'b')

for graph_num in range(7):
    plt.subplot(2,7,1+ graph_num + 7)
    plt.xlabel('num_steps')
    plt.ylabel('Torque (? Nm)')
    plt.ylim(-bullet_sawyer._joint_max_forces[graph_num]*1.1,
        bullet_sawyer._joint_max_forces[graph_num]*1.1)
    plt.plot(
        range(steps),
        torque_joints[graph_num],'g')
plt.show()
