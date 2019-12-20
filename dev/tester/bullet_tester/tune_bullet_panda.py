""" Script for tuning gains on Bullet Sawyer Controller
"""
import pybullet
from perls2.robots.bullet_panda_interface import BulletPandaInterface
from perls2.utils.yaml_config import YamlConfig
import os
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
#################### SETUP ################################
# TODO: Change this to its own file in the tester folder
# Create a pybullet simulation in isolation
physics_id = pybullet.connect(pybullet.GUI)
dir_path = os.path.dirname(os.path.realpath(__file__))
yaml_dir = os.path.join(dir_path, 'panda_tester_config.yaml')
config = YamlConfig(yaml_dir)
data_dir = config['data_dir']
dof = 7

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


# Set simulation parameters
time_step = config['sim_params']['time_step']
pybullet.setJointMotorControlArray(arm_id,
                                 range(dof),
                                 pybullet.VELOCITY_CONTROL,
                                 forces=[0.0]*dof)
pybullet.setGravity(0,0,-10, physicsClientId=physics_id)
pybullet.setTimeStep(time_step, physicsClientId=physics_id)

# Create Bullet Sawyer Interface
bullet_panda = BulletPandaInterface(
    physics_id=physics_id,
    arm_id=arm_id,
    config=config, controlType='osc')

##### Controllers Test ##############
bullet_panda.set_joints_to_neutral_positions()
print(bullet_panda.motor_joint_positions)
pybullet.stepSimulation()
print(bullet_panda.jacobian)
print(bullet_panda.mass_matrix)

f = open("bullet_jacobian.txt", "w")
for row in bullet_panda.jacobian:
    for elem in row:
        elem_to_write = elem
        if np.abs(elem) < .0001:
            elem_to_write = 0

        f.write(str(elem_to_write))
        f.write(",  ")
    f.write("\n")

f = open("bullet_mass_matrix.txt", "w")
for row in bullet_panda.mass_matrix:
    for elem in row:
        elem_to_write = elem
        if np.abs(elem) < .0001:
            elem_to_write = 0

        f.write(str(elem_to_write))
        f.write(",  ")
    f.write("\n")
