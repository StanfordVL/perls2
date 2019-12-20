""" Testing script for BulletSawyerInterface

Tests Bullet Sawyer Interface code in isolation from rest of PERLS library.
Methods tested:

    Getters:
        *_num_joints
    Setup:
        *Arena loading of robot from URDF at right joint config
    Properties:
        *q: getter/setter
        *ee_pose: getter/setter
    Controls:
        *set_neutral_positions
        *open_gripper
        *close_gripper
"""
import pybullet
from perls2.robots.bullet_sawyer_interface import BulletSawyerInterface
from perls2.utils.yaml_config import YamlConfig
import os
import numpy as np
import time
import pybullet_data


def step_sim(physics_id):
    ctrl_steps_per_action = 200

    for exec_steps in range(ctrl_steps_per_action):
        pybullet.stepSimulation(physics_id)


def step_till_close_enough(
        attribute,
        goal,
        max_steps,
        rtol=1,
        atol=1):
    global physics_id
    global bullet_sawyer

    steps = 0
    while(
        (not np.allclose(
            getattr(bullet_sawyer, attribute),
            goal,
            rtol,
            atol) and
         (steps <= max_steps))):

        pybullet.stepSimulation(physics_id)
        steps += 1
    print(np.allclose(
            getattr(bullet_sawyer, attribute),
            goal,
            rtol,
            atol))
    return steps


def step_till_close_enough_index(
        attribute,
        index,
        goal,
        max_steps=500,
        rtol=1e-02,
        atol=1e-02):
    global physics_id
    global bullet_sawyer

    steps = 0
    while(
        (not np.allclose(
            getattr(bullet_sawyer, attribute)[index],
            goal,
            rtol,
            atol) and
         (steps <= max_steps))):

        pybullet.stepSimulation(physics_id)
        steps += 1
    print("result " + attribute + " " + str(
            np.allclose(
                getattr(bullet_sawyer, attribute)[index],
                goal,
                rtol,
                atol)))
    return steps


# ###################SETUP################################
# TODO: Change this to its own file in the tester folder
# Create a pybullet simulation in isolation

physics_id = pybullet.connect(pybullet.GUI)
config = YamlConfig('./tester_config.yaml')
data_dir = config['data_dir']

# Load URDFs

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground plane
os.chdir('..')
plane_path = os.path.join(data_dir, config['ground']['path'])
plane_path = '../' + plane_path
plane_id = pybullet.loadURDF(
    fileName=plane_path,
    basePosition=config['ground']['pose'][0],
    baseOrientation=pybullet.getQuaternionFromEuler(
            config['ground']['pose'][1]),
    globalScaling=1.0,
    useFixedBase=config['ground']['is_static'],
    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    physicsClientId=physics_id)

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
time_step = config['sim_params']['time_step']  # TODO: remove magic number

pybullet.setGravity(0, 0, -10, physicsClientId=physics_id)
pybullet.setTimeStep(time_step, physicsClientId=physics_id)

# ## Begin testing code ###

# Create Bullet Sawyer Interface
bullet_sawyer = BulletSawyerInterface(
    physics_id=physics_id,
    arm_id=arm_id,
    config=config)

# Check initialization
# Check physics id

# Check arm id

# Check link id dictionary

# check config exists

# Check number of joints
if (bullet_sawyer._num_joints != 14):
    raise ValueError("_num_joints is incorrect value")
# Check ee_index

# Check that arena loaded arm correctly.
joint_positions = []
for joint_index in range(bullet_sawyer._num_joints):
    joint_angle, _, _, _ = pybullet.getJointState(
        arm_id,
        joint_index,
        physicsClientId=physics_id)
    joint_positions.append(joint_angle)

# Pybullet should load urdf with initial joint positions at 0
init_joint_positions = [0.0] * bullet_sawyer._num_joints

if (joint_positions != init_joint_positions):
    print("joint positions  " + str(joint_positions))
    print("desired: " + str(init_joint_positions))
    raise ValueError('BulletArena loaded robot incorrectly')
"""
Check joint positions at initialization.
These should be the same as the config file definition.
"""

# q getter tester
if (bullet_sawyer.q is None):
    # q is empty
    raise ValueError('BulletSawyerInterface.q returns None')
elif (len(bullet_sawyer.q) != bullet_sawyer._num_joints):
    # q is wrong length
    raise ValueError(
        'BulletSawyerInterface.q returns wrong size list')
if (bullet_sawyer.q != init_joint_positions):
    # q is incorrect value
    raise ValueError(
        'BulletSawyerInterface.q returns incorrect joint positions')

# Test reset to neutral
bullet_sawyer.set_joints_to_neutral_positions()
if (bullet_sawyer.q != bullet_sawyer.limb_neutral_positions):
    # reset to wrong neutral position / failed
    raise ValueError(
        "BulletSawyerInterface.set_joints_neutral_positions failed")

# q setter tester
bullet_sawyer.q = init_joint_positions

# step simulation forward
steps = 0
while ((np.allclose(
            bullet_sawyer.q,
            init_joint_positions,
            rtol=1e-02,
            atol=1e-02) is False) and
        (steps < 200)):
        pybullet.stepSimulation(physics_id)
        steps += 1

if (np.allclose(
            bullet_sawyer.q, init_joint_positions,
            rtol=1e-02,
            atol=1e-02) is False):
    raise ValueError("BulletSawyerInterface.q setter failed")
# TODO  test random joint positions
# TODO test invalid joint angles

# dq getter tester
# return 0 when static
if (bullet_sawyer.dq is None):
    raise ValueError('BulletSawyerInterface.dq returns None')
elif (len(bullet_sawyer.dq) != bullet_sawyer._num_joints):
    # q is wrong length
    raise ValueError('BulletSawyerInterface.dq returns wrong size list')
# dq setter test
# TODO need to tune gains to make this work.

# steps = 0
# bullet_sawyer.dq = [0.2] *bullet_sawyer._num_joints
# while((np.allclose(
#             bullet_sawyer.dq,
#             [0.2] *bullet_sawyer._num_joints,
#             rtol=1e-02,
#             atol=1e-02) == False) and
#         (steps < 500)):
#      pybullet.stepSimulation(physics_id)
#      steps+=1
# print(steps)

# if (np.allclose(
#     bullet_sawyer.dq[0:bullet_sawyer.ee_index],
#     [0.2]*bullet_sawyer.ee_index,
#     rtol=1e-02,
#     atol=1e-02) == False):
#     print(bullet_sawyer.dq[0:bullet_sawyer.ee_index])
#     raise ValueError("BulletSawyerInterface.dq setter failed")

# Set base joint to 90 degrees via position control
bullet_sawyer.set_joint_position_control(
    joint_ind=0,
    target_position=1.57,
    force=50,
    max_velocity=1.0)
step_till_close_enough_index(
    attribute='q',
    index=0,
    goal=1.57,
    max_steps=5000,
    rtol=1e-02,
    atol=1e-02)
# while ((np.allclose(
#             bullet_sawyer.q[0],
#             1.57,
#             rtol=1e-02,
#             atol=1e-02) == False) and
#         (steps < 5000)):
#         pybullet.stepSimulation(physics_id)
#         steps+=1
if (not(np.allclose(
                bullet_sawyer.q[0],
                1.57,
                rtol=1e-02,
                atol=1e-02))):
    print("bulet sawyer_base : " + str(bullet_sawyer.q[0]))
    raise ValueError("set joint position control failed")

bullet_sawyer.close_gripper()
input('did gripper close?')
step_sim(physics_id)
bullet_sawyer.open_gripper()
step_sim(physics_id)
input('did gripper open?')
bullet_sawyer.close_gripper()
step_sim(physics_id)
input('did gripper close?')
bullet_sawyer.open_gripper()
step_sim(physics_id)
input('did gripper open?')
print("############ All tests complete. ###########")
