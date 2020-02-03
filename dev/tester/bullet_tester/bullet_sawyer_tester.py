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
<<<<<<< HEAD
ctrl_steps_per_action = 1

=======
import pybullet_data
>>>>>>> cfg_dir

def step_till_close_enough_fn(
        attribute,
        exec_fn,
        goal,
        max_steps,
        rtol=1,
        atol=1):
    """
    Step the simulation until attribute reaches goal while executing a fn

    Args:
        attribute (string): string of the robot to monitor
        exec_fn (string): string of the function of the robot to execute at each
            step
        goal (array): goal for the attribute to reach.
        max_steps (int): max steps before returning an error
        rtol (float): relative tolerance for attribute to reach goal
        atol (float): absolute tolerance for attribute to reach goal

    Returns:
        -1 if failed or
        num steps (int) if complete.
    """


    global physics_id
    global bullet_sawyer
    global ctrl_steps_per_action

    global tuning_file # For logging
    steps = 0
    # while the attribute has not reached the bounds for the goal
    # and the max steps have not been exceeded, execute action
    # and step sim forward.
    while(
        (not np.allclose(
            getattr(bullet_sawyer,attribute), goal, rtol, atol)
        and (steps < max_steps))):

        joint_torques = getattr(bullet_sawyer,exec_fn)(goal)

        bullet_sawyer.set_torques(joint_torques)
        pybullet.stepSimulation(physics_id)
        steps+=1
        # Log the state
        print(getattr(bullet_sawyer, attribute))
        for joint in range(9):
            tuning_data[joint].append(getattr(bullet_sawyer, attribute)[joint])
        tuning_torques.append(joint_torques)
    if (np.allclose(
            getattr(bullet_sawyer,attribute),
            goal,
            rtol,
            atol)):
        return steps
    else:
        return -1

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
            getattr(bullet_sawyer,attribute),
            goal,
            rtol,
            atol) and
        (steps < max_steps))):

        pybullet.stepSimulation(physics_id)
        steps+=1
    if (np.allclose(
            getattr(bullet_sawyer,attribute),
            goal,
            rtol,
            atol)):
        return steps
    else:
        return -1

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
        steps+=1
    print("result " + attribute + " "  + str(
            np.allclose(
                getattr(bullet_sawyer, attribute)[index],
                goal,
                rtol,
                atol)))
    return steps

####################SETUP################################
# TODO: Change this to its own file in the tester folder
# Create a pybullet simulation in isolation
physics_id = pybullet.connect(pybullet.DIRECT)
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
    raise ValueError("BulletSawyerInterface.set_joints_neutral_positions failed")
pybullet.stepSimulation()

# q setter tester
bullet_sawyer.q = init_joint_positions

# step simulation forward
steps = 0
while ((np.allclose(
            bullet_sawyer.q,
            init_joint_positions,
            rtol=1e-02,
            atol=1e-02) == False) and
        (steps < 200)):
        pybullet.stepSimulation(physics_id)
        steps+=1
reached_goal = np.allclose(
            bullet_sawyer.q,init_joint_positions,
            rtol=1e-02,
            atol=1e-02)
if (reached_goal== False):
    print(bullet_sawyer.q)
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
    print("bullet sawyer_base : " + str(bullet_sawyer.q[0]))
    raise ValueError("set joint position control failed")

bullet_sawyer.close_gripper()
input('did gripper close?')
for step in range(500):
    pybullet.stepSimulation()

bullet_sawyer.open_gripper()
for step in range(500):
    pybullet.stepSimulation()
# step_sim(physics_id)
# input('did gripper open?')
# bullet_sawyer.close_gripper()
# step_sim(physics_id)
# input('did gripper close?')
# bullet_sawyer.open_gripper()
# step_sim(physics_id)
# input('did gripper open?')

print("############ All tests complete. ###########")
