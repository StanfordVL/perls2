""" Script for tuning gains on Bullet Sawyer Controller
"""
import pybullet
from perls2.robots.bullet_sawyer_interface import BulletSawyerInterface
from perls2.utils.yaml_config import YamlConfig
import os
import numpy as np
import time


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
pybullet.setJointMotorControlArray(arm_id,
                                 range(9),
                                 pybullet.VELOCITY_CONTROL,
                                 forces=[0.0]*9)
pybullet.setGravity(0,0,-10, physicsClientId=physics_id)
pybullet.setTimeStep(time_step, physicsClientId=physics_id)

# Create Bullet Sawyer Interface
bullet_sawyer = BulletSawyerInterface(
    physics_id=physics_id,
    arm_id=arm_id,
    config=config, controlType='osc')
max_steps=5000
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
        #print(getattr(bullet_sawyer, attribute))

        tuning_data.append(getattr(bullet_sawyer, attribute))
        tuning_torques.append(joint_torques)
    if (np.allclose(
            getattr(bullet_sawyer,attribute),
            goal,
            rtol,
            atol)):
        return steps
    else:
        return -1
##### Controllers Test ##############
bullet_sawyer.set_joints_to_neutral_positions()
print(np.shape(bullet_sawyer.jacobian))
print("reset")
# #target_joint_positions = [0] * 14
print(pybullet.getMatrixFromQuaternion([0, 0, 0, 1]))
target_joint_positions = (np.asarray([0,
  -1.18,
  0.00,
  2.18,
  0.00,
  0.57,
  3.3161, 0, 0,
]) +
 np.asarray([-0.1]*9))
#bullet_sawyer.set_joints_uncompensated(
#    target_joint_positions)
tuning_data = []
tuning_torques = []
# Add a disturbance to current position
xd_pos = (np.array(bullet_sawyer.ee_pose[:3]) +
    np.array([0.0, 0.10, 0.00]))

# Maintain initial orientation
xd =  np.hstack(
    (xd_pos,
     np.asarray(bullet_sawyer.ee_orientation)))


print(bullet_sawyer.N_q)

import matplotlib.pyplot as plt

steps=0
pose_step = []
kpkv_forces_list = []
mass_kpkv_list = []
osc_torques_list = []
comp_torques_list = []
grav_list = []


max_steps = 2000
input ("start recording?")
while (steps <max_steps):
    # if(np.allclose(xd[0:3], bullet_sawyer.ee_pose[:3], 1e-2, 1e-2)):
    #     print("goal acheived")
    #     print("steps " +str(steps))
    #     break
    kpkv_forces, mass_kpkv_forces, osc_torques, comp_torques = bullet_sawyer.compute_osc_compensated(xd)
    set_torques = bullet_sawyer.set_torques(comp_torques)
    pybullet.stepSimulation()

    # Save all the values at each step
    grav_list.append(bullet_sawyer.N_q)
    orn_eul = np.asarray(pybullet.getEulerFromQuaternion(bullet_sawyer.ee_pose[3:]))
    pose = np.hstack((np.asarray(bullet_sawyer.ee_pose[:3]),orn_eul))
    pose_step.append(pose)

    kpkv_forces_list.append(kpkv_forces)
    mass_kpkv_list.append(mass_kpkv_forces)
    osc_torques_list.append(osc_torques)
    comp_torques_list.append(comp_torques)

    # kill if it goes crazy
    # keep_going = input("step sim with ENTER, kill with x")
    # if keep_going == 'x':
    #     break
    steps+=1

############# DATA REORGANIZING #########################################
# Data is in format for each joint/dof at a time step [q1, q2, q3 ...]
# we want the data for each joint/dof over time. q1[t=0, t=1, t=...]
pose_error =[]
kpkv_forces_all = []
mass_kpkv_forces_all = []
osc_torques_all = []
comp_torques_all =[]
grav_all = []
for index in range(6):
    pose_error.append([(step[index] - xd[index]) for step in pose_step])
    kpkv_forces_all.append([step[index] for step in kpkv_forces_list])
    mass_kpkv_forces_all.append([step[index] for step in mass_kpkv_list])
for index in range(7):
    grav_all.append([step[index] for step in grav_list])
    osc_torques_all.append([step[index] for step in osc_torques_list])
    comp_torques_all.append([step_torque[index] for step_torque in comp_torques_list])

###################### PLOTTING ##########################################
# Position error
pose_error_titles = ['X error', 'Y Error', 'Z Error', 'Alpha Error', 'Beta Error', 'Gamma Error']
kpkv_titles = ['(kpkv force x)','(kpkv force y)', '(kpkv force z)', '(kpkv torque alpha)', '(kpkv torque beta)', '(kpkv torque gamma)']
mass_titles = [['mass*' + title] for title in kpkv_titles]
osc_titles = [['osc_torques q' + str(num)] for num in range(7)]
comp_titles = [['grav_comp_torques q' + str(num)] for num in range(7)]
for graph_num in range(6):
    plt.subplot(5,7,1+ graph_num)
    plt.xlabel('num_steps')
    if (graph_num < 3):
        plt.ylabel('position_error (m)')
    else:
        plt.ylabel('orientation error (rad)')

    plt.title(pose_error_titles[graph_num])
    plt.plot(
        range(steps),
        pose_error[graph_num],'b')
# kpkv forces
for graph_num in range(6):
    plt.subplot(5,7,1 + graph_num + 7)
    plt.xlabel('num_steps')
    if (graph_num < 3):
        plt.ylabel('Force (N)')

    else:
        plt.ylabel('Torque (Nm)')

    plt.title(kpkv_titles[graph_num])
    plt.plot(
        range(steps),
        kpkv_forces_all[graph_num],'b')
# Mass matrix * kpkv forces
for graph_num in range(6):
    plt.subplot(5,7,1 + graph_num + 14)
    plt.xlabel('num_steps')
    if (graph_num < 3):
        plt.ylabel('Force (N)')

    else:
        plt.ylabel('Torque (Nm)')

    plt.title(mass_titles[graph_num])
    plt.plot(
        range(steps),
        mass_kpkv_forces_all[graph_num],'b')

for graph_num in range(7):
    plt.subplot(5,7,1+ graph_num + 21)
    plt.xlabel('num_steps')
    plt.ylabel('Torque (Nm)')
    plt.title(osc_titles[graph_num])
    # plt.ylim(-bullet_sawyer._joint_max_forces[graph_num]*1.1,
    #   bullet_sawyer._joint_max_forces[graph_num]*1.1)
    plt.plot(
        range(steps),
        osc_torques_all[graph_num],'g')

for graph_num in range(7):
    plt.subplot(5,7,1+ graph_num + 28)
    plt.xlabel('num_steps')
    plt.ylabel('Torque (Nm)')
    plt.title(comp_titles[graph_num])
    # plt.ylim(-bullet_sawyer._joint_max_forces[graph_num]*1.1,
    #   bullet_sawyer._joint_max_forces[graph_num]*1.1)
    plt.plot(
        range(steps),
        comp_torques_all[graph_num],'g')

plt.show()
print(np.shape(bullet_sawyer.JBar))

home_pos = bullet_sawyer.ee_pose[:3]
xd_orn = bullet_sawyer.ee_orientation[3:]

