""" Script for tuning gains on Bullet Sawyer Controller
"""
import pybullet
from perls2.robots.bullet_sawyer_sai2_interface import BulletSawyerSai2Interface
from perls2.utils.yaml_config import YamlConfig
import os
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import sai2python as sai2
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
bullet_sawyer = BulletSawyerSai2Interface(
    physics_id=physics_id,
    arm_id=arm_id,
    config=config, controlType='osc')

bullet_sawyer.set_joints_to_neutral_positions()
# Set desired position small distance from current
xd_pos = (np.array(bullet_sawyer.ee_pose[:3]) +
    np.array([0.0, -0.1, 0.00]))

xd_orn = R.from_quat(np.asarray(bullet_sawyer.ee_orientation))
# Maintain initial orientation
xd =  np.hstack(
    (xd_pos,
     bullet_sawyer.ee_orientation))

print("mass matrix")
print(bullet_sawyer.mass_matrix)
print("lambda x")
print(bullet_sawyer.op_space_mass_matrix[0])
print("lambda r")
print(bullet_sawyer.op_space_mass_matrix[1])
print("lambda")
print(bullet_sawyer.op_space_mass_matrix[2])

import matplotlib.pyplot as plt

steps=0
pose_step = []
orn_list = []
orn_aa_list = []
kpkv_forces_list = []
mass_kpkv_list = []
osc_torques_list = []
comp_torques_list = []
grav_list = []

posture = np.asarray(bullet_sawyer.motor_joint_positions[:7])
max_steps = 100
input ("start recording?")
while (steps <max_steps):

    orn_error, error, kpkv_forces, mass_kpkv_forces, osc_torques, comp_torques = (
        bullet_sawyer.controller.compute_torques_compensated(
            bullet_sawyer.state_dict, posture, xd))
    set_torques = bullet_sawyer.set_torques(comp_torques)
    pybullet.stepSimulation()

    # Save all the values at each step
    grav_list.append(bullet_sawyer.N_q)
    pose_step.append(error)
    orn_list.append(orn_error[0])
    orn_aa_list.append(orn_error[1])
    kpkv_forces_list.append(kpkv_forces)
    mass_kpkv_list.append(mass_kpkv_forces)
    osc_torques_list.append(osc_torques)
    comp_torques_list.append(comp_torques)

    steps+=1

############# DATA REORGANIZING #########################################
# Data is in format for each joint/dof at a time step [q1, q2, q3 ...]
# we want the data for each joint/dof over time. q1[t=0, t=1, t=...]
pose_error =[]
orn_error_all = []
orn_error_aa_all = []
kpkv_forces_all = []
mass_kpkv_forces_all = []
osc_torques_all = []
comp_torques_all =[]
grav_all = []
orn_error_all = np.transpose(orn_list)
orn_error_aa_all = np.transpose(orn_aa_list)
# for index in range(3):
#     orn_error_all[index].append([step[index] for step in orn_list])
#     orn_error_aa_all[index].append([step[index] for step in orn_aa_list])
for index in range(6):
    pose_error.append([step[index] for step in pose_step])
    kpkv_forces_all.append([step[index] for step in kpkv_forces_list])
    mass_kpkv_forces_all.append([step[index] for step in mass_kpkv_list])
for index in range(7):
    grav_all.append([step[index] for step in grav_list])
    osc_torques_all.append([step[index] for step in osc_torques_list])
    comp_torques_all.append([step_torque[index] for step_torque in comp_torques_list])

# # PLOT ORIENTATION ERROR
# for graph_num in range(3):
#     plt.subplot(2,3,1+ graph_num)
#     plt.xlabel('num_steps')

#     plt.ylabel('orientation error (rad)')

#     plt.title("orn error cross product")
#     plt.plot(
#         range(steps),
#         orn_error_all[graph_num],'b')
# for graph_num in range(3):
#     plt.subplot(2,3,4+ graph_num)
#     plt.xlabel('num_steps')

#     plt.ylabel('orientation error (rad)')

#     plt.title("orn error axis angle")
#     plt.plot(
#         range(steps),
#         orn_error_aa_all[graph_num],'b')

# ###################### PLOTTING ##########################################
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