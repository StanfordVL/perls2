"""Script to save demo files to text. 
"""
import numpy as np


# Joint position delta demo. 
joint_position_delta_demo = []
NUM_JOINTS = 7

# Move each joint individually by +0.05 radians
for joint_num in range(NUM_JOINTS):
    delta = np.zeros(NUM_JOINTS)
    delta[joint_num] = 0.05
    joint_position_delta_demo.append(delta)

# Move each joint individually by -0.05 radians
for joint_num in range(NUM_JOINTS):
    delta = np.zeros(NUM_JOINTS)
    delta[-1 - joint_num] = -0.05
    joint_position_delta_demo.append(delta)
np.savez('dev/demo_control/joint_position_delta_demo.npz', action_list=joint_position_delta_demo, allow_pickle=True)

joint_pos_delta_demo= np.load('dev/demo_control/joint_position_delta_demo.npz')
print(joint_pos_delta_demo['action_list'])