"""Project template example running the environment.
"""
from __future__ import division
import time
from safenet_env import SafenetEnv
import logging
import numpy as np
from demos.demo_path import Line
# logging.basicConfig(level=logging.DEBUG)

DELTA_STEP_SIZE = 0.015

def get_next_action(curr_pose, path, step_num):
    """Get action as delta to next waypoint on path.
    """
    action = np.subtract(path[step_num][:3], curr_pose[:3])
    action = action / np.linalg.norm(action) * DELTA_STEP_SIZE
    return action

# def get_next_action(curr_pose, axis):
#     """Set next ee_pose fixed amount in x direction.
#     """
#     action = np.zeros(3)
#     action[axis] = 0.015 
#     return action

def within_tol(current, goal):
    return np.any(np.abs(current - goal) > 0.01)

env = SafenetEnv('safenet_example.yaml', True, "SafenetEnv")

step = 0
observation = env.reset()
initial_ee_pos = observation['ee_position']

if env.robot_interface.use_safenet:
    upper_limit = env.robot_interface.safenet_ee_pos_upper
    lower_limit = env.robot_interface.safenet_ee_pos_lower
else:
    upper_limit = np.add(initial_ee_pos, [0.1, 0.1, 0.1])
    lower_limit = np.add(initial_ee_pos, [-0.05, -0.05, -0.05])
    env.robot_interface.set_safenet_boundaries(lower_limit, upper_limit)

print("#############################################")
print("Safenet Example")
print("Current ee position: \t{}".format(initial_ee_pos))
print("Safenet upper limit: \t{}".format(env.robot_interface.safenet_ee_pos_upper))
print("Safenet lower limit: \t{}".format(env.robot_interface.safenet_ee_pos_lower))
print("#############################################")

axis_names = ["X", "Y", "Z"]
for axis_num, axis in enumerate(axis_names):
    print("Stepping forward along {} axis".format(axis))
    env.reset()
    step_num = 0
    path = Line(env.robot_interface.ee_pose, 
                num_pts=(0.12 / DELTA_STEP_SIZE), 
                delta_val=DELTA_STEP_SIZE, 
                dim_num=axis)
    done = False
    while not done:
        start = time.time()
        action = get_next_action(observation['ee_position'], path, step_num)
        step_num += 1
        print("Waypoint goal {}".format(path[step_num]))
        # Pass the start time to enforce policy frequency.
        observation, reward, termination, info = env.step(action, start=start)
        # if not env.world.is_sim:
        #     input("press enter to continue")
        # print("Curr ee_pos: \t{}".format(observation['ee_position']))
        if (np.any(np.subtract(observation['ee_position'], upper_limit) > 0.01) 
            or np.any(np.subtract(lower_limit, observation['ee_position']) > 0.01)) :
            print("Safenet boundary exceeded! Stopping.")
            done = True
        else: # input("press enter to step")
            done = termination

env.reset()
# In the real robot we have to use a ROS interface. Disconnect the interface
# after completing the experiment.
if (not env.world.is_sim):
    env.robot_interface.disconnect()

