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
PATH_LENGTH = 0.15
NUM_STEPS = int(PATH_LENGTH / DELTA_STEP_SIZE)
BOUNDARY_TOL = 0.01
# Length of boundary box from intial pose
UPPER_LIMIT_DELTA = 0.1
LOWER_LIMIT_DELTA = -0.05


def get_next_action(curr_pose, path, step_num):
    """Get action as delta to next waypoint on path.
    """
    action = np.subtract(path[step_num][:3], curr_pose[:3])
    if np.all(action != np.zeros(3)):
        action = action / np.linalg.norm(action) * DELTA_STEP_SIZE
    return action

def exceeds_boundaries(goal, upper, lower, tol):
    """Determine if waypoint goal exceeds safenet boundaries.
    """
    return (np.any(np.subtract(goal, upper) > 0.01) 
        or np.any(np.subtract(lower, goal) > 0.01))


env = SafenetEnv('safenet_example.yaml', True, "SafenetEnv")

step = 0
observation = env.reset()
initial_ee_pos = observation['ee_position']

if env.robot_interface.use_safenet:
    upper_limit = env.robot_interface.safenet_ee_pos_upper
    lower_limit = env.robot_interface.safenet_ee_pos_lower
else:
    upper_limit = np.add(initial_ee_pos, [UPPER_LIMIT_DELTA]*3)
    lower_limit = np.add(initial_ee_pos, [LOWER_LIMIT_DELTA]*3)
    env.robot_interface.set_safenet_boundaries(lower_limit, upper_limit)

print("#############################################")
print("Safenet Example")
print("Current ee position: \t{}".format(initial_ee_pos))
print("Safenet upper limit: \t{}".format(env.robot_interface.safenet_ee_pos_upper))
print("Safenet lower limit: \t{}".format(env.robot_interface.safenet_ee_pos_lower))
print("#############################################")

axis_names = ["X", "Y", "Z"]
for axis_num, axis in enumerate(axis_names):
    input("Press Enter to begin")

    env.reset()
    step_num = 0
    path = Line(env.robot_interface.ee_pose, 
                num_pts=NUM_STEPS, 
                delta_val=DELTA_STEP_SIZE, 
                dim=axis_num)
    done = False
    goal_exceeds_boundary = False
    print("Stepping forward along {} axis".format(axis))
    while not done and step_num < NUM_STEPS:
        start = time.time()
        action = get_next_action(observation['ee_position'], path.path, step_num)

        # Alert if goal exceeds boundary box set by safenet.
        if exceeds_boundaries(path.path[step_num][:3], 
                              env.robot_interface.safenet_ee_pos_upper,
                              env.robot_interface.safenet_ee_pos_lower, BOUNDARY_TOL):
            goal_exceeds_boundary = True
            print("Waypoint goal exceeds safenet boundary \t {}".format(path.path[step_num][:3]))
        
        step_num += 1
        observation, reward, termination, info = env.step(action, start=start)
        
        if goal_exceeds_boundary:
            if not exceeds_boundaries(observation['ee_position'], upper_limit, lower_limit, BOUNDARY_TOL) :
                print("EE still within boundary\t{}".format(observation['ee_position']))

        # Alert if the ee_position exceeds safenet boundary.
        if exceeds_boundaries(observation['ee_position'], upper_limit, lower_limit, BOUNDARY_TOL) :
            print("Safenet boundary exceeded!")

        done = termination

env.reset()
# In the real robot we have to use a ROS interface. Disconnect the interface
# after completing the experiment.
if (not env.world.is_sim):
    env.robot_interface.disconnect()

