"""Project template example running the environment.
"""
from __future__ import division
import time
from safenet_env import SafenetEnv
import logging
import numpy as np
import pybullet as pb
from demos.demo_path import Line

# Step size between each action
DELTA_STEP_SIZE = 0.001 # [m]

# Length path travels in one direction
PATH_LENGTH = 0.3 # [m]
NUM_STEPS = int(PATH_LENGTH / DELTA_STEP_SIZE)

# Tolerance to alert for safenet.
BOUNDARY_TOL = 0.01 # [m]

# Length of boundary box from intial pose
UPPER_LIMIT_DELTA = [0.15]*3  # x, y, z [m]
LOWER_LIMIT_DELTA = [-0.1]*3 # x, y, z, [m]


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
    return np.any(goal > upper) or np.any(goal < lower)
    # dims = ["x", "y", "z"]
    # bounds = ["upper", "lower"]
    # bounds_exceeded = false
    # for j, bound in enumerate(bounds):
    #     for i, dim in enumerate(dims):
    #         if bound == "upper":
    #             delta = np.subtract(goal[i], upper[i])
    #         if bound == "lower":
    #             delta = np.subtract(lower[i], upper[i])
    #         if np.subtract(goal[i],upper[i]) > 0.01
    #             print("{} {} exceeded by {}".format(dims[i], ))



env = SafenetEnv('safenet_example.yaml', True, "SafenetEnv")

pb_id = 0
# Get pybullet id for visualization.
if env.world.is_sim:
    pb_id = env.world.physics_id
step = 0
observation = env.reset()
initial_ee_pos = observation['ee_position']

# You can set safenet boundaries via config or by function call. 


if env.robot_interface.use_safenet:
    # Config dict 'safenet' loaded by default
    upper_limit = env.robot_interface.safenet_ee_pos_upper
    lower_limit = env.robot_interface.safenet_ee_pos_lower
else:
    # Set boundary using function call.
    upper_limit = np.add(initial_ee_pos, UPPER_LIMIT_DELTA).tolist()
    lower_limit = np.add(initial_ee_pos, LOWER_LIMIT_DELTA).tolist()
    env.robot_interface.set_safenet_boundaries(lower_limit, upper_limit)

# Get current safenet boundaries
(lower, upper) = env.robot_interface.get_safenet_limits()

print("#############################################")
print("Safenet Example")
print("Current ee position: \t{}".format(initial_ee_pos))
print("Safenet upper limit: \t{}".format(upper))
print("Safenet lower limit: \t{}".format(lower))
print("#############################################")

# Visualize safenet box.
if env.world.is_sim:
    env.visualize_boundaries()
axis_names = ["X", "Y", "Z", "-X", "-Y", "-Z"]

for axis_num, axis in enumerate(axis_names):
    input("Press Enter to begin")

    env.reset()
    step_num = 0
    # Go in opposite direction for each axis.
    if axis in ["-X", "-Y", "-Z"]:
        delta_val = -DELTA_STEP_SIZE
        print("Stepping backward along {} axis".format(axis[1]))
    else:
        delta_val = DELTA_STEP_SIZE
        print("Stepping forward along {} axis".format(axis))

    path = Line(env.robot_interface.ee_pose, 
                num_pts=NUM_STEPS, 
                delta_val=delta_val, 
                dim=axis_num % 3)
    
    if env.world.is_sim:
        # Add a marker to visualize path in pybullet
        goal_marker_id = pb.addUserDebugLine(path.path[0][:3], path.path[-1][:3], [0, 0, 1.0], lineWidth=5.0, physicsClientId=env.world.physics_id)
    
    done = False
    goal_exceeds_boundary = False

    while not done and step_num < NUM_STEPS:
        start = time.time()
        action = get_next_action(observation['ee_position'], path.path, step_num)

        # Alert if goal exceeds boundary box set by safenet.
        if exceeds_boundaries(path.path[step_num][:3], 
                              env.robot_interface.safenet_ee_pos_upper,
                              env.robot_interface.safenet_ee_pos_lower, BOUNDARY_TOL):
            goal_exceeds_boundary = True
            if not env.world.is_sim:
                print("Waypoint goal exceeds safenet boundary \t {}".format(path.path[step_num][:3]))

        step_num += 1
        observation, reward, termination, info = env.step(action, start=start)
        
        if goal_exceeds_boundary and not env.world.is_sim:
            if not exceeds_boundaries(observation['ee_position'], upper_limit, lower_limit) :
                print("EE still within boundary\t{}".format(observation['ee_position']))

        # Alert if the ee_position exceeds safenet boundary.
        if exceeds_boundaries(observation['ee_position'], upper_limit, lower_limit) :
            print("Safenet boundary exceeded!")
            print(env.robot_interface.ee_position)

        done = termination

input("press enter to finish")
env.reset()

# In the real robot we have to use a CtrlInterface. Disconnect the interface
# after completing the experiment.
if (not env.world.is_sim):
    env.robot_interface.disconnect()

