"""Project template example running the environment.
"""
from __future__ import division
import time
from safenet_env import SafenetEnv
import logging
import numpy as np
# logging.basicConfig(level=logging.DEBUG)


def get_action(observation):
    """Run your policy to produce an action.
    """
    action = [0, 0, 0, 0, 0, 0]
    return action

def get_next_action(curr_pose, axis):
    """Set next ee_pose fixed amount in x direction.
    """
    action = np.zeros(3)
    action[axis] = 0.01 
    return action

def within_tol(current, goal):
    return np.any(np.abs(current - goal) > 0.01)

env = SafenetEnv('safenet_example.yaml', True, "SafenetEnv")


step = 0
observation = env.reset()
initial_ee_pos = observation['ee_position']
upper_limit = np.add(initial_ee_pos, [0.1, 0.1, 0.1])
lower_limit = np.add(initial_ee_pos, [-0.1, -0.1, -0.1])

# Set safenet boundaries
env.robot_interface.set_safenet_boundaries(lower_limit, upper_limit)

print("#############################################")
print("Safenet Example")
print("Current ee position: \t{}".format(initial_ee_pos))
print("Safenet upper limit: \t{}".format(upper_limit))
print("Safenet lower limit: \t{}".format(lower_limit))
print("#############################################")

axis_names = ["X", "Y", "Z"]
for axis_num, axis in enumerate(axis_names):
    print("Stepping forward along {} axis".format(axis))
    env.reset()
    done = False
    while not done:
        start = time.time()
        action = get_next_action(observation['ee_position'], axis_num)
        # Pass the start time to enforce policy frequency.
        observation, reward, termination, info = env.step(action, start=start)
        # print("Curr ee_pos: \t{}".format(observation['ee_position']))
        if (np.any(np.subtract(observation['ee_position'], upper_limit) > 0.01) 
            or np.any(np.subtract(lower_limit, observation['ee_position']) > 0.01)) :
            print("Safenet boundary exceeded!")
            done = True
        # input("press enter to step")
        done = termination

    # In the real robot we have to use a ROS interface. Disconnect the interface
    # after completing the experiment.
    if (not env.world.is_sim):
        env.robot_interface.disconnect()

