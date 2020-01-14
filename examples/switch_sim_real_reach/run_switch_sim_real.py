"""Run Switch between sim and real reach task example.
"""
from __future__ import division

from switch_sim_real_env import SwitchSimRealEnv
import perls2.utils.exp_save as saver
import numpy as np
import gym
import logging
logging.basicConfig(level=logging.DEBUG)
import os
import sys

import time


def get_action(observation):
    """Dummy policy to get action based on policy

    Given a delta xyz position from the end effector to the goal, return a
    vector in that direction with fixed magnitude

    Args:
        observation (3f): a vector of 3 floats corresponding to
          goal_position - current_ee_position.
        Returns:
        action (3f): vector of 3 floats corresponding to a delta end effector
          position.

    """
    # Get components from observations
    step = 0.05
    delta = observation
    action = step * delta / np.linalg.norm(delta)
    return action


env = SwitchSimRealEnv('./simple_reach.yaml', True, None)

# Lists for saving demonstrations
training_list = []
image_list = []
pose_list = []
action_list = []
step = 0
for ep_num in range(10):
    # Wait for real robots to show episode is complete
    if not env.world.is_sim:
        time.sleep(3)

    step = 0
    observation = env.reset()
    logging.debug("ep num " + str(ep_num))
    done = False
    while done is False:
        action = get_action(observation[0])
        start = time.time()
        observation, reward, termination, info = env.step(action)

        step_record = (action, observation, reward,  termination)

        # Add observations-actions to demonstration lists.
        pose_list.append(observation[1])
        image_list.append(observation[2])
        action_list.append(action)

        # enforce policy frequency by waiting
        while ((time.time() - start) < 0.05):
            pass
        step += 1
        #saver.save_image(observation[2],  ep_num, step, folder_path='output', invert=False)
        logging.debug("step taken" + str(step))
        done = termination
# In the real robot we have to use a ROS interface. Disconnect the interface
# after completing the experiment.
if (env.world.is_sim is False):
    env.robot_interface.disconnect()
    env.sensor_interface.disconnect()
