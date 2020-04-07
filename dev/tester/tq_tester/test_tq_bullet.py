""" Class for Pybullet Sawyer environments performing a reach task using tq_control
"""
from __future__ import division

import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig
from simple_reach_env_tq import SimpleReachEnvTq
import logging 
logging.basicConfig(level=logging.DEBUG)

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
    step = 0.1
    delta = observation
    action = step * delta/np.linalg.norm(delta)
    return action


env = SimpleReachEnvTq('dev/tester/tq_tester/tester_config.yaml', True, None)

# Lists for saving demonstrations
training_list = []
image_list = []
pose_list = []
action_list = []

for ep_num in range(10):
    if ep_num == 0:
        logging.info("Beginning episode 0")
    else: 
        logging.info('episode ' +  str(ep_num-1) + ' complete...pausing...')
  # Wait for real robots to show episode is complete
    if env.world.is_sim is False:
        time.sleep(3)
    step = 0
    observation = env.reset()

    done = False
    while done != True:
        action = get_action(observation[0])
        logging.debug("action: " + str(action))
        start = time.time()
        observation, reward, termination, info = env.step(action)
        step_record = (action, observation, reward,  termination)

        # Add observations-actions to demonstration lists.
        pose_list.append(observation[1])
        image_list.append(observation[2])
        action_list.append(action)

        if env.world.is_sim is False:
            # enforce policy frequency by waiting
            while ((time.time() - start) < 0.05):
                pass
        step += 1
        done = termination
# In the real robot we have to use a ROS interface. Disconnect the interface
# after completing the experiment.
if (not env.world.is_sim):
    env.robot_interface.disconnect()
    env.sensor_interface.disconnect()
