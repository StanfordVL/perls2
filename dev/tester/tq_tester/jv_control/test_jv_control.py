""" Class for Pybullet Sawyer environments performing a reach task using tq_control
"""
from __future__ import division

import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig
from jvenv import JVEnv
import logging 
logging.basicConfig(level=logging.DEBUG)

import matplotlib.pyplot as plt

INTERPOLATOR_FREQ = 3
MAX_T = 500
env = JVEnv('dev/tester/tq_tester/jv_config.yaml', True, 'Joint Velocity Environment')



WAYPOINTS_DQ = {0: [0, 0, 0, 0, 0,  0, -0.1],
                6: [0, 0, 0, 0, 0, -0.2, -0.2], 
                9: [0, 0, 0, 0, -0.3, -0.3, -0.3],
                12:[0, 0, 0, -0.4, -0.4, -0.4, -0.4],
                15:[0, 0, -0.5, -0.5, -0.5, -0.5, -0.5],
                18:[0, -0.6, -0.6, -0.6, -0.6, -0.6, -0.6]}


# reset environment to get robot state. 
robot_state = env.reset()

interpolator_count = 0

error_t = []
for t in range(MAX_T):
    # if t in WAYPOINTS_DQ:
    #     waypoint = WAYPOINTS_DQ[t]
    # if t == 0:
    #     logging.info("Starting...t=0")
    #     action = waypoint
    #     #action = interpolate(robot_state, waypoint)
    # else: 
    #     logging.info('t = ' + str(t))

    # if interpolator_count % INTERPOLATOR_FREQ == 0: 
    #     logging.debug("action: " + str(action))
    #     # action = interpolate(robot_state, waypoint)
    #     action = waypoint
    action = [-0.1] * 7 #[0, 0, 0, 0, 0.0,  0.0, -0.1]
    robot_state, _, _, _ = env.step(action)
    error = np.subtract(action,robot_state['dq'][:7])
    logging.debug("vel error: " + str(error))

    error_t.append(error[6])
    # input("Enter to continue....")

plt.plot(range(MAX_T), error_t)
plt.ylabel('velocity error in joint 7 (rad/s)')
plt.xlabel('t (timesteps)')
plt.show()
