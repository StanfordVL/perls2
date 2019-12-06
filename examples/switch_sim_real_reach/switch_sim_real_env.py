import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig

from perls2.envs.env import Env

import gym.spaces as spaces


class SwitchSimRealEnv(Env):
    """Class to demo switching between real and sim

    This class highlights logic may be shared for a task for both real and sim
    worlds. The task is to reach a pose. In simulation, this pose will come
    from the object interface. In reality, we will randomly generate a pose
    within some bounds.

    Attributes:
        goal_position (list): xyz position to reach.
    """

    def __init__(self,
                cfg_path=None,
                use_visualizer=False,
                name=None):
    """Initialize.

    Args:
        cfg_path (str): string for the path to get the config file
        use_visualizer (bool): whether or not to use the visualizer
        name (str): name identifying the environment
    """
    super().__init__(cfg_path,use_visualizer,name)
    self.goal_position = [0,0,0]

    # The is_sim attribute of world is used to designate code for running in
    # simulation only. In simulation, we get our goal position from the object
    # interface. We want the goal position to be a little higher than the
    # object, so we update the goal position.
    if self.world.is_sim:
        self.raise_goal_position()