""" object loading example
"""

from examples.simple_reach.simple_reach_env import SimpleReachEnv
import numpy as np

class ObjectEnv(Env):
    def __init__(self, 
                 cfg_path=object_cfg.yaml,
                 use_visualizer=None, 
                 name=None):
    """Initialize
    """
    super().__init__(cfg_path, use_visualizer, name)
    self.object_inte