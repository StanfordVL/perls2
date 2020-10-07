"""The parent class for environments.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os.path
import abc  # For abstract class definitions
import six  # For abstract class definitions
import gym


@six.add_metaclass(abc.ABCMeta)
class World():
    """Abstract base class for Worlds.

    A World encompasses all necessary components for managing an experiment in
    simulation or reality. Worlds generate the appropriate robot, sensor and
    object interfaces based on the configuration file.

    Attributes:
        config (dict): Config files contain parameters to create an arena,
            robot interface, sensor interface and object interface. They also
            contain specs for learning, simulation and experiment setup.
        arena (Arena): Manages the sim by loading models for real and sim
            and for simulations, randomizing objects and sensors parameters
        robot_interface (RobotInterface): Communicates with robots and executes
            robot commands.
        sensor_interface (SensorInterface): Retrieves sensor info and executes
            changes to params
        is_sim (bool): Flag that is true if world is simulation.
    """

    def __init__(self,
                 cfg_path):
        """Initialize.

        Args:
            cfg_path (str): A relative filepath to the config file.
        Returns: None

        Notes:

            *Worlds should be created using world factory. This ensures the
                proper world type is created

        Examples:
            world = World('cfg/my_cfg.yaml')

                See documentation for more details about config files.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def reset(self):
        """Reset the world to initial state.

        Returns:
            The observation.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def step(self, start=None):
        """Step simulation forward.
        """
        raise NotImplementedError
