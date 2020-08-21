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
        object_interface (ObjectInterface): Retrieves object info and excecutes
            changes to params

    Public methods (similar to openAI gym):

        step:
            Step env forward and return observation, reward, termination, info.
            Not typically user-defined but may be modified.

        reset:
            Reset env to initial state and return observation.
            Some aspects such as randomization are user-defined
        render:
        close:
        seed:

    Notes:

        *Arenas and interfaces are, by default, initialized specific to
         the domain of the env i.e. BulletWorld creates a BulletArena and
         Bullet_Robot/Sensor/Object_Interfaces.

        *For any env, users need to set the following methods:
            reset:
                Define aspects for randomization/positioning. Maybe set up in
                config file.
            get_observation:
                Define the observation returned by env upon reset, step.
            rewardFunction:
                Compute the reward received by the agent given current state of
                the environment
            exec_action:
                Define how action received by agent should be executed.


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
        """Reset the environment.

        Returns:
            The observation.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def step(self, action, start=None):
        """Take a step.

        Args:
            action: The action to take.
        Returns:
            Observation, reward, termination, info for the environment.
        """
        raise NotImplementedError

    def visualize(self, observation, action):
        """Visualize the action - that is,
        add visual markers to the world (in case of sim)
        or execute some movements (in case of real) to
        indicate the action about to be performed.

        Args:
            observation: The observation of the current step.
            action: The selected action.
        """
        pass

    def render(self, mode='human', close=False):
        """ Render the gym environment
        """
        pass

    def handle_exception(self, e):
        """Handle an exception.
        """
        pass

    @property
    def info(self):
        """ Return dictionary with env info

            This may include name, number of steps, whether episode was
            successful, or other useful information for the agent.
        """

        return {
                'name': type(self).__name__,
                }
