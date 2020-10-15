"""The parent class for environments.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import abc  # For abstract class definitions
import six  # For abstract class definitions
import gym
import gym.spaces as spaces
import numpy as np
import logging
# Use YamlConfig for config files
from perls2.utils.yaml_config import YamlConfig
# Use world factory to create world based on config
import perls2.worlds.world_factory as God


@six.add_metaclass(abc.ABCMeta)
class Env(gym.Env):
    """Abstract base class for environments.

    Attributes:
        config (dict): A dict containing parameters to create an arena, robot
            interface, sensor interface and object interface. They also contain
            specs for learning, simulation and experiment setup.
        arena (Arena): Manages the sim by loading models in both sim/real envs.
            and for simulations, randomizing objects and sensors parameters.
        robot_interface (perls2.RobotInterface): Communicates with robots and
            executes robot commands.
        sensor_interface (perls2.SensorInterface): Retrieves sensor info and
            executes changes to extrinsic/intrinsic params.
        object_interfaces (dict): Dictionary of ObjectInterfaces
        action_space
    """

    def __init__(self,
                 config,
                 use_visualizer=False,
                 name=None):
        """Initialize.

        Args:
            config (str, YamlConfig): A relative filepath to the config file. Or a parsed YamlConfig file as a dictionary.
                e.g. cfg/my_config.yaml

            use_visualizer (bool): A flag for whether or not to use visualizer

            name (str): of the environment

        Notes:
            See documentation for more details about config files.
        """

        # Get config dictionary.
        if isinstance(config, YamlConfig):
            self.config = config
        else:
            self.config = YamlConfig(config)
        self.world = God.make_world(self.config,
                                    use_visualizer,
                                    name)

        self.initialize()

    def initialize(self):
        """Create attributes for environment.

        This function creates the following attributes:
            - Arena
            - RobotInterface
            - SensorInterface
            - ObjectInterface(s) (if applicable)
            - observation_space
            - action_space
            - various counters.

        This is a public function as sometimes it is necessary to reinitialize
        an environment to fully reset a simulation.

        Args: None
        Returns: None
        """

        # Environment access the following attributes of the world directly.
        self.arena = self.world.arena
        self.robot_interface = self.world.robot_interface
        if self.world.has_camera:
            self.camera_interface = self.world.camera_interface

        self.has_object = self.world.has_object
        # Currently only sim worlds support object interfaces
        if self.world.is_sim:
            if self.has_object:
                self.object_interfaces = self.world.object_interfaces

        # Set observation space using gym spaces
        #    - Box for continuous, Discrete for discrete
        if 'env' in self.config:
            if 'observation_space' in self.config['env']:
                self.observation_space = spaces.Box(
                    low=np.array(self.config['env']['observation_space']['low']),
                    high=np.array(self.config['env']['observation_space']['high']),
                    dtype=np.float32)
            else:
                self.observation_space = spaces.Box(
                    low=np.array([0]*3),
                    high=np.array([1]*3),
                    dtype=np.float32)
        # Set action space using gym spaces.
        if 'env' in self.config:
            if 'action_space' in self.config['env']:
                self.action_space = spaces.Box(
                    low=np.array(self.config['env']['action_space']['low']),
                    high=np.array(self.config['env']['action_space']['high']),
                    dtype=np.float32)
            else:
                self.action_space = spaces.Box(
                    low=np.array([-1.0]*3),
                    high=np.array([1.0]*3),
                    dtype=np.float32)
        # Real worlds use pybullet for IK and robot control.
        if (self.config['world']['type'] == 'Bullet'):
            self._physics_id = self.world._physics_id

        self.MAX_STEPS = self.config['sim_params']['MAX_STEPS']
        self.episode_num = 0
        self.num_steps = 0

    def reset(self):
        """Reset the environment.

        Returns:
            The observation.
        """
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        self.camera_interface.reset()
        observation = self.get_observation()

        return observation

    def step(self, action, start=None):
        """Take a step.

        Execute the action first, then step the world.
        Update the episodes / steps and determine termination
        state, compute the reward function and get the observation.

        Args:
            action: The action to take.
            start: timestamp (time.time()) taken before policy computes action.
                This helps enforce policy frequency.
        Returns:
            Observation, reward, termination, info for the environment
                as a tuple.
        """
        self._exec_action(action)
        self.world.step(start)
        self.num_steps = self.num_steps + 1

        termination = self._check_termination()

        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        observation = self.get_observation()

        info = self.info

        return observation, reward, termination, info

    def get_observation(self):
        """Get observation of current env state.

        Returns:
            An observation, typically a dict with multiple things
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
        raise NotImplementedError

    def render(self, mode='human', close=False):
        """ Render the gym environment.

        See OpenAI.gym reference.

        Args:
            mode (str): string indicating type of rendering mode.
            close (bool): open/closed rendering.
        """
        raise NotImplementedError

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

        return {'name': type(self).__name__}

    def rewardFunction(self):
        """ Compute and return user-defined reward for agent given env state.
        """
        logging.warning("rewardFunction not defined!")
        return 0

    def _check_termination(self):
        """Check if episode has reached max number of steps.
        """
        return self.num_steps >= self.MAX_STEPS

    def is_done(self):
        """Public wrapper to check episode termination.
        """
        return self._check_termination()

    def is_success(self):
        """Check if the task condition is reached."""
        logging.warning("is_success not defined!")
        return False
