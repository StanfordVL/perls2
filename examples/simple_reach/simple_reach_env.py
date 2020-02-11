""" Class for Pybullet Sawyer environments performing a reach task.
"""
from __future__ import division

import time
import math
import pybullet
import numpy as np
from perls2.utils.yaml_config import YamlConfig

from perls2.envs.env import Env

import gym.spaces as spaces
import logging


class SimpleReachEnv(Env):
    """The class for Pybullet Sawyer Robot environments performing a reach task.
    """

    def __init__(self,
                 cfg_path=None,
                 use_visualizer=False,
                 name=None):
        """Initialize.

        Parameters
        ----------
        config: dict
            A dict with config parameters
        arena:
            container for setting up the world in which robot and objects
            interact
        use_visualizer:
            Whether or not to use visualizer. NOTE: Pybullet only allows for one
            simulation to be connected to GUI. It is up to the user to manage
            this
        """
        super().__init__(cfg_path, use_visualizer, name)

        self.goal_position = [0, 0, 0]

        # for sim we are tracking an object, increase goal position to be above
        # the actual position of the object.
        self.object_interface = self.world.objects['013_apple']

        self.target_object = self.world.objects['013_apple']


        if (self.world.is_sim):
            self.update_goal_position()

        self._initial_ee_orn = []
    def reset(self):
        """Reset the environment.

        This reset function is different from the parent Env function.
        The object placement and camera intrinsics/extrinsics are
        are randomized if we are in simulation.

        Returns:
            The observation.
        """

        logging.info(
            "Environment reset - physicsClient: " + str(self._physics_id))
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        self._initial_ee_orn = self.robot_interface.ee_orientation
        if (self.world.is_sim):
            logging.debug("randomizing object positions")
            if self.config['object']['random']['randomize']:
                self.target_object.place(self.arena.randomize_obj_pos())
            else:
                self.target_object.place(
                    self.target_object.default_position)

            self.sensor_interface.set_view_matrix(self.arena.view_matrix)
            self.sensor_interface.set_projection_matrix(
                self.arena.projection_matrix)
            logging.debug("waiting till stable")
            self.world._wait_until_stable()
            logging.debug("stable")

        else:
            self.goal_position = self.arena.goal_position

        observation = self.get_observation()

        return observation

    def step(self, action):
        """Take a step.

        Args:
            action: The action to take.
        Returns:
            -observation: based on user-defined functions
            -reward: from user-defined reward function
            -done: whether the task was completed or max steps reached
            -info: info about the episode including success

        Takes a step forward similar to openAI.gym's implementation.
        """
        logging.debug('stepping')
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._exec_action(action)
        self.world.step()
        self.num_steps = self.num_steps+1

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        observation = self.get_observation()

        info = self.info()

        return observation, reward, termination, info

    def get_observation(self):
        """Get observation of current env state

        Returns:
            An observation, a tuple with the delta from goal position,
            current_ee_pose, and the rgb image from camera at the step.
        """

        """To avoid dealing with collisions, we want the robot to reach
        for a target above the object position in simulation.
        """
        if(self.world.is_sim):
            self.update_goal_position()

        current_ee_pose = self.robot_interface.ee_pose
        camera_img = self.sensor_interface.frames()
        delta = (self.goal_position - current_ee_pose[0:3])
        observation = (delta, current_ee_pose, camera_img.get('image'))
        return observation

    def update_goal_position(self):
        """Take current object position to get new goal position

            Helper function to raise the goal position a bit higher
            than the actual object.
        """
        goal_height_offset = 0.2
        object_pos = self.target_object.get_position()
        object_pos[2] += goal_height_offset
        self.goal_position = object_pos

    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """

        if self.world.is_sim:
            next_position = np.clip(
                list(action + self.robot_interface.ee_position),
                [-100, -1, 0.20], [100, 100, 10])

        else:
            next_position = self.robot_interface.ee_pose[0:3] + action

            logging.debug(
                'Currente ee pose: ' + str(self.robot_interface.ee_pose[0:3]))
            logging.debug(
                'dist_to_goal      ' + str(self._get_dist_to_goal()))

            lower_bound = self.config['goal_position']['lower']
            upper_bound = self.config['goal_position']['upper']

            next_position = np.clip(
                next_position, lower_bound, upper_bound)

            logging.debug(
                'Next position: ' + str(next_position))

        self.robot_interface.ee_pose = (list(next_position) +
                                        [0, 0.952846, 0, 0.303454])

    def _check_termination(self):
        """ Query state of environment to check termination condition

        Check if end effector position is within some absolute distance
        radius of the goal position or if maximum steps in episode have
        been reached.

            Args: None
            Returns: bool if episode has terminated or not.
        """
        # radius for convergence
        convergence_radius = 0.1

        abs_dist = self._get_dist_to_goal()
        if (abs_dist < convergence_radius):
            logging.debug("done - success!")
            return True
        if (self.num_steps > self.MAX_STEPS):
            logging.debug("done - max steps reached")
            return True
        else:
            return False

    def _get_dist_to_goal(self):
        if (self.world.is_sim):
            current_ee_pos = np.asarray(self.robot_interface.ee_position)
            abs_dist = np.linalg.norm(self.goal_position - current_ee_pos)

            return abs_dist
        else:
            current_ee_pos = np.asarray(self.robot_interface.ee_pose[0:3])
            abs_dist = np.linalg.norm(self.goal_position - current_ee_pos)

            return abs_dist

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

    def handle_exception(self, e):
        """Handle an exception.
        """
        pass

    def info(self):
        return {

                }

    def rewardFunction(self):
        dist_to_goal = self._get_dist_to_goal()
        reward = 1 - math.tanh(dist_to_goal)
        return reward
