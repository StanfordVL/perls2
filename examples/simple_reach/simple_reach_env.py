""" Class for Pybullet Sawyer environments performing a reach task.
"""
from __future__ import division

import time
import math
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

        # for sim we are tracking an object, increase goal position to be above
        # the actual position of the object.

        if (self.world.is_sim):
            self.object_interface = self.world.object_interfaces['013_apple']
            self.update_goal_position()

        self._initial_ee_orn = []
        self.robot_interface.reset()
        self.reset_position = self.robot_interface.ee_position

        # For randomizing goal position in real world cases. 
        self.goal_upper_bound = np.add(self.reset_position, [0.1, 0.0, 0])
        self.goal_lower_bound = np.add(self.reset_position, [-0.1, -0.0, 0])

        self._initial_ee_orn = self.robot_interface.ee_orientation

    def reset(self):
        """Reset the environment.

        This reset function is different from the parent Env function.
        The object placement and camera intrinsics/extrinsics are
        are randomized if we are in simulation.

        Returns:
            The observation.
        """
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        self._initial_ee_orn = self.robot_interface.ee_orientation
        if (self.world.is_sim):
            if self.config['object']['random']['randomize']:
                self.object_interface.place(self.arena.randomize_obj_pos())
            else:
                self.object_interface.place(
                    self.config['object']['object_dict']['object_0']['default_position'])

            self.camera_interface.set_view_matrix(self.arena.view_matrix)
            self.camera_interface.set_projection_matrix(
                self.arena.projection_matrix)
            self.world.wait_until_stable()
        else:
            self.goal_position = self.arena.random_vec_bounded(
                lower_bound=self.goal_lower_bound, upper_bound=self.goal_upper_bound)
            print("Current pose: {}".format(self.robot_interface.ee_pose))
            print("New Goal Position:\n\t {}\n".format(self.goal_position))
            print("Goal orientation :\n\t {}\n".format(self._initial_ee_orn))
        input("Press enter to continue.")
        observation = self.get_observation()

        return observation


    def step(self, action, start=None):
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
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._exec_action(action)
        self.world.step(start)
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
        delta = self.goal_position - self.robot_interface.ee_position
        
        if (self.world.is_sim):
            camera_img = self.camera_interface.frames()
            observation = (delta, current_ee_pose, camera_img.get('image'))
        else: 
            observation = (delta, current_ee_pose, None)        
        return observation

    def update_goal_position(self):
        """Take current object position to get new goal position

            Helper function to raise the goal position a bit higher
            than the actual object.
        """
        goal_height_offset = 0.2
        object_pos = self.object_interface.position
        object_pos[2] += goal_height_offset
        self.goal_position = object_pos

    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """
        
        action = np.hstack((action, np.zeros(3)))
        self.robot_interface.move_ee_delta(delta=action, set_ori=self._initial_ee_orn)

    def _check_termination(self):
        """ Query state of environment to check termination condition

        Check if end effector position is within some absolute distance
        radius of the goal position or if maximum steps in episode have
        been reached.

            Args: None
            Returns: bool if episode has terminated or not.
        """
        # radius for convergence
        convergence_radius = 0.01

        abs_dist = self._get_dist_to_goal()
        if (abs_dist < convergence_radius):
            logging.debug("done - success!")
            return True
        if (self.num_steps > self.MAX_STEPS):
            logging.debug("done - max steps reached")
            logging.debug("final delta to goal \t{}".format(abs_dist))
            return True
        else:
            return False

    def _get_dist_to_goal(self):

        current_ee_pos = np.asarray(self.robot_interface.ee_position)
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
        return {}

    def rewardFunction(self):
        dist_to_goal = self._get_dist_to_goal()
        reward = 1 - math.tanh(dist_to_goal)
        return reward
