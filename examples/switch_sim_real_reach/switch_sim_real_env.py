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

        # store the initial orientation to be maintained
        self._initial_ee_orn = []

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

        # if we are in a simulation, reset the sim
        # otherwise just update the goal position.
        if (self.world.is_sim):
            if self.config['object']['random']['randomize']:
                self.object_interface.place(self.arena.randomize_obj_pos())
            else:
                self.object_interface.place(self.config['object']['default_position'])

            self.sensor_interface.set_view_matrix(self.arena.view_matrix)
            self.sensor_interface.set_projection_matrix(self.arena.projection_matrix)
            self.world._wait_until_stable()
        else:
            self.goal_position = self.arena.goal_position

        observation = self.get_observation()

        return observation

    def step(self):
        """ Step the environment forward.

        This only implements the parent class and is only included
        to show that it explicitly remains the same.
        """
        return super().step()

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
        observation = (delta, current_ee_pose, camera_img.get('image') )
        return observation

    def raise_goal_position(self):
        """Take current object position to get new goal position

            Helper function to raise the goal position a bit higher
            than the actual object.
        """
        goal_height_offset = 0.2
        object_pos = self.object_interface.get_position()
        #print("object pos " + str(object_pos))
        object_pos[2] += goal_height_offset
        self.goal_position =  object_pos

    def _exec_action(self, action):
        """Applies the given action to the simulation.
        """

        if self.world.is_sim:
            next_position = np.clip(
                list(action + self.robot_interface.ee_position),
                [-100, -1, 0.20], [100, 100, 10])

        else:
            next_position =  self.robot_interface.ee_pose[0:3] + action

            print('Currente ee pose: ' + str(self.robot_interface.ee_pose[0:3]))
            print('dist_to_goal      ' + str(self._get_dist_to_goal()))

            lower_bound = self.config['goal_position']['lower']
            upper_bound = self.config['goal_position']['upper']

            next_position = np.clip(
                next_position, lower_bound, upper_bound)

            print('Next position: ' + str(next_position))

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

        abs_dist= self._get_dist_to_goal()
        if (abs_dist < convergence_radius):
            print("done - success!")
            return True
        if (self.num_steps > self.MAX_STEPS):
            print("done - max steps reached")
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
        #print('goal     ' + str(self.goal_position))
        #print('current_ee_pos       ' + str(current_ee_pos))

            abs_dist = np.linalg.norm(self.goal_position - current_ee_pos)
        #print('dist ' + str(abs_dist))

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
                #'name': type(self).__name__,

                }

    def rewardFunction(self):
        dist_to_goal = self._get_dist_to_goal()
        reward = 1 - math.tanh(dist_to_goal)
        return reward


