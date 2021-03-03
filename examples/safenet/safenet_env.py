"""Template Environment for Projects.
"""
from perls2.envs.env import Env
import numpy as np
import pybullet as pb


class SafenetEnv(Env):
    """Environment to demonstrate safenet functionality.
    """

    def __init__(self,
                 cfg_path='template_project.yaml',
                 use_visualizer=True,
                 name="SafenetEnv"):
        """Initialize the environment.

        Set up any variables that are necessary for the environment and your task.
        """
        super().__init__(cfg_path, use_visualizer, name)
        self.robot_interface.reset()
        self.init_ori = self.robot_interface.ee_orientation
        self.boundary_color = [1, 0, 0] # Red for safenet boundary box.


    def _draw_boundary_line(self, c1, c2):
        pb.addUserDebugLine(c1, c2, self.boundary_color, self.world.physics_id)

    def visualize_boundaries(self):
        """Add visualization to pb sim for safenet boundary.
        """
        if self.world.is_sim:
            if self.robot_interface.use_safenet:
                (upper, lower) = self.robot_interface.get_safenet_limits()
                if (upper is not None) and (lower is not None):
                    corners = {}
                    corners['000'] = lower
                    corners['100'] = [upper[0], lower[1], lower[2]]
                    corners['110'] = [upper[0], upper[1], lower[2]]
                    corners['010'] = [lower[0], upper[1], lower[2]]
                    corners['001'] = [lower[0], lower[1], upper[2]]
                    corners['011'] = [lower[0], upper[1], upper[2]]
                    corners['111'] = upper
                    corners['101'] = [upper[0], lower[1], upper[2]]

                    self._draw_boundary_line(corners['000'], corners['100'])
                    self._draw_boundary_line(corners['100'], corners['110'])
                    self._draw_boundary_line(corners['110'], corners['010'])
                    self._draw_boundary_line(corners['010'], corners['000'])

                    self._draw_boundary_line(corners['000'], corners['001'])
                    self._draw_boundary_line(corners['001'], corners['011'])
                    self._draw_boundary_line(corners['011'], corners['010'])

                    self._draw_boundary_line(corners['011'], corners['111'])
                    self._draw_boundary_line(corners['111'], corners['101'])
                    self._draw_boundary_line(corners['101'], corners['001'])
                    
                    self._draw_boundary_line(corners['111'], corners['110'])
                    self._draw_boundary_line(corners['101'], corners['100'])


    def visualize_controller_goal(self):
        pb.addUserDebugLine(self.robot_interface.ee_position, self.robot_interface.controller.goal_pos, [0, 1, 0], lineWidth=2.0)

    def get_observation(self):
        """Get observation of current env state

        Returns:
            observation (dict): dictionary with key values corresponding to
                observations of the environment's current state.

        """
        obs = {}
        obs['ee_position'] = self.robot_interface.ee_position
        return obs

    def _exec_action(self, action):
        """Applies the given action to the environment.

        Args:
            action (list): usually a list of floats bounded by action_space.

        Examples:

            # move ee by some delta in position while maintaining orientation
            desired_ori = [0, 0, 0, 1] # save this as initial reset orientation.
            self.robot_interface.move_ee_delta(delta=action, set_ori=desired_ori)

            # Set ee_pose (absolute)
            self.robot_interface.set_ee_pose(set_pos=action[:3], set_ori=action[3:])

            # Open Gripper:
            self.robot_interface.open_gripper()

            # Close gripper:
            self.robot_interface.close_gripper()

        """
        # make delta 6f to include orientation
        action = np.hstack((action, np.zeros(3)))
        self.robot_interface.move_ee_delta(action, set_ori=self.init_ori)
        if self.world.is_sim:
            """ Special cases for sim
            """
            pass
        else:
            """ Special cases for real world.
            """
            pass

    def reset(self):
        """Reset the environment.

        This reset function is different from the parent Env function.
        The object placement and camera intrinsics/extrinsics are
        are randomized if we are in simulation.

        Returns:
            The observation (dict):
        """
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        if (self.world.is_sim):
            """
            Insert special code for resetting in simulation:
            Examples:
                Randomizing object placement
                Randomizing camera parameters.
            """
            pass
        else:
            """
            Insert code for reseting in real world.
            """
            pass

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
        if self.world.is_sim:
            self.visualize_controller_goal()
        self.world.step(start)
        self.num_steps = self.num_steps + 1

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        observation = self.get_observation()

        info = self.info()

        return observation, reward, termination, info

    def _check_termination(self):
        """ Query state of environment to check termination condition

        Check if end effector position is within some absolute distance
        radius of the goal position or if maximum steps in episode have
        been reached.

            Args: None
            Returns: bool if episode has terminated or not.
        """
        if (self.num_steps > self.MAX_STEPS):
            return True
        else:
            return False

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
        """Implement reward function here.
        """
        return -1
