"""Class definition for environment with generic lift task
"""
from perls2.envs.env import Env
import logging
import numpy as np
class LiftEnv(Env):
    def get_observation(self):
        observation_dict = {}

        # observation_dict['rgb'] = self.camera_interface.frames_rgb()['rgb']
        observation_dict['ee_pose'] = self.robot_interface.ee_pose
        observation_dict['object_pose'] = self.object_interface.pose
        observation_dict['q'] = self.robot_interface.q
        observation_dict['dq'] = self.robot_interface.dq
        observation_dict['last_torques'] = self.robot_interface.last_torques
        return observation_dict

    @property
    def name(self):
        return "RobotTeleop"

    def get_grasp_from_command(self, gripper_command):
        """
        Returns a boolean grasp value for the input continuous control command.
        This function uses a decision boundary to decide whether the input
        command corresponds to closing the gripper or opening it.
        """
        return (gripper_command[0] < 0.5)

    def _exec_action(self, position_control, rotation_control, gripper_control,
        force_control=None, absolute=False):

        action = np.concatenate([position_control, rotation_control, gripper_control])

        if absolute:
            print("absolute")
            arm_action = np.hstack((position_control, rotation_control))

            self.robot_interface.set_ee_pose(arm_action)

        else:
            position_control = np.clip(position_control,[-0.05] * 3, [0.05]*3)
            #rotation_control = [0, 0, 0]#np.clip(rotation_control,[-0.05] * 3, [0.05]*3)
            delta = np.hstack((position_control, rotation_control))

            self.robot_interface.move_ee_delta(delta=delta)

        grasp = self.get_grasp_from_command(gripper_control)

        if grasp:
            self.robot_interface.close_gripper()
        else:
            self.robot_interface.open_gripper()

        # remember the last gripper action taken
        self.last_grasp = grasp
        # print("returning action")


        return action

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
            if self.config['object']['random']['randomize']:
                self.object_interface.place(self.arena.randomize_obj_pos())
            else:
                self.object_interface.place(
                    self.config['object']['default_position'])

            self.camera_interface.set_view_matrix(self.arena.view_matrix)
            self.camera_interface.set_projection_matrix(
                self.arena.projection_matrix)
            self.world.wait_until_stable()

        observation = None
        return observation

    def _step(self):
        self.world.step()
        #obs = self.get_observation()
        self.num_steps = self.num_steps + 1


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

        self._exec_action(action[0:3],action[3:6],action[6:])
        self._step()
        observation = None

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        info = self.info

        return observation, reward, termination, info

    def is_success(self):
        if (self.object_interface.position[2] > 0.05):
            return True
        else:
            return False

    def is_done(self):
        return self._check_termination()

    def rewardFunction(self):
        if self.is_success():
            return 1.0
        else:
            return 0.0