"""Class definition for environment with generic lift task
"""
from perls2.envs.env import Env
import logging
class LiftEnv(Env):

    def get_observation(self):
        observation_dict = {}

        # observation_dict['rgb'] = self.sensor_interface.frames_rgb()['rgb']
        observation_dict['ee_pose'] = np.array(self.robot_interface.ee_pose)
        observation_dict['object_pose'] = self.object_interface.pose
        return observation_dict

    @property
    def name(self):
        return "LiftEnv"

    def _exec_action(self, action):
        # import pdb; pdb.set_trace()
        self.robot_interface.move_ee_delta(delta=action[:6])
        # if action[7] > 0: 
        #     self.open_gripper()

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

            self.sensor_interface.set_view_matrix(self.arena.view_matrix)
            self.sensor_interface.set_projection_matrix(
                self.arena.projection_matrix)
            self.world._wait_until_stable()

        observation = None
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

        self._exec_action(action)
        self.world.step()
        self.num_steps = self.num_steps+1

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        observation = None

        info = self.info

        return observation, reward, termination, info
    
    def is_success(self):
        if (self.object_interface.position[2] > 0.1):
            return True
        else:
            return False

    def is_done(self):
        return self._check_termination()