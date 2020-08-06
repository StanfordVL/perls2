class DummyEnv(Env):
    """ An perls2 environment wrapped by BulletSawyerRobot. This is the env responsible
    for running the simulation.
    """

    def get_observation(self):
        """ Return observations as a dictionary. 
        """
        observation_dict = {}

        # observation_dict['rgb'] = self.sensor_interface.frames_rgb()['rgb']
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

        Was originally outside in Bullet Sawyer Wrapper but added here to ensure
        parity between this env and EnvPerls2.
        """
        return (gripper_command[0] < 0.5)

    def _exec_action(self, action, 
        force_control=None, absolute=False):
        """
        Version of control_with_osc for use with EnvPerls2
        """
        position_control = action[:3]
        rotation_control = action[3:6]
        gripper_control = action[6:]

        #action = np.concatenate([position_control, rotation_control, gripper_control])
  
        self.robot_interface.move_ee_delta(delta=action[:6])

        if gripper_control < 0:
            self.robot_interface.close_gripper()
        else:
            self.robot_interface.open_gripper()

        # print("returning action")

        return action

    def reset(self):
        """Reset the environment.

        This reset function is different from the parent Env function.
        The Bullet Sawyer Robot wrapper is responsible for reseting the sim.

        Returns:
            None
        """
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()

        if self.world.config['object']['random']['randomize']:
            print("RANDOMIZED OBJ")
            self.object_interface.place(self.arena.randomize_obj_pos())
        else:
            self.object_interface.place(
                self.config['object']['default_position'])

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
        self.num_steps = self.num_steps + 1
        #logging.info("step {}".format(self.num_steps))
        observation = None

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = self.rewardFunction()

        info = self.info

        return observation, reward, termination, info
    
    def is_success(self):
        """Success definition for task
        """
        if (self.object_interface.position[2] > 0.12):
            return True
        else:
            return False

    def is_done(self):
        return self._check_termination()

    def rewardFunction(self):
        """ Sparse reward only if successful
        """

        if self.is_success():
            return 1.0
        else:
            return 0.0