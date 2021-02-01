"""Class definition for demonstration.
"""
import numpy as np
import time
from datetime import datetime
import logging
logger = logging.getLogger(__name__)

import json
from perls2.utils.yaml_config import YamlConfig
class Demo(object):
    """Abstract class definition for demonstration.
    Demonstrations step an environment according to a series of actions
    that follow a specified pattern and are of appropriate dimension for the controller.

    Attributes:
        env (DemoControlEnv): environment to step with action generated
            by demo.
        ctrl_type (str): string identifying type of controller:
            EEPosture, EEImpedance, JointImpedance, JointTorque
        demo_type (str): type of demo to perform (zero, line, sequential,
            square.)
        test_fn (str): RobotInterface function to test.
        use_abs (bool): Use absolute values for poses or goal states.
        demo_name (str): name of the demo control environment.
        plot_pos (bool): flag to plot positions of states during demo.
        plot_error (bool): flag to plot errors of controllers during demo.
        save (bool): flag to save states, errors, actions of demo as npz.
        world_type: Type of world Bullet or Real.
        initial_pose (list): initial end-effector pose.

    """
    def __init__(self,
                 ctrl_type,
                 demo_type,
                 test_fn,
                 config_file="demos/demo_control_cfg.yaml",
                 **kwargs):
        self.ctrl_type = ctrl_type
        self.config_file = config_file

        # Overwrite config file controlType
        self.config = YamlConfig(self.config_file)
        self.config['world']['controlType'] = self.ctrl_type
        # Use command line argument over the config spec. 
        if kwargs['world'] is not None: 
            self.config['world']['type'] = kwargs['world']

        self.demo_type = demo_type
        self.test_fn = test_fn

        self.plot_pos = kwargs['plot_pos']
        self.plot_error = kwargs['plot_error']
        self.save_fig = kwargs['save_fig']

        self.save = kwargs['save']

        self.demo_name = kwargs['demo_name']
        if self.demo_name is None:
            now = datetime.now()
            demo_name = now.strftime("%d%m%y%H%M%S")
            self.demo_name = "{}_{}_{}".format(demo_name, self.ctrl_type, self.demo_type)

        # self.axis = kwargs['axis']


        # Initialize lists for storing data.
        self.errors = []
        self.actions = []
        self.states = []
        self.observations = []

    @property
    def world_type(self):
        return self.env.config['world']['type']

    def get_action_list(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError

    def save_data(self):
        fpath = "demos/results/{}.npz".format(self.demo_name)
        np.savez(fpath, states=self.states,
                 errors=self.errors, actions=self.actions,
                 goals=self.goal_states, obs=self.observations, allow_pickle=True)

    def get_goal_state(self, delta):
        raise NotImplementedError

    def connect_and_reset_robot(self):
        # Connect to redis and reset robot.
        if not self.env.world.is_sim:
            logger.info("connecting perls2 Robot Interface to redis")
            self.env.robot_interface.connect()

        self.env.robot_interface.reset()

    def print_demo_banner(self):
        print("\n**************************************************************************************")
        print("***********************   perls2 Controller Demo    ************************************")
        self.print_controller_params()

    def print_demo_info(self):
        print("\n\t Running {} demo \n\t with control type {}. \
            \n\t Test function {}".format(
            self.ctrl_type, self.demo_type, self.test_fn))

    def print_controller_params(self):
        print("\n Config loaded from: {}\n".format(self.config_file))
        print("\n {} Controller parameters:\n {}\n".format(self.ctrl_type,
            json.dumps(self.env.robot_interface.controller_cfg[self.world_type][self.ctrl_type], indent=4)))

    def run(self):
        """Run the demo. Execute actions in sequence and calculate error.
        """
        self.print_demo_banner()
        self.print_demo_info()
        logger.debug("EE Pose initial:\n{}\n".format(self.env.robot_interface.ee_pose))
        print("--------")
        input("Press Enter to start sending commands.")
        self.step_through_demo()
        self.env.robot_interface.reset()
        self.env.robot_interface.disconnect()

        if self.plot_error:
            self.plot_errors()
        if self.plot_pos:
            self.plot_positions()

        if self.save:
            self.save_data()

    def step_through_demo(self):
        for i, goal_pose in enumerate(self.goal_poses):

            # Get action corresponding to test_fn and goal pose
            action= self.get_action(goal_pose, self.get_state())
            action_kwargs = self.get_action_kwargs(action)

            # Step environment forward
            obs, reward, done, info = self.env.step(action_kwargs, time.time())
            self.observations.append(obs)
            self.actions.append(action)
            new_state = self.get_state()
            self.states.append(new_state)
            self.errors.append(
                self.get_delta(goal_pose, new_state))

    def reset_log(self):
        """Reset states, actions and observation logs between demos.
        """
        self.states = []
        self.errors = []
        self.actions = []
        self.observations = []

    # def make_demo(**kwargs):
    #     """Factory method for making the write demo per params.
    #     """
    #     if kwargs['ctrl_type'] in ["EEImpedance", "EEPosture"]:
    #         return OpSpaceDemo(**kwargs)
    #     elif kwargs['ctrl_type'] in ["JointImpedance", "JointTorque", "JointVelocity"]:
    #         return JointSpaceDemo(**kwargs)
    #     else:
    #         raise ValueError("invalid demo / controller pairing.")