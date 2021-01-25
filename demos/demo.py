"""Class definition for demonstration.
"""
from demo_control_env import DemoControlEnv
import numpy as np
import time
from datetime import datetime

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
                 config_file="demo_control_cfg.yaml",
                 **kwargs):
        self.ctrl_type = ctrl_type

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

    # def make_demo(**kwargs):
    #     """Factory method for making the write demo per params.
    #     """
    #     if kwargs['ctrl_type'] in ["EEImpedance", "EEPosture"]:
    #         return OpSpaceDemo(**kwargs)
    #     elif kwargs['ctrl_type'] in ["JointImpedance", "JointTorque", "JointVelocity"]:
    #         return JointSpaceDemo(**kwargs)
    #     else:
    #         raise ValueError("invalid demo / controller pairing.")