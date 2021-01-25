"""Environment for Controller Demonstrations

These environments step actions as specified by a test function, 
with the appropriate keyword arguments for said test function.
"""

from perls2.envs.env import Env
import time

VALID_JOINTSPACE_FNS = ["set_joint_delta", 
                              "set_joint_positions",
                              "set_joint_velocities",
                              "set_joint_torques"]

VALID_JOINTSPACE_CTLRS = ["JointImpedance", 
                         "JointVelocity", 
                         "JointTorque"]

VALID_OSC_FNS = ["move_ee_delta", "set_ee_pose"]
VALID_OSC_CTLRS = ["EEImpedance", "EEPosture"]

class DemoEnv(Env):
    def __init__(self,
                 config,
                 use_visualizer,
                 name,
                 test_fn):
        super().__init__(config, use_visualizer, name)
        self.test_fn = test_fn

    def reset(self):
        """ Reset world and robot and return observation.
        """
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()

        observation = self.get_observation()    

    def get_observation(self):
        """Return observation as dict
        """
        obs = {}
        obs['ee_pose'] = self.robot_interface.ee_pose
        obs['q'] = self.robot_interface.q
        obs['dq'] = self.robot_interface.dq

        # optional image if camera available.
        if self.world.has_camera:
            obs['rgb'] = self.camera_interface.frames()['rgb']

        return obs

    def rewardFunction(self):
        return None


class JointDemoEnv(DemoEnv):
    """Class for Joint Space Controller Demonstrations
    """
    def __init__(self,
                 config,
                 use_visualizer=False,
                 name=None, 
                 test_fn="set_joint_delta",
                 ctrl_type="JointImpedance"):
        """Initialize.

        Args:
            config (str, dict): A relative filepath to the config file. Or a
                parsed YamlConfig file as a dictionary.
                e.g. 'cfg/my_config.yaml'
            use_visualizer (bool): A flag for whether or not to use visualizer
            name (str): of the environment
            test_fn (str): name of robot interface function to use for demo.

                See documentation for more details about config files.
        """  
        super().__init__(config, use_visualizer, name, test_fn)

        if not test_fn in VALID_JOINTSPACE_FNS:
            raise ValueError("Invalid test function for Joint space demo")

        if not ctrl_type in VALID_JOINTSPACE_CTLRS:
            raise ValueError("Invalid controller for Joint Space Demo.")
        self.ctrl_type = ctrl_type
        self.robot_interface.change_controller(self.ctrl_type)
    

    def _exec_action(self, action_kw):
        """Applies the given action to the simulation.

            Args: action_kw (dict): dictionary of commands specific to test_fn to
                execute action.

            actions are usually specified as a list, but in for demo purposes it's 
            easier to add the relevant kwargs directly for the controller.
        """
        if self.robot_interface.controlType == 'JointVelocity':
            self.robot_interface.set_joint_velocities(**action_kw)
        elif self.robot_interface.controlType == 'JointImpedance':
            if self.test_fn == 'set_joint_delta':
                self.robot_interface.set_joint_delta(**action_kw)
            elif self.test_fn == 'set_joint_positions':
                self.robot_interface.set_joint_positions(**action_kw)
            else:
                raise ValueError("invalid test function.")
        elif self.robot_interface.controlType == 'JointTorque':
            if self.test_fn == 'set_joint_torques':
                self.robot_interface.set_joint_torques(**action_kw)
            else:
                raise ValueError("invalid test function.")
        else:
            raise ValueError("Invalid controller.")
        self.robot_interface.action_set = True


class OSCDemoEnv(DemoEnv):
    """Environment for Operational Space Control Demos.
    """
    def __init__(self,                 
                 use_visualizer=False,
                 name=None, 
                 test_fn="set_ee_pose",
                 ctrl_type="EEImpedance"):
        super().__init__(config, use_visualizer, name)

