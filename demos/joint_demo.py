"""Class definition for JointSpace controller demonstration.
"""

from demo_envs import JointDemoEnv
from demo_path import RampSingleJoint
from demo import Demo
import numpy as np
import logging
logger = logging.getLogger(__name__)
import time

import matplotlib.pyplot as plt
# hide verbose matplotlib logging
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING)

MAX_JOINT_DELTA = 0.005
NUM_JOINTS = 6

class JointSpaceDemo(Demo):

    """Class definition for joint space demonstrations.

    Joint space demonstrations are 7 dof for each motor.
    They include joint positions, velocities and torques.

    Attributes:
        env (DemoControlEnv): environment to step with action generated
            by demo.
        ctrl_type (str): string identifying type of controller:
            JointImpedance, JointVelocity or JointTorque
        demo_type (str): type of demo to perform (sequential only for JointSpace)
        num_steps (int): number of steps in the demo.
        test_fn (str): RobotInterface function to test.
        delta_val (float): delta joint angle (rad) between each desired
            joint position in the demo.
        kwargs (dict): catch all other keyword arguments.
    """
    def __init__(self,
                 ctrl_type,
                 demo_type,
                 config_file="demos/demo_control_cfg.yaml",
                 delta_val=0.005, num_steps=30,
                 test_fn='set_joint_delta',
                 **kwargs):
        """ Initial Joint Space demo and calculate joint trajectory to take.

        Args:
            config_file (str): filepath of the config file for the environment.

        """
        super().__init__(ctrl_type=ctrl_type,
                         demo_type=demo_type,
                         test_fn=test_fn, **kwargs)

        self.env = JointDemoEnv(config=self.config,
                                use_visualizer=True,
                                name=None,
                                test_fn=self.test_fn,
                                ctrl_type=self.ctrl_type)

        self.delta_val = self.clip_delta(delta_val)
        self.num_steps = num_steps

    def get_state(self):
        if self.ctrl_type == "JointTorque":
            return np.subtract(self.env.robot_interface.tau[:7], self.env.robot_interface.gravity_vector)
        elif self.ctrl_type == "JointImpedance":
            return self.env.robot_interface.q[:7]
        elif self.ctrl_type == "JointVelocity":
            return self.env.robot_interface.dq[:7]
        else:
            raise ValueError("Invalid ctrl type.")

    def get_delta(self, goal_pose, current_pose):
        """Get delta between joint space poses.
        """
        return np.subtract(goal_pose, current_pose)

    def get_action_kwargs(self, action):
        """
        Args:
            action (list): action to be converted in kwargs for robot_interface functions.
        Returns
            action_kwargs (dict): dictionary with key-value pairs corresponding to
                arguments for robot_interface command being tested.
        """
        action_kwargs = {}
        if isinstance(action, np.ndarray):
            action = action.tolist()

        if self.test_fn == "set_joint_delta":
            action_kwargs['delta'] = action
            action_kwargs['set_qpos'] = None
        if self.test_fn == "set_joint_positions":
            action_kwargs['delta'] = None
            action_kwargs['set_qpos'] = action
        if self.test_fn == "set_joint_torques":
            action_kwargs['torques'] = action
        if self.test_fn == "set_joint_velocities":
            action_kwargs['velocities'] = action

        return action_kwargs

    def get_action(self, goal_state, current_state):
        """Get action corresponding to test_fn

        Args:
            goal_pose (list): 7f desired joint positions
            current_pose (list): 7f current joint positions.

        Return:
            action (list): 7f joint delta or joint positions.
        """
        if self.test_fn == "set_joint_delta":
            action = np.subtract(goal_state, current_state)
        else:
            action = goal_state


        return action

    def compute_error(self, goal, current):
        return np.subtract(goal, current)

    def plot_positions(self):
        """ Plot 6 plots for joint position.
        Helps for visualizing decoupling.
        """

        goal_0 = [goal[0] for goal in self.goal_poses]
        goal_1 = [goal[1] for goal in self.goal_poses]
        goal_2 = [goal[2] for goal in self.goal_poses]
        goal_3 = [goal[3] for goal in self.goal_poses]
        goal_4 = [goal[4] for goal in self.goal_poses]
        goal_5 = [goal[5] for goal in self.goal_poses]
        goal_6 = [goal[6] for goal in self.goal_poses]

        state_0 = [state[0] for state in self.states]
        state_1 = [state[1] for state in self.states]
        state_2 = [state[2] for state in self.states]
        state_3 = [state[3] for state in self.states]
        state_4 = [state[4] for state in self.states]
        state_5 = [state[5] for state in self.states]
        state_6 = [state[6] for state in self.states]

        fig, ((ax_0, ax_1, ax_2, ax_3), (ax_4, ax_5, ax_6, ax_7)) = plt.subplots(2, 4)

        ax_0.plot(goal_0, 'b')
        ax_1.plot(goal_1, 'b')
        ax_2.plot(goal_2, 'b')
        ax_3.plot(goal_3, 'b')
        ax_4.plot(goal_4, 'b')
        ax_5.plot(goal_5, 'b')
        ax_6.plot(goal_6, 'b')

        ax_0.plot(state_0, 'r')
        ax_1.plot(state_1, 'r')
        ax_2.plot(state_2, 'r')
        ax_3.plot(state_3, 'r')
        ax_4.plot(state_4, 'r')
        ax_5.plot(state_5, 'r')
        ax_6.plot(state_6, 'r')

        fname = self.demo_name + '_pos.png'

        if self.save_fig:
            print("saving figure")
            plt.savefig(fname)

        plt.show()


    def plot_errors(self):
        """ Plot 6 plots showing errors for each dimension.
        x, y, z and qx, qy, qz euler angles (from C.orientation_error)
        """
        error_0 = [error[0] for error in self.errors]
        error_1 = [error[1] for error in self.errors]
        error_2 = [error[2] for error in self.errors]
        error_3 = [error[3] for error in self.errors]
        error_4 = [error[4] for error in self.errors]
        error_5 = [error[5] for error in self.errors]
        error_6 = [error[6] for error in self.errors]

        fig, ((e_0, e_1, e_2, e_3), (e_4, e_5, e_6, e_7)) = plt.subplots(2, 4)

        e_0.plot(error_0, 'r')
        e_1.plot(error_1, 'r')
        e_2.plot(error_2, 'r')
        e_3.plot(error_3, 'r')
        e_4.plot(error_4, 'r')
        e_5.plot(error_5, 'r')
        e_6.plot(error_6, 'r')

        fname = self.demo_name + '_error.png'
        if self.save_fig:
            plt.savefig(fname)
        plt.show()

    def clip_delta(self, delta_val):
        """Clip delta value to max, preserving sign.
        """
        return delta_val


class SingleJointDemo(JointSpaceDemo):
    """Joint Space demonstration for a single joint.
    """
    def __init__(self,
                 config_file="demos/demo_control_cfg.yaml",
                 delta_val=0.005, num_steps=30,
                 test_fn='set_joint_delta',
                 joint_num=6,
                 **kwargs):
        super().__init__(ctrl_type="JointImpedance",
                         demo_type="SingleJoint",
                         config_file=config_file,
                         delta_val=delta_val,
                         num_steps=num_steps,
                         test_fn=test_fn,
                         **kwargs)

        self.joint_num = joint_num
        self.connect_and_reset_robot()
        self.init_joint_state = self.get_state()

        self.path = RampSingleJoint(start_pose=self.init_joint_state,
                                    delta_val=self.delta_val,
                                    joint_num=self.joint_num,
                                    num_steps=self.num_steps)

        self.goal_poses = self.path.path

        self.step_num = 0

    def clip_delta(self, delta_val):
        """Clip delta to max value for joint impedance control.

        """
        if (np.abs(delta_val) > MAX_JOINT_DELTA):
            logger.warn("Specified joint delta_val exceeds limit, clipping to {} rad".format(MAX_JOINT_DELTA))
            delta_val = np.sign(delta_val) * MAX_JOINT_DELTA
        return delta_val


class GravityCompDemo(JointSpaceDemo):
    """Gravity Compensation to demonstrate "floating" arm.
    """
    def __init__(self,
                 config_file="demos/demo_control_cfg.yaml",
                 num_steps=100,
                 **kwargs):

        super().__init__(ctrl_type="JointTorque",
                         demo_type="GravityCompensation",
                         config_file=config_file,
                         delta_val=0.0,
                         num_steps=num_steps,
                         test_fn="set_joint_torques",
                         **kwargs)

        self.connect_and_reset_robot()
        self.init_joint_state = np.zeros(7)
        self.path = RampSingleJoint(start_pose=self.init_joint_state,
                                    delta_val=self.delta_val,
                                    joint_num=0,
                                    num_steps=self.num_steps)

        self.goal_poses = self.path.path

        self.step_num = 0

    def print_demo_info(self):
        print("\n\t Running Gravity compensation demo")

    def get_action(self, goal_state, current_state):
        return np.zeros(7)


class JointImpDemoSeq(SingleJointDemo):
    def __init__(self,
             config_file="demos/demo_control_cfg.yaml",
             delta_val=0.005, num_steps=30,
             test_fn='set_joint_delta',
             **kwargs):
        super().__init__(config_file=config_file,
                         delta_val=delta_val,
                         num_steps=num_steps,
                         test_fn=test_fn,**kwargs)

    def print_demo_info(self):
        print("\n\tRunning Joint Impedance demo for all joints \n with test_function: {}".format(self.test_fn))
        print("Robot will perform 6 demonstrations: \n Moving joints 0-6 (base to end-effector) individually. \nRobot will reset before continuing to next demo.")


    def run_single_joint_demo(self, joint_num):
        self.reset_log()
        self.joint_num = joint_num
        self.path = RampSingleJoint(start_pose=self.init_joint_state,
                                    delta_val=self.delta_val,
                                    joint_num=self.joint_num,
                                    num_steps=self.num_steps)
        self.goal_poses = self.path.path
        self.step_num = 0

        print("------------------------")
        print("Running Joint Impedance Demo on joint num {} \n \
            \nTest function {}".format(
            self.joint_num, self.ctrl_type,  self.test_fn))
        logger.debug("EE Pose initial:\n{}\n".format(self.env.robot_interface.ee_pose))
        print("--------")
        input("Press Enter to start sending commands.")
        self.step_through_demo()
        self.env.robot_interface.reset()
        if self.plot_error:
            self.plot_errors()
        if self.plot_pos:
            self.plot_positions()

        if self.save:
            self.save_data()

    def run(self):
        """Run the demo. Execute actions in sequence and calculate error.
        """
        self.print_demo_banner()
        self.print_demo_info()

        # Run joint impedance demos for each joint
        for joint_index in range(NUM_JOINTS+1):
            self.run_single_joint_demo(joint_index)

        print("Demo complete...disconnecting robot.")
        self.env.robot_interface.disconnect()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description="Test controllers and measure errors.")
    parser.add_argument('--ctrl_type',
                        default="JointImpedance",
                        help='Type of controller to test')
    parser.add_argument('--demo_type',
                        default=None,
                        help='Type of menu to run.')
    parser.add_argument('--test_fn',
                        default='set_joint_delta',
                        help='Function to test',
                        choices=['set_joint_delta', 'set_joint_positions', 'set_joint_torques', 'set_joint_velocities'])
    parser.add_argument('--delta_val',
                        default=0.001, type=float,
                        help="Max step size (rad) to take for demo.")
    parser.add_argument('--joint_num',
                        default=6, type=int,
                        help='joint index to test for Joint space demos')
    parser.add_argument('--num_steps', default=50, type=int,
                        help="max steps for demo.")
    parser.add_argument('--plot_pos', action="store_true",
                        help="whether to plot positions of demo.")
    parser.add_argument('--plot_error', action="store_true",
                        help="whether to plot errors.")
    parser.add_argument('--save', action="store_true",
                        help="whether to store data to file")
    parser.add_argument('--demo_name', default=None,
                        type=str, help="Valid filename for demo.")
    parser.add_argument('--save_fig', action="store_true",
                        help="whether to save pngs of plots")
    parser.add_argument('--config_file', default='demos/demo_control_cfg.yaml', help='absolute filepath for config file.')

    args = parser.parse_args()
    kwargs = vars(args)
    demo = SingleJointDemo(**kwargs)
    demo.run()