"""Class definition for JointSpace controller demonstration.
"""

from demo_envs import JointDemoEnv
from demo_path import SequentialJoint
from demo import Demo
import numpy as np
import logging
logger = logging.getLogger(__name__)
import time 

MAX_JOINT_DELTA = 0.005

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
                 config_file="demo_control_cfg.yaml",
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
        
        self.env = JointDemoEnv(config=config_file,
                                use_visualizer=True,
                                name=None)

        self.joint_num = kwargs['joint_num']

        # Joint torque requires edge case to not include gravity comp.
        if ctrl_type == "JointTorque":
            self.init_joint_state = [0]*7;
        else:
            self.init_joint_state = self.get_state()
        
        if (delta_val > MAX_JOINT_DELTA):
            logger.warn("Specified joint delta_val exceeds limit, clipping to {} rad".format(MAX_JOINT_DELTA))
            delta_val = MAX_JOINT_DELTA

        self.delta_val = delta_val 
        self.num_steps = num_steps

        self.path = SequentialJoint(start_pose=self.init_joint_state,
                                    delta_val=self.delta_val,
                                    joint_num=self.joint_num, 
                                    num_steps=self.num_steps)
        
        self.goal_poses = self.path.path


        self.step_num = 0

        self.connect_and_reset_robot()
    
    def get_state(self):
        if self.ctrl_type == "JointTorque":
            return self.env.robot_interface.tau[:7]
        elif self.ctrl_type == "JointImpedance":
            return self.env.robot_interface.q[:7]
        elif self.ctrl_type == "JointVelocity":
            return self.env.robot_interface.dq[:7]
        else:
            raise ValueError("Invalid ctrl type.")

    def run(self):
        """Run the demo. Execute actions in sequence and calculate error.
        """
        logger.info("Running {} demo \n with control type {}.\n Test \
            function {} on Joint Num {}".format(self.demo_type, self.ctrl_type,  self.test_fn, self.joint_num))

        logger.debug("Initial joint pose \n{}".format(self.env.robot_interface.q))

        input("Press enter to confirm and begin running demo.")

        for i, goal_pose in enumerate(self.goal_poses):
            action = self.get_action(goal_pose, self.get_state())
            action_kwargs = self.get_action_kwargs(action)
            logger.debug("Action for joint num {} \t{}".format(self.joint_num, action[self.joint_num]))

            self.env.step(action_kwargs, time.time())
            self.actions.append(action)
            new_state = self.get_state()

            self.states.append(new_state)
            self.errors.append(
                self.compute_error(goal_pose, new_state))
            #logger.debug("Errors:\t{}".format(self.errors[-1]))

        logger.debug("Final (actual) joint pose \n{}".format(self.env.robot_interface.q))

        self.env.robot_interface.reset()
        self.env.robot_interface.disconnect()
        if self.plot_error:
            self.plot_errors()
        if self.plot_pos:
            self.plot_positions()

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
            action_kwargs['pose'] = action
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
        elif self.test_fn == "set_joint_positions":
            action = goal_state
        elif self.test_fn == "set_joint_torques":
            action = goal_state
        elif self.test_fn == "set_joint_velocities":
            action = goal_state
        else:
            raise ValueError("Invalid test function")

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

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description="Test controllers and measure errors.")
    parser.add_argument('--ctrl_type',
                        default=None,
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
    demo = JointSpaceDemo(**kwargs)
    demo.run()