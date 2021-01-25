from demo_control_env import DemoControlEnv
import numpy as np
import time


from datetime import datetime
import argparse

import matplotlib.pyplot as plt
import logging
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

import perls2.controllers.utils.control_utils as C
import perls2.controllers.utils.transform_utils as T

AXIS_DIM_NUM = {'x': 0, 'y': 1, 'z': 2}
MAX_JOINT_DELTA = 0.005

class Demo():
    """Class definition for demonstration.
    Demonstrations are a series of actions that follow a specified pattern
    and are of appropriate dimension for the controller.

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
    def __init__(self, config_file, ctrl_type, demo_type, test_fn,  use_abs=True, **kwargs):
        self.env = DemoControlEnv(config=config_file,
                                  use_visualizer=True,
                                  use_abs=use_abs,
                                  test_fn=test_fn,
                                  name='Demo Control Env')
        self.ctrl_type = ctrl_type

        self.env.robot_interface.change_controller(self.ctrl_type)
        self.demo_type = demo_type
        self.use_abs = use_abs
        self.test_fn = test_fn
        self.axis = kwargs['axis']
        self.plot_pos = kwargs['plot_pos']
        self.plot_error = kwargs['plot_error']
        self.save_fig = kwargs['save_fig']
        self.joint_num = kwargs['joint_num']

        self.save = kwargs['save']
        self.demo_name = kwargs['demo_name']
        if self.demo_name is None:
            self.demo_name = "{}_{}_{}".format("0915",self.ctrl_type, self.demo_type)

        # Initialize lists for storing data.
        self.errors = []
        self.actions = []
        self.states = []
        self.observations = []
        self.world_type = self.env.config['world']['type']
        self.initial_pose = self.env.robot_interface.ee_pose

        if self.env.world.is_sim == False:
            logger.info("connecting perls2 Robot Interface to redis")
            self.env.robot_interface.connect()
        self.env.robot_interface.reset()

    def get_action_list(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError

    def save_data(self):
        fpath = "dev/validation/{}.npz".format(self.demo_name)
        np.savez(fpath, states=self.states,
                 errors=self.errors, actions=self.actions,
                 goals=self.goal_states, obs=self.observations, allow_pickle=True)

    def get_goal_state(self, delta):
        raise NotImplementedError

    def make_demo(**kwargs):
        """Factory method for making the write demo per params.
        """
        if kwargs['ctrl_type'] in ["EEImpedance", "EEPosture"]:
            return OpSpaceDemo(**kwargs)
        elif kwargs['ctrl_type'] in ["JointImpedance", "JointTorque", "JointVelocity"]:
            return JointSpaceDemo(**kwargs)
        else:
            raise ValueError("invalid demo / controller pairing.")


class JointSpaceDemo(Demo):

    """Class definition for joint space demonstrations.

    Joint space demonstrations are 7 dof for each motor.
    They include joint positions, velocities and torques.

    Attributes:
        ctrl_type (str): string identifying type of controller:
            EEPosture, EEImpedance, JointImpedance, JointTorque
        demo_type (str): type of demo to perform (zero, line, sequential,
            square.)
        use_abs (bool): Use absolute values for poses or goal states.
        num_steps (int): number of steps in the demo.
        test_fn (str): RobotInterface function to test.
        delta_val (float): magnitude of delta from current position.
            Absolute joint space demos are also determined by adding
            this delta to current joint position.
    """
    def __init__(self, ctrl_type, demo_type, use_abs=False,
                 delta_val=0.05, num_steps=30, test_fn='set_joint_delta',
                 **kwargs):
        super().__init__(ctrl_type=ctrl_type,
                         demo_type=demo_type,
                         use_abs=use_abs, test_fn=test_fn, **kwargs)
        # Get goal states based on demo and control type.

        # Joint torque requires edge case to not include gravity comp.
        if ctrl_type == "JointTorque":
            self.start_pos = [0, 0, 0, 0, 0, 0, 0]
        else:
            self.start_pos = self.get_state() #self.env.robot_interface.q
        
        if (delta_val > MAX_JOINT_DELTA):
            logger.warn("Specified joint delta_val exceeds limit, clipping to {} rad".format(MAX_JOINT_DELTA))
            delta_val = MAX_JOINT_DELTA

        self.delta_val = delta_val 
        self.path = SequentialJoint(start_pose=self.start_pos,
                                    delta_val=self.delta_val,
                                    joint_num=self.joint_num)
        self.goal_poses = self.path.path
        self.num_steps = len(self.goal_poses)
        self.step_num = 0

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

class JointSpaceDeltaDemo(JointSpaceDemo):

    def get_goal_states(self):
        goal_states = [self.start_pos]
        for action in self.action_list:
            goal_states.append(np.add(goal_states[-1], action))
        return goal_states

    def get_action_list(self):
        """ Get action list for random demo.

            Each action will move only one joint in a random direction
            at a time. the magnitude of each action is determined by
            the delta_val.

            e.g.
            [0, -0.05, 0, 0, 0, 0]

        """
        # Joint position delta demo.

        NUM_JOINTS = 7
        joint_position_delta_demo = [np.zeros(NUM_JOINTS)]
        # Move each joint individually by +0.05 radians
        for joint_num in range(NUM_JOINTS):
            delta = np.zeros(NUM_JOINTS)
            delta[joint_num] = self.delta_val
            joint_position_delta_demo.append(delta)

        # Move each joint individually by -0.05 radians
        for joint_num in range(NUM_JOINTS):
            delta = np.zeros(NUM_JOINTS)
            delta[-1 - joint_num] = -self.delta_val
            joint_position_delta_demo.append(delta)

        return joint_position_delta_demo


class JointSpaceAbsDemo(JointSpaceDeltaDemo):
    def __init__(self, ctrl_type, demo_type, start_pos, use_abs=True,):
        self.start_pos = start_pos
        super().__init__(ctrl_type, demo_type, use_abs)

    def get_action_list(self):
        """Add actions to joint position and update to get
        absolute positions at each step.
        """

        jp_delta_demo = super().get_action_list()
        jp_abs_demo = []

        for action in jp_abs_demo:
            jp_abs_demo.append(self.states, jp_delta_demo)
            self.start_pos = jp_abs_demo[-1]
        return jp_abs_demo


class OpSpaceDemo(Demo):
    """ Demos for testing operational space control. These include
    End-Effector Imedance, and End-Effector with nullspace posture control.
    """
    def __init__(self, ctrl_type, demo_type, use_abs=False,
                 path_length=0.15, delta_val=None, num_steps=50, test_fn='set_ee_pose',
                 fix_ori=True, fix_pos=False, cycles=1,**kwargs):
        """
        ctrl_type (str): string identifying type of controller:
            EEPosture, EEImpedance, JointImpedance, JointTorque
        demo_type (str): type of demo to perform (zero, line, sequential,
            square.)
        """
        super().__init__(ctrl_type=ctrl_type, demo_type=demo_type, use_abs=use_abs,
                         test_fn=test_fn, **kwargs)

        self.initial_pose = self.env.robot_interface.ee_pose
        self.path_length = path_length
        self.num_steps = num_steps
        if delta_val is None:
            if self.path_length is not None:
                self.delta_val = self.path_length / self.num_steps
        self.delta_val = delta_val

        self.goal_poses= self.get_goal_poses(cycles)
        self.goal_states = self.get_goal_states(cycles)


        self.fix_ori = fix_ori
        self.fix_pos = fix_pos


    def run(self):
        """Run the demo. Execute actions in sequence and calculate error.
        """
        logger.info("Running {} demo \nwith control type {}. \
            \nTest function {}".format(
            self.ctrl_type, self.demo_type, self.test_fn))

        logger.debug("EE Pose initial:\n{}\n".format(self.env.robot_interface.ee_pose))
        logger.info("--------")
        input("Press Enter to start sending commands.")
        # if self.env.world.is_sim == False:
        #     self.env.robot_interface.connect()
        # Execute actions based on goal poses
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
                self.compute_error(goal_pose, new_state))
            #input("Press Enter to continue")
        self.env.robot_interface.reset()
        self.env.robot_interface.disconnect()
        if self.plot_error:
            self.plot_errors()
        if self.plot_pos:
            self.plot_positions()

        if self.save:
            self.save_data()

    def get_action(self, goal_pose, current_pose):
        """ Return action corresponding to goal and test_fn

        Args:
            goal_pose (list): 7f position and quaternion of goal pose
            current_pose (list): 7f position and quaternion of current
                pose.

        Returns:
            action (list): action corresponding to robot_interface
                function being tested as a list.

        Notes:
            - If test function is "move_ee_delta" returns
                (list 6f) [dx, dy, dz, ax, ay, az] with
                delta position and delta orientation as axis-angle.
            - If test function is "set_ee_pose" returns (list 6f)
                [x,y,z,qx,qy,qz, w] absolute position and orientation
                as quaternion.
        """
        if self.test_fn =="move_ee_delta":
            action = get_delta(goal_pose, current_pose)
        elif self.test_fn =="set_ee_pose":
            action = goal_pose
        else:
            raise ValueError("invalid test fn")
        return action

    def get_action_kwargs(self, action):
        """Return dict of kwargs specific to function being tested.

        Args:
            action (list): action to be converted in kwargs for robot_interface functions.
        Returns
            action_kwargs (dict): dictionary with key-value pairs corresponding to
                arguments for robot_interface command being tested.
        """
        action_kwargs  = {}
        if self.test_fn =="move_ee_delta":
            action_kwargs['delta'] = action
            action_kwargs['set_pos'] = None
            action_kwargs['set_ori'] = None
            if self.fix_ori:
                action_kwargs['set_ori'] = self.initial_pose[3:]
            if self.fix_pos:
                action_kwargs['set_pos'] = self.initial_pose[:3]
        if self.test_fn=="set_ee_pose":
            action_kwargs['delta'] = None
            action_kwargs['set_pos'] = action[:3]
            action_kwargs['set_ori'] = action[3:]


        return action_kwargs

    def get_action_list(self):
        """Get the set of actions based on the type of the demo,
        """

        if self.demo_type == "Zero":
            self.path = Line(start_pose=self.initial_pose,
                             num_pts=self.num_steps,
                             delta_val=0,
                             path_length=0)

        elif self.demo_type == "Square":
            self.path = Square(start_pose=self.initial_pose,
                               side_num_pts=int(self.num_steps/4),
                               delta_val=self.delta_val)
        elif self.demo_type == "Line":
            self.path = Line(start_pose=self.initial_pose,
                             num_pts=self.num_steps,
                             delta_val=self.delta_val,
                             path_length=self.path_length,
                             dim=AXIS_DIM_NUM[self.axis])

        elif self.demo_type == "Rotation":
            self.path = Rotation(
                start_pose=self.initial_pose,
                num_pts=self.num_steps,
                delta_val=self.delta_val,
                dim=AXIS_DIM_NUM[self.axis])
        else:
            raise ValueError("Invalid Demo type")

        if self.test_fn=="set_ee_pose":
            action_list = self.path.path
        elif self.test_fn=="move_ee_delta":
            action_list = self.path.deltas
        else:
            raise ValueError("Invalid test_fn")
        return action_list

    def get_goal_poses(self, repeat=1):
        """ Get a list of absolute end_effector states

        Args:
            repeat (int): optional argument for square paths to repeat. 
        """
        if self.demo_type == "Zero":
            self.path = Line(start_pose=self.initial_pose,
                             num_pts=self.num_steps,
                             delta_val=0,
                             path_length=0)
        elif self.demo_type == "Square":
            self.path = Square(start_pose=self.initial_pose,
                               side_num_pts=int(self.num_steps/4),
                               delta_val=self.delta_val)
            # repeat path only for square

        elif self.demo_type == "Line":
            self.path = Line(start_pose=self.initial_pose,
                             num_pts=self.num_steps,
                             delta_val=self.delta_val,
                             path_length=self.path_length,
                             dim=AXIS_DIM_NUM[self.axis])
        elif self.demo_type == "Rotation":
            self.path = Rotation(
                start_pose=self.initial_pose,
                num_pts=self.num_steps,
                delta_val=self.delta_val,
                dim=AXIS_DIM_NUM[self.axis])
        else:
            raise ValueError("Invalid Demo type")

        goal_poses = self.path.path
        # repeat path only for square
        if self.demo_type == "Square":
            goal_poses = goal_poses * repeat
        return goal_poses

    def get_goal_states(self, repeat=1):
        """ Get goal states based on the demo type and action list.

        For Zero goal states with use_abs, goal states are just
        copied initial end-effector pose.
        """
        if self.path is not None:
            goal_states = self.path.path

        else:
            raise ValueError("Get actions list before goal states")
        
        # special case to handle repeat for square paths.
        if self.demo_type == "Square":
            goal_states = goal_states * repeat
        return goal_states

    def get_state(self):
        """ Proprio state for robot we care about for this demo: ee_pose.
        Used to compute error.
        """
        return self.env.robot_interface.ee_pose

    def compute_error(self, goal_state, new_state):
        """ Compute the error between current state and goal state.
        For OpSpace Demos, this is position and orientation error.
        """
        goal_pos = goal_state[:3]
        new_pos = new_state[:3]
        # Check or convert to correct orientation
        goal_ori = T.convert_euler_quat_2mat(goal_state[3:])
        new_ori = T.convert_euler_quat_2mat(new_state[3:])

        pos_error = np.subtract(goal_pos, new_pos)
        ori_error = C.orientation_error(goal_ori, new_ori)

        return np.hstack((pos_error, ori_error))

    def plot_sequence_arrows(self, axes, x, y, color):
        """Plot a sequence of arrows given a list of x y points.
        """
        for pt_idx in range(len(x)-1):
            dx = np.subtract(x[pt_idx+1], x[pt_idx])
            dy = np.subtract(y[pt_idx+1], y[pt_idx])

            axes.arrow(x[pt_idx], y[pt_idx], dx, dy, color=color)

    def plot_positions(self):
        """ Plot 3 plots showing xy, xz, and yz position.
        Helps for visualizing decoupling.
        """

        goal_x = [goal[0] for goal in self.goal_states]
        goal_y = [goal[1] for goal in self.goal_states]
        goal_z = [goal[2] for goal in self.goal_states]

        state_x = [state[0] for state in self.states]
        state_y = [state[1] for state in self.states]
        state_z = [state[2] for state in self.states]

        fig, (ax_xy, ax_xz, ax_yz) = plt.subplots(1, 3)

        ax_xy.plot(goal_x, goal_y, 'or')
        ax_xy.plot(state_x, state_y, '*b')
        # self.plot_sequence_arrows(ax_xy, state_x, state_y, 'b')
        ax_xy.set_xlabel("x position (m)")
        ax_xy.set_ylabel("y position (m)")
        ax_xy.set_ylim(bottom=-0.1, top=0.4)
        ax_xy.set_xlim(left=0.35, right=0.75)

        ax_xz.plot(goal_x, goal_z, 'or')
        ax_xz.plot(state_x, state_z, '*b')
        ax_xz.set_xlabel("x position (m)")
        ax_xz.set_ylabel("z position (m)")
        ax_xz.set_ylim(bottom=-0.5, top=0.5)
        ax_xz.set_xlim(left=0.35, right=0.75)

        ax_yz.plot(goal_y, goal_z, 'or')
        ax_yz.plot(state_y, state_z, '*b')
        ax_yz.set_xlabel("y position (m)")
        ax_yz.set_ylabel("z position(m)")
        ax_yz.set_ylim(bottom=0, top=2.0)
        ax_yz.set_xlim(left=-0.5, right=0.5)

        fname = self.demo_name + '_pos.png'
        if self.save_fig:
            print("saving figure")
            plt.savefig(fname)
        plt.show()

    def plot_errors(self):
        """ Plot 6 plots showing errors for each dimension.
        x, y, z and qx, qy, qz euler angles (from C.orientation_error)
        """
        errors_x = [error[0] for error in self.errors]
        errors_y = [error[1] for error in self.errors]
        errors_z = [error[2] for error in self.errors]

        errors_qx = [error[3] for error in self.errors]
        errors_qy = [error[4] for error in self.errors]
        errors_qz = [error[5] for error in self.errors]

        fig, ((e_x, e_y, e_z), (e_qx, e_qy, e_qz)) = plt.subplots(2, 3)

        e_x.plot(errors_x)
        e_x.set_title("X error per step.")
        e_x.set_ylabel("error (m)")
        e_x.set_xlabel("step num")

        e_y.plot(errors_y)
        e_y.set_ylabel("error (m)")
        e_y.set_xlabel("step num")
        e_y.set_title("y error per step")

        e_z.plot(errors_z)
        e_z.set_title("z error per step.")
        e_z.set_ylabel("error (m)")
        e_z.set_xlabel("step num")

        e_qx.plot(errors_qx)
        e_qx.set_title("qx error per step.")
        e_qx.set_ylabel("error (rad)")
        e_qx.set_xlabel("step num")

        e_qy.plot(errors_qy)
        e_qy.set_title("qy error per step.")
        e_qy.set_ylabel("error (rad)")
        e_qy.set_xlabel("step num")

        e_qz.plot(errors_qz)
        e_qz.set_title("qz error per step.")
        e_qz.set_ylabel("error (rad)")
        e_qz.set_xlabel("step num")

        fname = self.demo_name + '_error.png'
        if self.save_fig:
            plt.savefig(fname)
        plt.show()
        plt.show()

def get_delta(goal_pose, current_pose):
    """Get delta between goal pose and current_pose.

    Args: goal_pose (list): 7f pose [x, y, z, qx, qy, qz, w] . Position and quaternion of goal pose.
          state (list): 7f Position and quaternion of current pose.

    Returns: delta (list) 6f [dx, dy, dz, ax, ay, az] delta position and delta orientation
        as axis-angle.
    """

    if len(goal_pose) != 7:
        raise ValueError("Goal pose incorrect dimension should be 7f")
    if len(current_pose) !=7:
        raise ValueError("Current pose incorrect dimension, should be 7f")

    dpos = np.subtract(goal_pose[:3], current_pose[:3])
    goal_mat = T.quat2mat(goal_pose[3:])
    current_mat = T.quat2mat(current_pose[3:])
    delta_mat_T = np.dot(goal_mat, current_mat.T)
    delta_quat = T.mat2quat(np.transpose(delta_mat_T))
    delta_aa = T.quat2axisangle(delta_quat)

    return np.hstack((dpos, delta_aa)).tolist()

def apply_delta(pose, delta):
    """ Applies delta to pose to obtain new 7f pose.
    Args: pose (7f): x, y, z, qx, qy, qz, w . Position and quaternion
          delta (6f): dx, dy, dz, ax, ay, az. delta position and axis-angle

    """
    if len(delta) != 6:
        raise ValueError("delta should be [x, y, z, ax, ay, az]. Orientation should be axis-angle")
    if len(pose) != 7:
        raise ValueError("pose should be [x, y, z, qx, qy, qz, w] Orientation should be quaternion.")
    pos = pose[:3]
    dpos = delta[:3]

    # Get current orientation and delta as matrices to apply 3d rotation.
    ori_mat = T.quat2mat(pose[3:])
    delta_quat = T.axisangle2quat(delta[3:])
    delta_mat = T.quat2mat(delta_quat)
    new_ori = np.dot(delta_mat.T, ori_mat)

    # convert new orientation to quaternion.
    new_ori_quat = T.mat2quat(new_ori)

    # add dpos to current position.
    new_pos = np.add(pos, dpos)

    return np.hstack((new_pos, new_ori_quat))

class Path():
    """Class definition for path definition (specific to ee trajectories)

    A Path is a series absolute goal poses (7f) used to produce actions
        for the agent to take.

    Attributes:
        num_pts (int): number of points in path.
        path (list): list 7f of absolute goal poses in path. Goal poses
            are specified by [x, y, z, qx, qy, qz, w] position and quaternion.
        deltas (list): list 6f deltas between poses to produce path.



    """

    def __init__(self, shape, num_pts):
        self.shape = shape
        self.num_pts = num_pts
        self.path = []

    def make_path(self):
        self.path = [self.start_pose]
        for delta in self._deltas:
            new_pose = apply_delta(self.path[-1], delta)
            # self.path.append(np.add(self.path[-1], delta))
            self.path.append(new_pose)

        # Append path to the same
        for _ in range(int(0.2*self.num_pts)):
            self.path.append(self.path[-1])


class SequentialJoint(Path):
    """Series of joint positions sequentially incremented/decremented by delta.
    """
    def __init__(self, start_pose, delta_val=0.01, num_steps=30, joint_num=0):
        logger.info("Sequential Joint path")

        self.start_pose = start_pose
        self.delta_val = delta_val
        self.num_steps = num_steps
        self.joint_num = joint_num
        self._deltas = self._get_deltas()

        self.path = []
        self.make_path()

    def _get_deltas(self):
        """Return series of joint deltas where each joint is individually
            incremented, and then decremented by delta.
        """

        deltas = []
        joint_i = 1

        for _ in range(self.num_steps):
            delta = np.zeros(7)
            delta[self.joint_num] = self.delta_val
            deltas.append(delta)

        for _ in range(self.num_steps):
            delta = np.zeros(7)
            deltas.append(delta)

        for _ in range(self.num_steps):
            delta = np.zeros(7)
            delta[self.joint_num] = -self.delta_val
            deltas.append(delta)

        for _ in range(self.num_steps):
            delta = np.zeros(7)
            deltas.append(delta)
        return deltas

    def make_path(self):
        """Create path by sequentially adding deltas to joint pose.
        """

        self.path = [self.start_pose]

        for delta in self._deltas:
            new_pose = np.add(self.path[-1], delta)
            self.path.append(new_pose)


def make_simple_rot_mat(angle=np.pi/4):
    """Make a simple rotation matrix, rotating about one axis.
    """
    rot_m = np.eye(3)
    rot_m[0, 0] = np.cos(angle)
    rot_m[0, 1] = -np.sin(angle)
    rot_m[0, 2] = 0
    rot_m[1, 0] = np.sin(angle)
    rot_m[1, 1] = np.cos(angle)
    rot_m[1, 2] = 0
    rot_m[2, 0] = 0
    rot_m[2, 1] = 0
    rot_m[2, 2] = 1
    return rot_m

class Rotation(Path):
    """ Class definition for path that rotating end effector in place.
    Start and end orientation should be in euler angles.
    """
    def __init__(self, start_pose, num_pts,
                 rotation_rad=np.pi/4, delta_val=None, dim=2, end_ori=None):
        logger.debug("Making Rotation Path")
        self.start_pose = start_pose
        self.end_ori = end_ori
        self.num_pts = num_pts
        self.rotation_rad = rotation_rad
        if delta_val is None:
            if num_pts == 0:
                delta_val = 0
            else:
                delta_val = np.divide(rotation_rad, num_pts)
        self.dim = dim
        self.delta_val = delta_val
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):
        """Convert euler angle rotation with magnitude delta in the direction
        specified by dim.
        """
        delta = np.zeros(3)
        delta[self.dim] = self.delta_val
        # pad with position deltas= 0
        delta = np.hstack(([0, 0, 0], delta))
        self._deltas = [delta]*self.num_pts

class Line(Path):
    """Class definition for straight line in given direction.
    """
    def __init__(self, start_pose, num_pts, path_length,
                 delta_val=None, dim=0, end_pos=None):
        """ Initialize Line class

        Args:
            start_pos (list): 7f pose at start of path. Best to
                set at robot reset pose.
            num_pts (int): number of points in path.
            length (float): length of path in m
            delta_val (float): (optional) delta in m between
                each step. If None, end_pos must be specified.
            dim (int): direction to move for line, x = 0, y=1,
                z=2.
            end_pos (list): (optional) end pose for path. If not
                None, dim, and delta_val are ignored. Straight
                Line is interpolated between start and end_pos.

        """
        self.start_pose = start_pose
        self.num_pts = num_pts
        self.path_length = path_length
        if delta_val is None:
            delta_val = np.divide(self.path_length, self.num_pts)
        self.delta_val = delta_val
        self.dim = dim
        self._deltas = []
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):

        delta = np.zeros(6)
        delta[self.dim] = self.delta_val
        self._deltas = [delta]*self.num_pts
        # self._deltas[0] = np.zeros(7)


class Square(Path):
    """Class def for square path.

    Square path defined by side length and start point.
    At step 4 * sidelength -1, ee is not at initial point.
    Last step returns to initial point.

    Square path is ordered in clockwise from origin (Bottom, Left, Top, Right)

    Attributes:
        start_pose (7f): start pose to begin square from.
        num_pts (int): number of steps to take on each side.
        delta_val (float): step size in m to take for each step.
        _deltas (list): list of delta xyz from a position to reach next position
             on path.
        path (list): list of actions to take to perform square path. Actions
            are either delta xyz from current position (if use_abs is False) or
            they are absolute positions taken by adding the deltas to start.

    """
    def __init__(self, start_pose, side_num_pts, delta_val):

        self.start_pose = start_pose
        self.num_pts = side_num_pts
        self.delta_val = delta_val
        self._deltas = []
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):
        """ Get a series of steps from current position that produce
        a square shape. Travel starts with bottom side in positive direction,
        then proceeds clockwise (left, top, right.)

        """
        self._deltas = [[0, 0, 0, 0, 0, 0]]
        # Bottom side.
        for pt in range(self.num_pts):
            self._deltas.append([self.delta_val, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Left Side
        for pt in range(self.num_pts):
            self._deltas.append([0.0, -self.delta_val, 0.0, 0.0, 0.0, 0.0])
        # Top side
        for pt in range(self.num_pts):
            self._deltas.append([-self.delta_val, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Right side
        for pt in range(self.num_pts):
            self._deltas.append([0.0, self.delta_val, 0.0, 0.0, 0.0, 0.0])
