"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""
import abc  # For abstract class definitions
import six  # For abstract class definitions
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
from perls2.controllers.joint_vel import JointVelController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.joint_torque import JointTorqueController

from perls2.controllers.robot_model.model import Model
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.interpolator.linear_ori_interpolator import LinearOriInterpolator
from perls2.controllers.utils import transform_utils as T
import numpy as np
import logging

AVAILABLE_CONTROLLERS = ["EEImpedance",
                         "EEPosture",
                         "Internal",
                         "JointVelocity",
                         "JointImpedance",
                         "JointTorque"]


@six.add_metaclass(abc.ABCMeta)
class RobotInterface(object):
    """Abstract interface to be implemented for each real and simulated
    robot.

    Attributes:
        action_set (bool): flag describing if action for controller has been
            set.
        config (dict): Dictionary describing robot parameters.
        control_type (str): Type of controller for robot to use
            e.g. "EEImpedance", "JointVelocity"
        interpolator_pos (perls2.interpolator.linear_interpolator): Interpolator for ee/joint position
        interpolator_ori (perls2.interpolator.linear_interpolator): Interpolator for ee orientation
        model (Model): a model of the robot state as defined by tq_control.
        controller (Controller): tq_control object that takes robot states and
            compute torques.


    """

    def __init__(self,
                 config,
                 controlType=None):
        """
        Initialize variables.

        Creates controller and interpolator from config file.

        Args:
            config (dict): Dictionary describing robot parameters.
            control_type (str): Type of controller for robot to use
                e.g. IK, OSC, Joint Velocity.
        """
        self.config = config
        self.controlType = controlType
        self.action_set = False
        self.model = Model()

        self.interpolator_pos = None
        self.interpolator_ori = None
        self.robot_type = self.config['world']['robot']
        self.controller_cfg = self.config[self.robot_type + "_controller"]

        # Make position interpolator from config
        interp_pos_cfg = self.controller_cfg['interpolator_pos']
        if interp_pos_cfg['type'] == 'linear':
            self.interpolator_pos = LinearInterpolator(
                max_dx=interp_pos_cfg['max_dx'],
                ndim=3,
                controller_freq=self.config['control_freq'],
                policy_freq=self.config['policy_freq'],
                ramp_ratio=interp_pos_cfg['ramp_ratio'])
        # Make orientation interpolator from config.
        interp_ori_cfg = self.controller_cfg['interpolator_ori']
        if interp_ori_cfg['type'] == 'linear':
            self.interpolator_ori = LinearOriInterpolator(
                controller_freq=self.config['control_freq'],
                policy_freq=self.config['policy_freq'],
                fraction=interp_ori_cfg['fraction'])

        # Safenet position limits


        self.use_safenet = False
        self.safenet_ee_pos_lower = None
        self.safenet_ee_pos_upper = None
        self.controller = None
        if 'safenet' in self.config:
            if self.config['safenet']['use_safenet']:
                self.set_safenet_boundaries(
                    self.config['safenet']['lower'], self.config['safenet']['upper'])

    def update(self):
        """Update robot interface model with states for controller.
        """
        raise NotImplementedError

    def set_safenet_boundaries(self, lower, upper):
        """Set position limits for existing controller.

        Controller will clip all end-effector goals to these limits.

        Args:
            lower (list): 3f lower boundary position limits for EE
            upper (list): 3f upper boundary position limits for EE

        Return: 
            None
        """
        assert (np.all(lower <= upper))
        self.safenet_ee_pos_upper = np.asarray(upper)
        self.safenet_ee_pos_lower = np.asarray(lower)
        self.use_safenet = True
        if self.controller is not None:
            self.controller.position_limits = np.array([lower, upper])

    def get_safenet_limits(self):
        """Return safenet boundary limits as tuple
        """
        return (self.safenet_ee_pos_lower, self.safenet_ee_pos_upper)
    

    def make_controller(self, control_type):
        """Returns a new controller type based on specs.

        Args:
            control_type (str) : name of the control type.
        """

        if control_type == "Internal":
            return "Internal"
        world_name = self.config['world']['type']
        controller_dict = self.controller_cfg[world_name][control_type]
        
        if self.use_safenet: 
            pos_limits = np.array([self.safenet_ee_pos_lower, self.safenet_ee_pos_upper])
        else: 
            pos_limits = None

        if control_type == "EEImpedance":
            return EEImpController(
                self.model,
                kp=controller_dict['kp'],
                damping=controller_dict['damping'],
                input_max=np.array(controller_dict['input_max']),
                input_min=np.array(controller_dict['input_min']),
                output_max=np.array(controller_dict['output_max']),
                output_min=np.array(controller_dict['output_min']),
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori,
                control_freq=self.config['control_freq'], 
                position_limits=pos_limits)
        elif control_type == "EEPosture":
            return EEPostureController(
                self.model,
                kp=controller_dict['kp'],
                damping=controller_dict['damping'],
                posture_gain=controller_dict['posture_gain'],
                posture=controller_dict['posture'],
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori,
                input_max=np.array(controller_dict['input_max']),
                input_min=np.array(controller_dict['input_min']),
                output_max=np.array(controller_dict['output_max']),
                output_min=np.array(controller_dict['output_min']),
                control_freq=self.config['control_freq'])
        elif control_type == "JointVelocity":
            return JointVelController(
                robot_model=self.model,
                kv=controller_dict['kv'])
        elif control_type == "JointImpedance":
            return JointImpController(
                robot_model=self.model,
                kp=controller_dict['kp'],
                damping=controller_dict['damping'])
        elif control_type == "JointTorque":
            return JointTorqueController(
                robot_model=self.model)
        else:
            return ValueError("Invalid control type")

    def change_controller(self, next_type):
        """Change to a different controller type.

        Args:
            next_type (str): keyword for desired control type.
                Choose from:
                -'EEImpedance'
                -'EEPosture'
                -'JointVelocity'
                -'JointImpedance'
                -'JointTorque'

        """
        if next_type in AVAILABLE_CONTROLLERS:
            self.controller = self.make_controller(next_type)
            self.controlType = next_type
            return self.controlType
        else:
            raise ValueError("Invalid control type " + "\nChoose from {}".format(AVAILABLE_CONTROLLERS))

    def step(self):
        """Update the robot state and model, set torques from controller.
        """
        self.update_model()
        if self.controller == "Internal":
            return
        else:
            if self.action_set:
                torques = self.controller.run_controller()
                self.set_torques(torques)


    def reset(self):
        """Reset robot to neutral joint state.
        """
        raise NotImplementedError
    
    def set_controller_goal(self, **kwargs):
        """Update model and set the goal for controller.

        Args:
            kwargs (dict): arguments specific to controller for setting goal.
        """
        self.update_model()
        self.controller.set_goal(**kwargs)
        self.action_set = True

    def update_model(self):
        """Update robot model with states and dynamics.
        """
        raise NotImplementedError

    def _check_controller(self, fn_control_types):

        if self.controlType not in fn_control_types:
            raise ValueError("Wrong Control Type for this command. Change to {}".format(fn_control_types))

    def _check_control_arg(self, argname, arg_val, mandatory, length):
        """Helper function to check for valid control arguments and dimensions.

        Args:
            argname (str): name of the argument for debug message.
            arg_val (list or None): the argument to be checked.
            mandatory (bool): if the argument can be None
            length (int): valid length of argument.

        Returns:
            valid (bool): True if argument is valid and not None
        """
        valid = False
        if mandatory and (arg_val is None):
            raise ValueError("{} is None. Should be list of {}".format(argname, length))
        if arg_val is not None:
            if len(arg_val) != length:
                raise ValueError("Invalid dim for {}. Should be list of length {}".format(argname, length))
            else:
                valid = True
        return valid

    def move_ee_delta(self, delta, set_pos=None, set_ori=None):
        """ Use controller to move end effector by some delta.

        Args:
            delta (6f): delta position (dx, dy, dz) concatenated with delta orientation.
                Orientation change is specified as an axis angle rotation from previous
                orientation.
            set_pos (3f): end effector position to maintain while changing orientation.
                [x, y, z]. If not None, the delta for position is ignored.
            set_ori (4f): end effector orientation to maintain while changing orientation
                as a quaternion [qx, qy, qz, w]. If not None, any delta for orientation is ignored.

        Notes:
            To fix position or orientation, it is better to specify using the kwargs than
            to use a 0 for the corresponding delta. This prevents any error from accumulating in
            that dimension.

        Examples:
            Specify delta holding fixed orientation::
            self.robot_interface.move_ee_delta([0.1, 0, 0, 0, 0, 0], set_pos=None, set_ori=[0, 0, 0, 1])

        """
        self._check_control_arg('delta', delta, mandatory=True, length=6)
        if self._check_control_arg('set_ori', set_ori, mandatory=False, length=4):
            set_ori = T.quat2mat(set_ori)
        self._check_control_arg('set_pos', set_pos, mandatory=False, length=3)
        self._check_controller(["EEImpedance", "EEPosture"])
        kwargs = {'delta': delta, 'set_pos': set_pos, 'set_ori': set_ori}
        self.set_controller_goal(**kwargs)

    def set_ee_pose(self, set_pos, set_ori, **kwargs):
        """ Use controller to set end effector pose.

        Args:
            set_pos (list): 3f [x, y , z] desired absolute ee position
                in world frame.
            set_ori (list): 4f [qx, qy, qz, w] desired absolute ee orientation
                in world frame.
            **kwargs (dict): used to catch all other arguments.

        Returns:
            None

        Notes: Only for use with EEImpedance and EEPosture controller.
        """
        if self._check_control_arg('set_ori', set_ori, mandatory=True, length=4):
            set_ori = T.quat2mat(set_ori)
        self._check_control_arg('set_pos', set_pos, mandatory=True, length=3)
        self._check_controller(["EEImpedance", "EEPosture"])
        kwargs = {'delta': None, 'set_pos': set_pos, 'set_ori': set_ori}
        self.set_controller_goal(**kwargs)

    def set_joint_velocities(self, velocities):
        """ Use controller to set joint velocity of the robot.

        Args:
            velocities (list): 7f desired joint velocities (rad/s) for each joint.
            Joint 0 is the base

        Returns: None.

        Notes: Only for use with JointVelocity controller.
        """

        self._check_controller("JointVelocity")
        kwargs = {'velocities': velocities}
        self.set_controller_goal(**kwargs)

    def set_joint_delta(self, delta, **kwargs):
        """ Use controller to set new joint position with a delta.

        Args:
            delta (ndarray): 7f delta joint position (rad) from current
                 joint position.

        Returns: None

        Notes: Only for use with JointImpedance controller.
               Does not check for exceeding maximum joint limits. (TODO)

        """
        self._check_controller("JointImpedance")
        kwargs = {"delta": delta}
        self.set_controller_goal(**kwargs)

    def set_joint_positions(self, set_qpos, **kwargs):
        """ Use controller to set new joint positions.

        Args:
            set_qpos: 7f absolute joint positions (rad)
        Returns:
            None
        Notes: Only for use with Joint Impedance Controller
        """
        self._check_controller("JointImpedance")
        kwargs['set_qpos'] = set_qpos
        kwargs['delta'] = None
        self.set_controller_goal(**kwargs)

    def set_joint_torques(self, torques):
        """Set joint torques to new joint torques.

        Args:
            torques (list): 7f list of toruqes to command.

        Notes:
            Only for use with Joint Torque Controlle
        """
        self._check_controller("JointTorque")
        kwargs = {'torques': torques}
        self.set_controller_goal(**kwargs)

    def create(config):
        """Factory for creating robot interfaces based on config type
        """
        raise NotImplementedError

    @property
    def base_pose(self):
        return self.pose

    @property
    @abc.abstractmethod
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def name(self):
        """str of the name of the robot
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_position(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_orientation(self):
        """list of four floats [qx, qy, qz, qw] of the orientation
        quaternion of the end-effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_pose(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_v(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_w(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_twist(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def q(self):
        """List of 7f describing joint positions (rad) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def dq(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def tau(self):
        """List of 7f describing joint torques (Nm)

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def linear_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def angular_jacobian(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def mass_matrix(self):
        """List of 7f describing joint velocities (rad/s) of the robot arm.

        Ordered from base to end_effector
        """
        raise NotImplementedError

    @abc.abstractmethod
    def open_gripper(self):
        """ Open robot gripper.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def close_gripper(self):
        """ Close robot gripper
        """
        raise NotImplementedError

    @abc.abstractmethod
    def set_gripper_to_value(self, value):
        """ Set gripper to desired open/close value
        """
        raise NotImplementedError
