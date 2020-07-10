"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
import time
from perls2.controllers.joint_vel import JointVelController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.joint_torque import JointTorqueController


from perls2.controllers.robot_model.model import Model
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.utils import transform_utils as T
import numpy as np
import logging
from scipy.spatial.transform import Rotation as R


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
        control_type (str): Type of controller for robot to use
            e.g. IK, OSC, Joint Velocity
        model (Model): a model of the robot state as defined by tq_control.
        controller (Controller): tq_control object that takes robot states and compute torques.

    """

    def __init__(self,
                 controlType,
                 config=None):
        """
        Initialize variables

        Args:
            control_type (str): Type of controller for robot to use
                e.g. IK, OSC, Joint Velocity
            model (Model): a model of the robot state as defined by tq_control.
            controller (Controller): tq_control object that takes robot states and compute torques.

        :TODO:
            * controller type not currently supported
        """
        self.controlType = controlType
        self.action_set = False
        self.model = Model()
        self.config = config
        if config is not None:
            world_name = config['world']['type']
            controller_config = config['controller'][world_name]
            if config['controller']['interpolator']['type'] == 'linear':
                self.interpolator = LinearInterpolator(max_dx=0.5, 
                                                       ndim=3, 
                                                       controller_freq=1000, 
                                                       policy_freq=20, 
                                                       ramp_ratio=0.02)
            else:
                self.interpolator = None
        self.interpolator_goal_set = False

    def update(self):
        raise NotImplementedError

    def make_controller(self, control_type, **kwargs):
        """Returns a new controller type based on specs. 

        Wrapper for Controller constructor in tq_control

        Args: 
            control_type (str) : name of the control type. 
            kwargs:  dependent on type of controller to modify those 
                found in the config file.

        EEImpedance kwargs: (not supported yet.)
            'input_max' : (float) max value for input delta. Does not apply for
                set_pose commands.
            'input_min' : (float) min value for input delta. Does not apply for 
                set pose commands
            'output_max' : (float) max value for scaled action delta.
            'output_min' : (float) min value for scaled action delta. 
            'kp' : (float) gain for position / orientation error
            'damping' : (float) [0,1] damping coefficient for error
        """

        if control_type == "Internal":
            return "Internal"
        world_name = self.config['world']['type']
        controller_dict = self.config['controller'][world_name][control_type]
        if control_type == "EEImpedance":
            return EEImpController(self.model,
                kp=controller_dict['kp'], 
                damping=controller_dict['damping'],
                interpolator_pos =self.interpolator,
                interpolator_ori=None,
                control_freq=self.config['sim_params']['control_freq'])
        elif control_type == "EEPosture":
            return EEPostureController(self.model, 
                kp=controller_dict['kp'], 
                damping=controller_dict['damping'],
                interpolator_pos =self.interpolator,
                interpolator_ori=None,
                input_max=np.array(controller_dict['input_max']),
                input_min=np.array(controller_dict['input_min']),
                output_max=np.array(controller_dict['output_max']), 
                output_min=np.array(controller_dict['output_min']),
                control_freq=self.config['sim_params']['control_freq'])
        elif control_type == "JointVelocity":
            return JointVelController(
                robot_model=self.model, 
                kv=controller_dict['kv'])
        elif control_type == "JointImpedance":
            return JointImpController(
                robot_model= self.model, 
                kp=controller_dict['kp'], 
                damping=controller_dict['damping']
                )
        elif control_type == "JointTorque":
            return JointTorqueController(
                robot_model=self.model )
        elif control_type == "EEPosture":
            return EEPostureController(self.model, 
                kp=controller_dict['kp'], 
                damping=controller_dict['damping'],
                posture_gain=controller_dict['posture_gain'],
                interpolator_pos =self.interpolator,
                interpolator_ori=None,
                control_freq=self.config['sim_params']['control_freq'])
        else: 
            return ValueError("Invalid control type")


    def change_controller(self, next_type):
        """Change to a different controller type.
        Args: 
            next_type (str): keyword for desired control type. 
                Choose from: 
                    -'EEImpedance'
                    -'JointVelocity'
                    -'JointImpedance'
                    -'JointTorque'
        """ 
        if next_type in AVAILABLE_CONTROLLERS:
            self.controller = self.make_controller(next_type)
            self.controlType = next_type
            return self.controlType
        else:
            raise ValueError("Invalid control type " + 
                "\nChoose from EEImpedance, JointVelocity, JointImpedance, JointTorque")



    def step(self):
        """Update the robot state and model, set torques from controller
        """
        self.update()
        if self.controller == "Internal":
            return
        else:
            if self.action_set:               
                torques = self.controller.run_controller() + self.N_q
                self.set_torques(torques)
            else:
                print("ACTION NOT SET")

    def set_controller_goal(self, **kwargs):
        self.controller.set_goal(**kwargs)
        self.action_set = True

    def check_controller(self, fn_control_type):
        if self.controlType != fn_control_type:
            raise ValueError('Wrong Control Type for this command. Change to ' + fn_control_type)

    def move_ee_delta(self, delta, fix_pos=None, fix_ori=None):
        """ Use controller to move end effector by some delta.

        Args: 
            delta (6f): delta position (dx, dy, dz) concatenated with delta orientation.
                Orientation change is specified as an Euler angle body XYZ rotation about the
                end effector link. 
            fix_pos (3f): end effector position to maintain while changing orientation. 
                [x, y, z]. If not None, the delta for position is ignored. 
            fix_ori (4f): end effector orientation to maintain while changing orientation
                as a quaternion [qx, qy, qz, w]. If not None, any delta for orientation is ignored. 
        
        Note: only for use with EE impedance controller
        Note: to fix position or orientation, it is better to specify using the kwargs than
            to use a 0 for the corresponding delta. This prevents any error from accumulating in 
            that dimension. 

        """
        #self.check_controller("EEImpedance")
        if fix_ori is not None:
            if len(fix_ori) != 4:
                raise ValueError('fix_ori incorrect dimensions, should be quaternion length 4')
            fix_ori= T.quat2mat(fix_ori)
        if fix_pos is not None:
            if len(fix_pos) != 3:
                raise ValueError('fix_pos incorrect dimensions, should be length 3')

        kwargs = {'delta': delta, 'set_pos': fix_pos, 'set_ori':fix_ori}
        self.set_controller_goal(**kwargs)

    def set_ee_pose(self, des_pose):
        """ Use controller to set end effector pose. 

        Args: des pose (7f): [x, y , z, qx, qy, qz, w]. end effector pose as position + quaternion orientation

        """
        self.check_controller("EEImpedance")
        kwargs = {'delta': None, 'set_pos': des_pose[:3], 'set_ori': des_pose[3:]}
        self.set_controller_goal(**kwargs)

        
    def set_joint_velocity(self, dq_des):
        """ Use controller to set joint velocity of the robot.
        Args:
            dq_des(ndarray): 7f desired joint velocities (rad/s) for each joint.
                Joint 0 is the base
        Returns: None.
        Notes: Only for use with JointVelocity controller.
        """        

        self.check_controller("JointVelocity")
        kwargs = {'velocities': dq_des}
        self.set_controller_goal(**kwargs)

    def set_joint_delta(self, delta):
        """ Use controller to set new joint position with a delta. 
        Args:
            delta (ndarray): 7f delta joint position (rad) from current
                 joint position. 
        Returns: None
        Notes: Only for use with JointImpedance controller.
               Does not check for exceeding maximum joint limits. (TODO)
        """
        self.check_controller("JointImpedance")
        kwargs = {'delta': delta}
        self.set_controller_goal(**kwargs)

    def set_joint_positions(self, pose):    
        self.check_controller("JointImpedance")
        kwargs = {'delta':None,'pose':pose}
        self.set_controller_goal(**kwargs)

    def set_joint_torque(self, torque):
        self.check_controller("JointTorque")
        kwargs = {'torque':torques}
        self.set_controller_goal(**kwargs)


    @abc.abstractmethod
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
