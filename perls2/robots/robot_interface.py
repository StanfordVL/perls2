"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
from tq_control.controllers.ee_imp import EEImpController
from tq_control.controllers.ee_imp import EEImpController
import time
from tq_control.controllers.joint_vel import JointVelController
from tq_control.controllers.joint_imp import JointImpController
from tq_control.controllers.joint_torque import JointTorqueController

from tq_control.robot_model.manual_model import ManualModel
#from tq_control.interpolator.reflexxes_interpolator import ReflexxesInterpolator
import numpy as np
import logging
from scipy.spatial.transform import Rotation as R
#logging.basicConfig(level=logging.DEBUG)


@six.add_metaclass(abc.ABCMeta)
class RobotInterface(object):
    """Abstract interface to be implemented for each real and simulated
    robot.

    Attributes:
        control_type (str): Type of controller for robot to use
            e.g. IK, OSC, Joint Velocity
        model (ManualModel): a model of the robot state as defined by tq_control.
        controller (Controller): tq_control object that takes robot states and compute torques.

    """

    def __init__(self,
                 controlType):
        """
        Initialize variables

        Args:
            control_type (str): Type of controller for robot to use
                e.g. IK, OSC, Joint Velocity
            model (ManualModel): a model of the robot state as defined by tq_control.
            controller (Controller): tq_control object that takes robot states and compute torques.

        :TODO:
            * controller type not currently supported
        """
        self.controlType = controlType
        self.action_set = False
        self.model = ManualModel()
    


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
        controller_dict = self.config['controller'][control_type]
        if control_type == "EEImpedance":
            return EEImpController(self.model,
                kp=controller_dict['kp'], 
                damping=controller_dict['damping'],
                input_max=controller_dict['input_max'], 
                input_min=controller_dict['input_min'],
                output_max=controller_dict['output_max'], 
                output_min=controller_dict['output_min'], 
                interpolator_pos =None,
                interpolator_ori=None,
                control_freq=self.config['sim_params']['control_freq'])
        elif control_type == "JointVelocity":
            return JointVelController(
                robot_model=self.model, 
                kv=self.config['controller']['JointVelocity']['kv'],
                input_max=controller_dict['input_max'], 
                input_min=controller_dict['input_min'],
                output_max=controller_dict['output_max'], 
                output_min=controller_dict['output_min'])
        elif control_type == "JointImpedance":
            return JointImpController(
                robot_model= self.model, 
                kp=controller_dict['kp'], 
                damping=controller_dict['damping'],
                input_max=controller_dict['input_max'], 
                input_min=controller_dict['input_min'],
                output_max=controller_dict['output_max'], 
                output_min=controller_dict['output_min'], 
                )
        elif control_type == "JointTorque":
            return JointTorqueController(
                robot_model=self.model,
                input_max=controller_dict['input_max'], 
                input_min=controller_dict['input_min'],
                output_max=controller_dict['output_max'], 
                output_min=controller_dict['output_min'], )
        else: 
            return ValueError("Invalid control type")


    def change_controller(self, next_type):
        """Change to a different controller type.
        Args: 
            next_type (str): keyword for desired control type. 
                Choose from: 
                    -'EEImpedance'
                    -'JointVelocity'
                    'JointImpedance'
        """ 

        if next_type == "EEImpedance":
            self.controller = self.make_controller(next_type)
        elif next_type == "JointVelocity":
            self.controller = self.make_controller(next_type)
        elif next_type =="Internal":
            self.controller = self.make_controller(next_type)
        elif next_type == "JointImpedance":
            self.controller = JointImpController(
                robot_model= self.model,
                kp=self.config['controller']['JointImpedance']['kp'],
                damping=self.config['controller']['JointImpedance']['damping'])
        elif next_type == "JointTorque":
            self.controller = self.make_controller(next_type)
        else:
            raise ValueError("Invalid control type " + str(next_type)  +
                "\nChoose from EEImpedance, JointVelocity, JointImpedance")
        self.controlType = next_type
        return self.controlType

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

    def move_ee_delta(self, delta):
        self.controller.set_goal(goal=delta, fn="ee_delta", delta=delta)
        self.action_set = True

    def set_dq(self, dq_des):
        self.controller.set_goal(dq_des)
        self.action_set = True

    def set_joint_delta(self, pose):
        self.controller.set_goal(pose)
        self.action_set = True

    def set_joint_torque(self, torque):
        self.controller.set_goal(torque)

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
