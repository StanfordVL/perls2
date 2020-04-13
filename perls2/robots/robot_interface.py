"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
from tq_control.controllers.ee_imp import EEImpController
from tq_control.robot_model.manual_model import ManualModel
#from tq_control.interpolator.reflexxes_interpolator import ReflexxesInterpolator
import numpy as np
import logging
from scipy.spatial.transform import Rotation as R
logging.basicConfig(level=logging.DEBUG)

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

    def update(self):
        orn = R.from_quat(self.ee_orientation)
        self.model.update_states(ee_pos=np.asarray(self.ee_position),
                                 ee_ori= np.asarray(self.ee_orientation),
                                 ee_pos_vel=np.asarray(self.ee_v),
                                 ee_ori_vel=np.asarray(self.ee_w),
                                 joint_pos=np.asarray(self.motor_joint_positions[:7]),
                                 joint_vel=np.asarray(self.motor_joint_velocities[:7]),
                                 joint_tau=np.asarray(self.motor_joint_accelerations[:7])
                                 )
                                

        self.model.update_model(J_pos=self.linear_jacobian,
                                J_ori=self.angular_jacobian,
                                mass_matrix=self.mass_matrix)

    def step(self):
        """Update the robot state and model, set torques from controller
        """
        self.update()
        if self.action_set:               
            torques = self.controller.run_controller() + self.N_q
            self.set_torques(torques)
        else:
            logging.ERROR("ACTION NOT SET")

    def move_ee_delta(self, delta):
        logging.debug("delta " + str(delta))
        self.controller.set_goal(goal=delta, fn="ee_delta", delta=delta)
        self.action_set = True

    def set_dq(self, dq_des):
        logging.debug("desired dq " + str(dq_des))
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
