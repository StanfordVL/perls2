"""
Abstract class defining the interface to the Mujoco Sawyer Robotsrobots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions

from perls2.robots.mujoco_robot_interface import MujocoRobotInterface


class MujocoSawyerInterface(MujocoRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 pose=None,
                 controlType=None):
