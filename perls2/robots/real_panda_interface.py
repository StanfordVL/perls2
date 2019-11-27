"""
Abstract class defining the interface to the Panda Robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions


@six.add_metaclass(abc.ABCMeta)

from perls2.robots.real_robot_interface import RealRobotInterface
class RealPandaInterface(RealRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 pose=None,
                 controlType=None):