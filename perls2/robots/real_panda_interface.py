"""
Class defining the interface to the Panda Robots.
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
import sys
import copy
import redis
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)
import pybullet as pb


from perls2.robots.real_robot_interface import RealRobotInterface

def bstr_to_ndarray(array_bstr):
    """Convert bytestring array to 1d array
    """
    return np.fromstring(array_bstr[1:-1], dtype=np.float, sep=',')

class RealPandaInterface(RealRobotInterface):
    """Class definition for interfacing to real Franka Panda Arms.
    """

    def __init__(self,
                 config,
                 pose=None,
                 controlType=None):
        """
        Initialize
        """
        super().__init__(config)
