"""Abstract Control Interface for Robots. 
"""

import time
import numpy as np 
import logging 
from perls2.utils.yaml_config import YamlConfig

from perls2.ros_interfaces.redis_interface import RobotRedisInterface as RobotRedis
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
# Robot Interface and Controller Imports
from perls2.robots.robot_interface import RobotInterface
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.joint_torque import JointTorqueController
from perls2.controllers.joint_vel import JointVelController
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.interpolator.linear_ori_interpolator import LinearOriInterpolator
from perls2.controllers.robot_model.model import Model


import perls2.controllers.utils.transform_utils as T

class CtrlInterface(object): 
    """ Abstract class definition for Control Interface.

    Interface creates and monitors RedisServer for new commands from RobotInterface.
    Commands are interpreted and converted into set of joint torques sent to robot.

    Attributes:
        config (YamlConfig): YamlConfig expressing default sawyer_ctrl_parameters and redis connection info.

    """
    def __init__(self,
                 config,
                 controlType):
        """
        Initialize robot for control. 
        """
        self.config = YamlConfig(config)

        # Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False

        # Control Init
        self.controlType = controlType
        self.model = Model()