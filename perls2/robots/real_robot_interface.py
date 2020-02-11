"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc   # For abstract class definitions
import six   # For abstract class definitions
import redis  # For communicating with RobotCtrlInterfaces

from perls2.robots.robot_interface import RobotInterface


class RealRobotInterface(RobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 pose=None,
                 controlType=None):

        self.config = config
        # Get the robot config dict by using the name of the robot
        # as a key. The robot config yaml should be included at
        # project config file level.
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]

    def create(config, physics_id, arm_id):
        """Factory for creating robot interfaces based on config type
        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.real_sawyer_interface import RealSawyerInterface
            return RealSawyerInterface(
                config=config, physics_id=physics_id, arm_id=arm_id)
        else:
            raise ValueError("invalid robot interface type. choose 'sawyer'")
