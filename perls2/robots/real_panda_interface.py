"""Class defining interface to Real Panda Robots.

RealPandaInterfaces provide access to robot states, and the ability to send
commands to real Franka Panda arms.

Operation:

RealPandaInterfaces connect to a redis-server, and send commands to robots by
setting keys on the redis db. These actions are interpreted by the PandaCtrlInterface
running on the NUC, which converts these actions along with the robot state into torque
commands. RealPandaInterfaces obtain robot proprioception by querying the redis-db.

"""

import abc  # For abstract class definitions
import six  # For abstract class definitions

from perls2.robots.real_robot_interface import RealRobotInterface


class RealPandaInterface(RealRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 controlType='EEImpedance')
