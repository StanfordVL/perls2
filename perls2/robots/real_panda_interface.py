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
from perls2.ros_interfaces.redis_interface import PandaRedisInterface

class RealPandaInterface(RealRobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 controlType='EEImpedance'):
        """Initialize the real panda interface.
        """
        super().__init__(controlType=controlType, config=config)

        # Create redis interface specific to panda.
        redis_config = self.config['redis']
        self.redisClient = PandaRedisInterface(**self.config['redis'])
