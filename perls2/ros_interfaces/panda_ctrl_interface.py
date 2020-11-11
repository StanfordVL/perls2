"""Control interface for Panda
"""
import time 

from perls2.ros_interfaces.ctrl_interface import CtrlInterface 
from perls2.ros_interfaces.panda_redis_keys import PandaKeys 
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig
import logging
logging.basicConfig(level=logging.DEBUG)

P = PandaKeys()
class PandaCtrlInterface(CtrlInterface):
    """Interface for franka-panda redis driver and perls2.RealPandaInterface. 

    The Panda Interface gets robot actions from persl2.RealPandaInterface via
    redis. Robot states are obtained from the franka-panda redis driver. The controllers, 
    and robot states are combined to calculate torques. The PandaCtrlInterface then updates redis
    with the robot state for RealPandaInterface. 

    """

    def __init__(self,
                 config,
                 controlType):
        """ Initialize the control interface.
        """
        self.config = YamlConfig(config)
        self.redisClient = PandaRedisInterface(**self.config['redis'])
    
        # Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False

        # Control Init
        self.controlType = controlType

    @property
    def driver_connected(self): 
        return self.redisClient.get(P.DRIVER_CONN_KEY) == P.DRIVER_CONNECTED_VALUE

    def run(self): 
        if self.driver_connected: 

            logging.info("driver is connected.")
        else:
            print(P.DRIVER_CONN_KEY)
            print(self.redisClient.get(P.DRIVER_CONN_KEY))
            raise ValueError("run driver first.")

if __name__ == '__main__':
    ctrl_interface = PandaCtrlInterface(
        config='cfg/panda_ctrl_config.yaml', controlType=None)
    ctrl_interface.run()