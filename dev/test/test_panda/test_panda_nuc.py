"""Test script for PandaCtrlInterface <-> RealPandaInterface

This script is meant to be run on the nuc, may be extended to ws.

## Instructions.
1. Start local redis-server
```bash
    redis-server
```
"""
import redis

from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface

from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig
P = PandaKeys('cfg/franka-panda.yaml')

# 1. Fake franka-panda redis driver set up.
class FakeFrankaPanda(object):
    """Class for faking panda redis driver.

    Sets fake values for redis, mocking franka-panda.
    """
    def __init__(self):
        self.driver_config = YamlConfig('cfg/franka-panda.yaml')
        # some renaming for the panda redis interface.
        redis_config = {"host": self.driver_config['redis']['ip'],
                        "port": self.driver_config['redis']['port'],
                        "driver_config":'cfg/franka-panda.yaml' }
        self.redisClient = PandaRedisInterface(**redis_config)

    def start(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_CONNECTED_VALUE)

    def stop(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_DISCONN_VALUE)

    def set_q(self, q):
        """ Set fake joint positions in format franka-panda redis uses

        Args:
            q (list): 7f joint positions.
        """
        self.redisClient.set(P.ROBOT_STATE_Q_KEY, str(q))

def test_fake_franka_panda():
    franka_panda = FakeFrankaPanda()
    # Start driver, check status changes.
    franka_panda.start()
    assert(franka_panda.redisClient.get(P.DRIVER_CONN_KEY) == b"running")
    # Stop driver, check status changes.
    franka_panda.stop()
    assert(franka_panda.redisClient.get(P.DRIVER_CONN_KEY) == b"off")
    #
    franka_panda.start()

if __name__ == '__main__':
    test_fake_franka_panda()