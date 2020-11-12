"""Test script for PandaCtrlInterface <-> RealPandaInterface

This script is meant to be run on the nuc, may be extended to ws.

## Instructions.
1. Start local redis-server
```bash
    redis-server
```
"""
import redis
import pytest
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.robots.real_panda_interface import RealPandaInterface

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

def test_fake_redis_driver():
    redis_driver = FakeFrankaPanda()
    # Start driver, check status changes.
    redis_driver.start()
    assert(redis_driver.redisClient.get(P.DRIVER_CONN_KEY) == b"running")
    # Stop driver, check status changes.
    redis_driver.stop()
    assert(redis_driver.redisClient.get(P.DRIVER_CONN_KEY) == b"off")

def test_panda_ctrl_setup():
    # Set up fake franka panda redis driver
    redis_driver = FakeFrankaPanda()
    redis_driver.start()

    panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml',
                                    controlType='EEImpedance')

def test_real_panda_setup():
    """Test for Setting up real Panda Interface.
    """
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    real_panda = RealPandaInterface(config=config, controlType='EEImpedance')
    # Test real panda connect / disconnect.
    real_panda.connect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'True')
    real_panda.disconnect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) != b'True')
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'False')


if __name__ == '__main__':
    test_fake_redis_driver()
    test_panda_ctrl_setup()
    test_real_panda_setup()