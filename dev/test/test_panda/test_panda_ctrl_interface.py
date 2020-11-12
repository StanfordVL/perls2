"""Unit tests for PandaCtrlInterface
"""
import pytest
import numpy as np

from perls2.utils.yaml_config import YamlConfig

from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
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

@pytest.fixture
def panda_ctrl():
    panda = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml',
                                    controlType='EEImpedance')
    return panda

def test_action_set_false_on_init(panda_ctrl)
    panda_ctrl = panda_ctrl
    assert(panda_ctrl.action_set == False)

