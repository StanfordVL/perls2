"""Test script for RealPandaInterface

## Instructions.
1. Start local redis-server
```bash
    redis-server
```
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.robots.real_panda_interface import RealPandaInterface
from perls2.ros_interfaces.redis_keys import *
import pytest

@pytest.fixture()
def real_panda():
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    panda = RealPandaInterface(config=config, controlType='EEImpedance')
    return panda

def test_connect(real_panda):
    """connect()
    """
    real_panda = real_panda
    real_panda.connect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'True')

def test_disconnect(real_panda):
    """disconnect()
    """
    real_panda = real_panda
    real_panda.disconnect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) != b'True')
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'False')

def test_set_controller_params_from_config(real_panda):
    """set_controller_params_from_config()
    """
    config = real_panda.config
    config['controller']['Real']['selected_type'] = "EEImpedance"
    real_panda.set_controller_params_from_config()
    assert(real_panda.redisClient.get(CONTROLLER_CONTROL_TYPE_KEY) == b'EEImpedance')
    assert(real_panda.redisClient.get(CONTROLLER_CONTROL_PARAMS_KEY) ==
        config['controller']['Real']['EEImpedance'])
