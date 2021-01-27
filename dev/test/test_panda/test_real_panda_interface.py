"""Test script for RealPandaInterface

## Instructions.
1. Start local redis-server
```bash
    redis-server
```
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.robots.real_panda_interface import RealPandaInterface
from perls2.redis_interfaces.redis_keys import *
from dev.test.test_panda.fake_panda_driver import FakePandaDriver
import pytest

@pytest.fixture()
def driver():
    driver = FakePandaDriver()
    driver.set_fake_state()
    return driver

# @pytest.fixture()
# def real_panda():
#     config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
#     panda = RealPandaInterface(config=config, controlType='EEImpedance')
#     return panda

@pytest.fixture()
def real_panda():
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    config['redis']['host'] = "127.0.0.1"
    config['redis']['port'] = 6379
    config['redis']['password'] = None
    panda = RealPandaInterface(config=config, controlType='EEImpedance')
    return panda

def test_connect(driver, real_panda):
    """connect()
    """
    real_panda = real_panda
    real_panda.connect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'True')

def test_disconnect(driver, real_panda):
    """disconnect()
    """
    real_panda = real_panda
    real_panda.disconnect()
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) != b'True')
    assert(real_panda.redisClient.get(ROBOT_ENV_CONN_KEY) == b'False')

def test_set_controller_params_from_config(driver, real_panda):
    """set_controller_params_from_config()
    """
    config = real_panda.config
    config['controller']['Real']['selected_type'] = "EEImpedance"
    real_panda.set_controller_params_from_config()
    assert(real_panda.redisClient.get(CONTROLLER_CONTROL_TYPE_KEY) == b'EEImpedance')
    assert(real_panda.redisClient.get(CONTROLLER_CONTROL_PARAMS_KEY) ==
        config['controller']['Real']['EEImpedance'])

def test_get_states_from_redis(driver, real_panda):
    """_get_state_from_redis()
    """
    real_panda._get_state_from_redis()
    assert( np.all(real_panda.ee_position == np.array([0.534449340443935, 0.0364261218566121, 0.313053490964604])))
    assert(np.all(real_panda.q == np.array([0.153399527304672, -0.0228098171871691, -0.0856113330690632, -2.13645188983878, 0.000673128167293589, 2.11842455338582, 2.61612591026992])))