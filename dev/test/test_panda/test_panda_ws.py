"""Test script for PandaCtrlInterface <-> RealPandaInterface

## Instructions.
1. Start local redis-server
```bash
    redis-server

These are more compatibility tests and not exhaustive, as both interfaces run on
different machines.
"""

import pytest
# from dev.test.test_panda.fake_real_panda import FakePandaInterface
from perls2.utils.yaml_config import YamlConfig
from perls2.robots.real_panda_interface import RealPandaInterface
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.ros_interfaces.redis_keys import *

@pytest.fixture()
def real_panda():
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    panda = RealPandaInterface(config=config, controlType='EEImpedance')
    return panda

@pytest.fixture()
def panda_ctrl():
    panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml',
                                controlType='EEImpedance')
    return panda_ctrl

def test_env_connection(real_panda, panda_ctrl):
    real_panda = real_panda
    panda_ctrl = panda_ctrl

    # connect real panda.
    real_panda.connect()
    assert(panda_ctrl.env_connected)

def test_env_disconnect(real_panda, panda_ctrl):
    real_panda = real_panda
    panda_ctrl = panda_ctrl

    # connect real panda.
    real_panda.disconnect()
    assert(not panda_ctrl.env_connected)
