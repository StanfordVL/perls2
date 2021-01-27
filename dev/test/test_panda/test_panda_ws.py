"""Test script for PandaCtrlInterface <-> RealPandaInterface

## Instructions.
1. Start local redis-server
```bash
    redis-server

These are more compatibility tests and not exhaustive, as both interfaces run on
different machines.
"""

import pytest
import numpy as np
# from dev.test.test_panda.fake_real_panda import FakePandaInterface
from perls2.utils.yaml_config import YamlConfig
from perls2.robots.real_panda_interface import RealPandaInterface
from perls2.ctrl_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.redis_interfaces.redis_keys import *

from dev.test.test_panda.fake_franka_panda import FakeFrankaPanda


@pytest.fixture()
def real_panda():
    driver = FakeFrankaPanda()
    driver.set_fake_state()
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    config['redis']['host'] = "127.0.0.1"
    config['redis']['port'] = 6379
    config['redis']['password'] = None
    panda = RealPandaInterface(config=config, controlType='EEImpedance')
    return panda

@pytest.fixture()
def panda_ctrl():
    config = YamlConfig('cfg/panda_ctrl_config.yaml')
    config['redis']['host'] = "127.0.0.1"
    config['redis']['port'] = 6379
    config['redis']['password'] = None
    panda_ctrl = PandaCtrlInterface(config=config,
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

# def test_get_redis_states(franka_panda, real_panda):
#     real_panda._get_state_from_redis()
#     assert( np.all(real_panda.ee_position == np.array([0.534449340443935, 0.0364261218566121, 0.313053490964604])))
#     #assert(np.all(real_panda.q == np.array([0.153399527304672, -0.0228098171871691, -0.0856113330690632, -2.13645188983878, 0.000673128167293589, 2.11842455338582, 2.61612591026992])))
