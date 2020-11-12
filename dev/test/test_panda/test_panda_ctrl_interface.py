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

@pytest.fixture
def panda_ctrl():
    panda = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml',
                                    controlType='EEImpedance')
    return panda

def test_action_set_false_on_init(panda_ctrl)
    panda_ctrl = panda_ctrl
    assert(panda_ctrl.action_set == False)

