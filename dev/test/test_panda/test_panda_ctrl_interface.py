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

@pytest.fixture
def real_panda_config():
    """ Test config as to be used by RealPandaInterface.
    """
    config = YamlConfig('dev/test/test_panda/test_panda_cfg.yaml')
    return config

def test_action_set_false_on_init(panda_ctrl):
    """ safety test: action_set flag is false on init.
    """
    panda_ctrl = panda_ctrl
    assert(panda_ctrl.action_set == False)

def test_num_joints(panda_ctrl):
    """test that number of joints is 7
    """
    panda_ctrl = panda_ctrl
    assert(panda_ctrl.num_joints == 7)


def test_get_neutral_position(panda_ctrl):
    """Check neutral joint position set on init. This is magic value.
    """
    panda_ctrl = panda_ctrl
    assert(panda_ctrl.neutral_joint_position == \
        [0.0, -0.524, 0.0, -2.617, 0.0, 2.094, 0.0])

def test_
def test_init(panda_ctrl):
    """Test initialization for panda_ctrl.

    Must pass before next functions will work.
    """
    pass

# Command tests
def test_set_torques(panda_ctrl):
    """ check set torques works for list commands
    """
    panda_ctrl = panda_ctrl
    panda_ctrl.set_torques([0, 0, 0, 0, 0, 0, 0])

    # Check torques set to proper format, control mode
    # set to torque mode.
    assert(panda_ctrl.redisClient.get(P.TORQUE_CMD_KEY) ==
        b"0 0 0 0 0 0 0")
    assert(panda_ctrl.redisClient.get(P.CONTROL_MODE_KEY) ==
        bytes(P.TORQUE_CTRL_MODE, 'utf-8'))

    # incorrect torques dim results in error.
    with pytest.raises(AssertionError):
        panda_ctrl.set_torques([0, 0, 0])

    # torques accepted as ndarray.
    panda_ctrl.set_torques(np.asarray([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
    assert(panda_ctrl.redisClient.get(P.TORQUE_CMD_KEY) ==
        b'0.1 0.1 0.1 0.1 0.1 0.1 0.1')
    assert(panda_ctrl.redisClient.get(P.CONTROL_MODE_KEY) ==
        bytes(P.TORQUE_CTRL_MODE, 'utf-8'))

def test_make_controller_from_redis(panda_ctrl, real_panda_config):
    panda_ctrl = panda_ctrl
    real_config = real_panda_config

    control_type = EE_IMPEDANCE
    control_params = real_config['controller']['Real']['EEImpedance']
    controller = panda_ctrl.make_controller_from_redis(control_type=control_type,
        controller_dict=control_params)
    # check control type changed.
    assert(panda_ctrl.controlType == control_type)
    # Check controller parameters.

