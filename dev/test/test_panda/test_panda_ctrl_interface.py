"""Unit tests for PandaCtrlInterface
"""
import pytest
import numpy as np

from perls2.utils.yaml_config import YamlConfig

import perls2.ros_interfaces.panda_redis_keys as P
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface

import logging
logging.basicConfig(level=logging.DEBUG)
from dev.test.test_panda.fake_franka_panda import  FakeFrankaPanda

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

def test_init_with_config_dict():
    """test initialization with config as dict. 
    """
    config = YamlConfig('cfg/panda_ctrl_config.yaml')
    try:
        panda = PandaCtrlInterface(config=config, controlType='EEImpedance')
    except TypeError:
        raise pytest.fail("PandaCtrlInterface init with config dict failed")


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

# Command tests
def test_set_torques(panda_ctrl):
    """ check set torques works for list commands
    """
    panda_ctrl = panda_ctrl
    panda_ctrl.set_torques(np.array([0, 0, 0, 0, 0, 0, 0]))

    # Check torques set to proper format, control mode
    # set to torque mode.
    assert(panda_ctrl.redisClient.get(P.TORQUE_CMD_KEY) ==
        b"0 0 0 0 0 0 0")
    assert(panda_ctrl.redisClient.get(P.CONTROL_MODE_KEY) ==
        bytes(P.TORQUE_CTRL_MODE, 'utf-8'))

    # torques accepted as ndarray.
    panda_ctrl.set_torques(np.asarray([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
    assert(panda_ctrl.redisClient.get(P.TORQUE_CMD_KEY) ==
        b'0.1 0.1 0.1 0.1 0.1 0.1 0.1')
    assert(panda_ctrl.redisClient.get(P.CONTROL_MODE_KEY) ==
        bytes(P.TORQUE_CTRL_MODE, 'utf-8'))

@pytest.fixture
def panda_ctrl_states(panda_ctrl):
    panda_ctrl = panda_ctrl
    states = {"ee_pos": np.array([0.3754993949271238, 0.0, 0.31592855773420786]),
              "ee_pos_vel" : np.array([0.0, 0.0, 0.0]),
              "ee_ori": np.array([0.9238783992095897, 0.38268585256980386, 0.000453515926605422, 0.0001898424422551022]),
              "ee_ori_vel": np.array([0.0, 0.0, 0.0]),
              "joint_pos":  np.array([0.0, -0.524, 0.0, -2.617, 0.0, 2.094, 0.0]),
              "joint_vel": np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
              "joint_tau": np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])}

    model = {
        "mass_matrix": np.array([[ 1.99909435e+00, -2.86743928e-02,  1.62201655e+00,
            -5.31061369e-02, -5.84427776e-01, -2.76533333e-02,
            -9.87095446e-01],
           [-2.86743928e-02,  2.24344552e+00, -9.21602111e-02,
            -1.44352426e+00,  1.40439137e-01, -9.01895177e-01,
             1.35257246e-03],
           [ 1.62201655e+00, -9.21602111e-02,  1.92451647e+00,
             1.51692472e-02, -9.51228777e-01,  8.02705635e-02,
            -8.54459727e-01],
           [-5.31061369e-02, -1.44352426e+00,  1.51692472e-02,
             1.80208344e+00, -7.81932060e-02,  9.57014201e-01,
             4.52801334e-02],
           [-5.84427776e-01,  1.40439137e-01, -9.51228777e-01,
            -7.81932060e-02,  1.15766083e+00, -1.66731388e-01,
             5.38257815e-01],
           [-2.76533333e-02, -9.01895177e-01,  8.02705635e-02,
             9.57014201e-01, -1.66731388e-01,  9.82393951e-01,
             2.74518387e-02],
           [-9.87095446e-01,  1.35257246e-03, -8.54459727e-01,
             4.52801334e-02,  5.38257815e-01,  2.74518387e-02,
             1.09269159e+00]]),
        "J_pos": np.array([[-2.28801352e-06,  8.79285070e-02, -1.94273168e-06,
             2.26950949e-01, -2.60771002e-07,  1.06913419e-01,
             0.00000000e+00],
           [ 3.75396150e-01, -7.65265682e-08,  3.69022302e-01,
             2.84110023e-06,  4.87159677e-02,  8.96493434e-07,
             0.00000000e+00],
           [ 1.05879118e-21, -3.75396150e-01, -1.30827416e-06,
             4.62074742e-01, -1.79258491e-07,  8.81051692e-02,
             0.00000000e+00]]),
        "J_ori": np.array([[-1.19262239e-16,  8.70348071e-07, -5.00345626e-01,
             5.28101200e-06,  8.66713793e-01,  5.35289099e-06,
             9.83287170e-04],
           [ 1.69406589e-21,  1.00000000e+00,  4.35475508e-07,
            -1.00000000e+00,  2.80398112e-06, -1.00000000e+00,
            -3.67440525e-06],
           [ 1.00000000e+00,  4.89663865e-12,  8.65825764e-01,
             3.55477024e-06, -4.98805775e-01,  3.67966556e-06,
            -9.99999517e-01]])
        }

    panda_ctrl.model.update_states(**states)
    panda_ctrl.model.update_model(**model)
    return panda_ctrl

def test_update_model_states(panda_ctrl_states):
    panda_ctrl = panda_ctrl_states
    assert(np.all(panda_ctrl.model.ee_pos == [0.3754993949271238, 0.0, 0.31592855773420786]))

def test_make_controller_from_redis(panda_ctrl_states, real_panda_config):
    """test make controller queries redis and property creates correct controller.
    """
    panda_ctrl = panda_ctrl_states
    real_config = real_panda_config

    control_type = EE_IMPEDANCE
    control_params = real_config['controller']['Real']['EEImpedance']
    controller = panda_ctrl.make_controller_from_redis(control_type=control_type,
        controller_dict=control_params)
    # check control type changed.
    assert(panda_ctrl.controlType == control_type)
    # Check each attribute of the controller.
    for key in control_params.keys():
        if key == 'damping':
            pass
        else:
            param = getattr(controller, key)
            assert(np.all(param == np.array(control_params[key])))


# def test_wait_for_env_connect(panda_ctrl):
#     logging.info("MANUAL TEST. SET ENV CONNECT VIA REDIS-CLI")
#     panda_ctrl.wait_for_env_connect()
    
####################################################
# Tests requiring franka-panda.

# @pytest.fixture
# def redis_driver():
#     driver = FakeFrankaPanda()
#     return driver

# @pytest.fixture
# def driver_fake_states():
#     """Driver set up with fake states initialized
#     """
#     driver = FakeFrankaPanda()
#     states = {
#     "q": "0.0 -0.524 0.0 -2.617 0.0 2.094 0.0",
#     "dq": "0.01 0.01 0.01 0.01 0.01 0.01 0.01",
#     "tau": "0.01 0.01 0.01 0.01 0.01 0.01 0.01",
#     "pose": "0.1 0.2 0.3 0.0 0.0 0.0 1.0"
#     }
#     driver.set_states(states)
#     return driver

# def test_update_states(driver_fake_states, panda_ctrl):
#     """Test Ctrl Interface updates internal model correctly from redis.
#     """
#     redis_driver = driver_fake_states
#     panda_ctrl = panda_ctrl
#     assert(panda_ctrl.model.ee_pos == [0.1, 0.2, 0.3])
#     assert(panda_ctrl.model.ee_ori_quat == [0.0, 0.0, 0.0, 1.0])
#     assert(panda_ctrl.model.joint_pos == [0.0, -0.524, 0.0, -2.617, 0.0, 2.094, 0.0])
#     assert(panda_ctrl.model.joint_vel == [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
#     assert(panda_ctrl.model.joint_tau == [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])



# def test_make_controller_from_redis(panda_ctrl, real_panda_config):
#     panda_ctrl = panda_ctrl
#     real_config = real_panda_config

#     control_type = EE_IMPEDANCE
#     control_params = real_config['controller']['Real']['EEImpedance']
#     controller = panda_ctrl.make_controller_from_redis(control_type=control_type,
#         controller_dict=control_params)
#     # check control type changed.
#     assert(panda_ctrl.controlType == control_type)
#     # Check controller parameters.

