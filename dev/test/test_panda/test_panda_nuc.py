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
import numpy as np
import time
from perls2.redis_interfaces.redis_keys import *
from perls2.redis_interfaces.redis_values import *
from perls2.ctrl_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.robots.real_panda_interface import RealPandaInterface

from perls2.redis_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig

from dev.test.test_panda.fake_franka_panda import FakeFrankaPanda
from dev.test.test_panda.fake_real_panda import FakePandaInterface

@pytest.fixture
def fake_driver():
	driver = FakeFrankaPanda()
	return driver

@pytest.fixture
def panda_ctrl():
	panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml', controlType=None)
	return panda_ctrl

@pytest.fixture
def real_panda():
	real_panda = FakePandaInterface()
	return real_panda



def test_get_driver_state_model(fake_driver, panda_ctrl):
	fake_driver.set_fake_state()
	np_states = panda_ctrl.redisClient.get_driver_state_model()

	for key, state in np_states.items():
		assert(isinstance(state, np.ndarray))
		if key == P.ROBOT_STATE_EE_POSE_KEY:
			assert(state.shape == (4,4))
		elif key == P.ROBOT_MODEL_MASS_MATRIX_KEY:
			assert(state.shape == (7,7))
		elif key == P.ROBOT_MODEL_JACOBIAN_KEY:
			assert(state.shape == (6,7))
		else:
			assert(state.shape == (7,))

def test_update_model(fake_driver, panda_ctrl):
	fake_driver.set_fake_state()
	panda_ctrl.update_model()
	assert len(panda_ctrl.model.ee_pos) == 3
	assert panda_ctrl.model.ee_ori_mat.shape == (3,3)
	assert panda_ctrl.model.ee_ori_quat.shape == (4,)
	assert panda_ctrl.model.joint_pos.shape == (7,)
	assert panda_ctrl.model.joint_vel.shape == (7,)
	assert panda_ctrl.model.joint_tau.shape == (7,)
	assert panda_ctrl.model.ee_pos_vel.shape == (3,)
	assert panda_ctrl.model.ee_ori_vel.shape == (3,)
	assert panda_ctrl.model.J_pos.shape == (3, 7)
	assert panda_ctrl.model.J_ori.shape == (3, 7)

def test_step(fake_driver, panda_ctrl, real_panda):
	fake_driver.set_fake_state()
	# Set up command to maintain ee_pose.
	panda_ctrl.update_model()
	panda_ctrl.controller = panda_ctrl.make_controller_from_redis(panda_ctrl.get_control_type(), panda_ctrl.get_controller_params())

	curr_ee_pos = panda_ctrl.model.ee_pos
	curr_ee_ori = panda_ctrl.model.ee_ori_quat

	real_panda.set_ee_pose(set_pos=curr_ee_pos, set_ori=curr_ee_ori)
	panda_ctrl.process_cmd(bytes(SET_EE_POSE, 'utf-8'))
	panda_ctrl.step(time.time())
