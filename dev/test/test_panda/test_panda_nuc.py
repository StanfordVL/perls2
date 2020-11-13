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

from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.robots.real_panda_interface import RealPandaInterface

from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig
P = PandaKeys('cfg/franka-panda.yaml')

from dev.test.test_panda.fake_franka_panda import FakeFrankaPanda

@pytest.fixture
def fake_driver():
	driver = FakeFrankaPanda()
	return driver

@pytest.fixture
def panda_ctrl():
	panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml', controlType=None)
	return panda_ctrl

# def test_redis_state_to_np(): 
# 	state_str = FakeFrankaPanda.FAKE_STATE[P.ROBOT_STATE_Q_KEY]
# 	state_np = PandaRedisInterface._redis_state_to_np(state_str)
# 	assert(state_np.shape == (7,))

# def test_redis_state_to_np_mat():
# 	state_str = FakeFrankaPanda.FAKE_STATE[P.ROBOT_MODEL_MASS_MATRIX_KEY]
# 	state_np = PandaRedisInterface._redis_state_to_np_mat(state_str, (7,7))
# 	assert(state_np.shape == (7,7))

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


# def test_get_states_from_redis(fake_driver, panda_ctrl)
# 	fake_driver = fake_driver
# 	fake_driver.set_fake_states()

# 	panda_ctrl = panda_ctrl
# 	states = panda_ctrl._get_states_from_redis()





if __name__ == '__main__':
    test_fake_redis_driver()
    test_panda_ctrl_setup()
    test_real_panda_setup()