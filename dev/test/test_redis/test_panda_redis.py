"""Test script for panda specific redis interface. 
"""

import pytest
import redis
import numpy as np
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from dev.test.test_panda.fake_franka_panda import FakeFrankaPanda

P = PandaKeys('cfg/franka-panda.yaml')


@pytest.fixture
def panda_redis():
	panda_redis = PandaRedisInterface(host='127.0.0.1', port=6379)
	return panda_redis

@pytest.fixture
def fake_driver():
	fake_driver = FakeFrankaPanda()
	fake_driver.set_fake_state()
	return fake_driver

def test_env_connected(panda_redis):
	 panda_redis._client.set(ROBOT_ENV_CONN_KEY, 'True')
	 assert(panda_redis.is_env_connected())

	 # test other values than true. 
	 panda_redis._client.set(ROBOT_ENV_CONN_KEY, 'NOT TRUE')
	 assert(panda_redis.is_env_connected() == False)

def test_get_driver_states(panda_redis, fake_driver):
	driver_states = panda_redis.get_driver_states()
	for key, state in driver_states.items():
		if key == P.ROBOT_STATE_EE_POSE_KEY:
			assert(driver_states[key].shape == (4,4))
			# make sure properly imported (column-major).
			# last row should be 0, 0, 0, 1
			assert np.all(driver_states[key][3, :] == [0, 0, 0, 1])

		else:
			assert(driver_states[key].shape == (7,))

