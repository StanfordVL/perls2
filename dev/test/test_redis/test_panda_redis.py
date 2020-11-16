"""Test script for panda specific redis interface. 
"""

import pytest
import redis
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.redis_values import *
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
P = PandaKeys('cfg/franka-panda.yaml')


@pytest.fixture
def panda_redis():
	panda_redis = PandaRedisInterface(host='127.0.0.1', port=6379)
	return panda_redis

def test_env_connected(panda_redis):
	 panda_redis._client.set(ROBOT_ENV_CONN_KEY, 'True')
	 assert(panda_redis.is_env_connected())

	 # test other values than true. 
	 panda_redis._client.set(ROBOT_ENV_CONN_KEY, 'NOT TRUE')
	 assert(panda_redis.is_env_connected() == False)
