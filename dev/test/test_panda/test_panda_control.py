""" Test script for real panda control
"""
from perls2.ctrl_interfaces.panda_ctrl_interface import PandaCtrlInterface
from dev.test.test_panda.fake_real_panda import FakePandaInterface
import perls2.redis_interfaces.panda_redis_keys as P
import perls2.controllers.utils.transform_utils as T
import numpy as np


real_panda = FakePandaInterface()
real_panda.connect()

# Get initial robot state from driver.
init_states = real_panda.redisClient.get_driver_states()
init_ee_pose = init_states[P.ROBOT_STATE_EE_POSE_KEY]
ee_pos, ee_ori_quat = T.mat2pose(init_ee_pose)


real_panda.set_ee_pose(set_pos=ee_pos, set_ori=ee_ori_quat)
print("set_pos {}".format(ee_pos))
print("set_ori {}".format(ee_ori_quat))

#panda_ctrl.set_to_float()

