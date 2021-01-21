"""Test script for gripper control PandaCtrlInterface
"""

from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
from dev.test.test_panda.fake_real_panda import FakePandaInterface
import perls2.controllers.utils.transform_utils as T 
import numpy as np
import time 

real_panda = FakePandaInterface()
real_panda.connect()

# range should be from 
print("setting gripper value ")
#for _ in range(10000):
real_panda.set_gripper_to_value(0.1)
time.sleep(2)
# time.sleep(5)
real_panda.set_gripper_to_value(0.9)
time.sleep(2)
real_panda.disconnect()