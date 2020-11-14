""" Test script for real panda control
"""
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
import numpy as np

# Test setting torques changes the control mode of the franka redis driver. 
panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml', controlType=None)
panda_ctrl.set_torques(np.array([0.0001]*7))
#panda_ctrl.set_to_float()

