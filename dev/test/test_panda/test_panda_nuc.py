"""Test script for PandaCtrlInterface <-> RealPandaInterface

This script is meant to be run on the nuc, may be extended to ws.
"""
from perls2.ros_interfaces.panda_keys import PandaKeys
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface

P = PandaKeys('cfg/franka-panda.yaml')
# 1. Fake franka-panda redis driver set up.
