"""Script to time panda ctrl interface.

Runs set torques commands for very small values.
Consecutive set_torque commands must be different or the
driver will register them as stale.
"""
from perls2.ctrl_interfaces.panda_ctrl_interface import PandaCtrlInterface
import numpy as np
import timeit


type_switch  = True
zero_torques_pos = np.array([0.00001] *7)
zero_torques_neg = np.array([-0.00001]*7)
assert(np.all(np.abs(zero_torques_pos) < 0.001))
assert(np.all(np.abs(zero_torques_neg) < 0.001))

panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml', controlType=None)
print("Setting random torques.")


try:
	while True:
		zero_torques = (np.random.rand(7) - 0.5)*0.00001
		panda_ctrl.set_torques(zero_torques)
		# if type_switch:
		# 	panda_ctrl.set_torques(zero_torques_pos)
		# else:
		# 	panda_ctrl.set_torques(zero_torques_neg)
		# type_switch = not type_switch
except KeyboardInterrupt:
	print("keyboard interrupt")