"""Constant keys for franka-panda redis driver. 
"""
from perls2.utils.yaml_config import YamlConfig 

class PandaKeys(object):

	def __init__(self, config='/home/robot/franka-panda/resources/default.yaml'): 
		"""get key constants from yaml config for franka-panda. 
		
			config (str): fpath for the franka-panda config file.
		"""
		self.config = YamlConfig(config)
		self.yaml_keys = self.config['redis']['keys']
		self.PREFIX = self.yaml_keys['prefix']


		self.TORQUE_CMD_KEY = self._get_full_key('tau_des')
		self.CONTROL_MODE_KEY = self._get_full_key('control_mode')

		self.GRIPPER_WIDTH_CMD_KEY = self._get_full_key('gripper_width_des')
		self.GRIPPER_MODE_KEY = self._get_full_key('gripper_mode')

		self.ROBOT_STATE_Q_KEY = self._get_full_key('q')
		self.ROBOT_STATE_DQ_KEY = self._get_full_key('dq')
		self.ROBOT_STATE_EE_POSE_KEY = self._get_full_key('pose')
		self.ROBOT_STATE_TAU = self._get_full_key('tau')

	def _get_full_key(self, key):
		if key in self.yaml_keys.keys():
			return self.PREFIX + self.yaml_keys[key]
		else:
			raise KeyError("key not found. Check default.yaml")


if __name__ == '__main__':
	keys = PandaKeys()
	assert(keys.TORQUE_CMD_KEY == "franka_panda::control::tau")
	assert(keys.CONTROL_MODE_KEY == "franka_panda::control::mode")
	assert(keys.ROBOT_STATE_Q_KEY == "franka_panda::sensor::q")