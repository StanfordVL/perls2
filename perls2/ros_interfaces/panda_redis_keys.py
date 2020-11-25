"""Constant keys for franka-panda redis driver. 
"""
from perls2.utils.yaml_config import YamlConfig 

class PandaKeys(object):

	def __init__(self, config='cfg/franka-panda.yaml'): 
		"""get key constants from yaml config for franka-panda. 
		
			config (str): fpath for the franka-panda config file.
		"""
		self.config = YamlConfig(config)
		self.yaml_keys = self.config['redis']['keys']
		self.perls2_keys = self.config['perls2_redis']
		self.PREFIX = self.yaml_keys['prefix']

		self.TORQUE_CMD_KEY = self._get_full_key('tau_des')
		self.CONTROL_MODE_KEY = self._get_full_key('control_mode')
		self.TORQUE_CTRL_MODE = "torque"
		self.FLOAT_CTRL_MODE = "floating"
		self.RESET_CTRL_MODE = "reset"
		self.IDLE_CTRL_MODE = "idle"
		
		self.GRIPPER_WIDTH_CMD_KEY = self._get_full_key('gripper_width_des')
		self.GRIPPER_SPEED_CMD_KEY = "franka_panda::gripper::control::speed"
		self.GRIPPER_FORCE_CMD_KEY = "franka_panda::gripper::control::force"
		self.GRIPPER_MODE_KEY = self._get_full_key('gripper_mode')

		self.ROBOT_STATE_Q_KEY = self._get_full_key('q')
		self.ROBOT_STATE_DQ_KEY = self._get_full_key('dq')
		self.ROBOT_STATE_EE_POSE_KEY = self._get_full_key('pose')
		self.ROBOT_STATE_TAU_KEY = self._get_full_key('tau')
		self.ROBOT_MODEL_MASS_MATRIX_KEY = self._get_full_key('mass_matrix')
		self.ROBOT_MODEL_JACOBIAN_KEY = self._get_full_key('jacobian')
		self.ROBOT_MODEL_GRAVITY_KEY = self._get_full_key('gravity')
		self.ROBOT_MODEL_CORIOLIS_KEY = self._get_full_key('coriolis')

		
		self.ROBOT_STATE_KEYS = [
			self.ROBOT_STATE_Q_KEY, 
			self.ROBOT_STATE_DQ_KEY,
			self.ROBOT_STATE_EE_POSE_KEY, 
			self.ROBOT_STATE_TAU_KEY, 
			]

		self.ROBOT_MODEL_KEYS = [
			self.ROBOT_MODEL_MASS_MATRIX_KEY, 
			self.ROBOT_MODEL_JACOBIAN_KEY, 
			self.ROBOT_MODEL_GRAVITY_KEY, 
			self.ROBOT_MODEL_CORIOLIS_KEY]

		self.DRIVER_CONN_KEY = "franka_panda::driver::status"

		self.DRIVER_CONNECTED_VALUE = bytes("running", 'utf-8')
		self.DRIVER_DISCONN_VALUE = bytes("off", 'utf-8')

		# Shapes for driver model and states. 
		self.MASS_MATRIX_SHAPE = (7,7)
		self.JACOBIAN_SHAPE = (6,7)
		self.EE_POSE_SHAPE = (4,4)

	def _get_full_key(self, key):
		if key in self.yaml_keys.keys():
			return self.PREFIX + self.yaml_keys[key]
		elif key in self.perls2_keys.keys():
			return self.PREFIX + self.perls2_keys[key]		
		else:
			raise KeyError("key not found. Check default.yaml")


if __name__ == '__main__':
	keys = PandaKeys()
	assert(keys.TORQUE_CMD_KEY == "franka_panda::control::tau")
	assert(keys.CONTROL_MODE_KEY == "franka_panda::control::mode")
	assert(keys.ROBOT_STATE_Q_KEY == "franka_panda::sensor::q")