from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np
import time

class EEPostureController(EEImpController):
	""" Class definition for end effector impedance controller with posture control. 

	End effector uses pd control, with a desired joint position in null space.
	
	Attributes:
		posture (list): 7f joint positions to maintain within nullspace of task. 
		posture_gain (float): constant gain to control posture. 

	"""



	def set_goal(self, delta, posture=[0,-1.18,0.00,2.18,0.00,0.57,3.3161], 
		set_pos=None, set_ori=None, **kwargs):
		super().set_goal(delta, set_pos, set_ori)
		self.goal_posture = posture

	def run_controller(self): 
		""" Adds torque to compensate for posture control within nullspace of task.
		"""
		torques = super().run_controller()
		
		posture_error = np.array(self.goal_posture) - np.array(self.model.joint_pos)
		self.posture_gain = 10
		torques = torques +  np.dot(np.array([self.posture_gain] *7), posture_error)
		return torques