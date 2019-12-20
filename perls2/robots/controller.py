""" Controller class definitions
"""

import numpy as np
import scipy


class Controller():
	""" Abstract Class for defining Controllers. This class takes a robot state and returns a set of torques to be commanded.

	Attributes:
		control_type (str) : type of controller ('osc','joint space')
	"""

	def __init__(self, control_type):
		""" Initializate the controller
		"""
		self.type = control_type


class OperationalSpaceController():
	""" Class for defining operational space controllers. This class takes in a robot state and returns a set of torques to be commanded.

	Attributes:
		control_type (str): type of controller ('osc')
	"""



