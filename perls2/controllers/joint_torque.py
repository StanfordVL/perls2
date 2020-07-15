from perls2.controllers.base_controller import Controller
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np

class JointTorqueController(Controller):
    """
    Controller for joint torque
    """

    def __init__(self,
                 robot_model,
                 input_max=1,
                 input_min=-1,
                 output_max=1.0,
                 output_min=-1.0,
                 control_freq=20,
                 interpolator=None,
                 ):

        super(JointTorqueController, self).__init__()
        # input and output max and min
        self.input_max = input_max
        self.input_min = input_min
        self.output_max = output_max
        self.output_min = output_min

        # joint dimension (action space for control)
        self.joint_dim = robot_model.joint_dim

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator = interpolator

        # initialize torques
        self.goal_torque = None                         # Goal torque desired, pre-compensation
        self.current_torque = np.zeros(self.joint_dim)  # Current torques being outputted, pre-compensation
        self.torques = None                             # Torques returned every time run_controller is called

        self.set_goal(np.zeros(self.joint_dim))

    def set_goal(self, torques):
        self.model.update()

        # Check to make sure delta is size self.joint_dim
        assert len(torques) == self.joint_dim, "Torque length must be equal to the robot's joint dimension space!"

        self.goal_torque = self.scale_action(torques)
        if self.interpolator is not None:
            self.interpolator.set_goal(self.goal_torque)

    def run_controller(self, action=None):
        # First, update goal if action is not set to none
        # Action will be interpreted as delta value from current
        if action is not None:
            self.set_goal(action)
        else:
            self.model.update()

        # Next, check whether goal has been set
        assert self.goal_torque is not None, "Error: Joint torque goal has not been set yet!"

        # Only linear interpolator is currently supported
        if self.interpolator is not None:
            if self.interpolator.order == 1:
                # Linear case
                self.current_torque = self.interpolator.get_interpolated_goal(self.current_torque)
            else:
                # Nonlinear case not currently supported
                pass
        else:
            self.current_torque = np.array(self.goal_torque)

        # Add torque compensation
        self.torques = self.current_torque + self.model.torque_compensation

        # Return final torques
        return self.torques
