from perls2.controllers.base_controller import Controller
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np

class JointImpController(Controller):
    """
    Controller for joint impedance
    """

    def __init__(self,
                 robot_model: Model,
                 input_max=1,
                 input_min=-1,
                 output_max= 1.0,
                 output_min=-1.0,
                 kp=50,
                 damping=1,
                 control_freq=20,
                 qpos_limits=None,
                 interpolator_qpos=None,
                 ):

        super(JointImpController, self).__init__()
        # input and output max and min
        self.input_max = input_max
        self.input_min = input_min
        self.output_max = output_max
        self.output_min = output_min

        # limits
        self.position_limits = qpos_limits

        # joint dimension (action space for control)
        self.joint_dim = robot_model.joint_dim

        # kp kv
        self.kp = np.ones(self.joint_dim) * kp
        self.kv = np.ones(self.joint_dim) * 2 * np.sqrt(self.kp) * damping

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator_qpos = interpolator_qpos

        # initialize
        self.goal_qpos = None

        self.set_goal(np.zeros(self.joint_dim))

    def set_goal(self, delta, set_qpos=None):
        self.model.update()

        if delta is not None:
            # Check to make sure delta is size self.joint_dim
            assert len(delta) == self.joint_dim,\
                "Delta length must be equal to the robot's joint dimension space! Expected {}, got {}".format(
                    self.joint_dim, len(delta)
                )
            scaled_delta = self.scale_action(delta)
        else:
            # Otherwise, check to make sure set_velocity is size self.joint_dim
            assert len(set_qpos) == self.joint_dim,\
                "Goal action must be equal to the robot's joint dimension space! Expected {}, got {}".format(
                    self.joint_dim, len(set_qpos)
                )
            scaled_delta = None

        self.goal_qpos = set_goal_position(scaled_delta,
                                          self.model.joint_pos,
                                          position_limit=self.position_limits,
                                          set_pos=set_qpos)

        if self.interpolator_qpos is not None:
            self.interpolator_qpos.set_goal(self.goal_qpos)

    def run_controller(self, action=None):
        # First, update goal if action is not set to none
        # Action will be interpreted as delta value from current
        if action is not None:
            self.set_goal(action)
        else:
            self.model.update()

        # Next, check whether goal has been set
        assert self.goal_qpos is not None, "Error: Joint qpos goal has not been set yet!"

        desired_qpos = 0
        desired_vel_pos = 0 # Note that this is currently unused
        desired_acc_pos = 0 # Note that this is currently unused

        if self.interpolator_qpos is not None:
            if self.interpolator_qpos.order == 1:
                # Linear case
                desired_qpos = self.interpolator_qpos.get_interpolated_goal(self.model.joint_pos)
            else:
                # Nonlinear case not currently supported
                pass
        else:
            desired_qpos = np.array(self.goal_qpos)

        position_error = desired_qpos - self.model.joint_pos
        vel_pos_error = desired_vel_pos - self.model.joint_vel
        desired_torque = (np.multiply(np.array(position_error), np.array(self.kp))
                         + np.multiply(vel_pos_error, self.kv)) + desired_acc_pos

        self.torques = np.dot(self.model.mass_matrix, desired_torque) + self.model.torque_compensation

        return self.torques