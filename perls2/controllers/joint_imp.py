from perls2.controllers.base_controller import Controller
import perls2.controllers.utils.control_utils as C
import numpy as np


class JointImpController(Controller):
    """
    Controller for joint impedance
    """

    def __init__(self,
                 robot_model,
                 input_max=1,
                 input_min=-1,
                 output_max=1.0,
                 output_min=-1.0,
                 kp=50,
                 kv=None,
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
        if kp is list:
            self.kp = np.ones(self.joint_dim) * kp
        else:
            self.kp = kp

        if kv is None:
            self.kv = np.ones(self.joint_dim) * 2 * np.sqrt(self.kp) * damping
        elif kv is list:
            self.kv = kv
        else:
            self.kv = np.ones(self.joint_dim) * kv

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator_qpos = interpolator_qpos

        # initialize
        self.goal_qpos = None
        self.prev_goal = None
        self.set_goal(np.zeros(self.joint_dim))

    def set_goal(self, delta, set_qpos=None, **kwargs):
        """Set goal for controller.
        Args:
            delta (list): (7f) list of deltas from current joint positions
            set_qpos (list): (7f) goal positions for the joints.
            kwargs (dict): additional keyword arguments.

        Returns:
            None

        Examples::
            $ self.controller.set_goal(delta=[0.1, 0.1, 0, 0, 0, 0], set_pos=None, set_ori=None)

            $self.controller.set_goal(delta=None, set_pose=[0.4, 0.2, 0.4], set_ori=[0, 0, 0, 1])
        """
        self.model.update()

        if delta is not None:
            # Check to make sure delta is size self.joint_dim
            assert len(delta) == self.joint_dim,\
                "Delta length must be equal to the robot's joint dimension space! Expected {}, got {}".format(
                    self.joint_dim, len(delta))
            scaled_delta = self.scale_action(delta)
        else:
            # Otherwise, check to make sure set_velocity is size self.joint_dim
            assert len(set_qpos) == self.joint_dim,\
                "Goal action must be equal to the robot's joint dimension space! Expected {}, got {}".format(
                    self.joint_dim, len(set_qpos))
            scaled_delta = None

        self.goal_qpos = C.set_goal_position(scaled_delta,
                                             self.model.joint_pos,
                                             position_limit=self.position_limits,
                                             set_pos=set_qpos)

        if self.interpolator_qpos is not None:
            self.interpolator_qpos.set_goal(self.goal_qpos)

    def run_controller(self):
        """Calculate torques to acheive goal state from current state.

        Args:
            None
        Returns:
            torques (list): 7f list of torques to command.

        """
        # Next, check whether goal has been set
        assert self.goal_qpos is not None, "Error: Joint qpos goal has not been set yet!"

        desired_qpos = 0
        desired_vel_pos = 0  # Note that this is currently unused
        desired_acc_pos = 0  # Note that this is currently unused

        if self.interpolator_qpos is not None:
            if self.interpolator_qpos.order == 1:
                # Linear case
                desired_qpos = self.interpolator_qpos.get_interpolated_goal()
            else:
                # Nonlinear case not currently supported
                pass
        else:
            desired_qpos = np.array(self.goal_qpos)

        position_error = desired_qpos - self.model.joint_pos
        vel_pos_error = desired_vel_pos - self.model.joint_vel
        desired_torque = (
            np.multiply(np.array(position_error), np.array(self.kp)) + np.multiply(vel_pos_error, self.kv)) + desired_acc_pos

        self.torques = np.dot(self.model.mass_matrix, desired_torque) + self.model.torque_compensation

        return self.torques
