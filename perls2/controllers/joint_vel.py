from perls2.controllers.base_controller import Controller
import numpy as np


class JointVelController(Controller):
    """
    Controller for joint velocity
    """

    def __init__(self,
                 robot_model,
                 input_max=1,
                 input_min=-1,
                 output_max=1,
                 output_min=-1,
                 kv=4.0,
                 control_freq=20,
                 interpolator=None,
                 ):

        super(JointVelController, self).__init__()
        # input and output max and min
        self.input_max = input_max
        self.input_min = input_min
        self.output_max = output_max
        self.output_min = output_min

        # joint dimension (action space for control)
        self.joint_dim = robot_model.joint_dim

        # kv
        self.kv = np.ones(self.joint_dim) * kv

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator = interpolator

        # initialize
        self.goal_vel = None

        self.set_goal(np.zeros(self.joint_dim))

    def set_goal(self, velocities, **kwargs):
        """ Set goal joint velocities

        Args:
            velocities (list): 7f list of joint velocities to command to the robot.
            kwargs (dict): additional keyword arguments.
        Returns:
            None
        """
        self.model.update()

        self.goal_vel = self.scale_action(velocities)

        if self.interpolator is not None:
            self.interpolator.set_goal(self.goal_vel)

    def run_controller(self):
        """ Run controller to calculate torques.

        Returns:
            torques (list): 7f list of torques to command.
        """
        assert self.goal_vel is not None, "Error: Joint velocity goal has not been set yet!"

        # Only linear interpolator is currently supported
        if self.interpolator is not None:
            if self.interpolator.order == 1:
                # Linear case
                desired_vel = self.interpolator.get_interpolated_goal()
            else:
                # Nonlinear case not currently supported
                pass
        else:
            desired_vel = np.array(self.goal_vel)

        # Compute torques (pre-compensation)
        self.torques = np.multiply(self.kv, (desired_vel - self.model.joint_vel)) + self.model.torque_compensation

        # Return final torques
        return self.torques
