from perls2.controllers.base_controller import Controller
import numpy as np


class JointTorqueController(Controller):
    """
    Controller for joint torque

    Attributes:
        robot_model (Model): model of robot containing state and parameters.

        input_max (float or list of floats): Maximum above which an inputted action will be clipped.

        input_min (float or list of floats): Minimum below which an inputted action will be clipped.

        output_max (float or list of float): Maximum which defines upper end of scaling range when scaling an input
            action.
        output_min (float or list of float): Minimum which defines lower end of scaling range when scaling an input
            action.
        interpolator (LinearInterpolator): Interpolator object to be used for interpolating between consecutive goals..
        goal_torque (list of float): goal torques for controller.
        current_torque (list of float): current torque being outputted.
        joint_dim (int): number of joints in the arm (usually 7.)
        torques (list): 7f list of torques.
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
        """ Initialize Joint Torque Controller.

        Args:
            robot_model (Model): model of robot containing state and parameters.

            input_max (float or list of floats): Maximum above which an inputted action will be clipped.

            input_min (float or list of floats): Minimum below which an inputted action will be clipped.

            output_max (float or list of float): Maximum which defines upper end of scaling range when scaling an input
                action.
            output_min (float or list of float): Minimum which defines lower end of scaling range when scaling an input
                action.
            interpolator (LinearInterpolator): Interpolator object to be used for interpolating between consecutive goals..
            goal_torque (list of float): goal torques for controller.
            current_torque (list of float): current torque being outputted.

        """
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

    def set_goal(self, torques, **kwargs):
        """ Set goal torques.

        Args:
            torques (list): 7f list of torques to command to the robot.
            kwargs (dict): additional keyword arguments.

        Returns:
            None
        """
        self.model.update()

        # Check to make sure delta is size self.joint_dim
        assert len(torques) == self.joint_dim, "Torque length must be equal to the robot's joint dimension space!"

        self.goal_torque = self.scale_action(torques)
        if self.interpolator is not None:
            self.interpolator.set_goal(self.goal_torque)

    def run_controller(self):
        """ Run controller to calculate torques.

        Returns:
            torques (list): 7f list of torques to command.
        """

        # Next, check whether goal has been set
        assert self.goal_torque is not None, "Error: Joint torque goal has not been set yet!"

        # Only linear interpolator is currently supported
        if self.interpolator is not None:
            if self.interpolator.order == 1:
                # Linear case
                self.current_torque = self.interpolator.get_interpolated_goal()
            else:
                # Nonlinear case not currently supported
                pass
        else:
            self.current_torque = np.array(self.goal_torque)

        # Add torque compensation
        self.torques = self.current_torque + self.model.torque_compensation

        # Return final torques
        return self.torques
