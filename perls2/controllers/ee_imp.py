from perls2.controllers.base_controller import Controller
import perls2.controllers.utils.control_utils as C
import perls2.controllers.utils.transform_utils as T
import numpy as np


class EEImpController(Controller):
    """ Class definition for End-effector Impedance Controller.

    End effector impedance uses PD control to reach desired end-effector
    position and orientation.

    Attributes:
        input_max (float or list of float): Maximum above which an inputted action will be clipped. Can be either be
            a scalar (same value for all action dimensions), or a list (specific values for each dimension). If the
            latter, dimension should be the same as the control dimension for this controller

        input_min (float or list of float): Minimum below which an inputted action will be clipped. Can be either be
            a scalar (same value for all action dimensions), or a list (specific values for each dimension). If the
            latter, dimension should be the same as the control dimension for this controller

        output_max (float or list of float): Maximum which defines upper end of scaling range when scaling an input
            action. Can be either be a scalar (same value for all action dimensions), or a list (specific values for
            each dimension). If the latter, dimension should be the same as the control dimension for this controller

        output_min (float or list of float): Minimum which defines upper end of scaling range when scaling an input
            action. Can be either be a scalar (same value for all action dimensions), or a list (specific values for
            each dimension). If the latter, dimension should be the same as the control dimension for this controller

        kp (float or list of float): positional gain for determining desired torques based upon the pos / ori errors.
            Can be either be a scalar (same value for all action dims), or a list (specific values for each dim)

        kv (float or list of float): velocity gain for determining desired torques based on vel / ang vel errors.
            Can be either a scalar (same value for all action dims) or list (specific values for each dim).
            If kv is defined, damping is ignored.

        damping (float or list of float): used in conjunction with kp to determine the velocity gain for determining
            desired torques based upon the pos / ori errors. If kv is defined, damping is ignored.

        policy_freq (int): Frequency at which actions from the robot policy are fed into this controller

        position_limits (2-list of float or 2-list of list of floats): Limits (m) below and above which the magnitude
            of a calculated goal eef position will be clipped. Can be either be a 2-list (same min/max value for all
            cartesian dims), or a 2-list of list (specific min/max values for each dim)

        orientation_limits (2-list of float or 2-list of list of floats): [Lower_bounds, upper_bounds]. Limits (rad) below and above which the
            magnitude of a calculated goal eef orientation will be clipped. Can be either be a 2-list
            (same min/max value for all joint dims), or a 2-list of list (specific min/mx values for each dim)

        interpolator_pos (Interpolator): Interpolator object to be used for interpolating from the current position to
            the goal position during each timestep between inputted actions

        interpolator_ori (Interpolator): Interpolator object to be used for interpolating from the current orientation
            to the goal orientation during each timestep between inputted actions

        control_ori (bool): Whether inputted actions will control both pos and ori or exclusively pos

        uncouple_pos_ori (bool): Whether to decouple torques meant to control pos and torques meant to control ori

        **kwargs: Does nothing; placeholder to "sink" any additional arguments so that instantiating this controller
            via an argument dict that has additional extraneous arguments won't raise an error

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
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=False,
                 **kwargs):
        """ Initialize EE Impedance Controller.

        Args:
            robot_model (Model): model of robot containing state and parameters.

            input_max (float or list of floats): Maximum above which an inputted action will be clipped.

            input_min (float or list of floats): Minimum below which an inputted action will be clipped.

            output_max (float or list of float): Maximum which defines upper end of scaling range when scaling an input
                action.

            kp (list): positional gain for determining desired torques based upon the pos / ori errors.
                        Can be either be a scalar (same value for all action dims), or a list (specific values for each dim)
            kv (float or list of float): velocity gain for determining desired torques based on vel / ang vel errors.
                Can be either a scalar (same value for all action dims) or list (specific values for each dim).
                If kv is defined, damping is ignored.

            damping (float or list of float): used in conjunction with kp to determine the velocity gain for determining
                desired torques based upon the pos / ori errors. If kv is defined, damping is ignored.

            policy_freq (int): Frequency at which actions from the robot policy are fed into this controller

            position_limits (2-list of float or 2-list of list of floats): Limits (m) below and above which the magnitude
                of a calculated goal eef position will be clipped. Can be either be a 2-list (same min/max value for all
                cartesian dims), or a 2-list of list (specific min/max values for each dim)

            orientation_limits (2-list of float or 2-list of list of floats): [Lower_bounds, upper_bounds]. Limits (rad) below and above which the
                magnitude of a calculated goal eef orientation will be clipped. Can be either be a 2-list
                (same min/max value for all joint dims), or a 2-list of list (specific min/mx values for each dim)

            interpolator_pos (Interpolator): Interpolator object to be used for interpolating from the current position to
                the goal position during each timestep between inputted actions

            interpolator_ori (Interpolator): Interpolator object to be used for interpolating from the current orientation
                to the goal orientation during each timestep between inputted actions.
        """
        super(EEImpController, self).__init__()
        # input and output max and min
        self.input_max = np.array(input_max)
        self.input_min = np.array(input_min)
        self.output_max = np.array(output_max)
        self.output_min = np.array(output_min)

        # limits
        if position_limits is not None:
            self.position_limits = np.array(position_limits)
        else:
            self.position_limits = position_limits
        self.orientation_limits = orientation_limits

        # kp kv
        # if kp is list:
        #     self.kp = kp
        # else:
        #     self.kp = np.ones(6) * kp
        self.kp = np.array(kp)
        
        # Set kv using damping if kv not explicitly set.
        if kv is not None:
            self.kv = kv
        else:
            self.kv = np.ones(6) * 2 * np.sqrt(self.kp) * damping

        # control frequency
        self.control_freq = control_freq

        # robot model
        self.model = robot_model

        # interpolator
        self.interpolator_pos = interpolator_pos
        self.interpolator_ori = interpolator_ori

        # whether or not pos and ori want to be uncoupled
        self.uncoupling = uncouple_pos_ori

        # Initialize position orientation goals.
        self.goal_ori = None
        self.goal_pos = None

        self.set_goal(np.zeros(6))

        # Run calculations with numba to reduce first call penalties.
        self._compile_jit_functions()

    def _compile_jit_functions(self):
        """
        Helper function to incur the cost of compiling jit functions.
        """
        dummy_mat = np.eye(3)
        dummy_quat = np.zeros(4)
        dummy_quat[-1] = 1.
        T.mat2quat(dummy_mat)
        T.quat2mat(dummy_quat)

        _, _, _, dummy_nullspace_matrix = C.opspace_matrices(
            mass_matrix=self.model.mass_matrix,
            J_full=self.model.J_full,
            J_pos=self.model.J_pos,
            J_ori=self.model.J_ori,
        )

        C.orientation_error(dummy_mat, dummy_mat)

    def set_goal(self, delta, set_pos=None, set_ori=None, **kwargs):
        """ Set goal for controller.

        Args:

            delta (list): (6f) list of deltas from current position
                [dx, dy, dz, ax, ay, az]. Deltas expressed as position and
                axis-angle in orientation.
            set_pos (list): (3f) fixed position goal. [x, y, z] in world
                frame. Only used if delta is None.
            set_ori (list): (4f) fixed orientation goal as quaternion in
                world frame. Only used if delta is None.
            kwargs (dict): additional keyword arguments.

        Returns:
            None

        Examples::
            $ self.controller.set_goal(delta=[0.1, 0.1, 0, 0, 0, 0], set_pos=None, set_ori=None)

            $ self.controller.set_goal(delta=None, set_pose=[0.4, 0.2, 0.4], set_ori=[0, 0, 0, 1])

        """
        # Check args for dims and type (change to ndarray).
        if not (isinstance(set_pos, np.ndarray)) and set_pos is not None:
            set_pos = np.array(set_pos)
        if not (isinstance(set_ori, np.ndarray)) and set_ori is not None:
            if len(set_ori) != 4:
                raise ValueError("invalid ori dimensions, should be quaternion.")
            else:
                set_ori = T.quat2mat(np.array(set_ori))
        # Update the model.

        self.model.update()

        # If using a delta. Scale delta and set position and orientation goals.
        if delta is not None:
            if (len(delta) < 6):
                raise ValueError("incorrect delta dimension")

            scaled_delta = self.scale_action(delta)

            self.goal_ori = C.set_goal_orientation(
                scaled_delta[3:],
                self.model.ee_ori_mat,
                orientation_limit=self.orientation_limits,
                set_ori=set_ori,
                axis_angle=True)

            self.goal_pos = C.set_goal_position(
                scaled_delta[:3],
                self.model.ee_pos,
                position_limit=self.position_limits,
                set_pos=set_pos)
        else:
            # If goal position and orientation are set.
            scaled_delta = None

            self.goal_ori = C.set_goal_orientation(
                None,
                self.model.ee_ori_mat,
                orientation_limit=self.orientation_limits,
                set_ori=set_ori,
                axis_angle=True)

            self.goal_pos = C.set_goal_position(
                None,
                self.model.ee_pos,
                position_limit=self.position_limits,
                set_pos=set_pos)

        # Set goals for interpolators.
        if self.interpolator_pos is not None:
            self.interpolator_pos.set_goal(self.goal_pos)

        if self.interpolator_ori is not None:
            self.interpolator_ori.set_goal(T.mat2quat(self.goal_ori))

    def run_controller(self):
        """ Calculate torques to acheive goal.

        See controllers documentation for more information on EEImpedance
            control. Set goal must be called before running controller.

        Args:
            None
        Returns:
            torques (list): 7f list of torques to command.

        Run impedance controllers.
        """
        # Check if the goal has been set.
        if self.goal_pos is None or self.goal_ori is None:
            raise ValueError("Set goal first.")

        # Default desired velocities and accelerations are zero.
        desired_vel_pos = np.asarray([0.0, 0.0, 0.0])
        desired_acc_pos = np.asarray([0.0, 0.0, 0.0])
        desired_vel_ori = np.asarray([0.0, 0.0, 0.0])
        desired_acc_ori = np.asarray([0.0, 0.0, 0.0])

        # Get interpolated goal for position
        if self.interpolator_pos is not None:
            desired_pos = self.interpolator_pos.get_interpolated_goal()
        else:
            desired_pos = np.array(self.goal_pos)

        # Get interpolated goal for orientation.
        if self.interpolator_ori is not None:

            desired_ori = T.quat2mat(self.interpolator_ori.get_interpolated_goal())
            ori_error = C.orientation_error(desired_ori, self.model.ee_ori_mat)
        else:
            desired_ori = np.array(self.goal_ori)

            ori_error = C.orientation_error(desired_ori, self.model.ee_ori_mat)

        # Calculate desired force, torque at ee using control law and error.
        position_error = desired_pos - self.model.ee_pos
        vel_pos_error = desired_vel_pos - self.model.ee_pos_vel
        desired_force = (
            np.multiply(np.array(position_error),
                        np.array(self.kp[0:3])) + np.multiply(vel_pos_error, self.kv[0:3])) + desired_acc_pos

        vel_ori_error = desired_vel_ori - self.model.ee_ori_vel
        desired_torque = (
            np.multiply(np.array(ori_error),
                        np.array(self.kp[3:])) + np.multiply(vel_ori_error, self.kv[3:])) + desired_acc_ori

        # Calculate Operational Space mass matrix and nullspace.
        lambda_full, lambda_pos, lambda_ori, nullspace_matrix = \
            C.opspace_matrices(self.model.mass_matrix,
                               self.model.J_full,
                               self.model.J_pos,
                               self.model.J_ori)

        self.nullspace_matrix = nullspace_matrix

        # If uncoupling position and orientation use separated lambdas.
        if self.uncoupling:
            decoupled_force = np.dot(lambda_pos, desired_force)
            decoupled_torque = np.dot(lambda_ori, desired_torque)
            decoupled_wrench = np.concatenate([decoupled_force, decoupled_torque])
        else:
            desired_wrench = np.concatenate([desired_force, desired_torque])
            decoupled_wrench = np.dot(lambda_full, desired_wrench)

        # Project torques that acheive goal into task space.
        self.torques = np.dot(self.model.J_full.T, decoupled_wrench) + self.model.torque_compensation

        return self.torques

    def reset_goal(self):
        """ Helper function that resets goal to all zeros from current ee_pose.
        """
        self.set_goal(np.zeros(6))
