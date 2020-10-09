from perls2.controllers.ee_imp import EEImpController
import perls2.controllers.utils.control_utils as C
import perls2.controllers.utils.transform_utils as T
import numpy as np


class EEPostureController(EEImpController):
    """ Class definition for End-effector Impedance Controller.

    End effector impedance uses PD control to reach desired end-effector
    position and orientation, torques are projected to nullspace to maintain posture.

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

        posture_gain (list): gains for determining torques to maintiain desired joint posture.
        posture (list): 7f joint positions for desired nullspace posture to maintain.
        control_freq (float): frequency at which controller sends to robot.

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
                 posture_gain=0,
                 posture=[0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161],
                 policy_freq=20,
                 control_freq=500,
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=False,
                 **kwargs):
        """ Initialize controller.
        """
        self.posture_gain = np.asarray(posture_gain)
        self.goal_posture = np.asarray(posture)
        super(EEPostureController, self).__init__(
            robot_model=robot_model,
            input_max=input_max,
            input_min=input_min,
            output_max=output_max,
            output_min=output_min,
            kp=kp,
            kv=kv,
            damping=damping,
            control_freq=control_freq,
            position_limits=position_limits,
            orientation_limits=orientation_limits,
            interpolator_pos=interpolator_pos,
            interpolator_ori=interpolator_ori,
            uncouple_pos_ori=uncouple_pos_ori)

        # Compile numba jit in advance to reduce initial calc time.
        self._compile_jit_functions()

    def _compile_jit_functions(self):
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

        C.nullspace_torques(
            mass_matrix=self.model.mass_matrix,
            nullspace_matrix=dummy_nullspace_matrix,
            initial_joint=self.goal_posture,
            joint_pos=self.model.joint_pos,
            joint_vel=self.model.joint_vel,
        )

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
        super(EEPostureController, self).set_goal(delta, set_pos, set_ori)

    def run_controller(self):
        """ Run controller to calculate torques to acheive desired pose.

        Args: None
        Return:
            torques (list): 7f list of torques to command.

        Runs impedance controllers and adds nullspace constrained posture compensation.
        """

        torques = super(EEPostureController, self).run_controller()
        self.torques = torques + C.nullspace_torques(self.model.mass_matrix,
                                                     self.nullspace_matrix,
                                                     self.goal_posture,
                                                     self.model.joint_pos,
                                                     self.model.joint_vel,
                                                     self.posture_gain)

        return self.torques
