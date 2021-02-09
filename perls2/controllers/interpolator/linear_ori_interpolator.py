from __future__ import division
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
import perls2.controllers.utils.transform_utils as T
import numpy as np


class LinearOriInterpolator(LinearInterpolator):
    """ SLERP Interpolator for orientation.

    Interpolates between previous goal commands to produce smooth trajectory.

    Attributes:
        total_steps (int): number of control steps per policy step.
        prev_goal (list): 4f previous goal commanded to controller.
        goal (list): 4f previous goal commanded to controller
        fraction (float): fraction of path to interpolate between goals
        step (int): num steps interpolated

    Notes:
        This does not interpolate between robot state and goal, but between
        previous controller goal commands.

    """
    def __init__(self, controller_freq=500, policy_freq=20, fraction=0.2, **kwargs):
        """ Initialize interpolator.

        Args:
            controller_freq (float): Frequency (Hz) of the controller
            policy_freq (float): Frequency (Hz) of the policy model
            fraction (float): 0 to 1 fraction of path to interpolate.

        """
        self.total_steps = np.floor((float(controller_freq) / float(policy_freq)) * 0.2)
        self.prev_goal = None
        self.goal = None
        self.fraction = fraction
        self.step = 1

    def set_goal(self, goal):
        """ Set goal for linear interpolator

        Args:
            goal (list): 4f goal orientation expressed as quaternion (absolute) in world frame.
        """
        if (goal.shape[0] != 4):
            raise ValueError("Incorrect goal dimension for orientation interpolator.")
        # Update goal and reset interpolation step
        if self.prev_goal is None:
            self.prev_goal = np.array(goal)
        else:
            self.prev_goal = self.goal
        self.goal = np.array(goal)
        self.step = 1

    def get_interpolated_goal(self):
        """ Get interpolated orientation using slerp.
        """
        # Also make sure goal has been set
        if self.goal is None:
            raise ValueError("LinearOriInterpolator: Goal has not been set yet!")

        if self.step <= self.total_steps:
            interp_fraction = (self.step / self.total_steps) * self.fraction
            interp_goal = T.quat_slerp(self.prev_goal, self.goal, fraction=interp_fraction)
            self.step += 1
        else:
            interp_goal = self.goal
        return interp_goal
