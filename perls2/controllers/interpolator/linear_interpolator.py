from __future__ import division
from perls2.controllers.interpolator.base_interpolator import Interpolator
import numpy as np


class LinearInterpolator(Interpolator):
    '''
    Simple class for implementing a linear interpolator.

    Abstracted to interpolate n-dimensions

    Args:
        max_dx: Maximum single change in dx allowed by the system.
                Note that this should be in units distance / second
        ndim: Number of dimensions to interpolate
        controller_freq: Frequency (Hz) of the controller
        policy_freq: Frequency (Hz) of the policy model
        ramp_ratio: Percentage of interpolation timesteps across which we will interpolate to a goal position.
            Note: Num total interpolation steps will be equal to np.floor(ramp_ratio * controller_freq / policy_freq)
                    i.e.: how many controller steps we get per action space update
    '''
    def __init__(self, max_dx, ndim, controller_freq, policy_freq, ramp_ratio=0.2):
        self.max_dx = max_dx                                       # Maximum allowed change per interpolator step
        self.goal = None                                           # Requested goal
        self.dim = ndim                                            # Number of dimensions to interpolate
        self.order = 1                                             # Order of the interpolator (1 = linear)
        self.step = 0                                              # Current step of the interpolator
        self.total_steps = \
            np.floor(ramp_ratio * float(controller_freq) / float(policy_freq))   # Total num steps per interpolator action
        # Save previous goal
        self.prev_goal = None

    '''
    set_goal: Takes a requested goal and updates internal parameters for next interpolation step
    Args:
        goal: Requested goal. Should be same dimension as self.dim
    Returns:
        None
    '''
    def set_goal(self, goal):
        # First, check to make sure requested goal shape is the same as self.dim
        if goal is list: 
            goal = np.asarray(goal)

        if len(goal) != self.dim:
            raise ValueError("LinearInterpolator: Input size wrong for goal; needs to be {}!".format(self.dim))

        # Set initial goal for smoothing.
        if self.prev_goal is None:
            self.prev_goal = np.array(goal)
        else:
            self.prev_goal = self.goal
        # Update goal and reset interpolation step
        self.goal = np.array(goal)

        self.step_num = 1

    '''
    get_interpolated_goal: Provides interpolated goal linearly interpolated between previous
        goal and current goal.
    Args:
        None
    Returns:
        interp_goal: Next position in the interpolated trajectory
    '''
    def get_interpolated_goal(self):

        # Also make sure goal has been set
        if self.goal is None:
            raise ValueError("LinearInterpolator: Goal has not been set yet!")

        # Calculate the desired next step based on remaining interpolation steps and increment step if necessary
        if self.step_num < self.total_steps:
            dx = (self.goal - self.prev_goal) / (self.total_steps)
            # Check if dx is greater than max value; if it is; clamp and notify user
            if np.any(abs(dx) > self.max_dx):
                dx = np.clip(dx, -self.max_dx, self.max_dx)

            interp_goal = self.prev_goal + np.multiply(self.step_num + 1, dx)
            self.step_num += 1
        else:
            # We've already completed all interpolation steps; return goal
            interp_goal = self.goal

        # Return the new interpolated step
        return interp_goal



    







