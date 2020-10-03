from __future__ import division
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
import perls2.controllers.utils.transform_utils as T
import numpy as np


class LinearOriInterpolator(LinearInterpolator):
    def __init__(self, controller_freq=500, policy_freq=20, fraction=0.2, **kwargs):
        self.prev_interp_goal = None
        self.total_steps = np.floor((float(controller_freq) / float(policy_freq)) *0.2)
        print("total_steps")

        self.prev_goal = None
        self.goal = None
        self.fraction = fraction
        self.step = 1
        self.order = 1

    def set_goal(self, goal):
        if (goal.shape[0] != 4):
            raise ValueError("Incorrect goal dimension for orientation interpolator.")
        # Update goal and reset interpolation step
        if self.prev_goal is None:
            self.prev_goal = np.array(goal)
        else: 
            self.prev_goal = goal
        self.goal = np.array(goal)
        self.step = 1



    def get_interpolated_goal(self):
        """ Get interpolated orientation using slerp.
        """
        # Also make sure goal has been set
        if self.goal is None:
            raise ValueError("LinearOriInterpolator: Goal has not been set yet!")

        if self.step <= self.total_steps:
            # if self.prev_interp_goal is None:
            #     interp_goal = T.quat_slerp( self.prev_goal, self.goal, fraction=self.fraction)
            #     self.prev_interp_goal = interp_goal
            # else: 
            #     interp_goal = T.quat_slerp(self.prev_interp_goal, self.goal,  fraction=self.fraction)
            interp_fraction = (self.step/self.total_steps)*self.fraction
            print("interp_fraction: {}/{}".format(interp_fraction, self.fraction))

            interp_goal = T.quat_slerp(self.prev_goal, self.goal, fraction=interp_fraction)
            self.step+=1
        else:
            interp_goal = self.goal
        return interp_goal