from __future__ import division
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
import perls2.controllers.utils.transform_utils as T
import numpy as np


class LinearOriInterpolator(LinearInterpolator):

    def set_goal(self, goal):
        if (goal.shape[0] != 4):
            raise ValueError("Incorrect goal dimension for orientation interpolator.")
        # Update goal and reset interpolation step
        self.goal = np.array(goal)
        self.step = 0


    def get_interpolated_goal(self, current_ori):
        """ Get interpolated orientation using slerp.
        """
        # Also make sure goal has been set
        if self.goal is None:
            raise ValueError("LinearOriInterpolator: Goal has not been set yet!")

        if self.step < self.total_steps:
            interp_goal = T.quat_slerp(current_ori, self.goal, fraction=0.2)
            self.step+=1
        else:
            interp_goal = self.goal
        return interp_goal