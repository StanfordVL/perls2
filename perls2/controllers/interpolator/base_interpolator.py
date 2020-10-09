import abc


class Interpolator(object):
    """
    General interpolator interface.
    """

    @abc.abstractmethod
    def get_interpolated_goal(self, goal):
        """
        Go from actions to torques
        """
        pass
