import abc
import numpy as np


class Controller(object):
    """
    General controller interface.
    """
    def __init__(self):
        self.action_scale = None

        # Note: the following attributes must be specified in extended controller subclasses during the init call:
        # self.output_min
        # self.output_max
        # self.input_min
        # self.input_max

    @abc.abstractmethod
    def run_controller(self, action):
        """
        Go from actions to torques
        """
        pass

    def scale_action(self, action):
        """
        Scale the action based on max and min of action
        """

        if self.action_scale is None:
            self.action_scale = abs(self.output_max - self.output_min) / abs(self.input_max - self.input_min)
            self.action_output_transform = (self.output_max + self.output_min) / 2.0
            self.action_input_transform = (self.input_max + self.input_min) / 2.0
        action = np.clip(action, self.input_min, self.input_max)
        transformed_action = (action - self.action_input_transform) * self.action_scale + self.action_output_transform

        return transformed_action



