"""
Abstract class defining the interface to the objects.
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions


@six.add_metaclass(abc.ABCMeta)
class ObjectInterface():
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self):
        """
        Initialize variables

        Parameters
        ----------

        """

    def get_position(self):
        """ get position in world coordinates of object
        """
        raise NotImplementedError
