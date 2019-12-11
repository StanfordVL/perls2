"""Abstract class for Sensor Interface
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions


@six.add_metaclass(abc.ABCMeta)
class SensorInterface(object):
    """Abstract interface to be implemented for a sensor
    """

    def __init__(self,
                 name):
        """
        Initialize variables

        Parameters
        ----------
            name: name of sensor
        """

    def reset(self):
        """ Reset sensor
        """
        raise NotImplementedError
