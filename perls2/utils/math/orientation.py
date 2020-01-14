"""Orientation in the 3D space.

Author: Kuan Fang
"""

import numpy as np

from perls2.utils.math.transformations import (
        euler_from_quaternion,
        euler_from_matrix3,
        quaternion_from_euler,
        quaternion_from_matrix3,
        matrix3_from_euler,
        matrix3_from_quaternion,
)


class Orientation(object):
    """Orientation in the 3D space."""

    def __init__(self, orientation):
        """Initialize the orientation.

        Args:
            orientation: The input orientation can be either an instance of
                Orientation, an Euler angle(roll, pitch, yall), a quaternion(x,
                y, z, w), or a 3x3 rotation matrix.
        """
        self._euler = None
        self._quaternion = None
        self._matrix3 = None

        if orientation is not None:
            if isinstance(orientation, Orientation):
                self._euler = np.copy(orientation.euler)
                self._quaternion = np.copy(orientation.quaternion)
                self._matrix3 = np.copy(orientation.matrix3)
            else:
                orientation = np.array(orientation, dtype=np.float32)
                if orientation.size == 3:
                    self._euler = orientation.reshape(3,)
                elif orientation.size == 4:
                    self._quaternion = orientation.reshape(4,)
                elif orientation.size == 9:
                    self._matrix3 = orientation.reshape([3, 3])
                else:
                    raise ValueError

    def __str__(self):
        if self.euler is None:
            euler_string = None
        else:
            euler_string = '%g, %g, %g' % (
                self.euler[0], self.euler[1], self.euler[2])

        return '[%s]' % euler_string

    @property
    def orientation(self):
        """ """
        return Orientation(self.euler)

    @property
    def euler(self):
        """ """
        if self._euler is not None:
            pass
        elif self._quaternion is not None:
            self._euler = np.array(euler_from_quaternion(self._quaternion),
                                   dtype=np.float32)
        elif self._matrix3 is not None:
            self._euler = np.array(euler_from_matrix3(self._matrix3),
                                   dtype=np.float32)
        else:
            pass
        return self._euler

    @property
    def quaternion(self):
        """ """
        if self._quaternion is not None:
            pass
        elif self._euler is not None:
            self._quaternion = quaternion_from_euler(
                    self._euler[0],
                    self._euler[1],
                    self._euler[2])
        elif self._matrix3 is not None:
            self._quaternion = quaternion_from_matrix3(self._matrix3)
        else:
            pass
        return self._quaternion

    @property
    def matrix3(self):
        """ """
        if self._matrix3 is not None:
            pass
        elif self._euler is not None:
            self._matrix3 = matrix3_from_euler(
                    self._euler[0],
                    self._euler[1],
                    self._euler[2])
        elif self._quaternion is not None:
            self._matrix3 = matrix3_from_quaternion(self._quaternion)
        else:
            pass
        return self._matrix3

    @orientation.setter
    def orientation(self, value):
        """

        Parameters
        ----------
        value :


        Returns
        -------

        """
        assert isinstance(value, Orientation)
        self._euler = value._euler
        self._quaternion = value._quaternion
        self._matrix3 = value._matrix3

    @euler.setter
    def euler(self, value):
        """

        Parameters
        ----------
        value :


        Returns
        -------

        """
        self._euler = value
        self._quaternion = None
        self._matrix3 = None

    @quaternion.setter
    def quaternion(self, value):
        """

        Parameters
        ----------
        value :


        Returns
        -------

        """
        self._euler = None
        self._quaternion = value
        self._matrix3 = None

    @matrix3.setter
    def matrix3(self, value):
        """

        Parameters
        ----------
        value :


        Returns
        -------

        """
        self._euler = None
        self._quaternion = None
        self._matrix3 = value
