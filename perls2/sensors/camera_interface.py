"""Abstract class for Camera Interface
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
import numpy as np

from perls2.sensors.sensor_interface import SensorInterface
import perls2.utils.image_utils as IU

@six.add_metaclass(abc.ABCMeta)
class CameraInterface(SensorInterface):
    """Abstract interface to be implemented for a sensor
    """

    def __init__(self,
                 image_height,
                 image_width,
                 intrinsics,
                 extrinsics=None,
                 distortion=np.zeros([5]),
                 name='camera'):
        """
        Initialize variables

          Args:
            image_height (int): height in pixels
            image_width (int): width in pixels
            intrinsics: a 3x3 camera matrix coefficient
            K: The intrinsics matrix.
            extrinsics: a pose object, or a tuple of translation and rotation.
            distortion (optional): a 4, 5, or 8 element opencv distortion
        """
        self._image_width = image_width
        self._image_height = image_height
        self._name = name
        if intrinsics is not None:
            self._intrinsics = intrinsics.copy()
        self._distortion = distortion.copy()

        if extrinsics is not None and type(extrinsics) == tuple:
            rotation, translation = extrinsics
            self.set_calibration(intrinsics, rotation, translation)
        elif extrinsics is not None:
            rotation = extrinsics.matrix3
            translation = extrinsics.position
            self.set_calibration(intrinsics, rotation, translation)

    def start(self):
        """Starts the sensor stream.
        """
        raise NotImplementedError

    def stop(self):
        """Stops the sensor stream.

        Returns:
            True if succeed, False if fail.
        """
        raise NotImplementedError

    def reset(self):
        """Restarts the sensor stream.
        """
        self.stop()
        self.start()

    @property
    def rotation(self):
        return self._rotation

    @property
    def translation(self):
        return self._translation

    @property
    def K(self):
        return self._K

    @K.setter
    def K(self, new_intrinsics):
        self._K = new_intrinsics

    @property
    def cx(self):
        return self.K[0, 2]

    @property
    def cy(self):
        return self.K[1, 2]

    @property
    def name(self):
        """Name of the camera"""
        return self._name

    @property
    def image_width(self):
        return self._image_width

    @property
    def image_height(self):
        return self._image_height

    @property
    def intrinsics(self):
        """Camera intrinsics"""
        return self._intrinsics.copy()

    @property
    def distortion(self):
        """Camera distortion coefficients"""
        return self._distortion.copy()

    @property
    def view_matrix(self):
        return self._view_matrix

    @property
    def projection_matrix(self):
        return self._projection_matrix
