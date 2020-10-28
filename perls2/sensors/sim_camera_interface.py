"""Abstract class for Simulated Camera Interface
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions

import numpy as np
from perls2.sensors.camera_interface import CameraInterface


class SimCameraInterface(CameraInterface):
    """Abstract interface to be implemented for a simulated camera interface

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
            image_height: int
            image_width: int
            intrinsics: a 3x3 camera matrix coefficient
            K: The intrinsics matrix.
            extrinsics: a pose object, or a tuple of translation and rotation.
            distortion (optional): a 4, 5, or 8 element opencv distortion
        """
        super().__init__(
                 image_height,
                 image_width,
                 intrinsics,
                 extrinsics=None,
                 distortion=np.zeros([5]),
                 name='camera')

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

    def capture_new_frames(self):
        """Get the latest set of frames. A blocking call.

        Returns:
            A dictionary of RGB image,  and optionally also
            depth, segmentation, or ir images.
            'rgb': The RGB image as an uint8 np array of [width, height, 3].
            'depth': The depth image as a float32 np array of [width, height].
        """
        raise NotImplementedError

    def set_calibration(self, K, rotation, translation):
        """Set the camera calibration data.

        Args:
            K: The intrinsics matrix.
            rotation: The rotation matrix.
            translation: The translation vector.
        """
        self._K = K
        self._rotation = Orientation(rotation).matrix3
        self._translation = translation

    def load_calibration(self, path, robot_pose=[[0, 0, 0], [0, 0, 0]]):
        """Set the camera by using the camera calibration results.

        Args:
            path: The data directory of the calibration results.
        """
        raise NotImplementedError

    def set_calibration(self, K, rotation, translation):
        """Set the camera calibration data.

        Args:
            K: The intrinsics matrix.
            rotation: The rotation matrix.
            translation: The translation vector.
        """
        raise NotImplementedError

    def callback_view(self, callback):
        """
        Create an OpenCV window with the passed in callback

        Args:
            callback: function to call on click. Should accept depth image,
            the button value (OpenCV - 1 is left click), and x and y
        """
        raise NotImplementedError

    def get_pose_with_click(self, prompt=None, w_rotation=False):
        """ get pose with opencvy gui click
        """
        raise NotImplementedError

    def project(self, points, from_world=False):
        """Project a set of points from 3d frame to 2d image frame.

        Project a set of points from the camera / world coordinate system to
        a image plane using the camera model (instrinsics and distortion).
        If from_world is True, transform the points from world coordinate frame
        to camera frame before projecting.

        Args:
            points: A numpy array of shape [N, 3] or just [3]
            from_world: If True, transform points to camera frame before
            projecting.
        Returns:
            A numpy array [N, 2] of 2D points
        """
        raise NotImplementedError

    def reproject(self, points, depth=None, to_world=False):
        """Reproject a set of points from 2d image frame to 3d frame.

        Project a set of points from the image frame to a 3D frame using the
        camera model (instrinsics only). If depth is None, the unprojected
        points will have depth = 1.
        If to_world is True, transform the points to world coordinate system
        to camera frame after reprojecting.

        Args:
            points: A numpy array of shape [N, 2] or [2]
            from_world: If True, transform points to camera frame before
            projecting.
        Returns:
            A numpy array [N, 2] of 2D points
        """
        raise NotImplementedError

    @property
    def pose(self):
        world_origin_in_camera = Pose(self._translation, self._rotation)
        return world_origin_in_camera.inverse()

    @property
    def rotation(self):
        return self._rotation

    @property
    def translation(self):
        return self._translation

    @property
    def intrinsics(self):
        """Camera intrinsics"""
        return self._intrinsics.copy()

    @property
    def distortion(self):
        """Camera distortion coefficients"""
        return self._distortion.copy()
