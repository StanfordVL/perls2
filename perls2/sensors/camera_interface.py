"""Abstract class for Camera Interface
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
import numpy as np

from perls2.sensors.sensor_interface import SensorInterface


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

    def set_calibrationGl(self, view_matrix, projection_matrix):
        """Set the camera calibration data using OpenGL Conventions
        """
        self._projection_matrix = projection_matrix
        self._view_matrix = view_matrix

    def load_calibration(self, path, robot_pose=[[0, 0, 0], [0, 0, 0]]):
        """Set the camera by using the camera calibration results.

        Args:
            path: The data directory of the calibration results.
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

    def intrinsic_to_projection_matrix(K, height, width, near, far,
                                       upside_down=True):
        """Convert the camera intrinsics to the projection matrix.

        Takes a Hartley-Zisserman intrinsic matrix and returns a Bullet/OpenGL
        style projection matrix. We pad with zeros on right and bottom and a 1
        in the corner.

        Uses algorithm found at:
        https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL#note-about-image-coordinates

        Parameters
        ----------
        K :
            The camera intrinsincs matrix.
        height :
            The image height.
        width :
            The image width.
        near :
            The distance to the near plane.
        far :
            The distance to the far plane.
        upside_down :
            (Default value = True)
        """
        projection_matrix = np.empty((4, 4), dtype=np.float32)

        f_x = K[0, 0]
        f_y = K[1, 1]
        x_0 = K[0, 2]
        y_0 = K[1, 2]
        s = K[0, 1]

        if upside_down:
            x_0 = width - x_0
            y_0 = height - y_0

        projection_matrix[0, 0] = 2 * f_x / width
        projection_matrix[0, 1] = -2 * s / width
        projection_matrix[0, 2] = (width - 2 * x_0) / width
        projection_matrix[0, 3] = 0

        projection_matrix[1, 0] = 0
        projection_matrix[1, 1] = 2 * f_y / height
        projection_matrix[1, 2] = (-height + 2 * y_0) / height
        projection_matrix[1, 3] = 0

        projection_matrix[2, 0] = 0
        projection_matrix[2, 1] = 0
        projection_matrix[2, 2] = (-far - near) / (far - near)
        projection_matrix[2, 3] = -2 * far * near / (far - near)

        projection_matrix[3, 0] = 0
        projection_matrix[3, 1] = 0
        projection_matrix[3, 2] = -1
        projection_matrix[3, 3] = 0

        projection_matrix = list(projection_matrix.transpose().flatten())

        return projection_matrix

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
    def K(self):
        return self._K

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
