""" Class implementation for Pybullet virtual cameras
"""
from perls2.sensors.sim_camera_interface import SimCameraInterface


import numpy as np
import pybullet

FOV =90
NEAR_PLANE = 0.02
FAR_PLANE = 100

IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920
DEPTH_HEIGHT = 424
DEPTH_WIDTH = 512
# DEPTH_HEIGHT = 428
# DEPTH_WIDTH = 482


class BulletCameraInterface(SimCameraInterface):
    """" Interface for rendering camera images from PyBullet.

    Attributes:
        image_height (int): Height of the image in pixels.
        image_width (int): Width of the image in pixels.
        near (float): The distance to the near plane.
        far (float): The distance to the far plane.
        distance (float): The distance from the camera to the object.
        cameraEyePosition (list): 3f xyz position of camera in world frame.
        cameraTargetPosition (list): 3f xyz position of camera target in world frame.
        cameraUpVectory (list): 3f vector describing camera up direction.
        view_matrix (np.npdarray): View matrix
        projection_matrix (np.ndarray): projection matrix.
        name (str): string identifying name of camera.
    """
    def __init__(self,
                 physics_id=None,
                 image_height=500,
                 image_width=500,
                 near=0.02,
                 far=100,
                 distance=1.0,
                 fov=60,
                 cameraEyePosition=[0.6, 0., 1.0],
                 cameraTargetPosition=[0.6, 0., 0],
                 cameraUpVector=[1., 0., 1.],
                 name='bullet_camera'):
        """Initialize

        Args:
            image_height (int): Height of the image in pixels.
            image_width (int): Width of the image in pixels.
            near (float): The distance to the near plane.
            far (float): The distance to the far plane.
            distance (float): The distance from the camera to the object.
            cameraEyePosition (list): 3f xyz position of camera in world frame.
            cameraTargetPosition (list): 3f xyz position of camera target in world frame.
            cameraUpVectory (list): 3f vector describing camera up direction.
            name (str): string identifying name of camera.

        """
        super().__init__(image_height, image_width, None)
        self._physics_id = physics_id
        self._near = near
        self._far = far
        self._distance = distance
        self._fov = fov
        self.cameraEyePosition = cameraEyePosition
        self.cameraTargetPosition = cameraTargetPosition
        self.cameraUpVector = cameraUpVector
        # The default orientation is top down.
        self._view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=self.cameraEyePosition,
            cameraTargetPosition=self.cameraTargetPosition,
            cameraUpVector=self.cameraUpVector)

        self._projection_matrix = pybullet.computeProjectionMatrixFOV(
            fov=self._fov,
            aspect=float(image_width) / float(image_height),
            nearVal=self._near,
            farVal=self._far,
            )


        self._K = self.get_intrinsics()

        self.start()

    def set_projection_matrix(self, projection_matrix):
        """Set projection matrix for the camera.
        Args:
            projection_matrix (list): list of 16 floats for pybullet projection matrix.
        """
        self._projection_matrix = projection_matrix

    def set_view_matrix(self, view_matrix):
        """ Set View matrix for the camera
        Args:
            view_matrix (list): list of 16 floats for pybullet view_matrix
        """
        self._view_matrix = view_matrix


    def get_intrinsics(self):
        """ Calculate camera intrinsic matrix.
        """
        # get projection matrix as 4x4
        P = self.projection_matrix
        w, h = self._image_width, self._image_height
        znear, zfar = self._near, self._far

        a = (2.0 * znear) / P[0, 0]
        b = P[2, 0] * a
        right = (a + b) / 2.0
        left = b - right
        c = (2.0 * znear) / P[1, 1]
        d = P[3, 1] * c
        top = (c + d) / 2.0
        bottom = d - top
        fu = w * znear / (right - left)
        fv = h * znear / (top - bottom)

        u0 = w - right * fu / znear
        v0 = h - top * fv / znear
        return np.array([[fu, 0, u0], [0, fv, v0], [0, 0, 1]])

    def deproject(self, pixel, depth, is_world_frame=True):
        """Deprojects a single pixel with a given depth into a 3D point.
        Parameters
        ----------
        pixel :
            2D point representing the pixel location in camera image.
        depth :
            Depth value at the given pixel location.
        is_world_frame :
            True if the 3D point is defined in the world frame, (Default value = True)
        Returns
        -------
        point
            The deprojected 3D point.
        """
        x, y = pixel
        cam_x = (x - self.K[0, 2]) / self.K[0, 0] * depth
        cam_y = (y - self.K[1, 2]) / self.K[1, 1] * depth
        point = np.asarray([cam_x, cam_y, depth])

        camera_R = -self.view_matrix[:3, :3]

        camera_pos = -self.view_matrix[3, :3]

        if is_world_frame:
            point = camera_pos + np.dot(point, camera_R.T)

        return point

    def start(self):
        """Starts the camera stream."""
        self.frames()

    def stop(self):
        """Stops the sensor stream.

        Returns:
            True if succeed, False if fail.
        """
        pass

    def set_physics_id(self, physics_id):
        """ Set unique physics client id
        Args:
            physics_id (int): new physics id to set.
        """
        self._physics_id = physics_id

    def frames(self):
        """Render the world at the current time step.
        Args: None
        Returns:
            dict with rgb, depth and segmask image.
        """
        _, _, rgba, depth, segmask = pybullet.getCameraImage(
            height=self._image_height,
            width=self._image_width,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix,
            physicsClientId=self._physics_id)

        rgba = np.array(rgba).astype('uint8')
        rgba = rgba.reshape((self._image_height, self._image_width, 4))
        # invert
        image = rgba[:, :, :3]
        image = np.invert(image)
        depth = np.array(depth).astype('float32')
        depth = depth.reshape((self._image_height, self._image_width))

        segmask = np.array(segmask).astype('uint8')
        segmask = segmask.reshape((self._image_height, self._image_width))

        # This is a fix for the depth image rendering in
        # pybullet, by following:
        # https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
        z_b = depth
        z_n = 2.0 * z_b - 1.0
        z_e = (2.0 * self._near * self._far /
               (self._far + self._near - z_n * (self._far - self._near)))
        depth = z_e

        return {'rgb': image,
                'depth': depth,
                'segmask': segmask,
                'rgba': rgba}

    def frames_rgb(self):
        """Render the world at the current time step.
            Args: None
            Returns:
                dict with rgb, depth and segmask image.
        """
        _, _, rgba, depth, segmask = pybullet.getCameraImage(
            height=self._image_height,
            width=self._image_width,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix,
            physicsClientId=self._physics_id)

        rgba = np.array(rgba).astype('uint8')
        rgba = rgba.reshape((self._image_height, self._image_width, 4))
        # invert
        image = rgba[:, :, :3]
        #image = np.invert(image)
        return {
                'rgb': image,
                }

    def place(self, new_camera_pos):
        """ Places camera in new position

        Modifies cameraEyePosition property and adjusts view and
        projection matrices

        Args:
            new_camera_pos (3f): x y z coordinates of new camera position.
        Returns: None
        """
        self.cameraEyePosition = new_camera_pos

        self._view_matrix = pybullet.computeViewMatrix(
                cameraEyePosition=self.cameraEyePosition,
                cameraTargetPosition=self.cameraTargetPosition,
                cameraUpVector=self.cameraUpVector,
                physicsClientId=self._physics_id)

        self._projection_matrix = pybullet.computeProjectionMatrixFOV(
                fov=self._fov,
                aspect=float(self.image_width) / float(self.image_height),
                nearVal=self._near,
                farVal=self._far,
                physicsClientId = self._physics_id)

        self.K = self.get_intrinsics()

    @property
    def image_height(self):
        """ Height of rendered image.
        """
        return self._image_height

    @property
    def image_width(self):
        """Width of rendered image.
        """
        return self._image_width

    @property
    def view_matrix(self):
        """View matrix.
        """
        return np.asarray(self._view_matrix).reshape(4,4)

    @property
    def projection_matrix(self):
        """Projection matrix.
        """
        return np.asarray(self._projection_matrix).reshape(4,4)
