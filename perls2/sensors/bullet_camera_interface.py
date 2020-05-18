""" Class implementation for Pybullet virtual cameras
"""
from perls2.sensors.sim_camera_interface import SimCameraInterface


import numpy as np
import pybullet

FOV = 60
NEAR_PLANE = 0.02
FAR_PLANE = 100

IMAGE_HEIGHT = 1080
IMAGE_WIDTH = 1920
DEPTH_HEIGHT = 424
DEPTH_WIDTH = 512
# DEPTH_HEIGHT = 428
# DEPTH_WIDTH = 482


class BulletCameraInterface(SimCameraInterface):
    def __init__(self,
                 physics_id=None,
                 image_height=500,
                 image_width=500,
                 near=0.02,
                 far=100,
                 distance=1.0,
                 cameraEyePosition=[0.6, 0., 1.75],
                 cameraTargetPosition=[0.6, 0., 0],
                 cameraUpVector=[1., 0., 1.],
                 name='bullet_camera'):
        """Initialize

        If there are extrinsics of the form RGB_intrisics.npy,
        robot_RGB_rotation.npy, robot_RGB_translation.npy in some
        directory, use that as an argument which takes higher priority than
        the view_matrix and projection_matrix.

        Args:
            image_height: The height of the image.
            image_width: The width of the image.
            near: The distance to the near plane.
            far: The distance to the far plane.
            distance: The distance from the camera to the object.
        """
        super().__init__(image_height, image_width, None)
        self._physics_id = physics_id
        self._near = near
        self._far = far
        self._distance = distance
        self.cameraEyePosition = cameraEyePosition
        self.cameraTargetPosition = cameraTargetPosition
        self.cameraUpVector = cameraUpVector
        # The default orientation is top down.
        self._view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=self.cameraEyePosition,
            cameraTargetPosition=self.cameraTargetPosition,
            cameraUpVector=self.cameraUpVector)

        self._projection_matrix = pybullet.computeProjectionMatrixFOV(
            fov=FOV,
            aspect=float(image_width) / float(image_height),
            nearVal=NEAR_PLANE,
            farVal=FAR_PLANE)
        self.start()

    def set_projection_matrix(self, projection_matrix):
        self._projection_matrix = projection_matrix

    def set_view_matrix(self, view_matrix):
        self._view_matrix = view_matrix

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

        # if self._upside_down:
        #     image = image[::-1, ::-1, :]
        #     depth = depth[::-1, ::-1]
        #     segmask = segmask[::-1, ::-1]

        # This is a fix for the depth image rendering in
        # pybullet, by following:
        # https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
        z_b = depth
        z_n = 2.0 * z_b - 1.0
        z_e = (2.0 * self._near * self._far /
               (self._far + self._near - z_n * (self._far - self._near)))
        depth = z_e

        return {
                'rgb': image,
                'depth': depth,
                'segmask': segmask,
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
                fov=FOV,
                aspect=float(self.image_width) / float(self.image_height),
                nearVal=NEAR_PLANE,
                farVal=FAR_PLANE,
                physicsClientId = self._physics_id)

    def set_calibration(self, K, rotation, translation):
        """Set the camera calibration data.

        Parameters
        ----------
        K :
            The intrinsics matrix.
        rotation :
            The rotation matrix.
        translation :
            The translation vector.

        Returns
        -------

        """
        # Set projection matrix and view matrix.
        self._projection_matrix = intrinsic_to_projection_matrix(
                self._K, self._image_height, self._image_width,
                self._near, self._far)

        self._view_matrix = extrinsic_to_view_matrix(
                self._rotation, self._translation, self._distance)

    def project(self, point, is_world_frame=True):
        """Projects a point cloud onto the camera image plane.

        Parameters
        ----------
        point :
            3D point to project onto the camera image plane.
        is_world_frame :
            True if the 3D point is defined in the world frame,
            (Default value = False)

        Returns
        -------
        pixel
            2D pixel location in the camera image.

        """
        point = np.array(point)

        if is_world_frame:
            point = np.dot(point - self.pose.position, self.pose.matrix3)

        projected = np.dot(point, self.K.T)
        projected = np.divide(projected, np.tile(projected[2], [3]))
        projected = np.round(projected)
        pixel = projected[:2].astype(np.int16)

        return pixel

    def deproject(self, pixel, depth, is_world_frame=True):
        """Deprojects a single pixel with a given depth into a 3D point.

        Parameters
        ----------
        pixel :
            2D point representing the pixel location in camera image.
        depth :
            Depth value at the given pixel location.
        is_world_frame :
            True if the 3D point is defined in the world frame,
            (Default value = True)

        Returns
        -------
        point
            The deprojected 3D point.

        """
        x, y = pixel
        cam_x = (x - self.K[0, 2]) / self.K[0, 0] * depth
        cam_y = (y - self.K[1, 2]) / self.K[1, 1] * depth
        point = [cam_x, cam_y, depth]

        if is_world_frame:
            point = self.pose.position + np.dot(point, self.pose.matrix3.T)

        return point

    def extrinsic_to_view_matrix(rotation, translation, distance):
        """Convert the camera extrinsics to the view matrix.

        The function takes HZ-style rotation matrix R and translation matrix t
        and converts them to a Bullet/OpenGL style view matrix. the derivation
        is pretty simple if you consider x_camera = R * x_world + t.

        Parameters
        ----------
        distance :
            The distance from the camera to the focus.
        rotation :

        translation :


        Returns
        -------

        """
        # The camera position in the world frame.
        camera_position = rotation.T.dot(-translation)

        # The focus in the world frame.
        focus = rotation.T.dot(np.array([0, 0, distance]) - translation)

        # The up vector is the Y-axis of the camera in the world frame.
        up_vector = rotation.T.dot(np.array([0, 1, 0]))

        # Compute the view matrix.
        view_matrix = pybullet.computeViewMatrix(
                    cameraEyePosition=camera_position,
                    cameraTargetPosition=focus,
                    cameraUpVector=up_vector, 
                    physicsClientId=self._physics_id)

        return view_matrix

    @property
    def image_height(self):
        """ """
        return self._image_height

    @property
    def image_width(self):
        """ """
        return self._image_width

    @property
    def view_matrix(self):
        """ """
        return self._view_matrix

    @property
    def projection_matrix(self):
        """ """
        return self._projection_matrix
