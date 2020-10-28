""" Library of helper functions for images
"""
import numpy as np


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
                cameraUpVector=up_vector)

