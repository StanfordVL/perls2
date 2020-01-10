"""Pose in the 3D space.

Author: Kuan Fang, Danfei Xu, Andrey Kurenkov
"""

import numpy as np

from perls2.utils.math.orientation import Orientation


class Pose(Orientation):
    """Pose in the 3D space."""

    def __init__(self, position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0]):
        """Initialize the pose.

        Parameters
        ----------
        position : array_like (optional)
            The position is a 3-dimensional float32 numpy array.
            Default is [0.0, 0.0, 0.0].
        orientation : array_like (optionla)
            The orientation can be either an instance of
            Orientation, an Euler angle(roll, pitch, yaw), a quaternion(x,
            y, z, w), or a 3x3 orientation matrix.
            Default is [0.0, 0.0, 0.0].
        """
        self._position = np.array(position, dtype=np.float32).reshape(3,)
        Orientation.__init__(self, orientation)

    def transform(self, pose):
        """Rigid transformation from one from to another.

        The rigid transformation from the source frame to the target frame is
        defined by self.position and self.orientation.

        Parameters
        ----------
        pose : perls2.utils.math.pose.Pose
            The pose defined in the source frame

        Returns
        -------
        perls2.utils.math.pose.Pose
            The corresponding pose defined in the target frame.
        """
        if not isinstance(pose, Pose):
            pose = Pose(pose[0], pose[1])

        position = self.position + pose.position.dot(self.matrix3.T)
        orientation = self.matrix3.dot(pose.matrix3)

        return Pose(position, orientation)

    def __str__(self):
        if self.position is None:
            position_string = None
        else:
            position_string = '%g, %g, %g' % (self.position[0],
                                              self.position[1],
                                              self.position[2]
                                              )

        if self.euler is None:
            euler_string = None
        else:
            euler_string = '%g, %g, %g' % (self.euler[0],
                                           self.euler[1],
                                           self.euler[2]
                                           )

        return '[position: %s, euler: %s]' % (position_string, euler_string)

    def inverse(self):
        """Get the inverse rigid transformation of the pose.

        If the pose is defined in the world frame, then the return value s the
        world origin in the frame of this pose.

        Returns
        -------
        perls2.utils.math.pose.Pose
            The inverse pose
        """
        position = np.dot(-self.position, self.matrix3)
        orientation = self.matrix3.T
        return Pose(position, orientation)

    def inverse_transform(self, pose):
        """Transform param by inverse rigid transformation of this pose.

        Parameters
        ----------
        pose : perls2.utils.math.pose.Pose
            The pose to be transformed

        Returns
        -------
        perls2.utils.math.pose.Pose
            The transformed pose
        """
        return self.inverse().transform(pose)

    def tolist(self):
        """Convert the Pose instance to a list.

        Returns
        -------
        list
            A list of position and euler angles.
        """
        return [[self.position[0], self.position[1], self.position[2]],
                [self.euler[0], self.euler[1], self.euler[2]]]

    def toarray(self):
        """Convert the Pose instance to a numpy array.

        Returns
        -------
        np.array
            A numpy array of position and euler angles.
        """
        return np.array(self.tolist())

    def totuple(self):
        """Convert the Pose instance to a tuple.

        Returns
        -------
        tuple
            A tuple of lists of position and euler angles.
        """
        return ([self.position[0], self.position[1], self.position[2]],
                [self.euler[0], self.euler[1], self.euler[2]])

    @property
    def pose(self):
        """Returns the copy of the pose."""
        return Pose(self.position, self.orientation)

    @property
    def position(self):
        """The position as a 3-dimensional float vector."""
        return self._position

    @property
    def matrix4(self):
        """The transformation matrix of shape [4, 4]."""
        matrix4 = np.eye(4)
        matrix4[:3, :3] = self.matrix3
        matrix4[:3, 3] = self.position
        return matrix4

    @pose.setter
    def pose(self, value):
        assert isinstance(value, Pose)
        self._position = value.position
        self._euler = value.euler
        self._quaternion = value.quaternion
        self._matrix3 = value.matrix3

    @position.setter
    def position(self, value):
        self._position = value

    @ staticmethod
    def uniform(x,
                y,
                z,
                roll=0.0,
                pitch=0.0,
                yaw=0.0):
        """Draw pose samples from a uniform distribution.

        Parameters
        ----------
        x :
            Value/range of the x position.
        y :
            Value/range of the y position.
        z :
            Value/range of the z position.
        roll :
            Value/range of the roll position. (Default value = 0.0)
        pitch :
            Value/range of the pitch position. (Default value = 0.0)
        yaw :
            Value/range of the yaw position. (Default value = 0.0)

        Returns
        -------
        perls2.utils.math.pose.Pose
            The pose drawn uniformly at random from the distribution
        """
        if isinstance(x, (list, tuple)):
            x = np.random.uniform(x[0], x[1])

        if isinstance(y, (list, tuple)):
            y = np.random.uniform(y[0], y[1])

        if isinstance(z, (list, tuple)):
            z = np.random.uniform(z[0], z[1])

        if isinstance(roll, (list, tuple)):
            roll = np.random.uniform(roll[0], roll[1])

        if isinstance(pitch, (list, tuple)):
            pitch = np.random.uniform(pitch[0], pitch[1])

        if isinstance(yaw, (list, tuple)):
            yaw = np.random.uniform(yaw[0], yaw[1])

        return Pose([x, y, z], [roll, pitch, yaw])


def sin_cos_encode_pose(pose):
    """Encode the 6-DOF pose.

    Parameters
    ----------
    pose : perls2.utils.math.pose.Pose
        The pose to encode

    Returns
    -------
    position : numpy.array
        The position.
    orientation: numpy.array
        The encoded orientation, where each Euler angle is encoded
        as a sine-cosine pair.
    """
    position = np.array(pose.position, dtype=np.float32)
    roll, pitch, yaw = pose.euler
    orientation = np.array(
            [
                np.sin(roll),
                np.cos(roll),
                np.sin(pitch),
                np.cos(pitch),
                np.sin(yaw),
                np.cos(yaw),
            ],
            dtype=np.float32)
    return position, orientation


def sin_cos_encode_pose4(pose):
    """Encode the 6-DOF pose.

    Parameters
    ----------
    pose : perls2.utils.math.pose.Pose
        The pose to encode

    Returns
    -------
    position: numpy.array
        The position.
    angle: numpy.array
        The encoded angle, which is encoded as a sine-cosine pair.

    """
    position = np.array(pose[:3])
    angle = np.array([np.sin(pose[3]), np.cos(pose[3])], dtype=np.float32)
    return position, angle


def get_transform(source=None, target=None):
    """Get rigid transformation from one frame to another. The transformation is
    represented as a Pose instance.

    Parameters
    ----------
    source :
        The source frame. (Default value = None)
    target :
        The target frame. (Default value = None)

    Returns
    -------
    perls2.utils.math.pose.Pose
        The transform.

    """
    if source is not None and not isinstance(source, Pose):
        source = Pose(source[0], source[1])

    if target is not None and not isinstance(target, Pose):
        target = Pose(target[0], target[1])

    if source is not None and target is not None:
        orientation = np.dot(target.matrix3.T, source.matrix3)
        position = np.dot(source.position - target.position, target.matrix3)

    elif source is not None:
        orientation = source.matrix3
        position = source.position

    elif target is not None:
        orientation = target.matrix3.T
        position = np.dot(-target.position, target.matrix3)

    else:
        orientation = np.eye(3, dtype=np.float32)
        position = np.ones(3, dtype=np.float32)

    return Pose(position, orientation)
