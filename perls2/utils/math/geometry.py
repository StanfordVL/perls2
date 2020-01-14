"""
Common geometry utility functions
"""

import numpy as np
from perls.utils.numpy_utils import assert_shape
from perls.math.transformations import rotation_matrix3 as rotation_matrix


class Point2D(object):
    """
    A generic Point2D class
    """
    def __init__(self, x, y, dtype=np.float64):
        self._x = x
        self._y = y
        self.dtype = dtype

    def __str__(self):
        return "(%f, %f)" % (self._x, self._y)

    @classmethod
    def from_array(cls, a):
        assert(a.shape == (2,))
        return Point2D(a[0], a[1], a.dtype)

    @property
    def x(self):
        return float(self._x)

    @property
    def y(self):
        return float(self._y)

    @property
    def array(self):
        return np.array([self._x, self._y], dtype=self.dtype)


class Point3D(object):
    """
    A generic Point3D class
    """
    def __init__(self, x, y, z, dtype=np.float64):
        self._x = x
        self._y = y
        self._z = z
        self.dtype = dtype

    def __str__(self):
        return "(%f, %f, %f)" % (self._x, self._y, self._z)

    @classmethod
    def from_array(cls, a):
        assert(a.shape == (3,))
        return Point3D(a[0], a[1], a[2], a.dtype)

    @property
    def x(self):
        return float(self._x)

    @property
    def y(self):
        return float(self._y)

    @property
    def z(self):
        return float(self._z)

    @property
    def homogeneous(self):
        return np.array([self.x, self.y, self.z, 1], dtype=self.dtype)

    @property
    def array(self):
        return np.array([self._x, self._y, self._z], dtype=self.dtype)


def normalize_angle(angle):
    """wrap an angle in radius to [-pi, pi]

    Args:
        angle: angle in radian
    Returns:
        normalized angle (radian) in [-pi, pi]
    """
    k2PI = np.pi * 2
    angle = angle - k2PI * np.floor((angle + np.pi) / k2PI)
    return np.maximum(np.minimum(np.pi, angle), -np.pi)  # numerical underflow


def transform_points_3d(pts, t):
    """Homogeneous transformation using a 4x4 matrix.

    Args:
        pts: a numpy array of 3D points [N, 3]
        t: a 4x4 transformation matrix

    Returns:
        a numpy array of transformed points
    """
    assert(is_transformation_matrix(t))
    assert_shape(pts, (None, 3))
    pts = pts.copy()
    # convert to homogeneous coordinate system
    pts = convert_points_to_homogeneous(pts)
    tpts = t.dot(pts.T).T
    tpts = tpts[:, :3] / tpts[:, 3, None]  # dehomogenize
    return tpts


def is_transformation_matrix(mat):
    """Check if a matrix is a transformation matrix"""
    assert_shape(mat, (4, 4))
    val = np.all(mat[3, :3] == 0)
    c = mat[3, 3] == 1
    R = mat[:3, :3]
    # check if orthogonal
    orthogonal = np.allclose(R.dot(R.T), np.eye(3))
    return val and c and orthogonal


def to_transformation_matrix(R, T):
    """Compose a 4x4 transformation matrix

    Args:
        R: a [3, 3] rotation matrix
        T: a [3] translation matrix
    Returns:
        a [4, 4] transformation matrix
    """
    assert_shape(R, (3, 3))
    assert_shape(T, (3, ))
    RT = np.eye(4, dtype=R.dtype)
    RT[:3, :3] = R
    RT[:3, 3] = T
    return RT


def from_transformation_matrix(transformation_matrix):
    """Converts a transformation matrix to a tuple."""
    assert(is_transformation_matrix(transformation_matrix))
    return (transformation_matrix[:, -1], transformation_matrix[:-1, :-1])


def convert_points_to_homogeneous(pts):
    """ Convert a list of points to homogeneous coordinate system

    Pads a list of points (2D or 3D) by 1's

    Args:
        pts: a numpy array of points [N, 2] or [N, 3]
    Returns:
        a list of points in homogenous coordinate system [N, 3] or [N, 4]
    """
    assert_shape(pts, [None, None])
    padding = np.ones(pts.shape[0], dtype=pts.dtype)
    return np.concatenate([pts, padding[:, None]], axis=1)


def Rx_matrix(theta):
    """Rotation matrix around the X axis.

    Args:
        theta: angle in radius
    Returns:
        a 3 by 3 rotation matrix
    """
    return rotation_matrix(theta, (1, 0, 0))[:3, :3]


def Ry_matrix(theta):
    """Rotation matrix around the Y axis

    Args:
        theta: angle in radius
    Returns:
        a 3 by 3 rotation matrix
    """
    return rotation_matrix(theta, (0, 1, 0))[:3, :3]


def Rz_matrix(theta):
    """Rotation matrix around the Z axis

    Args:
        theta: angle in radius
    Returns:
        a 3 by 3 rotation matrix
    """
    return rotation_matrix(theta, (0, 0, 1))[:3, :3]


def rpy_matrix(roll, pitch, yaw):
    """Returns a rotation matrix described by roll, pitch, yaw."""
    return np.dot(Rz_matrix(yaw), np.dot(Ry_matrix(pitch), Rx_matrix(roll)))


def axis_rotation_matrix(axis, theta):
    """Returns a rotation matrix around the given axis"""
    [x, y, z] = axis
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [x**2 + (1 - x**2) * c,
         x * y * (1 - c) - z * s,
         x * z * (1 - c) + y * s],
        [x * y * (1 - c) + z * s,
         y ** 2 + (1 - y**2) * c,
         y * z * (1 - c) - x * s],
        [x * z * (1 - c) - y * s,
         y * z * (1 - c) + x * s,
         z**2 + (1 - z**2) * c]
    ])


def homogeneous_translation_matrix(trans_x, trans_y, trans_z):
    """Returns a translation matrix the homogeneous space"""
    t = np.eye(4)
    t[:3, 3] = [trans_x, trans_y, trans_z]
    return t


def cartesian_to_homogeneous(cartesian_matrix, matrix_type="numpy"):
    """Converts a cartesian matrix to an homogenous matrix"""
    assert_shape(cartesian_matrix, (None, None))
    dimension_x, dimension_y = cartesian_matrix.shape
    # Square matrix
    # Manage different types fo input matrixes
    if matrix_type == "numpy":
        homogeneous_matrix = np.eye(dimension_x + 1)
    elif matrix_type == "sympy":
        import sympy
        homogeneous_matrix = sympy.eye(dimension_x + 1)
    # Add a column filled with 0 and finishing with 1 to the cartesian
    # matrix to transform it into an homogeneous one.
    homogeneous_matrix[:-1, :-1] = cartesian_matrix

    return homogeneous_matrix


def cartesian_to_homogeneous_vectors(cartesian_vector, matrix_type="numpy"):
    """Converts a cartesian vector to an homogenous vector"""
    assert_shape(cartesian_vector, (None,))
    dimension_x = cartesian_vector.shape[0]
    # Vector
    if matrix_type == "numpy":
        homogeneous_vector = np.zeros(dimension_x + 1)
        # Last item is a 1
        homogeneous_vector[-1] = 1
        homogeneous_vector[:-1] = cartesian_vector
    return homogeneous_vector


def homogeneous_to_cartesian_vectors(homogeneous_vector):
    """Converts a cartesian vector to an homogenous vector"""
    assert_shape(homogeneous_vector, (None,))
    return homogeneous_vector[:-1]


def homogeneous_to_cartesian(homogeneous_matrix):
    """Converts a cartesian matrix to an homogenous matrix"""
    # Remove the last column
    assert_shape(homogeneous_matrix, (None, None))
    return homogeneous_matrix[:-1, :-1]
