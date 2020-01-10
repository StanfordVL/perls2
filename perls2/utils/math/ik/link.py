"""This module implements the Link class.
"""

import numpy as np
import sympy

import perls.math.geometry as geometry
import perls.simulation.ik.sym_geometry as sym_geometry


class Link(object):
    """Base Link class.
    """

    def __init__(self, name, bounds=(None, None)):
        """Initialize.

        Args:
            name: string
            bounds:The bounds of the link. Defaults to None.
        """
        self.bounds = bounds
        self.name = name

    def _get_rotation_axis(self):
        # Defaults to None
        return [0, 0, 0, 1]

    def get_transformation_matrix(self, theta):
        raise NotImplementedError


class OriginLink(Link):
    """The link at the origin of the robot.
    """
    def __init__(self):
        Link.__init__(self, name='Base link')
        self._length = 1

    def _get_rotation_axis(self):
        return [0, 0, 0, 1]

    def get_transformation_matrix(self, theta):
        return np.eye(4)


class URDFLink(Link):
    """Link in URDF representation.

    name: The name of the link
    bounds: Optional : The bounds of the link. Defaults to None
    translation: The translation vector. (In URDF, attribute "xyz"
        of the "origin" element).
    orientation: The orientation of the link. (In URDF, attribute "rpy"
        of the "origin" element).
    rotation: The rotation axis of the link. (In URDF, attribute "xyz" of
        the "axis" element).
    angle_representation: Optionnal : The representation used by the
    angle. Currently supported representations : rpy. Defaults to rpy, the URDF
        standard.
    use_symbolic_matrix: wether the transformation matrix is stored as a
        Numpy array or as a Sympy symbolic matrix.

    Returns:
        The link object.
    """

    def __init__(self,
                 name,
                 translation,
                 orientation,
                 rotation,
                 bounds=(None, None),
                 angle_representation='rpy',
                 use_symbolic_matrix=True):

        Link.__init__(self, name=name, bounds=bounds)
        self.use_symbolic_matrix = use_symbolic_matrix
        self.translation = np.array(translation)
        self.orientation = np.array(orientation)
        self.rotation = np.array(rotation)

        self._length = np.linalg.norm(translation)
        self._axis_length = self._length

        if use_symbolic_matrix:
            theta = sympy.symbols('theta')

            symbolic_frame_matrix = np.eye(4)

            # Apply translation matrix
            symbolic_frame_matrix = symbolic_frame_matrix * sympy.Matrix(
                geometry.homogeneous_translation_matrix(*translation))

            # Apply orientation matrix
            symbolic_frame_matrix = (
                symbolic_frame_matrix *
                geometry.cartesian_to_homogeneous(
                    geometry.rpy_matrix(*orientation))
            )

            # Apply rotation matrix
            symbolic_frame_matrix = (
                symbolic_frame_matrix *
                geometry.cartesian_to_homogeneous(
                    sym_geometry.symbolic_axis_rotation_matrix(
                        rotation, theta), matrix_type='sympy')
                )

            self.symbolic_transformation_matrix = sympy.lambdify(
                theta, symbolic_frame_matrix, 'numpy')

    def _get_rotation_axis(self):
        return np.dot(
            geometry.homogeneous_translation_matrix(*self.translation),
            np.dot(geometry.cartesian_to_homogeneous(
                geometry.rpy_matrix(*self.orientation)),
                geometry.cartesian_to_homogeneous_vectors(
                    self.rotation * self._axis_length))
        )

    def get_transformation_matrix(self, theta):
        if self.use_symbolic_matrix:
            frame_matrix = self.symbolic_transformation_matrix(theta)
        else:
            # Init the transformation matrix
            frame_matrix = np.eye(4)

            # First, apply translation matrix
            frame_matrix = np.dot(
                frame_matrix,
                geometry.homogeneous_translation_matrix(
                    *self.translation)
            )

            # Apply orientation
            frame_matrix = np.dot(
                frame_matrix,
                geometry.cartesian_to_homogeneous(
                    geometry.rpy_matrix(*self.orientation))
            )

            # Apply rotation matrix
            frame_matrix = np.dot(
                frame_matrix,
                geometry.cartesian_to_homogeneous(
                    geometry.axis_rotation_matrix(
                        self.rotation, theta))
            )

        return frame_matrix
