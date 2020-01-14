"""
Symbolic geometry utility functions for computing IK
"""

import sympy
from perls.math.geometry import Rx_matrix, Rz_matrix


def symbolic_Rz_matrix(symbolic_theta):
    """Symbolic rotation matrix around the Z axis."""
    return sympy.Matrix([
        [sympy.cos(symbolic_theta), -sympy.sin(symbolic_theta), 0],
        [sympy.sin(symbolic_theta), sympy.cos(symbolic_theta), 0],
        [0, 0, 1]
    ])


def symbolic_rotation_matrix(phi, theta, symbolic_psi):
    """ Construct a symbolic rotation matrix"""
    return (sympy.Matrix(Rz_matrix(phi)) * sympy.Matrix(Rx_matrix(theta)) *
            symbolic_Rz_matrix(symbolic_psi))


def symbolic_axis_rotation_matrix(axis, symbolic_theta):
    """Returns a rotation matrix around the given axis"""
    [x, y, z] = axis
    c = sympy.cos(symbolic_theta)
    s = sympy.sin(symbolic_theta)
    return sympy.Matrix([
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
