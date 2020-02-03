import sys
import os
sys.path.append(os.getcwd())  # noqa
import unittest
import numpy as np
import transformations as ttf
from orientation import Orientation


class TestCommon(unittest.TestCase):
    def test_quaternion_init(self):
        random_q = ttf.random_quaternion()
        ornt = Orientation(random_q)
        euler = ornt.euler
        quat = ornt.quaternion
        matrix = ornt.matrix3

        self.assertTrue(np.allclose(euler,
                                    ttf.euler_from_quaternion(random_q)))
        self.assertTrue(np.allclose(matrix,
                                    ttf.quaternion_matrix(random_q)[:3, :3]))
        self.assertTrue(np.allclose(quat, random_q))

    def test_euler_init(self):
        random_e = ttf.euler_from_quaternion(ttf.random_quaternion())
        ornt = Orientation(random_e)
        euler = ornt.euler
        quat = ornt.quaternion
        matrix = ornt.matrix3

        self.assertTrue(np.allclose(euler, random_e))
        self.assertTrue(np.allclose(
            matrix, ttf.euler_matrix(*random_e)[:3, :3]))
        self.assertTrue(np.allclose(
            quat, ttf.quaternion_from_euler(*random_e)))

    def test_matrix_init(self):
        random_m = ttf.random_rotation_matrix()
        ornt = Orientation(random_m[:3, :3])
        euler = ornt.euler
        quat = ornt.quaternion
        matrix = ornt.matrix3

        self.assertTrue(np.allclose(euler,
                                    ttf.euler_from_matrix(random_m)))
        self.assertTrue(np.allclose(quat,
                                    ttf.quaternion_from_matrix(random_m)))
        self.assertTrue(np.allclose(matrix, random_m[:3, :3]))


if __name__ == '__main__':
    unittest.main()
