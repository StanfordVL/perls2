import sys
import os
sys.path.append(os.getcwd())  # noqa
import unittest
import numpy as np
import transformations as ttf
from pose import Pose


class TestCommon(unittest.TestCase):
    def assert_pose_matrix_eq(self, pose, matrix):
        self.assertTrue(np.allclose(pose.matrix3,
                                    matrix[:3, :3]))
        self.assertTrue(np.allclose(pose.position, matrix[:3, 3]))
        self.assertTrue(np.allclose(pose.matrix4, matrix))

    def test_init(self):
        random_e = np.random.rand(3) * np.pi * 2
        random_t = np.random.rand(3)
        pose = Pose(random_t, random_e)
        matrix = ttf.compose_matrix(angles=random_e, translate=random_t)
        self.assert_pose_matrix_eq(pose, matrix)

    def test_transform(self):
        random_e = np.random.rand(3) * np.pi * 2
        random_t = np.random.rand(3)
        pose = Pose(random_t, random_e)
        matrix = ttf.compose_matrix(angles=random_e, translate=random_t)

        random_e = np.random.rand(3) * np.pi * 2
        random_t = np.random.rand(3)
        pose_2 = Pose(random_t, random_e)
        matrix_2 = ttf.compose_matrix(angles=random_e, translate=random_t)

        matrix_t = matrix_2.dot(matrix)
        pose_t = pose_2.transform(pose)
        self.assert_pose_matrix_eq(pose_t, matrix_t)

    def test_inverse(self):
        random_e = np.random.rand(3) * np.pi * 2
        random_t = np.random.rand(3)
        pose = Pose(random_t, random_e).inverse()
        matrix = np.linalg.inv(ttf.compose_matrix(angles=random_e,
                                                  translate=random_t))
        self.assert_pose_matrix_eq(pose, matrix)


if __name__ == '__main__':
    unittest.main()
