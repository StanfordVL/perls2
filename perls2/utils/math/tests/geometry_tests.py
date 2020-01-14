from perls.math.geometry import (
    transform_points_3d,
    Point3D,
    normalize_angle,
    is_transformation_matrix,
    to_transformation_matrix,
    convert_points_to_homogeneous,
    cartesian_to_homogeneous,
    cartesian_to_homogeneous_vectors,
    homogeneous_to_cartesian,
    homogeneous_to_cartesian_vectors,
    homogeneous_translation_matrix
)
import unittest
import numpy as np
from perls.math.transformations import (
    random_rotation_matrix
)


class TestCommon(unittest.TestCase):
    def test_point3d(self):
        pp = np.random.random(3)
        p = Point3D(*pp, dtype=np.float64)
        pb = Point3D.from_array(pp)
        self.assertEqual(p.x, pp[0])
        self.assertEqual(p.y, pp[1])
        self.assertEqual(p.z, pp[2])
        self.assertTrue(np.all(p.array == pp))
        self.assertTrue(np.all(p.homogeneous[:3] == pp))
        self.assertTrue(np.all(p.array == pb.array))

    def test_transform_points_3d(self):
        # identity
        pts = np.random.random((10, 3))
        tpts = transform_points_3d(pts, np.eye(4))
        self.assertTrue(np.all(pts == tpts))

        # random transform and invert
        randomRT = random_rotation_matrix()
        randomRT[:3, 3] += np.random.random(3)
        tpts = transform_points_3d(pts, randomRT)
        self.assertFalse(np.allclose(pts, tpts))
        ipts = transform_points_3d(tpts, np.linalg.inv(randomRT))
        self.assertTrue(np.allclose(pts, ipts))

    def test_normalize_angle(self):
        self.assertEqual(normalize_angle(np.pi * 4 + 1), 1)
        self.assertEqual(normalize_angle(-np.pi * 4 + 1), 1)
        self.assertEqual(normalize_angle(-np.pi * 5), -np.pi)
        self.assertEqual(normalize_angle(-np.pi * 4), 0)
        self.assertEqual(normalize_angle(np.pi * 10000), 0)

    def test_compose_transform_matrix(self):
        randomRT = random_rotation_matrix()
        cRT = to_transformation_matrix(randomRT[:3, :3], randomRT[:3, 3])
        self.assertTrue(np.all(randomRT == cRT))

    def test_is_transformation_matrix(self):
        randomRT = random_rotation_matrix()
        self.assertTrue(is_transformation_matrix(randomRT))
        self.assertFalse(is_transformation_matrix(randomRT + 1))
        randomRT[3, 3] = 0
        self.assertFalse(is_transformation_matrix(randomRT))

    def test_convert_points_to_homogeneous(self):
        random_pts = np.random.random((10, 3))
        h_pts = convert_points_to_homogeneous(random_pts)
        self.assertEqual(h_pts.shape[1], 4)
        self.assertTrue(np.allclose(random_pts, h_pts[:, :3]))
        self.assertTrue(np.allclose(np.ones(random_pts.shape[0]), h_pts[:, 3]))

    def test_cartesian_to_homogeneous(self):
        cat = np.random.random((3, 3))
        hom = np.eye(4)
        hom[:3, :3] = cat
        self.assertTrue(np.allclose(cartesian_to_homogeneous(cat), hom))

    def test_cartesian_to_homogeneous_vectors(self):
        cat = np.random.random(10)
        hom = np.ones(11)
        hom[:10] = cat
        self.assertTrue(
            np.allclose(cartesian_to_homogeneous_vectors(cat), hom))

    def test_homogeneous_to_cartesian(self):
        hom = np.random.random((4, 4))
        cat = hom[:3, :3]
        self.assertTrue(np.allclose(homogeneous_to_cartesian(hom), cat))

    def test_homogeneous_to_cartesian_vectors(self):
        hom = np.random.random(11)
        cat = hom[:10]
        self.assertTrue(
            np.allclose(homogeneous_to_cartesian_vectors(hom), cat))

    def test_homogeneous_translation_matrix(self):
        t = np.random.random(3)
        mat = homogeneous_translation_matrix(t[0], t[1], t[2])
        gmat = np.eye(4)
        gmat[:3, 3] = t
        self.assertTrue(np.allclose(mat, gmat))


if __name__ == '__main__':
    unittest.main()
