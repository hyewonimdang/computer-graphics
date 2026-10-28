import unittest
import numpy as np
from geometry import to_homogeneous, from_homogeneous
from epipolar import (estimate_fundamental_matrix, compute_epipolar_errors,
                      epipolar_constraint_error, compute_epipolar_distances, point_to_line_distances)
from visualization import line_segment_in_image


class Phase2Tests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(7)
        xyz = rng.uniform([-2, -1.5, 4], [2, 1.5, 12], (150, 3))
        self.left = xyz[:, :2] / xyz[:, 2:] * 600 + [320, 240]
        shifted = xyz - [0.4, 0, 0]
        self.right = shifted[:, :2] / shifted[:, 2:] * 600 + [320, 240]
        self.F = np.array([[0., 0, 0], [0, 0, 1], [0, -1, 0]])

    def test_homogeneous_roundtrip_and_infinity(self):
        for points in ([1, 2], [[1, 2, 3], [4, 5, 6]], np.empty((0, 2))):
            np.testing.assert_allclose(from_homogeneous(to_homogeneous(points)), points)
        np.testing.assert_allclose(from_homogeneous(np.array([2., 4, 2]) * 1e-30), [1, 2])
        self.assertTrue(np.isnan(from_homogeneous([[1, 2, 0], [np.inf, 2, 1], [0, 0, 0]])).all())

    def test_known_geometry_and_pixel_distance(self):
        np.testing.assert_allclose(compute_epipolar_errors(self.F, self.left, self.right), 0, atol=1e-10)
        perturbed = self.right.copy()
        perturbed[:, 1] += 3
        self.assertAlmostEqual(epipolar_constraint_error(self.F, self.left[0], perturbed[0]), 3)
        for scale in (1, -10, 1e-20):
            dl, dr = compute_epipolar_distances(self.F * scale, self.left, perturbed)
            np.testing.assert_allclose(dl, 3, atol=1e-9)
            np.testing.assert_allclose(dr, 3, atol=1e-9)
        self.assertTrue(np.isinf(point_to_line_distances([[0, 0]], [[0, 0, 0]])[0]))

    def test_ransac_with_outliers(self):
        right = self.right.copy()
        right[:30] = np.random.default_rng(1).uniform([0, 0], [640, 480], (30, 2))
        F, mask = estimate_fundamental_matrix(self.left, right, threshold=0.5)
        self.assertEqual(np.linalg.matrix_rank(F), 2)
        self.assertGreaterEqual(mask[30:].sum(), 110)
        self.assertLessEqual(mask[:30].sum(), 5)
        dl, dr = compute_epipolar_distances(F, self.left[30:], self.right[30:])
        self.assertLess(max(dl.mean(), dr.mean()), 0.01)

    def test_degenerate_and_insufficient_points(self):
        with self.assertRaises(ValueError):
            estimate_fundamental_matrix(self.left[:7], self.right[:7])
        with self.assertRaises(ValueError):
            estimate_fundamental_matrix(np.zeros((10, 2)), np.zeros((10, 2)))
        self.assertEqual(line_segment_in_image([1, 0, -20], 100, 80), ((20, 0), (20, 79)))
        self.assertEqual(line_segment_in_image([0, 1, -10], 100, 80), ((0, 10), (99, 10)))
        self.assertIsNone(line_segment_in_image([0, 0, 0], 100, 80))


if __name__ == '__main__':
    unittest.main()
