import unittest
import numpy as np
from camera import build_projection_matrix
from geometry import to_homogeneous, from_homogeneous
from triangulation import triangulate_point_dlt, triangulate_points_dlt
from validation import compare_with_opencv


class Phase4Tests(unittest.TestCase):
    def setUp(self):
        kl = np.array([[600, 2, 320], [0, 620, 240], [0, 0, 1.]])
        kr = np.array([[700, 0, 310], [0, 680, 250], [0, 0, 1.]])
        a = .12
        self.R = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
        self.P1 = build_projection_matrix(kl, np.eye(3), [0,0,0])
        self.P2 = build_projection_matrix(kr, self.R, [-.4,.02,.03])
        self.xyz = np.random.default_rng(9).uniform([-2,-1,4], [2,1,10], (100,3))
        self.p1 = from_homogeneous(to_homogeneous(self.xyz) @ self.P1.T)
        self.p2 = from_homogeneous(to_homogeneous(self.xyz) @ self.P2.T)

    def test_known_3d_rotated_cameras(self):
        xyz, details = triangulate_points_dlt(self.p1, self.p2, self.P1, self.P2, return_details=True)
        np.testing.assert_allclose(xyz, self.xyz, atol=1e-10)
        self.assertTrue(details['numerical_valid'].all())
        np.testing.assert_allclose(triangulate_point_dlt(self.p1[0], self.p2[0], self.P1, self.P2), self.xyz[0], atol=1e-10)
        np.testing.assert_allclose(triangulate_points_dlt(self.p1, self.p2, -3*self.P1, 7*self.P2), self.xyz, atol=1e-10)

    def test_noisy_opencv_comparison(self):
        p2 = self.p2 + np.random.default_rng(3).normal(0, .2, self.p2.shape)
        xyz = triangulate_points_dlt(self.p1, p2, self.P1, self.P2)
        comparison = compare_with_opencv(self.p1, p2, self.P1, self.P2, xyz)
        self.assertEqual(comparison['compared_count'], 100)
        self.assertLess(comparison['max_difference'], 1e-9)
        # 정답까지의 오차도 검사: 두 구현이 같은 오류를 내는 경우를 구분한다.
        self.assertLess(np.linalg.norm(xyz-self.xyz, axis=1).mean(), .1)

    def test_infinity_nonfinite_empty_and_degenerate(self):
        P1 = build_projection_matrix(np.eye(3), np.eye(3), [0,0,0])
        P2 = build_projection_matrix(np.eye(3), np.eye(3), [-1,0,0])
        xyz, details = triangulate_points_dlt([[0,0], [np.nan,0]], [[0,0],[1,0]], P1, P2, True)
        self.assertTrue(np.isnan(xyz).all())
        self.assertFalse(details['numerical_valid'].any())
        self.assertEqual(triangulate_points_dlt(np.empty((0,2)), np.empty((0,2)), P1, P2).shape, (0,3))
        with self.assertRaises(ValueError):
            triangulate_points_dlt(self.p1, self.p2, P1, P1)
        with self.assertRaises(ValueError):
            triangulate_points_dlt(self.p1, self.p2[:-1], P1, P2)
        with self.assertRaises(ValueError):
            triangulate_points_dlt(self.p1, self.p2, np.zeros((3,4)), P2)

    def test_scale_and_behind_camera_preserved_for_phase5(self):
        P1 = build_projection_matrix(np.eye(3), np.eye(3), [0,0,0])
        P2 = build_projection_matrix(np.eye(3), np.eye(3), [-.4,0,0])
        P2_twice = build_projection_matrix(np.eye(3), np.eye(3), [-.8,0,0])
        X = np.array([[.2,.1,4], [.3,.2,-3]])
        p1 = from_homogeneous(to_homogeneous(X) @ P1.T)
        p2 = from_homogeneous(to_homogeneous(X) @ P2.T)
        np.testing.assert_allclose(triangulate_points_dlt(p1,p2,P1,P2), X, atol=1e-12)
        np.testing.assert_allclose(triangulate_points_dlt(p1,p2,P1,P2_twice), 2*X, atol=1e-12)
