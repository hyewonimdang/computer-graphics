import tempfile
import json
import unittest
from pathlib import Path
import numpy as np
from camera import (compute_essential_matrix, enforce_essential_constraints,
                    build_projection_matrix, skew_symmetric, load_calibration)


class Phase3Tests(unittest.TestCase):
    def test_essential_relation_and_singular_values(self):
        kl = np.array([[600, 2, 320], [0, 620, 240], [0, 0, 1.]])
        kr = np.array([[700, 0, 310], [0, 680, 250], [0, 0, 1.]])
        a = .12
        R = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
        t = np.array([-.4, .02, .03])
        expected = skew_symmetric(t) @ R
        F = np.linalg.inv(kr).T @ expected @ np.linalg.inv(kl)
        np.testing.assert_allclose(compute_essential_matrix(F, kl, kr), expected, atol=1e-14)
        noisy = expected + np.random.default_rng(3).normal(0, .01, (3, 3))
        singular = np.linalg.svd(enforce_essential_constraints(noisy), compute_uv=False)
        self.assertAlmostEqual(singular[0], singular[1], places=12)
        self.assertLess(singular[2], 1e-14)
        np.testing.assert_allclose(skew_symmetric(t) @ [1,2,3], np.cross(t, [1,2,3]), atol=1e-14)

    def test_projection_coordinates_and_camera_center(self):
        K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1.]])
        P1 = build_projection_matrix(K, np.eye(3), np.zeros(3))
        P2 = build_projection_matrix(K, np.eye(3), [-.4, 0, 0])
        x1 = P1 @ [1, .5, 5, 1]
        x2 = P2 @ [1, .5, 5, 1]
        np.testing.assert_allclose(x1[:2]/x1[2], [440, 300])
        np.testing.assert_allclose(x2[:2]/x2[2], [392, 300])
        np.testing.assert_allclose(P2 @ [.4, 0, 0, 1], 0)

    def test_calibration_validation(self):
        with self.assertRaises(ValueError):
            build_projection_matrix(np.eye(3), np.diag([1,1,-1]), [1,0,0])
        with self.assertRaises(ValueError):
            build_projection_matrix(np.zeros((3,3)), np.eye(3), [1,0,0])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'calibration.json'
            data = dict(K_left=np.eye(3).tolist(), K_right=np.eye(3).tolist(),
                        R=np.eye(3).tolist(), t=[-.4,0,0], image_size_left=[900,600])
            path.write_text(json.dumps(data))
            self.assertEqual(load_calibration(path, (600,900,3))[0].shape, (3,3))
            with self.assertRaises(ValueError):
                load_calibration(path, (300,450,3))
            data['t'] = [0,0,0]
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_calibration(path)
