import unittest
from unittest.mock import patch
import numpy as np
from camera import build_projection_matrix
from reprojection import project_point, project_points, compute_reprojection_error
from point_cloud import validate_3d_points
from filtering import filter_reconstructed_points


class Phase5Tests(unittest.TestCase):
    def setUp(self):
        self.K = np.array([[100.,0,50],[0,100,50],[0,0,1]])
        self.R, self.t = np.eye(3), np.array([-1.,0,0])
        self.P1 = build_projection_matrix(self.K, self.R, [0,0,0])
        self.P2 = build_projection_matrix(self.K, self.R, self.t)

    def test_projection_known_pixels_and_error(self):
        with patch('cv2.projectPoints', side_effect=AssertionError('must use NumPy')):
            np.testing.assert_allclose(project_point(self.P1, [1,2,10]), [60,70])
            np.testing.assert_allclose(project_point(self.P2, [1,2,10]), [50,70])
            points = project_points(self.P1, [[1,2,10], [0,0,0], [np.nan,0,1]])
        self.assertTrue(np.isnan(points[1:]).all())
        self.assertEqual(compute_reprojection_error([0,0], [3,4]), 5)
        errors = compute_reprojection_error([[0,0],[np.nan,0]], [[3,4],[1,2]])
        self.assertEqual(errors[0], 5)
        self.assertTrue(np.isinf(errors[1]))
        self.assertEqual(project_points(self.P1, np.empty((0,3))).shape, (0,2))

    def test_depth_in_both_camera_frames(self):
        # 右 camera is rotated 180 degrees around Y; left-positive point is behind right.
        R = np.diag([-1.,1,-1])
        mask = validate_3d_points([[0,0,5]], R, [1,0,0])
        self.assertFalse(mask[0])
        points = [[0,0,5], [0,0,-1], [0,0,0], [0,0,11], [20,0,5], [np.inf,0,1]]
        np.testing.assert_array_equal(validate_3d_points(points, max_depth=10, max_distance=10),
                                      [True,False,False,False,False,False])
        with self.assertRaises(ValueError):
            validate_3d_points([[0,0,1]], min_depth=2, max_depth=1)

    def test_sequential_reasons_and_both_reprojection_errors(self):
        xyz = np.array([[0,0,5], [np.nan,0,1], [0,0,-1], [20,0,5],
                        [0,0,5], [0,0,5], [0,0,5]])
        left = project_points(self.P1, xyz)
        right = project_points(self.P2, xyz)
        left[4] = [-1,50]  # invalid image coordinate
        right[5,0] += 3   # mean=1.5<2, but right error=3>2 must be removed
        right[6,0] += 2   # threshold boundary is accepted
        result = filter_reconstructed_points(xyz, left, right, self.P1, self.P2,
                    self.R, self.t, left_shape=(100,100,3), right_shape=(100,100,3),
                    max_depth=10, max_distance=10, reprojection_threshold=2)
        np.testing.assert_array_equal(result['final_mask'], [True,False,False,False,False,False,True])
        self.assertEqual(result['removal_counts'], dict(numerical=1,depth=1,distance=1,image_bounds=1,reprojection=1))
        self.assertEqual(sum(result['removal_counts'].values()), 5)

    def test_empty_and_all_rejected(self):
        result = filter_reconstructed_points(np.empty((0,3)), np.empty((0,2)), np.empty((0,2)),
                                             self.P1,self.P2,self.R,self.t)
        self.assertEqual(result['final_mask'].shape, (0,))
        result = filter_reconstructed_points([[0,0,5]], [[50,50]], [[30,50]],
                    self.P1,self.P2,self.R,self.t, numerical_valid=np.array([False]))
        self.assertFalse(result['final_mask'][0])
