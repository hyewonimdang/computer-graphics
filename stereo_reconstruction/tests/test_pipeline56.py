import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from camera import build_projection_matrix
from reprojection import project_points
from main import run_phase5, run_phase6


class Pipeline56Tests(unittest.TestCase):
    def test_filter_color_export_alignment_and_empty(self):
        K = np.array([[100.,0,50],[0,100,50],[0,0,1]])
        R, t = np.eye(3), np.array([-1.,0,0])
        P1, P2 = build_projection_matrix(K,R,[0,0,0]), build_projection_matrix(K,R,t)
        xyz = np.array([[0,0,5],[0,0,-5],[.5,0,5]])
        pl, pr = project_points(P1,xyz), project_points(P2,xyz)
        image = np.zeros((100,100,3),np.uint8)
        image[50,50] = [10,20,30]
        image[50,60] = [40,50,60]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            np.savez(root/'triangulation_data.npz', points_3d=xyz, points_left=pl,
                     points_right=pr, source_indices=[4,8,12], numerical_valid=np.ones(3,bool))
            for max_depth, expected_count in ((10,2),(1,0)):
                (root/'diagnostics.json').write_text('{}')
                with patch('cv2.projectPoints', side_effect=AssertionError('must use NumPy')):
                    accepted, observations, indices = run_phase5(image,image,P1,P2,
                       (K,K,R,t,'m'),root,max_depth=max_depth)
                cloud = run_phase6(accepted,image,observations,indices,'m',root,csv=True)
                self.assertEqual(cloud.shape,(expected_count,6))
                data = json.loads((root/'07_point_cloud.json').read_text())
                self.assertEqual(data['count'],expected_count)
                self.assertEqual(data['points'],cloud.tolist())
                self.assertTrue((root/'05_reprojection.jpg').is_file())
                if expected_count:
                    np.testing.assert_array_equal(indices,[4,12])
                    np.testing.assert_array_equal(cloud[:,3:],[[30,20,10],[60,50,40]])
                    np.testing.assert_array_equal(np.load(root/'point_cloud_source_indices.npy'),indices)
                else:
                    self.assertIn('element vertex 0', (root/'06_point_cloud.ply').read_text())
                    stats=json.loads((root/'diagnostics.json').read_text())
                    self.assertIsNone(stats['mean_left_reprojection_error_px'])
                    self.assertEqual(stats['exported_points'],0)
