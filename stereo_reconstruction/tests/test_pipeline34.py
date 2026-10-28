import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from camera import build_projection_matrix, skew_symmetric
from main import run_phase3, run_phase4


class PipelineTests(unittest.TestCase):
    def test_artifact_mapping_and_optional_opencv(self):
        K, R, t = np.eye(3), np.eye(3), np.array([-1.,0,0])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'diagnostics.json').write_text('{}')
            P1, P2 = run_phase3(skew_symmetric(t), (K,K,R,t,'m'), root)
            pl = np.array([[0,0], [0,0], [0,0.]])
            pr = np.array([[-.2,0], [-.1,0], [0,0.]])
            with patch('cv2.triangulatePoints', side_effect=AssertionError('core must not use OpenCV')):
                xyz = run_phase4(pl, pr, np.array([True,False,True]), P1, P2, root)
            np.testing.assert_allclose(xyz[0], [0,0,5], atol=1e-12)
            self.assertTrue(np.isnan(xyz[1]).all())
            with np.load(root/'triangulation_data.npz') as data:
                np.testing.assert_array_equal(data['source_indices'], [0,2])
                np.testing.assert_array_equal(data['numerical_valid'], [True,False])
            stats = json.loads((root/'diagnostics.json').read_text())
            self.assertEqual(stats['phase'], 4)
            self.assertIsNone(stats['opencv_comparison'])
            self.assertFalse((root/'opencv_comparison.npz').exists())
