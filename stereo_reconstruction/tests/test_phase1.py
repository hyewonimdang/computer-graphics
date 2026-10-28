import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from image_io import save_image, load_stereo_images
from features import detect_features, match_features, matched_points
from main import run_phase1


class Phase1Tests(unittest.TestCase):
    def test_image_pipeline_and_correspondences(self):
        rng = np.random.default_rng(12)
        left = rng.integers(0, 256, (360, 480, 3), dtype=np.uint8)
        right = cv2.warpAffine(left, np.float32([[1, 0, -12], [0, 1, 0]]), (480, 360))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            save_image(root / '왼쪽.png', left)
            save_image(root / 'right.png', right)
            _, _, pl, pr, stats = run_phase1(root / '왼쪽.png', root / 'right.png', root)
            self.assertGreater(stats['filtered_matches'], 30)
            np.testing.assert_allclose(np.median(pr - pl, axis=0), [-12, 0], atol=0.2)
            self.assertEqual(pl.shape[1], 2)
            self.assertEqual(len(list(root.glob('*.jpg'))), 3)

    def test_no_features_and_missing_file(self):
        _, descriptors = detect_features(np.zeros((100, 100, 3), np.uint8))
        self.assertEqual(match_features(descriptors, descriptors), ([], []))
        self.assertEqual(matched_points([], [], [])[0].shape, (0, 2))
        with self.assertRaises(FileNotFoundError):
            load_stereo_images('missing-left.png', 'missing-right.png')


if __name__ == '__main__':
    unittest.main()
