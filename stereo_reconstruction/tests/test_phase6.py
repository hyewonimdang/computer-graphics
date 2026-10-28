import csv
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from point_cloud import assign_colors
from exporter import export_json, export_ply, export_csv


class Phase6Tests(unittest.TestCase):
    def test_rgb_order_rounding_and_border(self):
        image = np.array([[[1,2,3],[10,20,30]], [[40,50,60],[70,80,90]]], np.uint8)
        xyz = np.array([[1,2,3],[4,5,6],[7,8,9.]])
        cloud = assign_colors(xyz, image, [[0,0],[.5,0],[1.9,1.9]])
        np.testing.assert_array_equal(cloud[:, :3], xyz)
        np.testing.assert_array_equal(cloud[:, 3:], [[3,2,1],[30,20,10],[90,80,70]])
        for point in ([-.01,0], [2,0], [np.nan,0]):
            with self.assertRaises(ValueError):
                assign_colors(xyz[:1], image, [point])
        self.assertEqual(assign_colors(np.empty((0,3)), image, np.empty((0,2))).shape, (0,6))

    def test_export_roundtrip_types_precision_and_empty(self):
        cloud = np.array([[.12345678901234567,-2,3.5,255,0,17],[4,5,6,1,2,3.]])
        with tempfile.TemporaryDirectory() as tmp:
            for sample in (cloud, np.empty((0,6))):
                root = Path(tmp)
                export_json(sample, root/'points.json', units='m')
                export_ply(sample, root/'points.ply')
                export_csv(sample, root/'points.csv')
                data = json.loads((root/'points.json').read_text())
                self.assertEqual(data['count'], len(sample))
                self.assertEqual(data['units'], 'm')
                self.assertEqual(data['columns'], ['X','Y','Z','R','G','B'])
                np.testing.assert_array_equal(np.array(data['points']).reshape(-1,6), sample)
                for row in data['points']:
                    self.assertTrue(all(isinstance(c,int) for c in row[3:]))
                lines = (root/'points.ply').read_text().splitlines()
                self.assertEqual(lines[0:2], ['ply','format ascii 1.0'])
                self.assertIn(f'element vertex {len(sample)}', lines)
                self.assertIn('property uchar red', lines)
                rows = lines[lines.index('end_header')+1:]
                self.assertEqual(len(rows),len(sample))
                decoded = np.array([[float(x) for x in row.split()] for row in rows]).reshape(-1,6)
                np.testing.assert_array_equal(decoded,sample)
                for row in rows:
                    self.assertTrue(all(0 <= int(c) <= 255 for c in row.split()[3:]))
                with (root/'points.csv').open() as f:
                    self.assertEqual(len(list(csv.reader(f))),len(sample)+1)

    def test_invalid_cloud_rejected_before_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'cloud.json'
            path.write_text('preserve')
            for row in ([0,0,np.nan,0,0,0], [0,0,1,256,0,0], [0,0,1,1.5,0,0]):
                for exporter in (export_json, export_ply, export_csv):
                    with self.assertRaises(ValueError):
                        exporter([row],path)
                self.assertEqual(path.read_text(),'preserve')
