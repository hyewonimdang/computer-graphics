import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from calibration_tools import (board_points,fit_calibration,distorted_project,read_pairs,
    pairs_from_directories,load_raw_model,reconstruction_calibration)
from calibrate_stereo import calibrate
from undistort_stereo import undistort_pair
from image_io import save_image,load_image
from camera import load_calibration


class CalibrationTests(unittest.TestCase):
    def model(self):
        K=[[100.,0,32],[0,100,24],[0,0,1]]
        return dict(K_left=K,K_right=K,D_left=[0.]*5,D_right=[0.]*5,
                    R=np.eye(3).tolist(),t=[-.1,0,0],units='m',
                    image_size_left=[64,48],image_size_right=[64,48],
                    camera_model='opencv_pinhole_5',image_state='raw')

    def test_synthetic_calibration_recovers_known_parameters(self):
        obj=board_points(9,6,.025)
        kl=np.array([[700.,0,320],[0,710,240],[0,0,1]])
        kr=np.array([[705.,0,318],[0,695,242],[0,0,1]])
        dl=np.array([-.08,.01,.001,-.001,.002])
        dr=np.array([-.07,.02,-.001,.001,.001])
        R=cv2.Rodrigues(np.array([.01,.025,-.005]))[0]
        t=np.array([-.12,.001,.002])
        objects,left,right=[],[],[]
        rng=np.random.default_rng(10)
        for i in range(15):
            rv=rng.uniform([-.5,-.5,-.3],[.5,.5,.3])
            br=cv2.Rodrigues(rv)[0]
            bt=rng.uniform([-.15,-.12,.55],[.02,.02,1.1])
            # 독립적인 OpenCV projection은 테스트 정답 생성에만 사용한다.
            lc=cv2.projectPoints(obj,rv,bt,kl,dl)[0].reshape(-1,2)
            rc=cv2.projectPoints(obj,cv2.Rodrigues(R@br)[0],R@bt+t,kr,dr)[0].reshape(-1,2)
            np.testing.assert_allclose(distorted_project(obj,br,bt,kl,dl),lc,atol=3e-5)
            objects.append(obj.copy()); left.append(lc); right.append(rc)
        result,metrics=fit_calibration(objects,left,right,(640,480))
        np.testing.assert_allclose(result['K_left'],kl,atol=.02)
        np.testing.assert_allclose(result['K_right'],kr,atol=.02)
        np.testing.assert_allclose(result['R'],R,atol=1e-5)
        np.testing.assert_allclose(result['t'],t,atol=1e-5)
        self.assertLess(metrics['stereo_rms_px'],1e-3)

    def test_pair_manifest_missing_duplicates_and_overlap(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'left').mkdir(); (root/'right').mkdir()
            (root/'left'/'001.png').touch(); (root/'right'/'001.png').touch()
            self.assertEqual(len(pairs_from_directories(root/'left',root/'right')),1)
            manifest=root/'pairs.json'
            manifest.write_text(json.dumps([dict(left='left/001.png',right='right/001.png')]))
            pairs=read_pairs(manifest)
            with self.assertRaises(ValueError):
                calibrate(pairs,root/'result',9,6,1,'board_square',pairs)
            manifest.write_text(json.dumps([dict(left='left/001.png',right='left/001.png')]))
            with self.assertRaises(ValueError):
                read_pairs(manifest)
            (root/'right'/'002.png').touch()
            with self.assertRaises(ValueError):
                pairs_from_directories(root/'left',root/'right')
            with self.assertRaises(ValueError):
                board_points(9,6,0)

    def test_undistortion_identity_preserves_k_and_rejects_wrong_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); model=self.model()
            mp=root/'raw.json'; mp.write_text(json.dumps(model))
            image=np.random.default_rng(5).integers(0,256,(48,64,3),dtype=np.uint8)
            lp=root/'l.png'; rp=root/'r.png'
            save_image(lp,image); save_image(rp,image)
            undistort_pair(lp,rp,mp,root/'output')
            np.testing.assert_array_equal(load_image(root/'output'/'left.png'),image)
            cal=json.loads((root/'output'/'calibration.json').read_text())
            for key in ('K_left','K_right','R','t','units','image_size_left'):
                self.assertEqual(cal[key],model[key])
            load_calibration(root/'output'/'calibration.json',(48,64,3),(48,64,3))
            with self.assertRaises(ValueError):
                load_calibration(mp)
            with self.assertRaises(ValueError):
                undistort_pair(lp,rp,root/'output'/'calibration.json',root/'twice')
            with self.assertRaises(ValueError):
                undistort_pair(root/'output'/'left.png',root/'output'/'right.png',mp,root/'output')
            save_image(lp,image[:20])
            with self.assertRaises(ValueError):
                undistort_pair(lp,rp,mp,root/'wrong_size')

    def test_invalid_distortion_and_detection_failures(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); model=self.model(); model['D_left']=[float('nan')]*5
            mp=root/'raw.json'; mp.write_text(json.dumps(model))
            with self.assertRaises(ValueError):
                load_raw_model(mp)
            lp=root/'l.png'; rp=root/'r.png'
            save_image(lp,np.zeros((48,64,3),np.uint8)); save_image(rp,np.zeros((48,64,3),np.uint8))
            with self.assertRaises(ValueError):
                calibrate([(lp,rp)],root/'failed',9,6,1,'board_square')
            report=json.loads((root/'failed'/'calibration_report.json').read_text())
            self.assertEqual(report['detected_pairs'],0)
            self.assertEqual(len(report['rejected_pairs']),1)
            self.assertFalse((root/'failed'/'raw_camera_model.json').exists())
