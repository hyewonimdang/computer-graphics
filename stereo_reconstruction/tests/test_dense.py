import unittest
from unittest.mock import patch
import numpy as np
from dense_stereo import rectification_geometry,box_sum,dense_correspondences
from geometry import from_homogeneous,to_homogeneous
from camera import build_projection_matrix
from triangulation import triangulate_points_dlt

class DenseTests(unittest.TestCase):
    def test_integral_box_sum_matches_direct(self):
        a=np.arange(35).reshape(5,7); padded=np.pad(a,1)
        expected=np.array([[padded[y:y+3,x:x+3].sum() for x in range(7)] for y in range(5)])
        np.testing.assert_array_equal(box_sum(a,1),expected)

    def test_rectification_equal_rows_and_original_coordinates(self):
        K=np.array([[150.,0,80],[0,145,50],[0,0,1]])
        angle=.08; R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        t=np.array([-.4,.02,.01]); kn,ql,qr,B=rectification_geometry(K,K,R,t,(160,100))
        np.testing.assert_allclose(ql@ql.T,np.eye(3),atol=1e-12)
        np.testing.assert_allclose(qr@t,[-B,0,0],atol=1e-12)
        X=np.random.default_rng(1).uniform([-1,-1,4],[1,1,8],(50,3))
        l=from_homogeneous((X@ql.T)@kn.T); r=from_homogeneous(((X@R.T+t)@qr.T)@kn.T)
        np.testing.assert_allclose(l[:,1],r[:,1],atol=1e-10)
        self.assertTrue(((l-r)[:,0]>0).all())
        ol=from_homogeneous(to_homogeneous(l)@(K@ql.T@np.linalg.inv(kn)).T)
        ore=from_homogeneous(to_homogeneous(r)@(K@qr.T@np.linalg.inv(kn)).T)
        reconstructed=triangulate_points_dlt(ol,ore,build_projection_matrix(K,np.eye(3),[0,0,0]),build_projection_matrix(K,R,t))
        np.testing.assert_allclose(reconstructed,X,atol=1e-10)

    def test_known_disparity_and_occlusion(self):
        rng=np.random.default_rng(12); left=rng.integers(0,256,(80,140,3),dtype=np.uint8)
        right=np.zeros_like(left); right[:,:-12]=left[:,12:]
        with patch('cv2.StereoSGBM_create',side_effect=AssertionError('custom matching only')):
            result=dense_correspondences(left,right,max_disparity=24,window=5)
        interior=np.zeros((80,140),bool); interior[10:-10,40:-10]=True
        self.assertGreater(result['valid'][interior].mean(),.9)
        self.assertLess(np.median(np.abs(result['disparity'][result['valid']]-12)),.1)
        right[20:60,45:75]=0
        result=dense_correspondences(left,right,max_disparity=24,window=5)
        self.assertLess(result['valid'][28:52,65:79].mean(),.1)

    def test_flat_and_invalid_images(self):
        flat=np.full((50,80,3),100,np.uint8)
        self.assertFalse(dense_correspondences(flat,flat,max_disparity=20,window=5)['valid'].any())
        with self.assertRaises(ValueError): dense_correspondences(flat,flat,max_disparity=20,lr_threshold=float('nan'))
        with self.assertRaises(ValueError): dense_correspondences(flat,flat,max_disparity=100)
        with self.assertRaises(ValueError): dense_correspondences(flat,flat,max_disparity=20,window=4)
        with self.assertRaises(ValueError): rectification_geometry(np.eye(3),np.eye(3),np.eye(3),np.zeros(3),(80,50))
