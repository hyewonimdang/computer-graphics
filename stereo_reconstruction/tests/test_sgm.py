import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from sgm_stereo import aggregate_sgm, path_step, sgm_correspondences, guided_cost, DIRECTIONS
from benchmark_sgm import read_pfm, metrics, boundary_mask, export_reconstruction


class SGMTests(unittest.TestCase):
    def test_nonzero_principal_point_offset_dlt_and_export(self):
        # 비대칭 cx를 무시하면 깊이가100이 되지만 실제 정답은50이다.
        k0=np.array([[100.,0,20],[0,100.,10],[0,0,1]])
        k1=k0.copy(); k1[0,2]+=10
        d=np.full((30,50),10,np.float32); mask=np.zeros(d.shape,bool); mask[10,20]=True
        result=dict(disparity=d,valid=mask,confidence=mask.astype(np.float32))
        with tempfile.TemporaryDirectory() as td:
            final=export_reconstruction(result,np.full((30,50,3),100,np.uint8),dict(cam0=k0,cam1=k1,baseline=10.,doffs=10.),td)
            c=np.load(Path(td)/'point_cloud.npy')
            np.testing.assert_allclose(c[0,:3],[0,0,50],atol=1e-10)
            self.assertEqual(int(final.sum()),1)

    def test_sgm_full_pipeline_exports_aligned_confidence(self):
        import cv2,json
        from dense_reconstruct import run
        rng=np.random.default_rng(17)
        left=rng.integers(50,180,(32,72,3),dtype=np.uint8)
        right=np.zeros_like(left); right[:,:-8]=left[:,8:]
        k=[[100.,0,35.5],[0,100.,15.5],[0,0,1]]
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            cv2.imwrite(str(root/'l.png'),left); cv2.imwrite(str(root/'r.png'),right)
            (root/'cal.json').write_text(json.dumps(dict(K_left=k,K_right=k,R=np.eye(3).tolist(),t=[-1,0,0],units='m')))
            cloud=run(root/'l.png',root/'r.png',root/'cal.json',root/'out',max_disparity=16,method='sgm')
            self.assertGreater(len(cloud),100)
            with np.load(root/'out/point_attributes.npz') as a:
                self.assertEqual(len(a['confidence']),len(cloud))
                self.assertEqual(int(a['valid_mask'].sum()),len(cloud))
                self.assertTrue((a['confidence']>=.35).all())
                self.assertLess(np.median(np.abs(cloud[:,2]-12.5)),.2)

    def test_guided_cost_does_not_average_across_strong_edge(self):
        cost=np.zeros((5,8),np.float32); cost[:,4:]=20
        guide=np.zeros((5,8),np.float32); guide[:,4:]=200
        weighted=guided_cost(cost,guide,guide,np.ones(cost.shape,bool))
        self.assertLess(weighted[2,3],.001)
        self.assertGreater(weighted[2,4],19.999)
        uniform=guided_cost(cost,np.zeros_like(guide),np.zeros_like(guide),np.ones(cost.shape,bool))
        self.assertAlmostEqual(float(uniform[2,3]),20/3,places=5)

    def test_eight_paths_against_scalar_reference(self):
        rng=np.random.default_rng(5)
        cost=rng.uniform(0,8,(5,4,6)).astype(np.float32)
        guide=rng.integers(0,256,(4,6)).astype(float)
        expected=np.zeros_like(cost)
        for dy,dx in DIRECTIONS:
            path=np.zeros_like(cost)
            for y in (range(4) if dy>=0 else range(3,-1,-1)):
                for x in (range(6) if dx>=0 else range(5,-1,-1)):
                    py,px=y-dy,x-dx
                    if not (0<=py<4 and 0<=px<6): path[:,y,x]=cost[:,y,x]; continue
                    prev=path[:,py,px]
                    p2=max(2.001,24/(1+abs(guide[y,x]-guide[py,px])/10))
                    for d in range(5):
                        choices=[prev[k]+(0 if k==d else 2 if abs(k-d)==1 else p2) for k in range(5)]
                        path[d,y,x]=cost[d,y,x]+min(choices)-min(prev)
            expected+=path/8
        np.testing.assert_allclose(aggregate_sgm(cost,guide),expected,atol=5e-6)

    def test_edge_adaptation_preserves_sharp_jump(self):
        cost=np.full((8,1,20),5.,np.float32)
        cost[2,:,:10]=0; cost[6,:,10:]=0
        guide=np.zeros((1,20)); guide[:,10:]=200
        adaptive=aggregate_sgm(cost,guide,directions=((0,1),)).argmin(0)
        fixed=aggregate_sgm(cost,guide,edge_scale=None,directions=((0,1),)).argmin(0)
        self.assertEqual(adaptive[0,10],6)
        self.assertNotEqual(fixed[0,10],6)

    def test_step_is_invariant_to_previous_constant(self):
        rng=np.random.default_rng(2); c=rng.random((6,3)); p=rng.random((6,3))*9
        np.testing.assert_allclose(path_step(c,p,2,24),path_step(c,p+400,2,24),atol=1e-12)

    def test_synthetic_shift_occlusion_and_threshold_nesting(self):
        rng=np.random.default_rng(10)
        left=rng.integers(50,180,(48,96,3),dtype=np.uint8)
        right=np.zeros_like(left); right[:,:-8]=left[:,8:]
        with patch('cv2.StereoSGBM_create',side_effect=AssertionError('forbidden')), \
             patch('cv2.StereoBM_create',side_effect=AssertionError('forbidden')):
            result=sgm_correspondences(left,right,max_disparity=16)
        mask=result['valid']; self.assertGreater(mask[8:-8,24:-8].mean(),.6)
        self.assertLess(np.median(np.abs(result['disparity'][mask]-8)),.1)
        self.assertTrue(np.all(result['confidence'][~mask]==0))
        self.assertTrue(np.all((result['confidence']>=0)&(result['confidence']<=1)))
        stricter=sgm_correspondences(left,right,max_disparity=16,min_confidence=.6)
        self.assertFalse((stricter['valid']&~mask).any())
        right[12:36,35:55]=0
        occluded=sgm_correspondences(left,right,max_disparity=16)
        self.assertLess(occluded['valid'][17:31,47:59].mean(),.1)

    def test_flat_invalid_and_masks(self):
        left=np.full((20,40,3),100,np.uint8)
        self.assertFalse(sgm_correspondences(left,left,max_disparity=10)['valid'].any())
        for kw in [dict(p1=0),dict(p2=1),dict(edge_scale=0),dict(min_confidence=float('nan')),
                   dict(max_disparity=40),dict(valid_left=np.zeros((2,2))),dict(support=9)]:
            with self.assertRaises(ValueError): sgm_correspondences(left,left,max_disparity=10,**kw) if 'max_disparity' not in kw else sgm_correspondences(left,left,**kw)
        rng=np.random.default_rng(1); left=rng.integers(0,255,(20,40,3),dtype=np.uint8)
        self.assertFalse(sgm_correspondences(left,left,max_disparity=10,valid_left=np.zeros((20,40),bool))['valid'].any())

    def test_middlebury_pfm_endian_orientation_and_raw_pixel_units(self):
        expected=np.array([[1,2,3],[4,np.inf,6]],np.float32)
        with tempfile.TemporaryDirectory() as td:
            for scale,endian in [(-.003922,'<'),(.003922,'>')]:
                p=Path(td)/'image.pfm'
                p.write_bytes(f'Pf\n3 2\n{scale}\n'.encode()+np.flipud(expected).astype(endian+'f4').tobytes())
                np.testing.assert_equal(read_pfm(p),expected)

    def test_metrics_count_rejections_and_common_mask(self):
        gt=np.array([[2.,4.,np.inf],[6.,8.,10.]])
        prediction=np.array([[2.,7.,3.],[6.,8.,11.]])
        accepted=np.array([[1,1,1],[0,0,1]],bool)
        m=metrics(prediction,accepted,gt,np.ones(gt.shape,bool))
        self.assertEqual(m['gt_pixels'],5); self.assertEqual(m['evaluated_pixels'],3)
        self.assertAlmostEqual(m['mae_px'],4/3)
        self.assertEqual(m['missing_or_bad2_percent'],60)
        self.assertFalse(boundary_mask(gt)[0,2])


if __name__=='__main__': unittest.main()
