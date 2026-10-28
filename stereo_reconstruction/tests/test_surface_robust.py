import unittest
import numpy as np
from surface_attributes import estimate_surface
from benchmark_surface import synthetic_case,angular

class RobustSurfaceTests(unittest.TestCase):
    def test_noisy_plane_and_boundary_accuracy(self):
        xyz,uv,size,truth,boundary=synthetic_case()
        old=estimate_surface(xyz,uv,size,method='legacy');new=estimate_surface(xyz,uv,size)
        self.assertGreater(new['valid'].sum(),old['valid'].sum())
        valid=old['valid']&new['valid']
        self.assertLess(np.median(angular(new['normal'][valid],truth[valid])),np.median(angular(old['normal'][valid],truth[valid]))*.3)
        edge=boundary&new['valid'];self.assertLess(np.median(angular(new['normal'][edge],truth[edge])),10)
        self.assertTrue(np.all(new['valid'][old['valid']]))
    def test_exact_depth_step_preserves_two_normal_directions(self):
        h,w=25,30;y,x=np.mgrid[:h,:w];rays=np.stack([(x-15)/150,(y-12)/150,np.ones_like(x)],axis=-1)
        truth=np.zeros((h,w,3));truth[:,:15]=[.3,.2,-1];truth[:,15:]=[-.4,-.1,-1];truth/=np.linalg.norm(truth,axis=2)[...,None]
        z=np.where(x<15,-3.,-6.)/np.sum(rays*truth,axis=2)
        xyz=(rays*z[...,None]).reshape(-1,3);uv=np.column_stack([x.ravel(),y.ravel()])
        a=estimate_surface(xyz,uv,(w,h));edge=np.abs(uv[:,0]-14.5)<=3
        self.assertTrue(a['valid'].all());self.assertLess(np.max(angular(a['normal'][edge],truth.reshape(-1,3)[edge])),.03)
    def test_disconnected_island_not_filled_and_collinear_rejected(self):
        u=np.arange(12);xyz=np.column_stack([(u-6)*.02,np.zeros(12),np.full(12,3.)]);uv=np.column_stack([u,np.full(12,4)])
        a=estimate_surface(xyz,uv,(12,9));self.assertFalse(a['valid'].any());self.assertFalse(a['plane_fitted'].any())
        b=estimate_surface([[0,0,3]],[[4,4]],(9,9));self.assertFalse(b['valid'][0]);self.assertEqual(b['radius_m'][0],0)
