import unittest
import numpy as np
from surface_attributes import estimate_surface,make_records,packetize_surface,SURFACE_HEADER


def plane(w=15,h=13,normal=(0,0,-1),offset=-3):
    y,x=np.mgrid[:h,:w]; rays=np.stack([(x-w/2)/100,(y-h/2)/100,np.ones_like(x)],axis=-1)
    normal=np.asarray(normal,float);normal/=np.linalg.norm(normal)
    xyz=rays*(offset/(rays@normal))[...,None]
    return xyz.reshape(-1,3),np.column_stack([x.ravel(),y.ravel()]),(w,h),normal

class SurfaceTests(unittest.TestCase):
    def test_plane_normal_and_radius(self):
        xyz,uv,size,n=plane(); a=estimate_surface(xyz,uv,size)
        self.assertTrue(a['valid'].all());np.testing.assert_allclose(a['normal'],np.tile(n,(len(xyz),1)),atol=1e-6)
        center=(uv[:,0]==7)&(uv[:,1]==6)
        np.testing.assert_allclose(a['radius_m'][center],.5*np.sqrt(2)*.03,rtol=1e-6)
    def test_tilted_plane(self):
        xyz,uv,size,n=plane(normal=(.2,.3,-1));a=estimate_surface(xyz,uv,size)
        self.assertTrue(a['valid'].all());np.testing.assert_allclose(a['normal'],np.tile(n,(len(xyz),1)),atol=1e-6)
    def test_depth_step_no_crossing(self):
        xyz,uv,size,n=plane();xyz[uv[:,0]>=7]*=2
        a=estimate_surface(xyz,uv,size)
        np.testing.assert_allclose(a['normal'],np.tile(n,(len(xyz),1)),atol=1e-6)
        boundary=(uv[:,0]==6)|(uv[:,0]==7);self.assertTrue(np.all(a['flags'][boundary]&2))
        self.assertTrue(np.all(a['radius_m'][boundary]<=.030001))
    def test_isolated_empty_and_invalid(self):
        a=estimate_surface([[0,0,3]],[[2,2]],(5,5));self.assertFalse(a['valid'][0]);self.assertEqual(a['radius_m'][0],0)
        a=estimate_surface(np.empty((0,3)),np.empty((0,2)),(5,5));self.assertEqual(len(a['normal']),0)
        with self.assertRaises(ValueError): estimate_surface([[0,0,-1]],[[0,0]],(1,1))
    def test_rotation_radius_and_wire(self):
        xyz,uv,size,n=plane();a=estimate_surface(xyz,uv,size)
        R=np.array([[0,0,1],[0,1,0],[-1,0,0.]])
        b=make_records(a,np.arange(len(xyz)),R)
        np.testing.assert_allclose(b['normal'],np.tile(R@n,(len(xyz),1)),atol=1e-6)
        np.testing.assert_array_equal(b['radius_m'],a['radius_m'])
        packets=list(packetize_surface(b,8,100));self.assertEqual(len(packets),2)
        self.assertEqual(SURFACE_HEADER.unpack_from(packets[-1]),(b'SRF1',1,32,8,1,95,32,1,0))
        self.assertEqual(b''.join(p[32:] for p in packets),b.tobytes())
    def test_cap_and_hole(self):
        xyz,uv,size,n=plane();keep=~((uv[:,0]==7)&(uv[:,1]==6));a=estimate_surface(xyz[keep],uv[keep],size,max_radius_m=.005)
        self.assertTrue(np.all(a['radius_m']<=.00500001));self.assertTrue(np.any(a['flags']&2));self.assertTrue(np.any(a['flags']&4))

if __name__=='__main__':unittest.main()
