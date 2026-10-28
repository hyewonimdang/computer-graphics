import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from kitti_io import transform,apply_pose,oxts_pose,projection_camera,KittiOdometry,validate_pose
from kitti_pipeline import scaled_camera,lidar_projection
from point_stream import VoxelMap,RECORD,packetize,unpack_packet


class KittiPipelineTests(unittest.TestCase):
    def test_mercator_translation_and_euler_order(self):
        p=np.zeros(30); p[1]=np.rad2deg(1/6378137.); p[5]=np.pi/2
        T=oxts_pose(p,1.)
        np.testing.assert_allclose(T[:3,3],[1,0,0],atol=1e-8)
        np.testing.assert_allclose(apply_pose(T,[[1,0,0]]),[[1,1,0]],atol=1e-8)
        with self.assertRaises(ValueError): oxts_pose([90,0,0,0,0,0],1.)

    def test_full_projection_offset_not_only_x(self):
        K=np.array([[100,0,20],[0,100,10],[0,0,1.]])
        t=np.array([.2,.01,.003]); P=K@np.column_stack((np.eye(3),t))
        k,T=projection_camera(P)
        np.testing.assert_allclose(T[:3,3],t)
        X=np.array([[1,2,7],[-2,3,8.]])
        np.testing.assert_allclose(apply_pose(T,X)@k.T,np.column_stack((X,np.ones(2)))@P.T)

    def test_odometry_camera0_to_camera2_lever_arm(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); seq=root/'sequences/00'; seq.mkdir(parents=True); (root/'poses').mkdir()
            K=np.array([[100.,0,20],[0,100,10],[0,0,1]])
            P2=K@np.column_stack((np.eye(3),[.2,0,0])); P3=K@np.column_stack((np.eye(3),[-.4,0,0]))
            (seq/'calib.txt').write_text('P2: '+' '.join(map(str,P2.flat))+'\nP3: '+' '.join(map(str,P3.flat)))
            T0=np.eye(4); T1=transform([[0,-1,0],[1,0,0],[0,0,1]],[1,0,0])
            np.savetxt(root/'poses/00.txt',np.stack([T0[:3].ravel(),T1[:3].ravel()]))
            (seq/'times.txt').write_text('0\n0.1\n')
            ds=KittiOdometry(root)
            np.testing.assert_allclose(ds.pose(0),np.eye(4),atol=1e-12)
            np.testing.assert_allclose(ds.pose(1)[:3,3],[1.2,-.2,0],atol=1e-12)
            np.testing.assert_allclose(ds.T_right_left[:3,3],[-.6,0,0])

    def test_resize_camera_pixel_centres(self):
        K=np.array([[200.,0,60],[0,180,30],[0,0,1]])
        resized=scaled_camera(K,(120,80),(60,40))
        uv=K@np.array([.2,.1,1]); new=resized@np.array([.2,.1,1])
        np.testing.assert_allclose(new[:2],(uv[:2]+.5)*.5-.5)
        with self.assertRaises(ValueError): validate_pose(np.diag([1,1,-1,1]))

    def test_lidar_projection_zbuffer_and_behind_camera(self):
        scan=np.array([[0,0,2,1],[0,0,4,1],[0,0,-1,1],[100,0,1,1]])
        K=np.array([[10.,0,5],[0,10,5],[0,0,1]])
        uv,z=lidar_projection(scan,np.eye(4),K,(10,10))
        np.testing.assert_array_equal(uv,[[5,5]]); np.testing.assert_array_equal(z,[2])

    def test_voxel_preserves_representative_attributes_and_deduplicates(self):
        m=VoxelMap(.5)
        xyz=np.array([[.1,0,2],[.2,0,2],[-.1,0,2]])
        rgb=np.array([[10,20,30],[40,50,60],[70,80,90]])
        uv=np.array([[1,2],[3,4],[5,6]])
        out,selected,stats=m.add(xyz,rgb,uv,[4,5,6],[.6,.9,.8])
        np.testing.assert_array_equal(selected,[1,2]); np.testing.assert_array_equal(out['rgba'][:,:3],rgb[[1,2]])
        np.testing.assert_array_equal(out['uv'],uv[[1,2]]); np.testing.assert_array_equal(out['disparity'],[5,6])
        np.testing.assert_array_equal(out['point_id'],[0,1])
        self.assertEqual(stats['new_points'],2)
        again,_,_=m.add(xyz,rgb,uv,[4,5,6],[.99,.99,.99]); self.assertEqual(len(again),0)
        self.assertEqual(m.count,2)
        new,_,_=m.add([[1,0,2]],[[1,2,3]],[[0,0]],[2],[.9]); self.assertEqual(new['point_id'][0],2)

    def test_voxel_capacity_is_atomic_and_rejects_bad_attributes(self):
        m=VoxelMap(.1,max_points=1)
        with self.assertRaises(MemoryError): m.add([[0,0,1],[1,0,1]],[[0,0,0]]*2,[[1,1]]*2,[1,1],[.9,.9])
        self.assertEqual(m.count,0); self.assertEqual(len(m.cells),0)
        with self.assertRaises(ValueError): m.add([[np.nan,0,1]],[[0,0,0]],[[1,1]],[1],[.9])

    def test_binary_packet_layout_chunks_empty_and_corruption(self):
        points=np.zeros(9,dtype=RECORD); points['point_id']=np.arange(9); points['xyz']=1; points['rgba'][:,3]=255
        packets=list(packetize(points,20,4)); self.assertEqual(len(packets),3)
        rebuilt=[]
        for i,p in enumerate(packets):
            frame,chunk,last,subset=unpack_packet(p)
            self.assertEqual(frame,20); self.assertEqual(chunk,i); self.assertEqual(last,i==2); rebuilt.append(subset)
        self.assertEqual(np.concatenate(rebuilt).tobytes(),points.tobytes())
        empty=list(packetize(points[:0],21)); self.assertEqual(len(empty),1)
        self.assertEqual(len(empty[0]),32); self.assertTrue(unpack_packet(empty[0])[2])
        with self.assertRaises(ValueError): unpack_packet(packets[0][:-1])
        with self.assertRaises(ValueError): unpack_packet(b'FAIL'+packets[0][4:])


if __name__=='__main__': unittest.main()
