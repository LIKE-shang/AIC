#!/usr/bin/env python3
import copy
from pathlib import Path
import sys
import types
import unittest
import cv2
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from window_pass.vision_frozen import detect_windows,WINDOW_DEFAULTS,checked_parameters
from window_pass.observation import inner_opening,camera_rays,undistorted_rays,digest
from window_pass.geometry import opening_polygon


def info():
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='optical',stamp=types.SimpleNamespace(to_sec=lambda:10.)),
        width=640,height=480,distortion_model='plumb_bob',K=[400.,0.,320.,0.,400.,240.,0.,0.,1.],D=[0.]*5,
        binning_x=0,binning_y=0,roi=types.SimpleNamespace(x_offset=0,y_offset=0,width=0,height=0))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.params=yaml.safe_load((ROOT/'config/vision_frozen.yaml').read_text(encoding='utf-8'))
    def test_real_rgb_checker_replay_and_geometry(self):
        image=cv2.imread(str(ROOT/'test/window_checker.png'))
        found,_=detect_windows(image,self.params)
        self.assertEqual(len(found),1)
        c,pts,kind=inner_opening(found[0],640,480)
        self.assertEqual(kind,'approximate_inner_opening');self.assertEqual(len(pts),4)
        self.assertTrue(0<c[0]<640 and 0<c[1]<480)
    def test_nested_reference_is_inner_not_outer(self):
        image=np.full((480,640,3),235,np.uint8)
        cv2.rectangle(image,(120,70),(520,390),(20,20,20),-1)
        cv2.rectangle(image,(155,115),(480,355),(235,235,235),-1)
        found,_=detect_windows(image,self.params)
        self.assertEqual(len(found),1)
        center,pts,kind=inner_opening(found[0],640,480)
        self.assertEqual(kind,'contour_inner_opening')
        self.assertLess(max(x for x,y in pts),500)
        self.assertGreater(min(x for x,y in pts),140)
    def test_entire_checker_parameter_profile_present(self):
        for k,v in WINDOW_DEFAULTS.items():
            self.assertIn(k,self.params)
            self.assertEqual(list(v) if isinstance(v,tuple) else v,self.params[k])
        self.assertEqual(digest(checked_parameters(WINDOW_DEFAULTS,self.params)),digest(WINDOW_DEFAULTS))
    def test_camera_rays_valid_and_finite(self):
        k,d,h=camera_rays(info(),'optical',640,480,10.)
        rays=undistorted_rays((320,240),((120,90),(520,90),(520,390),(120,390)),k,d)
        np.testing.assert_allclose(rays[0],(0,0,1))
        np.testing.assert_allclose(rays[1],(-.5,-.375,1))
        self.assertEqual(len(h),64)
    def test_invalid_camera_info_and_timing(self):
        cases=[('width',320),('distortion_model','equidistant'),('K',[0.]*9),('D',[float('nan')]*5),('binning_x',2)]
        for key,value in cases:
            m=info();setattr(m,key,value)
            with self.subTest(key=key),self.assertRaises(ValueError):camera_rays(m,'optical',640,480,10.)
        for stamp in (0.,8.,12.):
            with self.assertRaises(ValueError):camera_rays(info(),'optical',640,480,stamp)
        with self.assertRaises(ValueError):camera_rays(info(),'wrong_frame',640,480,10.)
        with self.assertRaises(ValueError):camera_rays(None,'optical',640,480,10.)
    def test_calibration_fingerprint_changes_with_intrinsics(self):
        a=info();b=copy.deepcopy(a);b.K[0]+=1
        self.assertNotEqual(camera_rays(a,'optical',640,480,10.)[2],camera_rays(b,'optical',640,480,10.)[2])
    def test_bow_tie_and_outer_only_rejected(self):
        with self.assertRaises(ValueError):opening_polygon(((100,100),(500,380),(100,380),(500,100)),640,480)
        with self.assertRaises(ValueError):inner_opening(dict(method='quad_only'),640,480)
    def test_perspective_center_is_diagonal_intersection(self):
        c,_=opening_polygon(((100,100),(500,120),(430,380),(150,300)),640,480)
        self.assertNotEqual(c,tuple(np.mean(np.array(((100,100),(500,120),(430,380),(150,300))),axis=0)))
    def test_no_target_and_two_targets(self):
        blank=np.full((480,640,3),235,np.uint8)
        self.assertEqual(detect_windows(blank,self.params)[0],[])
        image=cv2.imread(str(ROOT/'test/window_checker.png'))
        found,_=detect_windows(np.concatenate((image,image),axis=1),self.params)
        self.assertEqual(len(found),2)


if __name__=='__main__':
    print('OpenCV',cv2.__version__,'NumPy',np.__version__)
    unittest.main(verbosity=2)
