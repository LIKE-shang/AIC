"""Projection optimization verified against installed OpenCV, not a sensor test."""
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import window_live_vision.rgbd as rgbd
import test_rgbd


class ProjectionTests(unittest.TestCase):
    def samples(self):
        rng=np.random.RandomState(53)
        xyz=rng.uniform([-2.,-1.5,.5],[2.,1.5,6.],(6000,3))
        k=np.array([[516.,0.,321.],[0.,507.,239.],[0.,0.,1.]])
        return xyz,k

    def test_brown_and_rational_pixels_match_opencv(self):
        xyz,k=self.samples()
        for d in ([.12,-.04,.001,-.002], [.12,-.04,.001,-.002,.003],
                  [.12,-.04,.001,-.002,.003,.02,-.003,.0004], [0.]*8):
            with self.subTest(coefficients=len(d)):
                d=np.asarray(d)
                expected=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
                actual=rgbd.project_rgb_pixels(xyz,k,d)
                np.testing.assert_allclose(actual,expected,atol=1e-8,rtol=1e-12)
                np.testing.assert_array_equal(np.rint(actual),np.rint(expected))

    def test_complex_camera_models_keep_opencv_fallback(self):
        xyz,k=self.samples()
        for count in (12,14):
            d=np.zeros(count);d[:5]=[.12,-.04,.001,-.002,.003];d[8:12]=[.001,-.001,.002,-.002]
            if count==14:d[-2:]=[.01,-.02]
            expected=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
            with patch.object(rgbd.cv2,'projectPoints',wraps=cv2.projectPoints) as call:
                actual=rgbd.project_rgb_pixels(xyz,k,d)
            call.assert_called_once();np.testing.assert_array_equal(actual,expected)

    def test_registered_zbuffer_and_unknown_mask_match_opencv(self):
        rgb,info=test_rgbd.RegistrationTests().raw()
        rgb=dict(rgb,D=[.12,-.04,.001,-.002,.003])
        rng=np.random.RandomState(3)
        depth=rng.randint(1000,4500,(240,320)).astype(np.uint16)
        depth[::11,::13]=0
        args=(depth,'16UC1',10.,'depth_optical',rgb,info,10.,'rgb_optical',(480,640),np.eye(3),[.04,0.,0.])
        actual=rgbd.DepthRegistrar().register(*args)[0]
        def reference(xyz,k,d):return cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
        with patch.object(rgbd,'project_rgb_pixels',side_effect=reference):
            expected=rgbd.DepthRegistrar().register(*args)[0]
        np.testing.assert_allclose(actual,expected,rtol=0,atol=1e-12,equal_nan=True)
