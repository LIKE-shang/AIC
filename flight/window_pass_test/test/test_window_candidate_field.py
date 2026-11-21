#!/usr/bin/env python3
"""Actual OpenCV regressions for the 2026-10-12 field frame; no ROS/network."""
from pathlib import Path
import copy
import hashlib
import sys
import unittest
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from window_pass import vision_frozen as vision
from window_pass.observation import inner_opening


class FieldWindowCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path=ROOT/'test/window_checker_field_20261012.jpg'
        cls.image=cv2.imread(str(cls.path))
        cls.params=dict(vision.WINDOW_DEFAULTS)

    def test_input_is_exact_uploaded_jpeg(self):
        self.assertEqual(hashlib.sha256(self.path.read_bytes()).hexdigest(),
                         '6ea96c045df42f873545a775a1d67f26016ebabc1ad257c940ba17696af55603')
        self.assertEqual(self.image.shape,(480,640,3))

    def test_old_path_reproduces_no_target_and_new_path_uses_four_borders(self):
        with patch.object(vision,'_texture_rectangle_candidate',return_value=None):
            old,_=vision.detect_windows(self.image,self.params)
        self.assertEqual(old,[])
        found,_=vision.detect_windows(self.image,self.params)
        self.assertEqual(len(found),1)
        target=found[0]
        self.assertEqual(target['candidate_geometry'],'bounded_min_area_rectangle')
        center,corners,kind=inner_opening(target,640,480)
        self.assertEqual(kind,'approximate_inner_opening')
        # Broad visual bounds, not a claim of surveyed millimetre geometry.
        self.assertLess(np.linalg.norm(np.array(center)-[354,240]),10)
        self.assertTrue(all(215 <= x <= 490 and 130 <= y <= 345 for x,y in corners))
        self.assertEqual(len(target['checker_support']['sides']),4)
        for side in target['checker_support']['sides']:
            for key,threshold in (('edge_fraction','checker_min_side_edge_fraction'),
                                  ('std','checker_min_side_std'),('dark_fraction','checker_min_dark_fraction'),
                                  ('light_fraction','checker_min_light_fraction'),
                                  ('alternation','checker_min_alternation'),('repeat','checker_min_repeat')):
                self.assertGreaterEqual(side[key],self.params[threshold])
        self.assertEqual(vision.annotate_windows(self.image,found).shape,self.image.shape)

    def test_previous_successful_image_geometry_is_unchanged(self):
        image=cv2.imread(str(ROOT/'test/window_checker.png'))
        with patch.object(vision,'_texture_rectangle_candidate',return_value=None):
            old,_=vision.detect_windows(image,self.params)
        found,_=vision.detect_windows(image,self.params)
        self.assertEqual(len(found),1)
        np.testing.assert_array_equal(found[0]['contour'],old[0]['contour'])
        self.assertNotIn('candidate_geometry',found[0])

    def test_translation_moves_result_instead_of_using_fixed_pixel_coordinates(self):
        for dx,dy in ((-20,0),(20,0),(0,-15),(0,15)):
            with self.subTest(dx=dx,dy=dy):
                image=cv2.warpAffine(self.image,np.float32([[1,0,dx],[0,1,dy]]),(640,480),borderMode=cv2.BORDER_REPLICATE)
                found,_=vision.detect_windows(image,self.params)
                self.assertEqual(len(found),1)
                center,_,_=inner_opening(found[0],640,480)
                self.assertLess(np.linalg.norm(np.array(center)-[353.5+dx,238+dy]),3)

    def test_jpeg_recompression_preserves_single_opening(self):
        for quality in (70,85,95):
            with self.subTest(quality=quality):
                ok,encoded=cv2.imencode('.jpg',self.image,[cv2.IMWRITE_JPEG_QUALITY,quality])
                self.assertTrue(ok)
                found,_=vision.detect_windows(cv2.imdecode(encoded,cv2.IMREAD_COLOR),self.params)
                self.assertEqual(len(found),1)

    def test_no_four_side_evidence_means_no_acceptance(self):
        with patch.object(vision,'_checkerboard_support',return_value=None):
            self.assertEqual(vision.detect_windows(self.image,self.params)[0],[])

    def test_blank_noise_filled_checker_and_background_crop_have_no_target(self):
        blank=np.full_like(self.image,128)
        random=np.random.default_rng(42).integers(0,256,self.image.shape,dtype=np.uint8)
        yy,xx=np.indices(self.image.shape[:2])
        checker=np.repeat((((xx//5+yy//5)%2)*220+20).astype(np.uint8)[:,:,None],3,axis=2)
        background=cv2.resize(self.image[160:310,240:465],(640,480))
        for label,image in [('blank',blank),('noise',random),('solid_checker',checker),('curtain_and_net',background)]:
            with self.subTest(label=label):
                self.assertEqual(vision.detect_windows(image,self.params)[0],[])

    def test_sparse_original_hole_cannot_be_expanded_to_rectangle(self):
        pts=np.array([[100,100],[400,100],[240,110],[240,390],[230,390],[230,110]],np.int32).reshape(-1,1,2)
        candidate=dict(texture_hole_contour=pts)
        self.assertIsNone(vision._texture_rectangle_candidate(candidate,self.image.shape,self.params))

    def test_disabled_checker_fallback_is_respected(self):
        self.assertEqual(vision.detect_windows(self.image,dict(self.params,checker_fallback=False))[0],[])

    def test_candidate_repair_never_relaxes_configuration(self):
        before=copy.deepcopy(self.params)
        vision.detect_windows(self.image,self.params)
        self.assertEqual(self.params,before)


if __name__=='__main__':
    unittest.main(verbosity=2)
