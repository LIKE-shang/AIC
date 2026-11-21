"""Early suppression must preserve final detections and all selected checks."""
import unittest
from unittest.mock import patch
import cv2,numpy as np
from pathlib import Path
from scenes import scene
from window_live_vision.detector import Detector

class EarlySuppression(unittest.TestCase):
    def compare(self,image,width):
        with patch.object(Detector,'_already_accepted',return_value=False):
            expected,old=Detector(width).detect(image)
        actual,new=Detector(width).detect(image)
        self.assertEqual(new['proposal_count'],old['proposal_count'])
        self.assertEqual(len(actual),len(expected))
        for a,b in zip(actual,expected):
            np.testing.assert_array_equal(a.corners,b.corners)
            self.assertEqual(a.score,b.score);self.assertEqual(a.method,b.method)
            self.assertEqual(a.sides,b.sides)
    def test_synthetic_and_field_equivalence(self):
        images=[]
        for w,h,solid in [(170,120,False),(300,220,False),(440,330,False),(260,200,True)]:images.append(scene(w,h,solid=solid)[0])
        image,q=scene();images.append(cv2.warpPerspective(image,cv2.getPerspectiveTransform(q,np.float32([[180,135],[475,165],[450,360],[165,350]])),(640,480),borderValue=(155,)*3))
        root=Path(__file__).resolve().parents[2]
        for name in ('window_checker_field_20261012.jpg','window_checker.png'):images.append(cv2.imread(str(root/'window_pass_test/test'/name)))
        for width in (480,640):
            for i,image in enumerate(images):
                with self.subTest(width=width,case=i):self.compare(image,width)
    def test_missing_borders_and_two_targets_equivalence(self):
        image,_=scene();image[98:145,120:510]=155
        self.compare(image,480)
        first,_=scene(160,120,center=(155,240),border=20)
        second,_=scene(160,120,center=(480,240),border=20)
        first[:,320:]=second[:,320:]
        self.compare(first,480)
