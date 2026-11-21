"""Competition centre detection; no hardware/ON decision."""
import sys,unittest
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from robot_vision_test.hover_detection import detect_hover_target
from robot_vision_test.pixel_preview import evaluate
from test_air_target import synthetic

class HoverDetectionTests(unittest.TestCase):
    def detect(self,img):return detect_hover_target(img,dict(diameter_definition='outer',dimensions_confirmed=True))[0]
    def test_full_target(self):
        d=self.detect(synthetic(center=(320,180),shape=(360,640),definition='outer'))
        self.assertTrue(d['valid']);self.assertEqual(d['hover_detection_mode'],'three_ring_original')
    def test_measured_middle_and_inner_pair_can_aim_without_outer_ring(self):
        img=synthetic(center=(320,180),shape=(360,640),missing=(0,),definition='outer')
        d=self.detect(img);self.assertTrue(d['valid'],d['reason'])
        self.assertEqual(d['hover_detection_mode'],'two_ring_measured_inner')
        row=evaluate(img,'circle_usb_optical_frame',d)
        self.assertTrue(row['valid']);np.testing.assert_allclose(row['target_centre_px'],[320,180],atol=1)
    def test_single_ring_is_not_arbitrarily_interpreted(self):
        for missing in [(0,1),(0,2),(1,2)]:
            self.assertFalse(self.detect(synthetic(missing=missing))['valid'])
    def test_missing_inner_does_not_invent_measured_inner(self):
        self.assertFalse(self.detect(synthetic(missing=(2,),definition='outer'))['valid'])
    def test_two_complete_targets_remain_ambiguous(self):
        image=np.minimum(synthetic((200,240),(480,880),definition='outer'),synthetic((650,240),(480,880),definition='outer'))
        self.assertFalse(self.detect(image)['valid'])
    def test_unrelated_extra_ring_does_not_hide_complete_assigned_target(self):
        image=synthetic((200,240),(480,880),definition='outer')
        cv2.circle(image,(650,240),60,(20,20,20),6)
        d=self.detect(image);self.assertTrue(d['valid'],d['reason'])
        self.assertEqual(d['hover_detection_mode'],'three_ring_relaxed')
    def test_saved_camera_burst_center_and_spot(self):
        folder=ROOT.parent/'_diagnostics/usb_raw_rbANV8'
        paths=list(folder.glob('capture_*.png'))
        if not paths:self.skipTest('no saved burst')
        for path in paths:
            img=cv2.imread(str(path));d=self.detect(img);row=evaluate(img,'circle_usb_optical_frame',d)
            self.assertTrue(row['valid'],path.name)
            self.assertIsNotNone(row['observed_error_px'])
            self.assertLess(np.linalg.norm(row['observed_minus_reference_px']),.5)

if __name__=='__main__':unittest.main()
