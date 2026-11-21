"""Saturated spot evidence, ambiguity and real repaired-focus image regressions."""
import json
import os
from pathlib import Path
import unittest
import cv2
import numpy as np
from robot_vision_test.pixel_preview import evaluate, parameters
from robot_vision_test.hover_detection import detect_hover_target


def geometry():
    rings=[]
    for radius in (80,50,18):
        e=dict(center=np.array([375.,205.]),axes=np.array([radius,radius],float),angle=0.)
        rings.append(dict(inner=e,outer=dict(e,axes=e['axes']+2)))
    return dict(valid=True,targets=[dict(rings=rings)])


class WhiteCoreTests(unittest.TestCase):
    def image(self,points=((370,194),)):
        frame=np.full((360,640,3),160,np.uint8)
        for x,y in points:frame[y:y+2,x:x+2]=253
        return frame

    def result(self,image):
        return evaluate(image,'circle_usb_optical_frame',geometry(),dict(white_core_fallback=True))

    def test_white_spot_without_pink_halo_is_observed_not_certified(self):
        frame=self.image()
        self.assertFalse(evaluate(frame,'circle_usb_optical_frame',geometry())['observed_unique'])
        d=self.result(frame)
        self.assertTrue(d['observed_unique'])
        self.assertEqual(d['observed_basis'],'white_core_local_contrast')
        np.testing.assert_allclose(d['observed_spot_px'],[370.5,194.5])
        self.assertFalse(d['physical_hit_verified'])

    def test_multiple_white_spots_are_ambiguous(self):
        self.assertFalse(self.result(self.image(((369,190),(377,198))))['observed_unique'])

    def test_large_glare_low_contrast_and_outside_calibration_region_rejected(self):
        large=self.image(());large[188:202,365:379]=253
        low=self.image(());low[:]=230;low[194:196,370:372]=240
        for frame in (large,low,self.image(((420,210),)),self.image(())):
            self.assertFalse(self.result(frame)['observed_unique'])

    def test_target_identity_still_required(self):
        self.assertFalse(evaluate(self.image(),'circle_usb_optical_frame',dict(valid=False,targets=[]),dict(white_core_fallback=True))['observed_unique'])

    def test_isolated_bright_pixel_is_not_a_spot(self):
        frame=self.image(());frame[194,370]=255
        self.assertFalse(self.result(frame)['observed_unique'])

    def test_fixed_search_and_photometric_bounds(self):
        for values in (dict(white_core_fallback=1),dict(white_core_search_radius_px=13),dict(white_core_max_area=26),dict(white_core_brightness=100)):
            with self.assertRaises(ValueError):parameters(values)

    def test_actual_on_and_off_frames_after_focus_repair(self):
        folder=Path(os.environ.get('FOCUS_REPLAY_DIR',str(Path(__file__).parent/'fixtures/focus_20261102')))
        manifest=json.loads((Path(__file__).parent/'fixtures/focus_20261102/manifest.json').read_text())
        for item in manifest:
            frame=cv2.imread(str(folder/item['file']))
            self.assertIsNotNone(frame,item['file'])
            d,_=detect_hover_target(frame,dict(diameters_m=[.35,.24,.12],ring_width_m=.008,diameter_definition='outer',dimensions_confirmed=True))
            result=evaluate(frame,'circle_usb_optical_frame',d,dict(white_core_fallback=True))
            self.assertEqual(result['observed_unique'],item['expected_candidate'],item['file'])
            self.assertFalse(result['physical_hit_verified'])
            if item['expected_candidate'] and item['actual_light_on']:
                self.assertLess(np.linalg.norm(np.asarray(result['observed_spot_px'])-[370.4,193.8]),2.,item['file'])


if __name__=='__main__':unittest.main()
