"""Real off-target light, local reflections, and scoring geometry regressions."""
import copy
import json
from pathlib import Path
import unittest
import cv2
import numpy as np
from robot_vision_test.pixel_preview import evaluate, parameters
from robot_vision_test.hover_detection import detect_hover_target


def geometry():
    rings=[]
    for radius in (40,18):
        e=dict(center=np.array([435.,175.]),axes=np.array([radius,radius],float),angle=0.)
        rings.append(dict(inner=e,outer=dict(e,axes=e['axes']+2)))
    return dict(valid=True,hover_detection_mode='two_ring_measured_inner',
                targets=[dict(rings=rings,aiming_inner=rings[-1]['inner'])])


class LocalSpotTests(unittest.TestCase):
    def image(self):
        frame=np.full((360,640,3),160,np.uint8)
        frame[193:195,369:371]=253
        return frame

    def evaluate(self,frame,local=True):
        return evaluate(frame,'circle_usb_optical_frame',geometry(),
                        dict(white_core_fallback=True,local_spot_search=local))

    def test_real_geometry_outside_middle_ring_can_steer_but_is_not_a_hit(self):
        old=self.evaluate(self.image(),False)
        new=self.evaluate(self.image())
        self.assertFalse(old['observed_unique'])
        self.assertTrue(new['observed_unique'])
        self.assertFalse(new['observed_inside_inner'])
        self.assertGreater(new['observed_normalized_radius'],1.)
        self.assertFalse(new['physical_hit_verified'])
        self.assertEqual(old['inner_semiaxes_px'],new['inner_semiaxes_px'])
        self.assertEqual(old['target_centre_px'],new['target_centre_px'])

    def test_off_reflection_uses_same_region_as_on_light(self):
        row=self.evaluate(self.image())
        self.assertEqual(len(row['observed_candidates']),1)
        self.assertNotEqual(row['observed_candidates'],[])

    def test_off_target_reflection_and_post_ack_temporal_guard(self):
        import sys
        flight=Path(__file__).resolve().parents[4]
        sys.path.insert(0,str(flight/'autonomous_window_test'))
        from aim_evidence import AimEvidence
        for reflection in (False,True):
            guard=AimEvidence()
            off=self.image() if reflection else np.full((360,640,3),160,np.uint8)
            for t in (1.,1.05,1.10):
                guard.observe(dict(self.evaluate(off),capture_stamp=t),'CENTER',None,.35)
            ready=False
            for t in (1.20,1.25,1.30):
                _,ready=guard.observe(dict(self.evaluate(self.image()),capture_stamp=t),'EXPOSE',1.15,.35)
            self.assertEqual(ready,not reflection)
            self.assertEqual(guard.baseline_clear,not reflection)

    def test_independent_local_white_reflection_cannot_hide_behind_pink_spot(self):
        frame=self.image()
        frame[199:202,376:379]=(180,140,190)
        frame[200:202,377:379]=253
        row=self.evaluate(frame)
        self.assertGreaterEqual(len(row['observed_candidates']),2)
        self.assertFalse(row['observed_unique'])

    def test_pink_and_white_descriptions_of_same_core_are_not_two_spots(self):
        frame=self.image()
        frame[191:197,368:374]=(180,140,190)
        frame[193:195,370:372]=253
        row=self.evaluate(frame)
        self.assertEqual(len(row['observed_candidates']),1)

    def test_distant_glare_not_selected_and_search_is_still_bounded(self):
        frame=np.full((360,640,3),160,np.uint8)
        frame[170:172,430:432]=253
        self.assertFalse(self.evaluate(frame)['observed_unique'])
        with self.assertRaises(ValueError):parameters(dict(local_spot_search=1))

    def test_recorded_first_target_and_off_frames(self):
        root=Path(__file__).parent/'fixtures/spot_tracking_20261102'
        detector=dict(diameters_m=[.35,.24,.12],ring_width_m=.008,
                      diameter_definition='outer',dimensions_confirmed=True)
        for item in json.loads((root/'manifest.json').read_text()):
            frame=cv2.imread(str(root/item['file']))
            self.assertIsNotNone(frame,item['file'])
            g,_=detect_hover_target(frame,detector)
            old=evaluate(frame,'circle_usb_optical_frame',g,dict(white_core_fallback=True))
            new=evaluate(frame,'circle_usb_optical_frame',g,dict(white_core_fallback=True,local_spot_search=True))
            if item['expect_new_candidate'] is not None:
                self.assertEqual(new['observed_unique'],item['expect_new_candidate'],item['file'])
            self.assertEqual(new['target_centre_px'],old['target_centre_px'],item['file'])
            self.assertEqual(new.get('inner_semiaxes_px'),old.get('inner_semiaxes_px'),item['file'])
            self.assertFalse(new['physical_hit_verified'])


if __name__=='__main__':unittest.main()
