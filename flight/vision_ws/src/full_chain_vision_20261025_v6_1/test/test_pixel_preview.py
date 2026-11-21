"""Pixel-only preview tests; synthetic frames + saved raw burst, no hardware."""
import ast
import copy
import json
from pathlib import Path
import sys
import time
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from robot_vision_test.pixel_preview import parameters, reference_point, evaluate, draw, unavailable
from robot_vision_test.air_target import detect_air_target, air_parameters


def scene(points=((373,195),)):
    image=np.full((360,640,3),160,np.uint8)
    for x,y in points:
        image[y-2:y+3,x-2:x+3]=(180,140,190)
        image[y,x]=(255,255,255)
    return image


def geometry():
    rings=[]
    for radius in (90,60,20):
        inner=dict(center=np.array([320.,180.]),axes=np.array([radius,radius],float),angle=0.)
        rings.append(dict(inner=inner,outer=dict(inner,axes=inner['axes']+2)))
    return dict(valid=True,targets=[dict(rings=rings)])


class GeometryTests(unittest.TestCase):
    def test_source_measurement_reference(self):
        self.assertEqual(reference_point(parameters()),[372.75,194.984375])

    def test_reference_and_observed_errors_are_separate(self):
        row=evaluate(scene(),'circle_usb_optical_frame',geometry())
        self.assertTrue(row['valid'])
        np.testing.assert_allclose(row['reference_error_px'],[52.75,14.984375])
        self.assertFalse(row['reference_inside_inner'])
        self.assertEqual(row['observed_spot_px'],[373.,195.])
        np.testing.assert_allclose(row['observed_minus_reference_px'],[.25,.015625])
        self.assertFalse(row['physical_hit_verified'])

    def test_adjustable_reference_and_tolerance(self):
        row=evaluate(scene(),'circle_usb_optical_frame',geometry(),dict(offset_x_px=10.5,offset_y_px=.5))
        self.assertTrue(row['reference_inside_inner'])
        row=evaluate(scene(),'circle_usb_optical_frame',geometry(),dict(offset_x_px=10.5,offset_y_px=.5,inner_radius_ratio=.4))
        self.assertFalse(row['reference_inside_inner'])

    def test_boundary_is_not_inside(self):
        row=evaluate(scene(),'circle_usb_optical_frame',geometry(),dict(offset_x_px=20.5,offset_y_px=.5))
        self.assertFalse(row['reference_inside_inner'])
        self.assertAlmostEqual(row['reference_normalized_radius'],1.)

    def test_no_target_clears_error_and_inside(self):
        row=evaluate(scene(),'circle_usb_optical_frame',dict(valid=False,targets=[]))
        self.assertFalse(row['valid'])
        self.assertIsNone(row['reference_inside_inner'])
        self.assertIsNone(row['observed_spot_px'])

    def test_identity_and_size_mismatch_never_scale_silently(self):
        for frame,frame_id in [(scene(),'other'),(np.zeros((480,640,3),np.uint8),'circle_usb_optical_frame')]:
            row=evaluate(frame,frame_id,geometry())
            self.assertFalse(row['valid'])
            self.assertIsNone(row['reference_point_px'])

    def test_missing_spot_is_unknown_not_zero_error(self):
        row=evaluate(scene(()),'circle_usb_optical_frame',geometry())
        self.assertTrue(row['valid'])
        self.assertIsNone(row['observed_error_px'])
        self.assertIsNone(row['observed_inside_inner'])

    def test_multiple_spots_not_arbitrarily_selected(self):
        row=evaluate(scene(((373,195),(330,200))),'circle_usb_optical_frame',geometry())
        self.assertEqual(len(row['observed_candidates']),2)
        self.assertIsNone(row['observed_spot_px'])

    def test_observed_radius_uses_rotated_inner_ellipse_and_candidate_is_unique(self):
        g=geometry();g['targets'][0]['rings'][2]['inner'].update(
            axes=np.array([40.,20.]),angle=90.)
        row=evaluate(scene(((320,190),)),'circle_usb_optical_frame',g)
        self.assertTrue(row['observed_unique'])
        self.assertAlmostEqual(row['observed_normalized_radius'],.25)
        self.assertTrue(row['observed_inside_inner'])

    def test_missing_and_ambiguous_spots_have_no_control_radius(self):
        for points in ((),((373,195),(330,200))):
            row=evaluate(scene(points),'circle_usb_optical_frame',geometry())
            self.assertFalse(row['observed_unique'])
            self.assertIsNone(row['observed_normalized_radius'])

    def test_invalid_parameters(self):
        for patch in [dict(offset_x_px=float('nan')),dict(offset_y_px=1000),dict(image_width=True),
                      dict(inner_radius_ratio=0),dict(inner_radius_ratio=1.1),dict(enabled='true'),
                      dict(halo_area_min=200),dict(unknown=2),dict(camera_frame_id='')]:
            with self.subTest(patch=patch),self.assertRaises(ValueError): parameters(patch)

    def test_disabled(self):
        self.assertEqual(evaluate(scene(),'circle_usb_optical_frame',geometry(),dict(enabled=False))['reason'],'disabled')

    def test_drawing_preserves_dimensions(self):
        image=scene(); before=image.copy()
        out=draw(image,geometry(),evaluate(image,'circle_usb_optical_frame',geometry()))
        self.assertEqual(out.shape,before.shape)
        self.assertFalse(np.array_equal(out,before))

    def test_saved_burst_and_runtime_profile(self):
        folder=ROOT.parent/'_diagnostics/usb_raw_rbANV8'
        if not (folder/'manifest.json').exists(): self.skipTest('saved burst not installed on this host')
        cfg=yaml.safe_load((ROOT/'config/air_hover_pixel_debug.yaml').read_text(encoding='utf-8-sig'))
        for record in json.loads((folder/'manifest.json').read_text()):
            image=cv2.imread(str(folder/record['file']))
            detection,_=detect_air_target(image,cfg['detector'])
            row=evaluate(image,record['camera_frame_id'],detection,cfg['pixel_preview'])
            self.assertTrue(row['valid'],record['file'])
            self.assertEqual(len(row['observed_candidates']),1,record['file'])
            self.assertLess(np.linalg.norm(row['observed_minus_reference_px']),.5)
            self.assertFalse(row['reference_inside_inner'])
            self.assertFalse(row['physical_hit_verified'])

    def test_launch_and_node_have_no_actuator_interface(self):
        root=ET.parse(ROOT/'launch/air_hover_pixel_debug.launch').getroot()
        args={x.attrib['name']:x.attrib.get('default') for x in root.findall('arg')}
        self.assertEqual(args['start_camera'],'false')
        self.assertEqual([n.attrib['type'] for n in root.findall('node')],['$(arg preview_executable)'])
        self.assertEqual(args['preview_executable'],'circle_pixel_preview_node.py')
        text=(ROOT/'scripts/circle_pixel_preview_node.py').read_text(encoding='utf-8-sig')
        ast.parse(text,feature_version=(3,8))
        for token in ('ServiceProxy','LaserDecision','mavros','CommandLong','ctypes','subprocess'):
            self.assertNotIn(token,text)


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_node_callbacks as fake
        cls.fake=fake
        cls.module=fake.load('circle_pixel_preview_node')

    def setUp(self):
        self.fake.params.clear()
        self.fake.params.update(pixel_preview=dict(offset_x_px=53.25,offset_y_px=15.484375),
                                detector=dict(diameter_definition='outer',dimensions_confirmed=True))
        self.node=self.module.PixelPreview()

    def tearDown(self):
        self.node.close();self.node.watchdog.join(1)

    def image(self):
        from test_air_target import synthetic
        image=synthetic(center=(320,180),shape=(360,640),definition='outer')
        message=self.fake.Image(image,stamp=time.time())
        message.header.frame_id='circle_usb_optical_frame'
        return message

    def status(self):return json.loads(self.node.status_pub.messages[-1].data)

    def test_valid_callback_and_headers(self):
        msg=self.image();self.node.callback(msg)
        self.assertTrue(self.status()['valid'])
        self.assertEqual(self.node.image_pub.messages[-1].header.stamp.value,msg.header.stamp.value)
        self.assertFalse(self.status()['hardware_commands_sent'])
        self.assertIsNone(self.status()['observed_spot_px'])

    def test_stale_and_duplicate_frame_clear_overlay_and_status(self):
        msg=self.image();self.node.callback(msg);self.node.callback(msg)
        self.assertFalse(self.status()['valid'])
        self.assertIsNone(self.status()['reference_point_px'])
        self.assertIsNone(self.status()['reference_inside_inner'])

    def test_wrong_frame_id_clears_preview(self):
        msg=self.image();msg.header.frame_id='different';self.node.callback(msg)
        self.assertFalse(self.status()['valid'])
        self.assertIn('mismatch',self.status()['reason'])

    def test_invalid_conversion(self):
        self.node.callback(self.fake.Image(None,stamp=time.time()))
        self.assertFalse(self.status()['valid'])
        self.assertIsNone(self.status()['reference_error_px'])

    def test_watchdog_clears_late_frame(self):
        self.node.callback(self.image())
        with self.node.lock: self.node.last_wall=time.monotonic()-1.
        until=time.monotonic()+1.
        while time.monotonic()<until and self.status()['valid']: time.sleep(.01)
        self.assertEqual(self.status()['reason'],'image_stream_stale')
        self.assertIsNone(self.status()['reference_point_px'])


if __name__=='__main__': unittest.main(verbosity=2)
