#!/usr/bin/env python3
"""Offline regression tests (no ROS, no camera, no hardware service calls)."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from robot_vision_test.vision import detect_windows, detect_circles, depth_to_gray, checked_parameters, CIRCLE_DEFAULTS, WINDOW_DEFAULTS
from robot_vision_test.safety import ConfirmationGate, FrameFreshness, safe_test_topic


def blank():
    return np.full((480, 640, 3), 235, np.uint8)


def window_image():
    frame = blank()
    cv2.rectangle(frame, (130, 100), (510, 380), (25, 25, 25), -1)
    cv2.rectangle(frame, (152, 122), (488, 358), (235, 235, 235), -1)
    return frame


def ring_image():
    frame = blank()
    cv2.circle(frame, (320, 240), 90, (15, 15, 15), 14)
    return frame


class ShapeTests(unittest.TestCase):
    def test_blank(self):
        self.assertEqual(len(detect_windows(blank())[0]), 0)
        self.assertEqual(len(detect_circles(blank())[0]), 0)

    def test_hollow_window_deduplicated(self):
        self.assertEqual(len(detect_windows(window_image())[0]), 1)

    def test_solid_rectangle_not_window(self):
        frame = blank()
        cv2.rectangle(frame, (120, 100), (520, 380), (20, 20, 20), -1)
        self.assertEqual(len(detect_windows(frame)[0]), 0)

    def test_perspective_window(self):
        frame = blank()
        cv2.fillConvexPoly(frame, np.array([[140,110],[500,85],[480,380],[155,355]]), (20,20,20))
        cv2.fillConvexPoly(frame, np.array([[166,131],[471,111],[454,350],[178,332]]), (235,235,235))
        self.assertEqual(len(detect_windows(frame)[0]), 1)

    def test_diamond_not_window(self):
        frame = blank()
        cv2.fillConvexPoly(frame, np.array([[320,70],[520,240],[320,310],[120,240]]), (20,20,20))
        cv2.fillConvexPoly(frame, np.array([[320,108],[480,238],[320,279],[160,238]]), (235,235,235))
        self.assertEqual(len(detect_windows(frame)[0]), 0)

    def test_ring_deduplicated(self):
        self.assertEqual(len(detect_circles(ring_image())[0]), 1)
        self.assertEqual(len(detect_circles(ring_image(), {'require_ring': True})[0]), 1)

    def test_disk_and_ring_requirement(self):
        frame=blank(); cv2.circle(frame, (320,240), 80, (15,15,15), -1)
        self.assertEqual(len(detect_circles(frame)[0]), 1)
        self.assertEqual(len(detect_circles(frame, {'require_ring': True})[0]), 0)

    def test_square_and_ellipse_rejected(self):
        frame=blank(); cv2.rectangle(frame, (220,140),(420,340),(15,15,15),-1)
        self.assertEqual(len(detect_circles(frame)[0]),0)
        frame=blank(); cv2.ellipse(frame,(320,240),(100,50),0,0,360,(15,15,15),-1)
        self.assertEqual(len(detect_circles(frame)[0]),0)

    def test_small_reflections_noise_rejected(self):
        frame=blank()
        rng=np.random.default_rng(71)
        for x,y in rng.integers([10,10],[630,470],size=(35,2)):
            cv2.circle(frame,(int(x),int(y)),3,(0,0,0),-1)
        self.assertEqual(len(detect_circles(frame)[0]),0)
        self.assertEqual(len(detect_windows(frame)[0]),0)

    def test_two_separate_circles(self):
        frame=blank()
        for x in (170,470): cv2.circle(frame,(x,240),60,(20,20,20),9)
        self.assertEqual(len(detect_circles(frame)[0]),2)

    def test_hough_confirmation(self):
        self.assertEqual(len(detect_circles(ring_image(), {'use_hough': True})[0]), 1)

    def test_depth_encodings(self):
        mm=np.array([[0,200,1000,5000]],np.uint16)
        metres=np.array([[np.nan,0.2,1,5]],np.float32)
        np.testing.assert_array_equal(depth_to_gray(mm,'16UC1'),depth_to_gray(metres,'32FC1'))
        self.assertEqual(depth_to_gray(mm,'16UC1').dtype,np.uint8)
        self.assertEqual(depth_to_gray(np.array([[np.nan,np.inf,-1,0]],np.float32),'32FC1').sum(),0)
        with self.assertRaises(ValueError): depth_to_gray(mm,'bgr8')


class CheckerWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = cv2.imread(str(PACKAGE/'test/artifacts/window_checker.png'))
        if cls.original is None:
            raise RuntimeError('Missing lossless user-supplied window_checker.png fixture')
        cls.config = yaml.safe_load((PACKAGE/'config/window_test.yaml').read_text(encoding='utf-8'))

    def test_original_capture_and_opening_geometry(self):
        targets, _ = detect_windows(self.original, self.config)
        self.assertEqual(len(targets), 1)
        target = targets[0]
        self.assertTrue(target['checker_fallback'])
        self.assertEqual(target['outline_kind'], 'opening')
        self.assertEqual(len(target['checker_support']['sides']), 4)
        self.assertTrue(300 < target['center'][0] < 330)
        self.assertTrue(275 < target['center'][1] < 300)
        self.assertTrue(260 < target['width'] < 335)
        self.assertTrue(210 < target['height'] < 275)

    def test_old_nested_only_path_reproduces_miss(self):
        self.assertEqual(len(detect_windows(self.original, {'checker_fallback': False})[0]), 0)

    def test_rotation_and_scale(self):
        for angle, scale in [(-8, .85), (8, .85), (0, .65), (0, .8)]:
            with self.subTest(angle=angle, scale=scale):
                matrix = cv2.getRotationMatrix2D((320, 240), angle, scale)
                frame = cv2.warpAffine(self.original, matrix, (640, 480), borderValue=(180,)*3)
                self.assertEqual(len(detect_windows(frame, self.config)[0]), 1)

    def test_brightness(self):
        for gain in [.8, 1.15]:
            with self.subTest(gain=gain):
                frame = np.clip(self.original.astype(np.float32)*gain, 0, 255).astype(np.uint8)
                self.assertEqual(len(detect_windows(frame, self.config)[0]), 1)

    def test_each_missing_side_rejected(self):
        for roi in [(45,125,575,184), (145,383,580,440), (42,130,180,440), (435,125,577,440)]:
            with self.subTest(roi=roi):
                x1, y1, x2, y2 = roi
                frame = self.original.copy()
                frame[y1:y2, x1:x2] = 128
                self.assertEqual(len(detect_windows(frame, self.config)[0]), 0)

    def test_whole_checker_and_solid_board_rejected(self):
        yy, xx = np.indices((480, 640))
        checker = np.where(((xx//7+yy//7) % 2) == 0, 20, 220).astype(np.uint8)
        board = np.full((480, 640), 220, np.uint8)
        board[100:380, 120:520] = checker[100:380, 120:520]
        for gray in [checker, board]:
            self.assertEqual(len(detect_windows(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), self.config)[0]), 0)

    def test_nonperiodic_texture_opening_rejected(self):
        rng = np.random.default_rng(193)
        noise = cv2.resize(rng.integers(0, 2, (60, 80), dtype=np.uint8)*200+20,
                           (640, 480), interpolation=cv2.INTER_NEAREST)
        noise[165:405, 163:460] = 140
        self.assertEqual(len(detect_windows(cv2.cvtColor(noise, cv2.COLOR_GRAY2BGR), self.config)[0]), 0)

    def test_nested_and_checker_both_detected(self):
        frame = np.concatenate([self.original, window_image()], axis=1)
        targets, _ = detect_windows(frame, self.config)
        self.assertEqual(len(targets), 2)
        self.assertEqual({t['outline_kind'] for t in targets}, {'opening', 'outer_frame'})

    def test_checker_validation(self):
        invalid = {'checker_fallback': 'true', 'checker_outer_scale': 1.0,
                   'checker_min_area_fraction': 1.0, 'checker_min_side_edge_fraction': 0,
                   'checker_min_cell_px': 1, 'checker_max_cell_px': 65,
                   'checker_min_repeat': 0, 'checker_texture_kernels': [4],
                   'checker_texture_thresholds': [float('nan')], 'checker_morph_open': 0}
        for name, value in invalid.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                checked_parameters(WINDOW_DEFAULTS, {name: value})

    def test_explicit_checker_yaml_defaults(self):
        for key, default in WINDOW_DEFAULTS.items():
            if key.startswith('checker_'):
                actual = self.config[key]
                self.assertEqual(tuple(actual) if isinstance(default, tuple) else actual, default)
        self.assertTrue(self.config['require_inner_frame'])


class SafetyTests(unittest.TestCase):
    def test_invalid_parameters_fail_closed(self):
        for default,p in [(CIRCLE_DEFAULTS,{'min_radius':-1}),
                          (CIRCLE_DEFAULTS,{'max_ring_radius_ratio':1}),
                          (WINDOW_DEFAULTS,{'blur_kernel':2}),
                          (WINDOW_DEFAULTS,{'max_area_fraction':2}),
                          (WINDOW_DEFAULTS,{'min_frame_gap':0})]:
            with self.assertRaises(ValueError): checked_parameters(default,p)
        with self.assertRaises(ValueError): FrameFreshness(float('nan'))

    def test_consecutive_n_m(self):
        gate=ConfirmationGate(3,2,0.6)
        self.assertFalse(gate.observe(True,1.0))
        self.assertFalse(gate.observe(True,1.1))
        self.assertTrue(gate.observe(True,1.2))
        self.assertTrue(gate.observe(False,1.3))
        self.assertFalse(gate.observe(False,1.4))

    def test_confirmation_must_be_consecutive(self):
        gate=ConfirmationGate(3,2,0.6)
        for present,t in [(True,1.0),(True,1.1),(False,1.2),(True,1.3),(True,1.4)]:
            self.assertFalse(gate.observe(present,t))
        self.assertTrue(gate.observe(True,1.5))

    def test_wall_timeout_without_callback(self):
        gate=ConfirmationGate(1,3,0.6); gate.observe(True,10)
        self.assertFalse(gate.watchdog(10.5)); self.assertTrue(gate.on)
        self.assertTrue(gate.watchdog(10.61)); self.assertFalse(gate.on)

    def test_gap_resets_confirmation(self):
        gate=ConfirmationGate(3,2,0.6)
        gate.observe(True,1); gate.observe(True,1.1)
        self.assertFalse(gate.observe(True,2)); self.assertEqual(gate.hits,1)

    def test_invalid_frame_immediately_off(self):
        gate=ConfirmationGate(1,3,0.6); gate.observe(True,10)
        gate.invalidate('conversion_error'); self.assertFalse(gate.on); self.assertEqual(gate.hits,0)

    def test_replay_stale_zero_stamp(self):
        fresh=FrameFreshness(0.6)
        fresh.check(10,10.1)
        for stamp,now in [(10,10.2),(11,12),(0,12),(15,14)]:
            with self.assertRaises(ValueError): fresh.check(stamp,now)

    def test_forbidden_output_topics(self):
        for topic in ['/object_position','/cmd_vel','/ctrl_mode','/move_base/goal','/mavros/setpoint_raw/local']:
            with self.assertRaises(ValueError): safe_test_topic(topic)
        self.assertEqual(safe_test_topic('/window_test/image'),'/window_test/image')


class StaticChecks(unittest.TestCase):
    def test_xml_yaml_python(self):
        for path in (PACKAGE/'launch').glob('*test*.launch'): ET.parse(path)
        ET.parse(PACKAGE/'launch/shape_detector.launch'); ET.parse(PACKAGE/'package.xml')
        for name in ['window_test.yaml','circle_laser_test.yaml','laser_hardware.yaml',
                     'window_competition_vision.yaml','window_competition_adapter.yaml']:
            self.assertIsInstance(yaml.safe_load((PACKAGE/'config'/name).read_text(encoding='utf-8-sig')),dict)
        for name in ['window_test_node.py','circle_laser_test_node.py','depth_preview_test_node.py']:
            source=(PACKAGE/'scripts'/name).read_text(encoding='utf-8-sig')
            compile(source,name,'exec')
            self.assertNotIn('object_position',source)
            self.assertNotIn('cmd_vel',source)
            self.assertNotIn('setpoint_raw',source)

    def test_safe_defaults(self):
        hardware=yaml.safe_load((PACKAGE/'config/laser_hardware.yaml').read_text(encoding='utf-8-sig'))
        self.assertFalse(hardware['hardware_wiring_confirmed']); self.assertEqual(hardware['laser_channel'],-1)
        args={x.attrib['name']:x.attrib.get('default') for x in ET.parse(PACKAGE/'launch/circle_laser_test.launch').getroot().findall('arg')}
        self.assertEqual(args['enable_laser'],'false')

    def test_stamped_interface_matches_bridge(self):
        bridge=(PACKAGE/'src/laser_test_bridge.cpp').read_text(encoding='utf-8')
        detector=(PACKAGE/'scripts/circle_laser_test_node.py').read_text(encoding='utf-8')
        self.assertIn('robot_vision::LaserDecision::ConstPtr',bridge)
        self.assertIn('msg->enable_laser',bridge)
        self.assertIn('msg.enable_laser = self.enable_laser',detector)
        for token in ['/circle_laser_test/decision','/circle_laser_test/laser_status']:
            self.assertIn(token,bridge); self.assertIn(token,detector)
        self.assertNotIn('std_msgs::Bool',bridge)

    def test_launch_parameter_names_match_detector(self):
        root=ET.parse(PACKAGE/'launch/circle_laser_test.launch').getroot()
        node=next(n for n in root.findall('node') if n.attrib['type']=='circle_laser_test_node.py')
        names={p.attrib['name'] for p in node.findall('param')}
        self.assertTrue({'on_frames','off_frames','image_timeout','output_topic','enable_laser'}<=names)
        hardware=next(n for n in root.findall('node') if n.attrib['type']=='laser_test_bridge')
        self.assertEqual(hardware.attrib['if'],'$(arg enable_laser)')
        cmake=(PACKAGE/'CMakeLists.txt').read_text(encoding='utf-8-sig')
        self.assertEqual(cmake.count('add_executable('),1)
        self.assertEqual(cmake.count('add_message_files('),1)

    def test_protected_files_unchanged(self):
        import hashlib
        files=json.loads((PACKAGE/'test/protected_files_baseline.json').read_text(encoding='utf-8-sig'))
        for item in files:
            self.assertEqual(hashlib.sha256((PACKAGE.parent/item['path']).read_bytes()).hexdigest().upper(),item['hash'],item['path'])

    def test_launch_isolation(self):
        for name in ['window_test.launch','window_test_camera.launch','circle_laser_test.launch','circle_usb_camera.launch',
                     'window_competition_calibration.launch','window_competition_adapter.launch']:
            root=ET.parse(PACKAGE/'launch'/name).getroot()
            for node in root.iter('node'):
                self.assertIn(node.attrib['pkg'], ['robot_vision','usb_cam'])
            for include in root.iter('include'):
                self.assertNotIn('mission',include.attrib['file'])
                self.assertNotIn('mavros',include.attrib['file'])


def save_fixtures():
    output=PACKAGE/'test'/'artifacts'; output.mkdir(exist_ok=True)
    for name,frame,detector in [('window',window_image(),detect_windows),('circle',ring_image(),detect_circles)]:
        targets,edges=detector(frame)
        for t in targets:
            cv2.drawContours(frame,[t['contour']],-1,(0,220,0),2)
            cv2.drawMarker(frame,tuple(map(int,t['center'])),(0,0,255),cv2.MARKER_CROSS,16,2)
        cv2.imwrite(str(output/(name+'_synthetic.png')),frame)
        cv2.imwrite(str(output/(name+'_edges.png')),edges)


def load_tests(loader, standard_tests, pattern):
    import test_air_target
    standard_tests.addTests(loader.loadTestsFromModule(test_air_target))
    import test_node_callbacks
    import test_competition_adapter
    standard_tests.addTests(loader.loadTestsFromModule(test_node_callbacks))
    standard_tests.addTests(loader.loadTestsFromModule(test_competition_adapter))
    return standard_tests


if __name__=='__main__':
    print('Offline environment: OpenCV',cv2.__version__,'NumPy',np.__version__)
    save_fixtures()
    unittest.main(verbosity=2)
