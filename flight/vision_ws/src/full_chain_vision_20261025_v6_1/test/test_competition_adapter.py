#!/usr/bin/env python3
"""Standard-library-only adapter regression; fake ROS transport, NO hardware."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from robot_vision_test.competition_adapter import Confirmation, capture_age, parse_window_status

METHODS = ('checker_texture', 'checker_contour', 'nested_frame')


def observation(stamp=1000.0):
    return dict(state='found', count=1, no_control=True, rgb_only=True,
                input_encoding='rgb8', image_width=640, image_height=480,
                capture_stamp=stamp, frame_seq=42, camera_frame_id='camera_optical',
                targets=[dict(method='checker_texture', outline_kind='opening',
                              center_px=[330.0, 230.0], width_px=200.0, height_px=120.0)])


class ValidationTests(unittest.TestCase):
    def parse(self, data):
        return parse_window_status(data, allowed_methods=METHODS)

    def test_valid_rgb_target(self):
        target, reason = self.parse(json.dumps(observation()))
        self.assertEqual(reason, 'found')
        self.assertEqual((target['center_x'], target['center_y']), (330.0, 230.0))
        self.assertEqual(target['frame_seq'], 42)

    def test_empty_or_invalid_json(self):
        for data in ('', '{', '[]', 'null', '{}', 'x' * 33000, '[' * 1500):
            with self.subTest(data=data[:10]):
                self.assertIsNone(self.parse(data)[0])

    def test_no_target_and_multiple_targets(self):
        for count in (0, 2, True, '1'):
            data = observation()
            data['count'] = count
            self.assertIsNone(self.parse(data)[0])
        data = observation()
        data['targets'] *= 2
        self.assertIsNone(self.parse(data)[0])

    def test_detector_identity_and_encoding(self):
        for key, value in (('no_control', False), ('rgb_only', False),
                           ('input_encoding', '16UC1'), ('input_encoding', 'mono8')):
            data = observation()
            data[key] = value
            self.assertIsNone(self.parse(data)[0])

    def test_dimensions_must_match_without_pixel_scaling(self):
        for key, value in (('image_width', 1280), ('image_height', 720),
                           ('image_width', 640.0), ('image_width', True)):
            data = observation()
            data[key] = value
            self.assertIsNone(self.parse(data)[0])

    def test_zero_nan_missing_and_overflow_stamp(self):
        for stamp in (0, -1, None, True, float('nan'), float('inf'), 2 ** 2048, 4294967296):
            data = observation()
            data['capture_stamp'] = stamp
            self.assertIsNone(self.parse(data)[0])

    def test_invalid_frame_sequence_rejected_without_exception(self):
        for sequence in ('bad', None, [], True, -1, 4294967296):
            data = observation()
            data['frame_seq'] = sequence
            self.assertIsNone(self.parse(data)[0])

    def test_invalid_geometry_and_nonfinite_values(self):
        for center in ([640, 240], [-1, 240], [320, 480], [True, 240],
                       [float('nan'), 240], [2 ** 2048, 240], [320], '320,240'):
            data = observation()
            data['targets'][0]['center_px'] = center
            self.assertIsNone(self.parse(data)[0])
        for width in (0, 20, True, float('inf'), 1000):
            data = observation()
            data['targets'][0]['width_px'] = width
            self.assertIsNone(self.parse(data)[0])

    def test_unapproved_method_or_outline(self):
        for key, value in (('method', 'hough'), ('outline_kind', 'unknown')):
            data = observation()
            data['targets'][0][key] = value
            self.assertIsNone(self.parse(data)[0])
        self.assertIsNone(parse_window_status(observation(), allowed_methods=())[0])

    def test_capture_age_and_invalid_ros_clock(self):
        target, _ = self.parse(observation())
        self.assertTrue(capture_age(target, 1000.2)[0])
        self.assertEqual(capture_age(target, 1001.0)[1], 'stale_capture')
        self.assertEqual(capture_age(target, 999.0)[1], 'future_capture')
        for now in (0, None, True, float('nan')):
            self.assertFalse(capture_age(target, now)[0])

    def test_n_confirmation_and_first_miss_revokes(self):
        gate = Confirmation(5, 3)
        for _ in range(4):
            self.assertFalse(gate.found())
        self.assertTrue(gate.found())
        self.assertFalse(gate.missing())
        self.assertEqual((gate.hits, gate.misses), (0, 1))
        for _ in range(4):
            self.assertFalse(gate.found())
        self.assertTrue(gate.found())

    def test_m_is_absence_diagnostic_only(self):
        gate = Confirmation(1, 3)
        gate.found()
        for missed in (1, 2, 3):
            self.assertFalse(gate.missing())
            self.assertEqual(gate.misses, missed)
        gate.invalidate()
        self.assertFalse(gate.on)
        self.assertEqual(gate.hits, 0)


class Stamp:
    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value


class PointStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=Stamp(), seq=0, frame_id='')
        self.point = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class String:
    def __init__(self, data=''):
        self.data = data


class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.params, self.publishers, self.remaps = {}, [], {}
        self.clock = dict(wall=100.0, ros=1000.0)
        self.graph = [[], [], []]
        self.graph_error, self.graph_calls = None, 0
        self.node = None
        fake_rospy = types.ModuleType('rospy')
        fake_rospy.get_param = lambda name, default=None: self.params.get(name.lstrip('~'), default)
        fake_rospy.resolve_name = lambda name: name if name.startswith('/') else '/' + name
        fake_rospy.get_name = lambda: '/window_competition_adapter'
        fake_rospy.Publisher = self.publisher
        fake_rospy.Subscriber = lambda *args, **kwargs: None
        fake_rospy.on_shutdown = lambda callback: None
        fake_rospy.Time = types.SimpleNamespace(now=lambda: Stamp(self.clock['ros']), from_sec=Stamp)
        for name in ('logwarn', 'logerr', 'logfatal', 'signal_shutdown'):
            setattr(fake_rospy, name, lambda *args: None)
        fake_rosgraph = types.ModuleType('rosgraph')
        fake_rosgraph.Master = lambda name: types.SimpleNamespace(getSystemState=self.get_graph)
        modules = {'rospy': fake_rospy, 'rosgraph': fake_rosgraph,
                   'geometry_msgs': types.ModuleType('geometry_msgs'),
                   'geometry_msgs.msg': types.SimpleNamespace(PointStamped=PointStamped),
                   'std_msgs': types.ModuleType('std_msgs'),
                   'std_msgs.msg': types.SimpleNamespace(String=String)}
        self.module_patch = patch.dict(sys.modules, modules)
        self.module_patch.start()
        spec = importlib.util.spec_from_file_location(
            'competition_adapter_callback_test', PACKAGE / 'scripts/window_competition_adapter.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.clock_patch = patch.object(self.module.time, 'monotonic', lambda: self.clock['wall'])
        self.clock_patch.start()

    def tearDown(self):
        if self.node is not None:
            self.node.shutdown()
        self.clock_patch.stop()
        self.module_patch.stop()

    def publisher(self, name, message_type, **options):
        pub = types.SimpleNamespace(resolved_name=self.remaps.get(name, name),
                                    messages=[], options=options, message_type=message_type)
        pub.publish = lambda msg: pub.messages.append(copy.deepcopy(msg))
        self.publishers.append(pub)
        return pub

    def get_graph(self):
        self.graph_calls += 1
        if self.graph_error:
            raise self.graph_error
        return self.graph

    def make_node(self, background=False):
        self.node = self.module.WindowCompetitionAdapter()
        if not background:
            self.node.stop_event.set()
            self.node.watchdog.join(1)
        return self.node

    def send(self, data=None):
        self.clock['wall'] += 0.1
        self.clock['ros'] += 0.1
        self.node.status_cb(String(json.dumps(data or observation(self.clock['ros']))))

    def confirm(self):
        for _ in range(self.node.on_frames):
            self.send()
        self.assertTrue(self.node.last_published_on)

    def last_diag(self):
        return json.loads(self.node.diag_pub.messages[-1].data)

    def test_default_isolated_startup_and_no_master_query(self):
        node = self.make_node()
        self.assertEqual(node.target_pub.resolved_name, '/window_test/mission_target')
        self.assertEqual(node.target_pub.messages[0].header.frame_id, 'no object')
        self.assertFalse(self.last_diag()['flight_ready'])
        self.assertEqual(self.graph_calls, 0)
        self.assertFalse(node.target_pub.options.get('latch', False))

    def test_confirmation_header_pixels_and_alignment(self):
        node = self.make_node()
        for _ in range(4):
            self.send()
            self.assertEqual(node.target_pub.messages[-1].header.frame_id, 'no object')
        self.send()
        msg = node.target_pub.messages[-1]
        self.assertEqual(msg.header.frame_id, 'square')
        self.assertEqual(msg.header.stamp.value, self.clock['ros'])
        self.assertEqual(msg.header.seq, 42)
        self.assertEqual((msg.point.x, msg.point.y, msg.point.z), (330.0, 230.0, 0.0))
        self.assertEqual(self.last_diag()['error_x_px'], 10.0)
        self.assertTrue(self.last_diag()['horizontal_centered'])

    def test_first_lost_frame_revokes_and_requires_n_new_frames(self):
        node = self.make_node()
        self.confirm()
        lost = observation(self.clock['ros'])
        lost.update(state='not_found', count=0, targets=[])
        self.send(lost)
        self.assertEqual(node.target_pub.messages[-1].header.frame_id, 'no object')
        self.assertFalse(self.last_diag()['absence_confirmed'])
        for _ in range(4):
            self.send()
            self.assertFalse(node.last_published_on)
        self.send()
        self.assertTrue(node.last_published_on)

    def test_replay_stale_or_future_capture_immediately_invalid(self):
        node = self.make_node()
        self.confirm()
        for stamp in (node.last_capture_stamp, self.clock['ros'] - 5, self.clock['ros'] + 5):
            self.send(observation(stamp))
            self.assertFalse(node.last_published_on)
            self.assertEqual(node.gate.hits, 0)

    def test_multiple_targets_dimensions_and_bad_json_revoke(self):
        node = self.make_node()
        self.confirm()
        for key, value in (('count', 2), ('image_width', 1280), ('frame_seq', 'invalid')):
            data = observation(self.clock['ros'])
            data[key] = value
            self.send(data)
            self.assertFalse(node.last_published_on)
        node.status_cb(String('{'))
        self.assertEqual(self.last_diag()['reason'], 'invalid_json')

    def test_watchdog_no_callback_and_no_ros_time_progress(self):
        node = self.make_node()
        self.confirm()
        self.clock['wall'] += 1.1
        node.watchdog_cb()
        self.assertFalse(node.last_published_on)
        self.assertEqual(self.last_diag()['reason'], 'status_timeout')
        before = len(node.target_pub.messages)
        node.watchdog_cb()
        self.assertGreater(len(node.target_pub.messages), before)

    def test_capture_expiry_precedes_status_timeout(self):
        node = self.make_node()
        self.confirm()
        self.clock['wall'] += 0.9
        self.clock['ros'] += 0.9
        node.watchdog_cb()
        self.assertEqual(self.last_diag()['reason'], 'stale_capture')
        self.assertFalse(node.last_published_on)

    def test_callback_gap_resets_without_watchdog_tick(self):
        node = self.make_node()
        self.confirm()
        self.clock['wall'] += 1.2
        self.clock['ros'] += 1.2
        self.send()
        self.assertEqual(node.gate.hits, 1)
        self.assertFalse(node.last_published_on)

    def test_background_wall_watchdog_actually_fires(self):
        self.params.update(status_timeout=0.12, max_capture_age=0.12, on_frames=1)
        node = self.make_node(background=True)
        self.send()
        self.assertTrue(node.last_published_on)
        self.clock['wall'] += 0.2
        deadline = time.perf_counter() + 1.0
        while node.last_published_on and time.perf_counter() < deadline:
            time.sleep(0.01)
        self.assertFalse(node.last_published_on)
        self.assertEqual(self.last_diag()['reason'], 'status_timeout')

    def test_shutdown_revokes_and_late_callback_cannot_reenable(self):
        node = self.make_node()
        self.confirm()
        node.shutdown()
        before = len(node.target_pub.messages)
        self.send()
        self.assertEqual(len(node.target_pub.messages), before)
        self.assertFalse(node.last_published_on)
        self.assertEqual(self.last_diag()['reason'], 'adapter_shutdown')

    def test_legacy_refused_without_both_flags(self):
        for flags in ({}, {'allow_legacy_output': True}, {'publish_legacy_object_position': True}):
            self.params.clear()
            self.params.update(output_topic='/object_position', **flags)
            with self.assertRaises(ValueError):
                self.module.WindowCompetitionAdapter()
        self.assertEqual(len(self.publishers), 0)

    def enable_legacy(self):
        self.params.update(output_topic='/object_position', allow_legacy_output=True,
                           publish_legacy_object_position=True)

    def test_explicit_legacy_inspection_matches_pointstamped(self):
        self.enable_legacy()
        node = self.make_node()
        self.confirm()
        self.assertEqual(node.target_pub.resolved_name, '/object_position')
        self.assertEqual(self.graph_calls, 1)

    def test_legacy_conflicting_publisher_mission_or_master_failure(self):
        self.enable_legacy()
        for graph in ([[['/object_position', ['/old_detector']]], [], []],
                      [[], [['/ctrl_mode', ['/uav_mission']]], []]):
            self.graph = graph
            with self.assertRaises(ValueError):
                self.module.WindowCompetitionAdapter()
        self.graph_error = RuntimeError('master unreachable')
        with self.assertRaises(ValueError):
            self.module.WindowCompetitionAdapter()
        self.assertEqual(len(self.publishers), 0)

    def test_parameter_types_methods_and_nan_fail_before_publish(self):
        for key, value in (('allow_legacy_output', 'false'), ('on_frames', True),
                           ('status_timeout', float('nan')), ('expected_width', 0),
                           ('allowed_methods', 'checker_texture'), ('allowed_methods', []),
                           ('require_capture_stamp', False), ('legacy_frame_id', 'ring')):
            self.params.clear()
            self.params[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.module.WindowCompetitionAdapter()
        self.assertEqual(len(self.publishers), 0)

    def test_legacy_rejects_non_640x480(self):
        self.enable_legacy()
        self.params['expected_width'] = 1280
        with self.assertRaises(ValueError):
            self.module.WindowCompetitionAdapter()

    def test_output_diagnostics_input_and_remap_isolation(self):
        for key, value in (('output_topic', '/cmd_vel'), ('diagnostics_topic', '/ctrl_mode'),
                           ('status_topic', '/object_position'),
                           ('output_topic', '/window_test/status')):
            self.params.clear()
            self.params[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.module.WindowCompetitionAdapter()
        self.params.clear()
        self.remaps['/window_test/mission_target'] = '/object_position'
        with self.assertRaises(ValueError):
            self.module.WindowCompetitionAdapter()
        self.assertTrue(all(not pub.messages for pub in self.publishers))


class StaticTests(unittest.TestCase):
    def test_protected_files_hashes_unchanged(self):
        baseline = json.loads((PACKAGE / 'test/protected_files_baseline.json').read_text(encoding='utf-8-sig'))
        for item in baseline:
            actual = hashlib.sha256((PACKAGE.parent / item['path']).read_bytes()).hexdigest().upper()
            self.assertEqual(actual, item['hash'], item['path'])

    def test_python38_syntax_and_no_motion_clients(self):
        for path in (PACKAGE / 'src/robot_vision_test/competition_adapter.py',
                     PACKAGE / 'scripts/window_competition_adapter.py',
                     PACKAGE / 'scripts/window_test_node.py'):
            source = path.read_text(encoding='utf-8')
            ast.parse(source, filename=str(path), feature_version=(3, 8))
            self.assertNotIn('ServiceProxy(', source)
            self.assertNotIn('setpoint_raw', source)
            self.assertNotIn('lib_pwm_control', source)

    def test_calibration_launch_forces_isolated_output(self):
        root = ET.parse(PACKAGE / 'launch/window_competition_calibration.launch').getroot()
        adapter = next(i for i in root.findall('include') if 'adapter.launch' in i.attrib['file'])
        args = {arg.attrib['name']: arg.attrib['value'] for arg in adapter.findall('arg')}
        self.assertEqual(args['allow_legacy_output'], 'false')
        self.assertEqual(args['publish_legacy_object_position'], 'false')
        for include in root.findall('include'):
            self.assertNotIn('mission', include.attrib['file'])
            self.assertNotIn('mavros', include.attrib['file'])
        self.assertEqual(root.findall('node'), [])

    def test_adapter_launch_defaults_and_install_dependencies(self):
        root = ET.parse(PACKAGE / 'launch/window_competition_adapter.launch').getroot()
        args = {arg.attrib['name']: arg.attrib.get('default') for arg in root.findall('arg')}
        self.assertEqual(args['output_topic'], '/window_test/mission_target')
        self.assertEqual(args['allow_legacy_output'], 'false')
        self.assertEqual(args['publish_legacy_object_position'], 'false')
        self.assertEqual(len(root.findall('node')), 1)
        package = ET.parse(PACKAGE / 'package.xml').getroot()
        self.assertIn('geometry_msgs', [d.text for d in package.findall('build_depend')])
        self.assertIn('rosgraph', [d.text for d in package.findall('run_depend')])
        cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
        self.assertIn('scripts/window_competition_adapter.py', cmake)

    def test_nested_camera_and_detector_include_arguments_exist(self):
        includes = {
            '$(find robot_vision)/launch/window_test.launch': PACKAGE / 'launch/window_test.launch',
            '$(find robot_vision)/launch/window_test_camera.launch': PACKAGE / 'launch/window_test_camera.launch',
            '$(find robot_vision)/launch/window_competition_adapter.launch': PACKAGE / 'launch/window_competition_adapter.launch',
            '$(find astra_camera)/launch/gemini.launch': PACKAGE.parent / 'ros_astra_camera-main/launch/gemini.launch',
        }
        for name in ('window_competition_calibration.launch', 'window_test.launch',
                     'window_test_camera.launch'):
            root = ET.parse(PACKAGE / 'launch' / name).getroot()
            for include in root.iter('include'):
                with self.subTest(parent=name, child=include.attrib['file']):
                    self.assertIn(include.attrib['file'], includes)
                    child = ET.parse(includes[include.attrib['file']]).getroot()
                    declared = {arg.attrib['name'] for arg in child.findall('arg')}
                    passed = {arg.attrib['name'] for arg in include.findall('arg')}
                    self.assertTrue(passed <= declared, passed - declared)

    def test_competition_profile_keeps_all_field_parameters(self):
        field = (PACKAGE / 'config/window_test.yaml').read_text(encoding='utf-8')
        frozen = (PACKAGE / 'config/window_competition_vision.yaml').read_text(encoding='utf-8')
        self.assertTrue(frozen.endswith(field))


if __name__ == '__main__':
    print('Adapter-only offline regression: fake ROS; no OpenCV/ROS master/hardware')
    unittest.main(verbosity=2)
