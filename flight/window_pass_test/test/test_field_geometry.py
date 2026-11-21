#!/usr/bin/env python3
"""User field dimensions + OBSERVE-only geometry; no real ROS/FCU execution."""
import copy
import importlib.util
import json
import math
import os
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as N

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'test'))
from window_pass.config import DEFAULTS
from window_pass.core import Controller
from window_pass.geometry import rotate
from fixtures import Feed, calibration, cleanup_tree

spec = importlib.util.spec_from_file_location('field_prepare', ROOT/'tools/prepare_observe_geometry_20261011_v2.py')
field = importlib.util.module_from_spec(spec)
spec.loader.exec_module(field)


def profile():
    c = calibration()
    c.update(geofence_min_m=[], geofence_max_m=[], position_uncertainty_m=0.0,
             stopping_margin_m=0.0, clearance_margin_m=0.04,
             yaw_limit_rad=DEFAULTS['yaw_limit_rad'], tilt_limit_rad=DEFAULTS['tilt_limit_rad'],
             max_tracking_error_m=0.12, max_setpoint_lead_m=0.04, align_tolerance_m=0.025)
    return field.installation_profile(c)


def derived(yaw=0.0):
    p = (0.14669425785541534, 0.015013015829026699, -0.9198312759399414)
    q = (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))
    # Camera is 1.04m high, window is 1.00m; 1.6m optical forward distance.
    rays = ((0.0,0.025,1.0), (-0.3125,-0.209375,1.0),
            (0.3125,-0.209375,1.0), (0.3125,0.259375,1.0), (-0.3125,0.259375,1.0))
    c, report = field.derive_profile(profile(), p, q, rays, 1.6, 1.1, 1.0)
    return c, report, p, q


def feed():
    c, report, p, q = derived()
    f = Feed(cfg=c)
    f.p, f.q = p, q
    f.prime()
    return f


class InstallationTests(unittest.TestCase):
    def test_cm_mm_conversion_and_optical_axes(self):
        c = profile()
        self.assertEqual(c['camera_translation_body_m'], [0.08,0.0,-0.06])
        self.assertEqual(c['envelope_center_body_m'], [0.0,0.0,-0.09])
        self.assertEqual(c['window_thickness_m'], 0.001)
        q = c['camera_rotation_body_from_optical_xyzw']
        self.assertEqual(rotate(q,(1,0,0)), (0,-1,0))
        self.assertEqual(rotate(q,(0,1,0)), (0,0,-1))
        self.assertEqual(rotate(q,(0,0,1)), (1,0,0))

    def test_defaults_and_limits_are_not_mutated_or_reduced(self):
        base = copy.deepcopy(DEFAULTS)
        before = copy.deepcopy(base)
        c = field.installation_profile(base)
        self.assertEqual(base, before)
        for key in ('vehicle_size_m','opening_size_m','clearance_margin_m','max_tracking_error_m',
                    'yaw_limit_rad','tilt_limit_rad','position_uncertainty_m','stopping_margin_m'):
            if key == 'opening_size_m':
                continue
            self.assertEqual(c[key], base[key])
        self.assertEqual(c['opening_size_m'], [1.05, 0.80])
        self.assertEqual(base['opening_size_m'], before['opening_size_m'])
        c['camera_translation_body_m'][0] = 42
        self.assertEqual(field.INSTALLATION['camera_translation_body_m'][0], 0.08)

    def test_permissions_never_inherited(self):
        c = field.installation_profile(calibration(True))
        for k,v in c.items():
            if k.endswith('_verified') or k in ('enable_motion','enable_traverse','pass_stage_approved'):
                self.assertFalse(v,k)
        c['enable_motion'] = True
        with self.assertRaises(ValueError):
            Controller(c)

    def test_negative_map_height_is_not_height_above_ground(self):
        c, r, p, _ = derived()
        self.assertAlmostEqual(r['ground_z_local_m'], -2.0198312759399414)
        self.assertAlmostEqual(c['window_center_local_m'][2], -1.0198312759399414)
        self.assertAlmostEqual(r['envelope_center_local_m'][2], p[2]-0.09)
        self.assertAlmostEqual(r['surveyed_vertical_error_m'], -0.01)
        self.assertAlmostEqual(r['visual_vs_height_delta_m'], 0.0)
        self.assertAlmostEqual(c['window_center_local_m'][0], p[0]+0.08+1.6)

    def test_camera_forward_distance_rotates_into_world_y(self):
        c, r, p, _ = derived(math.pi/2)
        self.assertAlmostEqual(c['window_center_local_m'][0], p[0])
        self.assertAlmostEqual(c['window_center_local_m'][1], p[1]+0.08+1.6)
        self.assertAlmostEqual(c['window_yaw_rad'], math.pi/2)

    def test_manual_height_is_not_overwritten_by_image(self):
        c, r, p, q = derived()
        rays = ((0.0,0.0,1.0),)*5
        _, result = field.derive_profile(c,p,q,rays,1.6,1.1,1.0)
        self.assertAlmostEqual(result['visual_vs_height_delta_m'], 0.04)
        self.assertAlmostEqual(result['window_center_local_m'][2], p[2]-0.1)

    def test_invalid_manual_numbers_and_rays_are_refused(self):
        c, _, p, q = derived()
        for d in (0.0,-1.0,math.nan,math.inf,10.0):
            with self.subTest(distance=d), self.assertRaises(ValueError):
                field.derive_profile(c,p,q,((0.,0.,1.),)*5,d,1.1,1.0)
        for rays in ((), ((0.,0.,0.),)*5, ((math.nan,0.,1.),)*5):
            with self.assertRaises(ValueError):
                field.derive_profile(c,p,q,rays,1.6,1.1,1.0)

    def test_large_tilt_is_not_accepted_as_vertical_plane_capture(self):
        c, _, p, _ = derived()
        q = (math.sin(0.2),0.0,0.0,math.cos(0.2))
        with self.assertRaisesRegex(ValueError,'approximately level'):
            field.derive_profile(c,p,q,((0.,0.,1.),)*5,1.6,1.1,1.0)

    def test_clearance_reports_real_geometry_not_abort_thresholds(self):
        c, _, _, _ = derived()
        r = field.clearance_report(c)
        self.assertAlmostEqual(r['nominal_level_side_gap_m'], 0.215)
        self.assertAlmostEqual(r['nominal_level_vertical_gap_m'], 0.225)
        # 2026-10-18: a 0.62m airframe in a 1.05x0.80m opening has real room.
        # The reserve now holds measurement-scale terms only; the abort
        # threshold and the longitudinal stopping margin no longer appear.
        self.assertTrue(all(x > 0 for x in r['configured_worst_case_residual_m']),
                        r['configured_worst_case_residual_m'])
        expected = (c['position_uncertainty_m'] + c['clearance_margin_m'] +
                    c['align_tolerance_m'] + c['clearance_tracking_budget_m'])
        self.assertAlmostEqual(r['clearance_reserve_m'], expected)
        self.assertLess(r['clearance_reserve_m'], c['max_tracking_error_m'] + c['stopping_margin_m'])
        # Neither measured nor self-certified by tuning limits down.
        self.assertFalse(r['uncertainty_and_stopping_measured'])
        self.assertFalse(r['limits_reduced_to_clear_warning'])

    def test_narrow_opening_still_reports_a_deficit(self):
        c, _, _, _ = derived()
        c = dict(c, opening_size_m=[0.70, 0.45])
        r = field.clearance_report(c)
        self.assertTrue(all(x < 0 for x in r['configured_worst_case_residual_m']))


class ObserveOnlyTests(unittest.TestCase):
    def test_metric_preview_with_full_motion_blockers_retained(self):
        f = feed()
        self.assertIn('missing:geofence_min_m', f.ctrl.blockers)
        self.assertIn('missing:position_uncertainty_m', f.ctrl.blockers)
        # Clearance itself now fits the measured aperture; observation stays
        # blocked by the survey inputs that the geometry capture does not supply.
        self.assertNotIn('insufficient:worst_case_clearance', f.ctrl.blockers)
        self.assertIn('missing:stopping_margin_m', f.ctrl.blockers)
        m = f.ctrl.measurement()
        self.assertAlmostEqual(m['vertical_error_m'], -0.01)
        self.assertAlmostEqual(m['lateral_error_m'], 0.0)
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))
        self.assertFalse(f.ctrl.request_hold(f.ros,f.wall)[0])

    def test_geometry_hash_and_pose_gates_are_not_exempted(self):
        for key, value in (('camera_translation_body_m',[]), ('envelope_center_body_m',[]),
                           ('window_center_local_m',[]), ('camera_rotation_body_from_optical_xyzw',[]),
                           ('camera_info_sha256',''), ('optical_frame','')):
            f = feed()
            c = copy.deepcopy(f.cfg)
            c[key] = value
            ctrl = Controller(c)
            ctrl.poses, ctrl.pose, ctrl.obs = f.ctrl.poses, f.ctrl.pose, f.ctrl.obs
            with self.subTest(key=key), self.assertRaises(ValueError):
                ctrl.measurement()
        f = feed()
        f.ctrl.obs.camera_hash = 'f'*64
        with self.assertRaisesRegex(ValueError,'camera_info_sha256_mismatch'):
            f.ctrl.measurement()
        f = feed()
        f.ctrl.poses.clear()
        with self.assertRaisesRegex(ValueError,'synchronised'):
            f.ctrl.measurement()

    def test_height_disagreement_still_rejected(self):
        f = feed()
        f.ctrl.c['window_center_local_m'][2] += 0.2
        with self.assertRaisesRegex(ValueError,'survey_visual_disagreement'):
            f.ctrl.measurement()

    def test_stale_measurement_revoked_and_never_outputs(self):
        f = feed()
        result = f.ctrl.tick(f.ros+1.0,f.wall+1.0)
        self.assertIn('measurement_rejected',result['measurement'])
        self.assertIsNone(result['suggested_velocity_local'])
        self.assertIsNone(f.ctrl.output(f.ros+1.0,f.wall+1.0))

    def test_motion_is_still_refused_before_and_after_preview(self):
        c, _, _, _ = derived()
        c['enable_motion'] = True
        with self.assertRaisesRegex(ValueError,'motion configuration refused'):
            Controller(c)
        f = feed()
        f.ctrl.c['enable_motion'] = True
        with self.assertRaisesRegex(ValueError,'geofence'):
            f.ctrl.measurement()


class PatchAndFileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT/'test')
        self.addCleanup(cleanup_tree, self.temp.name)
        self.path = Path(self.temp.name).resolve()
        self.path.relative_to((ROOT/'test').resolve())
    def test_patch_backup_and_idempotence(self):
        src = (ROOT/'src/window_pass/core.py').read_bytes()
        before = src.replace(field.NEW.encode(),field.OLD.encode())
        path = self.path/'core.py'
        path.write_bytes(before)
        self.assertTrue(field.patch_core(path))
        self.assertEqual(path.read_bytes(),src)
        self.assertFalse(field.patch_core(path))
        backup = path.with_name(path.name+'.before_observe_geometry_20261011_v2')
        self.assertEqual(backup.read_bytes(),before)
    def test_unknown_or_conflicting_source_is_not_overwritten(self):
        path = self.path/'core.py'
        path.write_text('# custom core\n',encoding='utf-8')
        with self.assertRaises(ValueError):
            field.patch_core(path)
        self.assertEqual(path.read_text(encoding='utf-8'),'# custom core\n')
    def test_profile_sidecar_never_overwrite_existing_files(self):
        c, report, _, _ = derived()
        path = self.path/'profile.json'
        field.save_new_profile(path,c,report)
        self.assertEqual(json.loads(path.read_text(encoding='utf-8')),c)
        before = path.read_bytes()
        with self.assertRaises(ValueError):
            field.save_new_profile(path,c,report)
        self.assertEqual(path.read_bytes(),before)


class StandaloneImportEnvironmentTests(unittest.TestCase):
    """Run the copied-tool use case, without inherited sys.path from test imports."""
    def run_offline_cli(self, include_source):
        directory = tempfile.TemporaryDirectory(dir=ROOT/'test')
        self.addCleanup(cleanup_tree, directory.name)
        work = Path(directory.name).resolve()
        work.relative_to((ROOT/'test').resolve())
        base = work/'base.json'
        base.write_text(json.dumps(DEFAULTS), encoding='utf-8')
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        if include_source:
            env['PYTHONPATH'] = str(ROOT/'src')
        # -S excludes site/user packages, but keeps the explicit PYTHONPATH.
        return subprocess.run(
            [sys.executable, '-S', str(ROOT/'tools/prepare_observe_geometry_20261011_v2.py'),
             '--base', str(base)], cwd=str(work), env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)

    def test_unsourced_standalone_tool_reproduces_reported_import_error(self):
        result = self.run_offline_cli(include_source=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("No module named 'window_pass'", result.stderr)

    def test_explicit_source_pythonpath_runs_offline_without_ros_or_rebuild(self):
        result = self.run_offline_cli(include_source=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report['result'], 'INSTALLATION_OFFLINE_ONLY')
        self.assertFalse(report['ROS_started'])


class CaptureTransportTests(unittest.TestCase):
    def run_capture(self, armed=False, calibration_valid=True, connected=True):
        c, _, p, q = derived()
        f = Feed(cfg=c)
        f.p, f.q = p, q
        subscribers = []
        class Stamp:
            def __init__(self, value): self.value = value
            def to_sec(self): return self.value
        class Subscriber:
            def __init__(self, topic, typ, callback, **kwargs):
                self.topic, self.callback, self.registered = topic, callback, True
                subscribers.append(self)
            def unregister(self): self.registered = False
        def advance(_):
            f.wall += 0.05
            f.ros += 0.05
            h = lambda frame: N(stamp=Stamp(f.ros),frame_id=frame)
            o = f.observation()
            msgs = {
                '/mavros/state': N(header=h(''), connected=connected, armed=armed, mode='MANUAL'),
                '/mavros/local_position/odom': N(header=h('map'), child_frame_id='base_link',
                    pose=N(pose=N(position=N(x=p[0],y=p[1],z=p[2]),
                                  orientation=N(x=q[0],y=q[1],z=q[2],w=q[3])))),
                '/window_pass_test/observation': N(header=h(o.frame), image_width=o.width,
                    image_height=o.height, input_encoding='rgb8', valid=True,
                    calibration_valid=calibration_valid, target_count=1,
                    center_px=N(x=o.center[0],y=o.center[1]),
                    opening_corners_px=[N(x=x,y=y) for x,y in o.corners],
                    detection_method=o.method, geometry_kind=o.geometry,
                    optical_rays=[N(x=x,y=y,z=z) for x,y,z in o.rays],
                    camera_info_sha256=o.camera_hash, vision_profile_sha256=o.profile_hash,
                    algorithm_sha256=o.algorithm_hash)}
            for sub in subscribers:
                if sub.registered: sub.callback(msgs[sub.topic])
        rospy = N(init_node=lambda *a,**kw:None, get_param=lambda *a:False,
                  Subscriber=Subscriber, Time=N(now=lambda:Stamp(f.ros)), is_shutdown=lambda:False)
        modules = {'rospy':rospy, 'mavros_msgs.msg':N(State=object),
                   'nav_msgs.msg':N(Odometry=object), 'window_pass_test.msg':N(WindowObservation=object)}
        # No Publisher or ServiceProxy exists in this fake API: any use would fail.
        with patch.dict(sys.modules,modules), patch.object(field.time,'sleep',advance), \
                patch.object(field.time,'monotonic',lambda:f.wall):
            try:
                result = field.capture(c,timeout_s=3.0)
                return result, subscribers
            finally:
                self.assertTrue(all(not sub.registered for sub in subscribers))

    def test_live_capture_read_only_subscriptions_and_metric(self):
        (ctrl, (p,q)), subs = self.run_capture()
        self.assertEqual(len(subs),3)
        self.assertIsNotNone(ctrl.obs)
        self.assertEqual(len(ctrl.obs.rays),5)
        self.assertEqual(ctrl.capture_pose(ctrl.obs.stamp),(p,q))
        self.assertFalse(ctrl.c['enable_motion'])
        self.assertIsNone(ctrl.output(0,0))

    def test_armed_aircraft_is_refused(self):
        with self.assertRaisesRegex(ValueError,'DISARMED'):
            self.run_capture(armed=True)

    def test_invalid_camera_calibration_cannot_generate_profile(self):
        with self.assertRaisesRegex(ValueError,'timeout'):
            self.run_capture(calibration_valid=False)

    def test_disconnected_fcu_cannot_generate_profile(self):
        with self.assertRaisesRegex(ValueError,'timeout'):
            self.run_capture(connected=False)


if __name__ == '__main__':
    unittest.main()
