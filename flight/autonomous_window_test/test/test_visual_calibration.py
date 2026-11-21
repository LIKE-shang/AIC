"""Two-run visual calibration, real TargetHold logic, no ROS or hardware IO."""
import copy
import json
import math
from pathlib import Path
import uuid
from types import SimpleNamespace as NS
import unittest
from hover_target import TargetHold, DEFAULTS, settings
from visual_calibration import CalibrationSession, tick_observed


def profile(mode='learn', seed=None):
    p = dict(DEFAULTS, enabled=True, visual_servo_enabled=True,
             visual_calibration_mode=mode, trigger_policy='hover_timed')
    c = dict(hover_target=p, full_chain={'selected_target_ids': [1, 3]},
             visual_calibration={'scene_key': 'original-field-camera-v1'})
    if seed is not None:
        c['visual_calibration']['seed'] = seed
    return c


class Plant:
    def __init__(self, bias=(0., 0.), mode='learn', seed=None):
        self.cfg = profile(mode, seed)
        self.h = TargetHold(self.cfg['hover_target'])
        self.session = CalibrationSession(self.cfg, {'boot_id': 'test-boot', 'run_id': 'test-run'})
        pose = NS(p=(0., 0., 1.), q=(0., 0., 0., 1.), stamp=100.)
        core = NS(pose=pose, c={'stable_speed_mps': .05, 'max_setpoint_lead_m': .04, 'yaw_limit_rad': .0872665})
        self.m = NS(core=core, position=(0., 0., 1.), target_z=1., origin=(0., 0., 0.),
                    takeoff_heading=0., heading=0., aim_anchor=(0., 0., 1.),
                    corridor=lambda point: True, hover_target=self.h,
                    visual_calibration=self.session)
        self.m.output = self.h.status
        self.m.handover = lambda *args: self.h.status()
        self.wall = 0.;self.bias = bias;self.spot = True;self.ambiguous = False
        self.error_override = None;self.allow_on = True;self.allow_off = True
        self.preexisting = False

    def step(self, dt=.05, freeze_image=False):
        self.wall += dt
        c = self.m.core
        old = c.pose.p
        delta = [max(-.002, min(.002, self.m.position[k]-old[k])) for k in range(2)]
        c.pose.p = (old[0]+delta[0], old[1]+delta[1], 1.)
        c.pose.stamp = 100.+self.wall
        stamp = self.h.last_stamp if freeze_image and self.h.last_stamp else c.pose.stamp
        on = self.h.request_on and self.allow_on
        if not self.h.request_on and not self.allow_off:
            on = True
        err = list(self.error_override or (self.bias[0]-c.pose.p[1]*400, self.bias[1]-c.pose.p[0]*400))
        seen = self.spot and (on or self.preexisting)
        candidates = ([{}]*2 if self.ambiguous else [{}]) if seen else []
        d = dict(valid=True, capture_stamp=stamp, camera_frame_id='circle_usb_optical_frame',
                 source_image_size_px=[640, 360], reference_point_px=[372.75, 194.984375],
                 reference_error_px=[0., 0.], inner_semiaxes_px=[20.8, 20.8], inner_angle_deg=0.,
                 reference_normalized_radius=0., observed_error_px=err if seen else None,
                 observed_spot_px=[372.75, 194.984375] if seen else None,
                 observed_candidates=candidates, observed_unique=seen and not self.ambiguous,
                 observed_inside_inner=seen and math.hypot(*err)<20.8,
                 observed_normalized_radius=math.hypot(*err)/20.8 if seen else None)
        bridge = dict(scope=self.h.scope, fault_latched=False,
                      state='ON_ACKNOWLEDGED' if on else 'OFF_ACKNOWLEDGED', request_id=self.h.request_id)
        i = NS(wall=self.wall, ros=100.+self.wall,
               hover_visual=(self.wall, d), hover_laser=(self.wall, bridge))
        return self.h.tick(self.m, i, dt)

    def until(self, phase, count=250):
        for _ in range(count):
            out = self.step()
            if self.h.phase == phase:
                return out
        raise AssertionError((phase, self.h.phase, self.h.reason))


class CalibrationTests(unittest.TestCase):
    def test_learn_saves_actual_average_only_after_three_seconds_and_off_ack(self):
        s = Plant();s.until('OFF_WAIT')
        self.assertFalse(s.session.pending)
        self.assertGreaterEqual(s.h.stable_exposure_s, 3.)
        s.step()
        row = s.session.document()['targets']['1']
        self.assertTrue(row['off_acknowledged'])
        self.assertGreater(row['distinct_pose_samples'], 30)
        self.assertEqual(row['actual_position_local_m'], [0., 0., 1.])
        self.assertFalse(row['physical_hit_verified'])

    def test_first_run_corrects_bias_second_holds_learned_position_without_correction(self):
        first = Plant((11., 0.));first.until('DONE')
        self.assertGreater(first.h.correction_updates, 0)
        doc = first.session.document()
        # Supply the other target only to exercise selected-target validation.
        doc['targets']['3'] = copy.deepcopy(doc['targets']['1'])
        second = Plant((11., 0.), 'reuse', doc)
        point = second.session.station(second.m, (0., 0., 1.), 1)
        second.m.position = point;second.m.core.pose.p = point;second.m.aim_anchor = point
        second.until('DONE')
        self.assertEqual(second.h.correction_updates, 0)
        self.assertTrue(second.h.completed)

    def test_missing_or_ambiguous_spot_never_uses_prediction_or_records_success(self):
        for ambiguous in (False, True):
            s = Plant();s.until('EXPOSE');s.ambiguous = ambiguous;s.spot = ambiguous
            s.until('ABORT')
            self.assertFalse(s.h.completed);self.assertEqual(s.h.correction_updates, 0)
            self.assertFalse(s.session.pending)

    def test_missing_spot_resets_continuous_exposure(self):
        s = Plant();s.until('EXPOSE')
        for _ in range(45):s.step()
        self.assertGreater(s.h.stable_exposure_s, 1.)
        s.spot = False;s.step();self.assertEqual(s.h.stable_exposure_s, 0.)
        s.spot = True;s.until('ABORT')
        self.assertFalse(s.h.completed)

    def test_replayed_image_does_not_count_or_move(self):
        s = Plant();s.until('EXPOSE');s.step()
        before = s.h.stable_exposure_s
        for _ in range(3):s.step(freeze_image=True)
        self.assertEqual(before, s.h.stable_exposure_s)
        self.assertEqual(s.h.correction_updates, 0)

    def test_preexisting_reflection_prevents_probe(self):
        s = Plant();s.preexisting = True
        for _ in range(40):s.step()
        self.assertFalse(s.h.request_on);self.assertIsNone(s.h.on_request_wall)

    def test_missing_on_or_off_ack_never_saves(self):
        s = Plant();s.allow_on = False;s.until('ABORT')
        self.assertFalse(s.session.pending)
        s = Plant();s.until('OFF_WAIT');s.allow_off = False;s.until('ABORT')
        self.assertFalse(s.session.pending)

    def test_reuse_rotates_relative_point_and_keeps_current_height(self):
        first = Plant();first.until('DONE');doc = first.session.document()
        row = doc['targets']['1'];row['position_forward_left_up_m'] = [3., -2., 1.]
        doc['targets']['3'] = copy.deepcopy(row)
        s = Plant(mode='reuse', seed=doc)
        s.m.origin = (10., 20., 5.);s.m.takeoff_heading = math.pi/2;s.m.heading = math.pi/2
        point = s.session.station(s.m, (12., 23., 6.), 1)
        self.assertEqual(point, (12., 23., 6.))

    def test_reuse_rejects_wrong_scene_profile_target_and_nonfinite_evidence(self):
        first = Plant();first.until('DONE');doc = first.session.document()
        doc['targets']['3'] = copy.deepcopy(doc['targets']['1'])
        for mutate in (lambda d:d.update(scene_key='changed-field'),
                       lambda d:d.update(profile_sha256='bad'),
                       lambda d:d['targets'].pop('3'),
                       lambda d:d['targets']['1'].update(yaw_relative_rad=float('nan')),
                       lambda d:d['targets']['1'].update(off_acknowledged=False)):
            bad = copy.deepcopy(doc);mutate(bad)
            with self.assertRaises(ValueError):Plant(mode='reuse', seed=bad)

    def test_reuse_rejects_point_outside_original_disk_or_incompatible_yaw(self):
        first = Plant();first.until('DONE');doc = first.session.document()
        doc['targets']['3'] = copy.deepcopy(doc['targets']['1'])
        s = Plant(mode='reuse', seed=doc)
        with self.assertRaises(ValueError):s.session.station(s.m, (2., 2., 1.), 1)
        s.m.heading = .2
        with self.assertRaises(ValueError):s.session.station(s.m, (0., 0., 1.), 1)

    def test_atomic_unique_record_file_and_no_write_without_evidence(self):
        folder=Path(__file__).resolve().parents[2]/'_diagnostics/visual_calibration_20261026/test_records'/uuid.uuid4().hex
        folder.mkdir(parents=True)
        s = Plant();self.assertIsNone(s.session.save(folder))
        s.until('DONE');a = s.session.save(folder);b = s.session.save(folder)
        self.assertNotEqual(a, b)
        self.assertEqual(json.loads(Path(a).read_text())['targets']['1']['source_identity']['boot_id'], 'test-boot')
        self.assertFalse(list(folder.glob('*.tmp')))

    def test_new_mode_cannot_be_mixed_with_old_fixed_timed_profile(self):
        p = profile()['hover_target'];p['fixed_position_timed'] = True
        with self.assertRaises(ValueError):settings(p)

    def test_changed_spot_in_second_run_triggers_bounded_correction(self):
        first = Plant();first.until('DONE');doc = first.session.document()
        doc['targets']['3'] = copy.deepcopy(doc['targets']['1'])
        second = Plant((11., 0.), 'reuse', doc);second.until('DONE')
        self.assertGreater(second.h.correction_updates, 0)
        self.assertLessEqual(second.h.servo_path_m, .15)

    def test_correction_never_expands_original_nominal_disk(self):
        first = Plant();first.until('DONE');doc = first.session.document()
        doc['targets']['3'] = copy.deepcopy(doc['targets']['1'])
        second = Plant(mode='reuse', seed=doc)
        second.session.nominal_stations['1'] = (0., 0., 1.)
        second.m.aim_anchor = (.14, 0., 1.)
        second.m.position = (.15, 0., 1.);second.m.core.pose.p = (.15, 0., 1.)
        d = dict(reference_error_px=[0., 100.], servo_error_px=[0., 100.],
                 inner_semiaxes_px=[20.8,20.8], inner_angle_deg=0.)
        second.h.correct_reference(second.m, d, .05, 1.)
        self.assertEqual(second.h.correction_updates, 0)
        self.assertEqual(second.h.last_correction_reason, 'correction_boundary_hold')

    def test_offline_builder_keeps_base_and_successful_ground_unmodified(self):
        import importlib.util
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('calibration_builder', root/'scripts/build_visual_calibration_profile.py')
        builder = importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        base = json.loads((root/'config/full_chain_second_window_return.json').read_text())
        before = copy.deepcopy(base)
        cfg = builder.build(base, 'learn', 'same-field')
        self.assertEqual(base, before)
        for key in ('mission','control','session_window','full_chain','site_identity'):
            self.assertEqual(cfg.get(key), base.get(key))
        self.assertEqual(cfg['hover_target']['visual_calibration_mode'], 'learn')
        self.assertFalse(cfg['hover_target']['paced_visual_servo'])

    def test_both_targets_and_two_windows_same_supervisor_automatic_landing(self):
        from test_second_window_return import ReturnSim
        from dataclasses import replace
        s = ReturnSim()
        p = s.m.hover_target.p
        p.update(fixed_position_timed=False, visual_servo_enabled=True,
                 visual_calibration_mode='learn', trigger_policy='hover_timed', exposure_s=3., max_light_s=5.)
        cfg = profile();cfg['hover_target'] = copy.deepcopy(p)
        session = CalibrationSession(cfg, {'boot_id':'synthetic','run_id':'same-run'})
        s.m.visual_calibration = session
        original = s.inputs
        def inputs():
            i = original();h = s.m.hover_target
            d = dict(i.hover_visual[1])
            on = h.request_on
            d.update(observed_unique=on, observed_candidates=[{}] if on else [],
                     observed_spot_px=[372.75,194.984375] if on else None,
                     observed_error_px=[0.,0.] if on else None,
                     observed_inside_inner=on, observed_normalized_radius=0. if on else None)
            return replace(i, hover_visual=(i.wall,d))
        s.inputs = inputs;s.start();origin = s.m.origin;owner = s.m.core
        s.until('AUTO_LANDING', limit=4500)
        self.assertEqual(sorted(session.pending), ['1','3'])
        self.assertEqual(s.actions, ['OFFBOARD','ARM','LAND'])
        self.assertIs(s.m.core, owner);self.assertEqual(s.m.origin, origin)
        self.assertTrue(s.m.return_journey.second_pass_complete)
        s.until('DONE')
        self.assertTrue(s.m.return_journey.off_ack)

    def test_new_two_target_profile_passes_existing_flight_policy_with_fresh_binding(self):
        from test_two_targets import binder
        from test_full_chain_binding import FullChainBinding
        from full_chain_policy import validate
        root=Path(__file__).resolve().parents[1]
        base=json.loads((root/'config/full_chain_two_targets.json').read_text())
        base['hover_target'].update(fixed_position_timed=False,visual_servo_enabled=True,
                                   visual_calibration_mode='learn',require_stable_target_before_light=True)
        base['visual_calibration']={'scene_key':'same-field'}
        _,snap,bounds=FullChainBinding().data()
        bound=binder.bind(base,snap,bounds,[1,3])
        self.assertTrue(validate(bound,True))

    def test_real_binder_uses_full_second_window_return_profile(self):
        root=Path(__file__).resolve().parents[1]
        source=(root/'scripts/bind_two_target_trial.py').read_text()
        self.assertIn("template=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())",source)
        cfg=json.loads((root/'config/full_chain_second_window_return.json').read_text())
        self.assertTrue(cfg['mission']['second_window_return_enabled'])
        self.assertTrue(cfg['full_chain']['second_window_return']['enabled'])

    def test_nan_duration_is_not_successful_calibration_evidence(self):
        first=Plant();first.until('DONE');doc=first.session.document()
        doc['targets']['3']=copy.deepcopy(doc['targets']['1'])
        doc['targets']['1']['stable_exposure_s']=float('nan')
        with self.assertRaises(ValueError):Plant(mode='reuse',seed=doc)


if __name__ == '__main__':
    unittest.main()
