#!/usr/bin/env python3
import dataclasses
import math
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from window_pass.config import DEFAULTS, load_config, blockers
from window_pass.core import Controller, Pose
from window_pass.geometry import rotate, envelope_half_extents, opening_polygon, norm, sub
from window_pass.graph import check_graph, OUTPUT, POSE, STATE, EXTENDED, OBSERVATION, IMAGE, INFO
from fixtures import Feed,calibration


class ConfigurationTests(unittest.TestCase):
    def test_default_observe_and_missing_measurements(self):
        c = Controller()
        self.assertEqual(c.state,'OBSERVE')
        self.assertIsNone(c.output(1,1))
        self.assertIn('missing:camera_translation_body_m',c.blockers)
        # Clearance is no longer blocked by default: the 1.05x0.80 opening with a
        # 0.62m airframe leaves real room. Motion stays blocked by the missing
        # measurements and attestations, which the next assertion covers.
        self.assertNotIn('insufficient:worst_case_clearance',c.blockers)
        self.assertIn('missing:envelope_center_body_m',c.blockers)
    def test_motion_configuration_refuses_every_missing_attestation(self):
        with self.assertRaises(ValueError): Controller({'enable_motion':True})
        c = calibration(True)
        for k in c:
            if k.endswith('_verified'):
                q = dict(c); q[k] = False
                with self.subTest(k=k), self.assertRaises(ValueError): Controller(q)
    def test_bad_types_and_limits(self):
        for k,v in [('enable_motion','false'),('enable_traverse',1),('pose_timeout_s',float('nan')),('confirm_frames',True),('control_hz',1.),('traverse_speed_mps',1.),('vehicle_size_m',[]),('new_typo',1)]:
            with self.subTest(k=k),self.assertRaises(ValueError): load_config({k:v})
    def test_geometry_and_attitude_reserve(self):
        c = calibration()
        self.assertEqual(blockers(c),[])
        c['opening_size_m'] = [.63,.36]
        self.assertIn('insufficient:worst_case_clearance',blockers(c))
        # A real competition window must pass, and the reserve must not treat
        # the abort threshold as a static lateral allowance.
        c['opening_size_m'] = [1.05,.80]
        self.assertEqual(blockers(c),[])
        widened = dict(c, max_tracking_error_m=.15, stopping_margin_m=.19)
        self.assertEqual(blockers(widened),[],
                         'abort threshold and stopping margin must not gate clearance')

    def test_envelope_centre_offset_does_not_widen_the_hull(self):
        base = calibration()
        shifted = dict(base, envelope_center_body_m=[0.,0.,-.09])
        self.assertEqual(blockers(base),blockers(shifted))
        # The fixture uses 2 deg limits, so the hull is close to the bare box.
        half = envelope_half_extents(base)
        self.assertAlmostEqual(half[1], .326585, places=5)
        self.assertAlmostEqual(half[2], .186114, places=5)
        # At the deployed 5 deg limits the widened aperture still fits.
        field = dict(DEFAULTS, yaw_limit_rad=math.radians(5),
                     tilt_limit_rad=math.radians(5), opening_size_m=[1.05,.80])
        self.assertAlmostEqual(envelope_half_extents(field)[1], .34981, places=4)
        self.assertAlmostEqual(envelope_half_extents(field)[2], .20360, places=4)

    def test_field_geofence_rejects_airspace_outside_the_venue(self):
        from window_pass.config import field_violations, FIELD_GEOFENCE_MIN_M, FIELD_GEOFENCE_MAX_M
        # The 2026-10-15 field config asked for 10 m ahead on a 5 m field.
        stale = calibration()
        stale['geofence_min_m'] = [-5., -5., .3]
        stale['geofence_max_m'] = [10., 5., 3.]
        self.assertTrue(field_violations(stale))
        # A fence inside the venue must pass untouched.
        inside = calibration()
        inside['geofence_min_m'] = list(FIELD_GEOFENCE_MIN_M)
        inside['geofence_max_m'] = list(FIELD_GEOFENCE_MAX_M)
        self.assertEqual(field_violations(inside), [])
        # An empty fence is a separate blocker, not a field violation.
        empty = calibration()
        empty['geofence_min_m'], empty['geofence_max_m'] = [], []
        self.assertEqual(field_violations(empty), [])

    def test_clearance_tracking_budget_rejects_abort_scale_values(self):
        for value in (0., -.01, .06, .2):
            with self.subTest(value=value), self.assertRaises(ValueError):
                load_config({'clearance_tracking_budget_m': value})
    def test_camera_optical_axes_are_body_right_down_forward(self):
        q = calibration()['camera_rotation_body_from_optical_xyzw']
        self.assertEqual(rotate(q,(1,0,0)),(0,-1,0))
        self.assertEqual(rotate(q,(0,1,0)),(0,0,-1))
        self.assertEqual(rotate(q,(0,0,1)),(1,0,0))


class TargetTests(unittest.TestCase):
    def test_confirm_needs_elapsed_time_not_only_count(self):
        f = Feed()
        for _ in range(20): f.step(dt=.01)
        self.assertFalse(f.ctrl.confirmed())
        for _ in range(110): f.step(dt=.01)
        self.assertTrue(f.ctrl.confirmed())
    def test_bad_frames_revoke_immediately(self):
        modifications = [dict(stamp=0),dict(stamp=1000),dict(stamp=9999),dict(width=320),dict(count=2),dict(valid=False),dict(center=(float('nan'),0)),dict(geometry='outer_frame'),dict(corners=((10,10),(20,20),(10,20),(20,10)))]
        for change in modifications:
            f = Feed(); f.prime()
            o = dataclasses.replace(f.observation(),stamp=f.ros+.001,**{k:v for k,v in change.items() if k!='stamp'})
            if 'stamp' in change: o.stamp=change['stamp']
            self.assertFalse(f.ctrl.observe(o,f.ros+.001,f.wall+.001),change)
            self.assertIsNone(f.ctrl.obs)
            self.assertFalse(f.ctrl.confirmed())
    def test_replay_out_of_order(self):
        f = Feed(); f.prime()
        self.assertFalse(f.ctrl.observe(f.observation(),f.ros,f.wall))
        self.assertFalse(f.ctrl.confirmed())
    def test_target_jump_and_method_switch(self):
        for kind in ('jump','method'):
            f = Feed(); f.ready()
            o = f.observation(); o.stamp+=.001
            if kind=='jump':
                o.center=(o.center[0]+50,o.center[1]); o.corners=tuple((x+50,y) for x,y in o.corners)
            else: o.method='nested_frame'; o.geometry='contour_inner_opening'
            f.ctrl.observe(o,f.ros+.001,f.wall+.001)
            self.assertEqual(f.ctrl.state,'ABORT')
    def test_camera_hash_and_pose_sync_are_required(self):
        f=Feed(); f.prime(); f.ctrl.obs.camera_hash='d'*64
        with self.assertRaisesRegex(ValueError,'sha256'): f.ctrl.measurement()
        f.ctrl.obs.camera_hash='a'*64; f.ctrl.poses.clear()
        with self.assertRaisesRegex(ValueError,'synchronised'): f.ctrl.measurement()
    def test_missing_calibrated_rays(self):
        f=Feed();f.prime();f.ctrl.obs.rays=()
        with self.assertRaisesRegex(ValueError,'optical_rays'): f.ctrl.measurement()
    def test_pose_interpolation(self):
        f=Feed();f.prime()
        p,q=f.ctrl.capture_pose(f.ros-.025)
        self.assertEqual(p,f.p); self.assertAlmostEqual(sum(x*x for x in q),1.)


class StateTests(unittest.TestCase):
    def test_shadow_all_services_never_output(self):
        f=Feed();f.ready()
        self.assertFalse(f.ctrl.request_hold(f.ros,f.wall)[0])
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))
        self.assertTrue(f.ctrl.request_pass(f.ros,f.wall)[0])
        for _ in range(400): f.step(follow=True)
        self.assertEqual(f.ctrl.state,'HOLD_COMPLETE')
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))
    def test_start_needs_hold_and_real_prestream(self):
        f=Feed(True);f.prime()
        self.assertFalse(f.ctrl.request_start(f.ros,f.wall)[0])
        self.assertIsNone(f.ctrl.output(f.ros,f.wall))
        self.assertTrue(f.ctrl.request_hold(f.ros,f.wall)[0])
        self.assertFalse(f.ctrl.request_start(f.ros,f.wall)[0])
        for _ in range(40): f.step()
        self.assertTrue(f.ctrl.request_start(f.ros,f.wall)[0])
    def test_stopping_margin_shortens_the_committed_path(self):
        # Regression: stopping_margin_m was required for motion but consumed by
        # nothing, so changing it altered no decision at all.
        short = Feed(True); short.cfg['stopping_margin_m'] = .01
        short.ctrl = Controller(short.cfg); short.ready()
        self.assertTrue(short.ctrl.request_pass(short.ros, short.wall)[0])
        long_ = Feed(True); long_.cfg['stopping_margin_m'] = .12
        long_.ctrl = Controller(long_.cfg); long_.ready()
        self.assertTrue(long_.ctrl.request_pass(long_.ros, long_.wall)[0])
        delta = norm(sub(short.ctrl.endpoint, long_.ctrl.endpoint))
        self.assertAlmostEqual(delta, .11, places=6,
                               msg='stopping margin must shorten the traverse distance')

    def test_no_auto_commit_and_separate_human_pass(self):
        f=Feed(True);f.ready()
        for _ in range(30): f.step(follow=True)
        self.assertEqual(f.ctrl.state,'PREPASS')
        self.assertAlmostEqual(f.ctrl.command[0],0)
        self.assertTrue(f.ctrl.request_pass(f.ros,f.wall)[0])
        self.assertFalse(f.ctrl.request_pass(f.ros,f.wall)[0])
    def test_finite_traverse_ignores_absent_target_then_holds(self):
        f=Feed(True);f.ready();self.assertTrue(f.ctrl.request_pass(f.ros,f.wall)[0])
        f.observe=False
        for _ in range(400): f.step(follow=True)
        self.assertEqual(f.ctrl.state,'HOLD_COMPLETE',f.ctrl.reason)
        self.assertEqual(f.ctrl.command,f.ctrl.endpoint)
        endpoint=f.ctrl.endpoint
        for _ in range(30): f.step(follow=True)
        self.assertEqual(f.ctrl.command,endpoint)
    def test_prepass_loss_aborts_and_never_recovers_automatically(self):
        f=Feed(True);f.ready();f.observe=False;f.step()
        self.assertEqual(f.ctrl.state,'ABORT')
        f.observe=True
        for _ in range(30):f.step()
        self.assertEqual(f.ctrl.state,'ABORT')
        self.assertFalse(f.ctrl.request_start(f.ros,f.wall)[0])
    def test_alignment_direction_and_no_forward_change(self):
        f=Feed();f.p=(0.,.04,1.2);f.start()
        for _ in range(45): f.step(follow=True)
        self.assertLess(f.ctrl.command[1],.04)
        self.assertAlmostEqual(f.ctrl.command[0],0.)
    def test_yaw_rotates_image_correction_not_fixed_world_y(self):
        c=calibration();c['window_yaw_rad']=math.pi/2;c['window_center_local_m']=[0.,1.,1.2]
        f=Feed(cfg=c);f.p=(.04,0.,1.2);f.start()
        for _ in range(45):f.step(follow=True)
        self.assertLess(f.ctrl.command[0],.04)
        self.assertAlmostEqual(f.ctrl.command[1],0.)
    def test_independent_watchdog_image_and_pose_timeout(self):
        for name in ('image','pose'):
            f=Feed(True);f.ready()
            for _ in range(8):f.step(**{name:False})
            self.assertIn(f.ctrl.state,('FAULT','ABORT'))
    def test_ros_clock_stall_still_times_out_on_wall_clock(self):
        f=Feed(True);f.ready()
        f.ctrl.watchdog(f.ros,f.wall+.5)
        self.assertEqual(f.ctrl.state,'FAULT')
    def test_loop_deadline_latches_fault(self):
        f=Feed(True);f.ready();f.ctrl.tick(f.ros+.2,f.wall+.2)
        self.assertEqual(f.ctrl.state,'FAULT')
    def test_pose_jump_latches_before_recovery(self):
        f=Feed(True);f.ready();f.p=(1.,0.,1.2);f.step()
        self.assertEqual(f.ctrl.state,'FAULT')
    def test_mode_exit_relinquishes_and_never_reacquires(self):
        f=Feed(True);f.ready();f.mode='POSCTL';f.step()
        self.assertEqual(f.ctrl.state,'RELEASED');self.assertIsNone(f.ctrl.output(f.ros,f.wall))
        f.mode='OFFBOARD';f.step();self.assertIsNone(f.ctrl.output(f.ros,f.wall))
    def test_graph_conflict_and_disconnect(self):
        for case in ('graph','fcu'):
            f=Feed(True);f.ready()
            if case=='graph': f.ctrl.graph(False,'rival',f.wall)
            else: f.ctrl.connected=False
            f.ctrl.watchdog(f.ros,f.wall)
            self.assertEqual(f.ctrl.state,'RELEASED')
            self.assertIsNone(f.ctrl.output(f.ros,f.wall))
    def test_manual_abort_freezes_not_reverse(self):
        f=Feed(True);f.ready();f.ctrl.request_pass(f.ros,f.wall)
        for _ in range(40):f.step(follow=True)
        f.ctrl.stop('human',f.ros,f.wall)
        fixed=f.ctrl.command
        for _ in range(30):f.step(follow=True)
        self.assertEqual(fixed,f.ctrl.command)
        self.assertEqual(f.ctrl.state,'ABORT')
    def test_pose_loss_freezes_previous_finite_setpoint_not_new_hover(self):
        f=Feed(True);f.ready();f.ctrl.request_pass(f.ros,f.wall)
        for _ in range(10): f.step(follow=True)
        for _ in range(5): f.step(pose=False,follow=False)
        self.assertEqual(f.ctrl.state,'FAULT')
        fixed=f.ctrl.command
        for _ in range(10):f.step(pose=False)
        self.assertEqual(fixed,f.ctrl.command)
    def test_stationary_pose_cannot_advance_trajectory_unbounded(self):
        f=Feed(True);f.ready();f.ctrl.request_pass(f.ros,f.wall)
        for _ in range(540):f.step()
        self.assertEqual(f.ctrl.state,'ABORT')
        self.assertLessEqual(f.ctrl.command[0],f.cfg['max_setpoint_lead_m'])
    def test_pass_gates_distance_survey_and_endpoint(self):
        for case in ('disabled','survey','fence'):
            f=Feed();f.ready()
            if case=='disabled': f.ctrl.c['enable_traverse']=False
            if case=='survey': f.ctrl.c['window_center_local_m']=[1.,.055,1.2]
            if case=='fence': f.ctrl.c['geofence_max_m']=[1.5,3,4]
            self.assertFalse(f.ctrl.request_pass(f.ros,f.wall)[0])
    def test_geofence_and_heading_fault(self):
        for case in ('fence','heading'):
            f=Feed(True);f.ready()
            if case=='fence':f.ctrl.c['geofence_max_m']=[.3,3,4]
            else:f.q=(0.,0.,math.sin(.03),math.cos(.03))
            f.step()
            self.assertEqual(f.ctrl.state,'FAULT')
    def test_release_requires_real_takeover(self):
        f=Feed(True);f.ready();self.assertFalse(f.ctrl.release(f.ros,f.wall)[0])
        f.mode='POSCTL';f.step();self.assertTrue(f.ctrl.release(f.ros,f.wall)[0])


class GraphTests(unittest.TestCase):
    def graph(self):
        pubs=[(t,['/mavros']) for t in (POSE,STATE,EXTENDED)]
        pubs += [(OBSERVATION,['/window_pass_observer']),(IMAGE,['/camera']),(INFO,['/camera'])]
        return [pubs,[],[]]
    def test_empty_competition_and_alternative_publishers(self):
        c=calibration(True);g=self.graph()
        self.assertTrue(check_graph(g,'/test',c)[0])
        for topic in (OUTPUT,'/mavros/setpoint_position/local','/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_raw/attitude','/mavros/trajectory/generated'):
            q=self.graph();q[0].append((topic,['/renamed_controller']))
            self.assertFalse(check_graph(q,'/test',c)[0])
    def test_mavros_known_feedback_and_trajectory_visualization_are_not_commands(self):
        for topic in ('/mavros/setpoint_raw/target_local', '/mavros/setpoint_raw/target_global',
                      '/mavros/setpoint_raw/target_attitude', '/mavros/setpoint_trajectory/desired',
                      '/mavros/trajectory/desired'):
            with self.subTest(topic=topic):
                g = self.graph(); g[0].append((topic, ['/mavros']))
                self.assertEqual(check_graph(g, '/test', calibration(True)),
                                 (True, 'graph_exclusive_snapshot'))

    def test_trajectory_visualization_does_not_allow_other_publishers(self):
        for topic in ('/mavros/setpoint_trajectory/desired', '/mavros/trajectory/desired'):
            for publishers in (['/rival'], ['/mavros', '/rival'], ['/test']):
                with self.subTest(topic=topic, publishers=publishers):
                    g = self.graph(); g[0].append((topic, publishers))
                    ok, reason = check_graph(g, '/test', calibration(True))
                    self.assertFalse(ok)
                    self.assertTrue(reason.startswith('competing_setpoint:'+topic+':'))

    def test_real_trajectory_input_is_blocked_even_if_publisher_is_mavros(self):
        for topic in ('/mavros/setpoint_trajectory/local', '/mavros/setpoint_trajectory/desired_extra',
                      '/mavros/setpoint_position/local', '/mavros/setpoint_unknown/input',
                      '/mavros/trajectory/generated', '/mavros/trajectory/path',
                      '/mavros/trajectory/desired_extra'):
            for publisher in ('/mavros', '/rival'):
                with self.subTest(topic=topic, publisher=publisher):
                    g = self.graph(); g[0].append((topic, [publisher]))
                    self.assertFalse(check_graph(g, '/test', calibration(True))[0])

    def test_feedback_does_not_hide_a_competing_real_trajectory_input(self):
        g = self.graph()
        g[0] += [('/mavros/setpoint_trajectory/desired', ['/mavros']),
                 ('/mavros/setpoint_trajectory/local', ['/other_controller'])]
        ok, reason = check_graph(g, '/test', calibration(True))
        self.assertFalse(ok)
        self.assertEqual(reason, 'competing_setpoint:/mavros/setpoint_trajectory/local:/other_controller')

    def test_expected_single_subscriber_and_inputs(self):
        c=calibration(True);g=self.graph();g[0].append((OUTPUT,['/test']));g[1].append((OUTPUT,['/mavros']))
        self.assertTrue(check_graph(g,'/test',c,True)[0])
        g[1][0][1].append('/rival');self.assertFalse(check_graph(g,'/test',c,True)[0])


if __name__=='__main__':unittest.main(verbosity=2)
