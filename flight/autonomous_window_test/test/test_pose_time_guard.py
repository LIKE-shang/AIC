"""Offline timestamp evidence and preflight admission tests. No ROS/hardware."""
import copy,json,math,unittest,types,threading
from pathlib import Path
from unittest.mock import Mock,patch
from window_pass.core import Controller,Pose
from pose_time_guard import PoseTimeGuard

ROOT=Path(__file__).resolve().parents[1]

class GuardTests(unittest.TestCase):
    def setUp(self):
        self.c=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())['control']
        self.core=Controller(copy.deepcopy(self.c));self.g=PoseTimeGuard(self.c,True)
    def feed(self,stamp=100.,ros=100.,wall=10.,p=(0.,0.,1.),q=(0.,0.,0.,1.),frame='map',child='base_link',phase='TAKEOFF'):
        pose=Pose(stamp,p,q,frame,child);prior=self.core.pose;latest=self.core.latest_stamps.get('pose',0.)
        accepted=self.core.pose_update(pose,ros,wall)
        self.g.pose(pose,ros,wall,prior,latest,accepted,phase,arrival=(ros,wall))
        return self.g.rows[-1]
    def healthy(self,age=.005,offset_span=0.,rtt=1.,caller='/mavros'):
        for n in range(151):
            t=n*.02;self.feed(100.+t-age,100.+t,10.+t,phase='IDLE')
            if n%5==0:self.g.timesync(100.+t,10.+t,100.+t,1e18,1e18+offset_span*n/150.,rtt,caller)
        return self.g.ground(103.,13.)
    def test_candidate_from_failure_only_future_check(self):
        self.feed(100.,100.,10.)
        row=self.feed(100.036189795,100.01599741,10.036189795,p=(.001380617,0.,1.))
        self.assertFalse(row['accepted']);self.assertEqual(row['failed_checks'],['future_timestamp'])
        self.assertAlmostEqual(row['age_s'],-.020192385)
        self.assertAlmostEqual(row['speed_mps'],.038149,places=5)
        self.assertEqual(self.g.first_active_rejection['rejected'],row)
    def test_exact_20ms_policy_retained(self):
        self.assertEqual(self.c['future_tolerance_s'],.02)
        self.assertTrue(self.feed(100.0199)['accepted'])
        self.assertFalse(self.feed(100.0202)['accepted'])
    def test_individual_rejection_reasons(self):
        cases=[({'stamp':99.84},'stale_timestamp'),({'stamp':float('nan')},'timestamp_finite_positive'),
               ({'frame':'wrong'},'local_frame'),({'child':'wrong'},'body_frame'),
               ({'p':(float('nan'),0.,1.)},'position_finite'),({'q':(0.,0.,0.,0.)},'quaternion_valid')]
        for kw,reason in cases:
            self.setUp();self.assertIn(reason,self.feed(**kw)['failed_checks'])
    def test_duplicate_jump_speed_attitude(self):
        cases=[({'stamp':100.},'timestamp_monotonic'),
               ({'stamp':100.1,'ros':100.1,'p':(.36,0.,1.)},'position_step'),
               ({'stamp':100.01,'ros':100.01,'p':(.03,0.,1.)},'pose_speed'),
               ({'stamp':100.1,'ros':100.1,'q':(0.,0.,math.sin(.2),math.cos(.2))},'attitude_step')]
        for kw,reason in cases:
            self.setUp();self.feed();self.assertIn(reason,self.feed(**kw)['failed_checks'])
    def test_first_rejection_not_overwritten_or_latched_on_ground(self):
        self.feed(100.03,phase='IDLE');self.assertIsNone(self.g.first_active_rejection)
        first=self.feed(100.04);self.feed(100.05)
        self.assertEqual(self.g.first_active_rejection['rejected'],first)
        self.assertNotEqual(self.g.last_rejection,first)
    def test_bounded_and_json_finite(self):
        for n in range(300):self.feed(stamp=float('nan'))
        for n in range(100):self.g.timesync(100.,10.,100.,float('inf'),0.,0.,'/mavros')
        self.assertEqual(len(self.g.rows),160);self.assertEqual(len(self.g.sync),80)
        json.dumps(self.g.evidence(),allow_nan=False)
    def test_healthy_window_admitted(self):self.assertTrue(self.healthy()['ready'])
    def test_insufficient_history_not_admitted(self):
        self.feed();self.assertFalse(self.g.ground(100.,10.)['ready'])
    def test_future_reserve_is_ground_only(self):
        r=self.healthy(age=-.016)
        self.assertIn('pose_future_headroom_insufficient',r['blockers'])
        self.assertTrue(all(x['accepted'] for x in self.g.rows))
        self.assertEqual(r['runtime_future_tolerance_s'],.02)
    def test_bad_pose_remains_blocker_after_recovery(self):
        self.healthy();self.feed(103.04,103.01,13.01,phase='IDLE');self.feed(103.02,103.02,13.02,phase='IDLE')
        self.assertIn('rejected_pose_in_window',self.g.ground(103.02,13.02)['blockers'])
    def test_stale_window_and_clock_reversal_not_admitted(self):
        self.healthy();self.assertFalse(self.g.ground(104.,14.)['ready'])
        self.assertFalse(self.g.ground(102.,12.)['ready'])
    def test_missing_sync(self):
        self.healthy();self.g.sync.clear()
        self.assertIn('insufficient_timesync_window',self.g.ground(103.,13.)['blockers'])
    def test_unstable_sync(self):self.assertIn('timesync_offset_unstable',self.healthy(offset_span=6e6)['blockers'])
    def test_high_rtt(self):self.assertIn('timesync_rtt_excessive',self.healthy(rtt=10.)['blockers'])
    def test_untrusted_caller(self):self.assertIn('invalid_or_untrusted_timesync',self.healthy(caller='/fake')['blockers'])
    def test_stale_sync(self):
        self.healthy()
        for x in self.g.sync:x['stamp']-=1.
        self.assertIn('timesync_stale_now',self.g.ground(103.,13.)['blockers'])

class RuntimeTests(unittest.TestCase):
    def test_runtime_keeps_original_latch_and_terminal_evidence(self):
        from test_auto_transport import Boundary,app
        from test_takeoff import TakeoffSim
        sim=TakeoffSim();sim.start();sim.until('HOVER');r=Boundary().runtime(sim)
        r.history=[];r.now=lambda:(sim.f.ros,sim.f.wall)
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=1.145),orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        msg=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:sim.f.ros+.0202),frame_id='map'),child_frame_id='base_link',pose=types.SimpleNamespace(pose=pose))
        r.pose(msg)
        self.assertEqual(sim.m.phase,'FAULT');self.assertEqual(sim.m.reason,'invalid_pose_pulse_latched')
        self.assertIn('future_timestamp',r.pose_time_guard.first_active_rejection['rejected']['failed_checks'])
        r.motion=None;r.stop=Mock();r.fcu_text=[];out={'phase':'FAULT'}
        with patch('builtins.print'):r.finish_terminal(out)
        self.assertEqual(out['pose_input_diagnostics']['first_active_rejection'],r.pose_time_guard.first_active_rejection)
    def preflight_runtime(self,enabled):
        from test_auto_transport import Boundary,app
        from test_airborne_geometry import DynamicSim
        sim=DynamicSim(False);r=Boundary().runtime(sim)
        r.pose_time_guard=PoseTimeGuard(sim.m.core.c,enabled)
        r.errors=[];r.airborne_geometry=True;r.control=sim.m.core.c
        r.params={k:[0,0.] for k in app.PARAMETERS}
        r.cfg={'mission':sim.options,'acceptance':{k:True for k in app.EVIDENCE},
               'evidence_id':'SYNTHETIC_TEST_ONLY','expected_fcu_parameters':r.params}
        r.cfg['acceptance']['rgbd_geometry_verified']=True
        r.history=[(sim.f.wall,.245)]*60;r.graph_reason='test'
        r.rgbd_row=(sim.f.wall,{'calibration_valid':True,'ready':False,'metric_geometry_ready':False})
        r.depth_row=(sim.f.wall,sim.f.ros)
        return r
    def test_disabled_does_not_add_preflight_blockers(self):
        report=self.preflight_runtime(False).report()
        self.assertTrue(report['ready'],report['blockers'])
        self.assertEqual(report['pose_time_guard'],{'enabled':False})
    def test_enabled_missing_time_history_blocks_actual_report(self):
        report=self.preflight_runtime(True).report()
        self.assertFalse(report['ready'])
        self.assertIn('pose_time_guard:insufficient_pose_window',report['blockers'])
        self.assertIn('pose_time_guard:insufficient_timesync_window',report['blockers'])
    def test_ground_binding_preserves_guard_and_policies(self):
        import importlib.util
        from test_full_chain_binding import FullChainBinding
        spec=importlib.util.spec_from_file_location('guard_pair_bind',ROOT/'scripts/bind_two_target_trial.py')
        binder=importlib.util.module_from_spec(spec);spec.loader.exec_module(binder)
        _,snapshot,boundaries=FullChainBinding().data()
        template=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())
        bound=binder.bind(template,snapshot,boundaries,[1,3])
        self.assertEqual(bound['pose_time_guard'],{'enabled':True})
        self.assertEqual(bound['control']['future_tolerance_s'],.02)
        self.assertTrue(bound['hover_target']['visual_target_hold'])
        self.assertFalse(bound['hover_target']['fixed_position_timed'])
    def test_recorder_preserves_exclusive_command_subscriber(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('recorder_test',ROOT/'scripts/run_recorded_hover_trial.py')
        recorder=importlib.util.module_from_spec(spec);spec.loader.exec_module(recorder)
        self.assertIn('/mavros/timesync_status',recorder.TOPICS)
        self.assertIn('/mavros/time_reference',recorder.TOPICS)
        self.assertNotIn('/mavros/setpoint_raw/local',recorder.TOPICS)

if __name__=='__main__':unittest.main()
