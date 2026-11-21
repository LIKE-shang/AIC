"""Synthetic closed-loop lifecycle and fault injection; no ROS/hardware."""
import sys,unittest,copy,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'window_pass_test/src'),str(ROOT.parent/'window_pass_test/test')]
from fixtures import Feed,calibration
from window_pass.core import Pose
from mission import Mission,Inputs,DEFAULTS

class Sim:
    def __init__(self,target=True):
        cfg=calibration(True)
        cfg.update(envelope_center_body_m=[0,0,-.07],opening_size_m=[1.05,.8],
                   window_center_local_m=[1.,0.,1.05],geofence_min_m=[-3.,-3.,-1.],
                   max_tracking_error_m=.12,max_setpoint_lead_m=.04,
                   max_total_time_s=60.,max_traverse_time_s=30.)
        self.options=dict(DEFAULTS,ground_fcu_height_m=.245)
        self.m=Mission(cfg,self.options)
        self.f=Feed(cfg=cfg);self.f.p=(0.,0.,.245);self.f.ctrl=self.m.core
        self.armed=False;self.airborne=False;self.mode='MANUAL';self.target=target
        self.actions=[];self.out=None;self.elapsed=0.
        self.healthy=dict(state_fresh=True,landed_fresh=True,rc_ok=True,estimator_ok=True,battery_ok=True,graph_ok=True)
        for _ in range(30):self.update()
    def inputs(self):return Inputs(self.f.wall,self.f.ros,landed=not self.airborne,**self.healthy)
    def update(self,follow=True):
        f=self.f;c=self.m.core;f.wall+=.05;f.ros+=.05;self.elapsed+=.05
        if follow and self.out and self.out['position'] is not None:f.p=tuple(self.out['position'])
        if self.mode=='AUTO.LAND':
            f.p=(f.p[0],f.p[1],max(.245,f.p[2]-.02))
            if f.p[2]<=.245:self.airborne=False;self.armed=False
        c.pose_update(Pose(f.ros,f.p,f.q,'map','base_link'),f.ros,f.wall)
        c.image(f.ros,640,480,f.ros,f.wall)
        c.flight_state(f.ros,True,self.armed,self.mode,f.ros,f.wall)
        c.extended_state(f.ros,self.airborne,f.ros,f.wall)
        c.graph(True,'synthetic_exclusive',f.wall)
        f.observe=self.target;c.observe(f.observation(),f.ros,f.wall)
        if self.armed and f.p[2]>.30:self.airborne=True
    def start(self):self.out=self.m.start(self.inputs())
    def step(self,follow=True,respond=True):
        self.update(follow)
        self.out=self.m.tick(self.inputs())
        if self.out['position'] is not None:self.m.sent(self.f.wall)
        action=self.out['action']
        if action:
            self.actions.append(action)
            if respond:
                if action=='OFFBOARD':self.mode='OFFBOARD'
                elif action=='ARM':self.armed=True
                elif action=='LAND':self.mode='AUTO.LAND'
        return self.out
    def until(self,phase,limit=1800):
        for _ in range(limit):
            if self.m.phase==phase:return
            if self.m.phase in ('FAULT','TAKEOVER','REFUSED','DONE'):break
            self.step()
        raise AssertionError((phase,self.m.phase,self.m.reason))

class Lifecycle(unittest.TestCase):
    def test_complete_takeoff_detect_align_pass_land(self):
        s=Sim();s.start();s.until('DONE')
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
        transitions=[e['new'] for e in s.m.events]
        for phase in ('TAKEOFF','SEARCH','ALIGN','PASS','LANDING'):self.assertIn(phase,transitions)
        self.assertGreater(s.f.p[0],1.5)
        self.assertIsNone(s.out['position'])
    def test_no_target_timeout_lands_without_commit(self):
        s=Sim(False);s.start();s.until('DONE')
        self.assertNotIn('PASS',[e['new'] for e in s.m.events])
        self.assertIn('recognition_timeout',[e['reason'] for e in s.m.events])
        self.assertAlmostEqual(s.f.p[0],0.)
    def test_mode_request_success_is_not_state_confirmation(self):
        s=Sim();s.start()
        for _ in range(180):s.step(respond=False)
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('ARM',s.actions)
    def test_prestream_needs_actual_publications(self):
        s=Sim();s.start()
        for _ in range(40):s.update();s.m.tick(s.inputs())
        self.assertEqual(s.m.phase,'PRESTREAM')
    def test_no_rearm_or_reacquire_after_pilot_mode_exit(self):
        s=Sim();s.start();s.until('SEARCH');s.mode='POSCTL';s.step()
        self.assertEqual(s.m.phase,'TAKEOVER')
        before=list(s.actions)
        s.mode='OFFBOARD'
        for _ in range(20):s.step()
        self.assertEqual(s.actions,before);self.assertIsNone(s.out['position'])
    def test_pose_loss_stops_output_without_blind_landing(self):
        s=Sim();s.start();s.until('SEARCH')
        s.m.core.streams['pose']=(s.f.ros,s.f.wall,False)
        out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'FAULT');self.assertIsNone(out['position']);self.assertIsNone(out['action'])
    def test_rc_loss_stops_output(self):
        s=Sim();s.start();s.until('SEARCH');s.healthy['rc_ok']=False;s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.out['position'])
    def test_estimator_failure_no_blind_land(self):
        s=Sim();s.start();s.until('SEARCH');s.healthy['estimator_ok']=False;s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('LAND',s.actions)
    def test_graph_loss(self):
        s=Sim();s.start();s.healthy['graph_ok']=False;s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.out['position'])
    def test_landing_requires_actual_landed_and_disarmed(self):
        s=Sim(False);s.start();s.until('LAND_REQUEST')
        self.assertNotEqual(s.m.phase,'DONE')
        s.until('LANDING');self.assertTrue(s.armed)
        s.until('DONE');self.assertFalse(s.armed)
    def test_ground_evidence_blocks_arm(self):
        s=Sim();s.armed=True;s.update();s.start()
        self.assertEqual(s.m.phase,'REFUSED');self.assertFalse(s.actions)
    def test_ground_column_outside_fence_refuses(self):
        s=Sim();s.m.core.c['geofence_min_m'][2]=.3;s.start()
        self.assertEqual(s.m.phase,'REFUSED');self.assertFalse(s.actions)
    def test_height_offset_not_absolute_map_z(self):
        s=Sim();s.f.p=(0,0,-.53);s.m.core.pose=None
        for _ in range(50):s.update()
        s.m.core.c['geofence_min_m'][2]=-2.
        s.start();self.assertAlmostEqual(s.m.target_z,.37)
    def test_mount_mismatch_refuses(self):
        s=Sim();cfg=dict(s.m.core.c,envelope_center_body_m=[0,0,-.09])
        with self.assertRaisesRegex(ValueError,'envelope'):Mission(cfg,s.options)
    def test_missing_attestation_still_refuses(self):
        s=Sim();cfg=dict(s.m.core.c,failsafe_verified=False)
        with self.assertRaises(ValueError):Mission(cfg,s.options)
    def test_core_timeout_during_pass_does_not_descend_inside_frame(self):
        s=Sim();s.start();s.until('PASS')
        while s.f.p[0]<.7:s.step()
        s.m.core.stop('injected',s.f.ros,s.f.wall)
        s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('LAND',s.actions)
    def test_loop_stall_latches(self):
        s=Sim();s.start();s.f.wall+=.3
        self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')
    def test_rejected_arm_service_latches_fault(self):
        s=Sim();s.start();s.until('ARM')
        s.m.command_finished('ARM',False);s.step(respond=False)
        self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.out['position'])
    def test_takeoff_stuck_times_out_and_lands(self):
        s=Sim(False);s.start();s.until('TAKEOFF')
        for _ in range(550):
            s.step(follow=False)
            if s.m.phase=='LAND_REQUEST':break
        self.assertEqual(s.m.phase,'LAND_REQUEST');self.assertEqual(s.m.reason,'takeoff_timeout')
    def test_landing_request_not_observed_cannot_succeed(self):
        s=Sim(False);s.start();s.until('SEARCH')
        for _ in range(400):
            s.step(respond=False)
            if s.m.phase=='FAULT':break
        self.assertEqual(s.m.phase,'FAULT');self.assertEqual(s.actions.count('LAND'),1)
    def test_emergency_latches_without_mode_requests(self):
        s=Sim();s.start();s.until('SEARCH');i=s.inputs();i.emergency=True
        out=s.m.tick(i)
        self.assertEqual(out['phase'],'TAKEOVER');self.assertIsNone(out['action'])
    def test_rotated_lowest_corner_used_for_height(self):
        s=Sim();s.start();c=s.m.core
        c.pose.q=(math.sin(.04/2),0,0,math.cos(.04/2))
        bottom,_=s.m.full_height_bounds((0,0,1.145))
        self.assertLess(bottom,.90)

if __name__=='__main__':unittest.main(verbosity=2)
