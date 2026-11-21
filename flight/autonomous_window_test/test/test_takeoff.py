"""No-vision takeoff/hover/land tests; synthetic feedback only."""
import copy,math,unittest
from unittest.mock import patch
from test_mission import Sim
from mission import Mission,Inputs
from takeoff import TakeoffCore,takeoff_blockers,check_takeoff_graph,EVIDENCE
from window_pass.core import Pose

class TakeoffSim(Sim):
    def __init__(self):
        super().__init__(False)
        cfg=copy.deepcopy(self.m.core.c)
        for k in cfg:
            if k.endswith('_verified'):cfg[k]=False
        cfg.update(enable_traverse=False,pass_stage_approved=False,window_center_local_m=[],
                   camera_translation_body_m=[],camera_rotation_body_from_optical_xyzw=[],
                   optical_frame='',camera_info_sha256='',vision_profile_sha256='',algorithm_sha256='')
        self.m=Mission(cfg,self.options,experiment='takeoff_land')
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def update(self,follow=True):
        f=self.f;c=self.m.core;f.wall+=.05;f.ros+=.05;self.elapsed+=.05
        if follow and self.out and self.out['position'] is not None:f.p=tuple(self.out['position'])
        if self.mode=='AUTO.LAND':
            f.p=(f.p[0],f.p[1],max(.245,f.p[2]-.02))
            if f.p[2]<=.245:self.airborne=False;self.armed=False
        c.pose_update(Pose(f.ros,f.p,f.q,'map','base_link'),f.ros,f.wall)
        c.flight_state(f.ros,True,self.armed,self.mode,f.ros,f.wall)
        c.extended_state(f.ros,self.airborne,f.ros,f.wall)
        c.graph(True,'synthetic_exclusive',f.wall)
        if self.armed and f.p[2]>.30:self.airborne=True

class TakeoffLifecycle(unittest.TestCase):
    def test_complete_without_any_camera_or_window(self):
        s=TakeoffSim();s.start();commands=[]
        for _ in range(1500):
            out=s.step()
            if out['position'] is not None:commands.append(out['position'])
            if out['phase']=='DONE':break
        self.assertEqual(s.m.phase,'DONE',s.m.reason)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
        self.assertTrue(s.out['takeoff_hover_completed'])
        self.assertIsNone(s.m.core.obs)
        self.assertNotIn('image',s.m.core.streams)
        self.assertTrue(all(p[:2]==s.m.origin[:2] for p in commands))
        self.assertAlmostEqual(max(p[2] for p in commands)-.245,.90)
        events=s.m.events;phases=[e['new'] for e in events]
        self.assertIn('HOVER',phases)
        self.assertTrue(set(phases).isdisjoint({'SEARCH','ALIGN','PASS'}))
        hover=next(e['wall'] for e in events if e['new']=='HOVER')
        land=next(e['wall'] for e in events if e['new']=='LAND_REQUEST')
        self.assertGreaterEqual(land-hover,s.m.o['hover_time_s'])
    def test_pass_permission_cannot_be_enabled(self):
        s=TakeoffSim()
        for key in ('enable_traverse','pass_stage_approved'):
            cfg=dict(s.m.core.c);cfg[key]=True
            with self.assertRaisesRegex(ValueError,'pass_disabled'):Mission(cfg,s.options,experiment='takeoff_land')
    def test_visual_services_always_refused(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        for method in (s.m.core.request_hold,s.m.core.request_start,s.m.core.request_pass):
            self.assertFalse(method(s.f.ros,s.f.wall)[0])
    def test_takeoff_test_acceptance_has_no_circular_gate(self):
        self.assertNotIn('auto_takeoff_land_verified',EVIDENCE)
        self.assertNotIn('rgbd_geometry_verified',EVIDENCE)
        self.assertIn('fcu_failsafe_verified',EVIDENCE)
        self.assertIn('simulation_verified',EVIDENCE)
    def test_fence_and_error_budget_still_required(self):
        s=TakeoffSim()
        for key,value in [('geofence_min_m',[]),('position_uncertainty_m',0),('stopping_margin_m',0),('body_frame','')]:
            cfg=dict(s.m.core.c);cfg[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):TakeoffCore(cfg)
    def test_motion_permission_still_required(self):
        s=TakeoffSim();cfg=dict(s.m.core.c,enable_motion=False)
        with self.assertRaises(ValueError):Mission(cfg,s.options,experiment='takeoff_land')
    def test_illegal_visual_phase_fails_without_pass(self):
        s=TakeoffSim();s.start();s.until('HOVER');s.m.phase='ALIGN'
        s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.out['position'])
    def test_pilot_takeover_no_automatic_land_or_rearm(self):
        s=TakeoffSim();s.start();s.until('HOVER');s.mode='POSCTL';s.step()
        before=list(s.actions);s.mode='OFFBOARD'
        for _ in range(20):s.step()
        self.assertEqual(s.m.phase,'TAKEOVER');self.assertEqual(s.actions,before)
        self.assertNotIn('LAND',s.actions)
    def test_stale_pose_no_blind_landing(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        s.m.core.streams['pose']=(s.f.ros,s.f.wall,False)
        out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'FAULT');self.assertIsNone(out['action'])
    def test_failed_climb_lands_but_does_not_pass_test(self):
        s=TakeoffSim();s.start();s.until('TAKEOFF')
        for _ in range(550):
            s.step(follow=False)
            if s.m.phase=='LAND_REQUEST':break
        self.assertEqual(s.m.phase,'LAND_REQUEST')
        s.until('DONE');self.assertFalse(s.out['takeoff_hover_completed'])
    def test_unstable_hover_lands_and_reports_incomplete(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        for _ in range(320):
            s.update();s.m.core.pose_speed=.06;s.out=s.m.tick(s.inputs())
            if s.out['position'] is not None:s.m.sent(s.f.wall)
            if s.out['action']=='LAND':s.mode='AUTO.LAND';break
        self.assertEqual(s.m.phase,'LAND_REQUEST');s.until('DONE')
        self.assertFalse(s.out['takeoff_hover_completed'])
    def test_low_height_during_hover_lands_not_pass(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        for _ in range(12):
            s.f.p=(0,0,s.f.p[2]-.01);s.m.position=s.f.p;s.step(follow=False)
            if s.m.phase=='LAND_REQUEST':break
        self.assertEqual(s.m.phase,'LAND_REQUEST');self.assertFalse(s.out['takeoff_hover_completed'])
    def test_startup_geometry_refuses_unsafe_column(self):
        s=TakeoffSim();s.m.core.c['geofence_min_m'][2]=.3;s.start()
        self.assertEqual(s.m.phase,'REFUSED');self.assertFalse(s.actions)

class TakeoffGraph(unittest.TestCase):
    def graph(self,publisher=False):
        cfg=TakeoffSim().m.core.c
        pubs=[[t,['/mavros']] for t in ('/mavros/state','/mavros/extended_state','/mavros/local_position/odom')]
        subs=[]
        if publisher:pubs.append(['/mavros/setpoint_raw/local',['/auto']]);subs.append(['/mavros/setpoint_raw/local',['/mavros']])
        return [pubs,subs,[]],cfg
    def test_no_camera_sources_required(self):
        graph,cfg=self.graph();self.assertTrue(check_takeoff_graph(graph,'/auto',cfg)[0])
    def test_other_setpoint_family_is_not_ignored(self):
        graph,cfg=self.graph();graph[0].append(['/mavros/setpoint_position/local',['/other']])
        self.assertFalse(check_takeoff_graph(graph,'/auto',cfg)[0])
    def test_direct_override_refused(self):
        graph,cfg=self.graph();graph[0].append(['/mavros/rc/override',['/other']])
        self.assertFalse(check_takeoff_graph(graph,'/auto',cfg)[0])
    def test_exclusive_publisher_and_subscriber_required(self):
        graph,cfg=self.graph(True);self.assertTrue(check_takeoff_graph(graph,'/auto',cfg,True)[0])
        graph[1][0][1].append('/another');self.assertFalse(check_takeoff_graph(graph,'/auto',cfg,True)[0])

class SiteBinding(unittest.TestCase):
    def test_measured_rear_gap_and_negative_local_height(self):
        from field_site import bind_bounds
        s=TakeoffSim();measurement={'boundary_distances_m':{'front':4.5,'back':.81,'left':1.,'right':3.3},'ceiling_above_ground_m':4.}
        out=bind_bounds(measurement,(.12,.03,-.16),(0,0,0,1),s.options,s.m.core.c)
        self.assertAlmostEqual(out['geofence_min_m'][0],.12-.81)
        self.assertAlmostEqual(out['geofence_max_m'][0],.12+4.5)
        self.assertAlmostEqual(out['target_fcu_position_local_m'][2],-.16+.90)
        self.assertEqual(out['software_envelope_ceiling_above_ground_m'],2.)
    def test_rotated_inscribed_fence_never_expands_measured_area(self):
        from field_site import bind_bounds
        s=TakeoffSim();angle=.01;c,ss=math.cos(angle),math.sin(angle)
        measurement={'boundary_distances_m':{'front':4.5,'back':.81,'left':1.,'right':3.3},'ceiling_above_ground_m':4.}
        out=bind_bounds(measurement,(0,0,.245),(0,0,math.sin(angle/2),math.cos(angle/2)),s.options,s.m.core.c)
        for x in (out['geofence_min_m'][0],out['geofence_max_m'][0]):
            for y in (out['geofence_min_m'][1],out['geofence_max_m'][1]):
                forward,left=c*x+ss*y,-ss*x+c*y
                self.assertTrue(-.81-1e-9<=forward<=4.5+1e-9)
                self.assertTrue(-3.3-1e-9<=left<=1+1e-9)
    def test_missing_ceiling_or_tiny_column_refused(self):
        from field_site import bind_bounds
        s=TakeoffSim()
        for ceiling,back in [(None,.81),(4.,.1)]:
            measurement={'boundary_distances_m':{'front':4.5,'back':back,'left':1.,'right':3.3},'ceiling_above_ground_m':ceiling}
            with self.assertRaises(ValueError):bind_bounds(measurement,(0,0,.245),(0,0,0,1),s.options,s.m.core.c)

if __name__=='__main__':unittest.main(verbosity=2)
