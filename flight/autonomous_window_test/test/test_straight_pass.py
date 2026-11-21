"""Straight pass decisions and synthetic closed loop. Never hardware tests."""
import unittest,copy,math
from dataclasses import replace
from unittest.mock import Mock
from test_alignment_only import AlignOnlySim
from test_mission import Sim
from test_airborne_geometry import status
from mission import Mission
from straight_pass import straight_route,PassageDecision,advance_route
from test_auto_transport import app

class StraightSim(AlignOnlySim):
    def __init__(self,target=True):
        super().__init__(target)
        c=copy.deepcopy(self.m.core.c);c.update(enable_traverse=True,pass_stage_approved=True,
            max_traverse_time_s=30.,max_total_time_s=60.)
        self.m=Mission(c,self.options,airborne_geometry=True,experiment='straight_pass');self.f.ctrl=self.m.core
        for _ in range(30):self.update()
        # This mode does not depend on these functions at all.
        self.m.core.target_fresh=Mock(side_effect=AssertionError('legacy RGB dependency'))
        self.m.core.confirmed=Mock(side_effect=AssertionError('legacy confirmation dependency'))
        self.m.core.request_hold=Mock(side_effect=AssertionError('alignment dependency'))
        self.m.core.request_start=Mock(side_effect=AssertionError('alignment dependency'))
    def update(self,follow=True):
        super().update(follow)
        if getattr(self.m,'direct_pass',False):
            # Legacy image observations have no authority in direct mode.
            self.m.core.obs=None;self.m.core.samples.clear()

class StraightRouteTests(unittest.TestCase):
    def data(self):
        s=Sim();s.f.p=(0.,0.,1.145)
        s.m.core.pose=None
        for _ in range(30):s.update()
        c=s.m.core.c;v=status(s);return s,c,v
    def route(self,s,c,v):return straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_fixed_altitude_heading_box_and_exit_clearance(self):
        s,c,v=self.data();p=self.route(s,c,v)
        self.assertEqual(p['endpoint'][2],s.f.p[2]);self.assertAlmostEqual(p['endpoint'][1],s.f.p[1])
        self.assertGreater(p['endpoint'][0],1.5);self.assertGreater(min(p['edge_clearance_after_budgets_m']),0)
        self.assertFalse(p['actual_output_permitted']);self.assertFalse(p['metric_accuracy_verified'])
    def test_high_low_lateral_or_back_facing_cannot_pass(self):
        for field,value in [('height',1.50),('lateral',.45),('heading',math.pi)]:
            s,c,v=self.data();pose=s.m.core.pose
            if field=='height':s.m.core.pose=replace(pose,p=(0.,0.,value))
            elif field=='lateral':s.m.core.pose=replace(pose,p=(0.,value,1.145))
            else:s.m.core.pose=replace(pose,q=(0.,0.,1.,0.))
            with self.subTest(field=field),self.assertRaises(ValueError):self.route(s,c,v)
    def test_rejected_route_records_edge_and_budget_evidence(self):
        s,c,v=self.data();d=PassageDecision()
        pose=replace(s.m.core.pose,p=(0.,.45,1.145))
        self.assertFalse(d.update(v,c,pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1))
        result=d.snapshot();e=result['geometry_diagnostics']
        self.assertTrue(result['final_rejection']);self.assertIsNone(result['plan'])
        self.assertEqual(e['pose_local_m'],list(pose.p));self.assertEqual(e['geometry_stamp'],v['metric_geometry']['source_stamp'])
        self.assertEqual(len(e['edges']),4)
        self.assertLessEqual(e['minimum_clearance_after_budget_m'],0)
        for edge in e['edges']:
            self.assertAlmostEqual(edge['raw_box_clearance_m']-edge['required_budget_m'],edge['clearance_after_budget_m'])
        # An unevaluated duplicate keeps the evidence's own timestamp intact.
        self.assertFalse(d.update(v,c,pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1))
        self.assertEqual(d.snapshot()['geometry_diagnostics'],e)

    def test_exit_fence_and_time_budget_rejected(self):
        for key,value in [('geofence_max_m',[1.3,3.,4.]),('max_traverse_time_s',1.)]:
            s,c,v=self.data();c=dict(c,**{key:value})
            with self.subTest(key=key),self.assertRaises(ValueError):self.route(s,c,v)
    def test_foreground_nonfinite_stale_and_epoch_metadata_rejected(self):
        s,c,v=self.data()
        for change in ('foreground','stale','corner','epoch'):
            x=copy.deepcopy(v)
            if change=='foreground':x['metric_geometry']['opening_evidence']['possible_opening']=False
            elif change=='stale':x['metric_geometry']['source_stamp']-=1
            elif change=='corner':x['metric_geometry']['corners_enu_m'][0][0]=float('nan')
            else:x['metric_geometry']['pose_epoch']=None
            with self.subTest(change=change),self.assertRaises(ValueError):self.route(s,c,x)
    def test_distinct_three_of_five_with_one_invalid_frame(self):
        s,c,v=self.data();d=PassageDecision();base=s.f.ros
        for i,valid in enumerate((True,False,True,True)):
            t=base+i*.2;x=copy.deepcopy(v);x['source_stamp']=t;x['metric_geometry']['source_stamp']=t
            if not valid:x['ready']=False
            p=replace(s.m.core.pose,stamp=t)
            ready=d.update(x,c,p,t,s.f.wall+i*.2,s.f.wall+i*.2,base-1)
            self.assertEqual(ready,i==3)
        self.assertEqual(d.snapshot()['safe_samples'],3)
    def test_current_bad_frame_cannot_be_outvoted_duplicates_not_counted(self):
        s,c,v=self.data();d=PassageDecision();base=s.f.ros
        for i in range(3):
            t=base+.2*i;x=copy.deepcopy(v);x['source_stamp']=t;x['metric_geometry']['source_stamp']=t
            d.update(x,c,replace(s.m.core.pose,stamp=t),t,s.f.wall+.2*i,s.f.wall+.2*i,base-1)
        self.assertIsNotNone(d.plan)
        self.assertFalse(d.update(x,c,s.m.core.pose,t,s.f.wall+.4,s.f.wall+.4,base-1))
        x['source_stamp']=t+.1;x['metric_geometry']['source_stamp']=t+.1;x['ready']=False
        self.assertFalse(d.update(x,c,replace(s.m.core.pose,stamp=t+.1),t+.1,s.f.wall+.5,s.f.wall+.5,base-1))
        self.assertIsNone(d.plan)
    def test_epoch_and_age_break_vote_reuse(self):
        s,c,v=self.data();d=PassageDecision();base=s.f.ros
        for i in range(2):
            t=base+i*.2;x=copy.deepcopy(v);x['source_stamp']=t;x['metric_geometry']['source_stamp']=t
            d.update(x,c,replace(s.m.core.pose,stamp=t),t,s.f.wall+i*.2,s.f.wall+i*.2,base-1)
        x=copy.deepcopy(v);t=base+.4;x['source_stamp']=t;x['metric_geometry'].update(source_stamp=t,pose_epoch=1)
        self.assertFalse(d.update(x,c,replace(s.m.core.pose,stamp=t),t,s.f.wall+.4,s.f.wall+.4,base-1));self.assertEqual(d.snapshot()['safe_samples'],1)
        t=base+2;x['source_stamp']=t;x['metric_geometry']['source_stamp']=t
        self.assertFalse(d.update(x,c,replace(s.m.core.pose,stamp=t),t,s.f.wall+2,s.f.wall+2,base-1));self.assertEqual(d.snapshot()['safe_samples'],1)
    def test_cross_track_violation_and_no_reverse(self):
        s,c,v=self.data();p=self.route(s,c,v);command=(.03,0.,1.145)
        candidate,_=advance_route(p,s.m.core.pose,command,c,.05,1)
        self.assertGreaterEqual(candidate[0],command[0])
        bad=replace(s.m.core.pose,p=(0.,c['clearance_tracking_budget_m']+.001,1.145))
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(p,bad,command,c,.05,1)

class StraightLifecycle(unittest.TestCase):
    def test_no_alignment_or_legacy_rgb_dependency_in_full_flow(self):
        s=StraightSim();s.start();s.until('PILOT_CONTROL')
        phases=[e['new'] for e in s.m.events]
        self.assertIn('PASS',phases);self.assertNotIn('ALIGN',phases)
        self.assertTrue(s.out['straight_pass_completed']);self.assertTrue(s.out['handover_confirmed'])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertGreater(s.f.p[0],1.5)
        s.m.core.request_hold.assert_not_called();s.m.core.target_fresh.assert_not_called()
    def test_path_frozen_after_frame_leaves_view(self):
        s=StraightSim();s.start();s.until('PASS');p=copy.deepcopy(s.m.direct_plan);s.target=False
        s.until('PILOT_CONTROL');self.assertEqual(s.m.direct_plan,p);self.assertTrue(s.out['straight_pass_completed'])
    def test_no_feasible_geometry_hands_over_without_pass(self):
        s=StraightSim(False);s.start();s.until('PILOT_CONTROL')
        self.assertNotIn('PASS',[e['new'] for e in s.m.events]);self.assertFalse(s.out['straight_pass_completed'])
    def test_real_input_loss_remains_fault_and_mode_exit_is_final(self):
        for cause in ('estimator','mode'):
            s=StraightSim();s.start();s.until('PASS')
            if cause=='estimator':s.healthy['estimator_ok']=False
            else:s.mode='POSCTL'
            s.step();self.assertEqual(s.m.phase,'FAULT' if cause=='estimator' else 'TAKEOVER')
            self.assertNotIn('LAND',s.actions);self.assertIsNone(s.out['position'])
    def test_missing_pass_permission_or_existing_alignment_exception_refused(self):
        s=StraightSim()
        with self.assertRaises(ValueError):Mission(dict(s.m.core.c,enable_traverse=False),s.options,True,'straight_pass')
        with self.assertRaises(ValueError):Mission(s.m.core.c,s.options,True,'straight_pass',alignment_commissioning=True)


class StraightSafetyRegression(unittest.TestCase):
    data=StraightRouteTests.data
    route=StraightRouteTests.route
    def test_budget_increase_can_turn_a_centerline_fit_into_rejection(self):
        s,c,v=self.data();self.route(s,c,v)
        c=dict(c,position_uncertainty_m=.09,clearance_margin_m=.15)
        with self.assertRaisesRegex(ValueError,'clip_window'):self.route(s,c,v)
    def test_stale_pose_and_current_nonfinite_status_cannot_commit(self):
        s,c,v=self.data();s.m.core.pose=replace(s.m.core.pose,stamp=s.f.ros-.2)
        with self.assertRaisesRegex(ValueError,'pose_stale'):self.route(s,c,v)
    def test_old_alignment_approval_does_not_construct_straight_motion(self):
        from test_alignment_scope import AlignmentScope
        x=AlignmentScope().config();x['control'].update(enable_traverse=True,pass_stage_approved=True)
        with self.assertRaises(ValueError):Mission(x['control'],x['mission'],True,'straight_pass')
    def test_actual_fence_violation_stops_during_route(self):
        s,c,v=self.data();plan=self.route(s,c,v)
        pose=replace(s.m.core.pose,p=(100.,0.,1.145))
        with self.assertRaises(ValueError):advance_route(plan,pose,plan['start'],c,.05,1.)
    def test_duplicate_statuses_cannot_form_three_votes(self):
        s,c,v=self.data();d=PassageDecision()
        for _ in range(20):self.assertFalse(d.update(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1))
        self.assertEqual(d.snapshot()['safe_samples'],1)
