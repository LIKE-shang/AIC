"""Analytic enclosure and bounded-correction regressions, never live flight."""
import copy,math,random,unittest
from dataclasses import replace
from window_pass.geometry import rotate,dot,norm,tilt,add,sub
from attitude_envelope import support,body_corners
from prepass_adjustment import settings,suggest,advance,validate_motion
from straight_pass import straight_route,RouteRejection
from test_straight_pass import StraightRouteTests,StraightSim
from mission import Mission

POLICY=dict(enabled=True,operator_authorized=True,operator_statement='User requested bounded measured left/right/up/down correction')

def euler(y,p,r):
    cy,sy=math.cos(y/2),math.sin(y/2);cp,sp=math.cos(p/2),math.sin(p/2);cr,sr=math.cos(r/2),math.sin(r/2)
    return (cy*cp*sr-sy*sp*cr,cy*sp*cr+sy*cp*sr,sy*cp*cr-cy*sp*sr,cy*cp*cr+sy*sp*sr)

class DirectionalEnvelope(unittest.TestCase):
    def test_contains_all_sampled_allowed_attitudes_and_corners(self):
        s,c,v=StraightRouteTests().data();rng=random.Random(23);heading=.73
        directions=[(1.,0.,0.),(0.,0.,1.),(0.,-1.,0.)]+[tuple(rng.uniform(-1,1) for _ in range(3)) for _ in range(14)]
        bounds=[support(c,heading,w) for w in directions];corners=body_corners(c)
        for iy in range(-4,5):
            for ip in range(-5,6):
                for ir in range(-5,6):
                    q=euler(heading+c['yaw_limit_rad']*iy/4,c['tilt_limit_rad']*ip/5,c['tilt_limit_rad']*ir/5)
                    if tilt(q)>c['tilt_limit_rad']+1e-10:continue
                    for corner in corners:
                        actual=rotate(q,corner)
                        for w,limit in zip(directions,bounds):self.assertLessEqual(dot(w,actual),limit+1e-10)
    def test_real_budgets_allow_two_cm_offset_without_deleting_reserves(self):
        s,c,v=StraightRouteTests().data();c=dict(c,position_uncertainty_m=.05,clearance_margin_m=.04,clearance_tracking_budget_m=.03,yaw_limit_rad=math.radians(5),tilt_limit_rad=math.radians(5))
        s.m.core.pose=replace(s.m.core.pose,p=(0.,.02,1.12))
        p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        self.assertGreater(min(p['edge_clearance_after_budgets_m']),0)
        self.assertEqual(p['envelope_method'],'directional_continuous_yaw_tilt_support')
        self.assertTrue(all(e['required_budget_m']>=.12 for e in p['edge_diagnostics']))
        self.assertTrue(all(e['attitude_inflation_m']>0 for e in p['edge_diagnostics']))
    def test_support_increases_with_limits_and_is_homogeneous(self):
        s,c,v=StraightRouteTests().data()
        for w in ((0,0,1),(0,0,-1),(0,1,0),(.2,-.7,.3)):
            self.assertAlmostEqual(support(c,.3,tuple(2*x for x in w)),2*support(c,.3,w),places=9)
            self.assertGreaterEqual(support(dict(c,yaw_limit_rad=.12,tilt_limit_rad=.12),.3,w),support(c,.3,w)-1e-10)

class CorrectionMath(unittest.TestCase):
    def data(self,lateral=.14,height=1.05):
        s,c,v=StraightRouteTests().data();c=dict(c,position_uncertainty_m=.05,clearance_margin_m=.04,clearance_tracking_budget_m=.03,yaw_limit_rad=math.radians(5),tilt_limit_rad=math.radians(5))
        for corner in v['metric_geometry']['corners_enu_m']:corner[1]+=lateral;corner[2]+=height-1.05
        v['metric_geometry']['center_enu_m']=[1.,lateral,height]
        return s,c,v
    def propose(self,s,c,v):
        return suggest(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1,0.,.8,(0.,0.,.245),.2,settings(POLICY))
    def test_left_right_and_up_down_have_correct_sign_without_forward_motion(self):
        for lateral,height,axis,sign in ((.14,1.05,1,1),(-.14,1.05,1,-1),(0.,1.18,2,1),(0.,.99,2,-1)):
            with self.subTest(lateral=lateral,height=height):
                s,c,v=self.data(lateral,height);p=self.propose(s,c,v)
                delta=sub(p['target'],p['start']);self.assertGreater(sign*delta[axis],0)
                self.assertAlmostEqual(delta[0],0);self.assertLessEqual(norm(delta),.20)
                self.assertLessEqual(abs(delta[2]),.15);self.assertFalse(p['actual_output_permitted'])
    def test_too_low_too_far_and_too_narrow_refuse_correction(self):
        for case in ('floor','far','narrow'):
            s,c,v=self.data(0.,.90) if case=='floor' else self.data(.6 if case=='far' else .14)
            if case=='narrow':
                center=v['metric_geometry']['center_enu_m'][1]
                for p in v['metric_geometry']['corners_enu_m']:p[1]=center+(p[1]-center)*(.93/1.05)
            with self.subTest(case=case),self.assertRaisesRegex(ValueError,'no_feasible'):self.propose(s,c,v)
    def test_motion_loses_geometry_or_epoch_or_clear_column_then_refuses(self):
        s,c,v=self.data();p=self.propose(s,c,v)
        for case in ('stale','epoch','moved','column'):
            x=copy.deepcopy(v);pose=s.m.core.pose;ros=s.f.ros
            if case=='stale':ros+=1.
            elif case=='epoch':x['metric_geometry']['pose_epoch']=1
            elif case=='moved':x['metric_geometry']['center_enu_m'][1]+=.1
            else:pose=replace(pose,p=(0.,.19,pose.p[2]))
            with self.subTest(case=case),self.assertRaises(ValueError):
                validate_motion(p,c,pose,x,ros,s.f.wall,s.f.wall,s.f.ros-1,0.,.8,(0.,0.,.245),.2,settings(POLICY))
    def test_step_speed_lead_and_timeout(self):
        s,c,v=self.data();p=self.propose(s,c,v);policy=settings(POLICY)
        command,done=advance(p,c,s.m.core.pose,tuple(p['start']),.05,0.,policy)
        self.assertLessEqual(norm(sub(command,p['start'])),.08*.05+1e-10);self.assertFalse(done)
        with self.assertRaisesRegex(ValueError,'timeout'):advance(p,c,s.m.core.pose,command,.05,8.,policy)
    def test_authorization_and_scope_limits_are_explicit(self):
        self.assertFalse(settings()['enabled'])
        for value in (dict(enabled=True),dict(POLICY,max_distance_m=.21),dict(POLICY,speed_mps=.09),dict(POLICY,settle_s=.1),dict(POLICY,extra=True)):
            with self.assertRaises(ValueError):settings(value)

class AdjustedSim(StraightSim):
    def __init__(self,lateral=.14,height=1.05):
        super().__init__();c=copy.deepcopy(self.m.core.c);c.update(position_uncertainty_m=.05,clearance_margin_m=.04,clearance_tracking_budget_m=.03,yaw_limit_rad=math.radians(5),tilt_limit_rad=math.radians(5))
        self.options.update(max_mission_time_s=75.)
        self.m=Mission(c,self.options,True,'straight_pass',adjustment_policy=POLICY);self.f.ctrl=self.m.core
        self.f.cfg['window_center_local_m']=[1.,lateral,height]
        for _ in range(30):self.update()

class CorrectionLifecycle(unittest.TestCase):
    def test_one_adjustment_then_new_frames_then_pass_then_observed_handover(self):
        s=AdjustedSim();s.start();s.until('ADJUST');self.assertEqual(s.m.adjust_count,1)
        start=s.f.p;self.assertIsNone(s.m.direct_plan);self.assertGreaterEqual(s.m.adjust_decision.snapshot()['safe_samples'],3)
        s.until('SEARCH');self.assertTrue(s.m.adjust_completed);self.assertIsNone(s.m.direct_plan)
        self.assertEqual(s.m.passage_decision.snapshot()['safe_samples'],0);self.assertAlmostEqual(s.f.p[0],start[0])
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed']);self.assertTrue(s.out['handover_confirmed'])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertEqual([x['new'] for x in s.m.events].count('ADJUST'),1)
    def test_already_fits_passes_without_adjustment(self):
        s=AdjustedSim(0.);s.start();s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed']);self.assertEqual(s.m.adjust_count,0)
    def test_unreachable_refuses_before_adjust_or_pass(self):
        s=AdjustedSim(.6);s.start();s.until('PILOT_CONTROL')
        self.assertEqual(s.m.adjust_count,0);self.assertIsNone(s.m.direct_plan);self.assertFalse(s.out['straight_pass_completed'])
    def test_new_rejection_after_one_adjustment_does_not_loop(self):
        s=AdjustedSim();s.start();s.until('ADJUST');s.until('SEARCH');s.f.cfg['window_center_local_m'][1]=-.4
        s.until('PILOT_CONTROL');self.assertEqual(s.m.adjust_count,1);self.assertFalse(s.out['straight_pass_completed'])
    def test_geometry_loss_during_adjust_hands_over_without_forward_motion(self):
        s=AdjustedSim();s.start();s.until('ADJUST');x=s.f.p[0];s.target=False
        s.until('PILOT_CONTROL');self.assertFalse(s.out['straight_pass_completed']);self.assertAlmostEqual(s.f.p[0],x)
    def test_rc_loss_or_pilot_mode_exit_during_adjust_remains_final(self):
        for case in ('rc','mode'):
            s=AdjustedSim();s.start();s.until('ADJUST')
            if case=='rc':s.healthy['rc_ok']=False
            else:s.mode='POSCTL'
            s.step();self.assertEqual(s.m.phase,'FAULT' if case=='rc' else 'TAKEOVER')
            self.assertIsNone(s.out['position']);self.assertNotIn('LAND',s.actions)
    def test_total_feedback_path_limit_hands_over_without_second_correction(self):
        s=AdjustedSim();s.start();s.until('ADJUST');s.m.adjust_path_m=.301
        s.until('PILOT_CONTROL');self.assertIn('adjustment_total_path_limit',[e['reason'].split('_pilot_landing_required')[0] for e in s.m.events])
        self.assertEqual(s.m.adjust_count,1);self.assertFalse(s.out['straight_pass_completed'])
    def test_stuck_adjustment_times_out_to_pilot(self):
        s=AdjustedSim();s.start();s.until('ADJUST');s.update(follow=False);s.m.core.pose_speed=0.
        for _ in range(200):
            s.step(follow=False)
            if s.m.phase=='PILOT_CONTROL':break
        self.assertEqual(s.m.phase,'PILOT_CONTROL');self.assertFalse(s.out['straight_pass_completed'])

class AdjustmentScope(unittest.TestCase):
    def test_scope_requires_declared_adjustment_behavior_and_keeps_false_flags(self):
        from test_straight_scope import StraightScope
        from straight_pass_policy import validate,ACCEPTANCE_DEFERRALS
        cfg=StraightScope().config();cfg['prepass_adjustment']=dict(POLICY)
        with self.assertRaisesRegex(ValueError,'route declaration'):validate(cfg,True)
        cfg['route_policy'].update(mode='bounded_adjustment_then_fixed_height_heading',no_alignment=False,no_legacy_alignment=True)
        before=copy.deepcopy(cfg)
        self.assertEqual(validate(cfg,True),ACCEPTANCE_DEFERRALS);self.assertEqual(cfg,before)
        cfg['prepass_adjustment']['operator_authorized']=False
        with self.assertRaises(ValueError):validate(cfg,True)
    def test_other_modes_cannot_enable_adjustment(self):
        from test_mission import Sim
        s=Sim()
        with self.assertRaisesRegex(ValueError,'straight-pass-only'):
            Mission(s.m.core.c,s.options,adjustment_policy=POLICY)
