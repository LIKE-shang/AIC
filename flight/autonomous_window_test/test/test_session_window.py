"""v24 tests are synthetic only: no ROS, Nano, arming, or live flight."""
import copy,json,math,random,unittest,threading,types
from pathlib import Path
from dataclasses import replace
from unittest.mock import Mock,patch
from test_straight_pass import StraightSim,StraightRouteTests
from test_straight_scope import StraightScope
from test_prepass_adjustment import euler
from test_auto_transport import app
from mission import Mission
from session_window import settings,decorate,Acquisition,localization_identity
from session_localization import Continuity
from attitude_envelope import support,nominal_support,cylinder_axis_support
from straight_pass import straight_route
from window_pass.geometry import rotate,dot
from window_pass.core import Pose

PROFILE=dict(enabled=True,horizontal_radius_m=.31,rigid_clearance_m=.04)
IDENTITY={'boot_id':'synthetic_boot','run_id':'synthetic_run','/mavros/state':['/mavros','uri1'],
    '/window_pass_lio/Odometry':['/window_pass_lio/laserMapping','uri2'],
    '/window_pass_competition/status':['/bridge','uri3']}

class SessionSim(StraightSim):
    def __init__(self,lateral=.10,height=1.05):
        super().__init__()
        self.ident=copy.deepcopy(IDENTITY);self.local_epoch=0
        c=copy.deepcopy(self.m.core.c)
        c.update(position_uncertainty_m=.05,clearance_tracking_budget_m=.03,clearance_margin_m=.04,
            yaw_limit_rad=math.radians(5),tilt_limit_rad=math.radians(5))
        self.options.update(simplified_takeoff_enabled=True,vendor_takeoff_arrival_enabled=True,max_mission_time_s=75.)
        self.m=Mission(c,self.options,True,'straight_pass',session_window_policy=PROFILE,session_identity=IDENTITY)
        self.f.ctrl=self.m.core;self.f.cfg['window_center_local_m']=[1.,lateral,height]
        for _ in range(30):self.update()
    def inputs(self):
        i=super().inputs();i.localization_identity=copy.deepcopy(self.ident);i.localization_epoch=self.local_epoch;return i

class SessionFlow(unittest.TestCase):
    def test_one_centering_even_when_current_ray_fits_and_frozen_pass_completes(self):
        s=SessionSim();s.start();s.until('ADJUST')
        snap=copy.deepcopy(s.m.session_window.snapshot)
        self.assertGreaterEqual(s.m.session_window.decision.snapshot()['safe_samples'],3)
        self.assertAlmostEqual(s.m.adjust_plan['target'][1],.10)
        self.assertAlmostEqual(s.m.adjust_plan['target'][2],s.m.target_z)
        self.assertIsNone(s.m.direct_plan)
        s.until('PASS');plan=copy.deepcopy(s.m.direct_plan)
        s.until('PILOT_CONTROL')
        self.assertEqual(s.m.session_window.snapshot,snap);self.assertEqual(s.m.direct_plan,plan)
        self.assertEqual([e['new'] for e in s.m.events].count('SEARCH'),1)
        self.assertEqual([e['new'] for e in s.m.events].count('ADJUST'),1)
        self.assertTrue(s.out['straight_pass_completed']);self.assertTrue(s.out['alignment_completed'])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertTrue(s.armed)
    def test_camera_loss_after_acquisition_does_not_reacquire_or_abort(self):
        s=SessionSim();s.start();s.until('ADJUST');snap=copy.deepcopy(s.m.session_window.snapshot)
        s.target=False;s.m.core.metric_input=None
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed']);self.assertEqual(s.m.session_window.snapshot,snap)
    def test_new_window_data_after_commit_cannot_retarget(self):
        s=SessionSim();s.start();s.until('PASS');plan=copy.deepcopy(s.m.direct_plan)
        s.f.cfg['window_center_local_m']=[-10.,9.,8.]
        s.until('PILOT_CONTROL');self.assertEqual(s.m.direct_plan,plan)
    def test_session_epoch_and_sources_invalidate_during_adjust_and_pass(self):
        for phase in ('ADJUST','PASS'):
            for cause in ('boot','run','source','epoch'):
                s=SessionSim();s.start();s.until(phase)
                if cause=='boot':s.ident['boot_id']='new'
                elif cause=='run':s.ident['run_id']='new'
                elif cause=='source':s.ident['/window_pass_lio/Odometry'][1]='new_uri'
                else:s.local_epoch+=1
                s.step();self.assertEqual(s.m.phase,'FAULT',(phase,cause));self.assertIsNone(s.out['position'])
                self.assertIsNotNone(s.out['session_window']['invalidated']);self.assertNotIn('LAND',s.actions)
    def test_invalid_pose_pulse_cannot_recover_into_route(self):
        s=SessionSim();s.start();s.until('PASS');p=s.m.core.pose
        s.m.core.pose_update(replace(p,p=(99.,0.,1.)),s.f.ros,s.f.wall)
        s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.out['position'])
    def test_missing_geometry_times_out_without_alignment_or_pass(self):
        s=SessionSim();s.target=False;s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.out['straight_pass_completed']);self.assertIsNone(s.m.session_window.snapshot)
        self.assertNotIn('ADJUST',[e['new'] for e in s.m.events])
    def test_unreachable_center_is_not_silently_replaced_with_current_ray(self):
        s=SessionSim(.40);s.start();s.until('PILOT_CONTROL')
        self.assertIsNone(s.m.direct_plan);self.assertIsNone(s.m.session_window.snapshot)
    def test_zero_translation_does_not_divide_by_zero(self):
        s=SessionSim(0.);s.start();s.until('SEARCH')
        # Set the measured centre to exactly the level body's actual centre.
        s.f.p=(0.,0.,1.12);s.m.position=s.f.p;s.out=dict(s.out,position=s.f.p);s.m.core.pose=None;s.update()
        s.f.cfg['window_center_local_m'][2]=s.f.p[2]-.07
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
    def test_stalled_centering_times_out_and_hands_over(self):
        s=SessionSim();s.start();s.until('ADJUST')
        for _ in range(200):
            s.step(follow=False)
            if s.m.phase=='PILOT_CONTROL':break
        self.assertEqual(s.m.phase,'PILOT_CONTROL');self.assertFalse(s.out['straight_pass_completed'])
    def test_posctl_service_acceptance_alone_does_not_stop_output(self):
        s=SessionSim();s.start();s.until('PASS')
        for _ in range(800):
            s.step(respond=False)
            if s.m.phase=='HANDOVER_REQUEST':break
        self.assertEqual(s.m.phase,'HANDOVER_REQUEST');self.assertIsNotNone(s.out['position'])
        s.m.command_finished('POSCTL',True);s.step(respond=False)
        self.assertEqual(s.m.phase,'HANDOVER_REQUEST');self.assertFalse(s.out['handover_confirmed'])
        s.mode='POSCTL';s.step();self.assertEqual(s.m.phase,'PILOT_CONTROL');self.assertIsNone(s.out['position'])
    def test_real_rc_and_estimator_failure_remain_fault(self):
        for flag in ('rc_ok','estimator_ok'):
            s=SessionSim();s.start();s.until('PASS');s.healthy[flag]=False;s.step()
            self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('LAND',s.actions)

class SimplifiedTakeoff(unittest.TestCase):
    def test_full_height_goal_no_precision_or_extra_ascent_gates(self):
        s=SessionSim();s.start();s.until('TAKEOFF');m=s.m
        self.assertEqual(m.position,(m.origin[0],m.origin[1],m.target_z))
        # Direct state injections isolate the removed tracking/attitude gates.
        p=m.core.pose
        m.core.pose=replace(p,p=(.31,0.,m.target_z+.21),q=euler(math.radians(31),math.radians(16),0))
        m.core.pose_speed=.12
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'TAKEOFF',out['reason'])
        self.assertFalse(out['attitude_checks_enabled']);self.assertIsNone(out['tracking']['horizontal_tracking_limit_m'])
    def test_vendor_arrival_accepts_speed_and_height_error_without_stable_wait(self):
        s=SessionSim();s.start();s.until('TAKEOFF');m=s.m
        m.arm_request_wall=s.f.wall-3.01;m.core.airborne=True
        m.core.pose=replace(m.core.pose,p=(.13,0.,m.target_z-.06));m.core.pose_speed=.08
        self.assertEqual(m.tick(s.inputs())['phase'],'SEARCH')
        self.assertIsNone(m.stable_since);self.assertEqual(m.position[:2],m.core.pose.p[:2]);self.assertEqual(m.position[2],m.target_z)
        self.assertEqual(m.takeoff_progress['required_continuous_stable_s'],0.)
    def test_takeoff_fence_and_input_faults_retained(self):
        for cause in ('fence','estimator'):
            s=SessionSim();s.start();s.until('TAKEOFF')
            if cause=='fence':s.m.core.pose=replace(s.m.core.pose,p=(99.,0.,1.))
            else:s.healthy['estimator_ok']=False
            self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')
    def test_armed_feedback_does_not_reapply_old_three_dimensional_limit(self):
        s=SessionSim();s.start();s.until('ARM');s.m.core.armed=True
        s.m.core.pose=replace(s.m.core.pose,p=(.13,0.,.245))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'TAKEOFF')
    def test_large_residual_tilt_after_arrival_requests_observed_handover(self):
        s=SessionSim();s.start();s.until('SEARCH')
        s.m.core.pose=replace(s.m.core.pose,q=euler(0.,math.radians(7),0.))
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST');self.assertEqual(out['action'],'POSCTL')
        self.assertIsNotNone(out['position']);s.m.core.mode='POSCTL'
        self.assertEqual(s.m.tick(s.inputs())['phase'],'PILOT_CONTROL')
    def test_minimum_floor_and_armed_time_still_required(self):
        s=SessionSim();s.start();s.until('TAKEOFF');m=s.m
        m.arm_request_wall=s.f.wall-4.;m.core.airborne=True
        m.core.pose=replace(m.core.pose,p=(0.,0.,1.035))
        self.assertFalse(m.takeoff_arrival(s.inputs()))
        m.core.pose=replace(m.core.pose,p=(0.,0.,m.target_z));m.arm_request_wall=s.f.wall-2.99
        self.assertFalse(m.takeoff_arrival(s.inputs()))

class CylinderEnvelope(unittest.TestCase):
    def data(self):
        s,c,v=StraightRouteTests().data();return s,decorate(dict(c),PROFILE),v
    def test_horizontal_radius_is_not_sphere_or_box_diagonal(self):
        s,c,v=self.data()
        for w in ((1.,0.,0.),(0.,1.,0.),(2**-.5,2**-.5,0.)):
            self.assertAlmostEqual(nominal_support(c,0,w),.31)
        self.assertAlmostEqual(nominal_support(c,0,(0,0,-1)),.245)
        self.assertAlmostEqual(nominal_support(c,0,(0,0,1)),.105)
    def test_continuous_tilt_bound_contains_sampled_cylinder_surfaces(self):
        s,c,v=self.data();c['tilt_limit_rad']=math.radians(5);rng=random.Random(24)
        for _ in range(300):
            heading=rng.uniform(-math.pi,math.pi);angle=rng.uniform(0,2*math.pi)
            q=euler(heading,rng.uniform(-c['tilt_limit_rad'],c['tilt_limit_rad']),0.)
            w=tuple(rng.uniform(-1,1) for _ in range(3))
            for z in (-.245,.105):
                point=(.31*math.cos(angle),.31*math.sin(angle),z)
                self.assertLessEqual(dot(w,rotate(q,point)),support(c,heading,w)+1e-10)
    def test_no_duplicate_spatial_budget_and_tilt_not_erased(self):
        s,c,v=self.data();c.update(position_uncertainty_m=.05,clearance_tracking_budget_m=.03,tilt_limit_rad=math.radians(5))
        p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        self.assertEqual(p['spatial_error_budget_m'],0.);self.assertFalse(p['clearance_is_accuracy_bound'])
        for edge in p['edge_diagnostics']:
            self.assertEqual(edge['position_and_tracking_projection_m'],0.)
            self.assertAlmostEqual(edge['required_budget_m'],.04+edge['attitude_inflation_m'])
        self.assertGreater(support(c,0,(0,0,-1)),.245)
    def test_actual_tilt_envelope_and_height_match(self):
        from envelope_fence import envelope_margins
        s,c,v=self.data();q=euler(.6,.08,-.01);axis=rotate(q,(0.,0.,1.))
        result=envelope_margins(c,(0,0,1.12),q)
        self.assertAlmostEqual(result['lower_m'][2],1.12-cylinder_axis_support(c,(0,0,-1),axis)-c['geofence_min_m'][2]-.04)

class ContinuityTests(unittest.TestCase):
    def guard(self):
        _,c,_=StraightRouteTests().data();return Continuity(c)
    def pose(self,t=10.,x=0.):return Pose(t,(x,0.,0.),(0.,0.,0.,1.),'camera_init','body')
    def bridge(self,n=5):return dict(real_published=n,real_fault='',real_output_state='FORWARDING_CONTINUOUS_POSE',horizontal_flags_valid=True)
    def test_valid_continuity_needs_raw_lio_and_bridge(self):
        g=self.guard();g.observe(self.pose(),1,10.,20.,'/lio','/lio')
        self.assertFalse(g.health(10.,20.)[0]);g.observe_bridge(self.bridge(),20.,'/bridge','/bridge')
        self.assertTrue(g.health(10.,20.)[0]);self.assertFalse(g.health(10.21,20.21)[0])
    def test_sequence_jump_frame_and_source_failures_latch_new_epoch(self):
        for cause in ('seq','jump','frame','source','clock','gap'):
            g=self.guard();g.observe(self.pose(),10,10.,20.,'/lio','/lio')
            p=self.pose(10.1,1. if cause=='jump' else 0.)
            if cause=='frame':p=replace(p,frame='map')
            if cause=='clock':p=replace(p,stamp=9.9)
            if cause=='gap':p=replace(p,stamp=10.6)
            g.observe(p,1 if cause=='seq' else 11,p.stamp,20.1,'wrong' if cause=='source' else '/lio','/lio')
            self.assertEqual(g.epoch,1,cause);self.assertFalse(g.health(p.stamp,20.1)[0])
            g.observe(self.pose(10.2),12,10.2,20.2,'/lio','/lio');self.assertEqual(g.epoch,1)
    def test_bridge_restart_or_fault_never_silently_recovers(self):
        for case in ('count','fault','state','horizontal'):
            g=self.guard();g.observe_bridge(self.bridge(),20.,'/bridge','/bridge');v=self.bridge(6)
            if case=='count':v['real_published']=1
            elif case=='fault':v['real_fault']='reset'
            elif case=='state':v['real_output_state']='STOPPED'
            else:v['horizontal_flags_valid']=False
            g.observe_bridge(v,20.1,'/bridge','/bridge');self.assertEqual(g.epoch,1)
            g.observe_bridge(self.bridge(7),20.2,'/bridge','/bridge');self.assertFalse(g.health(10.,20.2)[0])

class ProfileAndTransport(unittest.TestCase):
    def test_profile_default_off_and_requires_explicit_model(self):
        self.assertFalse(settings()['enabled'])
        for p in ({'enabled':True},dict(PROFILE,horizontal_radius_m=.35),dict(PROFILE,rigid_clearance_m=0.)):
            with self.assertRaises(ValueError):settings(p)
        with self.assertRaises(ValueError):Acquisition({'boot_id':'x'})
    def test_old_scope_cannot_enable_new_profile(self):
        from straight_pass_policy import validate
        c=StraightScope().config();c['session_window']=PROFILE;c['mission']['simplified_takeoff_enabled']=True
        with self.assertRaises(ValueError):validate(c,True)
        c['straight_pass_authorization']['scope']='supervised_session_window_pass'
        c['route_policy'].update(mode='one_acquisition_center_then_session_frozen_line',no_alignment=False,post_commit_visual_dependency=False,localization_binding='boot_run_source_epoch')
        before=copy.deepcopy(c);self.assertTrue(validate(c,True));self.assertEqual(c,before)
    def test_template_disabled_unbound_and_does_not_claim_precision(self):
        c=json.loads((Path(__file__).resolve().parents[1]/'config/session_window.json').read_text(encoding='utf-8'))
        self.assertIsNone(c['site_identity'])
        for key in ('enable_motion','enable_traverse','pass_stage_approved'):self.assertFalse(c['control'][key])
        for key in ('body_origin_verified','ground_height_verified','rgbd_geometry_verified','simulation_verified'):self.assertFalse(c['acceptance'][key])
        self.assertEqual(c['control']['geofence_min_m'],[])
    def test_visual_graph_is_required_only_until_acquisition(self):
        s=SessionSim();r=app.Runtime.__new__(app.Runtime);r.mission=s.m
        self.assertTrue(r.visual_required());s.start();s.until('ADJUST');self.assertFalse(r.visual_required())
        s.until('PASS');self.assertFalse(r.visual_required())
    def test_localization_identity_excludes_perception_but_keeps_boot_run_and_sources(self):
        full=dict(IDENTITY,**{'/window_live/status':['/vision','camera_uri']})
        self.assertEqual(localization_identity(full),IDENTITY)
        r=app.Runtime.__new__(app.Runtime);r.airborne_geometry=True;r.passive_rgbd=False
        r.master=Mock();r.master.getSystemState.return_value=([ [k,v[:1]] for k,v in IDENTITY.items() if k.startswith('/') ],[],[])
        r.master.lookupNode.side_effect=lambda name:next(v[1] for k,v in IDENTITY.items() if k.startswith('/') and v[0]==name)
        r.ros=Mock();r.ros.get_param.return_value=IDENTITY['run_id']
        with patch.object(app,'Path') as P:
            P.return_value.read_text.return_value=IDENTITY['boot_id']
            self.assertEqual(r.identity(include_perception=False),IDENTITY)
            with self.assertRaisesRegex(ValueError,'localization_source_not_unique'):r.identity()


class AdditionalSessionRegressions(unittest.TestCase):
    def test_invalid_acquisition_frames_count_toward_latest_five(self):
        a=Acquisition(IDENTITY)
        for n in range(6):a.reject_frame({'source_stamp':10.+n*.1},10.+n*.1,'invalid')
        self.assertEqual(len(a.decision.rows),5);self.assertFalse(any(x['safe'] for x in a.decision.rows))
        a.reject_frame({'source_stamp':10.5},10.5,'duplicate');self.assertEqual(len(a.decision.rows),5)
    def test_graph_worker_accepts_camera_disappearance_after_freeze_only(self):
        s=SessionSim();s.start();s.until('PASS')
        r=app.Runtime.__new__(app.Runtime);r.mission=s.m;r.core=s.m.core;r.control=s.m.core.c
        r.airborne_geometry=True;r.passive_rgbd=False;r.direct_pass=True;r.takeoff_only=False
        r.cfg={'mission':{'companion_battery_monitor_enabled':False}}
        r.source_identity=dict(IDENTITY,**{'/window_live/status':['/vision','camera_uri']})
        r.localization=Mock();r.lock=threading.RLock();r.execute=False;r.motion=object()
        r.stop=Mock();r.stop.is_set.side_effect=[False,True]
        r.ros=Mock();r.ros.get_name.return_value='/supervisor';r.ros.get_param.return_value=False
        pubs=[['/mavros/rc/in',['/mavros']],['/mavros/estimator_status',['/mavros']]]
        r.master=Mock();r.master.getSystemState.return_value=(pubs,[],[])
        r.identity=Mock(return_value=IDENTITY)
        with patch.object(app,'check_takeoff_graph',return_value=(True,'ok')):r.graph_worker()
        self.assertTrue(r.graph_ok,r.graph_reason);r.identity.assert_called_once_with(include_perception=False)
        r.localization.invalidate.assert_not_called()


class StaticBudgetRegression(unittest.TestCase):
    def test_old_box_budget_cannot_block_new_cylinder_before_live_geometry(self):
        from live_geometry import flight_blockers
        s=SessionSim();c={k:v for k,v in s.m.core.c.items() if not k.startswith('_')}
        c.update(opening_size_m=[.85,.8],position_uncertainty_m=.09)
        self.assertIn('insufficient:worst_case_clearance',flight_blockers(c))
        self.assertNotIn('insufficient:worst_case_clearance',flight_blockers(c,session_window=True))
        m=Mission(c,s.options,True,'straight_pass',session_window_policy=PROFILE,session_identity=IDENTITY)
        self.assertIsNone(m.direct_plan)
        self.assertIn('unverified:camera_verified',flight_blockers(dict(c,camera_verified=False),session_window=True))
    def test_committed_route_keeps_capture_and_departure_timestamps_distinct(self):
        s=SessionSim();s.start();s.until('ADJUST');captured=s.m.session_window.snapshot['geometry']['source_stamp']
        s.until('PASS');p=s.m.direct_plan
        self.assertEqual(p['geometry_stamp'],captured);self.assertGreater(p['departure_pose_stamp'],captured)


class HeightReferenceRegression(unittest.TestCase):
    def test_recorded_early_arrival_does_not_lower_height_reference(self):
        s=SessionSim();s.start();s.until('TAKEOFF');m=s.m
        m.arm_request_wall=s.f.wall-3.1;m.core.airborne=True
        m.core.pose=replace(m.core.pose,p=(.03,.01,m.target_z-.0897224523),q=euler(0.,.02,0.))
        m.core.pose_speed=.08
        out=m.tick(s.inputs())
        self.assertEqual(out['phase'],'SEARCH');self.assertAlmostEqual(out['position'][2],m.target_z)
        self.assertGreater(out['height']['actual_floor_margin_m'],0.)
        self.assertGreater(out['height']['planned_bottom_height_m'],.89)
        m.core.metric_input={};out=m.tick(s.inputs())
        self.assertEqual(out['phase'],'SEARCH');self.assertAlmostEqual(out['position'][2],m.target_z)
    def test_actual_floor_violation_retained_and_trigger_saved_before_hold(self):
        s=SessionSim();s.start();s.until('SEARCH');m=s.m
        original=m.position;m.core.pose=replace(m.core.pose,p=(0.,0.,m.ground_z+.245+.799),q=(0.,0.,0.,1.))
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST')
        snapshot=out['handover_snapshot']
        self.assertEqual(snapshot['reason'],'session_search_height_floor')
        self.assertAlmostEqual(snapshot['height']['actual_bottom_height_m'],.799)
        self.assertEqual(snapshot['tracking']['target_position_local_m'],original)
        self.assertEqual(out['action'],'POSCTL');self.assertIsNotNone(out['position'])
    def test_measured_center_below_90cm_is_allowed_above_80cm(self):
        s=SessionSim(height=.85);s.start();s.until('ADJUST')
        bottom=s.m.adjust_plan['target'][2]-.245-s.m.ground_z
        self.assertLess(bottom,.90);self.assertGreaterEqual(bottom,.80)
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])


class FeasibleApertureRegression(unittest.TestCase):
    def test_low_center_selects_upper_aperture_without_lowering_floor(self):
        s=SessionSim(.10,.95);s.start();s.until('ADJUST')
        e=s.m.session_window.candidate_evidence
        self.assertLess(e['exact_center_worst_bottom_height_m'],.8)
        self.assertGreaterEqual(e['selected_worst_bottom_height_m'],.8-1e-10)
        self.assertGreater(e['selected_target_local_m'][2],e['exact_center_target_local_m'][2])
        self.assertGreater(min(x['clearance_after_budget_m'] for x in e['selected_edges']),0.)
        s.target=False;s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
        self.assertEqual(s.m.o['minimum_bottom_height_m'],.8)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
    def test_center_beyond_translation_limit_can_use_inside_aperture(self):
        s=SessionSim(.25,1.05);s.start();s.until('ADJUST')
        e=s.m.session_window.candidate_evidence
        self.assertGreater(e['exact_center_delta_m'][1],.20)
        self.assertLessEqual(s.m.adjust_plan['length_m'],.20+1e-10)
        self.assertLess(e['selected_target_local_m'][1],.25)
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
    def test_window_too_low_is_refused_with_replayable_geometry(self):
        s=SessionSim(.0,.65);s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.out['straight_pass_completed']);self.assertIsNone(s.m.direct_plan)
        e=s.out['session_window']['candidate_evidence']
        self.assertEqual(len(e['geometry']['corners_enu_m']),4)
        self.assertEqual(e['minimum_bottom_height_m'],.8)
        self.assertEqual(e['reason'],'no_feasible_aperture_target_within_floor_and_scope')
        self.assertTrue(s.out['session_window']['candidate_history'])
        self.assertLessEqual(len(s.out['session_window']['candidate_history']),40)
    def test_coupled_floor_and_translation_disk_intersection(self):
        from prepass_adjustment import bounded_candidates
        # Closest point to the preference on a disk + y>=.6 is their intersection.
        constraints=[(0.,1.,.6)];preferred=(2.,0.)
        candidates=[p for p in bounded_candidates(constraints,preferred,1.)
                    if p[1]>=.6-1e-10 and math.hypot(*p)<=1.+1e-10]
        chosen=min(candidates,key=lambda p:math.hypot(p[0]-2.,p[1]))
        self.assertAlmostEqual(chosen[0],.8);self.assertAlmostEqual(chosen[1],.6)
    def test_duplicates_do_not_replan_or_add_vote_history(self):
        s=SessionSim(.10,.95);s.start();s.until('SEARCH')
        for _ in range(20):
            s.step()
            if s.m.session_window.candidate_evidence is not None:break
        a=s.m.session_window;before=len(a.candidate_history);rows=len(a.decision.rows)
        if a.snapshot is None:
            with patch('session_window.select_target',side_effect=AssertionError('duplicate recomputation')):
                a.update(s.m.core.metric_input,s.m.core.c,s.m.core.pose,s.f.ros,s.f.wall,s.m.core.metric_received,
                    s.m.search_started_ros,s.m.ground_z,.8,s.m.origin,.2)
            self.assertEqual(len(a.candidate_history),before);self.assertEqual(len(a.decision.rows),rows)
    def test_choice_respects_each_edge_after_upward_offset(self):
        s=SessionSim(.10,.95);s.start();s.until('PASS')
        p=s.m.direct_plan
        self.assertEqual(p['spatial_error_budget_m'],0.)
        self.assertGreater(min(p['edge_clearance_after_budgets_m']),0.)
        self.assertEqual(p['geometry_source'],'immutable_session_snapshot')
        self.assertEqual(s.m.adjust_count,1)


EXPANDED=dict(PROFILE,alignment_max_distance_m=.50,alignment_clear_column_radius_m=.60)

class ExpandedSim(SessionSim):
    def __init__(self,lateral=.40):
        super().__init__(lateral)
        c={k:v for k,v in self.m.core.c.items() if not k.startswith('_')}
        c.update(max_align_distance_m=.5,max_align_path_m=.75,max_align_time_s=15.)
        self.m=Mission(c,self.options,True,'straight_pass',session_window_policy=EXPANDED,session_identity=IDENTITY)
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()

class ExpandedAlignmentRegression(unittest.TestCase):
    def test_40cm_correction_then_frozen_pass_and_observed_handover(self):
        s=ExpandedSim();s.start();s.until('ADJUST')
        self.assertGreater(s.m.adjust_plan['length_m'],.39);self.assertLessEqual(s.m.adjust_plan['length_m'],.5)
        self.assertAlmostEqual(s.m.adjust_plan['target'][1],.40)
        self.assertEqual(s.out['session_window']['alignment_policy']['timeout_s'],15.)
        self.assertEqual(s.out['prepass_adjustment']['policy']['max_distance_m'],.5)
        self.assertTrue(s.out['prepass_adjustment']['policy']['enabled'])
        s.target=False;s.until('PILOT_CONTROL')
        self.assertTrue(s.out['straight_pass_completed']);self.assertEqual(s.m.adjust_count,1)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
    def test_beyond_scope_does_not_invent_a_longer_route(self):
        s=ExpandedSim(.9);s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.out['straight_pass_completed']);self.assertIsNone(s.m.direct_plan)
    def test_search_to_adjust_keeps_ongoing_climb_command(self):
        s=ExpandedSim(.1);s.start();s.until('SEARCH');previous=s.m.position
        # Feed a stable early-arrival height while new frames authorize geometry.
        s.f.p=(s.f.p[0],s.f.p[1],s.m.target_z-.08)
        s.m.core.pose=None
        for _ in range(30):
            s.step(follow=False)
            if s.m.phase=='ADJUST':break
        self.assertEqual(s.m.phase,'ADJUST')
        self.assertEqual(s.m.position,previous)
        self.assertGreater(s.m.position[2]-s.m.core.pose.p[2],.07)
        s.step(follow=False)
        self.assertEqual(s.m.phase,'ADJUST');self.assertAlmostEqual(s.m.position[2],previous[2])
    def test_adjustment_uses_target_tracking_not_3cm_ray_error(self):
        from session_window import advance_alignment
        s=ExpandedSim();c=s.m.core.c;policy=s.m.session_alignment_policy
        pose=replace(s.m.core.pose,p=(.024,-.009,1.145))
        plan=dict(target=[0.,-.07,1.145],yaw_rad=0.)
        command=(0.,-.048,1.145)
        target,arrived=advance_alignment(plan,c,pose,command,.05,.8,policy)
        self.assertLessEqual(math.dist(command,target),.08*.05+1e-10)
        self.assertFalse(arrived)
        with self.assertRaisesRegex(ValueError,'tracking'):
            advance_alignment(plan,c,replace(pose,p=(.2,0.,1.145)),command,.05,.8,policy)
    def test_pass_cross_track_3cm_is_unchanged(self):
        from straight_pass import advance_route
        s=ExpandedSim();s.start();s.until('PASS');p=s.m.core.pose
        bad=replace(p,p=(p.p[0],p.p[1]+.031,p.p[2]))
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(s.m.direct_plan,bad,s.m.position,s.m.core.c,.05,0.)
    def test_expanded_area_requires_matching_physical_confirmation(self):
        from straight_pass_policy import validate
        c=StraightScope().config();c['session_window']=EXPANDED;c['mission']['simplified_takeoff_enabled']=True
        c['control'].update(max_align_distance_m=.5,max_align_path_m=.75,max_align_time_s=15.)
        c['straight_pass_authorization']['scope']='supervised_session_window_pass'
        c['route_policy'].update(mode='one_acquisition_center_then_session_frozen_line',no_alignment=False,post_commit_visual_dependency=False,localization_binding='boot_run_source_epoch')
        with self.assertRaisesRegex(ValueError,'clear area'):validate(c,True)
        with self.assertRaisesRegex(ValueError,'clear area'):validate(c,False)
        c['alignment_area_confirmation']=dict(operator_confirmed_clear=True,center_path_radius_m=.2)
        with self.assertRaisesRegex(ValueError,'clear area'):validate(c,True)
        c['alignment_area_confirmation']['center_path_radius_m']=.6
        self.assertTrue(validate(c,True))
    def test_actual_floor_violation_still_stops_adjustment(self):
        s=ExpandedSim();s.start();s.until('ADJUST')
        s.m.core.pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],s.m.core.pose.p[1],s.m.ground_z+.245+.795))
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST')
        self.assertIn('height_floor',out['reason'])
    def test_expanded_profile_numeric_caps_and_pending_template(self):
        for key,value in [('alignment_max_distance_m',.51),('alignment_clear_column_radius_m',.61),('alignment_max_distance_m',float('nan'))]:
            with self.assertRaises(ValueError):settings(dict(EXPANDED,**{key:value}))
        c=json.loads((Path(__file__).resolve().parents[1]/'config/session_window.json').read_text(encoding='utf-8'))
        self.assertEqual(c['session_window']['alignment_max_distance_m'],.2)
        self.assertEqual(c['session_window']['alignment_clear_column_radius_m'],.2)
        self.assertEqual(c['session_window']['pass_cross_track_limit_m'],.08)
        from session_window import alignment_policy
        policy=alignment_policy(settings(c['session_window']))
        self.assertEqual(policy['max_path_m'],.3);self.assertEqual(policy['timeout_s'],8.)
        self.assertEqual(c['control']['max_align_distance_m'],.2)
        self.assertFalse(c['alignment_area_confirmation']['operator_confirmed_clear'])


class ForwardLeadRegression(unittest.TestCase):
    def route(self,cross_track=0.):
        s,c,v=StraightRouteTests().data();c=decorate(dict(c),settings(dict(PROFILE,faster_pass=True,pass_cross_track_limit_m=.08)))
        s.m.core.c=c
        p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        p['live_cross_track_abort_m']=.08
        pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],s.m.core.pose.p[1]+cross_track,s.m.core.pose.p[2]))
        return s,c,p,pose
    def test_forward_lead_12cm_at_zero_cross_track(self):
        from straight_pass import advance_route
        s,c,p,pose=self.route();command=tuple(p['start'])
        for i in range(40):command,_=advance_route(p,pose,command,c,.05,i*.05)
        self.assertAlmostEqual(command[0]-p['start'][0],.12,places=8)
        self.assertAlmostEqual(command[1]-p['start'][1],0.,places=8)
    def test_cross_track_consumes_same_12cm_tracking_vector(self):
        from straight_pass import advance_route
        s,c,p,pose=self.route(.08);command=tuple(p['start'])
        for i in range(40):command,_=advance_route(p,pose,command,c,.05,i*.05)
        self.assertAlmostEqual(command[0]-p['start'][0],math.sqrt(.12**2-.08**2),places=7)
        self.assertLessEqual(math.sqrt((command[0]-pose.p[0])**2+(command[1]-pose.p[1])**2),.12+1e-9)
    def test_lateral_clearance_gate_is_still_8cm(self):
        from straight_pass import advance_route
        s,c,p,pose=self.route(.081)
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(p,pose,tuple(p['start']),c,.05,0.)
    def test_faster_profile_is_12cm_but_normal_config_lead_remains_4cm(self):
        s,c,p,_=self.route();self.assertEqual(c['_session_forward_lead_m'],.12)
        self.assertEqual(c['max_setpoint_lead_m'],.04);self.assertEqual(p['maximum_forward_lead_m'],.12)

class PassageSlackRegression(unittest.TestCase):
    def data(self):
        s,c,v=StraightRouteTests().data();c=decorate(dict(c),settings(dict(PROFILE,pass_cross_track_limit_m=.08)))
        p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        return s,c,p
    def test_6cm_sideways_motion_allowed_when_each_edge_has_clearance(self):
        from straight_pass import advance_route
        s,c,p=self.data();pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],.06,s.m.core.pose.p[2]))
        target,done=advance_route(p,pose,tuple(p['start']),c,.05,.2)
        self.assertFalse(done);self.assertEqual(target[1],p['start'][1])
    def test_8cm_is_hard_cap_even_for_large_opening(self):
        from straight_pass import advance_route
        s,c,p=self.data();pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],.081,s.m.core.pose.p[2]))
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(p,pose,tuple(p['start']),c,.05,.2)
    def test_less_than_8cm_refused_when_specific_edge_clearance_is_consumed(self):
        from straight_pass import advance_route
        s,c,p=self.data();top=next(e for e in p['edge_diagnostics'] if e['side_in_window_basis']=='top')
        top['clearance_after_budget_m']=.04
        pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],0.,s.m.core.pose.p[2]+.05))
        with self.assertRaisesRegex(ValueError,'clearance_exhausted'):advance_route(p,pose,tuple(p['start']),c,.05,.2)
        # The opposite signed error moves away from this edge and is allowed.
        pose=replace(pose,p=(pose.p[0],0.,s.m.core.pose.p[2]-.05))
        advance_route(p,pose,tuple(p['start']),c,.05,.2)
    def test_along_route_progress_does_not_consume_aperture_slack(self):
        from straight_pass import advance_route
        s,c,p=self.data();pose=replace(s.m.core.pose,p=(p['start'][0]+.5,p['start'][1]+.06,p['start'][2]))
        advance_route(p,pose,tuple(p['start']),c,.05,5.)
    def test_missing_frozen_edges_and_overlarge_profile_refused(self):
        from straight_pass import advance_route
        s,c,p=self.data();p.pop('edge_diagnostics')
        with self.assertRaisesRegex(ValueError,'edges_missing'):advance_route(p,s.m.core.pose,tuple(p['start']),c,.05,.2)
        with self.assertRaises(ValueError):settings(dict(PROFILE,pass_cross_track_limit_m=.081))
    def test_config_change_cannot_widen_frozen_route_limit(self):
        from straight_pass import advance_route
        s,c,p=self.data();p['live_cross_track_abort_m']=.03
        pose=replace(s.m.core.pose,p=(s.m.core.pose.p[0],.04,s.m.core.pose.p[2]))
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(p,pose,tuple(p['start']),c,.05,.2)

class FastSessionConfirmation(unittest.TestCase):
    def test_two_current_feasible_results_and_three_metric_frames(self):
        s,c,v=StraightRouteTests().data()
        c=decorate(dict(c),settings(dict(PROFILE,fast_confirmation=True)))
        a=Acquisition(IDENTITY,fast_confirmation=True)
        t=s.f.ros
        for j in range(2):
            stamp=t+j*.34;x=copy.deepcopy(v);x['source_stamp']=stamp
            x['metric_geometry'].update(source_stamp=stamp,confirmation_frames=3,confirmation_span_s=.68)
            pose=replace(s.m.core.pose,stamp=stamp)
            ready=a.update(x,c,pose,stamp,s.f.wall+j*.34,s.f.wall+j*.34,t-1,0.,.8,(0,0,.245),.2)
            self.assertEqual(ready,j==1)
        self.assertIsNotNone(a.snapshot);self.assertEqual(a.decision.required_frames,2)
    def test_fast_profile_does_not_accept_two_metric_frames_or_missing_span(self):
        from live_geometry import validate_metric
        s,c,v=StraightRouteTests().data();c=decorate(dict(c),settings(dict(PROFILE,fast_confirmation=True)))
        for frames,span in ((2,.7),(3,None),(3,.1),(3,2.)):
            x=copy.deepcopy(v);x['metric_geometry'].update(confirmation_frames=frames,confirmation_span_s=span)
            with self.assertRaises(ValueError):validate_metric(x,c,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_bad_current_frame_and_duplicate_cannot_supply_second_vote(self):
        from straight_pass import PassageDecision
        s,c,v=StraightRouteTests().data();d=PassageDecision(required_frames=2,window_s=1.2)
        t=s.f.ros
        self.assertFalse(d.update(v,c,s.m.core.pose,t,s.f.wall,s.f.wall,t-1))
        self.assertFalse(d.update(v,c,s.m.core.pose,t,s.f.wall,s.f.wall,t-1))
        x=copy.deepcopy(v);x['source_stamp']=t+.2;x['metric_geometry']['source_stamp']=t+.2;x['ready']=False
        self.assertFalse(d.update(x,c,replace(s.m.core.pose,stamp=t+.2),t+.2,s.f.wall+.2,s.f.wall+.2,t-1))
        self.assertIsNone(d.plan)

class FastSessionSim(SessionSim):
    def __init__(self):
        super().__init__()
        c={k:v for k,v in self.m.core.c.items() if not k.startswith('_')}
        self.m=Mission(c,self.options,True,'straight_pass',session_window_policy=dict(PROFILE,fast_confirmation=True),session_identity=IDENTITY)
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def update(self,follow=True):
        super().update(follow)
        value=getattr(self.m.core,'metric_input',None) or {}
        if isinstance(value.get('metric_geometry'),dict):
            value['metric_geometry'].update(confirmation_frames=3,confirmation_span_s=.4)

class FastFullLifecycle(unittest.TestCase):
    def test_fast_full_flow_keeps_single_snapshot_and_manual_landing(self):
        s=FastSessionSim();s.start();s.until('ADJUST')
        self.assertEqual(s.m.session_window.decision.required_frames,2)
        self.assertEqual(s.m.session_window.decision.snapshot()['safe_samples'],2)
        frozen=copy.deepcopy(s.m.session_window.snapshot);s.target=False
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
        self.assertEqual(s.m.session_window.snapshot,frozen);self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
    def test_fast_template_uses_matching_vote_declaration(self):
        from straight_pass_policy import validate
        c=StraightScope().config();c['session_window']=dict(PROFILE,fast_confirmation=True)
        c['mission']['simplified_takeoff_enabled']=True;c['straight_pass_authorization']['scope']='supervised_session_window_pass'
        c['route_policy'].update(mode='one_acquisition_center_then_session_frozen_line',no_alignment=False,post_commit_visual_dependency=False,localization_binding='boot_run_source_epoch')
        with self.assertRaisesRegex(ValueError,'vote'):validate(c,True)
        c['route_policy']['vote_rule']='2_feasible_distinct_frames_of_latest_5_within_1p2_seconds_current_frame_must_be_feasible'
        self.assertTrue(validate(c,True))


class RelaxedSim(SessionSim):
    def __init__(self):
        super().__init__()
        c={k:v for k,v in self.m.core.c.items() if not k.startswith('_')}
        self.m=Mission(c,self.options,True,'straight_pass',session_window_policy=dict(PROFILE,relaxed_alignment=True),session_identity=IDENTITY)
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()

class DisplacementArrivalRegression(unittest.TestCase):
    def test_accumulated_jitter_does_not_end_adjustment(self):
        s=RelaxedSim();s.start();s.until('ADJUST');s.m.adjust_path_m=1.0
        s.step(follow=False)
        self.assertEqual(s.m.phase,'ADJUST');self.assertGreater(s.out['prepass_adjustment']['actual_path_m'],.3)
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
    def test_5cm_error_and_8cmps_needs_no_precision_hold(self):
        s=RelaxedSim();s.start();s.until('ADJUST');m=s.m;target=tuple(m.adjust_plan['target'])
        m.position=target;m.core.pose=replace(m.core.pose,p=(target[0]+.05,target[1],target[2]))
        m.core.pose_speed=.08;m.adjust_path_m=.4
        out=m.tick(s.inputs())
        self.assertEqual(out['phase'],'PASS');self.assertIsNone(m.adjust_arrival_since)
        self.assertEqual(m.direct_plan['start'],list(m.core.pose.p))
    def test_actual_20cm_displacement_is_still_a_limit(self):
        s=RelaxedSim();s.start();s.until('ADJUST');m=s.m;start=m.adjust_plan['start']
        m.core.pose=replace(m.core.pose,p=(start[0],start[1]+.201,start[2]));m.position=m.core.pose.p
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST')
        self.assertIn('displacement_limit',out['reason']);self.assertIsNone(m.direct_plan)
    def test_close_enough_cannot_bypass_actual_departure_geometry(self):
        s=RelaxedSim();s.start();s.until('ADJUST');m=s.m;target=tuple(m.adjust_plan['target'])
        m.position=target;m.core.pose=replace(m.core.pose,p=target);m.core.pose_speed=.08
        m.session_window.route=Mock(side_effect=ValueError('actual_departure_clips_window'))
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST')
        self.assertIn('departure_rejected',out['reason']);self.assertIsNone(m.direct_plan)
    def test_outside_6cm_waits_and_still_has_finite_timeout(self):
        s=RelaxedSim();s.start();s.until('ADJUST');m=s.m;target=tuple(m.adjust_plan['target'])
        m.position=target;m.core.pose=replace(m.core.pose,p=(target[0]+.065,target[1],target[2]))
        self.assertEqual(m.tick(s.inputs())['phase'],'ADJUST')
        m.entered=s.f.wall-8.01
        self.assertEqual(m.tick(s.inputs())['phase'],'HANDOVER_REQUEST')
    def test_new_profile_has_no_cumulative_speed_or_hold_gate(self):
        from session_window import alignment_policy
        p=alignment_policy(settings(dict(PROFILE,relaxed_alignment=True)))
        self.assertFalse(p['cumulative_path_gate_enabled']);self.assertFalse(p['arrival_speed_gate_enabled'])
        self.assertEqual(p['arrival_tolerance_m'],.06);self.assertEqual(p['settle_s'],0.)
        self.assertEqual(p['max_distance_m'],.2);self.assertEqual(p['timeout_s'],8.)

class ThreeMetreRegression(unittest.TestCase):
    def data(self):
        s,c,v=StraightRouteTests().data();c.update(max_traverse_time_s=55.,max_total_time_s=75.)
        c=decorate(c,settings(dict(PROFILE,fixed_pass_distance_m=3.,pass_cross_track_limit_m=.08)))
        return s,c,v
    def test_exact_3m_keeps_speed_and_checks_endpoint(self):
        s,c,v=self.data();p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        self.assertEqual(p['length_m'],3.);self.assertAlmostEqual(math.dist(p['start'],p['endpoint']),3.)
        self.assertLess(p['minimum_clear_travel_m'],3.)
        c['geofence_max_m'][0]=2.8
        with self.assertRaisesRegex(ValueError,'fence'):straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_old_time_budget_cannot_silently_accept_3m(self):
        s,c,v=self.data();c['max_traverse_time_s']=25.
        with self.assertRaisesRegex(ValueError,'time_budget'):straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_geometry_needing_more_than_3m_is_rejected(self):
        s,c,v=self.data();c['post_plane_clearance_m']=2.
        with self.assertRaisesRegex(ValueError,'insufficient'):straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
    def test_retains_5cm_higher_reference_at_commit_and_rechecks_both_rays(self):
        s=RelaxedSim();c={k:v for k,v in s.m.core.c.items() if not k.startswith('_')}
        c.update(max_traverse_time_s=55.,max_total_time_s=75.)
        profile=dict(PROFILE,relaxed_alignment=True,fixed_pass_distance_m=3.,pass_cross_track_limit_m=.08)
        s.m=Mission(c,s.options,True,'straight_pass',session_window_policy=profile,session_identity=IDENTITY);s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('ADJUST');m=s.m;target=tuple(m.adjust_plan['target'])
        m.position=target;m.core.pose=replace(m.core.pose,p=(target[0],target[1],target[2]-.05));m.core.pose_speed=.08
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'PASS')
        self.assertAlmostEqual(out['position'][2],target[2]);self.assertEqual(m.direct_plan['length_m'],3.)
        self.assertEqual(m.direct_plan['actual_departure_position_local_m'],list(m.core.pose.p))
        with self.assertRaises(ValueError):m.session_window.route(m.core.c,m.core.pose,(target[0],target[1],target[2]+.4))

class SeventyCmFloorRegression(unittest.TestCase):
    def pass_sim(self,bottom):
        s=RelaxedSim();s.m.o['minimum_bottom_height_m']=.70;s.start();s.until('PASS');m=s.m
        z=m.ground_z+.245+bottom
        m.core.pose=replace(m.core.pose,p=(m.core.pose.p[0],m.core.pose.p[1],z),q=(0.,0.,0.,1.))
        m.direct_plan['start'][2]=z;m.direct_plan['endpoint'][2]=z;m.direct_plan['height_local_m']=z
        m.position=tuple(m.direct_plan['start']);return s
    def test_75cm_pass_not_rejected_by_old_80cm_floor(self):
        s=self.pass_sim(.75);out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'PASS');self.assertAlmostEqual(out['height']['actual_bottom_height_m'],.75)
    def test_below_70cm_still_faults(self):
        s=self.pass_sim(.699);out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'FAULT');self.assertIn('height_floor',out['reason'])
    def test_legacy_mode_and_below_70cm_config_still_rejected(self):
        from mission import settings as mission_settings
        s=RelaxedSim();self.assertEqual(mission_settings(dict(s.options,minimum_bottom_height_m=.70))['minimum_bottom_height_m'],.70)
        with self.assertRaises(ValueError):mission_settings(dict(s.options,minimum_bottom_height_m=.699))
        with self.assertRaises(ValueError):mission_settings(dict(s.options,simplified_takeoff_enabled=False,minimum_bottom_height_m=.70))
    def test_trial_policy_accepts_explicit_session_70cm(self):
        from straight_pass_policy import validate
        c=StraightScope().config();c['session_window']=dict(PROFILE,relaxed_alignment=True)
        c['mission'].update(simplified_takeoff_enabled=True,minimum_bottom_height_m=.70)
        c['straight_pass_authorization']['scope']='supervised_session_window_pass'
        c['route_policy'].update(mode='one_acquisition_center_then_session_frozen_line',no_alignment=False,post_commit_visual_dependency=False,localization_binding='boot_run_source_epoch')
        self.assertTrue(validate(c,True))


class SeventyCmTransitionRegression(unittest.TestCase):
    def test_vendor_early_arrival_keeps_climbing_without_precision_gate(self):
        s=RelaxedSim();s.m.o['minimum_bottom_height_m']=.70;s.start();s.until('SEARCH')
        self.assertGreater(s.m.position[2]-s.m.core.pose.p[2],.12)
        s.step();self.assertEqual(s.m.phase,'SEARCH')
        self.assertEqual(s.out['reason'],'continuing_climb_before_lateral_adjustment')
        s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed'])
    def test_climb_continuation_cannot_hide_actual_floor_violation(self):
        s=RelaxedSim();s.m.o['minimum_bottom_height_m']=.70;s.start();s.until('SEARCH');m=s.m
        m.core.pose=replace(m.core.pose,p=(m.core.pose.p[0],m.core.pose.p[1],m.ground_z+.245+.699),q=(0.,0.,0.,1.))
        m.position=(m.core.pose.p[0],m.core.pose.p[1],m.core.pose.p[2]+.15)
        out=m.tick(s.inputs());self.assertEqual(out['phase'],'HANDOVER_REQUEST');self.assertIn('height_floor',out['reason'])

class FasterPassRegression(unittest.TestCase):
    def route(self):
        s,c,v=StraightRouteTests().data();c=decorate(dict(c,traverse_speed_mps=.30,max_setpoint_lead_m=.04),settings(dict(PROFILE,pass_cross_track_limit_m=.08,faster_pass=True)))
        p=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        return s,c,p
    def test_speed_step_and_forward_lead_are_both_bounded(self):
        from straight_pass import advance_route
        s,c,p=self.route();command=tuple(p['start'])
        for i in range(20):
            before=command;command,_=advance_route(p,s.m.core.pose,command,c,.05,i*.05)
            self.assertLessEqual(math.dist(command,before),.015+1e-10)
            self.assertLessEqual(math.dist(command,s.m.core.pose.p),.12+1e-10)
        self.assertAlmostEqual(math.dist(command,p['start']),.12)
    def test_config_change_cannot_raise_frozen_speed_or_lead(self):
        from straight_pass import advance_route
        s,c,p=self.route();c.update(traverse_speed_mps=.4,max_setpoint_lead_m=.12,_session_forward_lead_m=.12)
        command=tuple(p['start'])
        for i in range(20):
            before=command;command,_=advance_route(p,s.m.core.pose,command,c,.05,i*.05)
            self.assertLessEqual(math.dist(command,before),.015+1e-10)
        self.assertAlmostEqual(math.dist(command,p['start']),.12)
    def test_8cm_cross_track_gate_not_relaxed_by_speed(self):
        from straight_pass import advance_route
        s,c,p=self.route();bad=replace(s.m.core.pose,p=(p['start'][0],p['start'][1]+.081,p['start'][2]))
        with self.assertRaisesRegex(ValueError,'cross_track'):advance_route(p,bad,tuple(p['start']),c,.05,.1)
    def test_trial_scope_requires_faster_profile_and_caps_both_values(self):
        from straight_pass_policy import validate
        c=StraightScope().config();c['session_window']=dict(PROFILE,faster_pass=True)
        c['mission']['simplified_takeoff_enabled']=True;c['straight_pass_authorization']['scope']='supervised_session_window_pass'
        c['route_policy'].update(mode='one_acquisition_center_then_session_frozen_line',no_alignment=False,post_commit_visual_dependency=False,localization_binding='boot_run_source_epoch')
        c['control'].update(traverse_speed_mps=.30,max_setpoint_lead_m=.04)
        self.assertTrue(validate(c,True))
        for k,value in [('traverse_speed_mps',.31),('max_setpoint_lead_m',.09)]:
            x=copy.deepcopy(c);x['control'][k]=value
            with self.assertRaises(ValueError):validate(x,True)
        c['session_window']['faster_pass']=False
        with self.assertRaises(ValueError):validate(c,True)
    def test_three_metre_fast_profile_full_synthetic_flow(self):
        s=RelaxedSim();c={k:v for k,v in s.m.core.c.items() if not k.startswith('_')}
        c.update(traverse_speed_mps=.30,max_setpoint_lead_m=.04,max_traverse_time_s=55.,max_total_time_s=75.)
        profile=dict(PROFILE,relaxed_alignment=True,fixed_pass_distance_m=3.,faster_pass=True,pass_cross_track_limit_m=.08)
        s.m=Mission(c,s.options,True,'straight_pass',session_window_policy=profile,session_identity=IDENTITY);s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.out['straight_pass_completed']);self.assertEqual(s.m.direct_plan['length_m'],3.)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])

    def test_forward_and_cross_track_errors_share_total_tracking_allowance(self):
        from straight_pass import advance_route
        s,c,p=self.route();pose=replace(s.m.core.pose,p=(p['start'][0],p['start'][1]+.079,p['start'][2]))
        command=tuple(p['start'])
        for i in range(20):command,_=advance_route(p,pose,command,c,.05,i*.05)
        self.assertLessEqual(math.dist(command,pose.p),c['max_tracking_error_m'])
        self.assertGreater(command[0]-p['start'][0],.08)


