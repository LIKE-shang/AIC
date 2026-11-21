"""Synthetic integrated state transitions; no physical flight or hit claims."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from test_session_window import SessionSim,IDENTITY,PROFILE
from mission import Mission
from hover_target import DEFAULTS as HOVER
from full_chain import settings,SCOPE
from full_chain_policy import validate
from full_chain_localization import RouteContinuity
from window_pass.core import Pose

CHAIN=dict(enabled=True,station_offset_forward_left_m=[0.,0.],station_relation_confirmed=True,
           station_area_confirmed=True,station_evidence='SYNTHETIC_ONLY')

class ChainSim(SessionSim):
    def __init__(self,offset=(0.,0.)):
        self.allow_on=True;self.allow_off=True;self.camera=True;self.detected=True
        super().__init__()
        c={k:copy.deepcopy(v) for k,v in self.m.core.c.items() if not k.startswith('_session_')};c['geofence_max_m']=[5.,5.,4.]
        policy=dict(CHAIN,station_offset_forward_left_m=list(offset))
        h=dict(HOVER,enabled=True,trigger_policy='hover_timed',visual_servo_enabled=True,
               hover_light_delay_s=1.,exposure_s=.4,max_light_s=2.)
        self.m=Mission(c,self.options,True,'straight_pass',session_window_policy=PROFILE,
                       session_identity=IDENTITY,hover_target_policy=h,full_chain_policy=policy)
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def update(self,follow=True):
        if getattr(self.m,'phase',None)=='PASS' and self.m.session_policy.get('smooth_pass'):
            from test_airborne_geometry import DynamicSim
            DynamicSim.update(self,follow)
        else:super().update(follow)
        if getattr(self.m,'session_policy',{}).get('fast_confirmation'):
            value=getattr(self.m.core,'metric_input',None) or {}
            if isinstance(value.get('metric_geometry'),dict):
                value['metric_geometry'].update(confirmation_frames=3,confirmation_span_s=.4)
    def inputs(self):
        i=super().inputs();h=getattr(self.m,'hover_target',None)
        if not h:return i
        anchor=self.m.aim_anchor or self.f.p
        error=[-(self.f.p[1]-anchor[1])*400,-(self.f.p[0]-anchor[0])*400]
        d=dict(valid=self.detected,capture_stamp=i.ros if self.camera else i.ros-2.,camera_frame_id='circle_usb_optical_frame',
               source_image_size_px=[640,360],reference_point_px=[372.75,194.984375],reference_error_px=error,
               inner_semiaxes_px=[20.8,20.8],inner_angle_deg=0.,reference_normalized_radius=math.hypot(*error)/20.8,
               observed_error_px=error,observed_inside_inner=self.detected)
        on=h.request_on and self.allow_on
        if not h.request_on and not self.allow_off:on=True
        b=dict(scope=SCOPE,fault_latched=False,state='ON_ACKNOWLEDGED' if on else 'OFF_ACKNOWLEDGED',request_id=h.request_id)
        return replace(i,hover_visual=(i.wall,d),hover_laser=(i.wall,b))

class FullChainFlow(unittest.TestCase):
    def test_single_owner_end_to_end_no_rearm_and_no_ground_or_epoch_reset(self):
        s=ChainSim((.08,-.05));s.start();origin=s.m.origin;ground=s.m.ground_z;core=s.m.core
        s.until('PASS');epoch=s.m.localization_epoch
        self.assertFalse(s.m.hover_target.request_on)
        s.until('POSTPASS');self.assertEqual(s.actions,['OFFBOARD','ARM'])
        self.assertTrue(s.m.direct_pass_completed);self.assertIsNone(s.m.aim_anchor)
        s.until('HOVER');self.assertTrue(s.m.full_chain.hold_complete)
        self.assertGreater(math.dist(s.m.aim_anchor[:2],origin[:2]),.15)
        self.assertIs(s.m.core,core);self.assertEqual(s.m.origin,origin);self.assertEqual(s.m.ground_z,ground);self.assertEqual(s.m.localization_epoch,epoch)
        s.until('PILOT_CONTROL');self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
        self.assertTrue(s.out['full_chain']['mission_complete']);self.assertTrue(s.out['full_chain']['laser_off_confirmed'])
        self.assertFalse(s.out['full_chain']['physical_hit_verified'])
        events=[e['new'] for e in s.m.events]
        self.assertEqual(events.count('TAKEOFF'),1);self.assertEqual(events.count('ARM'),1)
    def test_deployed_three_metre_faster_profile_completes_same_supervisor_chain(self):
        s=ChainSim();root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config/full_chain_single_target.json').read_text())
        control={k:copy.deepcopy(v) for k,v in s.m.core.c.items() if not k.startswith('_session_')}
        control.update({k:cfg['control'][k] for k in ('traverse_speed_mps','max_pose_step_m','max_pose_speed_mps','tilt_limit_rad')});control.update(max_traverse_distance_m=3.,max_traverse_time_s=55.)
        options=dict(s.options,minimum_bottom_height_m=.7)
        s.m=Mission(control,options,True,'straight_pass',session_window_policy=cfg['session_window'],
                    session_identity=IDENTITY,hover_target_policy=cfg['hover_target'],full_chain_policy=dict(CHAIN,station_reference='takeoff_body',station_offset_forward_left_m=[3.15,-.05]))
        s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.out['full_chain']['mission_complete'])
        self.assertEqual(s.m.direct_plan['length_m'],3.)
        self.assertEqual(s.m.direct_plan['command_speed_mps'],1.5)
        self.assertEqual(s.m.direct_plan['maximum_forward_lead_m'],.35)
        self.assertAlmostEqual(s.m.aim_anchor[0]-s.m.origin[0],3.15)
        self.assertAlmostEqual(s.m.aim_anchor[1]-s.m.origin[1],-.05)
        self.assertNotEqual(s.m.aim_anchor[1],s.m.direct_plan['endpoint'][1]-.05)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])

    def test_target_missing_is_precision_failure_not_timed_exposure_veto(self):
        s=ChainSim();s.detected=False;s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.out['full_chain']['mission_complete']);self.assertFalse(s.out['full_chain']['target_acquired'])
        self.assertFalse(s.out['full_chain']['aim_converged']);self.assertEqual(s.m.hover_target.observed_hit_frames,0)
    def test_postpass_missing_geometry_cannot_restart_acquisition(self):
        s=ChainSim();s.start();s.until('POSTPASS');snap=copy.deepcopy(s.m.session_window.snapshot)
        s.target=False;s.m.core.metric_input=None;s.until('PILOT_CONTROL')
        self.assertEqual(s.m.session_window.snapshot,snap)
        self.assertEqual([e['new'] for e in s.m.events].count('SEARCH'),1)
    def test_identity_change_postpass_and_aim_stops_never_requests_light(self):
        for phase in ('POSTPASS','HOVER'):
            s=ChainSim();s.start();s.until(phase);s.local_epoch+=1;s.step()
            self.assertEqual(s.m.phase,'FAULT');self.assertFalse(s.out['hover_target']['request_on'])
            self.assertFalse(s.out['full_chain']['mission_complete'])
    def test_postpass_timeout_does_not_claim_chain_success(self):
        s=ChainSim((.5,0.));s.start();s.until('POSTPASS')
        for _ in range(650):
            s.step(follow=False)
            if s.m.phase=='PILOT_CONTROL':break
        self.assertEqual(s.m.phase,'PILOT_CONTROL');self.assertFalse(s.out['full_chain']['mission_complete'])
        self.assertIsNone(s.m.hover_target.on_request_wall)
    def test_camera_lost_during_light_waits_off_ack_before_posctl(self):
        s=ChainSim();s.start();s.until('HOVER')
        while s.m.hover_target.phase!='EXPOSE':s.step()
        s.allow_off=False;s.camera=False;s.step()
        self.assertFalse(s.out['hover_target']['request_on']);self.assertNotIn('POSCTL',s.actions)
        for _ in range(5):s.step()
        self.assertNotIn('POSCTL',s.actions)
        s.allow_off=True;s.until('PILOT_CONTROL')
        self.assertTrue(s.out['full_chain']['laser_off_confirmed']);self.assertFalse(s.out['full_chain']['mission_complete'])
    def test_off_ack_timeout_reports_unknown_and_incomplete(self):
        s=ChainSim();s.start();s.until('HOVER')
        while s.m.hover_target.phase!='EXPOSE':s.step()
        s.allow_off=False;s.until('PILOT_CONTROL')
        self.assertFalse(s.out['full_chain']['mission_complete']);self.assertFalse(s.out['full_chain']['laser_off_confirmed'])
        self.assertIn('hardware_unknown',s.m.handover_snapshot['reason'])
    def test_mode_override_while_on_clears_request_no_reacquisition(self):
        s=ChainSim();s.start();s.until('HOVER')
        while not s.m.hover_target.request_on:s.step()
        s.mode='POSCTL';s.step();self.assertEqual(s.m.phase,'TAKEOVER')
        self.assertFalse(s.out['hover_target']['request_on']);self.assertFalse(s.out['full_chain']['mission_complete'])
    def test_simplified_takeoff_does_not_abort_at_12cm(self):
        s=ChainSim();s.start();s.until('TAKEOFF')
        for _ in range(13):
            s.f.p=(s.f.p[0]+.01,s.f.p[1],s.f.p[2]);s.step(follow=False)
            if s.m.phase=='FAULT':break
        self.assertEqual(s.m.phase,'TAKEOFF');self.assertIsNone(s.m.horizontal_tracking_limit())
    def test_backwards_station_cannot_reenter_window_clearance(self):
        s=ChainSim((-1.,0.));s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.out['full_chain']['mission_complete']);self.assertIsNone(s.m.hover_target.on_request_wall)
    def test_station_range_is_distinct_from_takeoff_origin(self):
        s=ChainSim();s.start();s.until('HOVER');h=s.m.hover_target
        p=s.m.position;d=s.inputs().hover_visual[1];d['reference_error_px']=[10.,10.]
        h.correct_reference(s.m,d,.05,s.f.wall)
        self.assertGreater(h.correction_updates,0);self.assertNotEqual(s.m.position,p)
        self.assertLess(math.dist(s.m.position[:2],s.m.aim_anchor[:2]),.15)
    def test_low_pass_height_climbs_before_any_horizontal_aim_or_on(self):
        s=ChainSim();s.start();s.until('POSTPASS')
        # Force an intentional remaining height gap while keeping route plane and ground unchanged.
        s.m.target_z+=.10;s.m.full_chain.station=tuple(list(s.m.full_chain.station[:2])+[s.m.target_z])
        initial=s.m.position
        s.step();self.assertEqual(s.m.position[:2],initial[:2]);self.assertFalse(s.m.hover_target.request_on)
        s.until('HOVER');self.assertLess(abs(s.f.p[2]-s.m.target_z),.04)

class FullChainScope(unittest.TestCase):
    def profile(self):
        root=Path(__file__).resolve().parents[1]
        c=json.loads((root/'config/full_chain_single_target.json').read_text())
        c['full_chain']=dict(CHAIN,fixed_aim_compensation_forward_left_m=c['full_chain'].get('fixed_aim_compensation_forward_left_m'));c['site_identity']=IDENTITY;c['evidence_id']='synthetic'
        c['control'].update(enable_motion=True,enable_traverse=True,pass_stage_approved=True)
        c['exit_corridor_confirmation']['operator_confirmed_clear']=True
        return c
    def test_explicit_combined_scope_only(self):
        c=self.profile();self.assertTrue(validate(c,True))
        with self.assertRaises(ValueError):validate(c,False)
        from straight_pass_policy import validate as old_pass
        from hover_trial_policy import validate as old_hover
        for policy in (old_pass,old_hover):
            with self.assertRaises(ValueError):policy(c,True)
    def test_missing_relation_cannot_be_silently_zero(self):
        for k,v in [('station_offset_forward_left_m',None),('station_relation_confirmed',False),('station_evidence',''),('station_area_confirmed',False)]:
            with self.assertRaises(ValueError):settings(dict(CHAIN,**{k:v}))
    def test_nonfinite_or_out_of_scope_station_rejected(self):
        for v in ([float('nan'),0],[1.01,0],[0,0,0]):
            with self.assertRaises(ValueError):settings(dict(CHAIN,station_offset_forward_left_m=v))
    def test_deferral_additions_and_second_target_rejected(self):
        c=self.profile();c['full_chain_authorization']['deferred_control_checks'].append('extra')
        with self.assertRaises(ValueError):validate(c,True)
        c=self.profile();c['full_chain_authorization']['target_count']=2
        with self.assertRaises(ValueError):validate(c,True)
    def test_route_guard_is_relative_to_new_raw_anchor(self):
        s=ChainSim();g=RouteContinuity({k:v for k,v in s.m.core.c.items() if not k.startswith('_session_')})
        # Fresh raw origin is anchored once; later discontinuity stays latched.
        p=Pose(100.,(1.,1.,0.),(0.,0.,0.,1.),'camera_init','body')
        self.assertTrue(g.observe(p,1,100.,100.,'lio','lio'))
        self.assertFalse(g.observe(replace(p,stamp=100.1,p=(7.,1.,0.)),2,100.1,100.1,'lio','lio'))
        self.assertFalse(g.observe(replace(p,stamp=100.2),3,100.2,100.2,'lio','lio'))

    def test_implausible_initial_raw_pose_cannot_become_route_anchor(self):
        s=ChainSim();g=RouteContinuity({k:v for k,v in s.m.core.c.items() if not k.startswith('_session_')})
        p=Pose(100.,(1306.,-1083.,-7661.),(0.,0.,0.,1.),'camera_init','body')
        self.assertFalse(g.observe(p,1,100.,100.,'lio','lio'));self.assertIsNone(g.anchor)
    def test_recorder_never_subscribes_to_flight_control_input(self):
        import importlib.util
        root=Path(__file__).resolve().parents[1]
        spec=importlib.util.spec_from_file_location('chain_recorder',root/'scripts/run_recorded_hover_trial.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertNotIn('/mavros/setpoint_raw/local',module.TOPICS)
        self.assertIn('/mavros/setpoint_raw/target_local',module.TOPICS)

if __name__=='__main__':unittest.main()

