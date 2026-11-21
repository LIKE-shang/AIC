"""Independent straight trial authorization and reject-to-pilot path."""
import copy,unittest
from test_straight_pass import StraightSim
from mission import Mission
from straight_pass_policy import validate,SCOPE,ACCEPTANCE_DEFERRALS,CONTROL_DEFERRALS

class StraightScope(unittest.TestCase):
    def config(self):
        s=StraightSim();c=copy.deepcopy(s.m.core.c);c.update(max_traverse_time_s=25.,max_traverse_distance_m=3.,post_plane_clearance_m=.4)
        for k in CONTROL_DEFERRALS:c[k]=False
        return {'experiment':'straight_pass','acquisition':'airborne_rgbd','control':c,'mission':dict(s.options,vendor_takeoff_arrival_enabled=True,takeoff_transient_tolerance_enabled=True,max_mission_time_s=75.),
            'installation_confirmation':{'operator_confirmed':True,'camera_translation_body_m':c['camera_translation_body_m'],'camera_rotation_body_from_optical_xyzw':c['camera_rotation_body_from_optical_xyzw']},
            'exit_corridor_confirmation':{'operator_confirmed_clear':True},
            'straight_pass_authorization':{'scope':SCOPE,'operator_authorized_test':True,'deferred_acceptance_checks':sorted(ACCEPTANCE_DEFERRALS),'deferred_control_checks':sorted(CONTROL_DEFERRALS)},
            'acceptance':{k:False for k in ACCEPTANCE_DEFERRALS},
            'route_policy':{'mode':'fixed_current_height_and_heading','no_alignment':True,'route_frozen_once':True,'vote_rule':'3_feasible_distinct_frames_of_latest_5_within_1_second_current_frame_must_be_feasible'}}
    def test_authorization_keeps_all_original_false_flags(self):
        c=self.config();before=copy.deepcopy(c);self.assertEqual(validate(c,True),ACCEPTANCE_DEFERRALS);self.assertEqual(c,before)
        m=Mission(c['control'],c['mission'],True,'straight_pass',straight_commissioning=True)
        self.assertTrue(all(m.core.c[k] is False for k in CONTROL_DEFERRALS))
    def test_scope_cannot_skip_live_gate_or_expand_route(self):
        for case in ('extra','mode','speed','time','camera','corridor','vote'):
            c=self.config()
            if case=='extra':c['straight_pass_authorization']['deferred_acceptance_checks'].append('rc_ok')
            elif case=='mode':c['experiment']='align_only'
            elif case=='speed':c['control']['traverse_speed_mps']=.2
            elif case=='time':c['control']['max_traverse_time_s']=30.
            elif case=='camera':c['installation_confirmation']['camera_translation_body_m']=[.5,0,0]
            elif case=='corridor':c['exit_corridor_confirmation']['operator_confirmed_clear']=False
            else:c['route_policy']['vote_rule']='one_frame'
            with self.subTest(case=case),self.assertRaises(ValueError):validate(c,True)
    def test_no_implicit_scope_or_other_mode_use(self):
        c=self.config();self.assertFalse(validate(c,False));c['straight_pass_authorization']['operator_authorized_test']=False
        with self.assertRaises(ValueError):validate(c,True)
        s=StraightSim()
        with self.assertRaises(ValueError):Mission(s.m.core.c,s.options,True,'align_only',straight_commissioning=True)
    def test_valid_trial_can_complete_without_legacy_rgb(self):
        c=self.config();s=StraightSim();s.options=c['mission'];s.m=Mission(c['control'],s.options,True,'straight_pass',straight_commissioning=True);s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('PILOT_CONTROL');self.assertTrue(s.out['straight_pass_completed']);self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
    def test_confirmed_geometry_rejection_hands_over_without_waiting_timeout(self):
        from test_airborne_geometry import status
        s=StraightSim();s.start();s.until('SEARCH')
        t=s.f.ros+.01;v=status(s,stamp=t)
        v['metric_geometry']['center_enu_m'][2]+=.4
        for corner in v['metric_geometry']['corners_enu_m']:corner[2]+=.4
        s.f.ros=t;s.m.core.pose.stamp=t
        s.m.core.ingest_metric(v,s.f.wall)
        out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'HANDOVER_REQUEST');self.assertEqual(out['action'],'POSCTL')
        self.assertIn('straight_route_rejected',out['reason']);self.assertFalse(out['straight_pass_completed'])
        self.assertNotIn('PASS',[e['new'] for e in s.m.events])
    def test_stale_input_is_wait_not_false_pass(self):
        s=StraightSim(False);s.start();s.until('SEARCH');s.step()
        self.assertEqual(s.m.phase,'SEARCH');self.assertIsNone(s.m.direct_plan)

    def test_route_geometry_can_allow_oblique_window_without_yaw_alignment(self):
        import math
        from test_straight_pass import StraightRouteTests
        from straight_pass import straight_route
        s,c,v=StraightRouteTests().data();angle=.12
        center=v['metric_geometry']['center_enu_m']
        v['metric_geometry']['normal_enu']=[math.cos(angle),math.sin(angle),0.]
        for corner in v['metric_geometry']['corners_enu_m']:
            x,y=corner[0]-center[0],corner[1]-center[1]
            corner[0]=center[0]+math.cos(angle)*x-math.sin(angle)*y
            corner[1]=center[1]+math.sin(angle)*x+math.cos(angle)*y
        plan=straight_route(v,c,s.m.core.pose,s.f.ros,s.f.wall,s.f.wall,s.f.ros-1)
        self.assertAlmostEqual(plan['yaw_rad'],0.)
        self.assertGreater(min(plan['edge_clearance_after_budgets_m']),0.)
