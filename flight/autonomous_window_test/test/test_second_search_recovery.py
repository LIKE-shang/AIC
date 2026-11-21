"""Second-search drift, bounded yaw scan and recorded 403882231760 geometry; no IO."""
import copy,math,unittest
from dataclasses import replace
from unittest.mock import patch
from test_second_window_return import ReturnSim
from window_pass.geometry import sub,norm,wrap,yaw
from session_window import select_target
from second_window_return import SEARCH_SCAN_AMPLITUDE_RAD,SEARCH_SCAN_RATE_RAD_S,SEARCH_SCAN_LEAD_RAD,PREPARATION_LEFT_BIAS_M

class SecondSearchRecoveryTests(unittest.TestCase):
    def searching(self):
        s=ReturnSim();s.start();s.until('YAW_TURN');s.until('SEARCH')
        self.assertEqual(s.m.return_journey.leg,2)
        return s

    def test_second_search_survives_recorded_hold_error_and_scans_past_ten_seconds(self):
        s=self.searching();m=s.m;origin=m.origin;epoch=m.localization_epoch;owner=m.core
        initial=tuple(m.position);center=m.heading;s.lower=False
        delta=(-.01729297637939453,.11954283714294434,-.015066742897033691)
        for n in range(1,11):
            s.f.p=tuple(initial[k]+delta[k]*n/10 for k in range(3));s.step(False)
            self.assertEqual(m.phase,'SEARCH',m.reason)
        for _ in range(240):
            s.step(False)
            self.assertEqual(m.phase,'SEARCH',m.reason)
            self.assertLessEqual(abs(wrap(m.heading-center)),SEARCH_SCAN_AMPLITUDE_RAD+1e-9)
        self.assertAlmostEqual(s.out['tracking']['error_norm_m'],.12172322574499957)
        self.assertFalse(s.out['tracking']['tracking_error_abort_enabled'])
        self.assertIsNone(s.out['tracking']['tracking_limit_m'])
        self.assertNotAlmostEqual(m.heading,center)
        self.assertEqual(m.position,initial);self.assertEqual(m.origin,origin)
        self.assertEqual(m.localization_epoch,epoch);self.assertIs(m.core,owner)
        self.assertFalse(m.return_journey.second_pass_complete);self.assertEqual(s.actions,['OFFBOARD','ARM'])

    def test_scan_rate_amplitude_lead_and_fixed_center(self):
        s=self.searching();m=s.m;j=m.return_journey;center=j.scan_center
        m.session_window.decision.latest_safe=False
        extrema=[]
        for _ in range(800):
            m.core.pose=replace(m.core.pose,q=(0.,0.,math.sin(m.heading/2),math.cos(m.heading/2)))
            previous=m.heading;j.scan(m,.05,s.f.ros)
            self.assertLessEqual(abs(wrap(m.heading-previous)),SEARCH_SCAN_RATE_RAD_S*.05+1e-9)
            extrema.append(wrap(m.heading-center))
        self.assertAlmostEqual(min(extrema),-SEARCH_SCAN_AMPLITUDE_RAD)
        self.assertAlmostEqual(max(extrema),SEARCH_SCAN_AMPLITUDE_RAD)
        self.assertEqual(j.scan_center,center)
        m.core.pose=replace(m.core.pose,q=(0.,0.,math.sin(center/2),math.cos(center/2)))
        j.scan_offset=0.;j.scan_direction=-1;m.heading=center
        for _ in range(100):j.scan(m,.05,s.f.ros)
        self.assertLessEqual(abs(wrap(m.heading-center)),SEARCH_SCAN_LEAD_RAD+1e-9)

    def test_feasible_frame_pauses_scan_but_stale_or_rejected_frame_resumes(self):
        s=self.searching();m=s.m;j=m.return_journey;d=m.session_window.decision
        d.latest_safe=True;d.last_stamp=s.f.ros;h=m.heading
        j.scan(m,.05,s.f.ros+.2);self.assertEqual(m.heading,h)
        j.scan(m,.05,s.f.ros+.51);self.assertNotEqual(m.heading,h)
        d.latest_safe=False;h=m.heading;j.scan(m,.05,s.f.ros);self.assertNotEqual(m.heading,h)

    def test_missing_frames_never_authorize_translation_or_pass(self):
        s=self.searching();s.lower=False;m=s.m;position=m.position
        for _ in range(220):s.step(False)
        self.assertEqual(m.phase,'SEARCH');self.assertIsNone(m.session_window.snapshot)
        self.assertEqual(m.position,position);self.assertFalse(m.direct_pass_completed)
        self.assertFalse(m.return_journey.second_pass_complete)
        self.assertNotIn('LAND',s.actions)

    def test_whole_mission_timeout_and_untrusted_input_still_stop_search(self):
        s=self.searching();s.m.began=s.f.wall-s.m.o['max_mission_time_s'];s.step(False)
        self.assertIn(s.m.phase,('FAULT','HANDOVER_REQUEST'));self.assertIn('timeout',s.m.reason)
        for key in ('estimator_ok','graph_ok','rc_ok'):
            s=self.searching();s.healthy[key]=False;s.step(False)
            self.assertEqual(s.m.phase,'FAULT',key)

    def test_field_floor_and_divider_cannot_be_scanned_through(self):
        for kind in ('field','floor','divider'):
            s=self.searching();m=s.m
            if kind=='field':s.f.p=(s.f.p[0],m.core.c['geofence_max_m'][1]+.1,s.f.p[2])
            elif kind=='floor':s.f.p=(s.f.p[0],s.f.p[1],m.ground_z+.3)
            else:s.f.p=(1.65,-2.65,s.f.p[2])
            s.step(False);self.assertIn(m.phase,('FAULT','HANDOVER_REQUEST'),kind)
            self.assertFalse(m.return_journey.second_pass_complete)

    def test_first_search_keeps_tracking_and_search_deadline(self):
        s=ReturnSim();s.start();s.until('SEARCH');self.assertEqual(s.m.return_journey.leg,1)
        s.m.o['minimum_bottom_height_m']=.8
        s.m.position=(s.f.p[0]+.13,s.f.p[1],s.f.p[2]);s.step(False)
        self.assertEqual(s.m.phase,'FAULT');self.assertTrue(s.m.reason.startswith('tracking_error'))
        s=ReturnSim();s.start();s.until('SEARCH');s.m.o['minimum_bottom_height_m']=.8
        s.m.position=tuple(s.f.p);s.m.entered=s.f.wall-11.
        with patch.object(s.m,'full_height_bounds',return_value=(1.,1.5)):s.step(False)
        self.assertEqual(s.m.phase,'SEARCH');self.assertEqual(s.m.reason,'retry_fresh_window_acquisition')
        for attempt in range(2):
            s.m.entered=s.f.wall-11.
            with patch.object(s.m,'full_height_bounds',return_value=(1.,1.5)):s.step(False)
        self.assertEqual(s.m.phase,'HANDOVER_REQUEST');self.assertIn('session_acquisition_timeout',s.m.reason)

    def test_preparation_uses_drawing_point_without_unreliable_fixed_drift_bias(self):
        s=ReturnSim();s.start();s.until('RETURN_PREP');m=s.m;j=m.return_journey
        a,b=j.coordinates(m,j.target)
        self.assertAlmostEqual(a,j.p['preparation_forward_left_m'][0])
        self.assertAlmostEqual(b,j.p['preparation_forward_left_m'][1])
        self.assertEqual(PREPARATION_LEFT_BIAS_M,0.)
        self.assertEqual(j.p['preparation_forward_left_m'],[3.15,-2.65])
        self.assertTrue(j.region(m,j.target,1))

    def test_confirmed_second_geometry_starts_adjustment_from_measured_pose(self):
        s=self.searching();m=s.m;s.lower=False
        initial=tuple(s.f.p);delta=(-.0173,.1196,-.0151)
        for n in range(1,11):
            s.f.p=tuple(initial[k]+delta[k]*n/10 for k in range(3));s.step(False)
        actual=tuple(m.core.pose.p)
        m.session_window.plan=dict(start=list(actual),target=list(actual),yaw_rad=m.heading)
        m.session_window.snapshot={}
        with patch.object(m.return_journey,'filter_geometry'),patch.object(m.session_window,'update',return_value=True):
            out=m.tick(s.inputs())
        self.assertEqual(out['phase'],'ADJUST',out['reason']);self.assertEqual(m.position,actual)

    def test_recorded_geometry_current_ray_rejected_scan_finds_fully_checked_target(self):
        s=self.searching();m=s.m
        m.core.c.update(geofence_min_m=[-.8375371503932867,-3.31041806882445,-.42021111354704477],
                        geofence_max_m=[4.120505705863678,.9878852138176755,1.6739039344588915])
        p=(3.2028183937072754,-2.495122194290161,.8353562355041504)
        q=(.016711632415075475,.0007044305722099962,-.9993615424244859,.031573721506057056)
        pose=replace(m.core.pose,p=p,q=q);h=yaw(q)
        metric=dict(center_enu_m=[1.5749873046564637,-2.997940624256809,.7719587193759514],
            normal_enu=[-.9994662829828775,-.032035747773834417,-.0063921862427284756],
            corners_enu_m=[[1.5893189632865468,-3.5205491266103253,1.1536737632229421],
                [1.556368839801993,-2.491108876286002,1.148420921588371],
                [1.5604950130238837,-2.469839300906576,.386233534532826],
                [1.5949610460552481,-3.5411604512126567,.36841969403260405]],pose_epoch=0,calibration_epoch=0)
        g=dict(center=metric['center_enu_m'],stamp=1793540623.0168855,epoch=(0,0))
        preferred=[3.228007179985117,-2.893395390650315,.8419587193759515]
        c=m.session_window.scoped_control(m.core.c);policy=m.session_alignment_policy
        def select(pp,evidence):
            return select_target(c,pp,g,metric,preferred,-.3176809501647949,.7,m.origin,None,policy,evidence,.03,True)
        with self.assertRaisesRegex(ValueError,'no_feasible_aperture_target'):select(pose,dict(exact_center_target_local_m=preferred))
        a=h-math.radians(4);pp=replace(pose,q=(0.,0.,math.sin(a/2),math.cos(a/2)))
        evidence=dict(exact_center_target_local_m=preferred);target=select(pp,evidence)
        self.assertGreaterEqual(evidence['selected_minimum_edge_clearance_m'],.03)
        self.assertTrue(all(math.isfinite(x) for x in target))
        self.assertIsNone(m.session_window.snapshot) # a hypothetical route is no acquisition vote

if __name__=='__main__':unittest.main(verbosity=2)
