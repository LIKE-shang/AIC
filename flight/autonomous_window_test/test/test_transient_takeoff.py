"""Transient ascent tolerance cannot propagate into alignment motion."""
import unittest,math
from dataclasses import replace
from test_alignment_only import AlignOnlySim
from mission import Mission
from test_alignment_scope import AlignmentScope
from alignment_policy import validate

class TransientTakeoff(unittest.TestCase):
    def sim(self):
        s=AlignOnlySim();s.m.o['takeoff_transient_tolerance_enabled']=True
        s.start();s.until('TAKEOFF');return s
    def set_pose(self,s,dx=0,yaw=0,tilt=0,z=None):
        q=(0,0,math.sin(yaw/2),math.cos(yaw/2)) if not tilt else (math.sin(tilt/2),0,0,math.cos(tilt/2))
        s.m.core.pose=replace(s.m.core.pose,p=(s.m.origin[0]+dx,s.m.origin[1],s.m.core.pose.p[2] if z is None else z),q=q)
    def test_logged_5_degree_heading_is_tolerated_during_ascent(self):
        s=self.sim();self.set_pose(s,dx=.03,yaw=s.m.heading+.08974746058)
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'TAKEOFF');self.assertTrue(out['takeoff_transient_tolerance_active'])
        self.assertFalse(out['takeoff_precise_ready'])
    def test_coarse_limits_remain(self):
        for kwargs,reason in [({'dx':.301},'takeoff_horizontal_tracking_error'),({'yaw':math.radians(31)},'heading_changed'),({'tilt':math.radians(16)},'attitude_limit')]:
            s=self.sim();self.set_pose(s,**kwargs)
            with self.subTest(kwargs=kwargs):self.assertIn(reason,s.m.tick(s.inputs())['reason'])
    def test_cannot_enter_search_until_heading_and_position_recovered(self):
        s=self.sim();self.set_pose(s,dx=.2,yaw=s.m.heading+.10,z=s.m.target_z);s.m.core.pose_speed=0
        s.m.stable_since=s.f.wall-5
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'TAKEOFF')
    def test_strict_checks_resume_after_ascent(self):
        s=self.sim();s.until('SEARCH');self.set_pose(s,yaw=s.m.heading+.09)
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'FAULT');self.assertIn('heading_changed',out['reason'])
    def test_actual_input_faults_still_stop(self):
        s=self.sim();s.healthy['estimator_ok']=False
        self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')
    def test_overshoot_remains_bounded(self):
        s=self.sim();self.set_pose(s,z=s.m.target_z+.201)
        self.assertIn('takeoff_height_overshoot',s.m.tick(s.inputs())['reason'])
    def test_permission_is_explicit_and_alignment_only(self):
        c=AlignmentScope().config();c['mission']['takeoff_transient_tolerance_enabled']=True
        with self.assertRaises(ValueError):validate(c,True)
        c['alignment_commissioning_authorization']['operator_accepts_takeoff_transient_tolerance']=True
        self.assertTrue(validate(c,True))
        with self.assertRaises(ValueError):Mission(c['control'],c['mission'],experiment='traverse',airborne_geometry=True)
