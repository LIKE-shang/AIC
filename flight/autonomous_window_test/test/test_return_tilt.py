"""Return translation tilt scope and recorded failure regression. No IO."""
import math,unittest
from dataclasses import replace
from test_second_window_return import ReturnSim
from window_pass.geometry import yaw
from attitude_envelope import support

RECORDED_Q=(-.00006658938646913222,-.07044503548869584,-.9970284614783169,.03117391650108267)

class ReturnTiltTests(unittest.TestCase):
    def sim(self,phase='RETURN_HOME'):
        s=ReturnSim();s.start();s.until(phase,limit=4500);return s
    def roll(self,s,degrees):
        # Preserve yaw while changing body tilt; avoid injecting a yaw fault.
        h=yaw(s.m.core.pose.q);a=math.radians(degrees)/2
        q=(math.cos(h/2)*math.sin(a),math.sin(h/2)*math.sin(a),math.sin(h/2)*math.cos(a),math.cos(h/2)*math.cos(a))
        s.m.core.pose=replace(s.m.core.pose,q=q)
    def test_recorded_8p079deg_return_home_not_rejected(self):
        s=self.sim();s.m.core.pose=replace(s.m.core.pose,q=RECORDED_Q)
        self.assertEqual(s.m.common_error(s.inputs()),'')
        self.assertAlmostEqual(s.m.attitude_limit(),math.radians(15))
    def test_15deg_ceiling_for_both_return_movements(self):
        for phase in ('RETURN_PREP','RETURN_HOME'):
            s=self.sim(phase)
            for degrees in (8.07910127,12.,14.99):
                self.roll(s,degrees);self.assertEqual(s.m.common_error(s.inputs()),'',(phase,degrees))
                self.assertTrue(s.m.corridor(s.m.core.pose.p))
            self.roll(s,15.01);self.assertEqual(s.m.common_error(s.inputs()),'attitude_limit')
    def test_turn_and_hold_phases_keep_original_limit(self):
        s=self.sim();base=s.m.core.c['tilt_limit_rad'];self.assertAlmostEqual(base,math.radians(8))
        for phase in ('YAW_TURN','HOVER','SEARCH','ADJUST','LANDING_LIGHT_ON','LANDING_HANDOVER','AUTO_LANDING'):
            s.m.phase=phase;self.assertEqual(s.m.attitude_limit(),base,phase)
        s.m.return_journey.active=False;s.m.phase='RETURN_HOME';self.assertEqual(s.m.attitude_limit(),base)
    def test_no_widening_without_session_cone_or_cylinder(self):
        s=self.sim();c=s.m.core.c;base=c['tilt_limit_rad']
        prior=c.pop('_session_pass_tilt_limit_rad');self.assertEqual(s.m.attitude_limit(),base)
        c['_session_pass_tilt_limit_rad']=prior
        c.pop('_session_cylinder_radius_m');self.assertEqual(s.m.attitude_limit(),base)
    def test_return_limit_never_exceeds_precomputed_cone_or_15deg(self):
        s=self.sim();c=s.m.core.c
        for cone,expected in ((12.,12.),(15.,15.),(25.,15.)):
            c['_session_pass_tilt_limit_rad']=math.radians(cone)
            self.assertAlmostEqual(s.m.attitude_limit(),math.radians(expected))
    def test_existing_fence_support_already_covers_return_15deg(self):
        s=self.sim();c=s.m.core.c;expanded=dict(c,tilt_limit_rad=math.radians(15))
        for axis in ((1.,0.,0.),(-1.,0.,0.),(0.,1.,0.),(0.,-1.,0.),(0.,0.,1.),(0.,0.,-1.)):
            self.assertAlmostEqual(support(c,s.m.heading,axis),support(expanded,s.m.heading,axis),places=12)
    def test_floor_and_corridor_remain_enforced_with_12deg_tilt(self):
        s=self.sim();self.roll(s,12.)
        p=s.m.core.pose.p;s.m.core.pose=replace(s.m.core.pose,p=(p[0],p[1],p[2]-.5))
        s.m.tick(s.inputs());self.assertEqual(s.m.phase,'FAULT');self.assertIn('height_floor',s.m.reason)
        s=self.sim();self.roll(s,12.)
        s.m.core.pose=replace(s.m.core.pose,p=(1.65,-2.65,s.m.target_z))
        s.m.tick(s.inputs());self.assertEqual(s.m.phase,'FAULT');self.assertIn('corridor',s.m.reason)
    def test_complete_return_after_recorded_tilt_policy_check(self):
        s=self.sim();s.m.core.pose=replace(s.m.core.pose,q=RECORDED_Q)
        self.assertEqual(s.m.common_error(s.inputs()),'')
        s.until('AUTO_LANDING',limit=4500)
        self.assertTrue(s.m.return_journey.origin_return_stopped)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])

if __name__=='__main__':unittest.main()
