import unittest
from dataclasses import replace
from test_alignment_only import AlignOnlySim

class VendorArrival(unittest.TestCase):
    def sim(self):
        s=AlignOnlySim();s.m.o.update(vendor_takeoff_arrival_enabled=True,takeoff_transient_tolerance_enabled=True)
        s.start();s.until('TAKEOFF');s.m.core.airborne=True;return s
    def test_vendor_band_does_not_require_precision_speed_or_two_second_hold(self):
        s=self.sim();s.m.arm_request_wall=s.f.wall-4
        p=s.m.core.pose;s.m.core.pose=replace(p,p=(s.m.origin[0],s.m.origin[1],s.m.target_z-.08));s.m.core.pose_speed=.08
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'SEARCH')
        self.assertEqual(out['takeoff_progress']['arrival_policy'],'vendor_height_band_and_elapsed_time')
    def test_time_alone_never_certifies_arrival(self):
        s=self.sim();s.m.arm_request_wall=s.f.wall-17
        self.assertFalse(s.m.takeoff_arrival(s.inputs()))
    def test_vendor_height_boundary_and_elapsed_time(self):
        s=self.sim();p=s.m.core.pose;s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z-.08))
        s.m.arm_request_wall=s.f.wall-2.9;self.assertFalse(s.m.takeoff_arrival(s.inputs()))
        s.m.arm_request_wall=s.f.wall-3.1;self.assertTrue(s.m.takeoff_arrival(s.inputs()))
        s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z-.201));self.assertFalse(s.m.takeoff_arrival(s.inputs()))
    def test_airborne_and_input_fault_checks_retained(self):
        s=self.sim();s.m.arm_request_wall=s.f.wall-4;p=s.m.core.pose
        s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z));s.m.core.airborne=False
        self.assertFalse(s.m.takeoff_arrival(s.inputs()))
        s.healthy['estimator_ok']=False;self.assertEqual(s.m.tick(s.inputs())['phase'],'FAULT')

    def test_vendor_band_cannot_enter_search_below_existing_height_floor(self):
        s=self.sim();s.m.arm_request_wall=s.f.wall-4;p=s.m.core.pose
        s.m.core.pose=replace(p,p=(p.p[0],p.p[1],s.m.target_z-.15))
        self.assertFalse(s.m.takeoff_arrival(s.inputs()))
        self.assertEqual(s.m.tick(s.inputs())['phase'],'TAKEOFF')
