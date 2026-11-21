"""Tracking diagnostics freeze the causal sample without changing flight limits."""
import unittest,copy
from test_takeoff import TakeoffSim

class TrackingDiagnostics(unittest.TestCase):
    def test_normal_climb_does_not_require_constant_coordinates(self):
        s=TakeoffSim();s.start();s.until('HOVER')
        row=s.m.output()['tracking']
        self.assertGreater(row['actual_position_local_m'][2]-s.m.origin[2],.8)
        self.assertLess(row['error_norm_m'],row['tracking_limit_m'])
    def test_horizontal_and_vertical_error_reported_separately(self):
        s=TakeoffSim();s.start();s.until('TAKEOFF')
        p=s.m.core.pose.p;s.m.position=(p[0]+.08,p[1]+.06,p[2]+.03)
        row=s.m.tracking_snapshot()
        self.assertAlmostEqual(row['horizontal_error_m'],.1)
        self.assertAlmostEqual(row['vertical_error_m'],.03)
    def test_fault_retains_original_target_and_feedback(self):
        s=TakeoffSim();s.start();s.until('TAKEOFF')
        p=s.m.core.pose.p;s.m.position=(p[0]+.13,p[1],p[2])
        out=s.m.tick(s.inputs())
        self.assertEqual(out['phase'],'FAULT')
        self.assertIsNone(out['position'])
        fault=copy.deepcopy(out['fault_snapshot'])
        self.assertEqual(fault['phase'],'TAKEOFF')
        self.assertEqual(fault['reason'],'tracking_error')
        self.assertAlmostEqual(fault['tracking']['horizontal_error_m'],.13)
        self.assertEqual(fault['tracking']['actual_position_local_m'],p)
        s.f.p=(p[0]+.01,p[1],p[2]);s.update(follow=False)
        self.assertEqual(s.m.output()['fault_snapshot'],fault)
    def test_idle_diagnostics_have_no_target(self):
        s=TakeoffSim();row=s.m.output()['tracking']
        self.assertIsNone(row['target_position_local_m'])
        self.assertIsNone(row['error_norm_m'])
