import unittest
from test_position_goal_takeoff import GoalSim
from test_pilot_landing import PilotSim

class TakeoffEvidence(unittest.TestCase):
    def test_timeout_preserves_original_goal_not_handover_hold(self):
        s=PilotSim();s.freeze=True;s.start();s.until('PILOT_CONTROL')
        d=s.out['takeoff_progress']
        self.assertAlmostEqual(d['original_target_position_local_m'][2],s.m.target_z)
        self.assertNotEqual(d['original_target_position_local_m'],s.m.position)
        self.assertIn('height',d['blocking_criteria'])
        self.assertNotIn('LAND',s.actions)
    def test_speed_reset_recorded_without_relaxing_gate(self):
        s=GoalSim();s.start();s.until('TAKEOFF')
        s.m.stable_since=s.f.wall-.8;s.m.core.pose_speed=.041
        s.m.update_takeoff_progress(s.inputs())
        d=s.m.output()['takeoff_progress']
        self.assertIn('speed',d['blocking_criteria'])
        self.assertEqual(d['stable_speed_limit_mps'],.04)
        self.assertEqual(d['stability_reset_count'],1)
        self.assertGreaterEqual(d['maximum_stable_duration_s'],.8-1e-8)
