"""Reported divergence must not pass fresh/valid FCU flags."""
import unittest
from test_mission import Sim
from hover_localization import HoverContinuity,require_local_origin
from window_pass.core import Pose

class HoverLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.guard=HoverContinuity(Sim().m.core.c)
    def pose(self,p=(0.,0.,0.),stamp=100.):
        return Pose(stamp,p,(0.,0.,0.,1.),'camera_init','body')
    def observe(self,p=(0.,0.,0.),seq=1,stamp=100.):
        return self.guard.observe(self.pose(p,stamp),seq,stamp,.02*seq,'/lio','/lio')
    def test_normal_origin_and_continuity(self):
        self.assertTrue(self.observe())
        self.assertTrue(self.observe((.001,0.,0.),2,100.02))
    def test_fresh_huge_initial_raw_position_rejected(self):
        self.assertFalse(self.observe((1306.,-1083.,-7661.)))
        self.assertIn('origin',self.guard.reason)
        self.assertFalse(self.observe())
    def test_fcu_huge_origin_rejected_with_nominal_raw_pose(self):
        self.assertTrue(self.observe())
        self.assertFalse(self.guard.observe_fcu(self.pose((1306.,-1083.,-7661.))))
        self.assertFalse(self.guard.health(100.,.02)[0])
    def test_raw_jump_latches_even_if_bridge_claims_valid(self):
        self.assertTrue(self.observe())
        self.assertFalse(self.observe((.5,0.,0.),2,100.02))
        self.guard.observe_bridge(dict(real_published=500,real_output_state='FORWARDING_CONTINUOUS_POSE',
             real_fault='',horizontal_flags_valid=True),.04,'/bridge','/bridge')
        self.assertFalse(self.guard.health(100.02,.04)[0])
    def test_unavailable_raw_cannot_pass_bridge_flags(self):
        self.guard.observe_bridge(dict(real_published=1,real_output_state='FORWARDING_CONTINUOUS_POSE',
             real_fault='',horizontal_flags_valid=True),.02,'/bridge','/bridge')
        self.assertEqual(self.guard.health(100.,.02),(False,'lio_pose_unavailable'))
    def test_nonfinite_origin_rejected(self):
        for point in ([float('nan'),0,0],[0,0,float('inf')],None):
            with self.assertRaises(ValueError):require_local_origin(point)

if __name__=='__main__':unittest.main()
