"""Final leg speed cap without changing earlier legs or heading hard limits."""
import math
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from ground_straight_nav.core import StraightRun, check_config


class FinalSpeedTests(unittest.TestCase):
    def make(self, index, **overrides):
        cfg=dict(max_speed_mps=1.5, final_max_speed_mps=.6,
                 accel_limit_mps2=.8, decel_limit_mps2=.8,
                 yaw_abort_rad=math.radians(10), max_run_distance_m=30.)
        cfg.update(overrides)
        r=StraightRun(cfg);r.pose_update(0,0,0,0)
        if r.cfg['heading_hold_enabled']:r.heading_update(0,1,0)
        r.start(0)
        r.begin([{'map':[10,0]}]*4);r.index=index
        return r

    def test_first_three_legs_keep_original_speed(self):
        for index in range(3):
            r=self.make(index)
            for i in range(1,51):
                r.pose_update(0,0,0,i*.05);r.update(i*.05)
            self.assertAlmostEqual(math.hypot(*r.command),1.5)

    def test_final_leg_caps_vector_speed_and_preserves_acceleration(self):
        r=self.make(3)
        r.waypoints[3]=(10,10)
        old=(0.,0.)
        for i in range(1,51):
            r.pose_update(0,0,0,i*.05);v=r.update(i*.05)
            self.assertLessEqual(math.hypot(*v),.6+1e-9)
            self.assertLessEqual(math.hypot(v[0]-old[0],v[1]-old[1]),.8*.05+1e-9)
            old=v
        self.assertAlmostEqual(math.hypot(*r.command),.6)
        self.assertEqual(r.summary()['active_speed_limit_mps'],.6)

    def test_final_cap_never_raises_lower_global_limit(self):
        r=self.make(3,max_speed_mps=.4)
        self.assertEqual(r.active_speed_limit(),.4)

    def test_legacy_default_keeps_existing_global_speed(self):
        self.assertEqual(self.make(3,final_max_speed_mps=1.5).active_speed_limit(),1.5)

    def test_waypoint_advance_selects_final_cap_without_new_reference(self):
        r=self.make(2)
        reference=r.start_yaw
        r._advance_waypoint()
        self.assertEqual(r.index,3)
        self.assertEqual(r.active_speed_limit(),.6)
        self.assertEqual(r.start_yaw,reference)

    def test_larger_heading_output_does_not_relax_ten_degree_abort(self):
        r=self.make(3,heading_hold_enabled=True,heading_max_rate_rad_s=math.radians(20))
        r.heading_update(0,1,0);r.heading.latch(0)
        for i in range(1,12):
            t=i*.05;r.heading_update(math.radians(i),1+t,t)
            r.pose_update(0,0,0,t);r.update(t)
            if r.state=='ABORT':break
        self.assertTrue(r.reason.startswith('YAW_DRIFT_ABORT'))
        self.assertEqual(r.command,(0.,0.))
        self.assertEqual(r.yaw_command,0.)

    def test_final_cap_validation(self):
        for cfg in (dict(final_max_speed_mps=0),dict(final_max_speed_mps=1.6),
                    dict(final_max_speed_mps=.02,min_speed_mps=.03)):
            with self.assertRaises(ValueError):check_config(cfg)


if __name__=='__main__':unittest.main()
