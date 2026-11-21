"""Intermediate handoff timing, fresh-position confirmation and final gates."""
import math
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from ground_straight_nav.core import StraightRun, check_config, wrap_angle


class FastIntermediateTests(unittest.TestCase):
    def make_run(self, degrees=4.0):
        r = StraightRun(dict(
            heading_hold_enabled=True, fast_intermediate_enabled=True,
            max_speed_mps=1.5, accel_limit_mps2=.8, decel_limit_mps2=.8,
            arrive_tolerance_m=.05, arrive_cycles=5, dwell_s=.2,
            yaw_abort_rad=math.radians(10), heading_settle_timeout_s=2.,
            heading_kp=2., heading_kd=.4,
            heading_max_rate_rad_s=math.radians(10),
            heading_accel_rad_s2=math.radians(20)))
        r.pose_update(0, 0, 0, 0)
        r.heading_update(0, 1, 0)
        r.start(0)
        r.begin([{'map':[.2,0]}, {'map':[1.,0]}])
        for i in range(1, 5):
            self.feed(r, i*.05, 0., degrees*i/4.)
        return r

    def feed(self, r, t, x, degrees):
        yaw = math.radians(degrees)
        r.heading_update(yaw, 1+t, t)
        r.pose_update(x, 0, yaw, t)
        return r.update(t)

    def test_small_error_advances_without_dwell_and_keeps_reference(self):
        r = self.make_run()
        reference = r.heading.reference
        self.feed(r, .25, .2, 4.)
        self.assertEqual(r.index, 0)
        self.feed(r, .35, .2, 4.)
        self.assertEqual((r.index, r.state), (1, 'RUNNING'))
        self.assertIsNone(r.dwell_until)
        self.assertEqual(r.heading.reference, reference)
        self.assertLess(r.yaw_command, 0.)
        vx, vy = self.feed(r, .4, .2, 4.)
        self.assertGreater(math.hypot(vx, vy), 0.)

    def test_repeated_control_tick_cannot_confirm_position(self):
        r = self.make_run()
        self.feed(r, .25, .2, 4.)
        r.update(.3)
        self.assertEqual(r.index, 0)
        self.assertEqual(r.hold_count, 1)

    def test_large_error_defers_after_half_second(self):
        r = self.make_run(8.)
        for i in range(10):
            self.feed(r, .25+i*.05, .2, 8.)
            self.assertEqual(r.index, 0)
        self.feed(r, .8, .2, 8.)
        self.assertEqual(r.index, 1)
        self.assertEqual(r.heading_deferred_count, 1)
        self.assertLess(r.yaw_command, 0.)

    def test_final_point_keeps_two_degree_gate_and_bounded_wait(self):
        r = self.make_run()
        r.begin([{'map':[.2,0]}])
        for i in range(42):
            self.feed(r, .25+i*.05, .2, 4.)
            if r.state == 'ABORT': break
        self.assertEqual(r.reason, 'HEADING_SETTLE_TIMEOUT')
        self.assertEqual(r.index, 0)
        self.assertEqual(r.yaw_command, 0.)

    def test_final_requires_original_position_confirmation_and_dwell(self):
        r = self.make_run(0.)
        r.begin([{'map':[.2,0]}])
        for i in range(4):
            self.feed(r, .25+i*.1, .2, 0.)
            self.assertEqual(r.state, 'RUNNING')
        self.feed(r, .65, .2, 0.)
        self.assertEqual(r.state, 'DWELL')
        self.feed(r, .8, .2, 0.)
        self.assertEqual(r.state, 'DWELL')
        self.feed(r, .9, .2, 0.)
        self.assertEqual(r.state, 'DONE')
        self.assertEqual(r.yaw_command, 0.)

    def test_ten_degree_abort_still_precedes_intermediate_handoff(self):
        r = self.make_run(8.)
        self.feed(r, .25, .2, 9.)
        self.feed(r, .35, .2, 10.1)
        self.assertEqual(r.state, 'ABORT')
        self.assertTrue(r.reason.startswith('YAW_DRIFT_ABORT'))
        self.assertEqual(r.index, 0)

    def test_leaving_position_resets_confirmation_and_wait(self):
        r = self.make_run(8.)
        self.feed(r, .25, .2, 8.)
        self.feed(r, .35, .1, 8.)
        self.assertIsNone(r.arrival_started)
        self.feed(r, .45, .2, 8.)
        self.feed(r, .55, .2, 8.)
        self.assertEqual(r.index, 0)

    def test_invalid_policy_rejected(self):
        for config in [dict(fast_intermediate_enabled=1),
                       dict(fast_intermediate_enabled=True),
                       dict(intermediate_arrive_cycles=1.5),
                       dict(heading_hold_enabled=True, fast_intermediate_enabled=True,
                            intermediate_heading_rad=.2)]:
            with self.assertRaises(ValueError): check_config(config)

    def test_four_points_with_strong_translation_disturbance(self):
        for sign in (-1, 1):
            r = self.make_run(0.)
            r.cfg['max_run_distance_m'] = 4.
            r.begin([{'map':[0,-.85]}, {'map':[.65,-1.85]},
                     {'map':[0,-2.9]}, {'map':[0,0]}])
            x=y=yaw=0.; t=.25
            peak=0.
            for step in range(1600):
                r.pose_update(x,y,yaw,t)
                if step%2 == 0:r.heading_update(yaw,1+t,t)
                vx,vy=r.update(t)
                peak=max(peak,abs(r.yaw_drift))
                if r.state in ('DONE','ABORT'):break
                disturbance=sign*.12 if math.hypot(vx,vy)>.01 else 0.
                yaw=wrap_angle(yaw+(r.yaw_command+disturbance)*.05)
                x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*.05
                y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*.05
                t+=.05
            self.assertEqual(r.state,'DONE',r.reason)
            self.assertLess(peak,math.radians(10))
            self.assertLess(abs(r.yaw_drift),math.radians(2))
            self.assertLess(math.hypot(x,y),.051)


if __name__ == '__main__': unittest.main()
