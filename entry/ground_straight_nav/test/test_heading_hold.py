"""Disturbance, source continuity, failure and translation coupling regressions."""
import math
import os
import sys
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from ground_straight_nav.core import StraightRun, check_config, wrap_angle

def run(**overrides):
    cfg = dict(heading_hold_enabled=True, max_speed_mps=1.5,
               accel_limit_mps2=.8, decel_limit_mps2=.8, pos_gain_per_s=.8,
               arrive_tolerance_m=.05, arrive_cycles=5, dwell_s=.2,
               yaw_abort_rad=math.radians(10), max_run_distance_m=30.)
    cfg.update(overrides)
    r = StraightRun(cfg)
    r.pose_update(0., 0., 0., 0.)
    r.heading_update(0., 1., 0.)
    r.start(0.)
    r.begin([{'map': [20., 0.]}])
    return r

class HeadingRegressions(unittest.TestCase):
    def test_correction_direction_and_angular_slew(self):
        for sign in (-1, 1):
            r = run()
            for i in range(1, 7):
                t = i * .05
                r.heading_update(sign * math.radians(4), 1+t, t)
                r.pose_update(0., 0., sign*.07, t)
                old = r.yaw_command
                r.update(t)
                self.assertLess(sign*r.yaw_command, 0)
                self.assertLessEqual(abs(r.yaw_command-old), math.radians(10)*.05+1e-9)

    def test_deadband_does_not_steer(self):
        r = run()
        r.heading_update(math.radians(.5), 1.05, .05)
        r.pose_update(0, 0, .01, .05)
        r.update(.05)
        self.assertEqual(r.yaw_command, 0.)

    def test_map_yaw_jump_does_not_steer_or_trigger_heading_abort(self):
        r = run()
        r.heading_update(0., 1.05, .05)
        r.pose_update(0., 0., math.radians(15), .05)
        r.update(.05)
        self.assertEqual(r.yaw_command, 0.)
        self.assertEqual(r.state, 'RUNNING')
        self.assertAlmostEqual(r.yaw_drift, 0.)

    def test_lio_time_reset_aborts(self):
        r = run()
        r.heading_update(0., .9, .05)
        self.assertEqual(r.reason, 'HEADING_TIME_RESET')
        self.assertEqual(r.yaw_command, 0.)

    def test_heading_jump_aborts(self):
        r = run()
        r.heading_update(.5, 1.05, .05)
        self.assertEqual(r.reason, 'HEADING_DISCONTINUITY')
        self.assertEqual(r.command, (0., 0.))

    def test_missing_heading_zeroes_translation_and_rotation(self):
        r = run()
        r.pose_update(0., 0., 0., .4)
        self.assertEqual(r.update(.4), (0., 0.))
        self.assertEqual(r.reason, 'HEADING_STALE')
        self.assertEqual(r.yaw_command, 0.)

    def test_repeat_heading_does_not_refresh_watchdog(self):
        r = run()
        r.heading_update(0., 1., .3)
        r.pose_update(0., 0., 0., .36)
        r.update(.36)
        self.assertEqual(r.reason, 'HEADING_STALE')

    def test_gap_cannot_be_hidden_by_new_sample(self):
        r = run()
        r.heading_update(0., 1.4, .4)
        self.assertEqual(r.reason, 'HEADING_STALE')

    def test_start_requires_fresh_heading(self):
        r = StraightRun(check_config({'heading_hold_enabled': True}))
        r.pose_update(0., 0., 0., 1.)
        with self.assertRaises(ValueError):r.start(1.)

    def test_relative_heading_wraps_at_pi(self):
        r = StraightRun({'heading_hold_enabled':True})
        r.pose_update(0, 0, 3.13, 0.)
        r.heading_update(3.13, 1., 0.)
        r.start(0.)
        r.begin([{'map':[1.,0.]}])
        r.heading_update(-3.13, 1.1, .1)
        r.pose_update(0, 0, -3.13, .1)
        r.update(.1)
        self.assertLess(abs(r.yaw_drift), .03)
        self.assertEqual(r.state, 'RUNNING')

    def test_original_ten_degree_limit_remains(self):
        r = run()
        for i in range(1, 13):
            t = .05*i
            r.heading_update(math.radians(i), 1+t, t)
            r.pose_update(0, 0, 0, t)
            r.update(t)
            if r.state == 'ABORT':break
        self.assertTrue(r.reason.startswith('YAW_DRIFT_ABORT'), r.reason)
        self.assertEqual(r.yaw_command, 0.)

    def test_stop_and_done_have_zero_angular_command(self):
        for state in ('ABORT','DONE','READY'):
            r = run();r.heading.command=.05;r.state=state
            self.assertEqual(r.yaw_command,0.)
            self.assertEqual(r.update(.05),(0.,0.))

    def test_correction_does_not_reduce_translation_speed_law(self):
        r = run();legacy = run(heading_hold_enabled=False)
        for i in range(1, 41):
            t=i*.05
            r.heading_update(math.radians(4),1+t,t)
            for x in (r,legacy):x.pose_update(0,0,.07,t)
            self.assertEqual(r.update(t),legacy.update(t))
        self.assertAlmostEqual(math.hypot(*r.command),1.5)
        self.assertLessEqual(abs(r.yaw_command),math.radians(5))

    def test_four_points_with_translation_dependent_drift(self):
        for sign in (-1,1):
            r=run(max_run_distance_m=4.)
            r.begin([{'map':[0,-.85]},{'map':[.65,-1.85]},{'map':[0,-2.9]},{'map':[0,0]}])
            x=y=yaw=t=0.;peak=0.
            for step in range(1800):
                vx,vy=r.update(t);peak=max(peak,abs(r.yaw_drift))
                if r.state in ('DONE','ABORT'):break
                disturbance=sign*.025*math.hypot(vx,vy)/1.5
                yaw=wrap_angle(yaw+(r.yaw_command+disturbance)*.05)
                x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*.05
                y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*.05
                t+=.05
                if step%2==1:r.heading_update(yaw,1+t,t)  # true 10 Hz heading
                r.pose_update(x,y,yaw,t)
            self.assertEqual(r.state,'DONE',r.reason)
            self.assertLess(peak,math.radians(3))
            self.assertLess(math.hypot(x,y),.051)

if __name__=='__main__':unittest.main()
