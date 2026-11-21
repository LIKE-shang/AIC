#!/usr/bin/env python3
"""Offline tests for the heading-frozen ground run. No ROS required."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src'))

from ground_straight_nav.core import (DEFAULTS, STATE_ABORT, STATE_DONE,
                                      STATE_DWELL, STATE_READY, STATE_RUNNING,
                                      STATE_WAITING_POSE, StraightRun,
                                      check_config, limit_step,
                                      resolve_waypoints, wrap_angle)


def make_run(**overrides):
    cfg = dict(DEFAULTS)
    cfg.update(overrides)
    return StraightRun(cfg)


def simulate(run, waypoints, start=(0.0, 0.0), yaw=0.0, seconds=30.0,
             yaw_drift_per_s=0.0, plant_gain=1.0):
    """Drive the state machine against a kinematic plant.

    Returns (history, elapsed). ``history`` holds (t, x, y, yaw, vx, vy).
    """
    dt = 1.0 / run.cfg['control_rate_hz']
    x, y, cur_yaw = start[0], start[1], yaw
    run.pose_update(x, y, cur_yaw, 0.0)
    run.start()
    run.begin(waypoints)
    history = []
    t = 0.0
    while t < seconds:
        vx, vy = run.update(t)
        history.append((t, x, y, cur_yaw, vx, vy))
        if run.state in (STATE_DONE, STATE_ABORT):
            run.pose_update(x, y, cur_yaw, t)
            break
        cur_yaw = wrap_angle(cur_yaw + yaw_drift_per_s * dt)
        x += (math.cos(cur_yaw) * vx - math.sin(cur_yaw) * vy) * dt * plant_gain
        y += (math.sin(cur_yaw) * vx + math.cos(cur_yaw) * vy) * dt * plant_gain
        t += dt
        run.pose_update(x, y, cur_yaw, t)
    return history, t


class TestNoRotationEver(unittest.TestCase):
    def test_command_has_no_angular_attribute_by_construction(self):
        run = make_run()
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [1.0, 0.5]}])
        vx, vy = run.update(0.0)
        self.assertIsInstance(vx, float)
        self.assertIsInstance(vy, float)
        # summary exposes wz and it is a literal zero
        self.assertEqual(run.summary()['command']['wz'], 0.0)
        self.assertTrue(run.summary()['angular_z_hardwired_zero'])

    def test_cross_track_goal_is_reached_without_turning(self):
        run = make_run(max_speed_mps=0.3, arrive_tolerance_m=0.02)
        history, elapsed = simulate(run, [{'map': [1.0, 0.6]}], seconds=40.0)
        self.assertEqual(run.state, STATE_DONE, run.reason)
        # Yaw never moved: the plant had no way to rotate it, and the node
        # commanded no rotation.
        for _t, _x, _y, yaw, _vx, _vy in history:
            self.assertAlmostEqual(wrap_angle(yaw), 0.0, places=9)
        final_x, final_y = history[-1][1], history[-1][2]
        self.assertAlmostEqual(final_x, 1.0, delta=0.03)
        self.assertAlmostEqual(final_y, 0.6, delta=0.03)
        self.assertLess(elapsed, 40.0)

    def test_lateral_only_goal_uses_vy(self):
        run = make_run(max_speed_mps=0.2)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [0.0, 0.5]}])
        vx, vy = run.update(0.0)
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertGreater(vy, 0.0)

    def test_command_respects_speed_cap(self):
        run = make_run(max_speed_mps=0.12, accel_limit_mps2=3.0)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [3.0, 0.0]}])
        for step in range(200):
            vx, vy = run.update(step * 0.05)
            self.assertLessEqual(math.hypot(vx, vy), 0.12 + 1e-9)

    def test_no_summary_field_can_carry_angular_velocity(self):
        run = make_run()
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [1.0, 0.0]}])
        run.update(0.0)
        summary = run.summary()
        # Only the explicit wz field exists, and it is a literal zero.
        angular = [k for k in summary if 'angular' in k or 'theta' in k]
        self.assertEqual(sorted(angular), ['angular_z_hardwired_zero'])
        self.assertNotIn('wz', {k for k in summary if k != 'command'})


class TestRestart(unittest.TestCase):
    def test_restart_relatches_heading(self):
        run = make_run(arrive_cycles=1, dwell_s=0.0)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [0.0, 0.0]}])
        for step in range(5):
            run.pose_update(0.0, 0.0, 0.0, step * 0.05)
            run.update(step * 0.05)
        self.assertEqual(run.state, STATE_DONE)
        # The chassis is now elsewhere and facing another way.
        run.pose_update(1.5, 0.4, 0.9, 10.0)
        run.start()
        self.assertAlmostEqual(run.start_yaw, 0.9, places=9)
        self.assertAlmostEqual(run.start_xy[0], 1.5, places=9)
        self.assertEqual(run.max_yaw_drift_seen, 0.0)
        run.begin([{'relative': [0.5, 0.0]}])
        expected = (1.5 + 0.5 * math.cos(0.9), 0.4 + 0.5 * math.sin(0.9))
        self.assertAlmostEqual(run.waypoints[0][0], expected[0], places=9)
        self.assertAlmostEqual(run.waypoints[0][1], expected[1], places=9)


class TestHeadingInvariant(unittest.TestCase):
    def test_aborts_when_yaw_drifts_past_limit(self):
        run = make_run(yaw_abort_rad=0.15)
        history, _elapsed = simulate(run, [{'map': [5.0, 0.0]}],
                                     yaw_drift_per_s=0.5, seconds=10.0)
        self.assertEqual(run.state, STATE_ABORT)
        self.assertTrue(run.reason.startswith('YAW_DRIFT_ABORT'), run.reason)
        # The abort happens with a zeroed command, never with a correction.
        self.assertEqual(history[-1][4], 0.0)
        self.assertEqual(history[-1][5], 0.0)
        self.assertGreater(run.max_yaw_drift_seen, 0.15)

    def test_small_drift_warns_but_continues(self):
        run = make_run(yaw_warn_rad=0.05, yaw_abort_rad=0.15,
                       arrive_tolerance_m=0.02)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [0.5, 0.0]}])
        run.pose_update(0.0, 0.0, 0.08, 0.0)
        run.update(0.0)
        self.assertEqual(run.state, STATE_RUNNING)
        self.assertTrue(run.status_warn.startswith('YAW_DRIFT_WARN'))

    def test_start_yaw_is_latched_not_re_read(self):
        run = make_run()
        run.pose_update(0.0, 0.0, 0.7, 0.0)
        run.start()
        run.begin([{'relative': [1.0, 0.0]}])
        # Waypoint resolved in the latched frame: 1 m along yaw 0.7.
        self.assertAlmostEqual(run.waypoints[0][0], math.cos(0.7), places=9)
        self.assertAlmostEqual(run.waypoints[0][1], math.sin(0.7), places=9)


class TestSafetyGates(unittest.TestCase):
    def test_stale_pose_zeroes_command(self):
        run = make_run(pose_timeout_s=0.5)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [2.0, 0.0]}])
        run.update(0.0)
        vx, vy = run.update(10.0)  # pose is now 10 s old
        self.assertEqual((vx, vy), (0.0, 0.0))
        self.assertEqual(run.reason, 'POSE_STALE')

    def test_geofence_aborts(self):
        run = make_run(max_run_distance_m=0.5)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [2.0, 0.0]}])
        for i in range(1, 7):
            run.pose_update(i * 0.1, 0.0, 0.0, i * 0.2)
            run.update(i * 0.2)
        self.assertEqual(run.state, STATE_ABORT)
        self.assertEqual(run.reason, 'GEOFENCE_ABORT')

    def test_waypoint_timeout_aborts(self):
        run = make_run(waypoint_timeout_s=1.0)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [5.0, 0.0]}])
        # The chassis is blocked, so the pose never changes but stays fresh.
        for step in range(40):
            now = step * 0.1
            run.pose_update(0.0, 0.0, 0.0, now)
            run.update(now)
            if run.state == STATE_ABORT:
                break
        self.assertEqual(run.state, STATE_ABORT)
        self.assertTrue(run.reason.startswith('WAYPOINT_TIMEOUT'), run.reason)

    def test_stale_pose_outranks_timeout(self):
        # Losing the pose stops the run before the waypoint clock matters.
        run = make_run(waypoint_timeout_s=1.0, pose_timeout_s=0.5)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [5.0, 0.0]}])
        self.assertEqual(run.update(2.0), (0.0, 0.0))
        self.assertEqual(run.reason, 'POSE_STALE')
        self.assertEqual(run.state, STATE_ABORT)

    def test_no_pose_means_no_motion(self):
        run = make_run()
        self.assertEqual(run.state, STATE_WAITING_POSE)
        vx, vy = run.update(0.0)
        self.assertEqual((vx, vy), (0.0, 0.0))
        self.assertRaises(ValueError, run.start)

    def test_arrival_holds_then_advances(self):
        run = make_run(arrive_cycles=3, dwell_s=0.2)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [0.005, 0.0]}, {'map': [0.02, 0.0]}])
        states = set()
        for step in range(40):
            now = step * 0.05
            run.pose_update(0.0, 0.0, 0.0, now)
            run.update(now)
            states.add(run.state)
        self.assertIn(STATE_DWELL, states)
        self.assertIn(STATE_DONE, states)
        self.assertEqual(run.reason, 'ALL_WAYPOINTS_REACHED')

    def test_stop_aborts_immediately(self):
        run = make_run()
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [3.0, 0.0]}])
        run.update(0.0)
        run.abort('STOP_REQUESTED')
        self.assertEqual(run.command, (0.0, 0.0))
        self.assertEqual(run.update(0.1), (0.0, 0.0))


class TestConfig(unittest.TestCase):
    def test_rejects_unknown_key(self):
        self.assertRaises(ValueError, check_config, {'no_such_key': 1})

    def test_rejects_out_of_range(self):
        self.assertRaises(ValueError, check_config, {'max_speed_mps': 9.9})

    def test_rejects_warn_above_abort(self):
        self.assertRaises(ValueError, check_config,
                          {'yaw_warn_rad': 0.3, 'yaw_abort_rad': 0.1})

    def test_accepts_defaults(self):
        cfg = check_config({})
        self.assertEqual(cfg['max_speed_mps'], DEFAULTS['max_speed_mps'])


class TestWaypoints(unittest.TestCase):
    def test_relative_rotates_with_start_yaw(self):
        pts = resolve_waypoints([{'relative': [1.0, 0.0]}], (2.0, 3.0),
                                math.pi / 2.0)
        self.assertAlmostEqual(pts[0][0], 2.0, places=9)
        self.assertAlmostEqual(pts[0][1], 4.0, places=9)

    def test_absolute_passthrough(self):
        pts = resolve_waypoints([{'map': [1.5, -2.5]}], (0.0, 0.0), 0.0)
        self.assertEqual(pts[0], (1.5, -2.5))

    def test_rejects_missing_frame_key(self):
        self.assertRaises(ValueError, resolve_waypoints, [{'x': 1}], (0, 0), 0)

    def test_rejects_empty(self):
        self.assertRaises(ValueError, resolve_waypoints, [], (0, 0), 0)

    def test_rejects_non_finite(self):
        self.assertRaises(ValueError, resolve_waypoints,
                          [{'map': [float('nan'), 0.0]}], (0, 0), 0)


class TestSlew(unittest.TestCase):
    def test_limit_step_caps_vector_length(self):
        out = limit_step((0.0, 0.0), (0.3, 0.4), 0.5)
        self.assertAlmostEqual(out[0], 0.3, places=9)
        self.assertAlmostEqual(out[1], 0.4, places=9)
        out = limit_step((0.0, 0.0), (0.3, 0.4), 0.25)
        self.assertAlmostEqual(math.hypot(*out), 0.25, places=9)

    def test_first_command_is_slew_limited_from_zero(self):
        run = make_run(accel_limit_mps2=0.2, control_rate_hz=20.0,
                       max_speed_mps=0.4, pos_gain_per_s=5.0)
        run.pose_update(0.0, 0.0, 0.0, 0.0)
        run.start()
        run.begin([{'map': [3.0, 0.0]}])
        vx, _vy = run.update(0.0)
        self.assertLessEqual(vx, 0.2 / 20.0 + 1e-9)


class TestAuditRegressions(unittest.TestCase):
    def test_nonzero_heading_closed_loop(self):
        for yaw in (math.pi/2, -1.2, 3.0):
            run = make_run()
            simulate(run, [{'relative': [0.6, 0.4]}], yaw=yaw, seconds=30)
            self.assertEqual(run.state, STATE_DONE, (yaw, run.reason))

    def test_small_yaw_drift_still_commands_towards_map_goal(self):
        run = make_run(accel_limit_mps2=3, decel_limit_mps2=3)
        run.pose_update(0, 0, 0, 0)
        run.start()
        run.begin([{'map': [1, 0]}])
        run.pose_update(0, 0, 0.1, 0.1)
        vx, vy = run.update(0.1)
        self.assertAlmostEqual(math.sin(0.1)*vx + math.cos(0.1)*vy, 0)

    def test_stale_pose_does_not_auto_resume(self):
        run = make_run()
        run.pose_update(0, 0, 0, 0)
        run.start()
        run.begin([{'relative': [1, 0]}])
        run.update(1)
        run.pose_update(0, 0, 0, 1.1)
        self.assertEqual(run.update(1.1), (0, 0))
        self.assertEqual(run.state, STATE_ABORT)

    def test_jump_aborts(self):
        run = make_run()
        run.pose_update(0, 0, 0, 0)
        run.start()
        run.begin([{'map': [1, 0]}])
        run.pose_update(1, 0, 0, 0.05)
        self.assertEqual(run.state, STATE_ABORT)
        self.assertEqual(run.command, (0, 0))

    def test_one_pose_cannot_count_as_multiple_arrival_samples(self):
        run = make_run(arrive_cycles=3)
        run.pose_update(0, 0, 0, 0)
        run.start()
        run.begin([{'map': [0, 0]}])
        for _ in range(5):
            run.update(0.1)
        self.assertEqual(run.state, STATE_RUNNING)
        self.assertEqual(run.hold_count, 1)

    def test_dwell_drift_does_not_finish(self):
        run = make_run(arrive_cycles=1, dwell_s=0.1)
        run.pose_update(0, 0, 0, 0)
        run.start()
        run.begin([{'map': [0, 0]}])
        run.update(0)
        run.pose_update(0.05, 0, 0, 0.2)
        run.update(0.2)
        self.assertEqual(run.state, STATE_RUNNING)


if __name__ == '__main__':
    unittest.main()
