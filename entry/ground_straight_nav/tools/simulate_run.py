#!/usr/bin/env python3
"""Offline closed-loop run of the heading-frozen controller.

Simulates a perfect holonomic plant (no slip, no yaw disturbance) and, with
``--yaw-drift``, a plant that slowly rotates during translation.  Prints the
trajectory summary and the heading evidence, so the control law and the
waypoint resolution can be reviewed without ROS or hardware.

    python3 tools/simulate_run.py --config config/straight_run.yaml \
        --start-x 0 --start-y 0 --start-yaw 0
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from ground_straight_nav.core import (STATE_ABORT, STATE_DONE, StraightRun,
                                      check_config, wrap_angle)


def load_config(path):
    try:
        import yaml
    except ImportError:
        raise SystemExit('PyYAML required')
    with open(path, 'r') as handle:
        return yaml.safe_load(handle) or {}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--start-x', type=float, default=0.0)
    parser.add_argument('--start-y', type=float, default=0.0)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--yaw-drift', type=float, default=0.0,
                        help='rad/s of unwanted rotation, to show the abort')
    parser.add_argument('--slip', type=float, default=1.0,
                        help='achieved/commanded speed ratio')
    parser.add_argument('--max-seconds', type=float, default=120.0)
    parser.add_argument('--every', type=float, default=1.0,
                        help='print a row every N simulated seconds')
    args = parser.parse_args(argv)

    raw = load_config(args.config)
    waypoints = raw.pop('waypoints', None)
    if not waypoints:
        raise SystemExit('config has no waypoints')
    try:
        cfg = check_config(raw)
    except ValueError as exc:
        raise SystemExit('bad config: %s' % exc)

    run = StraightRun(cfg)
    dt = 1.0 / cfg['control_rate_hz']
    x, y, yaw = args.start_x, args.start_y, args.start_yaw
    if cfg['heading_hold_enabled']:
        run.heading_update(yaw, 1.0, 0.0)
    run.pose_update(x, y, yaw, 0.0)
    run.start()
    run.begin(waypoints)
    goal_text = ', '.join('(%.3f, %.3f)' % p for p in run.waypoints)
    print('start   : (%.3f, %.3f) yaw %.4f rad' % (x, y, yaw))
    print('goals   : %s' % goal_text)
    print('limits  : vmax %.2f m/s, accel %.2f m/s2, yaw_abort %.3f rad'
          % (cfg['max_speed_mps'], cfg['accel_limit_mps2'],
             cfg['yaw_abort_rad']))
    if args.yaw_drift:
        print('plant   : %.3f rad/s unwanted rotation, slip %.2f'
              % (args.yaw_drift, args.slip))
    print()
    print('%7s %8s %8s %8s %8s %8s %s' % ('t', 'x', 'y', 'yaw', 'vx', 'vy',
                                          'state'))

    t = 0.0
    next_print = 0.0
    closest = [float('inf')] * len(run.waypoints)
    while t < args.max_seconds:
        vx, vy = run.update(t)
        if t >= next_print or run.state in (STATE_DONE, STATE_ABORT):
            print('%7.2f %8.3f %8.3f %8.4f %8.3f %8.3f %s %s'
                  % (t, x, y, yaw, vx, vy, run.state, run.reason))
            next_print += args.every
        if run.state in (STATE_DONE, STATE_ABORT):
            break
        yaw = wrap_angle(yaw + (args.yaw_drift + run.yaw_command) * dt)
        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt * args.slip
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt * args.slip
        t += dt
        if cfg['heading_hold_enabled']:
            run.heading_update(yaw, 1.0 + t, t)
        run.pose_update(x, y, yaw, t)
        goal = run.target()
        if goal is not None:
            reach = math.hypot(x - goal[0], y - goal[1])
            closest[run.index] = min(closest[run.index], reach)

    summary = run.summary()
    print()
    print('final   : state=%s reason=%s' % (summary['state'], summary['reason']))
    print('heading : start %.4f rad, final %.4f rad, peak drift %.4f rad'
          % (summary['start_yaw'], yaw, summary['max_yaw_drift_seen']))
    print('travel  : %.3f m' % summary['distance_travelled_m'])
    print('wz      : %.4f rad/s; heading hold %s' % (summary['command']['wz'], cfg['heading_hold_enabled']))
    print('accuracy: %s'
          % ', '.join('wp%d closest %.4f m' % (i, d)
                      for i, d in enumerate(closest) if d != float('inf')))
    overshoot = [i for i, d in enumerate(closest)
                 if d > cfg['arrive_tolerance_m'] and d != float('inf')]
    if overshoot:
        print('WARNING : waypoints never approached within tolerance: %s'
              % overshoot)
    ok = (summary['state'] == STATE_DONE and
          summary['max_yaw_drift_seen'] <= cfg['yaw_abort_rad'] and
          not overshoot)
    print('RESULT  : %s' % ('PASS' if ok else 'NOT PASSED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
