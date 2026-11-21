#!/usr/bin/env python3
"""Ground chassis translation with optional bounded continuous-LIO heading hold.

Legacy configurations keep angular.z exactly zero. Explicit heading-hold
profiles add bounded yaw correction while leaving the translation law intact.

Pose source: ``/localization`` (``nav_msgs/Odometry``, ``frame_id: map``),
published by ``slam_navigation/transform_fusion.py`` after ICP alignment of
FAST-LIO's ``/Odometry`` against the prior map.  ``/Odometry`` alone lives in
``camera_init`` and is not map-aligned, so it cannot take map coordinates.
"""
import argparse
import json
import math
import os
import sys
import time
import threading
from functools import wraps

import rospy
import rosgraph
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# This package is pure Python with no build products, so the module must be
# importable straight from the source tree.  catkin's devel space only has it
# after catkin_make has run, and a robot whose workspace is already built for
# other packages should not be reconfigured just to add this one.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SRC = os.path.join(_PKG_ROOT, 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from ground_straight_nav.core import (DEFAULTS, VERSION, StraightRun,  # noqa: E402
                                      check_config, wrap_angle)

NODE = 'ground_straight_nav'
STATUS_TOPIC = '/ground_straight_nav/status'
CMD_TOPIC = '/cmd_vel'
START_SRV = '/ground_straight_nav/start'
STOP_SRV = '/ground_straight_nav/stop'


def serialized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


def quat_to_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    norm = sum(v * v for v in values)
    if not all(math.isfinite(v) for v in values) or not 0.9 <= norm <= 1.1:
        raise ValueError('INVALID_QUATERNION')

    # Prefer the planar form; FAST-LIO quaternions are normalised but the
    # full-atan2 form avoids singularity at pitch = +-90 deg anyway.
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class StraightNavNode(object):
    def __init__(self, cfg, waypoints, pose_topic, rate_hz, allow_non_map=False,
                 dry_run=False):
        self._lock = threading.RLock()
        self._last_stamp = None
        self._pose_frame = None
        self.run = StraightRun(cfg)
        self.waypoints = waypoints
        self.rate_hz = rate_hz
        self.started = False
        self.last_status = 0.0
        self.allow_non_map = allow_non_map
        self.dry_run = dry_run
        self._warned_frame = False
        self._heading_frame = None
        self._heading_pin = None
        self._heading_graph_wall = None

        if dry_run:
            self.cmd_pub = None
        else:
            self.cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1)
        self.zero = Twist()

        rospy.Subscriber(pose_topic, Odometry, self._pose_cb, queue_size=1)
        if cfg['heading_hold_enabled']:
            rospy.Subscriber('/window_pass_lio/Odometry', Odometry, self._heading_cb, queue_size=1)
            self.heading_timer = rospy.Timer(rospy.Duration(0.5), self._heading_identity_tick)
        rospy.Service(START_SRV, Trigger, self._start_srv)
        rospy.Service(STOP_SRV, Trigger, self._stop_srv)

        self.pose_topic = pose_topic
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._tick)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('%s %s %s: pose=%s waypoints=%d max_speed=%.2f m/s',
                      NODE, VERSION, 'DRY-RUN' if dry_run else 'ready',
                      pose_topic, len(waypoints), cfg['max_speed_mps'])
        rospy.loginfo('%s heading hold=%s; yaw abort %.2f rad, rate cap %.3f rad/s',
                      NODE, cfg['heading_hold_enabled'], cfg['yaw_abort_rad'], cfg['heading_max_rate_rad_s'])
        if dry_run:
            rospy.loginfo('%s dry-run publishes no command', NODE)
        if allow_non_map:
            rospy.logwarn('%s accepting non-map pose frames; map goals will be '
                          'interpreted in that frame', NODE)

    @serialized
    def _pose_cb(self, msg):
        frame = msg.header.frame_id
        if frame != 'map' and not self.allow_non_map:
            # Silently accepting camera_init would make map goals meaningless.
            if not self._warned_frame:
                rospy.logwarn('%s ignoring pose in frame %r (map required; '
                              'use --allow-non-map-frame only if that frame '
                              'really is the map frame)', NODE, frame)
                self._warned_frame = True
            return
        try:
            stamp = msg.header.stamp.to_sec()
            age = rospy.Time.now().to_sec() - stamp
            if not math.isfinite(stamp) or stamp <= 0 or not -0.1 <= age <= self.run.cfg['pose_timeout_s']:
                raise ValueError('POSE_STAMP_STALE')
            if self._last_stamp is not None and stamp <= self._last_stamp:
                return  # repeated fusion output cannot refresh the watchdog
            if self._pose_frame is not None and frame != self._pose_frame:
                raise ValueError('POSE_FRAME_CHANGED')
            p = msg.pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                raise ValueError('POSE_NOT_FINITE')
            yaw = quat_to_yaw(msg.pose.pose.orientation)
        except (ValueError, AttributeError) as exc:
            if self.run.state in ('RUNNING', 'DWELL'):
                self.run.abort(str(exc))
                self._publish_zero()
            return
        self._last_stamp = stamp
        self._pose_frame = frame
        self.run.pose_update(p.x, p.y, yaw, time.monotonic())
        if self.run.state == 'ABORT':
            self._publish_zero()

    @serialized
    def _heading_cb(self, msg):
        try:
            if (getattr(msg, '_connection_header', None) or {}).get('callerid') != '/window_pass_lio/laserMapping':
                raise ValueError('HEADING_SOURCE_CHANGED')
            frame = (msg.header.frame_id, getattr(msg, 'child_frame_id', ''))
            if self._heading_frame is not None and frame != self._heading_frame:
                raise ValueError('HEADING_FRAME_CHANGED')
            stamp = msg.header.stamp.to_sec()
            if not math.isfinite(stamp) or stamp <= 0 or not -.02 <= rospy.Time.now().to_sec() - stamp <= self.run.cfg['heading_timeout_s']:
                raise ValueError('HEADING_STAMP_STALE')
            heading = quat_to_yaw(msg.pose.pose.orientation)
            self._heading_frame = frame
            self.run.heading_update(heading, stamp, time.monotonic())
        except (ValueError, AttributeError) as exc:
            self.run.heading.invalidate(str(exc))
            if self.run.state in ('RUNNING', 'DWELL'):
                self.run.abort(str(exc))
        if self.run.state == 'ABORT':
            self._publish_zero()

    def _heading_identity(self):
        master = rosgraph.Master(rospy.get_name())
        names = dict(master.getSystemState()[0]).get('/window_pass_lio/Odometry', [])
        if names != ['/window_pass_lio/laserMapping']:
            raise ValueError('HEADING_PUBLISHER_AMBIGUOUS')
        run_id = rospy.get_param('/window_pass_session_launch/run_id', '')
        if not run_id:
            raise ValueError('HEADING_RUN_ID_MISSING')
        return (names[0], master.lookupNode(names[0]), run_id)

    def _heading_identity_tick(self, _event):
        with self._lock:
            if self.run.state not in ('RUNNING', 'DWELL'):
                return
            pin = self._heading_pin
        try:
            if self._heading_identity() != pin:
                raise ValueError('HEADING_IDENTITY_CHANGED')
        except Exception as exc:
            with self._lock:
                if self._heading_pin == pin and self.run.state in ('RUNNING', 'DWELL'):
                    self.run.abort(str(exc))
                    self._publish_zero()
            return
        with self._lock:
            if self._heading_pin == pin:
                self._heading_graph_wall = time.monotonic()

    @serialized
    def _start_srv(self, _req):
        try:
            if self.run.state in ('RUNNING', 'DWELL'):
                raise ValueError('START_REJECTED_IN_STATE:' + self.run.state)
            if self.run.pose is not None and not self.run._pose_fresh(time.monotonic()):
                raise ValueError('START_REJECTED_STALE_POSE')
            if not self.dry_run:
                conflicts = other_cmd_vel_publishers()
                if conflicts:
                    raise ValueError('CMD_VEL_CONFLICT:' + ','.join(conflicts))
            if self.run.cfg['heading_hold_enabled']:
                self._heading_pin = self._heading_identity()
                self._heading_graph_wall = time.monotonic()
            self.run.start(time.monotonic())
            self.run.begin(self.waypoints)
            self.started = True
            return TriggerResponse(True, '%s %s' % (self.run.state, self.run.reason))
        except ValueError as exc:
            return TriggerResponse(False, str(exc))

    @serialized
    def _stop_srv(self, _req):
        self.run.abort('STOP_REQUESTED')
        self.started = False
        self._publish_zero()
        return TriggerResponse(True, 'STOPPED')

    def _publish_zero(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(self.zero)

    @serialized
    def _tick(self, _event):
        now = time.monotonic()
        if self.started and self.run.state in ('RUNNING', 'DWELL'):
            if self.run.cfg['heading_hold_enabled'] and (self._heading_graph_wall is None or now - self._heading_graph_wall > 1.5):
                self.run.abort('HEADING_IDENTITY_STALE')
            vx, vy = self.run.update(now)
        else:
            self.run.update(now)
            vx, vy = 0.0, 0.0
        if self.run.state in ('DONE', 'ABORT'):
            # Terminal status must survive long enough for the flight handoff.
            # Identity polling stops at completion; its watchdog must stop too.
            self.started = False
            vx, vy = 0.0, 0.0
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = self.run.yaw_command if self.started else 0.0
        if self.cmd_pub is not None:
            self.cmd_pub.publish(cmd)
        if now - self.last_status >= 0.2:
            self.last_status = now
            summary = self.run.summary()
            if self.dry_run:
                summary['dry_run'] = True
            self.status_pub.publish(json.dumps(summary, sort_keys=True))

    @serialized
    def _shutdown(self):
        self.run.abort('NODE_SHUTDOWN')
        for _ in range(5):
            self._publish_zero()
            time.sleep(0.02)


def other_cmd_vel_publishers():
    """Fail closed if master cannot establish exclusive command ownership."""
    try:
        master = rosgraph.Master(rospy.get_name())
        pubs = master.getSystemState()[0]
        topic_name = rospy.resolve_name(CMD_TOPIC)
        return sorted({name for topic, nodes in pubs if topic == topic_name
                       for name in nodes if name != rospy.get_name()})
    except Exception as exc:
        return ['<rosgraph unavailable: %s>' % exc]


def topics_with_publishers():
    try:
        master = rosgraph.Master(rospy.get_name())
        return {topic for topic, nodes in master.getSystemState()[0] if nodes}
    except Exception:
        return set()


def pick_pose_topic(requested, waypoints, allow_non_map):
    """Resolve ``auto`` to a topic, refusing an unaligned fallback for map goals.

    ``/Odometry`` lives in ``camera_init``, so it can only serve waypoints that
    are all ``relative`` — those are frozen against the start pose and never
    need a map-aligned frame.  ``map`` waypoints require ``/localization``.
    """
    if requested != 'auto':
        return requested, allow_non_map
    live = topics_with_publishers()
    if '/localization' in live:
        return '/localization', allow_non_map
    all_relative = all('relative' in wp and 'map' not in wp for wp in waypoints)
    if '/Odometry' in live and all_relative:
        rospy.logwarn('%s /localization absent; falling back to /Odometry in '
                      'camera_init. All waypoints are relative, so this stays '
                      'valid, but distances are measured in the LiDAR start '
                      'frame.', NODE)
        return '/Odometry', True
    if '/Odometry' in live:
        raise SystemExit(
            '%s: /localization absent and %d waypoint(s) use map coordinates. '
            'Finish ICP alignment (global_localization) or switch those '
            'waypoints to relative.' % (NODE, len(waypoints)))
    raise SystemExit('%s: neither /localization nor /Odometry is publishing; '
                     'start FAST-LIO first.' % NODE)


def load_yaml(path):
    try:
        import yaml
    except ImportError:
        raise SystemExit('%s: PyYAML missing on this machine' % NODE)
    with open(path, 'r') as handle:
        return yaml.safe_load(handle) or {}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='',
                        help='YAML with waypoints and parameter overrides')
    parser.add_argument('--pose-topic', default='/localization',
                        help="odometry topic, or 'auto' to prefer "
                             "/localization and fall back to /Odometry")
    parser.add_argument('--allow-non-map-frame', action='store_true',
                        help='accept pose frames other than map (only when '
                             'that frame really is map-aligned)')
    parser.add_argument('--dry-run', action='store_true',
                        help='compute and print, publish nothing')
    args = parser.parse_args(rospy.myargv(argv=[sys.argv[0]] + argv if argv is not None else None)[1:])

    raw = load_yaml(args.config) if args.config else {}
    waypoints = raw.pop('waypoints', None)
    rate_hz = float(raw.get('control_rate_hz', DEFAULTS['control_rate_hz']))
    try:
        cfg = check_config(raw)
    except ValueError as exc:
        raise SystemExit('%s: bad config: %s' % (NODE, exc))
    if not waypoints:
        raise SystemExit('%s: config has no waypoints' % NODE)

    rospy.init_node(NODE, anonymous=False)
    if not args.dry_run and rospy.get_param('/use_sim_time', False):
        raise SystemExit('real chassis control requires /use_sim_time=false')
    pose_topic, allow_non_map = pick_pose_topic(args.pose_topic, waypoints,
                                                args.allow_non_map_frame)
    StraightNavNode(cfg, waypoints, pose_topic, rate_hz,
                    allow_non_map=allow_non_map,
                    dry_run=args.dry_run)
    rospy.spin()
    return 0


if __name__ == '__main__':
    sys.exit(main())
