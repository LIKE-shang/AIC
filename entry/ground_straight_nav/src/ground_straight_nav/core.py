"""Holonomic translation with optional bounded relative-LIO heading hold.

Zero commanded rotation does not guarantee physical heading invariance.
Measured heading drift is monitored and aborts above yaw_abort_rad.
Map goals are fixed at start; linear velocities use the live body frame.
"""
from __future__ import division

import math

VERSION = '20261025_ground_heading_hold_v3_return060'

DEFAULTS = {
    # Command shaping. Ground chassis, no reason to move fast.
    'max_speed_mps': 0.20,
    'final_max_speed_mps': 1.50,
    'min_speed_mps': 0.03,
    'pos_gain_per_s': 1.2,
    'accel_limit_mps2': 0.5,
    'decel_limit_mps2': 0.5,
    # Arrival. Tolerance is a position radius, cycles must hold it.
    'arrive_tolerance_m': 0.02,
    'arrive_cycles': 5,
    'dwell_s': 0.3,
    # Time and freshness limits.
    'pose_timeout_s': 0.5,
    'waypoint_timeout_s': 45.0,
    'control_rate_hz': 20.0,
    # Heading invariant. Warn first, abort when it is actually broken.
    'yaw_warn_rad': 0.05,
    'yaw_abort_rad': 0.15,
    'heading_hold_enabled': False,
    'heading_kp': 1.2,
    'heading_kd': 0.3,
    'heading_deadband_rad': math.radians(1.0),
    'heading_arrive_rad': math.radians(2.0),
    'heading_max_rate_rad_s': math.radians(5.0),
    'heading_accel_rad_s2': math.radians(10.0),
    'heading_timeout_s': 0.35,
    'heading_jump_rad': math.radians(5.0),
    'heading_rate_guard_rad_s': 0.5,
    'heading_rate_filter_s': 0.2,
    'heading_settle_timeout_s': 8.0,
    'fast_intermediate_enabled': False,
    'intermediate_arrive_cycles': 2,
    'intermediate_heading_rad': math.radians(5.0),
    'intermediate_heading_wait_s': 0.5,
    # Geofence: planar radius from the latched start pose.
    'max_run_distance_m': 5.0,
}

BOUNDS = {
    'max_speed_mps': (0.02, 1.50),
    'final_max_speed_mps': (0.02, 1.50),
    'min_speed_mps': (0.0, 0.15),
    'pos_gain_per_s': (0.1, 5.0),
    'accel_limit_mps2': (0.05, 3.0),
    'decel_limit_mps2': (0.05, 3.0),
    'arrive_tolerance_m': (0.005, 0.20),
    'arrive_cycles': (1, 200),
    'dwell_s': (0.0, 10.0),
    'pose_timeout_s': (0.05, 5.0),
    'waypoint_timeout_s': (0.5, 600.0),
    'control_rate_hz': (2.0, 100.0),
    'yaw_warn_rad': (0.0, 0.5),
    'yaw_abort_rad': (0.01, 1.57),
    'heading_kp': (0.1, 3.0),
    'heading_kd': (0.0, 1.0),
    'heading_deadband_rad': (0.0, 0.05),
    'heading_arrive_rad': (0.005, 0.0873),
    'heading_max_rate_rad_s': (0.01, 0.3491),
    'heading_accel_rad_s2': (0.01, 0.3491),
    'heading_timeout_s': (0.1, 0.5),
    'heading_jump_rad': (0.01, 0.1746),
    'heading_rate_guard_rad_s': (0.1, 1.0),
    'heading_rate_filter_s': (0.05, 1.0),
    'heading_settle_timeout_s': (1.0, 20.0),
    'intermediate_arrive_cycles': (1, 20),
    'intermediate_heading_rad': (0.005, 0.1746),
    'intermediate_heading_wait_s': (0.0, 2.0),
    'max_run_distance_m': (0.1, 50.0),
}

STATE_WAITING_POSE = 'WAITING_POSE'
STATE_READY = 'READY'
STATE_RUNNING = 'RUNNING'
STATE_DWELL = 'DWELL'
STATE_DONE = 'DONE'
STATE_ABORT = 'ABORT'


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def check_config(raw, base=None):
    """Merge ``raw`` over ``base`` and validate. Returns a plain dict."""
    cfg = dict(DEFAULTS if base is None else base)
    for key, value in (raw or {}).items():
        if key not in DEFAULTS and key not in ('waypoints', 'pose_source',
                                               'pose_message_type', 'cmd_vel_topic',
                                               'status_topic', 'publish_zero_on_idle'):
            raise ValueError('UNKNOWN_CONFIG_KEY:' + str(key))
        cfg[key] = value
    for key, (low, high) in BOUNDS.items():
        value = cfg[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('INVALID_CONFIG_VALUE:' + key)
        if not low <= value <= high:
            raise ValueError('CONFIG_OUT_OF_RANGE:%s=%r not in [%s,%s]'
                             % (key, value, low, high))
    if cfg['min_speed_mps'] > cfg['max_speed_mps']:
        raise ValueError('MIN_SPEED_ABOVE_MAX_SPEED')
    if cfg['min_speed_mps'] > cfg['final_max_speed_mps']:
        raise ValueError('MIN_SPEED_ABOVE_FINAL_MAX_SPEED')
    if cfg['yaw_warn_rad'] > cfg['yaw_abort_rad']:
        raise ValueError('YAW_WARN_ABOVE_YAW_ABORT')
    if type(cfg['arrive_cycles']) is not int:
        raise ValueError('ARRIVE_CYCLES_MUST_BE_INTEGER')
    if type(cfg['heading_hold_enabled']) is not bool:
        raise ValueError('HEADING_HOLD_MUST_BE_BOOL')
    if type(cfg['fast_intermediate_enabled']) is not bool:
        raise ValueError('FAST_INTERMEDIATE_MUST_BE_BOOL')
    if type(cfg['intermediate_arrive_cycles']) is not int:
        raise ValueError('INTERMEDIATE_ARRIVE_CYCLES_MUST_BE_INTEGER')
    if cfg['fast_intermediate_enabled'] and (not cfg['heading_hold_enabled'] or not
            cfg['heading_arrive_rad'] <= cfg['intermediate_heading_rad'] < cfg['yaw_abort_rad']):
        raise ValueError('INTERMEDIATE_HEADING_TOLERANCES_ORDER')
    if cfg['heading_hold_enabled'] and not cfg['heading_deadband_rad'] < cfg['heading_arrive_rad'] < cfg['yaw_abort_rad']:
        raise ValueError('HEADING_TOLERANCES_ORDER')
    return cfg


def resolve_waypoints(entries, start_xy, start_yaw):
    """Turn config entries into absolute map-frame ``(x, y)`` pairs.

    An entry is ``{'map': [x, y]}`` for an absolute map position, or
    ``{'relative': [dx, dy]}`` where dx points along the chassis nose at the
    moment the run started and dy points to its left.  Relative entries are
    frozen against the start pose, so they stay meaningful even if the pose
    drifts afterwards.
    """
    if not entries:
        raise ValueError('NO_WAYPOINTS')
    out = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError('WAYPOINT_NOT_A_MAPPING:%d' % index)
        keys = set(entry) & {'map', 'relative'}
        if len(keys) != 1:
            raise ValueError('WAYPOINT_NEEDS_EXACTLY_ONE_OF_map_relative:%d' % index)
        key = keys.pop()
        pair = entry[key]
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError('WAYPOINT_NOT_A_PAIR:%d' % index)
        a, b = pair
        if type(a) not in (int, float) or type(b) not in (int, float):
            raise ValueError('WAYPOINT_NOT_NUMERIC:%d' % index)
        if not (math.isfinite(a) and math.isfinite(b)):
            raise ValueError('WAYPOINT_NOT_FINITE:%d' % index)
        if key == 'map':
            out.append((float(a), float(b)))
        else:
            c, s = math.cos(start_yaw), math.sin(start_yaw)
            out.append((start_xy[0] + c * a - s * b,
                        start_xy[1] + s * a + c * b))
    return out


def limit_step(previous, target, max_delta):
    """Slew-limit each component of a 2-D command."""
    if previous is None:
        return target
    dx = target[0] - previous[0]
    dy = target[1] - previous[1]
    span = math.hypot(dx, dy)
    if span <= max_delta or span <= 0.0:
        return target
    scale = max_delta / span
    return (previous[0] + dx * scale, previous[1] + dy * scale)


class StraightRun(object):
    """State machine for a heading-frozen run through map waypoints."""

    def __init__(self, cfg):
        self.cfg = check_config(cfg)
        self.state = STATE_WAITING_POSE
        self.reason = ''
        self.start_xy = None
        self.start_yaw = None
        self.waypoints = None
        self.index = 0
        self.command = (0.0, 0.0)
        self.pose = None
        self.pose_wall = None
        self.yaw_drift = 0.0
        self.status_warn = ''
        self.hold_count = 0
        self.dwell_until = None
        self.waypoint_started = None
        self.commands_emitted = 0
        self.max_yaw_drift_seen = 0.0
        self.distance_travelled = 0.0
        self._last_pose_xy = None
        self._pose_seq = 0
        self._arrival_seq = -1
        self.arrival_started = None
        self.heading_deferred_count = 0
        from .heading_hold import HeadingHold
        self.heading = HeadingHold(self.cfg)

    @property
    def yaw_command(self):
        if self.cfg['heading_hold_enabled'] and self.state in (STATE_RUNNING, STATE_DWELL):
            return self.heading.command
        return 0.0

    def heading_update(self, yaw, stamp, wall):
        if (self.cfg['heading_hold_enabled'] and self.state in (STATE_RUNNING, STATE_DWELL)
                and self.heading.wall is not None and wall - self.heading.wall > self.cfg['heading_timeout_s']):
            self.heading.invalidate('HEADING_STALE')
        self.heading.push(yaw, stamp, wall)
        if self.cfg['heading_hold_enabled'] and self.heading.fault and self.state in (STATE_RUNNING, STATE_DWELL):
            self.abort(self.heading.fault)

    # -- inputs ---------------------------------------------------------
    def pose_update(self, x, y, yaw, wall):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return
        if not math.isfinite(wall):
            return
        if self.pose is not None and self.state in (STATE_RUNNING, STATE_DWELL):
            gap = wall - self.pose_wall
            jump = math.hypot(x - self.pose[0], y - self.pose[1])
            if gap < 0 or gap > self.cfg['pose_timeout_s']:
                self.abort('POSE_STALE')
            elif jump > 0.15 + 2.0 * self.cfg['max_speed_mps'] * gap:
                self.abort('POSE_JUMP')
        self._pose_seq += 1
        if self.pose is not None and wall - self.pose_wall > self.cfg['pose_timeout_s']:
            self._last_pose_xy = None  # a gap invalidates the travelled sum
        if self._last_pose_xy is not None:
            self.distance_travelled += math.hypot(x - self._last_pose_xy[0],
                                                  y - self._last_pose_xy[1])
        self._last_pose_xy = (x, y)
        self.pose = (x, y, yaw)
        self.pose_wall = wall
        if self.start_xy is None:
            self.start_xy = (x, y)
            self.start_yaw = yaw
        if self.state == STATE_WAITING_POSE:
            self.state = STATE_READY
            self.reason = 'POSE_LATCHED_WAITING_FOR_START'

    def relatch(self):
        """Freeze a new start pose and heading at the current pose.

        A second run must freeze the heading it actually starts with, not the
        heading of the previous run.  Relative waypoints resolve from here.
        """
        if self.pose is None:
            raise ValueError('RELATCH_REJECTED_NO_POSE')
        self.start_xy = (self.pose[0], self.pose[1])
        self.start_yaw = self.pose[2]
        self.distance_travelled = 0.0
        self.max_yaw_drift_seen = 0.0
        self.yaw_drift = 0.0
        self._last_pose_xy = self.start_xy
        self.status_warn = ''

    def start(self, wall=None):
        """Arm the run. Requires a latched pose and configured waypoints."""
        if self.state not in (STATE_READY, STATE_DONE, STATE_ABORT):
            raise ValueError('START_REJECTED_IN_STATE:' + self.state)
        if self.start_xy is None:
            raise ValueError('START_REJECTED_NO_POSE')
        if self.cfg['heading_hold_enabled']:
            self.heading.latch(self.pose_wall if wall is None else wall)
        # Every explicit start uses the pose at arming, including the first run.
        self.relatch()
        return self.state

    def begin(self, waypoints):
        self.waypoints = resolve_waypoints(waypoints, self.start_xy, self.start_yaw)
        self.index = 0
        self.state = STATE_RUNNING
        self.reason = 'RUNNING'
        self.hold_count = 0
        self._arrival_seq = -1
        self.arrival_started = None
        self.heading_deferred_count = 0
        self.dwell_until = None
        self.waypoint_started = None
        self.max_yaw_drift_seen = 0.0
        self.command = (0.0, 0.0)

    def _advance_waypoint(self):
        self.index += 1
        self.hold_count = 0
        self._arrival_seq = -1
        self.arrival_started = None
        self.waypoint_started = None
        self.dwell_until = None
        self.state = STATE_RUNNING if self.target() else STATE_DONE
        self.reason = ('NEXT_WAYPOINT:%d' % self.index if self.target()
                       else 'ALL_WAYPOINTS_REACHED')

    def abort(self, reason):
        # Retain the FIRST fault; cleanup STOP/shutdown must not erase evidence.
        if self.state != STATE_ABORT:
            self.reason = reason
        self.state = STATE_ABORT
        self.command = (0.0, 0.0)

    # -- control --------------------------------------------------------
    def _pose_fresh(self, wall):
        return (self.pose is not None and
                0.0 <= wall - self.pose_wall <= self.cfg['pose_timeout_s'])

    def target(self):
        if not self.waypoints or self.index >= len(self.waypoints):
            return None
        return self.waypoints[self.index]

    def active_speed_limit(self):
        cap = self.cfg['max_speed_mps']
        if self.waypoints and self.index == len(self.waypoints) - 1:
            cap = min(cap, self.cfg['final_max_speed_mps'])
        return cap

    def update(self, wall):
        """Advance one control cycle. Always returns ``(vx, vy)``."""
        cfg = self.cfg
        dt = 1.0 / cfg['control_rate_hz']
        self.commands_emitted += 1

        if self.state in (STATE_WAITING_POSE, STATE_READY):
            self.command = (0.0, 0.0)
            return self.command

        if self.state in (STATE_DONE, STATE_ABORT):
            self.command = (0.0, 0.0)
            return self.command

        if not self._pose_fresh(wall):
            # Losing the pose zeroes the command; it does not resume blind.
            self.abort('POSE_STALE')
            return self.command

        x, y, yaw = self.pose
        self.yaw_drift = wrap_angle(yaw - self.start_yaw)
        if cfg['heading_hold_enabled']:
            if not self.heading.fresh(wall):
                self.abort(self.heading.fault or 'HEADING_STALE')
                return self.command
            self.yaw_drift = self.heading.error
        self.max_yaw_drift_seen = max(self.max_yaw_drift_seen, abs(self.yaw_drift))

        if abs(self.yaw_drift) > cfg['yaw_abort_rad']:
            self.abort('YAW_DRIFT_ABORT:%.3f' % self.yaw_drift)
            return self.command
        if abs(self.yaw_drift) > cfg['yaw_warn_rad']:
            self.status_warn = 'YAW_DRIFT_WARN:%.3f' % self.yaw_drift
        else:
            self.status_warn = ''

        if math.hypot(x - self.start_xy[0], y - self.start_xy[1]) > cfg['max_run_distance_m']:
            self.abort('GEOFENCE_ABORT')
            return self.command

        if cfg['heading_hold_enabled']:
            self.heading.update(wall)
        heading_ready = not cfg['heading_hold_enabled'] or abs(self.yaw_drift) <= cfg['heading_arrive_rad']

        goal = self.target()
        if goal is None:
            self.state = STATE_DONE
            self.reason = 'ALL_WAYPOINTS_REACHED'
            self.command = (0.0, 0.0)
            return self.command

        if self.state == STATE_DWELL:
            if cfg['heading_hold_enabled'] and not heading_ready and wall > self.dwell_until + cfg['heading_settle_timeout_s']:
                self.abort('HEADING_SETTLE_TIMEOUT')
                return self.command
            if math.hypot(goal[0] - x, goal[1] - y) > cfg['arrive_tolerance_m']:
                self.state = STATE_RUNNING
                self.hold_count = 0
                self.dwell_until = None
                self.arrival_started = None
            elif wall >= self.dwell_until and heading_ready:
                self._advance_waypoint()
            self.command = (0.0, 0.0)
            return self.command

        if self.waypoint_started is None:
            self.waypoint_started = wall
        elif wall - self.waypoint_started > cfg['waypoint_timeout_s']:
            self.abort('WAYPOINT_TIMEOUT:%d' % self.index)
            return self.command

        # Twist linear velocities are body-frame commands. Goals stay frozen
        # in map; use measured yaw only to express that translation in body.
        c, s = math.cos(yaw), math.sin(yaw)
        dxw = goal[0] - x
        dyw = goal[1] - y
        ex = c * dxw + s * dyw
        ey = -s * dxw + c * dyw
        distance = math.hypot(ex, ey)

        if distance <= cfg['arrive_tolerance_m']:
            if self.arrival_started is None:
                self.arrival_started = wall
            fast_intermediate = cfg['fast_intermediate_enabled'] and self.index < len(self.waypoints) - 1
            if fast_intermediate:
                if self._arrival_seq != self._pose_seq:
                    self.hold_count += 1
                    self._arrival_seq = self._pose_seq
                self.command = (0.0, 0.0)
                small_error = abs(self.yaw_drift) <= cfg['intermediate_heading_rad']
                wait_elapsed = wall - self.arrival_started >= cfg['intermediate_heading_wait_s']
                if self.hold_count >= cfg['intermediate_arrive_cycles'] and (small_error or wait_elapsed):
                    if not small_error:
                        self.heading_deferred_count += 1
                    self._advance_waypoint()
                else:
                    self.reason = 'INTERMEDIATE_CONFIRM_AT:%d' % self.index
                return self.command
            if not heading_ready:
                self.hold_count = 0
                self.command = (0.0, 0.0)
                self.reason = 'WAITING_HEADING_AT:%d' % self.index
                if wall - self.arrival_started >= cfg['heading_settle_timeout_s']:
                    self.abort('HEADING_SETTLE_TIMEOUT')
                return self.command
            if self._arrival_seq != self._pose_seq:
                self.hold_count += 1
                self._arrival_seq = self._pose_seq
            self.command = (0.0, 0.0)
            if self.hold_count >= cfg['arrive_cycles']:
                self.state = STATE_DWELL
                self.dwell_until = wall + cfg['dwell_s']
                self.reason = 'ARRIVED:%d' % self.index
            return self.command

        self.hold_count = 0
        self.arrival_started = None

        speed = min(self.active_speed_limit(), cfg['pos_gain_per_s'] * distance)
        braking = math.sqrt(2.0 * cfg['decel_limit_mps2'] *
                            max(distance - cfg['arrive_tolerance_m'], 0.0))
        speed = min(speed, braking + 0.02)
        if distance > 0.08:
            speed = max(speed, cfg['min_speed_mps'])

        unit = (ex / distance, ey / distance)
        raw = (unit[0] * speed, unit[1] * speed)
        # Acceleration limit is shared with deceleration; a smaller decel
        # limit would make the braking curve above unreachable.
        step_cap = min(cfg['accel_limit_mps2'], cfg['decel_limit_mps2']) * dt
        self.command = limit_step(self.command, raw, step_cap)
        self.reason = 'MOVING_TO:%d' % self.index

        # Secondary guard: keep the achieved speed inside the envelope even
        # if the slew limiter was given an out-of-band previous command.
        span = math.hypot(self.command[0], self.command[1])
        cap = self.active_speed_limit()
        if span > cap:
            self.command = (self.command[0] * cap / span,
                            self.command[1] * cap / span)
        return self.command

    # -- reporting ------------------------------------------------------
    def summary(self):
        return {
            'version': VERSION,
            'state': self.state,
            'reason': self.reason,
            'warning': self.status_warn,
            'waypoint_index': self.index,
            'waypoint_count': len(self.waypoints) if self.waypoints else 0,
            'target': list(self.target()) if self.target() else None,
            'pose': ([self.pose[0], self.pose[1], self.pose[2]]
                     if self.pose else None),
            'start_xy': list(self.start_xy) if self.start_xy else None,
            'start_yaw': self.start_yaw,
            'yaw_drift': self.yaw_drift,
            'max_yaw_drift_seen': self.max_yaw_drift_seen,
            'distance_travelled_m': self.distance_travelled,
            'active_speed_limit_mps': self.active_speed_limit(),
            'final_max_speed_mps': self.cfg['final_max_speed_mps'],
            'command': {'vx': self.command[0], 'vy': self.command[1],
                        'wz': self.yaw_command},
            'angular_z_hardwired_zero': not self.cfg['heading_hold_enabled'],
            'arrival_policy': {
                'fast_intermediate_enabled': self.cfg['fast_intermediate_enabled'],
                'intermediate_heading_rad': self.cfg['intermediate_heading_rad'],
                'intermediate_heading_wait_s': self.cfg['intermediate_heading_wait_s'],
                'heading_deferred_count': self.heading_deferred_count,
                'final_heading_rad': self.cfg['heading_arrive_rad'],
                'heading_settle_timeout_s': self.cfg['heading_settle_timeout_s'],
            },
            'heading_hold': {
                'enabled': self.cfg['heading_hold_enabled'],
                'source': '/window_pass_lio/Odometry',
                'reference_rad': self.heading.reference,
                'yaw_rad': self.heading.yaw,
                'filtered_rate_rad_s': self.heading.rate,
                'map_yaw_drift_rad': wrap_angle(self.pose[2] - self.start_yaw) if self.pose else None,
                'fault': self.heading.fault,
            },
        }
