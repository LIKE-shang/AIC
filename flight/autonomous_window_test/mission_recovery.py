"""Finite recovery of an unfinished competition stage; no authority or IO."""
import copy
from window_pass.geometry import sub, norm, dot
from attitude_envelope import enclosed, support, spatial_budget
from pass_terminal import MotionWindow


def enabled(m):
    return bool(getattr(m, 'return_journey', None) and m.session_window and
                m.session_policy.get('field_bounded_alignment'))


class Progress:
    def __init__(self):
        self.best = None
        self.since = None

    def stalled(self, error, wall, timeout=6., improvement=.01):
        if self.best is None or error < self.best - improvement:
            self.best, self.since = error, wall
        return wall - self.since >= timeout

    def reset(self):
        self.best = self.since = None


class Recovery:
    def __init__(self):
        self.counts = {}
        self.events = []
        self.active = None
        self.expected_geometry = None
        self.motion = MotionWindow()

    def take(self, m, i, kind, reason, maximum=2):
        if not enabled(m):
            return False
        if m.phase in ('FAULT', 'REFUSED', 'TAKEOVER', 'PILOT_CONTROL', 'DONE'):
            return False
        if not m.core.armed or not m.core.airborne or i.emergency:
            return False
        if i.wall - m.began >= m.o['max_mission_time_s']:
            return False
        target = getattr(m.full_chain,'index','-') if m.phase in ('HOVER','POSTPASS') else '-'
        key = '%s:%s:%s:%s' % (m.return_journey.leg, m.phase, target, kind)
        count = self.counts.get(key, 0)
        if count >= maximum:
            return False
        self.counts[key] = count + 1
        self.events.append(dict(key=key, attempt=count + 1, reason=reason, wall=i.wall))
        self.events = self.events[-40:]
        return True

    def status(self):
        return dict(active=copy.deepcopy(self.active), attempts=dict(self.counts),
                    events=list(self.events), expected_geometry=copy.deepcopy(self.expected_geometry))

    def _control(self, m):
        c = m.core.c
        if m.return_journey.leg == 2:
            c = m.session_window.scoped_control(c)
        return c

    def _outside_opening(self, m, p):
        s = m.session_window.snapshot
        if not s:
            return True
        center = s['geometry']['center_enu_m']
        normal = s['geometry']['normal_enu']
        # Either fitted normal sign is legal; recovery stays on its current side.
        distance = abs(dot(sub(p, center), normal))
        return distance > support(self._control(m), m.heading, normal) + .01

    def holding_allowed(self, m, p):
        if not enabled(m):
            return False
        c = dict(self._control(m))
        c['_session_pass_tilt_limit_rad'] = c['tilt_limit_rad']
        if not (m.corridor(p) and enclosed(c, p, m.heading) and
                m.full_height_bounds(p)[0] >= m.o['minimum_bottom_height_m'] and
                self._outside_opening(m, p)):
            return False
        j = m.return_journey
        if j.active:
            plane, normal = j.first_plan['geometry_center'], j.first_plan['window_normal_local_m']
            if dot(sub(p, plane), normal) < support(c, m.heading, tuple(-x for x in normal)) + .09:
                return False
        return True

    def begin_prepass(self, m, i, reason):
        if not enabled(m) or m.phase not in ('SEARCH', 'ADJUST') or self.active:
            return False
        actual = tuple(m.core.pose.p)
        if not self.holding_allowed(m, actual):
            return False
        c = self._control(m)
        reserve = spatial_budget(c) + c.get('_session_field_fence_margin_m', c['clearance_margin_m'])
        target = list(actual)
        # Move only as far as necessary to recover the original PASS envelope.
        for k in range(3):
            axis = tuple(float(k == n) for n in range(3))
            slack = c.get('_session_horizontal_cone_slack_m', 0.) if k < 2 else 0.
            lo = c['geofence_min_m'][k] + support(c, m.heading, tuple(-v for v in axis)) + reserve - slack + .005
            hi = c['geofence_max_m'][k] - support(c, m.heading, axis) - reserve + slack - .005
            if lo > hi:
                return False
            target[k] = max(lo, min(hi, target[k]))
        target = tuple(target)
        if norm(sub(target, actual)) > .35:
            return False
        # Check the entire short recovery segment under the hold attitude bound.
        for n in range(9):
            p = tuple(a + (b - a) * n / 8. for a, b in zip(actual, target))
            if not self.holding_allowed(m, p):
                return False
        if not self.take(m, i, 'prepass', reason):
            return False
        snapshot = m.session_window.snapshot
        if snapshot:
            self.expected_geometry = copy.deepcopy(snapshot['geometry'])
        self.active = dict(kind='prepass', reason=reason, target=target, started=i.wall)
        self.motion = MotionWindow()
        m.position = actual
        m.pass_velocity = m.station_velocity = None
        m.reason = 'recovering_prepass:' + reason
        return True

    def tick(self, m, i, dt):
        a = self.active
        if not a:
            return None
        if not self.holding_allowed(m, m.core.pose.p) or not self.holding_allowed(m, m.position):
            self.active = None
            return m.fail('recovery_actual_or_reference_outside_hold_region', i)
        if i.wall - a['started'] > 12.:
            self.active = None
            return m.handover('prepass_recovery_no_progress', i)
        self.motion.update(m.core.pose.stamp, m.core.pose.p)
        delta = sub(a['target'], m.position)
        length = norm(delta)
        step = min(length, .04 * dt)
        candidate = tuple(p + d * step / length for p, d in zip(m.position, delta)) if length > 1e-9 else a['target']
        if norm(sub(candidate, m.core.pose.p)) <= .08 and self.holding_allowed(m, candidate):
            m.position = candidate
        if (norm(sub(m.position, a['target'])) <= 1e-6 and
                norm(sub(m.core.pose.p, a['target'])) <= .04 and self.motion.stable(.12, .08)):
            from session_window import Acquisition
            old = m.session_window
            m.session_window = Acquisition(copy.deepcopy(old.identity), old.fast_confirmation,
                old.tracking_reserve_m, old.clearance_relaxation_m,
                old.maximize_minimum_clearance, old.horizontal_cone_slack_m)
            m.adjust_plan = None
            m.adjust_completed = False
            m.position = tuple(m.core.pose.p)
            m.search_started_ros = i.ros
            m.core.metric_input = None
            m.core.metric_received = -1e9
            self.active = None
            m.transition('SEARCH', 'recovered_hold_reacquire_same_opening', i.wall)
        return m.output()

    def check_geometry(self, metric):
        if self.expected_geometry is None:
            return
        old = self.expected_geometry
        if (metric.get('pose_epoch'), metric.get('calibration_epoch')) != (old['pose_epoch'], old['calibration_epoch']):
            raise ValueError('recovery_geometry_epoch_changed')
        center = metric.get('center_enu_m')
        if not isinstance(center, (list, tuple)) or len(center) != 3 or norm(sub(center, old['center_enu_m'])) > .20:
            raise ValueError('recovery_requires_same_opening')
