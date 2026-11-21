"""Finite exposure servo: short ramps separated by measured tracking pauses.

No timers, IO or mission transitions. The owner retains the fixed ON deadline.
"""
import math
from collections import deque


class ObservedServoPacer:
    pulse_path_m = .01
    settle_s = .15
    capture_age_s = .15
    tracking_tolerance_m = .02
    position_span_m = .02
    error_span_px = 4.
    trend_residual_px = 2.

    def __init__(self):
        self.samples = deque(maxlen=16)
        self.moving = False
        self.pending = False
        self.path = 0.
        self.start_actual = None
        self.command_delta = [0., 0.]
        self.reason = 'await_stable_observed_spot'
        self.pulses = 0
        self.filtered_error_px = None
        self.diagnostics = {}
        self.reduce_lead_only = False
        self.actual_at_proposal = None

    def hold(self, reason):
        self.moving = False
        self.samples.clear()
        self.reason = reason
        self.filtered_error_px = None

    def status(self):
        return dict(state='ramping' if self.moving else 'waiting', reason=self.reason,
                    pulses=self.pulses, pulse_path_m=self.path,
                    pulse_limit_m=self.pulse_path_m, maximum_capture_age_s=self.capture_age_s,
                    settle_s=self.settle_s, tracking_tolerance_m=self.tracking_tolerance_m,
                    filtered_error_px=self.filtered_error_px, reduce_lead_only=self.reduce_lead_only,
                    diagnostics=dict(self.diagnostics))

    def error_trend(self):
        """Fit only through the newest capture; never extrapolate future motion."""
        times = [s[0]-self.samples[-1][0] for s in self.samples]
        mean_time = sum(times)/len(times)
        variance = sum((t-mean_time)**2 for t in times)
        if variance <= 1e-12:
            return list(self.samples[-1][2]), 0.
        slopes, intercepts = [], []
        for axis in range(2):
            mean = sum(s[2][axis] for s in self.samples)/len(times)
            slope = sum((t-mean_time)*(s[2][axis]-mean)
                        for t,s in zip(times,self.samples))/variance
            slopes.append(slope)
            intercepts.append(mean-slope*mean_time)
        residual = max(math.hypot(*(s[2][k]-(intercepts[k]+slopes[k]*t)
                                    for k in range(2)))
                       for t,s in zip(times,self.samples))
        return intercepts, residual

    def allowed_dt(self, mission, detection, dt, speed_limit):
        age = detection.get('servo_capture_age_s')
        if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= self.capture_age_s:
            self.hold('await_fresh_spot_for_correction')
            return None
        stamp = detection['capture_stamp']
        actual = tuple(mission.core.pose.p[:2])
        error = tuple(detection['servo_error_px'])
        if self.samples and stamp <= self.samples[-1][0]:
            self.reason = 'await_new_spot_for_correction'
            return None
        self.samples.append((stamp, actual, error))
        self.actual_at_proposal = actual
        while len(self.samples) > 1 and self.samples[1][0] <= stamp-self.settle_s:
            self.samples.popleft()
        duration = stamp-self.samples[0][0]
        self.filtered_error_px = None
        if duration >= self.settle_s-1e-6:
            filtered, residual = self.error_trend()
            span = max(math.dist(a[1], b[1]) for a in self.samples for b in self.samples)
            image_span = max(math.dist(a[2], b[2]) for a in self.samples for b in self.samples)
            self.diagnostics = dict(position_span_m=span, error_span_px=image_span,
                                    trend_residual_px=residual, observation_span_s=duration,
                                    horizontal_lead_m=math.dist(mission.position[:2], actual))
            if image_span <= self.error_span_px or residual <= self.trend_residual_px:
                self.filtered_error_px = filtered
            else:
                self.hold('await_coherent_spot_motion')
                return None
        if not self.moving:
            if duration < self.settle_s-1e-6:
                self.reason = 'await_stable_observed_spot'
                return None
            lead = math.dist(mission.position[:2], actual)
            if span > self.position_span_m:
                self.reason = 'await_slow_position_motion'
                return None
            # The final proposed-point guard admits bounded steps that reduce
            # an existing lead excess. Do not veto those before it runs.
            # Once we have issued motion, keep the original response gate.
            if self.pending:
                length = math.hypot(*self.command_delta)
                progress = (sum((actual[k]-self.start_actual[k])*self.command_delta[k]
                                for k in range(2))/length) if length > 1e-9 else 0.
                if progress < min(.003, .3*length):
                    self.reason = 'await_motion_response_before_next_pulse'
                    return None
            # An outstanding reference farther than 2 cm can only be brought
            # closer, even after the previous pulse has a measured response.
            self.reduce_lead_only = self.pulses > 0 and lead > self.tracking_tolerance_m
            self.moving = True
            self.pending = False
            self.path = 0.
            self.command_delta = [0., 0.]
            self.start_actual = actual
        self.reason = 'observed_spot_short_ramp'
        remaining = max(0., self.pulse_path_m-self.path)
        return min(max(0., dt), .05, remaining/speed_limit)

    def applied(self, before, after):
        delta = [after[k]-before[k] for k in range(2)]
        if self.path <= 1e-12:
            self.pulses += 1
            # Rejected proposals must not count earlier drift as a response
            # to a command that had not yet been issued.
            self.start_actual = self.actual_at_proposal
        self.path += math.hypot(*delta)
        self.command_delta = [self.command_delta[k]+delta[k] for k in range(2)]
        self.pending = True
        if self.path >= self.pulse_path_m-1e-9:
            self.hold('await_tracking_after_short_ramp')
