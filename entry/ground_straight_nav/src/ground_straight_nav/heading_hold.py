"""Relative continuous-LIO heading hold, independent of map/ICP yaw corrections."""
import math

def wrap(value):
    return (value + math.pi) % (2 * math.pi) - math.pi

class HeadingHold:
    def __init__(self, cfg):
        self.cfg = cfg
        self.yaw = self.stamp = self.wall = self.reference = None
        self.rate = self.command = 0.0
        self.fault = None
        self.tick_wall = None

    def invalidate(self, reason):
        self.fault = reason
        self.command = 0.0

    def push(self, yaw, stamp, wall):
        if not all(math.isfinite(v) for v in (yaw, stamp, wall)):
            self.invalidate('HEADING_INVALID')
            return
        if self.stamp is not None:
            dt = stamp - self.stamp
            if dt == 0:
                return  # repeated samples cannot refresh the heading watchdog
            if dt < 0 or wall < self.wall:
                self.invalidate('HEADING_TIME_RESET')
                return
            delta = wrap(yaw - self.yaw)
            if abs(delta) > self.cfg['heading_jump_rad'] + self.cfg['heading_rate_guard_rad_s'] * dt:
                self.invalidate('HEADING_DISCONTINUITY')
                return
            alpha = dt / (self.cfg['heading_rate_filter_s'] + dt)
            self.rate += alpha * (delta / dt - self.rate)
        self.yaw, self.stamp, self.wall = yaw, stamp, wall

    def fresh(self, wall):
        return (self.fault is None and self.wall is not None and
                0 <= wall - self.wall <= self.cfg['heading_timeout_s'])

    def latch(self, wall):
        if not self.fresh(wall):
            raise ValueError(self.fault or 'START_REJECTED_STALE_HEADING')
        self.reference = self.yaw
        self.command = 0.0
        self.tick_wall = None

    @property
    def error(self):
        return wrap(self.yaw - self.reference) if self.reference is not None else 0.0

    def update(self, wall):
        error = self.error
        deadband = self.cfg['heading_deadband_rad']
        effective = math.copysign(max(0.0, abs(error) - deadband), error)
        target = (-self.cfg['heading_kp'] * effective - self.cfg['heading_kd'] * self.rate
                  if effective else 0.0)
        cap = self.cfg['heading_max_rate_rad_s']
        target = max(-cap, min(cap, target))
        period = 1.0 / self.cfg['control_rate_hz']
        dt = period if self.tick_wall is None else max(0.0, min(period, wall - self.tick_wall))
        step = self.cfg['heading_accel_rad_s2'] * dt
        self.command += max(-step, min(step, target - self.command))
        self.tick_wall = wall
        return self.command
