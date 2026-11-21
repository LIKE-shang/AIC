"""ROS-free safety primitives. All deadlines use monotonic wall time."""
import math


class ConfirmationGate:
    def __init__(self, on_frames=5, off_frames=3, image_timeout=0.6):
        if on_frames < 1 or off_frames < 1 or not math.isfinite(image_timeout) or image_timeout <= 0:
            raise ValueError('invalid confirmation counts / image_timeout')
        self.on_frames, self.off_frames, self.timeout = on_frames, off_frames, image_timeout
        self.hits = self.misses = 0
        self.on = False
        self.last_frame = None
        self.reason = 'startup'

    def invalidate(self, reason):
        self.hits = self.misses = 0
        self.on = False
        self.reason = reason

    def observe(self, present, now):
        # Long gaps must break consecutive-frame confirmation even if watchdog was delayed.
        if self.last_frame is not None and now-self.last_frame >= self.timeout:
            self.invalidate('image_timeout')
        self.last_frame = now
        if present:
            self.hits = min(self.hits+1, self.on_frames)
            self.misses = 0
            if self.hits >= self.on_frames:
                self.on = True
            self.reason = 'target_confirmed' if self.on else 'confirming_target'
        else:
            self.misses = min(self.misses+1, self.off_frames)
            self.hits = 0
            if self.misses >= self.off_frames:
                self.on = False
            self.reason = 'loss_grace' if self.on else 'target_absent'
        return self.on

    def watchdog(self, now):
        if self.last_frame is None or now-self.last_frame >= self.timeout:
            self.invalidate('image_timeout')
            return True
        return False


class FrameFreshness:
    def __init__(self, max_age, future_tolerance=0.2, require_stamp=True):
        if not math.isfinite(max_age) or max_age <= 0 or not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('invalid capture freshness thresholds')
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.require_stamp = require_stamp
        self.previous_stamp = None

    def check(self, stamp, ros_now):
        if not math.isfinite(stamp) or not math.isfinite(ros_now):
            raise ValueError('nonfinite camera/ROS time')
        if self.require_stamp and (stamp <= 0 or ros_now <= 0):
            raise ValueError('missing/zero capture stamp or ROS clock')
        if stamp > 0:
            age = ros_now-stamp
            if age > self.max_age or age < -self.future_tolerance:
                raise ValueError('stale/future capture stamp; age=%.3fs' % age)
            previous = self.previous_stamp
            # High-water mark: camera/clock reset requires an explicit detector restart.
            if previous is not None and stamp <= previous:
                raise ValueError('capture stamp did not advance')
            self.previous_stamp = stamp


def safe_test_topic(topic):
    """Fail closed against accidental remapping to any mission/control topic."""
    if not isinstance(topic, str) or not (topic.startswith('/window_test/') or topic.startswith('/circle_laser_test/')):
        raise ValueError('test output must stay inside /window_test/ or /circle_laser_test/')
    return topic

class ContinuousHitGate:
    """One-shot, immediate-OFF gate. ACK and visible evidence are NOT equivalent.
    Counts both monotonic and capture time; neither a frozen ROS clock nor fast
    replay can manufacture a continuous hold. Any ON interruption latches OFF
    until this independent detector is restarted after physical inspection.
    """
    def __init__(self, on_frames=5, image_timeout=0.35, hold_seconds=3.0,
                 feedback_mode='prediction_only', acquire_timeout=0.30, max_on_seconds=5.0):
        if type(on_frames) is not int or on_frames < 1:
            raise ValueError('on_frames must be a positive integer')
        for value in (image_timeout, hold_seconds, acquire_timeout, max_on_seconds):
            if not math.isfinite(value) or value <= 0: raise ValueError('invalid timing parameter')
        if hold_seconds < 3 or max_on_seconds < hold_seconds or max_on_seconds > 5:
            raise ValueError('hold must be >=3 seconds, independent exposure limit <=5 seconds')
        if image_timeout > .7 or acquire_timeout > .5:
            raise ValueError('image/acquisition timeout too large for handheld test')
        if feedback_mode not in ('camera', 'prediction_only'): raise ValueError('bad feedback mode')
        self.on_frames, self.off_frames, self.timeout = on_frames, 1, image_timeout
        self.hold_seconds, self.acquire_timeout, self.max_on_seconds = hold_seconds, acquire_timeout, max_on_seconds
        self.feedback_mode = feedback_mode
        self.hits = self.misses = 0
        self.on = self.latched = False
        self.last_frame = self.last_stamp = None
        self.exposure_start = self.hold_start = self.hold_stamp = self.ack_start = None
        self.elapsed = 0.0
        self.result = 'none'
        self.completed_seconds = 0.0
        self.completed_capture_interval = None
        self.reason = 'startup'

    def invalidate(self, reason, latch=False):
        self.latched = self.latched or self.on or latch
        self.on = False
        self.hits = 0; self.misses = 1
        self.exposure_start = self.hold_start = self.hold_stamp = self.ack_start = None
        self.elapsed = 0.0
        self.reason = reason

    def observe(self, eligible, now, stamp, reason='not_eligible', acknowledged=False, observed_hit=False):
        if not math.isfinite(now) or not math.isfinite(stamp) or stamp <= 0:
            self.invalidate('invalid_time'); return False
        if self.last_frame is not None and (now < self.last_frame or now-self.last_frame >= self.timeout or
                                           stamp <= self.last_stamp or stamp-self.last_stamp >= self.timeout):
            self.invalidate('capture_or_processing_gap')
            self.last_frame, self.last_stamp = now, stamp
            return False
        self.last_frame, self.last_stamp = now, stamp
        if self.latched: return False
        if not eligible:
            self.invalidate(reason); return False
        self.misses = 0
        self.hits = min(self.on_frames, self.hits+1)
        if self.hits < self.on_frames:
            self.reason = 'confirming_target_and_aim'; return False
        if not self.on:
            self.on = True; self.exposure_start = now
        if now-self.exposure_start >= self.max_on_seconds:
            self.invalidate('maximum_exposure', latch=True); return False
        if not acknowledged:
            if self.ack_start is not None: self.invalidate('controller_ack_lost')
            else: self.reason = 'waiting_on_ack_no_hit_credit'
            return self.on
        if self.ack_start is None: self.ack_start = now
        if self.feedback_mode == 'camera' and not observed_hit:
            if self.hold_start is not None or now-self.ack_start >= self.acquire_timeout:
                self.invalidate('actual_spot_missing_outside_or_uncertain')
            else: self.reason = 'acquiring_spot_no_hit_credit'
            return self.on
        if self.hold_start is None:
            self.hold_start, self.hold_stamp = now, stamp
        self.elapsed = max(0., min(now-self.hold_start, stamp-self.hold_stamp))
        self.reason = 'camera_observed_hold' if self.feedback_mode == 'camera' else 'predicted_hold_NOT_actual_hit'
        if self.elapsed >= self.hold_seconds:
            self.completed_seconds = self.elapsed
            self.completed_capture_interval = [self.hold_stamp, stamp]
            self.result = 'camera_observed_3s' if self.feedback_mode == 'camera' else 'predicted_only_3s'
            self.invalidate('completed_'+self.result, latch=True)
            return False
        return True

    def watchdog(self, now):
        if self.on and self.exposure_start is not None and now-self.exposure_start >= self.max_on_seconds:
            self.invalidate('maximum_exposure'); return True
        if self.last_frame is None or now-self.last_frame >= self.timeout:
            self.invalidate('image_timeout'); return True
        return False


class ContentFreshness:
    """Conservative repeated-payload guard for drivers that stamp at publication.
    Exact byte duplicates only; cannot prove exposure time or detect all replays.
    Real camera noise normally changes payload even for a stationary target.
    """
    def __init__(self, timeout=.25):
        if not math.isfinite(timeout) or not 0 < timeout <= .7:
            raise ValueError('invalid frozen-content timeout')
        self.timeout=timeout; self.digest=None; self.changed_at=None

    def check(self, payload, now):
        import hashlib
        digest=hashlib.sha256(payload).digest()
        if not math.isfinite(now): raise ValueError('invalid wall time')
        if self.digest != digest:
            self.digest=digest; self.changed_at=now
        elif now-self.changed_at >= self.timeout:
            raise ValueError('frozen identical image content despite advancing headers')
