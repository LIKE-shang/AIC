"""Bounded visual evidence for aiming. No ROS, motion or hardware IO."""
import math


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def vector(value):
    return (isinstance(value, list) and len(value) == 2 and
            all(type(v) in (int, float) and math.isfinite(v) for v in value))


class AimEvidence:
    def __init__(self):
        self.valid_frames = 0
        self.invalid_frames = 0
        self.last_invalid = None
        self.last_stamp = None
        self.last_center = None
        self.target_run = 0
        self.clear_run = 0
        self.baseline_clear = False
        self.spot_run = 0
        self.last_spot_error = None
        self.candidate_frames = 0
        self.qualified_frames = 0
        self.error_sum = [0., 0.]
        self.error_square_sum = [0., 0.]
        self.last_reason = 'await_visual_evidence'

    def missing(self, reason):
        self.invalid_frames += 1
        self.last_invalid = reason
        self.target_run = self.clear_run = self.spot_run = 0
        # Suspend correction immediately. A short missed ring frame does not
        # erase the measured OFF baseline: observe() still rejects a resumed
        # target after a time gap or position jump, then requires fresh runs.
        self.last_spot_error = None
        self.last_reason = 'visual_unavailable_hold_position'

    def observe(self, d, phase, on_ack_ros, timeout):
        """Called only after TargetHold.vision accepts a NEW capture stamp."""
        stamp = d['capture_stamp']
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False, False
        center = [p-e for p, e in zip(d['reference_point_px'], d['reference_error_px'])]
        gap = self.last_stamp is not None and stamp-self.last_stamp > timeout
        jump = self.last_center is not None and distance(center, self.last_center) > max(8., min(d['inner_semiaxes_px'])*.35)
        if gap or jump:
            self.target_run = self.clear_run = self.spot_run = 0
            self.baseline_clear = False
            self.last_spot_error = None
        self.last_stamp, self.last_center = stamp, center
        self.valid_frames += 1
        self.target_run += 1
        target_ready = self.target_run >= 3
        if phase == 'CENTER':
            self.clear_run = self.clear_run+1 if d.get('observed_candidates') == [] else 0
            self.baseline_clear = target_ready and self.clear_run >= 3
            self.last_reason = 'target_track_ready' if target_ready else 'confirming_target_track'
            return target_ready, False
        if phase != 'EXPOSE':
            return target_ready, False
        error = d.get('observed_error_px')
        spot = d.get('observed_spot_px')
        candidates = d.get('observed_candidates')
        radius = d.get('observed_normalized_radius')
        coherent = (d.get('observed_unique') is True and isinstance(candidates, list) and len(candidates) == 1 and
                    isinstance(candidates[0], dict) and vector(candidates[0].get('centre_px')) and
                    vector(error) and vector(spot) and distance(spot, candidates[0]['centre_px']) < 1e-6 and
                    distance(error, [spot[k]-center[k] for k in range(2)]) < 1e-6 and
                    type(radius) in (int, float) and math.isfinite(radius) and radius >= 0 and
                    on_ack_ros is not None and stamp > on_ack_ros)
        if coherent:
            self.candidate_frames += 1
        consistent = coherent and (self.last_spot_error is None or
                                    distance(error, self.last_spot_error) <= max(4., min(d['inner_semiaxes_px'])*.30))
        self.spot_run = self.spot_run+1 if consistent else (1 if coherent else 0)
        self.last_spot_error = list(error) if coherent else None
        qualified = target_ready and self.baseline_clear and self.spot_run >= 3
        self.last_reason = ('observed_spot_track_ready' if qualified else
                            'no_clear_off_baseline' if not self.baseline_clear else
                            'confirming_observed_spot' if coherent else 'spot_missing_or_ambiguous_hold_position')
        if qualified:
            self.qualified_frames += 1
            for k in range(2):
                self.error_sum[k] += error[k]
                self.error_square_sum[k] += error[k]**2
        return target_ready, qualified

    def status(self):
        n = self.qualified_frames
        mean = [v/n for v in self.error_sum] if n else None
        std = [math.sqrt(max(0., self.error_square_sum[k]/n-mean[k]**2)) for k in range(2)] if n else None
        return dict(valid_target_frames=self.valid_frames, invalid_observations=self.invalid_frames,
                    last_invalid_reason=self.last_invalid, target_consecutive_frames=self.target_run,
                    clear_off_baseline=self.baseline_clear, spot_candidate_frames=self.candidate_frames,
                    qualified_spot_frames=n, observed_error_mean_px=mean, observed_error_std_px=std,
                    last_reason=self.last_reason, physical_hit_verified=False)
