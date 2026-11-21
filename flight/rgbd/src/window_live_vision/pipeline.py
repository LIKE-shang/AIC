"""Wall-clock pipeline evidence, separate from target confirmation history.

All PipelineMonitor methods are called under the LiveNode lock. Profiling is
informational only: it cannot change any freshness or flight permission gate.
"""
from collections import deque, Counter
from contextlib import contextmanager
import math
import time
import statistics
import uuid


def finite_or_none(value):
    return float(value) if value is not None and math.isfinite(value) else None


class FrameProfile:
    def __init__(self, stamp, received, started):
        self.stamp = stamp
        self.received = received
        self.started_wall = started
        self.started_perf = time.perf_counter()
        self.current_stage = 'input_validation'
        self.stage_ms = {}

    @contextmanager
    def stage(self, name):
        previous = self.current_stage
        self.current_stage = name
        start = time.perf_counter()
        try:
            yield
        finally:
            self.stage_ms[name] = self.stage_ms.get(name, 0.) + (time.perf_counter()-start)*1000.
            self.current_stage = previous

    def elapsed_ms(self):
        return (time.perf_counter()-self.started_perf)*1000.


class PipelineMonitor:
    def __init__(self):
        self.instance_id = uuid.uuid4().hex
        self.receipts = {'rgb': deque(maxlen=180), 'depth': deque(maxlen=180)}
        self.counts = {'rgb': 0, 'depth': 0}
        self.completions = deque(maxlen=60)
        self.process_sequence = 0
        self.active = None
        self.last_frame = None

    def receive(self, name, wall, stamp):
        self.receipts[name].append((wall, finite_or_none(stamp)))
        self.counts[name] += 1

    def begin(self, profile):
        self.active = profile

    def finish(self, status, profile, wall, now_ros):
        self.process_sequence += 1
        self.completions.append((wall, profile.stamp))
        fields = ('reason', 'ready', 'confirmed_frames', 'tracking_reset_reason',
                  'tracking_source_gap_s', 'tracking_receipt_gap_s', 'tracking_iou',
                  'target_count', 'depth_reason', 'depth_stamp_delta_s', 'metric_reason',
                  'metric_geometry_ready', 'metric_geometry_reason', 'traversal_reason')
        row = {key: status.get(key) for key in fields}
        row.update(process_sequence=self.process_sequence, source_stamp=finite_or_none(profile.stamp),
                   source_received_wall=finite_or_none(profile.received), completed_wall=finite_or_none(wall),
                   queue_wait_ms=finite_or_none((profile.started_wall-profile.received)*1000.),
                   processing_ms=profile.elapsed_ms(), processing_ms_scope='through_result_decision',
                   cycle_ms=None, stage_ms=dict(profile.stage_ms),
                   result_age_ros_s=finite_or_none(now_ros-profile.stamp),
                   result_age_wall_s=finite_or_none(wall-profile.received))
        self.last_frame = row
        self.active = None
        return row

    def complete_cycle(self, profile):
        # Called after image annotation/publication; one worker processes frames.
        if self.last_frame and self.last_frame['source_stamp'] == finite_or_none(profile.stamp):
            self.last_frame['cycle_ms'] = profile.elapsed_ms()
            self.last_frame['stage_ms'] = dict(profile.stage_ms)

    @staticmethod
    def rate(rows, now):
        recent = [wall for wall, _ in rows if 0 <= now-wall <= 2.]
        if len(recent) < 2 or recent[-1] <= recent[0]:
            return None
        return (len(recent)-1)/(recent[-1]-recent[0])

    def snapshot(self, now, now_ros, max_age_s):
        streams = {}
        for name, rows in self.receipts.items():
            last = rows[-1] if rows else (None, None)
            age = finite_or_none(now-last[0]) if last[0] is not None else None
            streams[name] = dict(received_count=self.counts[name],
                callback_rate_hz=self.rate(rows, now), receipt_age_s=age,
                receipt_fresh=age is not None and 0 <= age <= max_age_s,
                source_stamp=last[1],
                source_age_ros_s=finite_or_none(now_ros-last[1]) if last[1] is not None else None)
        active = self.active
        frame = self.last_frame
        wall_age = finite_or_none(now-frame['source_received_wall']) if frame and frame['source_received_wall'] is not None else None
        ros_age = finite_or_none(now_ros-frame['source_stamp']) if frame and frame['source_stamp'] is not None else None
        return dict(instance_id=self.instance_id, streams=streams, process_sequence=self.process_sequence,
                    processed_rate_hz=self.rate(self.completions, now),
                    processing_active=active is not None,
                    processing_stage=active.current_stage if active else None,
                    in_flight_source_age_wall_s=finite_or_none(now-active.received) if active else None,
                    result_age_wall_s=wall_age, result_age_ros_s=ros_age,
                    result_fresh=wall_age is not None and ros_age is not None and
                        0 <= wall_age <= max_age_s and -.02 <= ros_age <= max_age_s,
                    callback_rates_are_not_exposure_rates=True)


def sample_stats(values):
    ordered = sorted(float(value) for value in values
                     if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value))
    if not ordered:
        return dict(samples=0, p50=None, p95=None, maximum=None)
    return dict(samples=len(ordered), p50=statistics.median(ordered),
                p95=ordered[max(0, math.ceil(.95*len(ordered))-1)], maximum=ordered[-1])


def summarize_frames(frames):
    """Summarize unique observed frame results, never heartbeat duplicates."""
    stages = sorted({name for frame in frames for name in frame.get('stage_ms', {})})
    return dict(observed_unique_frames=len(frames),
        frame_reasons=dict(Counter(frame.get('reason', 'missing') for frame in frames)),
        tracking_resets=dict(Counter(frame['tracking_reset_reason'] for frame in frames if frame.get('tracking_reset_reason'))),
        confirmed_frames_max=max((frame.get('confirmed_frames') or 0 for frame in frames), default=0),
        processing_ms=sample_stats(frame.get('processing_ms') for frame in frames),
        cycle_ms=sample_stats(frame.get('cycle_ms') for frame in frames),
        queue_wait_ms=sample_stats(frame.get('queue_wait_ms') for frame in frames),
        tracking_source_gap_s=sample_stats(frame.get('tracking_source_gap_s') for frame in frames),
        tracking_receipt_gap_s=sample_stats(frame.get('tracking_receipt_gap_s') for frame in frames),
        stage_ms={name: sample_stats(frame.get('stage_ms', {}).get(name) for frame in frames) for name in stages},
        first_observed_result_may_precede_snapshot=True)
