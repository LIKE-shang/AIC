"""Tracking and image guidance. Invalid input always cancels the last proposal."""
import math
import numpy as np
from .detector import polygon_iou


class Tracker:
    def __init__(self, confirm_frames=3, confirm_s=.18, max_gap_s=.40, max_age_s=.50):
        self.confirm_frames = confirm_frames
        self.confirm_s = confirm_s
        self.max_gap_s = max_gap_s
        self.max_age_s = max_age_s
        self.last_stamp = None
        self.reset('first_target')

    def reset(self, reason='track_reset'):
        self.last = None
        self.last_wall = None
        self.first_stamp = None
        self.count = 0
        self.reset_reason = reason

    def cancel(self, reason):
        self.reset(reason)
        return dict(ready=False, reason=reason, visual_instruction=None, confirmed_frames=0,
                    tracking_reset_reason=reason, tracking_source_gap_s=None,
                    tracking_receipt_gap_s=None, tracking_iou=None)

    def update(self, candidates, stamp, now_ros, now_wall, shape, principal_point=None):
        if not all(math.isfinite(v) for v in (stamp, now_ros, now_wall)):
            return self.cancel('nonfinite_clock')
        if stamp <= 0 or not -.02 <= now_ros-stamp <= self.max_age_s:
            return self.cancel('stale_or_future_image')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # A ROS clock restart needs a fresh track, never a reused target.
            self.last_stamp = stamp
            return self.cancel('duplicate_reordered_or_clock_reset')
        previous_stamp = self.last_stamp
        self.last_stamp = stamp
        if len(candidates) != 1:
            return self.cancel('multiple_candidates' if candidates else 'no_complete_frame')
        current = candidates[0]
        source_gap = stamp-previous_stamp if previous_stamp is not None else None
        receipt_gap = now_wall-self.last_wall if self.last_wall is not None else None
        overlap = polygon_iou(current.corners, self.last.corners) if self.last is not None else None
        reset_reason = ''
        if self.last is None:
            reset_reason = self.reset_reason
        elif receipt_gap is None or not 0 <= receipt_gap <= self.max_gap_s:
            reset_reason = 'image_receipt_gap_exceeded'
        elif source_gap is None or not 0 < source_gap <= self.max_gap_s:
            reset_reason = 'image_stamp_gap_exceeded'
        elif overlap is None or overlap < .50:
            reset_reason = 'target_overlap_below_threshold'
        if reset_reason:
            self.reset(reset_reason)
            self.first_stamp = stamp
        self.count += 1
        self.last, self.last_wall = current, now_wall
        ready = self.count >= self.confirm_frames and stamp-self.first_stamp >= self.confirm_s
        data = dict(ready=ready, reason='live_visual_tracking' if ready else 'confirming_current_target',
                    confirmed_frames=self.count, visual_instruction=None,
                    tracking_reset_reason=reset_reason, tracking_source_gap_s=source_gap,
                    tracking_receipt_gap_s=receipt_gap, tracking_iou=overlap)
        if ready:
            h, w = shape[:2]
            ref = np.array(principal_point if principal_point is not None else [(w-1)/2., (h-1)/2.])
            error = current.center-ref
            q = current.corners
            width = (np.linalg.norm(q[1]-q[0])+np.linalg.norm(q[2]-q[3]))/2.
            height = (np.linalg.norm(q[3]-q[0])+np.linalg.norm(q[2]-q[1]))/2.
            # Deadband scales with the visible opening; it is not a metric
            # clearance allowance and not a commanded vehicle displacement.
            normalized = error/np.array([width, height])
            direction = 'IMAGE_RIGHT' if normalized[0] > .035 else ('IMAGE_LEFT' if normalized[0] < -.035 else 'IMAGE_CENTERED')
            data['visual_instruction'] = dict(direction=direction, error_px=error.tolist(),
                error_fraction_of_opening=normalized.tolist(), reference_pixel=ref.tolist(),
                reference='camera_principal_point' if principal_point is not None else 'image_center_uncalibrated',
                vertical_error_informational_only=True, altitude_change_requested=False,
                yaw_change_requested=False, forward_change_requested=False,
                metric_delta_m=None, output_permitted=False)
        return data

    def watchdog(self, now_wall):
        if self.last_wall is None or not 0 <= now_wall-self.last_wall <= self.max_age_s:
            return self.cancel('image_stream_stale')
        return None
