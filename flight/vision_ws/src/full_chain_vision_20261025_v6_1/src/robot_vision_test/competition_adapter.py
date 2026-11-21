"""Pure helpers for the optional competition window adapter.

The legacy mission consumes ``geometry_msgs/PointStamped`` where ``point.x``
and ``point.y`` are image pixels and ``header.frame_id`` is a class string.
This module deliberately contains no ROS imports so its validation can be
regression-tested on the development machine.
"""
import json
import math


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(float(value))
    except OverflowError:
        return False


def parse_window_status(raw, expected_width=640, expected_height=480,
                        min_width_px=25.0,
                        min_height_px=25.0, allowed_methods=()):
    """Validate one ``window_test`` JSON status and return a target dict.

    A target is accepted only when the producer explicitly identifies itself
    as the RGB/no-control detector, the image dimensions are known, exactly
    one target exists, and its capture timestamp is nonzero. The caller must
    also check timestamp age/replays. ``None`` requires a no-target heartbeat.
    """
    try:
        if isinstance(raw, str) and len(raw) > 32768:
            return None, 'status_too_large'
        data = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError, RecursionError):
        return None, 'invalid_json'
    if not isinstance(data, dict):
        return None, 'invalid_status'
    if data.get('no_control') is not True or data.get('rgb_only') is not True:
        return None, 'untrusted_detector'
    if data.get('input_encoding') not in ('rgb8', 'bgr8'):
        return None, 'unexpected_image_encoding'
    if (data.get('state') != 'found' or type(data.get('count')) is not int
            or data['count'] != 1):
        return None, 'target_absent_or_ambiguous'
    width = data.get('image_width')
    height = data.get('image_height')
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        return None, 'missing_image_dimensions'
    if expected_width > 0 and int(width) != int(expected_width):
        return None, 'unexpected_image_width'
    if expected_height > 0 and int(height) != int(expected_height):
        return None, 'unexpected_image_height'
    stamp = data.get('capture_stamp')
    if not _finite(stamp) or not 0.0 < stamp < 4294967296.0:
        return None, 'missing_capture_stamp'
    sequence = data.get('frame_seq')
    if type(sequence) is not int or not 0 <= sequence <= 4294967295:
        return None, 'invalid_frame_sequence'
    if type(data.get('camera_frame_id', '')) is not str:
        return None, 'invalid_camera_frame'
    # The caller supplies now separately so this function remains ROS-free.
    target_list = data.get('targets')
    if not isinstance(target_list, list) or len(target_list) != 1:
        return None, 'target_list_invalid'
    target = target_list[0]
    if not isinstance(target, dict):
        return None, 'target_invalid'
    method = target.get('method', '')
    if not allowed_methods or method not in allowed_methods:
        return None, 'method_not_allowed'
    outline = target.get('outline_kind')
    if outline not in ('opening', 'outer_frame'):
        return None, 'outline_kind_invalid'
    center = target.get('center_px')
    if not isinstance(center, (list, tuple)) or len(center) != 2:
        return None, 'center_invalid'
    if not all(_finite(v) for v in center):
        return None, 'center_invalid'
    cx, cy = float(center[0]), float(center[1])
    if not (0.0 <= cx < float(width) and 0.0 <= cy < float(height)):
        return None, 'center_out_of_bounds'
    tw = target.get('width_px')
    th = target.get('height_px')
    if not _finite(tw) or not _finite(th) or float(tw) < min_width_px or float(th) < min_height_px:
        return None, 'target_too_small'
    if max(tw, th) > math.hypot(width, height):
        return None, 'target_geometry_invalid'
    # The age check is done by ``capture_age`` below; retain the original
    # timestamp so the ROS node can preserve it in the PointStamped header.
    return dict(center_x=cx, center_y=cy, width=float(tw), height=float(th),
                method=method, capture_stamp=float(stamp),
                frame_seq=sequence, image_width=width, image_height=height,
                outline_kind=outline,
                camera_frame_id=data.get('camera_frame_id', '')), 'found'


def capture_age(target, now, max_capture_age=0.8, future_tolerance=0.2):
    """Return ``(ok, reason)`` for a validated target's capture timestamp."""
    stamp = target.get('capture_stamp')
    if not _finite(stamp) or not 0.0 < stamp < 4294967296.0:
        return False, 'missing_capture_stamp'
    if not _finite(now) or now <= 0:
        return False, 'invalid_ros_time'
    age = now - stamp
    if age > float(max_capture_age):
        return False, 'stale_capture'
    if age < -float(future_tolerance):
        return False, 'future_capture'
    return True, 'fresh'


class Confirmation:
    """N fresh hits authorize coordinates; any miss revokes them immediately.

    M misses confirm absence for diagnostics only. They never authorize the
    use of an old coordinate, and recovery always needs N new valid frames.
    """
    def __init__(self, on_frames=5, off_frames=3):
        if int(on_frames) < 1 or int(off_frames) < 1:
            raise ValueError('on_frames/off_frames must be >= 1')
        self.on_frames = int(on_frames)
        self.off_frames = int(off_frames)
        self.hits = 0
        self.misses = 0
        self.on = False

    def found(self):
        self.hits = min(self.on_frames, self.hits + 1)
        self.misses = 0
        if self.hits >= self.on_frames:
            self.on = True
        return self.on

    def missing(self):
        self.misses = min(self.off_frames, self.misses + 1)
        self.hits = 0
        self.on = False
        return self.on

    def invalidate(self):
        self.hits = 0
        self.misses = self.off_frames
        self.on = False

