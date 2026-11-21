"""Optional registered-depth geometry and bounded step PREVIEW. No flight I/O.

Monocular pixels are deliberately never converted to meters using a guessed
opening size. Only registered RGB-depth measurements can supply this scale.
"""
import math
from collections import deque
import cv2
import numpy as np


def finite(value, shape=None):
    x = np.asarray(value, np.float64)
    if (shape is not None and x.shape != shape) or not np.isfinite(x).all():
        raise ValueError('invalid_shape_or_nonfinite_value')
    return x


def camera_model(info, frame_id, width, height, stamp):
    if info is None:
        raise ValueError('camera_info_missing')
    if info['frame_id'] != frame_id or (info['width'], info['height']) != (width, height):
        raise ValueError('camera_info_frame_or_size_mismatch')
    if not math.isfinite(info['stamp']) or not 0 < info['stamp'] or abs(info['stamp']-stamp) > 2.:
        raise ValueError('camera_info_stale')
    if info['model'] not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('unsupported_distortion_model')
    k = finite(info['K']).reshape(3, 3)
    d = finite(info['D']).ravel()
    if len(d) not in (4, 5, 8, 12, 14) or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('uncalibrated_camera')
    if not np.allclose(k[2], [0, 0, 1]) or not 0 <= k[0, 2] < width or not 0 <= k[1, 2] < height:
        raise ValueError('invalid_intrinsic_matrix')
    if info.get('binning_x', 0) not in (0, 1) or info.get('binning_y', 0) not in (0, 1) or any(info.get('roi', (0, 0, 0, 0))):
        raise ValueError('binned_or_cropped_camera_info_unsupported')
    return k, d


def rays(pixels, k, d):
    xy = cv2.undistortPoints(finite(pixels).reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return np.column_stack((xy, np.ones(len(xy))))


def quaternion_matrix(q):
    q = finite(q, (4,))
    norm = np.linalg.norm(q)
    if not .98 <= norm <= 1.02:
        raise ValueError('quaternion_not_normalized')
    x, y, z, w = q/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class PoseBuffer:
    def __init__(self):
        self.samples = deque(maxlen=120)
        self.epoch = 0

    def add(self, stamp, position, quaternion, frame_id):
        position = finite(position, (3,))
        quaternion_matrix(quaternion)
        q = finite(quaternion, (4,))
        if not math.isfinite(stamp) or stamp <= 0 or not frame_id:
            raise ValueError('invalid_pose_header')
        if self.samples and (stamp <= self.samples[-1][0] or frame_id != self.samples[-1][3] or
                             np.linalg.norm(position-self.samples[-1][1]) > max(.10, 4.*(stamp-self.samples[-1][0]))):
            self.samples.clear()
            self.epoch += 1
        self.samples.append((stamp, position, q, frame_id))

    def at(self, stamp):
        rows = list(self.samples)
        for row in rows:
            if abs(stamp-row[0]) < 1e-6:
                return dict(position=row[1], rotation=quaternion_matrix(row[2]), frame_id=row[3], stamp=stamp, epoch=self.epoch)
        for a, b in zip(rows, rows[1:]):
            if a[0] <= stamp <= b[0] and b[0]-a[0] <= .10 and a[3] == b[3]:
                ratio = (stamp-a[0])/(b[0]-a[0])
                qa, qb = a[2].copy(), b[2].copy()
                if np.dot(qa, qb) < 0:
                    qb = -qb
                # SLERP, including the small-angle limit.
                cosine = float(np.clip(np.dot(qa, qb), -1, 1))
                if cosine > .9995:
                    q = (1-ratio)*qa+ratio*qb
                else:
                    angle = math.acos(cosine)
                    q = (math.sin((1-ratio)*angle)*qa+math.sin(ratio*angle)*qb)/math.sin(angle)
                q /= np.linalg.norm(q)
                return dict(position=(1-ratio)*a[1]+ratio*b[1], rotation=quaternion_matrix(q), frame_id=a[3], stamp=stamp, epoch=self.epoch)
        raise ValueError('no_pose_bracketing_image_stamp')


def matched_depth(depth, encoding, depth_stamp, depth_frame, rgb_info, depth_info, image_stamp, image_frame, shape):
    if not math.isfinite(depth_stamp) or depth_stamp <= 0 or abs(depth_stamp-image_stamp) > .035:
        raise ValueError('depth_not_time_aligned')
    h, w = shape[:2]
    kr, dr = camera_model(rgb_info, image_frame, w, h, image_stamp)
    kd, dd = camera_model(depth_info, image_frame, w, h, depth_stamp)
    if depth_frame != image_frame or depth.shape != (h, w) or not np.allclose(kr, kd, atol=1e-6) or not np.array_equal(dr, dd):
        raise ValueError('depth_not_registered_to_rgb')
    if encoding == '16UC1' and depth.dtype == np.uint16:
        meters = depth.astype(np.float64)*.001
    elif encoding == '32FC1' and depth.dtype == np.float32:
        meters = depth.astype(np.float64)
    else:
        raise ValueError('unsupported_depth_units_or_encoding')
    return meters, kr, dr


def border_samples(detection, shape):
    q = finite(detection.corners, (4, 2)).astype(np.float32)
    canonical = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    transform = cv2.getPerspectiveTransform(canonical, q)
    all_pixels, labels = [], []
    for side in range(4):
        width = detection.sides[side]['band_fraction']
        along, offset = np.meshgrid(np.linspace(.12, .88, 32), np.linspace(.40*width, .90*width, 6))
        if side == 0:
            points = np.stack((along, -offset), axis=-1)
        elif side == 1:
            points = np.stack((1+offset, along), axis=-1)
        elif side == 2:
            points = np.stack((1-along, 1+offset), axis=-1)
        else:
            points = np.stack((-offset, 1-along), axis=-1)
        pixels = cv2.perspectiveTransform(points.astype(np.float32).reshape(-1, 1, 2), transform).reshape(-1, 2)
        pixels = np.rint(pixels).astype(int)
        if np.any(pixels < 0) or np.any(pixels[:, 0] >= shape[1]) or np.any(pixels[:, 1] >= shape[0]):
            raise ValueError('border_depth_outside_image')
        pixels = np.unique(pixels, axis=0)
        all_pixels.extend(pixels)
        labels.extend([side]*len(pixels))
    return np.asarray(all_pixels), np.asarray(labels)


def estimate_plane(detection, meters, k, d):
    pixels, sides = border_samples(detection, meters.shape)
    z = meters[pixels[:, 1], pixels[:, 0]]
    valid = np.isfinite(z) & (z >= .20) & (z <= 8.)
    for side in range(4):
        if np.sum(valid & (sides == side)) < 20 or np.mean(valid[sides == side]) < .65:
            raise ValueError('insufficient_valid_depth_on_every_border')
    xyz = rays(pixels[valid], k, d)*z[valid, None]
    labels = sides[valid]
    best = None
    rng = np.random.RandomState(7)
    for _ in range(96):
        a, b, c = xyz[rng.choice(len(xyz), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-6:
            continue
        normal /= length
        residual = np.abs(xyz.dot(normal)-np.dot(a, normal))
        inliers = residual < .02
        support = min(float(np.mean(inliers[labels == side])) for side in range(4))
        if support >= .65 and (best is None or np.sum(inliers) > np.sum(best)):
            best = inliers
    if best is None or float(np.mean(best)) < .75:
        raise ValueError('four_borders_do_not_share_depth_plane')
    points = xyz[best]
    mean = points.mean(axis=0)
    _, values, vectors = np.linalg.svd(points-mean, full_matrices=False)
    if values[1] < .01:
        raise ValueError('degenerate_depth_samples')
    normal = vectors[-1]
    distance = float(np.dot(normal, mean))
    if distance < 0:
        normal, distance = -normal, -distance
    residual = np.abs(xyz.dot(normal)-distance)
    if any(np.mean(residual[labels == side] < .02) < .65 for side in range(4)):
        raise ValueError('refined_plane_lost_border_support')
    rms = float(np.sqrt(np.mean(residual[best]**2)))
    if rms > .015 or abs(normal[2]) < .2:
        raise ValueError('unstable_depth_plane')
    rr = rays(np.vstack((detection.corners, detection.center)), k, d)
    denominators = rr.dot(normal)
    if np.min(denominators) <= .15:
        raise ValueError('grazing_or_backward_ray')
    intersections = rr*(distance/denominators[:, None])
    if not np.isfinite(intersections).all() or np.max(np.linalg.norm(intersections, axis=1)) > 10:
        raise ValueError('invalid_plane_intersections')
    return dict(center_optical_m=intersections[4], corners_optical_m=intersections[:4],
                normal_optical=normal, plane_distance_m=distance, rms_m=rms,
                depth_inlier_fraction=float(np.mean(best)),
                metric_source='registered_depth_on_four_frame_borders')


def step_preview(plane, pose, camera_to_body_xyz=(.08, 0., -.06), camera_to_body_quaternion=(-.5, .5, -.5, .5)):
    rb = finite(pose['rotation'], (3, 3))
    position = finite(pose['position'], (3,))
    rc = quaternion_matrix(camera_to_body_quaternion)
    translation = finite(camera_to_body_xyz, (3,))
    rotation = rb.dot(rc)
    center = position+rb.dot(translation)+rotation.dot(plane['center_optical_m'])
    normal = rotation.dot(plane['normal_optical'])
    lateral = np.cross([0., 0., 1.], normal)
    if np.linalg.norm(lateral) < .8:
        raise ValueError('window_plane_not_sufficiently_vertical')
    lateral /= np.linalg.norm(lateral)
    if np.dot(lateral, rotation[:, 0]) < 0:
        lateral = -lateral
    error = float(np.dot(center-position, lateral))
    delta = float(np.clip(error, -.10, .10))
    if abs(error) < .015:
        delta = 0.
    target = position+delta*lateral
    corners = plane['corners_optical_m']
    observed_width = float((np.linalg.norm(corners[1]-corners[0])+np.linalg.norm(corners[2]-corners[3]))/2.)
    observed_height = float((np.linalg.norm(corners[3]-corners[0])+np.linalg.norm(corners[2]-corners[1]))/2.)
    return dict(observed_opening_width_m=observed_width, observed_opening_height_m=observed_height,
        metric_geometry_is_approximate=True, kind='BOUNDED_LATERAL_STEP_PREVIEW', origin_position_enu=position.tolist(),
        target_position_enu=target.tolist(), target_yaw_enu_rad=float(math.atan2(rb[1, 0], rb[0, 0])),
        window_center_enu_m=center.tolist(), window_normal_enu=normal.tolist(),
        lateral_unit_enu=lateral.tolist(), lateral_error_m=error, planned_lateral_step_m=delta,
        max_step_m=.10, max_target_speed_mps=.04, target_progress_duration_s=abs(delta)/.04,
        height_target_changes=False, yaw_target_changes=False, normal_target_changes=False,
        local_frame_id=pose['frame_id'], captured_stamp=pose['stamp'],
        source=plane['metric_source'], plane_rms_m=plane['rms_m'],
        actual_output_permitted=False, position_accuracy_measured=False,
        clearance_verified=False, accepted_real_window_pass=False)


def preview_setpoint(preview, elapsed_s):
    """Offline sample of one frozen preview; never publishes or re-captures it."""
    elapsed = float(elapsed_s)
    if not math.isfinite(elapsed):
        raise ValueError('nonfinite_preview_time')
    origin = finite(preview['origin_position_enu'], (3,))
    final = finite(preview['target_position_enu'], (3,))
    delta = final-origin
    length = float(np.linalg.norm(delta))
    if length > .10000001:
        raise ValueError('preview_exceeds_total_step_budget')
    # Hard limits, not parameters supplied by another process.
    progress = min(length, .04*max(0., elapsed))
    target = origin if length < 1e-9 else origin+delta*(progress/length)
    return dict(target_position_enu=target.tolist(),
                target_yaw_enu_rad=float(preview['target_yaw_enu_rad']),
                command_progress_m=progress, actual_output_permitted=False)
