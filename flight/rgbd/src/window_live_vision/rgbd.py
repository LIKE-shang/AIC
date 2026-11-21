"""RGB-D observations, not a flight authority. No assumed opening size.

Software registration explicitly reprojects calibrated RAW depth into RGB.
No image resize, frame-name rewrite, depth hole filling or unknown->free rule.
"""
import math
from collections import deque
import cv2
import numpy as np
from .geometry import finite, rays, camera_model, quaternion_matrix, estimate_plane
from .detector import Detection, contour_quad, quad_center, polygon_iou


def depth_meters(depth, encoding):
    if encoding == '16UC1' and depth.dtype == np.uint16:
        return depth.astype(np.float64)*.001
    if encoding == '32FC1' and depth.dtype == np.float32:
        return depth.astype(np.float64)
    raise ValueError('unsupported_depth_units_or_encoding')


def project_rgb_pixels(xyz, k, distortion):
    """Project already transformed optical XYZ without an unused Jacobian.

    Exact OpenCV Brown/rational model for 4/5/8 coefficients and standard K.
    Thin-prism/tilted sensor models and nonstandard K use the original OpenCV
    operation. This does not resize, approximate, or fill the depth image.
    """
    d = np.asarray(distortion, np.float64).ravel()
    if len(d) not in (4, 5, 8) or k[0, 1] != 0 or k[1, 0] != 0:
        return cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
    coeff = np.zeros(8, np.float64)
    coeff[:len(d)] = d
    k1, k2, p1, p2, k3, k4, k5, k6 = coeff
    x, y = xyz[:, 0]/xyz[:, 2], xyz[:, 1]/xyz[:, 2]
    r2 = x*x + y*y
    with np.errstate(divide='ignore', invalid='ignore', over='ignore'):
        radial = (1. + r2*(k1 + r2*(k2 + r2*k3)))/(1. + r2*(k4 + r2*(k5 + r2*k6)))
        xy2 = 2.*x*y
        xd = x*radial + p1*xy2 + p2*(r2 + 2.*x*x)
        yd = y*radial + p1*(r2 + 2.*y*y) + p2*xy2
        return np.column_stack((k[0, 0]*xd+k[0, 2], k[1, 1]*yd+k[1, 2]))


class DepthRegistrar:
    def __init__(self):
        self.key = None
        self.cached_rays = None

    def register(self, depth, encoding, depth_stamp, depth_frame, rgb_info,
                 depth_info, image_stamp, image_frame, shape, rotation, translation):
        if depth.ndim != 2 or not math.isfinite(depth_stamp) or depth_stamp <= 0:
            raise ValueError('invalid_depth_header_or_dimensions')
        if not math.isfinite(image_stamp) or abs(depth_stamp-image_stamp) > .035:
            raise ValueError('depth_not_time_aligned')
        if not depth_frame or depth_frame == image_frame:
            raise ValueError('software_registration_requires_distinct_raw_optical_frames')
        h, w = shape[:2]
        kr, dr = camera_model(rgb_info, image_frame, w, h, image_stamp)
        kd, dd = camera_model(depth_info, depth_frame, depth.shape[1], depth.shape[0], depth_stamp)
        r, t = finite(rotation, (3, 3)), finite(translation, (3,))
        if not np.allclose(r.T.dot(r), np.eye(3), atol=1e-5) or abs(np.linalg.det(r)-1) > 1e-5:
            raise ValueError('invalid_depth_to_rgb_rotation')
        if not .001 <= np.linalg.norm(t) <= .25:
            # This Gemini profile requires a real inter-camera baseline, never
            # the driver's zero-transform fallback for an invalid calibration.
            raise ValueError('invalid_or_zero_depth_rgb_baseline')
        key = (depth.shape, tuple(kd.ravel()), tuple(dd))
        if self.key != key:
            yy, xx = np.indices(depth.shape)
            self.cached_rays = rays(np.column_stack((xx.ravel(), yy.ravel())), kd, dd)
            self.key = key
        z = depth_meters(depth, encoding).ravel()
        good = np.isfinite(z) & (z >= .20) & (z <= 8.)
        xyz = (self.cached_rays[good]*z[good, None]).dot(r.T)+t
        good = xyz[:, 2] > .20
        xyz = xyz[good]
        if len(xyz) < 100:
            raise ValueError('insufficient_raw_depth_for_registration')
        if np.all(np.abs(dr) < 1e-12):
            projected = xyz.dot(kr.T)
            pixels = projected[:, :2]/projected[:, 2:3]
        else:
            pixels = project_rgb_pixels(xyz, kr, dr)
        rounded = np.rint(pixels)
        inside = (np.isfinite(rounded).all(axis=1) & (rounded[:, 0] >= 0) & (rounded[:, 0] < w) &
                  (rounded[:, 1] >= 0) & (rounded[:, 1] < h))
        pixels = rounded[inside].astype(np.int32)
        index = pixels[:, 1]*w+pixels[:, 0]
        buffer = np.full(h*w, np.inf, np.float64)
        np.minimum.at(buffer, index, xyz[inside, 2])  # Z-buffer: foreground wins.
        buffer[~np.isfinite(buffer)] = np.nan
        return buffer.reshape(h, w), kr, dr


def aperture_evidence(detection, meters, k, d, plane):
    mask = np.zeros(meters.shape, np.uint8)
    q = detection.corners
    inner = q.mean(axis=0)+.80*(q-q.mean(axis=0))
    cv2.fillConvexPoly(mask, np.rint(inner).astype(np.int32), 1)
    yy, xx = np.where(mask != 0)
    # Sparse deterministic sampling, separate from four frame-border samples.
    stride = max(1, len(xx)//3000)
    pixels = np.column_stack((xx[::stride], yy[::stride]))
    z = meters[pixels[:, 1], pixels[:, 0]]
    valid = np.isfinite(z) & (z >= .20) & (z <= 8.)
    rr = rays(pixels, k, d)
    separation = z*rr.dot(plane['normal_optical'])-plane['plane_distance_m']
    foreground = valid & (separation < -.03)
    coplanar = valid & (np.abs(separation) <= .03)
    farther = valid & (separation > .03)
    total = max(1, len(z))
    # An RGB rectangle painted on a wall is NOT an opening. Invalid samples
    # can support a possible silhouette, but never certify a free corridor.
    plausible = float(np.mean(foreground)) <= .01 and float(np.mean(coplanar)) <= .10
    return dict(possible_opening=plausible, interior_unknown_fraction=float(np.sum(~valid)/total),
                interior_foreground_fraction=float(np.sum(foreground)/total),
                interior_coplanar_fraction=float(np.sum(coplanar)/total),
                interior_farther_fraction=float(np.sum(farther)/total),
                unknown_is_free=False, free_corridor_verified=False)


class DepthOpeningDetector:
    """Four-supported planar holes; no RGB texture/color or metric size prior.

    This is an additional candidate generator, not a semantic guarantee that
    every depth hole is a traversable window. Each side is re-fitted from data.
    """
    def __init__(self):
        self.ray_key = None
        self.full_rays = None

    def detect(self, meters, k, d):
        h, w = meters.shape
        z = meters[::8, ::8].ravel()
        valid = np.isfinite(z) & (z >= .20) & (z <= 8.)
        if np.sum(valid) < 120:
            return []
        key = (meters.shape, tuple(np.asarray(k).ravel()), tuple(np.asarray(d).ravel()))
        if key != self.ray_key:
            yy, xx = np.indices(meters.shape)
            self.full_rays = rays(np.column_stack((xx.ravel(), yy.ravel())), k, d)
            self.ray_key = key
        coarse = self.full_rays.reshape(h, w, 3)[::8, ::8].reshape(-1, 3)
        xyz = coarse[valid]*z[valid, None]
        rng = np.random.RandomState(31)
        hypotheses = []
        samples = [xyz]*70
        # Fit also stratified depth modes: a thin near frame must not lose to
        # a huge far background plane merely because it has fewer pixels.
        bins = np.floor(xyz[:, 2]/.05).astype(int)
        unique, counts = np.unique(bins, return_counts=True)
        selected = []
        for index in np.argsort(-counts):
            mode = unique[index]
            if any(abs(mode-old) <= 3 for old in selected):
                continue
            group = xyz[np.abs(xyz[:, 2]-(mode+.5)*.05) < .10]
            if len(group) >= 30:
                samples.extend([group]*20)
                selected.append(mode)
            if len(selected) >= 6:
                break
        for group in samples:
            a, b, c = group[rng.choice(len(group), 3, replace=False)]
            n = np.cross(b-a, c-a)
            length = np.linalg.norm(n)
            if length < 1e-4:
                continue
            n /= length
            distance = float(n.dot(a))
            if distance < 0:
                n, distance = -n, -distance
            if abs(n[2]) < .35 or distance < .20:
                continue
            inliers = np.abs(xyz.dot(n)-distance) < .018
            count = int(np.sum(inliers))
            if count < 80:
                continue
            hypotheses.append((count, n, distance))
        hypotheses.sort(key=lambda row: -row[0])
        planes = []
        for _, n, distance in hypotheses:
            if any(abs(distance-p[1]) < .04 and n.dot(p[0]) > .995 for p in planes):
                continue
            planes.append((n, distance))
            if len(planes) == 4:
                break
        rr = self.full_rays
        result = []
        for n, distance in planes:
            residual = np.abs(meters.ravel()*rr.dot(n)-distance).reshape(h, w)
            mask = (np.isfinite(meters) & (residual < .025)).astype(np.uint8)*255
            # Only repair one-pixel sampling gaps; never bridge a missing side.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
            if hierarchy is None:
                continue
            for contour, tree in zip(contours, hierarchy[0]):
                if tree[3] < 0:
                    continue  # require an enclosed hole, not a clipped edge
                q = contour_quad(contour, meters.shape)
                if q is None or any(polygon_iou(q, old.corners) > .55 for old in result):
                    continue
                for band in (.035, .055, .08):
                    candidate = Detection(q, quad_center(q), .8, 'depth_four_border_plane_hole',
                                          [dict(band_fraction=band, depth_plane_supported=True)]*4)
                    try:
                        fitted = estimate_plane(candidate, meters, k, d)
                        if abs(fitted['plane_distance_m']-distance) > .04 or fitted['normal_optical'].dot(n) < .99:
                            continue
                        evidence = aperture_evidence(candidate, meters, k, d, fitted)
                        if not evidence['possible_opening']:
                            continue
                        result.append(candidate)
                        break
                    except (ValueError, cv2.error):
                        continue
        return result


def fuse_candidates(rgb, depth):
    result = list(rgb)
    for candidate in depth:
        matches = [i for i, old in enumerate(result) if polygon_iou(old.corners, candidate.corners) > .55]
        if len(matches) == 1:
            # Keep the generally sharper RGB inner edge, record independent
            # depth support. Distinct targets remain distinct and block choice.
            old = result[matches[0]]
            old.method += '+depth_plane' if '+depth_plane' not in old.method else ''
        elif not matches:
            result.append(candidate)
    return result


def world_geometry(plane, pose, camera_xyz=(.08, 0., -.06), camera_q=(-.5, .5, -.5, .5)):
    rb = finite(pose['rotation'], (3, 3))
    origin = finite(pose['position'], (3,))+rb.dot(finite(camera_xyz, (3,)))
    rotation = rb.dot(quaternion_matrix(camera_q))
    corners = finite(plane['corners_optical_m'], (4, 3)).dot(rotation.T)+origin
    normal = rotation.dot(finite(plane['normal_optical'], (3,)))
    return dict(center_enu_m=(rotation.dot(plane['center_optical_m'])+origin).tolist(),
                corners_enu_m=corners.tolist(), normal_enu=normal.tolist(),
                source_stamp=float(pose['stamp']), local_frame_id=pose['frame_id'],
                pose_epoch=int(pose.get('epoch', 0)), plane_rms_m=plane['rms_m'],
                depth_inlier_fraction=plane['depth_inlier_fraction'],
                metric_source='live_rgbd_four_frame_borders',
                geometry_is_approximate=True, metric_accuracy_verified=False,
                opening_evidence=plane.get('opening_evidence', {}), actual_output_permitted=False)


class MetricTracker:
    def __init__(self, fast_confirmation=False):
        if type(fast_confirmation) is not bool:raise ValueError('confirmation policy must be boolean')
        self.fast_confirmation=fast_confirmation
        self.required_frames=3 if fast_confirmation else 5
        self.minimum_span_s=.35 if fast_confirmation else .7
        self.maximum_gap_s=.60 if fast_confirmation else .40
        self.rows = deque(maxlen=5 if fast_confirmation else 8)

    def reset(self, reason):
        self.rows.clear()
        return dict(ready=False, reason=reason, geometry=None)

    def update(self, geometry, now):
        stamp = geometry['source_stamp']
        if not math.isfinite(now) or not -.02 <= now-stamp <= .50:
            return self.reset('metric_geometry_stale_or_future')
        if self.fast_confirmation:
            while self.rows and stamp-self.rows[0]['source_stamp']>1.2:self.rows.popleft()
        if self.rows:
            old = self.rows[-1]
            gap = stamp-old['source_stamp']
            same_session = (old['local_frame_id'], old['pose_epoch'], old.get('calibration_epoch', 0)) == (geometry['local_frame_id'], geometry['pose_epoch'], geometry.get('calibration_epoch', 0))
            jump = np.linalg.norm(finite(geometry['center_enu_m'])-finite(old['center_enu_m']))
            corner_jump = np.max(np.linalg.norm(finite(geometry['corners_enu_m'])-finite(old['corners_enu_m']), axis=1))
            angle = math.acos(float(np.clip(np.dot(geometry['normal_enu'], old['normal_enu']), -1, 1)))
            if not same_session or not 0 < gap <= self.maximum_gap_s or jump > .06 or corner_jump > .08 or angle > math.radians(4):
                self.reset('metric_geometry_discontinuity')
        self.rows.append(geometry)
        if len(self.rows) < self.required_frames or stamp-self.rows[0]['source_stamp'] < self.minimum_span_s:
            return dict(ready=False, reason='confirming_metric_geometry', geometry=None)
        corners = np.asarray([r['corners_enu_m'] for r in self.rows])
        centers = np.asarray([r['center_enu_m'] for r in self.rows])
        scatter = float(np.max(np.std(corners, axis=0)))
        if scatter > .025 or np.max(np.std(centers, axis=0)) > .020:
            return dict(ready=False, reason='metric_geometry_not_stable', geometry=None)
        mean = dict(geometry)
        n = np.mean([r['normal_enu'] for r in self.rows], axis=0)
        n /= np.linalg.norm(n)
        mean.update(corners_enu_m=np.mean(corners, axis=0).tolist(), center_enu_m=np.mean(centers, axis=0).tolist(),
                    normal_enu=n.tolist(), confirmation_frames=len(self.rows),
                    temporal_corner_scatter_m=scatter, scatter_is_accuracy_bound=False,
                    confirmation_span_s=stamp-self.rows[0]['source_stamp'],
                    confirmation_policy='three_frames_035s' if self.fast_confirmation else 'five_frames_07s')
        # Do not average away a newly observed foreground obstruction.
        mean['opening_evidence'] = dict(geometry.get('opening_evidence', {}))
        return dict(ready=True, reason='stable_live_metric_geometry', geometry=mean)