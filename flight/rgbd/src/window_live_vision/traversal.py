"""Metric aperture erosion, swept-volume checks and OFFLINE traversal model.

All return values remain advisory. There are no ROS publishers or FCU calls.
A frozen path is not a certificate of a clear, dynamically changing exit area.
"""
import hashlib
import json
import math
from dataclasses import dataclass
import numpy as np
from .geometry import finite


class PlanRejected(ValueError):
    pass


@dataclass
class Safety:
    vehicle_size_m: tuple = (.62, .62, .35)
    vehicle_center_body_m: tuple = (0., 0., -.09)
    position_error_bound_m: object = None
    depth_geometry_error_bound_m: object = None
    extrinsic_error_bound_m: object = None
    tracking_error_bound_m: object = None
    stopping_margin_m: object = None
    frame_thickness_bound_m: object = None
    geofence_min_m: object = None
    geofence_max_m: object = None
    clearance_margin_m: float = .04
    yaw_error_bound_rad: float = math.radians(5)
    tilt_bound_rad: float = math.radians(5)
    align_tolerance_m: float = .015
    pre_plane_distance_m: float = .8
    post_plane_clearance_m: float = .25
    max_speed_mps: float = .04
    max_accel_mps2: float = .02
    max_target_lead_m: float = .05
    yaw_rate_rad_s: float = .10
    bounds_source: str = 'UNMEASURED'

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise ValueError('safety_config_must_be_object')
        unknown = set(data)-set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError('unknown_safety_keys:'+','.join(sorted(unknown)))
        return cls(**data)

    def validate(self):
        required = ('position_error_bound_m', 'depth_geometry_error_bound_m', 'extrinsic_error_bound_m',
                    'tracking_error_bound_m', 'stopping_margin_m', 'frame_thickness_bound_m',
                    'geofence_min_m', 'geofence_max_m')
        missing = [key for key in required if getattr(self, key) is None]
        if missing:
            raise PlanRejected('missing_safety_bounds:'+','.join(missing))
        size = finite(self.vehicle_size_m, (3,))
        finite(self.vehicle_center_body_m, (3,))
        if np.min(size) <= 0 or np.max(size) > 3:
            raise PlanRejected('invalid_vehicle_envelope')
        for key in required[:6]+('clearance_margin_m', 'align_tolerance_m', 'pre_plane_distance_m',
                    'post_plane_clearance_m', 'max_speed_mps', 'max_accel_mps2', 'max_target_lead_m', 'yaw_rate_rad_s'):
            value = float(getattr(self, key))
            if not math.isfinite(value) or value <= 0:
                raise PlanRejected('invalid_positive_safety_bound:'+key)
        for key in ('yaw_error_bound_rad', 'tilt_bound_rad'):
            if not math.isfinite(float(getattr(self, key))) or not 0 <= getattr(self, key) <= math.radians(15):
                raise PlanRejected('invalid_attitude_bound:'+key)
        low, high = finite(self.geofence_min_m, (3,)), finite(self.geofence_max_m, (3,))
        if np.any(high <= low):
            raise PlanRejected('invalid_geofence')
        # Offline engineering limits, not flight-validated controller settings.
        if self.max_speed_mps > .15 or self.max_accel_mps2 > .10 or self.max_target_lead_m > .10:
            raise PlanRejected('trajectory_exceeds_design_limits')
        if self.align_tolerance_m >= self.tracking_error_bound_m:
            raise PlanRejected('alignment_tolerance_must_be_inside_tracking_bound')
        if not isinstance(self.bounds_source, str) or not self.bounds_source or self.bounds_source == 'UNMEASURED':
            raise PlanRejected('safety_bounds_need_explicit_provenance')

    @property
    def margin(self):
        return sum(float(getattr(self, key)) for key in ('position_error_bound_m', 'depth_geometry_error_bound_m',
                 'extrinsic_error_bound_m', 'tracking_error_bound_m', 'stopping_margin_m', 'clearance_margin_m'))

    @property
    def attitude_inflation(self):
        radius = np.linalg.norm(np.asarray(self.vehicle_size_m)/2)+np.linalg.norm(self.vehicle_center_body_m)
        return float(2*radius*math.sin((self.yaw_error_bound_rad+self.tilt_bound_rad)/2))


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def polygon_area(q):
    return float(np.sum(q[:, 0]*np.roll(q[:, 1], -1)-q[:, 1]*np.roll(q[:, 0], -1))/2)


def polygon_centroid(q):
    area = polygon_area(q)
    cross = q[:, 0]*np.roll(q[:, 1], -1)-q[:, 1]*np.roll(q[:, 0], -1)
    if area <= 1e-7:
        raise PlanRejected('empty_safe_aperture')
    return np.sum((q+np.roll(q, -1, axis=0))*cross[:, None], axis=0)/(6*area)


def clip_halfplane(polygon, normal, bound):
    if len(polygon) == 0:
        return polygon
    result = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        da, db = float(normal.dot(a)-bound), float(normal.dot(b)-bound)
        if da >= -1e-10:
            result.append(a)
        if (da > 0) != (db > 0):
            result.append(a+(b-a)*(da/(da-db)))
    return np.asarray(result, np.float64).reshape(-1, 2)


def basis(geometry):
    center = finite(geometry['center_enu_m'], (3,))
    n = finite(geometry['normal_enu'], (3,))
    if abs(np.linalg.norm(n)-1) > .01 or abs(n[2]) > .20:
        raise PlanRejected('unsupported_window_normal_or_tilt')
    n /= np.linalg.norm(n)
    u = np.cross([0., 0., 1.], n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    corners = finite(geometry['corners_enu_m'], (4, 3))
    if np.max(np.abs((corners-center).dot(n))) > .03:
        raise PlanRejected('opening_corners_not_coplanar')
    q = np.column_stack(((corners-center).dot(u), (corners-center).dot(v)))
    if polygon_area(q) < 0:
        q = q[::-1].copy()
    if polygon_area(q) <= 1e-6:
        raise PlanRejected('degenerate_opening_polygon')
    edges = np.roll(q, -1, axis=0)-q
    turn = edges[:, 0]*np.roll(edges[:, 1], -1)-edges[:, 1]*np.roll(edges[:, 0], -1)
    if np.any(turn <= 1e-7) or np.min(np.linalg.norm(edges, axis=1)) < .08:
        raise PlanRejected('opening_must_be_convex_with_four_supported_edges')
    return center, n, u, v, q


def pose_valid(pose, now, max_age=.20):
    stamp = float(pose['stamp'])
    if not math.isfinite(now) or not math.isfinite(stamp) or not -.02 <= now-stamp <= max_age:
        raise PlanRejected('pose_stale_or_future')
    finite(pose['position'], (3,))
    r = finite(pose['rotation'], (3, 3))
    if not np.allclose(r.T.dot(r), np.eye(3), atol=1e-5) or np.linalg.det(r) < .999:
        raise PlanRejected('invalid_body_rotation')
    if not pose.get('frame_id'):
        raise PlanRejected('pose_frame_missing')


def plan_route(geometry, pose, safety, now):
    safety.validate()
    pose_valid(pose, now)
    if (geometry['local_frame_id'], geometry.get('pose_epoch', 0)) != (pose['frame_id'], pose.get('epoch', 0)):
        raise PlanRejected('geometry_pose_session_mismatch')
    if not -.02 <= now-float(geometry['source_stamp']) <= .50:
        raise PlanRejected('geometry_stale_or_future')
    if not geometry.get('opening_evidence', {}).get('possible_opening', False):
        raise PlanRejected('depth_does_not_support_an_aperture')
    center, n, u, v, polygon = basis(geometry)
    position = finite(pose['position'], (3,))
    yaw = math.atan2(n[1], n[0])
    rotation = yaw_rotation(yaw)
    half = finite(safety.vehicle_size_m, (3,))/2
    offset = rotation.dot(finite(safety.vehicle_center_body_m, (3,)))
    safe = polygon.copy()
    constraints = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = b-a
        inward = np.array([-edge[1], edge[0]])/np.linalg.norm(edge)
        world_inward = inward[0]*u+inward[1]*v
        support = float(np.abs(world_inward.dot(rotation)).dot(half))+safety.attitude_inflation
        bound = float(inward.dot(a))+support-float(world_inward.dot(offset))+safety.margin
        constraints.append((inward, bound))
        safe = clip_halfplane(safe, inward, bound)
        if len(safe) < 3:
            raise PlanRejected('vehicle_plus_error_budget_does_not_fit_opening')
    crossing2 = polygon_centroid(safe)
    slack = min(float(e.dot(crossing2)-b) for e, b in constraints)
    if slack < 2*safety.align_tolerance_m:
        raise PlanRejected('safe_center_region_too_narrow_for_alignment')
    crossing = center+u*crossing2[0]+v*crossing2[1]
    normal_support = float(np.abs(n.dot(rotation)).dot(half))+abs(float(n.dot(offset)))+safety.attitude_inflation
    slab = safety.frame_thickness_bound_m/2
    commit_distance = max(safety.pre_plane_distance_m, normal_support+safety.margin+slab+.10)
    distance = float(n.dot(center-position))
    if distance < commit_distance+safety.align_tolerance_m:
        raise PlanRejected('start_too_close_to_frame_for_safe_alignment')
    align = crossing-n*distance
    pre = crossing-n*commit_distance
    exit_distance = normal_support+safety.margin+slab+safety.post_plane_clearance_m
    exit_point = crossing+n*exit_distance
    waypoints = np.asarray([position, align, pre, crossing, exit_point])
    # The AABB of each full body envelope remains inside the local geofence.
    extent = np.abs(rotation).dot(half)+safety.attitude_inflation+safety.margin
    low, high = finite(safety.geofence_min_m), finite(safety.geofence_max_m)
    for point in waypoints:
        if np.any(point+offset-extent < low) or np.any(point+offset+extent > high):
            raise PlanRejected('swept_body_exceeds_geofence')
    start_yaw = math.atan2(pose['rotation'][1, 0], pose['rotation'][0, 0])
    if abs(wrap(start_yaw-yaw)) > .01:
        turning_radius = float(np.linalg.norm(half)+np.linalg.norm(safety.vehicle_center_body_m))+safety.margin
        if np.any(position-turning_radius < low) or np.any(position+turning_radius > high):
            raise PlanRejected('yaw_alignment_swept_body_exceeds_geofence')
    geometry_json = dict(center_enu_m=center.tolist(), normal_enu=n.tolist(), corners_enu_m=geometry['corners_enu_m'])
    identity = dict(geometry=geometry_json, waypoints_enu_m=waypoints.tolist(), safety=safety.__dict__,
                    local_frame_id=pose['frame_id'], pose_epoch=pose.get('epoch', 0))
    plan_id = hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()[:20]
    return dict(plan_id=plan_id, kind='RGBD_CONDITIONAL_TRAVERSAL_PLAN',
                geometry=geometry_json, geometry_stamp=geometry['source_stamp'],
                local_frame_id=pose['frame_id'], pose_epoch=pose.get('epoch', 0),
                safe_center_polygon_uv_m=safe.tolist(), safe_center_min_slack_m=slack,
                opening_polygon_uv_m=polygon.tolist(), plane_basis_u_enu=u.tolist(), plane_basis_v_enu=v.tolist(),
                body_origin_crossing_enu_m=crossing.tolist(), target_yaw_rad=yaw, start_yaw_rad=start_yaw,
                waypoints_enu_m=waypoints.tolist(), waypoint_names=['START', 'ALIGN', 'COMMIT_STANDOFF', 'PLANE', 'EXIT_FULL_BODY_CLEAR'],
                commit_distance_m=commit_distance, exit_distance_m=exit_distance,
                envelope_normal_support_m=normal_support, frame_half_thickness_m=slab,
                margin_m=safety.margin, attitude_inflation_m=safety.attitude_inflation,
                path_length_m=float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))),
                limits=dict(speed_mps=safety.max_speed_mps, acceleration_mps2=safety.max_accel_mps2,
                            target_lead_m=safety.max_target_lead_m, yaw_rate_rad_s=safety.yaw_rate_rad_s),
                safety=safety.__dict__.copy(), corridor_status='UNASSESSED',
                clearance_is_conditional=True, bounds_source=safety.bounds_source,
                actual_output_permitted=False, accepted_real_window_pass=False,
                requires_independent_flight_validation=True)


def swept_cells(plan, cell_m=.05, max_cells=120000):
    """All grid cubes intersecting a conservative continuous swept envelope."""
    if not .025 <= cell_m <= .10:
        raise PlanRejected('invalid_corridor_grid_resolution')
    safety = Safety.from_dict(plan['safety'])
    safety.validate()
    points = finite(plan['waypoints_enu_m'])
    r = yaw_rotation(plan['target_yaw_rad'])
    offset = r.dot(finite(safety.vehicle_center_body_m))
    half = finite(safety.vehicle_size_m)/2+safety.margin+safety.attitude_inflation
    extent = np.abs(r).dot(half)+math.sqrt(3)*cell_m
    low = np.floor((np.min(points, axis=0)+offset-extent)/cell_m).astype(int)
    high = np.ceil((np.max(points, axis=0)+offset+extent)/cell_m).astype(int)
    turning = abs(wrap(plan['start_yaw_rad']-plan['target_yaw_rad'])) > .01
    turn_radius = float(np.linalg.norm(np.asarray(safety.vehicle_size_m)/2)+np.linalg.norm(safety.vehicle_center_body_m))+safety.margin+safety.attitude_inflation
    if turning:
        low = np.minimum(low, np.floor((points[0]-turn_radius-cell_m)/cell_m).astype(int))
        high = np.maximum(high, np.ceil((points[0]+turn_radius+cell_m)/cell_m).astype(int))
    if np.prod(high-low+1) > max_cells:
        raise PlanRejected('corridor_grid_too_large')
    grid = np.stack(np.meshgrid(*(np.arange(low[i], high[i]+1) for i in range(3)), indexing='ij'), axis=-1).reshape(-1, 3)
    xyz = (grid+.5)*cell_m
    required = np.zeros(len(xyz), bool)
    inflated = half+math.sqrt(3)*cell_m/2
    for a, b in zip(points, points[1:]):
        local = (xyz-a-offset).dot(r)
        delta = (b-a).dot(r)
        lo, hi = np.zeros(len(xyz)), np.ones(len(xyz))
        intersects = np.ones(len(xyz), bool)
        for axis in range(3):
            if abs(delta[axis]) < 1e-10:
                intersects &= np.abs(local[:, axis]) <= inflated[axis]
            else:
                t1 = (local[:, axis]-inflated[axis])/delta[axis]
                t2 = (local[:, axis]+inflated[axis])/delta[axis]
                lo = np.maximum(lo, np.minimum(t1, t2))
                hi = np.minimum(hi, np.maximum(t1, t2))
        required |= intersects & (lo <= hi)
    if turning:
        required |= np.linalg.norm(xyz-points[0], axis=1) <= turn_radius+math.sqrt(3)*cell_m/2
    return xyz[required], cell_m


def evaluate_corridor(plan, meters, k, d, optical_to_world_r, camera_origin_world, stamp, now, cell_m=.05):
    """One-view conservative evidence. Missing/FOV/occluded cells stay UNKNOWN.

    No depth-to-ground assumption. No corridor is approved by a zero-return.
    Requires a zero-distortion grid (rectify deliberately, never just relabel).
    Independent time-sync/calibration/range validation is still required.
    """
    if not -.02 <= now-stamp <= .30:
        raise PlanRejected('corridor_image_stale_or_future')
    if np.any(np.abs(finite(d)) > 1e-10):
        raise PlanRejected('corridor_requires_rectified_zero_distortion_depth')
    k = finite(k, (3, 3))
    if min(k[0, 0], k[1, 1]) <= 0 or abs(k[0, 1])+abs(k[1, 0]) > 1e-8:
        raise PlanRejected('invalid_corridor_intrinsics')
    r = finite(optical_to_world_r, (3, 3))
    if not np.allclose(r.T.dot(r), np.eye(3), atol=1e-5):
        raise PlanRejected('invalid_corridor_rotation')
    xyz, cell_m = swept_cells(plan, cell_m)
    optical = (xyz-finite(camera_origin_world, (3,))).dot(r)
    safety = Safety.from_dict(plan['safety'])
    radius = math.sqrt(3)*cell_m/2+safety.position_error_bound_m+safety.extrinsic_error_bound_m
    h, w = meters.shape
    known_free = np.zeros(len(xyz), bool)
    occupied = np.zeros(len(xyz), bool)
    candidates = np.where(optical[:, 2] > .20+radius)[0]
    if len(candidates):
        q = optical[candidates]
        px = k[0, 0]*q[:, 0]/q[:, 2]+k[0, 2]
        py = k[1, 1]*q[:, 1]/q[:, 2]+k[1, 2]
        # Project a circumscribed ball conservatively onto the pinhole grid.
        rx = k[0, 0]*radius*(1+np.abs(q[:, 0]/q[:, 2]))/(q[:, 2]-radius)+1
        ry = k[1, 1]*radius*(1+np.abs(q[:, 1]/q[:, 2]))/(q[:, 2]-radius)+1
        usable = (px-rx >= 0) & (px+rx < w-1) & (py-ry >= 0) & (py+ry < h-1)
        # Integral unknown mask + range-minimum pyramid, no interpolation.
        import cv2
        invalid = ~np.isfinite(meters) | (meters < .20) | (meters > 8.)
        unknown_integral = cv2.integral(invalid.astype(np.uint8))
        safe_depth = np.where(invalid, 0., meters).astype(np.float32)
        for idx in np.where(usable)[0]:
            x1, x2 = int(math.floor(px[idx]-rx[idx])), int(math.ceil(px[idx]+rx[idx]))
            y1, y2 = int(math.floor(py[idx]-ry[idx])), int(math.ceil(py[idx]+ry[idx]))
            unknown = unknown_integral[y2+1, x2+1]-unknown_integral[y1, x2+1]-unknown_integral[y2+1, x1]+unknown_integral[y1, x1]
            if unknown:
                continue
            nearest = float(np.min(safe_depth[y1:y2+1, x1:x2+1]))
            furthest_cell_z = q[idx, 2]+radius
            source_index = candidates[idx]
            if nearest-safety.depth_geometry_error_bound_m > furthest_cell_z:
                known_free[source_index] = True
            elif nearest+safety.depth_geometry_error_bound_m >= q[idx, 2]-radius:
                occupied[source_index] = True
            # A nearer surface fully occluding a cell is UNKNOWN behind it.
    unknown = ~(known_free | occupied)
    count = len(xyz)
    clear = count > 0 and int(np.sum(known_free)) == count
    return dict(plan_id=plan['plan_id'], local_frame_id=plan['local_frame_id'], pose_epoch=plan['pose_epoch'],
                source_stamp=stamp, grid_m=cell_m, required_cells=count, observed_free_cells=int(np.sum(known_free)),
                occupied_cells=int(np.sum(occupied)), unknown_cells=int(np.sum(unknown)),
                clear=clear, reason='OBSERVED_FREE_CONDITIONAL' if clear else 'CORRIDOR_OCCUPIED_OR_UNKNOWN',
                evidence_scope='single_view_depth_conditional_on_sensor_bounds', unknown_is_free=False,
                actual_output_permitted=False, flight_validated=False)


def evidence_ready(evidence, plan, now):
    try:
        return _evidence_ready(evidence, plan, now)
    except (ValueError, TypeError, KeyError, OverflowError):
        return False


def _evidence_ready(evidence, plan, now):
    if not isinstance(evidence, dict):
        return False
    if (evidence.get('plan_id'), evidence.get('local_frame_id'), evidence.get('pose_epoch')) != (plan['plan_id'], plan['local_frame_id'], plan['pose_epoch']):
        return False
    age = now-float(evidence.get('source_stamp', -1e9))
    required = evidence.get('required_cells', 0)
    return (math.isfinite(age) and -.02 <= age <= .30 and isinstance(required, int) and required > 0
            and evidence.get('observed_free_cells') == required and evidence.get('unknown_cells') == 0
            and evidence.get('occupied_cells') == 0 and evidence.get('clear') is True)


def motion_profile(length, elapsed, max_speed, max_accel):
    """Analytic rest-to-rest trapezoid/triangle; hard speed/acceleration caps."""
    if not all(math.isfinite(float(x)) for x in (length, elapsed, max_speed, max_accel)) or length < 0 or min(max_speed, max_accel) <= 0:
        raise ValueError('invalid_motion_profile')
    if length < 1e-10:
        return 0., 0., 0.
    peak = min(max_speed, math.sqrt(length*max_accel))
    ramp = peak/max_accel
    cruise = max(0., (length-peak*peak/max_accel)/peak)
    duration = 2*ramp+cruise
    t = max(0., elapsed)
    if t >= duration:
        return length, 0., duration
    if t <= ramp:
        return .5*max_accel*t*t, max_accel*t, duration
    if t <= ramp+cruise:
        return .5*peak*ramp+peak*(t-ramp), peak, duration
    remaining = duration-t
    return length-.5*max_accel*remaining*remaining, max_accel*remaining, duration

class TraversalModel:
    """Offline-only state machine; tests replay a plant, never command MAVROS.

    Fresh geometry is required before COMMIT. After COMMIT the captured plane
    is frozen; independent fresh corridor/pose evidence is STILL required.
    Failure in/near the frame requests manual emergency takeover, not blind
    automatic retreat or an assertion that stationary hold is safe.
    """
    def __init__(self, plan, pose, now, evidence, authorization):
        if authorization != 'OFFLINE_SIMULATION_ONLY':
            raise PlanRejected('this_is_not_a_flight_controller')
        pose_valid(pose, now)
        if not evidence_ready(evidence, plan, now):
            raise PlanRejected('full_swept_corridor_not_observed_free')
        self.plan = json.loads(json.dumps(plan))
        self.safety = Safety.from_dict(plan['safety'])
        self.safety.validate()
        if (pose['frame_id'], pose.get('epoch', 0)) != (plan['local_frame_id'], plan['pose_epoch']):
            raise PlanRejected('start_pose_session_does_not_match_plan')
        if not -.02 <= now-float(plan['geometry_stamp']) <= .50:
            raise PlanRejected('start_plan_geometry_stale')
        points = finite(plan['waypoints_enu_m'], (5, 3))
        if np.linalg.norm(finite(pose['position'])-points[0]) > self.safety.align_tolerance_m:
            raise PlanRejected('start_pose_moved_since_planning')
        start_yaw = math.atan2(pose['rotation'][1, 0], pose['rotation'][0, 0])
        if abs(wrap(start_yaw-plan['start_yaw_rad'])) > self.safety.yaw_error_bound_rad:
            raise PlanRejected('start_yaw_moved_since_planning')
        if math.acos(float(np.clip(pose['rotation'][2, 2], -1, 1))) > self.safety.tilt_bound_rad:
            raise PlanRejected('start_attitude_bound_exceeded')
        self.target = finite(pose['position'], (3,)).copy()
        self.yaw = math.atan2(pose['rotation'][1, 0], pose['rotation'][0, 0])
        self.state, self.reason = 'ALIGN', ''
        self.last_time, self.started = now, now
        self.speed = 0.
        self.stable_since = None
        self.frozen_stamp = None
        self.completed_stamp = None
        self.profile_goal = None
        self.profile_origin = self.target.copy()
        self.profile_elapsed = 0.

    def fail(self, reason, pose=None):
        self.state, self.reason = 'ABORT', reason
        self.speed = 0.
        self.abort_action = 'MANUAL_TAKEOVER_REQUIRED_NO_AUTOMATIC_TARGET'
        if pose is not None:
            n, c = finite(self.plan['geometry']['normal_enu']), finite(self.plan['geometry']['center_enu_m'])
            signed = float(n.dot(finite(pose['position'])-c))
            body_slab = self.plan['envelope_normal_support_m']+self.plan['frame_half_thickness_m']+self.plan['margin_m']
            if abs(signed) <= body_slab:
                self.abort_action = 'EMERGENCY_MANUAL_TAKEOVER_NEAR_OR_IN_FRAME_NO_BLIND_HOLD_OR_RETREAT'
        return self.output()

    def output(self):
        return dict(state=self.state, reason=self.reason,
                    advisory_target_position_enu_m=None if self.state == 'ABORT' else self.target.tolist(),
                    advisory_yaw_rad=None if self.state == 'ABORT' else self.yaw,
                    advisory_speed_mps=self.speed, geometry_frozen=self.frozen_stamp is not None,
                    abort_action=getattr(self, 'abort_action', None),
                    actual_output_permitted=False, output_sent_count=0, accepted_real_window_pass=False)

    def tick(self, pose, now, geometry, evidence, pilot_authorized=True):
        try:
            return self._tick(pose, now, geometry, evidence, pilot_authorized)
        except (ValueError, TypeError, KeyError, IndexError, OverflowError):
            return self.fail('malformed_input_no_automatic_target')

    def _tick(self, pose, now, geometry, evidence, pilot_authorized=True):
        if self.state in ('DONE', 'ABORT'):
            return self.output()
        try:
            pose_valid(pose, now)
        except (ValueError, KeyError):
            return self.fail('pose_invalid_or_stale')
        dt = now-self.last_time
        if not math.isfinite(dt) or not 0 < dt <= .15 or now-self.started > 180:
            return self.fail('clock_or_execution_timeout', pose)
        self.last_time = now
        if pilot_authorized is not True:
            return self.fail('pilot_takeover', pose)
        if (pose['frame_id'], pose.get('epoch', 0)) != (self.plan['local_frame_id'], self.plan['pose_epoch']):
            return self.fail('localization_session_changed', pose)
        r = pose['rotation']
        tilt = math.acos(float(np.clip(r[2, 2], -1, 1)))
        yaw_measured = math.atan2(r[1, 0], r[0, 0])
        if tilt > self.safety.tilt_bound_rad+1e-6:
            return self.fail('attitude_bound_exceeded', pose)
        if not evidence_ready(evidence, self.plan, now):
            return self.fail('swept_corridor_stale_occupied_or_unknown', pose)
        if self.frozen_stamp is None:
            if geometry is None or not -.02 <= now-float(geometry.get('source_stamp', -1e9)) <= .50:
                return self.fail('live_geometry_required_before_commit', pose)
            if (geometry.get('local_frame_id'), geometry.get('pose_epoch', 0)) != (self.plan['local_frame_id'], self.plan['pose_epoch']):
                return self.fail('live_geometry_session_changed', pose)
            if not geometry.get('opening_evidence', {}).get('possible_opening', False):
                return self.fail('opening_obstruction_before_commit', pose)
            center, normal, _, _, _ = basis(geometry)
            if np.linalg.norm(center-finite(self.plan['geometry']['center_enu_m'])) > self.safety.depth_geometry_error_bound_m:
                return self.fail('geometry_center_changed_replan_before_commit', pose)
            if normal.dot(finite(self.plan['geometry']['normal_enu'])) < math.cos(self.safety.yaw_error_bound_rad):
                return self.fail('geometry_normal_changed_replan_before_commit', pose)
            current = finite(geometry['corners_enu_m'], (4, 3))
            captured = finite(self.plan['geometry']['corners_enu_m'], (4, 3))
            if np.max(np.linalg.norm(current-captured, axis=1)) > self.safety.depth_geometry_error_bound_m:
                return self.fail('geometry_changed_replan_before_commit', pose)
        elif now-self.frozen_stamp > 60:
            return self.fail('frozen_geometry_time_budget_exceeded', pose)
        error = float(np.linalg.norm(self.target-finite(pose['position'])))
        if error > self.safety.tracking_error_bound_m+1e-6:
            return self.fail('tracking_error_bound_exceeded', pose)
        yaw_error = wrap(self.plan['target_yaw_rad']-self.yaw)
        self.yaw += float(np.clip(yaw_error, -self.safety.yaw_rate_rad_s*dt, self.safety.yaw_rate_rad_s*dt))
        yaw_target_error = abs(wrap(self.yaw-yaw_measured))
        if yaw_target_error > self.safety.yaw_error_bound_rad+1e-6:
            return self.fail('yaw_tracking_bound_exceeded', pose)
        yaw_ready = abs(wrap(self.plan['target_yaw_rad']-yaw_measured)) <= math.radians(1.)
        goal_index = {'ALIGN': 1, 'APPROACH': 2, 'COMMIT': 2, 'TRAVERSE': 4, 'EXIT': 4}[self.state]
        goal = finite(self.plan['waypoints_enu_m'][goal_index], (3,))
        goal_key = tuple(goal)
        if self.profile_goal != goal_key:
            self.profile_goal = goal_key
            self.profile_origin = self.target.copy()
            self.profile_elapsed = 0.
        path = goal-self.profile_origin
        length = float(np.linalg.norm(path))
        if yaw_ready:
            next_elapsed = self.profile_elapsed+dt
            progress, speed, _ = motion_profile(length, next_elapsed, self.safety.max_speed_mps, self.safety.max_accel_mps2)
            proposed = goal.copy() if length < 1e-10 else self.profile_origin+path*(progress/length)
            if np.linalg.norm(proposed-pose['position']) > self.safety.max_target_lead_m:
                return self.fail('target_lead_bound_exceeded', pose)
            self.target, self.speed, self.profile_elapsed = proposed, speed, next_elapsed
        elif self.speed > 1e-8:
            return self.fail('yaw_alignment_lost_during_motion', pose)
        distance = float(np.linalg.norm(goal-self.target))
        arrived = np.linalg.norm(goal-pose['position']) <= self.safety.align_tolerance_m and distance <= 1e-7 and self.speed <= 1e-7 and yaw_ready
        if arrived:
            if self.stable_since is None:
                self.stable_since = now
            if now-self.stable_since >= .20:
                self.stable_since = None
                self.speed = 0.
                if self.state == 'ALIGN':
                    self.state = 'APPROACH'
                elif self.state == 'APPROACH':
                    self.state = 'COMMIT'
                elif self.state == 'COMMIT':
                    self.frozen_stamp = now
                    self.state = 'TRAVERSE'
                elif self.state == 'EXIT':
                    self.state, self.completed_stamp = 'DONE', now
        else:
            self.stable_since = None
        if self.state == 'TRAVERSE':
            n, c = finite(self.plan['geometry']['normal_enu']), finite(self.plan['geometry']['center_enu_m'])
            if n.dot(finite(pose['position'])-c) > self.plan['envelope_normal_support_m']+self.plan['frame_half_thickness_m']+self.plan['margin_m']:
                self.state = 'EXIT'
        return self.output()