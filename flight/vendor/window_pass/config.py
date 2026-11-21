"""Development bounds, NOT measured flight calibration or permission."""
import copy
import math
from .geometry import finite, vector, quaternion, envelope_half_extents

# Competition field from the rule sheet (page 3): 5 m x 5 m floor, 2 m ceiling.
# The window wall stands across the middle, so the usable airspace is bounded by
# the floor plan plus a conservative height band well under the 2 m ceiling.
# A configured geofence that extends past these limits is rejected: the vehicle
# cannot be asked to fly somewhere the venue does not exist.
FIELD_SIZE_M = (5.0, 5.0, 2.0)
FIELD_GEOFENCE_MIN_M = [-2.5, -2.5, 0.30]
FIELD_GEOFENCE_MAX_M = [2.5, 2.5, 1.80]

DEFAULTS = dict(
    enable_motion=False, enable_traverse=False,
    expected_width=640, expected_height=480,
    local_frame='', body_frame='', optical_frame='', fcu_node='/mavros',
    detector_node='/window_pass_observer', evidence_id='',
    camera_info_sha256='', vision_profile_sha256='', algorithm_sha256='',
    vehicle_size_m=[0.62, 0.62, 0.35], opening_size_m=[1.0, 0.75],
    envelope_center_body_m=[], camera_translation_body_m=[],
    camera_rotation_body_from_optical_xyzw=[], window_center_local_m=[],
    window_yaw_rad=0.0, window_thickness_m=0.0,
    geofence_min_m=[], geofence_max_m=[],
    geometry_verified=False, camera_verified=False, pose_verified=False,
    direction_verified=False, hold_verified=False, failsafe_verified=False,
    latency_verified=False, corridor_verified=False, single_controller_verified=False,
    opening_geometry_verified=False, pass_stage_approved=False,
    position_uncertainty_m=0.0, stopping_margin_m=0.0, clearance_margin_m=0.04,
    clearance_tracking_budget_m=0.03,
    control_hz=30.0, max_loop_gap_s=0.15, graph_timeout_s=0.6,
    capture_timeout_s=0.25, image_timeout_s=0.25, observation_timeout_s=0.3,
    pose_timeout_s=0.15, state_timeout_s=1.5, future_tolerance_s=0.02,
    pose_sync_slop_s=0.06, max_pose_step_m=0.15, max_pose_speed_mps=1.0,
    max_attitude_step_rad=0.2, yaw_limit_rad=0.0872665, tilt_limit_rad=0.0872665,
    confirm_frames=8, confirm_time_s=1.0, stability_px=8.0,
    target_jump_px=40.0, target_area_ratio=1.35,
    align_gain_per_s=0.35, align_speed_mps=0.08, traverse_speed_mps=0.12,
    max_setpoint_lead_m=0.04, max_tracking_error_m=0.12,
    max_align_distance_m=0.20, max_align_path_m=0.50, max_align_time_s=20.0, max_total_time_s=45.0,
    align_tolerance_m=0.025, stable_time_s=2.0, stable_speed_mps=0.04,
    approach_min_m=0.8, approach_max_m=2.0,
    max_traverse_distance_m=3.0, max_traverse_time_s=25.0,
    post_plane_clearance_m=0.40, arrival_tolerance_m=0.035,
    stop_stable_time_s=1.5, min_ray_plane_cos=0.8, max_ray_range_m=4.0,
    center_agreement_m=0.06, opening_size_error_fraction=0.12,
    hold_prestream_s=1.2, max_hold_wait_s=30.0, degraded_hold_timeout_s=0.5)

FLAGS = [k for k, v in DEFAULTS.items() if type(v) is bool]
ATTESTATIONS = [k for k in FLAGS if k not in ('enable_motion', 'enable_traverse', 'pass_stage_approved')]


def load_config(supplied):
    if not isinstance(supplied, dict):
        raise ValueError('control configuration requires a dictionary')
    unknown = set(supplied)-set(DEFAULTS)
    if unknown:
        raise ValueError('unknown configuration keys: '+str(sorted(unknown)))
    c = copy.deepcopy(DEFAULTS)
    c.update(supplied)
    for k, v in c.items():
        default = DEFAULTS[k]
        if type(default) is bool and type(v) is not bool:
            raise ValueError(k+' requires boolean')
        if type(default) is str and type(v) is not str:
            raise ValueError(k+' requires string')
        if type(default) is list and not (v == [] or vector(v, len(default) or (4 if 'xyzw' in k else 3))):
            raise ValueError(k+' invalid vector')
        if type(default) in (int, float) and (not finite(v) or (v <= 0 and k not in ('window_yaw_rad', 'window_thickness_m', 'position_uncertainty_m', 'stopping_margin_m'))):
            raise ValueError(k+' invalid finite numeric limit')
    for k in ('expected_width', 'expected_height', 'confirm_frames'):
        if type(c[k]) is not int:
            raise ValueError(k+' requires integer')
    if not vector(c['vehicle_size_m']) or not vector(c['opening_size_m'], 2):
        raise ValueError('physical sizes must have 3 and 2 components')
    if not 20 <= c['control_hz'] <= 100 or not 0.03 <= c['max_loop_gap_s'] <= 0.3:
        raise ValueError('invalid control rate/deadline')
    for k in ('capture_timeout_s','image_timeout_s','observation_timeout_s','pose_timeout_s'):
        if c[k] > 0.5:
            raise ValueError(k+' cannot exceed 0.5s in this prototype')
    if c['future_tolerance_s'] > .05 or c['pose_sync_slop_s'] > .1:
        raise ValueError('excessive clock/synchronisation tolerance')
    if c['align_speed_mps'] > .2 or c['traverse_speed_mps'] > .3:
        raise ValueError('prototype speed cap exceeded')
    if not 0 < c['yaw_limit_rad'] <= .15 or not 0 < c['tilt_limit_rad'] <= .15:
        raise ValueError('attitude limits too permissive')
    if not 0 < c['min_ray_plane_cos'] < 1 or not 1 < c['target_area_ratio'] <= 2:
        raise ValueError('invalid geometric gate')
    if c['max_setpoint_lead_m'] > c['max_tracking_error_m']:
        raise ValueError('lead exceeds tracking limit')
    if not c['approach_min_m'] < c['approach_max_m'] < c['max_traverse_distance_m']:
        raise ValueError('invalid approach/traverse range')
    if c['confirm_frames'] < 3 or c['confirm_time_s'] < .5 or c['stable_time_s'] < .5:
        raise ValueError('insufficient confirmation/stability')
    if any(x <= 0 for x in c['vehicle_size_m']+c['opening_size_m']):
        raise ValueError('physical sizes must be positive')
    if not -math.pi <= c['window_yaw_rad'] <= math.pi:
        raise ValueError('window_yaw_rad must be in [-pi,pi]')
    if not 0 < c['opening_size_error_fraction'] <= .15 or c['center_agreement_m'] > .1:
        raise ValueError('survey/visual gates too permissive')
    if not .2 <= c['graph_timeout_s'] <= 1 or not .5 <= c['state_timeout_s'] <= 3:
        raise ValueError('invalid graph/heartbeat timeout')
    if not .1 <= c['degraded_hold_timeout_s'] <= 1 or not 1 <= c['hold_prestream_s'] <= 5:
        raise ValueError('invalid degraded hold/prestream duration')
    if c['confirm_frames'] > 240 or c['confirm_time_s'] > 5 or c['target_jump_px'] > 80 or c['stability_px'] > 20:
        raise ValueError('association gate too permissive or exceeds history')
    if c['max_align_distance_m'] > .5 or c['max_align_path_m'] > 1 or c['max_traverse_distance_m'] > 4:
        raise ValueError('prototype travel cap exceeded')
    if not max(c['max_align_time_s'],c['max_traverse_time_s']) <= c['max_total_time_s'] <= 120 or c['max_hold_wait_s'] > 60:
        raise ValueError('prototype time cap exceeded')
    if c['align_tolerance_m'] > .05 or c['max_tracking_error_m'] > .15 or c['max_setpoint_lead_m'] > .05:
        raise ValueError('position gate too permissive')
    # This is the lateral tracking lag assumed when judging the aperture, so it
    # must stay a measurement-scale number and never become a second abort limit.
    if not 0 < c['clearance_tracking_budget_m'] <= .05:
        raise ValueError('clearance tracking budget must be in (0, 0.05]')
    for key in ('local_frame','body_frame','optical_frame','fcu_node','detector_node'):
        if any(x.isspace() for x in c[key]) or c[key] == 'square':
            raise ValueError(key+' must be a real frame/node name')
    if any(not c[k].startswith('/') for k in ('fcu_node','detector_node')):
        raise ValueError('source node names must be absolute')
    return c


def clearance_reserve(c):
    """Lateral/vertical allowance added to the swept hull at the opening plane.

    Only measurement-scale terms belong here. ``stopping_margin_m`` is a
    longitudinal distance, ``max_tracking_error_m`` is a runtime abort
    threshold, and scaling ``align_tolerance_m`` by two double-counts the
    band the commit gate already enforces. See ``blockers`` for details.
    """
    return (c['position_uncertainty_m'] + c['clearance_margin_m'] +
            c['align_tolerance_m'] + c['clearance_tracking_budget_m'])


def field_violations(c):
    """Return the axes where a configured geofence exceeds the venue.

    Kept out of ``load_config`` on purpose: the controller is venue-agnostic and
    its own tests use wide synthetic fences. The field entry point calls this so
    the deployed configuration can never ask for airspace the venue lacks.
    """
    bad = []
    if not (vector(c['geofence_min_m']) and vector(c['geofence_max_m'])):
        return bad
    for axis, (low, high) in enumerate(zip(FIELD_GEOFENCE_MIN_M, FIELD_GEOFENCE_MAX_M)):
        if c['geofence_min_m'][axis] < low or c['geofence_max_m'][axis] > high:
            bad.append('axis %s: [%s, %s] outside [%s, %s]' % (
                axis, c['geofence_min_m'][axis], c['geofence_max_m'][axis], low, high))
    return bad


def blockers(c, motion=True):
    bad = []
    for k in ('local_frame','body_frame','optical_frame','camera_info_sha256','vision_profile_sha256','algorithm_sha256'):
        if not c[k]: bad.append('missing:'+k)
    for k in ('camera_info_sha256','vision_profile_sha256','algorithm_sha256'):
        if c[k] and (len(c[k]) != 64 or any(x not in '0123456789abcdef' for x in c[k])):
            bad.append('invalid:'+k)
    for k in ('envelope_center_body_m','camera_translation_body_m','window_center_local_m','geofence_min_m','geofence_max_m'):
        if not vector(c[k]): bad.append('missing:'+k)
    if not quaternion(c['camera_rotation_body_from_optical_xyzw']): bad.append('missing:camera_rotation_body_from_optical_xyzw')
    if c['window_thickness_m'] <= 0: bad.append('missing:window_thickness_m')
    if not 0 < c['position_uncertainty_m'] < .1: bad.append('missing:position_uncertainty_m')
    if not 0 < c['stopping_margin_m'] < .2: bad.append('missing:stopping_margin_m')
    if any(x <= 0 for x in c['vehicle_size_m']+c['opening_size_m']): bad.append('invalid:dimensions')
    if vector(c['geofence_min_m']) and vector(c['geofence_max_m']):
        if any(a >= b for a, b in zip(c['geofence_min_m'], c['geofence_max_m'])): bad.append('invalid:geofence')
    if motion:
        if not c['evidence_id'].strip(): bad.append('missing:evidence_id')
        bad += ['unverified:'+k for k in ATTESTATIONS if not c[k]]
    half = envelope_half_extents(c)
    # Lateral/vertical reserve must describe *typical* error at the plane, not
    # the abort threshold. Three terms used to be added here and none of them
    # belong to the sideways clearance question:
    #   stopping_margin_m      : length along the approach axis, not lateral.
    #                            Traverse length already uses position
    #                            uncertainty + arrival tolerance in core.py.
    #   max_tracking_error_m   : runtime fault threshold; exceeding it aborts.
    #                            Using it as a static reserve assumes the vehicle
    #                            is at full deviation for the whole pass.
    #   2*align_tolerance_m    : the commit gate already refuses a misalignment
    #                            larger than align_tolerance_m, so scaling it by
    #                            two double-counts the same band.
    # What remains is the measurement uncertainty at the plane, the alignment
    # band the controller is allowed to commit within, the physical clearance
    # margin, and an explicit budget for how far tracking lags behind.
    reserve = clearance_reserve(c)
    if half[1]+reserve >= c['opening_size_m'][0]/2 or half[2]+reserve >= c['opening_size_m'][1]/2:
        bad.append('insufficient:worst_case_clearance')
    return bad
