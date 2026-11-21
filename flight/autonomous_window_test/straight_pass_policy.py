"""Separate supervised straight-passage scope; never grants verified flags."""
import math
from mission import settings
from window_pass.config import load_config

SCOPE='supervised_fixed_height_straight_pass'
ACCEPTANCE_DEFERRALS=frozenset(('body_origin_verified','fcu_failsafe_verified','simulation_verified','rgbd_geometry_verified'))
CONTROL_DEFERRALS=frozenset(('camera_verified','pose_verified','direction_verified','hold_verified','failsafe_verified','latency_verified','corridor_verified','single_controller_verified'))

def validate(config,requested=False):
    if requested and config.get('full_chain',{}).get('enabled'):
        raise ValueError('old trial scope cannot authorize full chain')
    from prepass_adjustment import settings as adjustment_settings
    adjustment=adjustment_settings(config.get('prepass_adjustment'))
    if adjustment['enabled'] and config.get('experiment')!='straight_pass':raise ValueError('adjustment is straight-pass-only')
    from session_window import settings as window_settings
    window=window_settings(config.get('session_window'))
    if window['enabled'] and (config.get('experiment')!='straight_pass' or adjustment['enabled']):
        raise ValueError('session-window incompatible with legacy route policy')
    if config.get('mission',{}).get('simplified_takeoff_enabled',False)!=window['enabled']:
        raise ValueError('simplified takeoff requires explicit session-window profile')
    if window['enabled'] and (window['alignment_max_distance_m']>.20 or window['alignment_clear_column_radius_m']>.20):
        area=config.get('alignment_area_confirmation',{})
        radius=area.get('center_path_radius_m')
        if area.get('operator_confirmed_clear') is not True or type(radius) not in (int,float) or radius<window['alignment_clear_column_radius_m']:
            raise ValueError('expanded alignment clear area confirmation required')
    if window.get('field_bounded_alignment'):
        area=config.get('alignment_area_confirmation',{})
        if area.get('operator_confirmed_clear') is not True or area.get('field_fence_alignment_authorized') is not True:raise ValueError('field-fence alignment authorization required')
    if not requested:return frozenset()
    if config.get('experiment')!='straight_pass' or config.get('acquisition')!='airborne_rgbd':raise ValueError('straight commissioning cannot authorize other modes')
    a=config.get('straight_pass_authorization',{})
    if a.get('scope')!=('supervised_session_window_pass' if window['enabled'] else SCOPE) or a.get('operator_authorized_test') is not True:raise ValueError('explicit straight passage authorization required')
    if set(a.get('deferred_acceptance_checks',[]))!=ACCEPTANCE_DEFERRALS or set(a.get('deferred_control_checks',[]))!=CONTROL_DEFERRALS:raise ValueError('straight scope cannot defer extra checks')
    c=load_config(config['control']);o=settings(config['mission'])
    if not o['pilot_landing_enabled'] or not o['takeoff_position_goal_enabled'] or not o['vendor_takeoff_arrival_enabled'] or not o['takeoff_attitude_checks_enabled'] or o['takeoff_vertical_velocity_enabled']:raise ValueError('straight flight policy invalid')
    if not o['minimum_bottom_height_m']>=(.70 if window['enabled'] else .80) or o['target_bottom_height_m']>.9 or o['takeoff_timeout_s']>25 or o['search_timeout_s']>10 or o['max_mission_time_s']>(240 if config.get('_validated_return_scope') is True else 120 if config.get('_validated_two_target_scope') is True else 75):raise ValueError('straight height/time scope exceeded')
    for k,limit in [('traverse_speed_mps',.30 if window.get('faster_pass',False) else .12),('max_traverse_distance_m',3.),('max_traverse_time_s',55. if window.get('fixed_pass_distance_m')==3. else 25.),('max_tracking_error_m',.12),('max_setpoint_lead_m',.04),('pose_timeout_s',.15),('image_timeout_s',.25),('yaw_limit_rad',math.radians(5)+1e-7),('tilt_limit_rad',math.radians(8 if window.get('smooth_pass') else 5)+1e-7)]:
        if c[k]>limit+1e-9:raise ValueError('straight scoped limit exceeded:'+k)
    if window.get('smooth_pass'):
        if c['max_pose_speed_mps']!=2.2 or c['max_pose_step_m']!=.35:raise ValueError('smooth pass requires scoped 10Hz pose limits')
        expected=config.get('expected_fcu_parameters',{})
        angle=window.get('pass_tilt_limit_rad')
        if angle is not None and (not math.isclose(c['tilt_limit_rad'],math.radians(8),abs_tol=1e-7) or angle>math.radians(15)):raise ValueError('pass tilt expansion requires unchanged 8deg baseline and <=15deg passage')
        for key,pair in [('MPC_XY_VEL_MAX',[0,1.5]),('MPC_ACC_HOR_MAX',[0,1.]),('MPC_ACC_HOR',[0,1.]),('MPC_JERK_AUTO',[0,4.]),('MPC_TILTMAX_AIR',[0,15.])]:
            if expected.get(key)!=pair:raise ValueError('smooth pass requires FCU readback:'+key)
    if c['approach_min_m']<.8 or c['approach_max_m']>2. or c['post_plane_clearance_m']<.4:raise ValueError('straight approach/exit limits invalid')
    install=config.get('installation_confirmation',{})
    if install.get('operator_confirmed') is not True or install.get('camera_translation_body_m')!=c['camera_translation_body_m'] or install.get('camera_rotation_body_from_optical_xyzw')!=c['camera_rotation_body_from_optical_xyzw']:raise ValueError('confirmed installation mismatch')
    if config.get('exit_corridor_confirmation',{}).get('operator_confirmed_clear') is not True:raise ValueError('operator must confirm exit corridor and stopping region clear')
    if window['enabled'] and (window['alignment_max_distance_m']>.20 or window['alignment_clear_column_radius_m']>.20):
        if c['max_align_distance_m']>window['alignment_max_distance_m'] or c['max_align_path_m']>.75 or c['max_align_time_s']>15.:
            raise ValueError('expanded alignment control scope exceeded')
    policy=config.get('route_policy',{})
    if window['enabled']:
        if policy.get('mode')!='one_acquisition_center_then_session_frozen_line' or policy.get('route_frozen_once') is not True or policy.get('no_alignment') is not False:
            raise ValueError('session-window route declaration missing')
        if policy.get('post_commit_visual_dependency') is not False or policy.get('localization_binding')!='boot_run_source_epoch':
            raise ValueError('session-window dependency declaration missing')
    elif adjustment['enabled']:
        if policy.get('mode')!='bounded_adjustment_then_fixed_height_heading' or policy.get('no_legacy_alignment') is not True or policy.get('no_alignment') is not False or policy.get('route_frozen_once') is not True:raise ValueError('bounded adjustment route declaration missing')
    elif policy.get('mode')!='fixed_current_height_and_heading' or policy.get('no_alignment') is not True or policy.get('route_frozen_once') is not True:raise ValueError('straight route declaration missing')
    if policy.get('vote_rule')!=('2_feasible_distinct_frames_of_latest_5_within_1p2_seconds_current_frame_must_be_feasible' if window.get('fast_confirmation',False) else '3_feasible_distinct_frames_of_latest_5_within_1_second_current_frame_must_be_feasible'):raise ValueError('straight vote declaration invalid')
    return ACCEPTANCE_DEFERRALS

