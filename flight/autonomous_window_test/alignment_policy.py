"""Narrow supervised alignment policy, never a traversal acceptance.

All original verification flags remain unchanged. A separate explicit operator
acceptance and CLI switch are required, in addition to every live flight gate.
"""
import math
from mission import settings
from window_pass.config import load_config

ACCEPTANCE_DEFERRALS=frozenset(('body_origin_verified','fcu_failsafe_verified','simulation_verified','rgbd_geometry_verified'))
CONTROL_DEFERRALS=frozenset(('camera_verified','pose_verified','direction_verified','hold_verified','failsafe_verified','latency_verified','corridor_verified','single_controller_verified'))
SCOPE='supervised_alignment_only_no_traverse'

def validate(config,requested=False):
    if not requested:return frozenset()
    if config.get('experiment')!='align_only' or config.get('acquisition')!='airborne_rgbd':
        raise ValueError('alignment commissioning cannot authorize another experiment')
    auth=config.get('alignment_commissioning_authorization',{})
    if auth.get('scope')!=SCOPE or auth.get('operator_accepts_unverified_alignment_tests') is not True:
        raise ValueError('explicit operator acceptance of alignment limitations required')
    if set(auth.get('deferred_acceptance_checks',[]))!=ACCEPTANCE_DEFERRALS or set(auth.get('deferred_control_checks',[]))!=CONTROL_DEFERRALS:
        raise ValueError('alignment commissioning cannot defer extra checks')
    c=load_config(config.get('control',{}));o=settings(config.get('mission',{}))
    if c['enable_traverse'] or c['pass_stage_approved'] or not o['pilot_landing_enabled']:
        raise ValueError('alignment commissioning forbids traversal and requires pilot handover')
    if not o['takeoff_attitude_checks_enabled'] or not o['takeoff_position_goal_enabled'] or o['takeoff_vertical_velocity_enabled']:
        raise ValueError('alignment commissioning requires bounded position takeoff and attitude checks')
    if (o['target_bottom_height_m']>.90 or o['minimum_bottom_height_m']<.80 or
        o['max_mission_time_s']>75 or o['takeoff_timeout_s']>25 or o['search_timeout_s']>10):
        raise ValueError('alignment height/time scope exceeded')
    for key,maximum in [('align_speed_mps',.08),('max_align_distance_m',.20),('max_align_path_m',.30),
                        ('max_align_time_s',20.),('max_total_time_s',60.),('max_tracking_error_m',.12),
                        ('max_setpoint_lead_m',.04),('tilt_limit_rad',math.radians(5)+1e-7),
                        ('yaw_limit_rad',math.radians(5)+1e-7),('pose_timeout_s',.15),('image_timeout_s',.25)]:
        if c[key]>maximum+1e-9:raise ValueError('alignment scoped limit exceeded: '+key)
    if c['approach_min_m']<.8 or c['approach_max_m']>2.:
        raise ValueError('alignment must remain at least 0.8m in front of window')
    if o['takeoff_transient_tolerance_enabled'] and auth.get('operator_accepts_takeoff_transient_tolerance') is not True:
        raise ValueError('explicit operator acceptance of transient takeoff tolerance required')
    installation=config.get('installation_confirmation',{})
    if (installation.get('operator_confirmed') is not True or
        installation.get('camera_translation_body_m')!=c['camera_translation_body_m'] or
        installation.get('camera_rotation_body_from_optical_xyzw')!=c['camera_rotation_body_from_optical_xyzw']):
        raise ValueError('operator installation confirmation missing or mismatched')
    return ACCEPTANCE_DEFERRALS
