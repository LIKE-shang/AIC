"""Scope for the explicitly requested single-target supervised field test.
Unperformed historical acceptance checks stay false; only this scope records
that the operator requested a test before those acceptance tests were done.
"""
import math
from hover_target import settings as target_settings
from mission import settings as mission_settings

SCOPE='supervised_single_hover_target_20261023'
DEFERRALS=frozenset(('body_origin_verified','fcu_failsafe_verified','simulation_verified'))


def validate(config,requested=False):
    if requested and config.get('full_chain',{}).get('enabled'):
        raise ValueError('old trial scope cannot authorize full chain')
    policy=target_settings(config.get('hover_target'))
    declaration=config.get('hover_trial_authorization')
    if not requested:
        if declaration is not None and policy is None:raise ValueError('hover authorization without hover policy')
        return frozenset()
    if policy is None or config.get('experiment')!='takeoff_land' or config.get('acquisition')!='usb_hover_target':
        raise ValueError('single-hover commissioning cannot authorize other modes')
    if not isinstance(declaration,dict) or declaration.get('scope')!=SCOPE or declaration.get('operator_requested_test') is not True:
        raise ValueError('explicit single-hover trial declaration required')
    if set(declaration.get('deferred_checks',[]))!=DEFERRALS:
        raise ValueError('hover trial cannot defer extra acceptance checks')
    if declaration.get('current_target1_heading_and_area_confirmed') is not True:
        raise ValueError('current target1 area confirmation required')
    c=config.get('control',{});o=mission_settings(config.get('mission',{}))
    if c.get('enable_traverse') or c.get('pass_stage_approved'):
        raise ValueError('single-hover trial forbids passage')
    if config.get('session_window',{}).get('enabled') or config.get('prepass_adjustment',{}).get('enabled'):
        raise ValueError('single-hover trial cannot enable window alignment')
    if o['target_bottom_height_m']>.9 or o['minimum_bottom_height_m']<.8 or o['climb_speed_mps']>.1:
        raise ValueError('single-hover height/climb scope exceeded')
    if not o['pilot_landing_enabled'] or not o['takeoff_position_goal_enabled'] or o['simplified_takeoff_enabled']:
        raise ValueError('single-hover requires position takeoff and observed pilot handover')
    if policy['max_correction_m']>.15 or policy['max_speed_mps']>.04:
        raise ValueError('single-hover motion scope exceeded')
    if not config.get('site_identity') or not config.get('evidence_id'):
        raise ValueError('single-hover requires current localization site binding')
    return DEFERRALS
