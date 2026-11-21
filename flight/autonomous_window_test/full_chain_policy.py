"""Explicit combined scope. Old single-hover/straight CLI scopes cannot grant it."""
import copy
from full_chain import settings
from hover_target import settings as hover_settings
from straight_pass_policy import validate as straight_validate,ACCEPTANCE_DEFERRALS,CONTROL_DEFERRALS
SCOPE='supervised_window_pass_single_target_20261024'

def validate(config,requested=False):
    enabled=config.get('full_chain',{}).get('enabled',False)
    if not enabled:
        if requested:raise ValueError('full_chain profile required')
        return frozenset()
    if not requested:raise ValueError('full_chain requires its dedicated commissioning flag')
    pair='selected_target_ids' in config['full_chain']
    if pair:
        from two_targets import settings as pair_settings,AUTH_SCOPE
        p=pair_settings(config['full_chain'])
    else:p=settings(config['full_chain'])
    journey=p.get('second_window_return')
    if bool(journey)!=config.get('mission',{}).get('second_window_return_enabled',False):raise ValueError('explicit return mission policy required')
    if journey and (not config['mission'].get('pilot_landing_enabled') or config['mission']['max_mission_time_s']!=240.):raise ValueError('return requires manual landing and 240s bounded budget')
    h=hover_settings(config.get('hover_target'))
    fixed=bool(h and h['fixed_position_timed'])
    if fixed and p['fixed_aim_compensation_forward_left_m'] is None:raise ValueError('fixed position requires explicit metre compensation')
    if not h or (not fixed and not h['visual_servo_enabled']) or h['trigger_policy']!='hover_timed':raise ValueError('full_chain requires timed single-target visual servo')
    if config.get('experiment')!='straight_pass' or not config.get('session_window',{}).get('enabled'):
        raise ValueError('full_chain requires existing session-window passage')
    if config.get('session_window',{}).get('terminal_capture') and not fixed and not h['require_stable_target_before_light'] and not (h['visual_target_hold'] and h['timed_visual_aim']):raise ValueError('terminal chain requires stable target exposure policy')
    a=config.get('full_chain_authorization',{})
    if a.get('scope')!=(AUTH_SCOPE if pair else SCOPE) or a.get('operator_requested_test') is not True or a.get('target_count')!=(2 if pair else 1):
        raise ValueError('explicit one-target full-chain declaration required')
    if set(a.get('deferred_acceptance_checks',[]))!=ACCEPTANCE_DEFERRALS or set(a.get('deferred_control_checks',[]))!=CONTROL_DEFERRALS:
        raise ValueError('full_chain cannot defer extra checks')
    if not config.get('site_identity') or not config.get('evidence_id'):raise ValueError('full_chain requires fresh site binding')
    # Reuse every existing passage scope check, with a private derived declaration.
    # The source profile never acquires an independent straight-pass authorization.
    c=copy.deepcopy(config);c.pop('full_chain',None)
    if pair:
        calibration=bool(h and h.get('visual_calibration_mode','off')!='off')
        if not fixed and not calibration and not h['visual_target_hold']:raise ValueError('two-targets requires explicit fixed or visual target exposure')
        if calibration:
            from visual_calibration import CalibrationSession
            CalibrationSession(config,config['site_identity'])
        c['_validated_two_target_scope']=True
        if journey:c['_validated_return_scope']=True
    c['straight_pass_authorization']={'scope':'supervised_session_window_pass','operator_authorized_test':True,
        'deferred_acceptance_checks':sorted(ACCEPTANCE_DEFERRALS),'deferred_control_checks':sorted(CONTROL_DEFERRALS)}
    return straight_validate(c,True)
