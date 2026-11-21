#!/usr/bin/env python3
"""Create a separate single-target profile from an existing takeoff profile.
Does not connect to ROS or copy passage-specific permissions into hover tests.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from hover_target import settings
from mission import settings as mission_settings


def prepare(source,offset_x=53.25,offset_y=15.484375):
    c=json.loads(json.dumps(source))
    if c.get('experiment')!='takeoff_land':
        raise ValueError('Use a takeoff-only profile bound at the current target position; do not reuse a passage site binding')
    if not c.get('site_identity') or not c.get('evidence_id'):
        raise ValueError('Existing current-site takeoff binding required')
    c['acquisition']='usb_hover_target'
    c['mission'].update(target_bottom_height_m=.9,pilot_landing_enabled=True)
    mission_settings(c['mission'])
    c['hover_target']=settings(dict(enabled=True,offset_x_px=offset_x,offset_y_px=offset_y,trigger_policy='hover_timed',hover_light_delay_s=2.,visual_servo_enabled=True))
    c['control']['enable_traverse']=False
    c['control']['pass_stage_approved']=False
    # Preserve the existing enable_motion and site/flight evidence; never manufacture it.
    return c



def from_traverse_template(source, source_hash, offset_x=53.25, offset_y=15.484375):
    """Reuse stable flight settings, never old passage geometry/permissions."""
    if source.get('experiment')!='straight_pass':
        raise ValueError('Expected the existing straight-pass configuration')
    base_path=ROOT/'config/takeoff_land.json'
    c=json.loads(base_path.read_text(encoding='utf-8-sig'))
    # Takeoff policy switches specific to window acquisition are not transferable.
    shared=('bottom_to_fcu_m','ground_fcu_height_m','target_bottom_height_m',
            'climb_speed_mps','takeoff_timeout_s','stable_time_s','mode_timeout_s',
            'landing_timeout_s','minimum_battery_voltage','ground_z_span_limit_m',
            'companion_battery_monitor_enabled','pilot_landing_enabled',
            'takeoff_position_goal_enabled','takeoff_vertical_velocity_enabled',
            'takeoff_attitude_checks_enabled','vendor_takeoff_arrival_enabled')
    for key in shared:
        if key in source.get('mission',{}):c['mission'][key]=source['mission'][key]
    c['mission'].update(target_bottom_height_m=.9,pilot_landing_enabled=True)
    c['mission']['simplified_takeoff_enabled']=False
    mission_settings(c['mission'])
    c['expected_fcu_parameters']=json.loads(json.dumps(source['expected_fcu_parameters']))
    # Keep only model/frame/tracking settings; measured site is bound separately.
    shared_control=('local_frame','body_frame','fcu_node','vehicle_size_m',
                    'envelope_center_body_m','position_uncertainty_m','stopping_margin_m',
                    'max_tracking_error_m','max_setpoint_lead_m','pose_timeout_s',
                    'state_timeout_s','max_loop_gap_s','control_hz','tilt_limit_rad','yaw_limit_rad')
    for key in shared_control:
        if key in source.get('control',{}):c['control'][key]=source['control'][key]
    c['control'].update(enable_motion=False,enable_traverse=False,pass_stage_approved=False,
                        geofence_min_m=[],geofence_max_m=[],evidence_id='')
    c.update(experiment='takeoff_land',acquisition='usb_hover_target',site_identity=None,evidence_id='')
    # Historical flight acceptance is not relabelled as a new target-site survey.
    for key in c.get('acceptance',{}):
        if key in ('takeoff_column_verified','takeoff_landing_site_verified'):
            c['acceptance'][key]=False
    c['hover_target']=settings(dict(enabled=True,offset_x_px=offset_x,offset_y_px=offset_y,trigger_policy='hover_timed',hover_light_delay_s=2.,visual_servo_enabled=True))
    c['source_reuse']={
        'source_sha256':source_hash,
        'entry':'scripts/auto_window.py',
        'takeoff':'mission.py position goal plus observed FCU state',
        'handover':'mission.py POSCTL readback then pilot landing',
        'copied_mission_keys':[k for k in shared if k in source.get('mission',{})],
        'old_site_identity_reused':False,'old_fence_reused':False,
        'passage_permissions_reused':False,
        'window_only_takeoff_exception_reused':False,
        'minimum_bottom_height_m':c['mission']['minimum_bottom_height_m'],
        'status':'unbound_template_not_ready_for_flight'}
    return c


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=ROOT/'config/session_window.json')
    p.add_argument('--from-traverse-template',action='store_true',
                   help='Extract existing passage takeoff settings into a NEW unbound hover template')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--offset-x-px',type=float,default=53.25)
    p.add_argument('--offset-y-px',type=float,default=15.484375)
    a=p.parse_args()
    raw=a.base.read_bytes()
    source=json.loads(raw.decode('utf-8-sig'))
    if a.from_traverse_template:
        c=from_traverse_template(source,hashlib.sha256(raw).hexdigest(),a.offset_x_px,a.offset_y_px)
    else:
        c=prepare(source,a.offset_x_px,a.offset_y_px)
    with a.output.open('x',encoding='utf-8') as f:json.dump(c,f,ensure_ascii=False,indent=2)
    print('Separate single-target profile written:',a.output)
    print('Unbound template: current target site binding required; motion disabled.' if a.from_traverse_template else 'Existing flight evidence and enable_motion preserved.')
    print('No hardware commands sent.')

if __name__=='__main__':main()
