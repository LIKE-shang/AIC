#!/usr/bin/env python3
"""Fresh 8s LIO/FCU ground binding. Writes a new trial; never executes flight."""
import argparse,copy,hashlib,json,math,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from bind_hover_target_trial import sample
from full_chain import settings as chain_settings
from full_chain_policy import validate
from mission import settings
from field_site import bind_bounds
from session_window import decorate,settings as window_settings
from window_pass.config import load_config
from window_pass.geometry import vector,quaternion,tilt

HISTORICAL_BOOT='523c469a-4806-441e-af45-fd0d53fb95aa'

def bind(template,snapshot,measurements,station,evidence,station_reference='route_endpoint'):
    c=copy.deepcopy(template);o=settings(c['mission']);ctrl=load_config(c['control'])
    if any(ctrl[k] for k in ('enable_motion','enable_traverse','pass_stage_approved')):raise ValueError('motion-disabled template required')
    p=snapshot['position_local_m'];q=snapshot['orientation_xyzw'];raw=snapshot['raw_lio'];identity=snapshot['identity']
    if not vector(p) or not quaternion(q) or tilt(q)>ctrl['tilt_limit_rad']:raise ValueError('ground pose invalid/tilted')
    if not identity.get('boot_id') or identity['boot_id']==HISTORICAL_BOOT or not identity.get('run_id') or '/window_live/status' not in identity:
        raise ValueError('new boot/run/current RGBD source binding required')
    if snapshot['local_frame']!=ctrl['local_frame'] or snapshot['body_frame']!=ctrl['body_frame']:raise ValueError('ground frames mismatched')
    for count,spans in [(snapshot['pose_samples'],snapshot['xyz_span_m']),(raw['samples'],raw['xyz_span_m'])]:
        if count<50 or not vector(spans) or min(spans)<0 or max(spans)>o['ground_z_span_limit_m']:raise ValueError('dual ground streams not stationary')
    if not vector(raw['position_local_m']):raise ValueError('invalid raw LIO position')
    c['full_chain'].update(station_reference=station_reference,station_offset_forward_left_m=list(station),station_relation_confirmed=True,
                           station_area_confirmed=True,station_evidence=evidence)
    chain_settings(c['full_chain'])
    geometry=bind_bounds(measurements,p,q,o,decorate(dict(ctrl),window_settings(c['session_window'])))
    ctrl.update({k:geometry[k] for k in ('geofence_min_m','geofence_max_m')})
    ctrl.update(enable_motion=True,enable_traverse=True,pass_stage_approved=True,evidence_id='full_chain_'+identity['boot_id'])
    c.update(control=ctrl,site_identity=identity,evidence_id=ctrl['evidence_id'])
    for k in ('ground_height_verified','takeoff_column_verified','takeoff_landing_site_verified','postpass_landing_site_verified'):
        c['acceptance'][k]=True
    c['exit_corridor_confirmation'].update(operator_confirmed_clear=True,status='current_ground_start_heading_full_pass_exit_station_and_aim_area_confirmed')
    c['alignment_area_confirmation'].update(operator_confirmed_clear=True,center_path_radius_m=.2)
    c['site_evidence']=dict(snapshot=snapshot,geometry=geometry,operator_confirmed_same_start=True,
                            physical_area_basis='Current on-site confirmation of surveyed start/heading, 20cm alignment, full 3m route, exit/stop, station transfer, 15cm aim column and pilot landing area.',
                            flight_acceptance=False,precision_acceptance=False,flight_authorized=False)
    c['preparation_status']='NEW_SESSION_BOUND_PENDING_READONLY_PREFLIGHT'
    validate(c,True);return c

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--station-reference',choices=['route_endpoint','takeoff_body'],default='route_endpoint')
    p.add_argument('--station-forward-m',type=float,required=True,help='Target station from frozen 3m endpoint along pass heading')
    p.add_argument('--station-left-m',type=float,required=True,help='Left positive; right negative, relative to pass heading')
    p.add_argument('--station-evidence',required=True,help='Source of operator-approved estimated or measured station relationship')
    p.add_argument('--current-flight-area-confirmed',action='store_true',help='Current surveyed start/heading, alignment, 3m pass, exit/stop, station path/column and pilot landing area are clear')
    p.add_argument('--boundaries',type=Path,default=ROOT/'FIELD_BOUNDARY_CORRECTED_20261018.json')
    a=p.parse_args()
    if not a.current_flight_area_confirmed:p.error('current start/heading and complete flight area must be confirmed on site')
    c=json.loads((ROOT/'config/full_chain_single_target.json').read_text())
    # Validate missing/invalid metre relationship before making ROS subscriptions.
    c['full_chain'].update(station_reference=a.station_reference,station_offset_forward_left_m=[a.station_forward_m,a.station_left_m],station_relation_confirmed=True,station_area_confirmed=True,station_evidence=a.station_evidence)
    chain_settings(c['full_chain'])
    snapshot=sample(local_origin_required=False,include_perception=True)
    c=bind(c,snapshot,json.loads(a.boundaries.read_text()),[a.station_forward_m,a.station_left_m],a.station_evidence,station_reference=a.station_reference)
    c['site_evidence']['deployment_manifest_sha256']=hashlib.sha256((ROOT.parent/'MANIFEST_SHA256.json').read_bytes()).hexdigest()
    c['site_evidence']['boundary_sha256']=hashlib.sha256(a.boundaries.read_bytes()).hexdigest()
    trial=Path(tempfile.mkdtemp(prefix='full_chain_trial_',dir='/home/abot'))
    (trial/'config.json').write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n')
    bundle=ROOT.parent
    launch='#!/usr/bin/env bash\nset -eo pipefail\npython3 '+str(bundle/'verify_manifest.py')+'\nsource '+str(bundle/'env.bash')+'\npython3 '+str(ROOT/'scripts/auto_window.py')+' --config '+str(trial/'config.json')+' --require-experiment straight_pass --commissioning-full-chain "$@" 2>&1 | tee -a '+str(trial/'mission.log')+'\n'
    (trial/'run.bash').write_text(launch)
    record='#!/usr/bin/env bash\nset -eo pipefail\nsource '+str(bundle/'env.bash')+'\npython3 '+str(ROOT/'scripts/run_recorded_hover_trial.py')+' --trial-dir '+str(trial)+' "$@"\n'
    (trial/'run_recorded.bash').write_text(record)
    result=subprocess.run(['bash',str(trial/'run.bash')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
    (trial/'preflight.txt').write_text(result.stdout);print(result.stdout,flush=True)
    print('TRIAL_DIRECTORY='+str(trial));print('PREFLIGHT_EXIT='+str(result.returncode))
    if result.returncode==0:
        print('USER_FLIGHT_COMMAND=bash '+str(trial/'run_recorded.bash')+' --execute')
        print('Command printed only. No takeoff executed. Rebind after boot/run/source/start changes.')
    return result.returncode

if __name__=='__main__':sys.exit(main())
