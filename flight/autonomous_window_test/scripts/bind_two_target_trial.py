#!/usr/bin/env python3
"""Explicit two-ID selection, fresh ground binding, read-only preflight. Never flies."""
import argparse,copy,hashlib,json,os,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from two_targets import selection,TARGETS,settings as pair_settings
from full_chain_policy import validate
from bind_full_chain_trial import bind as single_bind
from bind_hover_target_trial import sample
from session_window import decorate,settings as window_settings
from fixed_aim import shift
from attitude_envelope import enclosed
from window_pass.geometry import yaw

def bind(template,snapshot,boundaries,ids):
    ids=selection(ids);template=copy.deepcopy(template)
    template['full_chain']['selected_target_ids']=ids
    pair_settings(template['full_chain'])
    # Existing single-session ground and evidence checks, without broadening its authorization.
    base=copy.deepcopy(template);base['full_chain'].pop('selected_target_ids');base['full_chain'].pop('target_positions_forward_left_m')
    journey=base['full_chain'].pop('second_window_return',None)
    base['mission']['second_window_return_enabled']=False
    base['mission']['max_mission_time_s']=75.
    base['session_window']['field_bounded_alignment']=False
    base['full_chain_authorization'].update(scope='supervised_window_pass_single_target_20261024',target_count=1)
    bound=single_bind(base,snapshot,boundaries,TARGETS[ids[0]],'Operator selected two targets; strict drawing layout previously approved; near-center target1 correction retained',station_reference='takeoff_body')
    bound['full_chain'].update(selected_target_ids=ids,target_positions_forward_left_m=template['full_chain']['target_positions_forward_left_m'])
    bound['full_chain_authorization']=copy.deepcopy(template['full_chain_authorization']);bound['mission']['max_mission_time_s']=120.
    bound['two_target_evidence']=copy.deepcopy(template['two_target_evidence'])
    bound['session_window']['field_bounded_alignment']=template['session_window'].get('field_bounded_alignment',False)
    if bound['session_window']['field_bounded_alignment']:bound['alignment_area_confirmation'].update(field_fence_alignment_authorized=True,basis='2026-10-25 operator requests continue aperture alignment within confirmed field; no fixed20cm takeoff circle')
    c=decorate(dict(bound['control']),window_settings(bound['session_window']));heading=yaw(snapshot['orientation_xyzw'])
    origin=snapshot['position_local_m'];z=origin[2]-bound['mission']['ground_fcu_height_m']+bound['mission']['bottom_to_fcu_m']+bound['mission']['target_bottom_height_m']
    stations=[]
    for ident in ids:
        center=shift((origin[0],origin[1],z),heading,TARGETS[ident]);station=shift(center,heading,bound['full_chain']['fixed_aim_compensation_forward_left_m'])
        if bound['full_chain']['empirical_aim_trim_forward_left_m'] is not None:station=shift(station,heading,bound['full_chain']['empirical_aim_trim_forward_left_m'])
        for dx in (-.15,.15):
            for dy in (-.15,.15):
                if not enclosed(c,(station[0]+dx,station[1]+dy,station[2]),heading):raise ValueError('selected target outside current ground fence:'+str(ident))
        stations.append(dict(target_id=ident,target_center_estimate_local_m=center,compensated_station_estimate_local_m=station))
    bound['two_target_evidence'].update(selected_target_ids=ids,execution_order=ids,ground_heading_estimated_stations=stations,
        runtime_full_body_window_clearance_required=True,second_target_accuracy_verified=False)
    if journey:
        from second_window_return import settings as return_settings
        bound['full_chain']['second_window_return']=return_settings(journey)
        bound['mission'].update(second_window_return_enabled=True,max_mission_time_s=240.)
        prep=shift((origin[0],origin[1],z),heading,journey['preparation_forward_left_m'])
        if not all(enclosed(c,p,heading) for p in (prep,(origin[0],origin[1],z))):raise ValueError('return preparation or home outside current ground fence')
        bound['second_window_return_evidence']=dict(drawing='original_dimensioned_field_drawing_20261017',independent_lower_window=True,preparation_local_m=prep,manual_landing=False,automatic_landing=True,physical_flight_verified=False)
    validate(bound,True);return bound

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',nargs='+',type=int,required=True)
    p.add_argument('--check-selection',action='store_true',help='Offline order preview only; no ROS or trial')
    p.add_argument('--current-flight-area-confirmed',action='store_true')
    p.add_argument('--boundaries',type=Path,default=ROOT.parent/'FIELD_BOUNDARY_CURRENT_20261024.json')
    a=p.parse_args();ids=selection(a.targets)
    if a.check_selection:print(json.dumps({'selected_target_ids':ids,'execution_order':ids,'flight_executed':False}));return 0
    if not a.current_flight_area_confirmed:p.error('current full selected flight area must be clear')
    template=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text());snapshot=sample(local_origin_required=False,include_perception=True)
    c=bind(template,snapshot,json.loads(a.boundaries.read_text()),ids)
    mode=os.environ.get('ORIGINAL_CAR_VISUAL_CALIBRATION','off')
    if mode!='off':
        from build_visual_calibration_profile import build
        seed_path=os.environ.get('ORIGINAL_CAR_VISUAL_CALIBRATION_RECORDS')
        seed=json.loads(Path(seed_path).read_text()) if seed_path else None
        c=build(c,mode,os.environ.get('ORIGINAL_CAR_VISUAL_SCENE',''),seed)
        validate(c,True)
    bundle=ROOT.parent;c['site_evidence']['deployment_manifest_sha256']=hashlib.sha256((bundle/'MANIFEST_SHA256.json').read_bytes()).hexdigest()
    c['site_evidence']['boundary_sha256']=hashlib.sha256(a.boundaries.read_bytes()).hexdigest()
    trial=Path(tempfile.mkdtemp(prefix='full_chain_trial_',dir='/home/abot'));(trial/'config.json').write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n')
    launch='#!/usr/bin/env bash\nset -eo pipefail\npython3 '+str(bundle/'verify_manifest.py')+'\nsource '+str(bundle/'env.bash')+'\npython3 '+str(ROOT/'scripts/auto_window.py')+' --config '+str(trial/'config.json')+' --require-experiment straight_pass --commissioning-full-chain "$@" 2>&1 | tee -a '+str(trial/'mission.log')+'\n'
    (trial/'run.bash').write_text(launch)
    (trial/'run_recorded.bash').write_text('#!/usr/bin/env bash\nset -eo pipefail\nsource '+str(bundle/'env.bash')+'\npython3 '+str(ROOT/'scripts/run_recorded_hover_trial.py')+' --trial-dir '+str(trial)+' "$@"\n')
    result=subprocess.run(['bash',str(trial/'run.bash')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
    (trial/'preflight.txt').write_text(result.stdout);print(result.stdout);print('SELECTED_TARGET_ORDER='+str(ids));print('TRIAL_DIRECTORY='+str(trial));print('PREFLIGHT_EXIT='+str(result.returncode))
    if result.returncode==0:print('USER_FLIGHT_COMMAND=bash '+str(trial/'run_recorded.bash')+' --execute')
    return result.returncode
if __name__=='__main__':sys.exit(main())
