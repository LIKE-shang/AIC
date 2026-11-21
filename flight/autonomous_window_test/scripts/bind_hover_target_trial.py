#!/usr/bin/env python3
"""Prepare current target-1 trial by fresh ground sampling. Never executes flight."""
import argparse,hashlib,json,math,statistics,sys,tempfile,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from hover_trial_policy import SCOPE,DEFERRALS,validate
from mission import settings as mission_settings
from window_pass.config import load_config
from window_pass.geometry import norm,vector,quaternion,tilt,rotate,add
from envelope_fence import envelope_inside
from hover_localization import require_local_origin


def bind(config,snapshot):
    c=json.loads(json.dumps(config));o=mission_settings(c['mission']);ctrl=load_config(c['control'])
    if c['experiment']!='takeoff_land' or c['acquisition']!='usb_hover_target':raise ValueError('single hover template required')
    p=snapshot['position_local_m'];q=snapshot['orientation_xyzw']
    if not vector(p) or not quaternion(q) or tilt(q)>ctrl['tilt_limit_rad']:raise ValueError('invalid or tilted current pose')
    if snapshot['local_frame']!=ctrl['local_frame'] or snapshot['body_frame']!=ctrl['body_frame']:raise ValueError('pose frame mismatch')
    if snapshot['pose_samples']<50 or max(snapshot['xyz_span_m'])>o['ground_z_span_limit_m']:raise ValueError('ground sample not stationary')
    require_local_origin(p)
    raw=snapshot.get('raw_lio')
    if not isinstance(raw,dict) or raw.get('samples',0)<50:
        raise ValueError('raw_lio_ground_evidence_required')
    require_local_origin(raw.get('position_local_m'))
    span=raw.get('xyz_span_m')
    if not vector(span) or min(span)<0 or max(span)>o['ground_z_span_limit_m']:
        raise ValueError('raw_lio_ground_not_stationary')
    ident=snapshot['identity']
    if not ident.get('boot_id') or not ident.get('run_id'):raise ValueError('missing session identity')
    radius=c['hover_target']['max_correction_m']
    # Computational fence encloses the operator-confirmed local swept column.
    # World axes contain the body box at any allowed yaw/tilt; actual reference
    # motion remains separately limited to the 15cm circle in TargetHold.
    reserve=ctrl['position_uncertainty_m']+ctrl['clearance_margin_m']
    half=[v/2 for v in ctrl['vehicle_size_m']]
    horizontal=math.hypot(half[0],half[1])+abs(ctrl['envelope_center_body_m'][2])*math.sin(ctrl['tilt_limit_rad'])+half[2]*math.sin(ctrl['tilt_limit_rad'])
    zsupport=half[2]+math.hypot(half[0],half[1])*math.sin(ctrl['tilt_limit_rad'])
    zcenter=ctrl['envelope_center_body_m'][2]
    target_z=p[2]-o['ground_fcu_height_m']+o['bottom_to_fcu_m']+o['target_bottom_height_m']
    lo=[p[0]-radius-horizontal-reserve,p[1]-radius-horizontal-reserve,
        p[2]+zcenter-zsupport-reserve-o['ground_z_span_limit_m']-.01]
    hi=[p[0]+radius+horizontal+reserve,p[1]+radius+horizontal+reserve,
        target_z+zcenter+zsupport+reserve+.01]
    ctrl.update(enable_motion=True,enable_traverse=False,pass_stage_approved=False,
                geofence_min_m=lo,geofence_max_m=hi,evidence_id='target1_current_'+ident['boot_id'])
    for reference in (p,[p[0],p[1],target_z]):
        if not envelope_inside(ctrl,reference,q):raise ValueError('current takeoff column outside fence')
    c.update(control=ctrl,site_identity=ident,evidence_id=ctrl['evidence_id'])
    c['acceptance'].update(ground_height_verified=True,takeoff_column_verified=True,takeoff_landing_site_verified=True)
    c['site_evidence']={'snapshot':snapshot,'physical_basis':'Operator confirmed target1 center, forward toward right edge, 15cm correction swept area and takeoff overhead clear on 2026-10-23.',
        'fence_source':'current local pose plus confirmed local swept column, not old passage coordinates',
        'reference_radius_m':radius,'target_fcu_z_local_m':target_z,'flight_authorized':False}
    c['hover_trial_authorization']={'scope':SCOPE,'operator_requested_test':True,
        'current_target1_heading_and_area_confirmed':True,'deferred_checks':sorted(DEFERRALS),
        'basis':'Explicit user request for single-target field test and current-site confirmation; historical unperformed tests stay false.'}
    validate(c,True)
    return c


TELEMETRY_TOPICS=('/mavros/local_position/odom','/mavros/state',
                  '/mavros/extended_state','/mavros/rc/in','/window_pass_lio/Odometry')


def sample_telemetry_details(streams,ros,wall):
    details={}
    for topic in TELEMETRY_TOPICS:
        rows=streams.get(topic,[])
        row={'count':len(rows)}
        if rows:
            received,msg=rows[-1]
            row.update(receipt_age_s=wall-received,
                       source_age_s=ros-msg.header.stamp.to_sec(),
                       source_callers=sorted({(m._connection_header or {}).get('callerid','') for _,m in rows}))
        details[topic]=row
    return json.dumps(details,sort_keys=True)


def require_sample_telemetry(streams,before,ros,wall):
    # Preserve the original five streams, deadlines, age limits and source rules.
    if len(streams)!=5:
        missing=[topic for topic in TELEMETRY_TOPICS if not streams.get(topic)]
        raise ValueError('missing telemetry: '+','.join(missing)+'; '+sample_telemetry_details(streams,ros,wall))
    for topic,rows in streams.items():
        received,msg=rows[-1]
        if not 0<=wall-received<1 or not -.02<=ros-msg.header.stamp.to_sec()<1:
            raise ValueError('stale:'+topic+'; '+sample_telemetry_details(streams,ros,wall))
        expected=before['/window_pass_lio/Odometry'][0] if topic=='/window_pass_lio/Odometry' else '/mavros'
        if any((m._connection_header or {}).get('callerid')!=expected for _,m in rows):
            raise ValueError('untrusted source:'+topic+'; expected='+expected+'; '+sample_telemetry_details(streams,ros,wall))


def sample(local_origin_required=True,include_perception=False):
    import rospy,rosgraph
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State,ExtendedState,RCIn
    rospy.init_node('bind_current_hover_target_readonly',anonymous=True)
    master=rosgraph.Master(rospy.get_name())
    def identity():
        pubs=dict(master.getSystemState()[0]);r={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'run_id':rospy.get_param('/window_pass_session_launch/run_id','')}
        topics=['/mavros/state','/window_pass_lio/Odometry','/window_pass_competition/status']
        if include_perception:topics.append('/window_live/status')
        for topic in topics:
            names=pubs.get(topic,[])
            if len(names)!=1:raise ValueError('source missing or multiple:'+topic)
            r[topic]=[names[0],master.lookupNode(names[0])]
        if pubs.get('/mavros/setpoint_raw/local'):raise ValueError('existing flight controller')
        return r
    before=identity();streams={};lock=threading.Lock()
    def receive(msg,topic):
        with lock:streams.setdefault(topic,[]).append((time.monotonic(),msg))
    subs=[rospy.Subscriber(t,k,receive,callback_args=t,queue_size=100) for t,k in [('/mavros/local_position/odom',Odometry),('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn),('/window_pass_lio/Odometry',Odometry)]]
    time.sleep(8)
    with lock:
        ros=rospy.Time.now().to_sec();wall=time.monotonic()
        require_sample_telemetry(streams,before,ros,wall)
        if any(m.armed or not m.connected or m.system_status!=3 or m.mode not in ('MANUAL','ALTCTL','POSCTL') for _,m in streams['/mavros/state']):raise ValueError('ground state changed')
        if any(m.landed_state!=1 for _,m in streams['/mavros/extended_state']):raise ValueError('not landed')
        if any(len(m.channels)<7 or not 900<=m.channels[6]<=1200 for _,m in streams['/mavros/rc/in']):raise ValueError('A must remain off')
        poses=[m for _,m in streams['/mavros/local_position/odom']]
        points=[(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z) for m in poses]
        if not all(math.isfinite(v) for p in points for v in p):raise ValueError('invalid position')
        if before!=identity():raise ValueError('localization session changed')
        raw_points=[(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z) for _,m in streams['/window_pass_lio/Odometry']]
        for point in raw_points:
            if not vector(point):raise ValueError('raw_lio_nonfinite')
            if local_origin_required:require_local_origin(point)
        q=poses[-1].pose.pose.orientation
        return dict(identity=before,position_local_m=[statistics.mean(p[i] for p in points) for i in range(3)],
            orientation_xyzw=[q.x,q.y,q.z,q.w],pose_samples=len(poses),
            xyz_span_m=[max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)],
            raw_lio=dict(samples=len(raw_points),position_local_m=list(raw_points[-1]),
                         xyz_span_m=[max(p[i] for p in raw_points)-min(p[i] for p in raw_points) for i in range(3)]),
            local_frame=poses[-1].header.frame_id,body_frame=poses[-1].child_frame_id)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target1-area-confirmed',action='store_true')
    a=p.parse_args()
    if not a.target1_area_confirmed:p.error('current target1 area and heading confirmation required')
    cfg=bind(json.loads((ROOT/'config/hover_target_from_traverse.json').read_text()),sample())
    out=Path(tempfile.mkdtemp(prefix='hover_target1_trial_',dir='/home/abot'))
    (out/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    launch='#!/usr/bin/env bash\nset -eo pipefail\nbash '+str(ROOT/'run_hover_target.bash')+' --config '+str(out/'config.json')+' --commissioning-hover-target "$@" 2>&1 | tee -a '+str(out/'mission.log')+'\n'
    (out/'run.bash').write_text(launch)
    print(json.dumps(dict(trial_directory=str(out),profile=str(out/'config.json'),pose_samples=cfg['site_evidence']['snapshot']['pose_samples'],xyz_span_m=cfg['site_evidence']['snapshot']['xyz_span_m'],deferred_checks=sorted(DEFERRALS),flight_executed=False),indent=2))

if __name__=='__main__':main()
