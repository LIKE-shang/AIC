#!/usr/bin/env python3
"""Sample a stationary, disarmed vehicle and write a NEW motion-disabled profile."""
import argparse,json,math,statistics,sys,time,threading,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from field_site import bind_bounds
from mission import settings
from takeoff import takeoff_blockers
from window_pass.config import load_config
from window_pass.geometry import tilt

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/takeoff_land.json')
    p.add_argument('--boundaries',type=Path,default=ROOT/'FIELD_BOUNDARY_CORRECTED_20261018.json')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--same-start-confirmed',action='store_true',help='Vehicle is at the surveyed start location and facing the surveyed direction')
    a=p.parse_args()
    if not a.same_start_confirmed:p.error('Confirm the vehicle is at the surveyed start; do not reuse distances after moving the start')
    if a.output.exists():raise ValueError('output already exists; choose a new path')
    cfg=json.loads(a.config.read_text());control=load_config(cfg['control']);options=settings(cfg['mission'])
    if cfg.get('experiment') not in ('takeoff_land','align_only','straight_pass') or any(control[k] for k in ('enable_motion','enable_traverse','pass_stage_approved')):
        raise ValueError('binding accepts only a motion-disabled takeoff profile')
    measurements=json.loads(a.boundaries.read_text())
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState,RCIn
    from nav_msgs.msg import Odometry
    from auto_window import Runtime
    rospy.init_node('bind_takeoff_site_readonly',anonymous=True)
    r=Runtime.__new__(Runtime);r.ros=rospy;r.master=rosgraph.Master(rospy.get_name());r.airborne_geometry=cfg.get("acquisition")=="airborne_rgbd";r.passive_rgbd=cfg.get("acquisition")=="takeoff_rgbd_observe"
    identity=r.identity();rows=[];states=[];landed=[];rc=[];lock=threading.Lock()
    def receive(dst,m):
        with lock:dst.append((time.monotonic(),m))
    subs=[rospy.Subscriber(t,k,lambda m,d=d:receive(d,m),queue_size=100) for t,k,d in
          [('/mavros/local_position/odom',Odometry,rows),('/mavros/state',State,states),
           ('/mavros/extended_state',ExtendedState,landed),('/mavros/rc/in',RCIn,rc)]]
    time.sleep(4)
    with lock:
        pose_samples=list(rows);state_samples=list(states);ground_samples=list(landed);rc_samples=list(rc)
    now=rospy.Time.now().to_sec();wall=time.monotonic()
    for stream in (pose_samples,state_samples,ground_samples,rc_samples):
        if not stream:raise ValueError('required live stream missing')
        received,last=stream[-1]
        if not 0<=wall-received<=1 or not 0<=now-last.header.stamp.to_sec()<=1:raise ValueError('stale telemetry')
        if any((getattr(m,'_connection_header',{}) or {}).get('callerid')!='/mavros' for _,m in stream):raise ValueError('unexpected telemetry source')
    if any(not m.connected or m.armed or m.system_status!=3 or m.mode not in ('MANUAL','POSCTL') for _,m in state_samples):raise ValueError('disarmed healthy ground state required')
    if any(m.landed_state!=1 for _,m in ground_samples):raise ValueError('landed state required')
    if any(len(m.channels)<7 or not 900<=m.channels[6]<=1200 for _,m in rc_samples):raise ValueError('A must remain off')
    if len(pose_samples)<50:raise ValueError('insufficient stationary pose samples')
    points=[(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z) for _,m in pose_samples]
    if any(not math.isfinite(v) for p in points for v in p):raise ValueError('nonfinite pose')
    spans=[max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)]
    if max(spans)>options['ground_z_span_limit_m']:raise ValueError('vehicle or localization not stationary')
    if any(m.header.frame_id!=control['local_frame'] or m.child_frame_id!=control['body_frame'] for _,m in pose_samples):raise ValueError('pose frame mismatch')
    q=pose_samples[-1][1].pose.pose.orientation;orientation=(q.x,q.y,q.z,q.w)
    if tilt(orientation)>control['tilt_limit_rad']:raise ValueError('vehicle not approximately level')
    position=[statistics.mean(p[i] for p in points) for i in range(3)]
    from session_window import settings as window_settings,decorate
    geometry=bind_bounds(measurements,position,orientation,options,decorate(dict(control),window_settings(cfg.get('session_window'))))
    control.update({k:geometry[k] for k in ('geofence_min_m','geofence_max_m')})
    control['evidence_id']='ground_site_binding_'+identity['boot_id']
    errors=takeoff_blockers(control)
    if errors:raise ValueError('invalid takeoff configuration: '+str(errors))
    if r.identity()!=identity:raise ValueError('localization session changed during binding')
    cfg['control']=control;cfg['site_identity']=identity;cfg['evidence_id']=control['evidence_id']
    cfg['site_evidence']={'geometry':geometry,'xyz_span_m':spans,'pose_samples':len(points),
                          'boundary_file_sha256':hashlib.sha256(a.boundaries.read_bytes()).hexdigest(),
                          'operator_confirmed_same_start':True,'flight_authorized':False}
    # Original acceptance values are preserved: sampling is not a flight test.
    with a.output.open('x') as out:json.dump(cfg,out,ensure_ascii=False,indent=2);out.write(chr(10))
    print(json.dumps({'result':'SITE_PROFILE_WRITTEN_MOTION_DISABLED','profile':str(a.output),
                      'geometry':geometry,'acceptance_changed':False},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
