#!/usr/bin/env python3
"""Ground-only startup wrappers. No FCU command or parameter writes."""
import argparse,fcntl,hashlib,os,time,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
LEGACY=Path('/home/abot/window_pass_session_20261013')
PINNED={'scripts/session.py':'d9443e4ab0dad2d5b0fb7df9244b31d69a62846defc6a4f2ae7b457827337273',
        'session.bash':'109cb821ac311c6c5b2c2f55c85488a4093844ecd7e5b7ac33ef64df285d8e40',
        'launch/base.launch':'b9097f8a7e682d6d36c92db44bf491d503dba98d05850f7e2d04f76249c8af0a'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('component',choices=['base','perception']);a=p.parse_args()
    import rosgraph
    if a.component=='base':
        lock=(Path.home()/'.window_pass_session_launch.lock').open('a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);os.set_inheritable(lock.fileno(),True)
        if rosgraph.is_master_online():raise RuntimeError('MASTER_ALREADY_RUNNING: preserve existing base; do not restart it')
        for name,digest in PINNED.items():
            if hashlib.sha256((LEGACY/name).read_bytes()).hexdigest()!=digest:raise RuntimeError('DEPLOYED_VERSION_MISMATCH:'+name)
        command=['roslaunch',str(LEGACY/'launch/base.launch'),'run_id:='+uuid.uuid4().hex,
                 'start_camera:=false','start_detector:=false']
        print('BASE: localization only. Keep this terminal. If MANUAL is required, prepare mode in the command terminal.',flush=True)
    else:
        lock=(Path.home()/'.autonomous_window_perception.lock').open('a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);os.set_inheritable(lock.fileno(),True)
        import rospy
        from mavros_msgs.msg import State,ExtendedState,RCIn
        rospy.init_node('window_perception_ground_start',anonymous=True)
        e=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=3)
        r=rospy.wait_for_message('/mavros/rc/in',RCIn,timeout=3)
        s=rospy.wait_for_message('/mavros/state',State,timeout=3)
        now=rospy.Time.now().to_sec()
        if not (s.connected and not s.armed and s.system_status==3 and e.landed_state==1 and len(r.channels)>=7 and 900<=r.channels[6]<=1200):raise RuntimeError('DISARMED_GROUND_A_OFF_REQUIRED')
        if not all(0<=now-m.header.stamp.to_sec()<2 and (getattr(m,'_connection_header',{}) or {}).get('callerid')=='/mavros' for m in (s,e,r)):raise RuntimeError('FRESH_MAVROS_TELEMETRY_REQUIRED')
        pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        for topic in ('/camera/rgb/image_raw','/window_live/status','/window_pass_test/observation','/mavros/setpoint_raw/local','/window_pass_align/status','/window_pass_hold/status','/window_pass_test/status'):
            if pubs.get(topic):raise RuntimeError('ALREADY_RUNNING_OR_CONTROL_PRESENT:'+topic)
        command=['roslaunch',str(ROOT/'launch/perception.launch')]
        print('PERCEPTION: RGB, depth and two observers; no flight output.',flush=True)
    # Apply the established CPU budget even when this Python entry is called
    # directly during ground recovery rather than via start_perception.bash.
    if a.component=='perception':
        for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
            os.environ[key]='1'
    os.execvp(command[0],command)

if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('STARTUP_STOP: '+str(exc))
