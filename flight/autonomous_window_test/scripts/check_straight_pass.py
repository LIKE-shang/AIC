#!/usr/bin/env python3
"""Subscribe-only feasibility audit. Never publishes setpoints or calls FCU services."""
import argparse,json,time,sys,threading
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from window_pass.config import load_config
from window_pass.core import Pose
from straight_pass import PassageDecision

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True,type=Path);p.add_argument('--seconds',type=float,default=10.)
    a=p.parse_args()
    if not 1<=a.seconds<=30:raise ValueError('audit duration must be 1..30 seconds')
    cfg=json.loads(a.config.read_text());c=load_config(cfg['control'])
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    rospy.init_node('straight_pass_feasibility_readonly',anonymous=True)
    lock=threading.RLock();rows={};decision=PassageDecision();reasons={};frames=[];last_stamp=None
    after=rospy.Time.now().to_sec()
    def pose(m):
        if (getattr(m,'_connection_header',{}) or {}).get('callerid')!='/mavros':return
        if m.header.frame_id!=c['local_frame'] or m.child_frame_id!=c['body_frame']:return
        q=m.pose.pose.orientation;v=m.pose.pose.position
        with lock:rows['pose']=Pose(m.header.stamp.to_sec(),(v.x,v.y,v.z),(q.x,q.y,q.z,q.w),m.header.frame_id,m.child_frame_id)
    def status(m):
        if (getattr(m,'_connection_header',{}) or {}).get('callerid')!='/window_live_vision':return
        try:value=json.loads(m.data)
        except (TypeError,ValueError):return
        if not isinstance(value,dict):return
        with lock:rows['status']=(time.monotonic(),value)
    subs=[rospy.Subscriber('/mavros/local_position/odom',Odometry,pose,queue_size=30),rospy.Subscriber('/window_live/status',String,status,queue_size=5)]
    end=time.monotonic()+a.seconds
    while time.monotonic()<end and not rospy.is_shutdown():
        with lock:
            if 'pose' in rows and 'status' in rows:
                received,s=rows['status'];stamp=s.get('source_stamp')
                # Heartbeat repeats are not independent camera evidence.
                if stamp is not None and stamp!=last_stamp:
                    last_stamp=stamp
                    safe=decision.update(s,c,rows['pose'],rospy.Time.now().to_sec(),time.monotonic(),received,after)
                    reasons[decision.reason]=reasons.get(decision.reason,0)+1
                    frames.append(dict(source_stamp=stamp,metric_geometry_ready=s.get('metric_geometry_ready'),reason=decision.reason,ready=safe))
        time.sleep(.02)
    print(json.dumps({'kind':'SUBSCRIBE_ONLY_STRAIGHT_PASS_AUDIT','actual_output_permitted':False,'flight_authority':False,
        'received_streams':sorted(rows),'unique_status_frames':len(frames),'reasons':reasons,'decision':decision.snapshot(),'recent_frames':frames[-30:]},indent=2))
    for s in subs:s.unregister()

if __name__=='__main__':main()
