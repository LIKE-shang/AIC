#!/usr/bin/env python3
"""Robot-side READ-ONLY USB topic audit; no publishers, PWM or flight commands.
Uses monotonic arrival time even when /clock stops. Captures raw PNG and manifest.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic',required=True); parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--output',required=True); parser.add_argument('--save-every',type=int,default=30)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=120 or args.save_every<1:
        raise ValueError('invalid bounded audit duration/save interval')
    import cv2
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    rospy.init_node('air_camera_readonly_audit',anonymous=True,disable_signals=True)
    bridge=CvBridge(); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    rows=[]; manifest=[]; errors=[]; lock=threading.Lock()
    def receive(msg):
        with lock:
            now=time.monotonic(); ros_now=rospy.Time.now().to_sec()
            row=dict(arrival_monotonic=now,capture_stamp=msg.header.stamp.to_sec(),received_stamp=ros_now,
                     frame_seq=msg.header.seq,camera_frame_id=msg.header.frame_id,width=msg.width,height=msg.height,
                     encoding=msg.encoding,publisher=getattr(msg,'_connection_header',{}).get('callerid','unknown'))
            rows.append(row)
            if (len(rows)-1)%args.save_every==0:
                try:
                    image=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    name='capture_%06d.png'%(len(rows)-1)
                    if not cv2.imwrite(str(out/name),image): raise ValueError('PNG write failed')
                    manifest.append(dict(row,file=name))
                except Exception as exc: errors.append(str(exc))
    sub=rospy.Subscriber(args.topic,Image,receive,queue_size=1,buff_size=2**24)
    stop=time.monotonic()+args.seconds
    while time.monotonic()<stop and not rospy.is_shutdown(): time.sleep(.05)
    sub.unregister()
    with lock:
        arrivals=[r['arrival_monotonic'] for r in rows]; stamps=[r['capture_stamp'] for r in rows]
        gaps=[b-a for a,b in zip(stamps,stamps[1:])]
        ages=[r['received_stamp']-r['capture_stamp'] for r in rows]
        report=dict(topic=rospy.resolve_name(args.topic),samples=len(rows),duration_wall_s=args.seconds,
                    arrival_fps=(len(rows)-1)/(arrivals[-1]-arrivals[0]) if len(rows)>1 else None,
                    capture_fps=1/statistics.median(gaps) if gaps and min(gaps)>0 else None,
                    repeated_or_backwards=sum(g<=0 for g in gaps),max_capture_gap_s=max(gaps) if gaps else None,
                    min_age_s=min(ages) if ages else None,max_age_s=max(ages) if ages else None,
                    zero_stamps=sum(s<=0 for s in stamps),
                    formats=sorted(set((r['width'],r['height'],r['encoding']) for r in rows)),
                    publishers=sorted(set(r['publisher'] for r in rows)),
                    frames=sorted(set(r['camera_frame_id'] for r in rows)),errors=errors,
                    device_identity_verified=False,physical_exposure_timestamp_verified=False,
                    note='Driver header time is not automatically sensor exposure time; verify latency on site.')
        (out/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        (out/'all_headers.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
        (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    rospy.signal_shutdown('read-only audit completed')
    if not rows: raise SystemExit('NO IMAGE RECEIVED: topic/camera must be checked, not verified')

if __name__=='__main__': main()
