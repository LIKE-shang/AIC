#!/usr/bin/env python3
"""Read-only image/manifest/ROS bag replay. Never starts ROS, publishes or calls PWM.
Scope: geometry, calibration and capture freshness; NOT physical/3-second acceptance.
"""
import argparse
import json
from pathlib import Path
import sys
import cv2
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from robot_vision_test.air_target import (air_parameters,detect_air_target,detect_spot,assess_aim,
                                          assess_spot,draw_air_overlay,jsonable)
from robot_vision_test.safety import FrameFreshness


def frames(args):
    if args.bag:
        try:
            import rosbag
            from cv_bridge import CvBridge
        except ImportError as exc:
            raise RuntimeError('--bag requires robot-side ROS1 rosbag/cv_bridge; use --input/--manifest locally') from exc
        bridge=CvBridge()
        with rosbag.Bag(args.bag,'r') as bag:
            for _,msg,received in bag.read_messages(topics=[args.topic]):
                if getattr(msg,'_type','') != 'sensor_msgs/Image':
                    raise ValueError('selected bag topic must contain uncompressed sensor_msgs/Image')
                yield bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8'),msg.header.stamp.to_sec(),received.to_sec(),msg.header.frame_id,False
    elif args.manifest:
        source=Path(args.manifest)
        records=json.loads(source.read_text(encoding='utf-8-sig'))
        for r in records:
            image=cv2.imread(str(source.parent/r['file']))
            if image is None: raise ValueError('cannot read manifest image '+r['file'])
            yield image,float(r['capture_stamp']),float(r['received_stamp']),r['camera_frame_id'],False
    else:
        source=Path(args.input)
        paths=sorted(p for p in source.iterdir() if p.suffix.lower() in ('.png','.jpg','.jpeg','.bmp')) if source.is_dir() else [source]
        for i,path in enumerate(paths):
            image=cv2.imread(str(path))
            if image is None: raise ValueError('cannot read image '+str(path))
            # No true timestamps in a still image; never present these as measured data.
            yield image,1+i*.1,1+i*.1,args.frame_id,True


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--input'); source.add_argument('--manifest'); source.add_argument('--bag')
    parser.add_argument('--topic',default='/usb_cam/image_raw')
    parser.add_argument('--frame-id',default='unverified_camera')
    parser.add_argument('--config',default=str(ROOT/'config/circle_laser_test.yaml'))
    parser.add_argument('--output',required=True)
    args=parser.parse_args(argv)
    p=air_parameters(yaml.safe_load(Path(args.config).read_text(encoding='utf-8-sig')))
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    fresh=FrameFreshness(.35,.05,True)
    report=[]
    for i,(image,stamp,received,frame_id,synthetic_clock) in enumerate(frames(args)):
        capture_valid=True; capture_error=''
        try: fresh.check(stamp,received)
        except ValueError as exc: capture_valid=False; capture_error=str(exc)
        d,mask=detect_air_target(image,p); s=detect_spot(image,p)
        target=d['targets'][0] if d['valid'] else None
        a=assess_aim(target,image.shape,frame_id,p) if target is not None else {}
        observed=bool(target is not None and assess_spot(s,a,target,p))
        item=dict(index=i,capture_stamp=stamp,received_stamp=received,camera_frame_id=frame_id,
                  synthetic_clock=synthetic_clock,capture_valid=capture_valid,capture_error=capture_error,
                  detection=d,aim=a,spot=s,spot_geometry_inside=observed,
                  dry_run=True,physical_hit_verified=False,request_on=False,
                  scope='geometry_and_capture_freshness_only_NO_ON_NO_3s_claim')
        report.append(jsonable(item))
        draw_air_overlay(image,d,a,s,['OFFLINE ONLY - NO laser command',d['reason'],a.get('reason','no_target'),
                                     'capture '+('SYNTHETIC' if synthetic_clock else 'VALID' if capture_valid else capture_error)])
        if not cv2.imwrite(str(output/('%06d_annotated.png'%i)),image): raise RuntimeError('image write failed')
        cv2.imwrite(str(output/('%06d_mask.png'%i)),mask)
    if not report: raise ValueError('no image messages/files found; check actual topic')
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(frames=len(report),unique_targets=sum(r['detection']['valid'] for r in report),
                         fresh_frames=sum(r['capture_valid'] for r in report),hardware_access=False,
                         report=str((output/'report.json').resolve())),ensure_ascii=False))
    return report

if __name__=='__main__': main()
