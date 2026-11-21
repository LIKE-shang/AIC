#!/usr/bin/env python3
"""Compute fixed-distance offsets from manually reviewed spot measurements.
Output always remains UNCONFIRMED; no hardware or ROS. Never infer distance.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from robot_vision_test.air_target import air_parameters


def compute(measurements,base=None):
    m=measurements
    required=['calibration_id','camera_frame_id','camera_settings_record','image_width','image_height',
              'distance_m','distance_tolerance_m','pixels_per_m_min','measurement_error_px',
              'additional_error_m','samples']
    if any(k not in m or m[k] is None for k in required): raise ValueError('incomplete measured data')
    for k in required[:-1]:
        if isinstance(m[k],(int,float)) and not math.isfinite(m[k]): raise ValueError('nonfinite '+k)
    if (not all(m[k] for k in ['calibration_id','camera_frame_id','camera_settings_record']) or
        any(m[k]<=0 for k in ['image_width','image_height','distance_m','distance_tolerance_m',
                             'pixels_per_m_min','measurement_error_px','additional_error_m']) or len(m['samples'])<5):
        raise ValueError('need >=5 reviewed samples and positive measured scale/error/distance values')
    pts=[]; sizes=[]; major=[]; ratios=[]; angles=[]
    for s in m['samples']:
        v=[s[k] for k in ['x_px','y_px','spot_radius_px','outer_major_px','axis_ratio','angle_deg','distance_m']]
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in v): raise ValueError('nonfinite sample')
        if not (0<=v[0]<m['image_width'] and 0<=v[1]<m['image_height'] and v[2]>0 and v[3]>0 and 0<v[4]<=1):
            raise ValueError('invalid sample bounds')
        if abs(v[6]-m['distance_m'])>m['distance_tolerance_m']:
            raise ValueError('mixed distances: make separate profiles, never silently interpolate')
        pts.append(v[:2]); sizes.append(v[2]); major.append(v[3]); ratios.append(v[4]); angles.append(v[5])
    pts=np.asarray(pts,float); mean=pts.mean(axis=0)
    # Max observed residual, not optimistic standard deviation. Additional pose/lens/
    # target-fitting uncertainty must be measured separately; not solved by more samples.
    residual=float(np.linalg.norm(pts-mean,axis=1).max())
    p=air_parameters(base or {})
    c=p['calibration']
    c.update(confirmed=False,calibration_id=m['calibration_id'],camera_frame_id=m['camera_frame_id'],
             image_width=int(m['image_width']),image_height=int(m['image_height']),
             offset_px=(mean-np.array([(m['image_width']-1)/2,(m['image_height']-1)/2])).tolist(),
             distance_m=float(m['distance_m']),distance_tolerance_m=float(m['distance_tolerance_m']),
             outer_major_px=float(np.median(major)),axis_ratio=float(np.median(ratios)),
             angle_deg=float((math.degrees(math.atan2(np.mean(np.sin(np.radians(np.array(angles)*2))),
                                                    np.mean(np.cos(np.radians(np.array(angles)*2)))))/2)%180),
             spot_radius_m=(max(sizes)+m['measurement_error_px'])/m['pixels_per_m_min'],
             error_bound_m=(residual+m['measurement_error_px'])/m['pixels_per_m_min']+m['additional_error_m'])
    p['dimensions_confirmed']=False
    p['working_distance_m']=m['distance_m']
    p['operating_envelope_confirmed']=False; p['live_camera_verified']=False
    p['prediction_only_acknowledged']=False; p['spot']['validated']=False
    return p,dict(samples=len(pts),max_residual_px=residual,physical_measurements_verified=False,
                  camera_settings_record=m['camera_settings_record'],review_required=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements',required=True); parser.add_argument('--output',required=True)
    parser.add_argument('--base-config',default=str(ROOT/'config/circle_laser_test.yaml'))
    args=parser.parse_args(argv)
    source=Path(args.measurements); raw=source.read_bytes()
    result,report=compute(json.loads(raw.decode('utf-8-sig')),yaml.safe_load(Path(args.base_config).read_text(encoding='utf-8-sig')))
    report['input_sha256']=hashlib.sha256(raw).hexdigest()
    out=Path(args.output); out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text('# UNCONFIRMED calibration from measured samples. Review before field use.\n'+yaml.safe_dump(result,sort_keys=False),encoding='utf-8')
    out.with_suffix('.review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('UNCONFIRMED profile written:',out)
    return result

if __name__=='__main__': main()
