"""Save auditable offline overlays and host timings; no live-flight inference."""
import hashlib
import json
import statistics
import sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'test'))
from window_live_vision.detector import Detector, annotate, quad_center
from scenes import scene


def main():
    cv2.setNumThreads(1)
    out=ROOT.parent/'_diagnostics/window_live_20261013'
    out.mkdir(parents=True,exist_ok=True)
    field_path=ROOT.parent/'window_pass_test/test/window_checker_field_20261012.jpg'
    previous_path=ROOT.parent/'window_pass_test/test/window_checker.png'
    field=cv2.imread(str(field_path));previous=cv2.imread(str(previous_path))
    small,_=scene(170,120,cell=5,border=20)
    plain,_=scene(260,200,border=24,solid=True)
    tall,_=scene(170,280,cell=7,border=26)
    ordinary,q=scene()
    dest=np.float32([[180,135],[475,165],[450,360],[165,350]])
    perspective=cv2.warpPerspective(ordinary,cv2.getPerspectiveTransform(q,dest),(640,480),borderValue=(155,155,155))
    dark=np.clip(ordinary.astype(float)*.4+25,0,255).astype(np.uint8)
    missing=ordinary.copy();missing[98:145,120:510]=155
    multiple,_=scene(170,130,center=(160,240),cell=6,border=20)
    other,_=scene(170,130,center=(480,240),cell=6,border=20);multiple[:,320:]=other[:,320:]
    cases=[('field_existing_image',field),('previous_existing_image',previous),('small_unknown_size',small),
           ('portrait_unknown_size',tall),('perspective',perspective),('low_light',dark),
           ('plain_border',plain),('missing_side_rejected',missing),('two_frames_ambiguous',multiple)]
    rows,tiles={},[]
    detector=Detector()
    for name,image in cases:
        detections,detail=detector.detect(image)
        overlay=annotate(image,detections,name+' / '+detail['reason'])
        cv2.imwrite(str(out/(name+'.jpg')),overlay)
        rows[name]=detail
        tiles.append(cv2.resize(overlay,(480,360)))
    mosaic=np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,9,3)])
    cv2.imwrite(str(out/'validation_grid.jpg'),mosaic)
    timings=[]
    for _ in range(12):
        _,detail=detector.detect(field);timings.append(detail['processing_ms'])
    report=dict(cases=rows,field_input_sha256=hashlib.sha256(field_path.read_bytes()).hexdigest(),
        previous_input_sha256=hashlib.sha256(previous_path.read_bytes()).hexdigest(),
        field_input_has_legacy_text_overlay=True,
        provenance='Existing local fixtures only; not the missing 2026-10-13 airborne bag.',
        desktop_single_thread_timings_ms=dict(n=len(timings),median=statistics.median(timings),
            p95=float(np.percentile(timings,95)),maximum=max(timings)),
        note='Desktop timings are not Jetson Nano measurements. No ROS live test or flight output.')
    (out/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(counts={n:r['target_count'] for n,r in rows.items()},
        desktop_single_thread_timings_ms=report['desktop_single_thread_timings_ms']),indent=2))


if __name__=='__main__':main()
