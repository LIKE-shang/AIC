"""Synthetic RGB-D and offline route report; never connected to a vehicle."""
import json
import math
from pathlib import Path
import statistics
import sys
import time
import cv2
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'test'))
from test_rgbd import depth_scene, geometry
from test_guidance import camera
from test_traversal import safety, pose, synthetic_evidence
from window_live_vision.detector import annotate
from window_live_vision.geometry import estimate_plane
from window_live_vision.rgbd import DepthOpeningDetector, DepthRegistrar, aperture_evidence
from window_live_vision.traversal import plan_route, TraversalModel, evaluate_corridor


def main():
    cv2.setNumThreads(1)
    out = ROOT.parent/'_diagnostics/window_live_rgbd_20261013_v2'
    out.mkdir(parents=True, exist_ok=True)
    k = np.asarray(camera()['K']).reshape(3, 3)
    d = np.zeros(5)
    obstructed = depth_scene(); obstructed[165:310,245:395] = 1.2
    tilted = depth_scene(); yy,xx=np.indices(tilted.shape)
    ring=tilted==2.; tilted[ring] = 2./(1.+.08*(xx[ring]-320.)/500.)
    cases = [('Valid opening; RGB has no texture',depth_scene()),
             ('Invalid inside = UNKNOWN, not free',depth_scene(float('nan'))),
             ('Foreground obstruction blocks opening',obstructed),
             ('Uniform wall is not an opening',np.full((480,640),2.)),
             ('Missing top border is rejected',depth_scene(missing='top')),
             ('Tilted plane estimated from four sides',tilted)]
    panels=[]; results={}; detector=DepthOpeningDetector()
    for name,meters in cases:
        valid=np.isfinite(meters)
        levels=np.uint8(np.clip(np.nan_to_num(meters,nan=0.)/5.*255,0,255))
        frame=cv2.applyColorMap(levels,cv2.COLORMAP_TURBO);frame[~valid]=[35,35,35]
        found=detector.detect(meters,k,d)
        detail={'targets':len(found),'source':'SYNTHETIC_DEPTH_NOT_GEMINI_CAPTURE'}
        if len(found)==1:
            plane=estimate_plane(found[0],meters,k,d)
            detail.update(plane_rms_m=plane['rms_m'],opening=aperture_evidence(found[0],meters,k,d,plane))
        panel=annotate(frame,found,'SYNTHETIC / '+str(len(found))+' target(s)')
        panel=cv2.copyMakeBorder(panel,34,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245))
        cv2.putText(panel,name,(10,23),cv2.FONT_HERSHEY_SIMPLEX,.58,(25,25,25),1,cv2.LINE_AA)
        panels.append(panel);results[name]=detail
    grid=np.vstack([np.hstack(panels[i:i+2]) for i in range(0,6,2)])
    cv2.imwrite(str(out/'synthetic_rgbd_grid.jpg'),grid)
    rgb_info=camera();depth_info=dict(rgb_info,frame_id='depth_optical')
    raw=(depth_scene()*1000).astype(np.uint16);registrar=DepthRegistrar();timings=[]
    for _ in range(8):
        start=time.perf_counter()
        projected,kr,dr=registrar.register(raw,'16UC1',10.,'depth_optical',rgb_info,depth_info,10.,'rgb_optical',(480,640),np.eye(3),[.004,0.,0.])
        found=detector.detect(projected,kr,dr)
        if len(found)==1:estimate_plane(found[0],projected,kr,dr)
        timings.append((time.perf_counter()-start)*1000)
    plan=plan_route(geometry(),pose(),safety(),10.)
    model=TraversalModel(plan,pose(),10.,synthetic_evidence(plan),'OFFLINE_SIMULATION_ONLY')
    rows=[];states=[];now=10.
    for _ in range(3000):
        now+=.05
        state=model.tick(pose(now,model.target,model.yaw),now,None if model.frozen_stamp else geometry(now),synthetic_evidence(plan,now))
        rows.append(dict(stamp=now,**state))
        if not states or states[-1]!=state['state']:states.append(state['state'])
        if state['state'] in ('DONE','ABORT'):break
    canvas=np.full((600,1200,3),248,np.uint8)
    cv2.putText(canvas,'OFFLINE SYNTHETIC: safe body-origin region and route (NOT flight results)',(20,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(20,20,20),1,cv2.LINE_AA)
    def plot_polygon(values,color,thickness):
        q=np.asarray(values);screen=np.column_stack((300+q[:,0]*270,310-q[:,1]*270)).astype(np.int32)
        cv2.polylines(canvas,[screen],True,color,thickness,cv2.LINE_AA)
    plot_polygon(plan['opening_polygon_uv_m'],(30,30,30),3)
    plot_polygon(plan['safe_center_polygon_uv_m'],(40,155,30),3)
    cv2.putText(canvas,'Black: measured aperture',(35,530),cv2.FONT_HERSHEY_SIMPLEX,.6,(20,20,20),1)
    cv2.putText(canvas,'Green: permissible BODY ORIGIN region',(35,560),cv2.FONT_HERSHEY_SIMPLEX,.55,(40,135,30),1)
    points=np.asarray(plan['waypoints_enu_m']);coords=np.column_stack((680+points[:,0]*130,390-(points[:,2]-1)*220)).astype(np.int32)
    cv2.line(canvas,(940,110),(940,480),(45,45,45),3)
    for a,b in zip(coords,coords[1:]):cv2.arrowedLine(canvas,tuple(a),tuple(b),(200,90,35),2,tipLength=.12)
    for i,point in enumerate(coords):
        cv2.circle(canvas,tuple(point),4,(40,40,210),-1)
        label=plan['waypoint_names'][i]
        cv2.putText(canvas,str(i)+': '+label,(650,80+i*28),cv2.FONT_HERSHEY_SIMPLEX,.52,(20,20,20),1)
    cv2.putText(canvas,'Side view: full body must clear plane',(650,530),cv2.FONT_HERSHEY_SIMPLEX,.55,(30,30,30),1)
    cv2.putText(canvas,'Example budgets are synthetic, never field config',(650,560),cv2.FONT_HERSHEY_SIMPLEX,.48,(30,30,150),1)
    cv2.imwrite(str(out/'synthetic_route_preview.png'),canvas)
    report=dict(source='SYNTHETIC_ONLY_NO_GEMINI_OR_ROS_HARDWARE_CAPTURE',cases=results,
        desktop_single_thread_ms=dict(samples=len(timings),median=statistics.median(timings),p95=float(np.percentile(timings,95)),maximum=max(timings)),
        timing_scope='raw projection + depth candidate generation + plane fit only; not whole ROS node, not Nano',
        offline_simulation=dict(final_state=model.state,states=states,elapsed_s=now-10,ideal_target_tracking=True,corridor='SYNTHETIC_ALL_FREE',actual_output_permitted=False),
        plan=plan)
    (out/'rgbd_report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    (out/'offline_simulation_trace.json').write_text(json.dumps(rows,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('source','desktop_single_thread_ms','timing_scope','offline_simulation')},indent=2))


if __name__=='__main__':main()
