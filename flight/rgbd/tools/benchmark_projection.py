"""Offline synthetic projection timing. Not a Nano performance measurement."""
import json
import platform
import statistics
import time
from pathlib import Path
import cv2
import numpy as np
from window_live_vision.rgbd import project_rgb_pixels

cv2.setNumThreads(1)
yy,xx=np.indices((400,640))
z=2.+.001*xx.ravel()
xyz=np.column_stack(((xx.ravel()-320.)/500.*z,(yy.ravel()-200.)/500.*z,z))
k=np.array([[516.,0.,321.],[0.,507.,239.],[0.,0.,1.]])
d=np.array([.12,-.04,.001,-.002,.003])
rows={}
values={}
for name,fn in [('opencv_projectPoints',lambda:cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)),
                ('direct_brown_projection',lambda:project_rgb_pixels(xyz,k,d))]:
    fn()
    durations=[]
    for _ in range(7):
        start=time.perf_counter();value=fn();durations.append(1000.*(time.perf_counter()-start))
    rows[name]=dict(median_ms=statistics.median(durations),samples_ms=durations)
    values[name]=value
np.testing.assert_allclose(values['opencv_projectPoints'],values['direct_brown_projection'],rtol=1e-12,atol=1e-8)
result=dict(synthetic_points=len(xyz),platform=platform.platform(),OpenCV=cv2.__version__,NumPy=np.__version__,
            timing=rows,max_abs_pixel_difference=float(np.max(np.abs(values['opencv_projectPoints']-values['direct_brown_projection']))),
            Nano_measured=False,full_pipeline_measured=False,actual_output_permitted=False)
out=Path(__file__).resolve().parents[2]/'_diagnostics/window_live_rgbd_20261014_v4'
out.mkdir(parents=True,exist_ok=True)
(out/'projection_benchmark.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
