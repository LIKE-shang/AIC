"""Airborne RGB-D window acquisition. No saved window coordinates required.
Only target-specific geometry gates are deferred until airborne acquisition.
Installation, calibration, fence, vehicle, budget and FCU gates remain mandatory.
"""
import math
from window_pass.core import Controller
from window_pass.config import load_config, blockers
from window_pass.geometry import vector, norm, sub, dot, yaw, wrap, lateral

DEFERRED={'missing:window_center_local_m','unverified:geometry_verified',
          'unverified:opening_geometry_verified'}

def flight_blockers(control,alignment_commissioning=False,straight_commissioning=False,session_window=False):
    skipped=set(DEFERRED)
    if session_window:
        # Legacy static box includes alignment + position + tracking reserves.
        # Session mode authorizes only through measured cylinder projection.
        skipped.add('insufficient:worst_case_clearance')
    if alignment_commissioning:
        from alignment_policy import CONTROL_DEFERRALS
        if control.get('enable_traverse') or control.get('pass_stage_approved'):
            raise ValueError('alignment commissioning forbids traversal permissions')
        skipped.update('unverified:'+k for k in CONTROL_DEFERRALS)
    if straight_commissioning:
        from straight_pass_policy import CONTROL_DEFERRALS
        skipped.update('unverified:'+k for k in CONTROL_DEFERRALS)
    return [b for b in blockers(control,motion=True) if b not in skipped]



def validate_metric(status, control, ros, wall, received, after_stamp, expected_epoch=None):
    if not (0<=wall-received<=.5):raise ValueError('rgbd_receipt_stale')
    if status.get('ready') is not True or status.get('metric_geometry_ready') is not True:
        raise ValueError('rgbd_target_not_confirmed')
    if status.get('target_count')!=1 or status.get('calibration_valid') is not True:
        raise ValueError('rgbd_unique_calibrated_target_required')
    if status.get('optical_frame')!=control['optical_frame']:
        raise ValueError('rgbd_optical_frame_mismatch')
    runtime=status.get('runtime') or {}
    if runtime.get('result_fresh') is not True:raise ValueError('rgbd_result_stale')
    g=status.get('metric_geometry') or {}
    stamp=g.get('source_stamp')
    if type(stamp) not in (int,float) or not math.isfinite(stamp) or not stamp>after_stamp or not -.02<=ros-stamp<=.5:
        raise ValueError('rgbd_geometry_not_fresh_after_takeoff')
    image_stamp=status.get('source_stamp')
    if type(image_stamp) not in (int,float) or not math.isfinite(image_stamp) or abs(image_stamp-stamp)>.06:
        raise ValueError('rgbd_image_pose_timestamp_mismatch')
    if g.get('local_frame_id')!=control['local_frame']:raise ValueError('rgbd_world_frame_mismatch')
    if g.get('metric_source')!='live_rgbd_four_frame_borders':raise ValueError('untrusted_metric_method')
    epoch=(g.get('pose_epoch'),g.get('calibration_epoch'))
    if not all(type(x) is int and x>=0 for x in epoch):raise ValueError('missing_rgbd_epoch')
    if expected_epoch is not None and epoch!=expected_epoch:raise ValueError('rgbd_epoch_changed')
    fast=control.get('_session_fast_confirmation',False)
    if g.get('confirmation_frames',0)<(3 if fast else 5):raise ValueError('rgbd_not_temporally_confirmed')
    if fast:
        span=g.get('confirmation_span_s')
        if type(span) not in (int,float) or not math.isfinite(span) or not .35<=span<=1.2:
            raise ValueError('rgbd_confirmation_span_invalid')
    for key,maximum in [('plane_rms_m',.02),('temporal_corner_scatter_m',.025)]:
        v=g.get(key)
        if type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=maximum:raise ValueError('invalid_'+key)
    inliers=g.get('depth_inlier_fraction')
    if type(inliers) not in (int,float) or not math.isfinite(inliers) or not .7<=inliers<=1:
        raise ValueError('insufficient_depth_inliers')
    if (g.get('opening_evidence') or {}).get('possible_opening') is not True:
        raise ValueError('opening_depth_evidence_required')
    center=g.get('center_enu_m');normal=g.get('normal_enu');corners=g.get('corners_enu_m')
    if not vector(center) or not vector(normal) or abs(norm(normal)-1)>.01:
        raise ValueError('invalid_metric_vectors')
    if not isinstance(corners,list) or len(corners)!=4 or not all(vector(p) for p in corners):
        raise ValueError('four_metric_corners_required')
    if abs(normal[2])>math.sin(control['tilt_limit_rad']):raise ValueError('window_plane_not_upright')
    # Either normal sign is legitimate for plane fitting; choose forward.
    heading=control['window_yaw_rad']
    if normal[0]*math.cos(heading)+normal[1]*math.sin(heading)<0:normal=[-x for x in normal]
    angle=math.atan2(normal[1],normal[0])
    if abs(wrap(angle-heading))>control['yaw_limit_rad']:raise ValueError('window_not_facing_vehicle')
    left=lateral(angle)
    yz=[(dot(sub(p,center),left),p[2]-center[2]) for p in corners]
    signs=[]
    for j in range(4):
        a,b,c=yz[j],yz[(j+1)%4],yz[(j+2)%4]
        signs.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not(all(x>0 for x in signs) or all(x<0 for x in signs)):raise ValueError('metric_corners_not_convex')
    if max(abs(dot(sub(p,center),normal)) for p in corners)>.025:raise ValueError('corners_not_on_window_plane')
    width=max(p[0] for p in yz)-min(p[0] for p in yz)
    height=max(p[1] for p in yz)-min(p[1] for p in yz)
    if any(abs(v-ref)/ref>control['opening_size_error_fraction'] for v,ref in zip((width,height),control['opening_size_m'])):
        raise ValueError('measured_window_size_mismatch')
    if any(abs(max(p[k] for p in yz)+min(p[k] for p in yz))>.08 for k in (0,1)):
        raise ValueError('metric_center_not_centered_in_opening')
    return dict(center=list(center),yaw=angle,stamp=stamp,epoch=epoch,width=width,height=height)


class AirborneController(Controller):
    def __init__(self,control,alignment_commissioning=False,alignment_only=False,straight_commissioning=False,session_window=False):
        self.straight_commissioning=straight_commissioning
        self.alignment_only=alignment_only
        if alignment_only and (control.get('enable_traverse') or control.get('pass_stage_approved')):raise ValueError('alignment envelope requires pass disabled')
        self.alignment_commissioning=alignment_commissioning
        cfg=load_config(control)
        errors=flight_blockers(cfg,alignment_commissioning,straight_commissioning,session_window)
        if errors:raise ValueError('static flight configuration refused: '+','.join(errors))
        if cfg['window_center_local_m']:raise ValueError('airborne mode refuses presurveyed window coordinates')
        super().__init__(dict(cfg,enable_motion=False))
        self.c=cfg
        self.blockers=['missing:live_window_geometry']
        self.live_bound=False
        self.live_epoch=None
        self.metric_input=None
        self.metric_received=-1e9
        self.acquisition_rows=[]
        self.acquisition_after=None
        self.live_reason='waiting_until_stable_airborne_search'

    def ingest_metric(self,status,received):
        self.metric_input=status
        self.metric_received=received
        # Invalid observations invalidate acquisition, not flight pose.
        if status.get('metric_geometry_ready') is not True:
            self.acquisition_rows=[]

    def acquire(self,ros,wall,after_stamp):
        if self.acquisition_after!=after_stamp:
            self.acquisition_after=after_stamp;self.acquisition_rows=[]
        c=dict(self.c,window_yaw_rad=yaw(self.pose.q))
        try:
            g=validate_metric(self.metric_input or {},c,ros,wall,self.metric_received,after_stamp)
            if self.acquisition_rows:
                old=self.acquisition_rows[-1]
                if g['stamp']==old['stamp']:return False
                if (g['epoch']!=old['epoch'] or not 0<g['stamp']-old['stamp']<=.4 or norm(sub(g['center'],old['center']))>.06):
                    self.acquisition_rows=[]
            self.acquisition_rows.append(g)
            self.acquisition_rows=self.acquisition_rows[-5:]
            if len(self.acquisition_rows)<3 or g['stamp']-self.acquisition_rows[0]['stamp']<.35:
                self.live_reason='confirming_airborne_geometry';return False
            supplied=dict(self.c,window_center_local_m=g['center'],window_yaw_rad=g['yaw'],
                          geometry_verified=True,opening_geometry_verified=True)
            errors=flight_blockers(supplied,self.alignment_commissioning)
            if errors:raise ValueError(';'.join(errors))
            # Geometry flags describe this actual runtime acquisition only.
            # A separate RGB-D pipeline acceptance is required by preflight.
            self.c=supplied;self.blockers=[]
            self.live_bound=True;self.live_epoch=g['epoch']
            self.live_reason='airborne_metric_geometry_acquired'
            return True
        except (ValueError,TypeError,KeyError) as exc:
            self.acquisition_rows=[];self.live_reason=str(exc);return False

    def live_geometry_current(self,ros,wall):
        try:
            g=validate_metric(self.metric_input or {},self.c,ros,wall,self.metric_received,
                              self.acquisition_after,self.live_epoch)
            if norm(sub(g['center'],self.c['window_center_local_m']))>self.c['center_agreement_m']:
                raise ValueError('window_geometry_moved')
            if abs(wrap(g['yaw']-self.c['window_yaw_rad']))>self.c['yaw_limit_rad']:
                raise ValueError('window_normal_changed')
            return True
        except (ValueError,TypeError,KeyError) as exc:
            self.live_reason=str(exc);return False

    def measurement(self):
        if not self.live_bound:raise ValueError('waiting_airborne_rgbd_geometry')
        return super().measurement()

    def inside(self,p):
        if self.alignment_only:
            from envelope_fence import envelope_inside
            return self.pose is not None and envelope_inside(self.c,p,self.pose.q)
        return super().inside(p)

    def request_pass(self,*args):
        if self.alignment_commissioning:return False,'alignment_commissioning_cannot_commit_pass'
        if self.alignment_only:return False,'alignment_only_cannot_commit_pass'
        return super().request_pass(*args)
