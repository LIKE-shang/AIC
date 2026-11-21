"""Bounded pose rejection evidence and ground-only clock stability checks.

Never edits timestamps, buffers poses, changes clock mappings or clears faults.
"""
from collections import deque
import math
from window_pass.geometry import vector,quaternion,norm,sub,qdistance


def finite(x):return type(x) in (int,float) and math.isfinite(x)


def clean(x):
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    if isinstance(x,float) and not math.isfinite(x):return None
    return x


class PoseTimeGuard:
    def __init__(self,control,enabled=False):
        self.c=control;self.enabled=enabled
        self.rows=deque(maxlen=160);self.sync=deque(maxlen=80)
        self.last_rejection=None;self.first_active_rejection=None
        self.setpoint=None

    def pose(self,p,ros,wall,prior,latest,accepted,phase,arrival=None):
        valid_time=finite(p.stamp) and finite(ros) and p.stamp>0 and ros>0
        age=ros-p.stamp if valid_time else None
        checks=dict(timestamp_finite_positive=valid_time,
                    future_timestamp=valid_time and age>=-self.c['future_tolerance_s'],
                    stale_timestamp=valid_time and age<=self.c['pose_timeout_s'],
                    timestamp_monotonic=valid_time and p.stamp>latest,
                    position_finite=vector(p.p),quaternion_valid=quaternion(p.q),
                    local_frame=not self.c['local_frame'] or p.frame==self.c['local_frame'],
                    body_frame=not self.c['body_frame'] or p.child==self.c['body_frame'])
        dt=step=speed=angle=None
        if prior is not None and valid_time and finite(prior.stamp):
            dt=p.stamp-prior.stamp;checks['source_dt_positive']=dt>0
            if vector(p.p) and vector(prior.p):
                step=norm(sub(p.p,prior.p));speed=step/dt if dt>0 else None
                checks['position_step']=step<=self.c['max_pose_step_m']
                checks['pose_speed']=speed is not None and speed<=self.c['max_pose_speed_mps']
            if quaternion(p.q) and quaternion(prior.q):
                angle=qdistance(p.q,prior.q);checks['attitude_step']=angle<=self.c['max_attitude_step_rad']
        row=clean(dict(ros=ros,wall=wall,arrival=arrival,stamp=p.stamp,age_s=age,
                       position=list(p.p),quaternion=list(p.q),frame=p.frame,child=p.child,
                       previous_pose_stamp=prior.stamp if prior else None,previous_stream_stamp=latest,
                       source_dt_s=dt,step_m=step,speed_mps=speed,attitude_step_rad=angle,
                       accepted=bool(accepted),phase=phase,failed_checks=[k for k,v in checks.items() if not v]))
        if not accepted:
            self.last_rejection=row
            if phase not in (None,'IDLE','FAULT','DONE','PILOT_CONTROL','TAKEOVER','REFUSED') and self.first_active_rejection is None:
                self.first_active_rejection=dict(rejected=row,preceding=list(self.rows)[-12:])
        self.rows.append(row)

    def timesync(self,ros,wall,stamp,observed,estimated,rtt,caller):
        self.sync.append(clean(dict(ros=ros,wall=wall,stamp=stamp,observed_offset_ns=observed,
                                    estimated_offset_ns=estimated,round_trip_time_ms=rtt,caller=caller)))

    def ground(self,ros,wall):
        poses=[r for r in self.rows if 0<=wall-r['wall']<=3.]
        sync=[r for r in self.sync if 0<=wall-r['wall']<=3.]
        errors=[]
        if len(poses)<50 or poses[-1]['wall']-poses[0]['wall']<2.5:errors.append('insufficient_pose_window')
        ages=[r['age_s'] for r in poses if finite(r['age_s'])]
        # Five milliseconds of reserve inside the unchanged 20ms acceptance bound.
        future_limit=min(.015,self.c['future_tolerance_s'])
        if len(ages)!=len(poses) or any(not r['accepted'] for r in poses):errors.append('rejected_pose_in_window')
        if ages and min(ages)<-future_limit:errors.append('pose_future_headroom_insufficient')
        if not poses or not 0<=wall-poses[-1]['wall']<=self.c['pose_timeout_s'] or not finite(poses[-1]['stamp']) or not -future_limit<=ros-poses[-1]['stamp']<=self.c['pose_timeout_s']:
            errors.append('pose_stale_now')
        if len(sync)<20 or sync[-1]['wall']-sync[0]['wall']<2.5:errors.append('insufficient_timesync_window')
        valid=[r for r in sync if r['caller']=='/mavros' and all(finite(r[k]) for k in ('stamp','observed_offset_ns','estimated_offset_ns','round_trip_time_ms')) and r['round_trip_time_ms']>=0]
        if len(valid)!=len(sync):errors.append('invalid_or_untrusted_timesync')
        offsets=[r['estimated_offset_ns'] for r in valid]
        if offsets and max(offsets)-min(offsets)>5e6:errors.append('timesync_offset_unstable')
        if valid and max(r['round_trip_time_ms'] for r in valid)>=10.:errors.append('timesync_rtt_excessive')
        if not sync or not 0<=wall-sync[-1]['wall']<=.5 or not finite(sync[-1]['stamp']) or not -.02<=ros-sync[-1]['stamp']<=.5:errors.append('timesync_stale_now')
        return dict(enabled=self.enabled,ready=not errors,blockers=errors,pose_samples=len(poses),timesync_samples=len(sync),
                    pose_age_min_s=min(ages) if ages else None,pose_age_max_s=max(ages) if ages else None,
                    preflight_future_limit_s=future_limit,runtime_future_tolerance_s=self.c['future_tolerance_s'],
                    offset_span_ns=max(offsets)-min(offsets) if offsets else None,
                    scope='ground_only_observed_stability_not_proof_of_future_convergence')

    def evidence(self):
        return dict(last_pose=self.rows[-1] if self.rows else None,last_timesync=self.sync[-1] if self.sync else None,
                    last_rejection=self.last_rejection,first_active_rejection=self.first_active_rejection,
                    last_published_setpoint=self.setpoint)
