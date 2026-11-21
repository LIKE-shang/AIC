"""Pure validation for ground-return -> the existing single flight supervisor."""
import math

def chosen(values):
    if len(values)!=2 or any(type(x) is not int or x not in (1,2,3) for x in values) or len(set(values))!=2:
        raise ValueError('choose exactly two different targets from 1,2,3')
    return sorted(values)

def ground_done(d):
    p=d.get('pose');v=d.get('command',{})
    return (d.get('state')=='DONE' and d.get('reason')=='ALL_WAYPOINTS_REACHED' and
            d.get('waypoint_index')==4 and d.get('waypoint_count')==4 and
            isinstance(p,list) and len(p)==3 and all(type(x) in (int,float) and math.isfinite(x) for x in p) and
            math.hypot(*p[:2])<=.10 and abs(p[2])<=.10 and
            all(type(v.get(k)) in (int,float) and v[k]==0 for k in ('vx','vy','wz')) and not d.get('dry_run',False))

def same_start(saved,current):
    if saved['identity']!=current['identity']:raise ValueError('boot_run_or_sources_changed_reprepare_required')
    p,q=saved['position_local_m'],current['position_local_m']
    if math.dist(p,q)>.10:raise ValueError('prepared_start_moved')

def validate_plan(d,boot,manifest,ground_hash):
    if d.get('schema') not in ('competition_20261025_v1','competition_20261101_aic234471') or d.get('selected_target_ids')!=chosen(d.get('selected_target_ids',[])):
        raise ValueError('invalid_prepared_competition')
    if d.get('boot_id')!=boot or d.get('flight_manifest_sha256')!=manifest or d.get('ground_config_sha256')!=ground_hash:
        raise ValueError('prepared_competition_stale')
    if d.get('ready') is not True:raise ValueError('competition_not_prepared')

class Handoff:
    """A stale/partial completion never authorizes the flight stage."""
    def __init__(self):self.seen_running=False;self.done=False
    def observe(self,d,age,source,expected):
        if self.done:raise ValueError('ground handoff already consumed')
        if source!=expected or not 0<=age<=.5:raise ValueError('ground status stale or source changed')
        state=d.get('state')
        if state=='ABORT':raise ValueError('ground stage aborted:'+str(d.get('reason','unknown'))+'; waypoint='+str(d.get('waypoint_index'))+'/'+str(d.get('waypoint_count'))+'; pose='+str(d.get('pose')))
        if state in ('RUNNING','DWELL'):self.seen_running=True
        if state=='DONE':
            if not self.seen_running or not ground_done(d):raise ValueError('ground done without real returned stop')
            self.done=True
        return self.done

def flight_failure_reason(terminal):
    for name in ('fault_snapshot','handover_snapshot'):
        row=terminal.get(name) or {}
        if row.get('reason'):return row['reason']
    return terminal.get('reason','flight_not_completed')

class ReturnHandoff:
    """Post-ground-only allowance; keep the original prestart 10cm gate intact."""
    horizontal_limit_m=.20
    vertical_limit_m=.10
    stationary_window_s=.5
    minimum_distinct_samples=4
    stationary_speed_mps=.08
    stationary_span_m=.04
    maximum_wait_s=3.
    def __init__(self,saved):
        from collections import deque
        self.saved=saved;self.rows=deque();self.reason='await_stationary_return_feedback';self.ready=False
        self.metrics={}
    def observe(self,current,ground,pose_stamp,wall):
        if self.ready:raise ValueError('return_handoff_already_confirmed')
        if current['identity']!=self.saved['identity']:raise ValueError('boot_run_or_sources_changed_reprepare_required')
        if not ground_done(ground):raise ValueError('return_handoff_requires_fresh_four_point_DONE_zero_command')
        p=current['position_local_m'];saved=self.saved['position_local_m']
        if not isinstance(p,(list,tuple)) or len(p)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in p):raise ValueError('invalid_return_handoff_pose')
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (pose_stamp,wall)):raise ValueError('invalid_return_handoff_time')
        delta=[p[k]-saved[k] for k in range(3)];xy=math.hypot(*delta[:2]);z=abs(delta[2])
        self.metrics=dict(saved_position_local_m=list(saved),actual_position_local_m=list(p),delta_local_m=delta,horizontal_error_m=xy,vertical_error_m=z,pose_stamp=pose_stamp)
        if xy>self.horizontal_limit_m or z>self.vertical_limit_m:
            self.rows.clear();self.reason='returned_horizontal_error_over20cm' if xy>self.horizontal_limit_m else 'returned_height_difference_over10cm';return False
        if self.rows and (pose_stamp<self.rows[-1][0] or wall<self.rows[-1][1]):raise ValueError('return_handoff_time_reversed')
        if self.rows and pose_stamp==self.rows[-1][0]:self.reason='await_new_return_pose_sample';return False
        self.rows.append((pose_stamp,wall,tuple(p)))
        while len(self.rows)>2 and self.rows[1][0]<=pose_stamp-self.stationary_window_s:self.rows.popleft()
        duration=pose_stamp-self.rows[0][0];wall_duration=wall-self.rows[0][1]
        span=max(math.dist(p,r[2]) for r in self.rows)
        speed=math.dist(p,self.rows[0][2])/duration if duration>0 else None
        self.metrics.update(stationary_pose_span_s=duration,stationary_wall_span_s=wall_duration,position_span_m=span,net_speed_mps=speed,distinct_samples=len(self.rows))
        self.ready=(duration>=self.stationary_window_s-1e-7 and wall_duration>=self.stationary_window_s-1e-7 and len(self.rows)>=self.minimum_distinct_samples and span<=self.stationary_span_m and speed<=self.stationary_speed_mps)
        self.reason='returned_within20cm_and_stationary_confirmed' if self.ready else 'await_stationary_return_feedback'
        return self.ready
    def status(self):
        return dict(self.metrics,ready=self.ready,reason=self.reason,horizontal_limit_m=self.horizontal_limit_m,vertical_limit_m=self.vertical_limit_m,stationary_window_s=self.stationary_window_s,minimum_distinct_samples=self.minimum_distinct_samples,stationary_speed_limit_mps=self.stationary_speed_mps,stationary_span_limit_m=self.stationary_span_m,maximum_wait_s=self.maximum_wait_s,prestart_limit_m=.10)
