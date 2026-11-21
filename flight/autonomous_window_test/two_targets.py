"""Two selected drawing targets, sorted by ID; same supervisor and one takeoff."""
import copy,math
from full_chain import Postpass,settings as single_settings
from window_pass.geometry import norm,sub
from pass_terminal import MotionWindow

SCOPE='window_pass_two_targets'
AUTH_SCOPE='supervised_window_pass_two_targets_20261024'
# Drawing start(.85,3.85); held forward=drawing right, left=drawing up.
TARGETS={1:(3.15,-.05),2:(3.15,-1.35),3:(3.15,-2.65)}

def selection(value):
    if not isinstance(value,(list,tuple)) or len(value)!=2 or any(type(x) is not int or x not in TARGETS for x in value) or len(set(value))!=2:
        raise ValueError('select exactly two different target IDs from 1,2,3')
    return sorted(value)

def settings(value):
    if not isinstance(value,dict):raise ValueError('explicit two-target profile required')
    extra={'selected_target_ids','target_positions_forward_left_m'}
    from second_window_return import settings as return_settings
    journey=return_settings(value.get('second_window_return'))
    value={k:v for k,v in value.items() if k!='second_window_return'}
    if not extra<=set(value):raise ValueError('two-target selection and drawing positions required')
    ids=selection(value['selected_target_ids'])
    if value['selected_target_ids']!=ids:raise ValueError('bound targets must be ascending')
    expected={str(k):list(v) for k,v in TARGETS.items()}
    if value['target_positions_forward_left_m']!=expected:raise ValueError('drawing target catalog mismatch')
    p=single_settings({k:v for k,v in value.items() if k not in extra})
    if not p or p['station_reference']!='takeoff_body':raise ValueError('two-target station reference must remain takeoff body')
    if p['fixed_aim_compensation_forward_left_m'] is None:raise ValueError('fixed compensation required')
    return dict(p,selected_target_ids=ids,target_positions_forward_left_m=expected,**({'second_window_return':journey} if journey else {}))

class TwoTargetPostpass(Postpass):
    def __init__(self,value):
        pair=settings(value)
        self.return_policy=pair.get('second_window_return')
        from transfer_heading import TransferHeading
        self.transfer_heading=TransferHeading()
        super().__init__({k:v for k,v in pair.items() if k not in ('selected_target_ids','target_positions_forward_left_m','second_window_return')})
        self.scope=SCOPE;self.selected=pair['selected_target_ids'];self.index=0;self.results=[];self.stations=[];self.transfer_plan=None;self.transfer_started=None
        self.p.update(max_station_transfer_m=2.8,timeout_s=60.,settle_s=.2,height_tolerance_m=.08)
        self.p['station_offset_forward_left_m']=list(TARGETS[self.selected[0]])
        from mission_recovery import Progress
        self.progress=Progress();self.transfer_elapsed=0.
    def begin(self,m,i,departure=None):
        if m.return_journey and m.return_journey.leg==2:
            return m.return_journey.begin_home(m,i)
        # Validate both complete swept translation paths before first postpass move.
        from fixed_aim import shift
        centers=[];stations=[]
        for ident in self.selected:
            center=shift((m.origin[0],m.origin[1],m.target_z),m.takeoff_heading,TARGETS[ident])
            station=shift(center,m.heading,self.p['fixed_aim_compensation_forward_left_m'])
            if self.p['empirical_aim_trim_forward_left_m'] is not None:station=shift(station,m.heading,self.p['empirical_aim_trim_forward_left_m'])
            calibration=getattr(m,'visual_calibration',None)
            if calibration:station=calibration.station(m,station,ident)
            for dx in (-.15,.15):
                for dy in (-.15,.15):
                    if not self.allowed(m,(station[0]+dx,station[1]+dy,station[2])):raise ValueError('selected_target_aim_region_outside_clear_fence:'+str(ident))
            centers.append(center);stations.append(station)
        if norm(sub(stations[1],stations[0]))>2.8:raise ValueError('two_target_transfer_too_long')
        self.stations=stations
        self.transfer_plan=None;self.transfer_started=None;m.station_velocity=None
        self.transfer_elapsed=0.;self.progress.reset()
        super().begin(m,i,departure=departure)
    def recover_transfer(self,m,i,reason):
        from mission_recovery import enabled
        if not enabled(m) or not self.allowed(m,m.core.pose.p) or not self.allowed(m,self.station):return False
        actual=tuple(m.core.pose.p)
        if norm(sub(self.station,actual))>self.p['max_station_transfer_m']:return False
        for n in range(9):
            if not self.allowed(m,tuple(a+(b-a)*n/8. for a,b in zip(actual,self.station))):return False
        if not m.recovery.take(m,i,'station_transfer',reason):return False
        m.position=actual;m.station_velocity=None
        self.stage='HEIGHT_HOLD';self.transfer_plan=None;self.transfer_started=None
        self.transfer_elapsed=0.;self.progress.reset();self.arrival_since=None
        if 'timeout' in reason:self.started=i.wall
        m.reason='recovering_station_transfer:'+reason
        return True
    def tick(self,m,i,dt):
        if self.stage!='STATION_TRANSFER':
            result=super().tick(m,i,dt)
            # Complete the rate-limited height reference before horizontal feedforward.
            if self.stage=='STATION_TRANSFER' and abs(m.position[2]-m.target_z)>1e-6:self.stage='HEIGHT_HOLD'
            return result
        from smooth_pass import trajectory,sample
        from window_pass.geometry import add,scale,tilt
        c=m.core;self.motion.update(c.pose.stamp,c.pose.p)
        if i.wall-self.started>self.p['timeout_s']:
            if self.recover_transfer(m,i,'transfer_timeout'):return m.output()
            return m.handover('two_target_transfer_timeout',i)
        if not self.allowed(m,c.pose.p):return m.fail('two_target_transfer_actual_outside_region',i)
        if self.progress.stalled(norm(sub(self.station,c.pose.p)),i.wall):
            if self.recover_transfer(m,i,'transfer_no_progress'):return m.output()
        if self.transfer_plan is None:
            start=tuple(m.position);delta=sub(self.station,start);length=norm(delta)
            if abs(delta[2])>1e-6:return m.handover('two_target_transfer_requires_finished_height_reference',i)
            self.transfer_plan=dict(start=start,direction=tuple(x/length for x in delta) if length>1e-9 else (0.,0.,0.),
                trajectory=trajectory(length,1.0,.6,1.5) if self.index>0 and length>1e-9 else trajectory(length,.35,.4,1.2) if length>1e-9 else None)
            self.transfer_started=i.wall
            self.transfer_elapsed=0.
        plan=self.transfer_plan
        next_elapsed=self.transfer_elapsed+dt
        distance,speed,_=sample(plan['trajectory'],next_elapsed) if plan['trajectory'] else (0.,0.,0.)
        candidate=add(plan['start'],scale(plan['direction'],distance))
        # Keep a 10 cm reserve inside the existing 35 cm tracking fault bound.
        # Paused trajectory time never catches up by jumping to wall-clock time.
        if norm(sub(candidate,c.pose.p))>.25 and norm(sub(candidate,c.pose.p))>=norm(sub(m.position,c.pose.p)):
            lo,hi=self.transfer_elapsed,next_elapsed
            for _ in range(16):
                mid=(lo+hi)/2.
                d=sample(plan['trajectory'],mid)[0] if plan['trajectory'] else 0.
                p=add(plan['start'],scale(plan['direction'],d))
                if norm(sub(p,c.pose.p))<=.25:lo=mid
                else:hi=mid
            next_elapsed=lo
            distance=sample(plan['trajectory'],next_elapsed)[0] if plan['trajectory'] else 0.
            candidate=add(plan['start'],scale(plan['direction'],distance))
            speed=norm(sub(candidate,m.position))/dt if dt>0 else 0.
        self.transfer_elapsed=next_elapsed
        if not self.allowed(m,candidate):return m.fail('two_target_transfer_reference_outside_region',i)
        m.position=candidate;m.station_velocity=scale(plan['direction'],speed)
        done=plan['trajectory'] is None or self.transfer_elapsed>=plan['trajectory']['duration_s']
        ready=(done and norm(sub(c.pose.p,self.station)[:2])<=.12 and abs(c.pose.p[2]-self.station[2])<=.08 and
               m.full_height_bounds(c.pose.p)[0]>=self.p['minimum_aim_bottom_m'] and self.motion.stable(.20,.12) and tilt(c.pose.q)<=c.c['tilt_limit_rad'] and
               (not m.heading_error_gate_enabled() or not self.return_policy or self.index==0 or self.transfer_heading.ready(m)))
        if m.heading_error_gate_enabled() and self.return_policy and self.index==1 and done and not self.transfer_heading.ready(m):
            m.reason='inter_target_endpoint_hold_wait_original_heading_recovery'
        if ready:
            if self.arrival_since is None:self.arrival_since=i.wall
            if i.wall-self.arrival_since>=self.p['settle_s']:
                self.hold_complete=True;self.stage='SINGLE_TARGET';m.position=self.station;m.aim_anchor=self.station;m.station_velocity=None
                m.transition('HOVER','selected_target_station_height_and_stop_confirmed',i.wall)
        else:self.arrival_since=None
        return m.output()
    def finish_target(self,m,i):
        h=m.hover_target
        if not (h.completed and h.exposure_completed and h.off_confirmed):return m.handover('two_target_incomplete_no_next_target',i)
        self.results.append(dict(target_id=self.selected[self.index],exposure_completed=True,laser_off_confirmed=True,
                                 completion_basis=h.completion_basis,observed_hit_frames=h.observed_hit_frames,
                                 visual_start_basis=h.visual_start_basis,visual_evidence=h.aim_evidence.status(),physical_hit_verified=False))
        if self.index==1:
            if m.return_journey:return m.return_journey.begin(m,i)
            return m.handover('selected_two_targets_complete_laser_off_confirmed',i)
        self.index+=1;self.p['station_offset_forward_left_m']=list(TARGETS[self.selected[self.index]])
        self.hold_complete=False;self.target_acquired=False;self.aim_converged=False;self.aim_since=None;self.arrival_since=None
        self.motion=MotionWindow();m.aim_anchor=None
        from hover_target import TargetHold
        next_hold=TargetHold(copy.deepcopy(h.p));next_hold.scope=SCOPE
        # Request IDs never reset within one supervisor/bridge episode.
        next_hold.request_id=h.request_id+1;m.hover_target=next_hold
        try:self.begin(m,i,departure=tuple(m.core.pose.p))
        except ValueError as exc:return m.handover(str(exc),i)
        return m.output()
    def status(self,m):
        d=super().status(m)
        d.update(scope=SCOPE,target_count=2,selected_target_ids=list(self.selected),target_index=self.index,
                 active_target_id=self.selected[self.index],completed_target_ids=[r['target_id'] for r in self.results],
                 target_results=copy.deepcopy(self.results),all_target_stations_local_m=self.stations,
                 station_transfer_trajectory=self.transfer_plan,inter_target_max_speed_mps=1.,inter_target_acceleration_mps2=.6,inter_target_jerk_mps3=1.5,
                 station_trajectory_elapsed_s=self.transfer_elapsed,station_reference_lead_limit_m=.25,
                 approximate_arrival_horizontal_m=.12,approximate_arrival_vertical_m=.08,arrival_net_speed_mps=.20,arrival_settle_s=.2,first_station_max_speed_mps=.35,
                 mission_complete=(m.phase=='PILOT_CONTROL' and len(self.results)==2 and m.hover_target.off_confirmed),
                 operator_reported_target1_near_center=True,second_target_accuracy_verified=False)
        d['transfer_heading_recovery']=dict(self.transfer_heading.status(),enabled=m.heading_error_gate_enabled())
        if m.return_journey:
            d['second_window_return']=m.return_journey.status()
            d['mission_complete']=m.return_journey.complete and m.phase=='DONE'
        return d
