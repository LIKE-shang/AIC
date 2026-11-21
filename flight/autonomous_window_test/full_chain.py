"""One supervisor: frozen window pass, stop, station, then one USB exposure.
No ROS IO and no second arm/takeoff. Station offsets require operator-approved estimated or measured evidence.
"""
import math
from window_pass.geometry import norm,sub,dot

SCOPE='window_pass_single_target'
DEFAULTS=dict(empirical_aim_trim_forward_left_m=None,fixed_aim_compensation_forward_left_m=None,enabled=False,station_reference='route_endpoint',station_offset_forward_left_m=None,station_relation_confirmed=False,
              station_evidence='',station_area_confirmed=False,max_station_transfer_m=1.,
              transfer_speed_mps=.08,vertical_speed_mps=.10,settle_s=1.,timeout_s=25.,
              minimum_aim_bottom_m=.8,height_tolerance_m=.04)

def settings(supplied=None):
    if supplied is None:return None
    if not isinstance(supplied,dict) or set(supplied)-set(DEFAULTS):raise ValueError('invalid full_chain settings')
    p=dict(DEFAULTS,**supplied)
    if type(p['enabled']) is not bool:raise ValueError('full_chain enable must be boolean')
    if not p['enabled']:return None
    for key in ('station_relation_confirmed','station_area_confirmed'):
        if p[key] is not True:raise ValueError('full_chain current approved station/area confirmation required:'+key)
    if p['station_reference'] not in ('route_endpoint','takeoff_body'):raise ValueError('invalid station reference')
    trim=p['empirical_aim_trim_forward_left_m']
    if trim is not None:
        if p['fixed_aim_compensation_forward_left_m'] is None or not isinstance(trim,list) or len(trim)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in trim) or norm(trim)>.15:raise ValueError('empirical fixed aim trim requires mechanical compensation and <=15cm forward/left')
    compensation=p['fixed_aim_compensation_forward_left_m']
    if compensation is not None:
        if not isinstance(compensation,list) or len(compensation)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in compensation) or norm(compensation)>.10:raise ValueError('fixed aim compensation must be <=10cm body forward/left')
    v=p['station_offset_forward_left_m']
    if not isinstance(v,list) or len(v)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in v):
        raise ValueError('confirmed target station forward/left metres required')
    if not isinstance(p['station_evidence'],str) or not p['station_evidence'].strip():raise ValueError('station evidence required')
    for key in set(DEFAULTS)-{'empirical_aim_trim_forward_left_m','fixed_aim_compensation_forward_left_m','enabled','station_offset_forward_left_m','station_relation_confirmed','station_area_confirmed','station_evidence','station_reference'}:
        if type(p[key]) not in (int,float) or not math.isfinite(p[key]) or p[key]<=0:raise ValueError('invalid full_chain number:'+key)
    if p['max_station_transfer_m']>1.:raise ValueError('single station transfer exceeds one metre scope')
    if norm(v)>(p['max_station_transfer_m'] if p['station_reference']=='route_endpoint' else 4.):raise ValueError('station reference offset exceeds scope')
    if p['transfer_speed_mps']>.08 or p['vertical_speed_mps']>.10 or not 1<=p['settle_s']<=3 or p['timeout_s']>30:
        raise ValueError('postpass speed/time scope exceeded')
    if not .8<=p['minimum_aim_bottom_m']<.9 or not .02<=p['height_tolerance_m']<=.04:raise ValueError('aim height scope invalid')
    return p

class Postpass:
    def __init__(self,p):
        self.p=settings(p);self.stage='WAIT_PASS';self.station=None;self.target_center=None;self.hold_complete=False
        from pass_terminal import MotionWindow
        self.motion=MotionWindow();self.stop_anchor=None
        self.aim_since=None;self.arrival_since=None;self.started=None;self.target_acquired=False;self.aim_converged=False

    def clear_of_window(self,m,p):
        # The frozen window plane and conservative body support enforce full-body
        # separation, including tilted/yawed attitudes, during station motion.
        plan=m.direct_plan
        return dot(sub(p,plan['geometry_center']),plan['window_normal_local_m'])>=plan['minimum_center_exit_signed_distance_m']-1e-9

    def allowed(self,m,p):
        from attitude_envelope import enclosed
        return m.corridor(p) and enclosed(m.core.c,p,m.heading) and self.clear_of_window(m,p)

    def begin(self,m,i,departure=None):
        if not m.direct_pass_completed:raise ValueError('postpass_before_full_body_stop')
        endpoint=departure if departure is not None else (m.terminal_capture.stop_position if getattr(m,'terminal_capture',None) and m.terminal_capture.stop_position is not None else m.direct_plan['endpoint'])
        self.stop_anchor=tuple(endpoint)
        a,b=self.p['station_offset_forward_left_m']
        base=m.origin if self.p['station_reference']=='takeoff_body' else endpoint
        heading=m.takeoff_heading if self.p['station_reference']=='takeoff_body' else m.heading
        co,si=math.cos(heading),math.sin(heading)
        # Keep the original ground_z and target_z; current airborne z is never a ground reference.
        self.station=(base[0]+co*a-si*b,base[1]+si*a+co*b,m.target_z)
        self.target_center=self.station
        if self.p['fixed_aim_compensation_forward_left_m'] is not None:
            from fixed_aim import shift
            self.station=shift(self.target_center,m.heading,self.p['fixed_aim_compensation_forward_left_m'])
            if self.p['empirical_aim_trim_forward_left_m'] is not None:
                self.station=shift(self.station,m.heading,self.p['empirical_aim_trim_forward_left_m'])
        calibration=getattr(m,'visual_calibration',None)
        if calibration:
            self.station=calibration.station(m,self.station,calibration.target_id(m))
        if norm(sub(self.station,endpoint)[:2])>self.p['max_station_transfer_m']:
            raise ValueError('actual_postpass_station_transfer_exceeds_one_metre_scope')
        # Convex fence + forward halfspace enclose both segments and the 15cm aiming disk.
        radius=m.hover_target.p['max_correction_m']
        for base in (m.core.pose.p,tuple(endpoint),(endpoint[0],endpoint[1],m.target_z),self.station):
            if not self.allowed(m,base):raise ValueError('postpass_station_or_exit_outside_confirmed_region')
        for dx in (-radius,radius):
            for dy in (-radius,radius):
                if not self.allowed(m,(self.station[0]+dx,self.station[1]+dy,self.station[2])):
                    raise ValueError('postpass_aim_column_outside_confirmed_region')
        self.started=i.wall;self.stage='HEIGHT_HOLD';m.position=tuple(endpoint)
        m.transition('POSTPASS','full_body_clear_and_stop_confirmed_same_owner',i.wall)

    def tick(self,m,i,dt):
        p=self.p;c=m.core
        self.motion.update(c.pose.stamp,c.pose.p)
        if i.wall-self.started>p['timeout_s']:return m.handover('postpass_hold_or_station_timeout',i)
        if not self.allowed(m,c.pose.p):return m.fail('postpass_actual_left_confirmed_region',i)
        if self.stage=='HEIGHT_HOLD':
            z=m.position[2]+max(-p['vertical_speed_mps']*dt,min(p['vertical_speed_mps']*dt,m.target_z-m.position[2]))
            z=max(c.pose.p[2]-c.c['max_setpoint_lead_m'],min(c.pose.p[2]+c.c['max_setpoint_lead_m'],z))
            candidate=(m.position[0],m.position[1],z)
            ready=(abs(c.pose.p[2]-m.target_z)<=p['height_tolerance_m'] and m.full_height_bounds(c.pose.p)[0]>=p['minimum_aim_bottom_m'])
            if ready:self.stage='STATION_TRANSFER';self.arrival_since=None
        else:
            delta=sub(self.station,m.position);length=norm(delta)
            step=min(length,p['transfer_speed_mps']*dt)
            candidate=tuple(m.position[k]+delta[k]*step/length for k in range(3)) if length>1e-9 else self.station
            if norm(sub(candidate,c.pose.p))>c.c['max_setpoint_lead_m']:candidate=m.position
        if not self.allowed(m,candidate):return m.fail('postpass_reference_left_confirmed_region',i)
        m.position=candidate
        ready=(self.stage=='STATION_TRANSFER' and norm(sub(c.pose.p,self.station))<=p['height_tolerance_m'] and
               m.full_height_bounds(c.pose.p)[0]>=p['minimum_aim_bottom_m'] and (self.motion.stable(.08,.05) if (m.hover_target.p.get('fixed_position_timed') or m.hover_target.p.get('visual_target_hold')) else self.motion.stable(c.c['stable_speed_mps']) if getattr(m,'terminal_capture',None) else c.pose_speed<=c.c['stable_speed_mps']))
        if ready:
            if self.arrival_since is None:self.arrival_since=i.wall
            if i.wall-self.arrival_since>=p['settle_s']:
                self.hold_complete=True;self.stage='SINGLE_TARGET';m.position=self.station;m.aim_anchor=self.station
                m.transition('HOVER','postpass_station_height_and_stop_confirmed',i.wall)
        else:self.arrival_since=None
        return m.output()

    def status(self,m):
        h=m.hover_target;off=h.off_confirmed
        return dict(enabled=True,scope=SCOPE,stage=self.stage,pass_complete=m.direct_pass_completed,
                    postpass_hold_complete=self.hold_complete,target_station_local_m=self.station,
                    empirical_aim_trim_forward_left_m=self.p['empirical_aim_trim_forward_left_m'],
                    target_center_estimate_local_m=self.target_center,fixed_aim_compensation_forward_left_m=self.p['fixed_aim_compensation_forward_left_m'],
                    stopped_anchor_local_m=self.stop_anchor,station_window_speed_mps=self.motion.speed,
                    station_reference=self.p['station_reference'],station_evidence=self.p['station_evidence'],
                    takeoff_origin_local_m=m.origin,aim_anchor_local_m=m.aim_anchor,
                    target_acquired=self.target_acquired,aim_converged=self.aim_converged,
                    laser_exposure_complete=h.exposure_completed,laser_off_confirmed=off,
                    handover_confirmed=m.phase=='PILOT_CONTROL',physical_hit_verified=False,
                    mission_complete=(m.phase=='PILOT_CONTROL' and m.direct_pass_completed and self.hold_complete and h.completed and h.exposure_completed and off),
                    precision_acceptance='not_verified',target_count=1)
