"""Same-owner second opening and automatic landing illumination. Pure logic, no IO.
Drawing regions identify distinct openings; live geometry alone authorizes passage.
"""
import copy, math
from window_pass.geometry import sub, norm, dot, yaw, wrap
from pass_terminal import MotionWindow
from attitude_envelope import enclosed, support
from envelope_fence import envelope_inside,envelope_margins
from fixed_aim import shift
from hover_target import fresh_bridge

AUTH='supervised_second_window_return_auto_landing_20261026'
PHASES=('RETURN_PREP','YAW_TURN','RETURN_HOME','LANDING_LIGHT_ON','LANDING_HANDOVER','AUTO_LANDING','LANDING_LIGHT_OFF')
LANDING=('AUTO_LANDING','LANDING_LIGHT_OFF')
# Operator-authorized horizontal slack for the worst-case tilt cone only.
# This cone allowance is separate from the measured-attitude field clearance.
HOME_TILT_FENCE_SLACK_M=.08
# After the second body exit, retain 2 cm horizontal actual-body clearance.
# Vertical reserves, physical boundaries and opening geometry stay unchanged.
EXIT_HORIZONTAL_CLEARANCE_M=.02
# Opposite drift signs in recorded turns do not support a fixed compensation.
# Keep the drawing preparation point instead of consuming fence clearance.
PREPARATION_LEFT_BIAS_M=0.
SEARCH_SCAN_AMPLITUDE_RAD=math.radians(6.)
SEARCH_SCAN_RATE_RAD_S=math.radians(6.)
SEARCH_SCAN_LEAD_RAD=math.radians(2.)
DEFAULTS=dict(enabled=True,authorization=AUTH,drawing_evidence='original_dimensioned_field_drawing_20261017',
    preparation_forward_left_m=[3.15,-2.65],second_window_forward_range_m=[1.2,2.1],
    second_window_left_range_m=[-3.85,-1.95],preparation_speed_mps=.5,return_speed_mps=1.5,
    preparation_acceleration_mps2=.6,return_acceleration_mps2=1.,reference_lead_m=.35,
    yaw_rate_rad_s=math.radians(150),yaw_reference_lead_rad=math.radians(15),
    movement_tracking_error_abort_enabled=False,
    return_tracking_error_limit_m=.20,
    yaw_timeout_s=20.,return_timeout_s=85.,landing_light_timeout_s=45.,ack_timeout_s=2.)

def settings(value):
    if value is None:return None
    if not isinstance(value,dict) or set(value)!=set(DEFAULTS):raise ValueError('explicit complete second-window return policy required')
    if value!=DEFAULTS:raise ValueError('second-window drawing/manual-landing scope mismatch')
    return copy.deepcopy(value)

class Journey:
    def __init__(self,p):
        self.p=settings(p);self.active=False;self.leg=1;self.first=None;self.first_plan=None
        self.second_pass_complete=False;self.origin_return_stopped=False;self.complete=False
        self.started=None;self.active_elapsed=0.;self.last_account_wall=None
        self.motion=MotionWindow();self.arrival=None;self.target=None
        self.reference_speed=0.;self.velocity=(0.,0.,0.)
        self.scan_offset=0.;self.scan_direction=-1;self.scan_center=None
        self.request_on=False;self.request_id=0;self.on_ack=False;self.off_ack=False
        self.light_started=None;self.failure=None;self.handover_observed=False
        self.descent_observed=False
        from mission_recovery import Progress
        self.progress=Progress();self.return_extra_s=0.;self.turn_extra_s=0.
    def coordinates(self,m,p):
        d=sub(p,m.origin);co,si=math.cos(m.takeoff_heading),math.sin(m.takeoff_heading)
        return (co*d[0]+si*d[1],-si*d[0]+co*d[1])
    def heading_adjustment(self,m):
        return self.active and (m.phase=='YAW_TURN' or
            (self.leg==2 and m.phase in ('SEARCH','ADJUST','ALIGN')))
    def search_active(self,m):
        return bool(self.active and self.leg==2 and m.phase=='SEARCH' and
                    m.session_window and m.session_policy.get('field_bounded_alignment'))
    def scan(self,m,dt,ros):
        if not self.search_active(m):return
        if self.scan_center is None:self.scan_center=m.heading
        decision=m.session_window.decision
        # Give a currently feasible frame time to acquire the second vote.
        # A rejected frame clears latest_safe; stale votes cannot stop scanning.
        if decision.latest_safe and decision.last_stamp is not None and 0<=ros-decision.last_stamp<=.5:return
        desired=self.scan_offset+self.scan_direction*SEARCH_SCAN_RATE_RAD_S*dt
        desired=max(-SEARCH_SCAN_AMPLITUDE_RAD,min(SEARCH_SCAN_AMPLITUDE_RAD,desired))
        observed=wrap(yaw(m.core.pose.q)-self.scan_center)
        # Pause if FCU yaw lags, instead of accumulating a larger yaw command.
        delta=desired-self.scan_offset
        available=max(0.,SEARCH_SCAN_LEAD_RAD-self.scan_direction*(self.scan_offset-observed))
        self.scan_offset+=self.scan_direction*min(abs(delta),available)
        if abs(self.scan_offset)>=SEARCH_SCAN_AMPLITUDE_RAD-1e-9:self.scan_direction*=-1
        m.heading=wrap(self.scan_center+self.scan_offset)
    def exit_clearance_active(self,m):
        if not (self.active and self.leg==2 and m.session_window and
                '_session_cylinder_radius_m' in m.core.c):return False
        if m.phase=='RETURN_HOME':return self.second_pass_complete
        # Re-evaluate the frozen plane at the current actual pose; a latched
        # terminal.clear from an earlier pose cannot authorize this allowance.
        return bool(m.phase=='PASS' and m.direct_plan and m.full_chain and
                    m.full_chain.clear_of_window(m,m.core.pose.p))
    def field_inside(self,m,control,p):
        if not self.exit_clearance_active(m):return envelope_inside(control,p,m.core.pose.q)
        try:
            margins=envelope_margins(control,p,m.core.pose.q)
            release=control.get('_session_field_fence_margin_m',control['clearance_margin_m'])-EXIT_HORIZONTAL_CLEARANCE_M
            return all(v+(release if k<2 else 0.)>=0. for values in margins.values() for k,v in enumerate(values))
        except (ValueError,TypeError,KeyError):return False
    def region(self,m,p,side):
        # Full-body clearance from the physical divider, plus actual field fence.
        plane=self.first_plan['geometry_center'];n=self.first_plan['window_normal_local_m']
        axis=tuple(side*x for x in n)
        control=m.core.c
        home=(side==-1 and self.leg==2 and self.second_pass_complete and
              m.phase in ('PASS','RETURN_HOME') and '_session_cylinder_radius_m' in control)
        if not home and self.leg==2 and m.phase in ('SEARCH','ADJUST','PASS'):
            control=m.session_window.scoped_control(control)
        cone=control
        if (self.active and self.leg==1 and m.phase=='YAW_TURN' and
                '_session_cylinder_radius_m' in control):
            # The common attitude guard enforces the current turn limit (8 deg).
            # A future PASS allowance (15 deg) is not a permitted turn attitude.
            cone=dict(control)
            cone['_session_pass_tilt_limit_rad']=m.attitude_limit()
        if home:
            cone=dict(control)
            cone['geofence_min_m']=[v-(HOME_TILT_FENCE_SLACK_M if k<2 else 0.) for k,v in enumerate(control['geofence_min_m'])]
            cone['geofence_max_m']=[v+(HOME_TILT_FENCE_SLACK_M if k<2 else 0.) for k,v in enumerate(control['geofence_max_m'])]
        divider=dot(sub(p,plane),axis)-support(m.core.c,m.heading,tuple(-x for x in axis))-.09
        self.last_region_check=dict(position=list(p),side=side,
            actual_field_ok=self.field_inside(m,control,p),
            predicted_tilt_ok=enclosed(cone,p,m.heading),divider_margin_m=divider)
        return (self.last_region_check['actual_field_ok'] and
                self.last_region_check['predicted_tilt_ok'] and divider>=0.)
    def begin(self,m,i):
        if self.active:raise ValueError('return journey cannot restart')
        if len(m.full_chain.results)!=2 or not m.hover_target.off_confirmed:raise ValueError('return requires both target OFF acknowledgments')
        self.active=True;self.started=i.wall;self.active_elapsed=0.;self.last_account_wall=i.wall
        self.progress.reset()
        self.first=m.session_window;self.first_plan=copy.deepcopy(m.direct_plan)
        preparation=list(self.p['preparation_forward_left_m']);preparation[1]+=PREPARATION_LEFT_BIAS_M
        self.target=shift((m.origin[0],m.origin[1],m.target_z),m.takeoff_heading,preparation)
        if not all(self.region(m,p,1) for p in (m.core.pose.p,m.position,self.target)):
            return m.handover('second_window_preparation_outside_right_corridor',i)
        self.motion=MotionWindow();self.arrival=None;m.station_velocity=None
        # Start each movement at the fresh measured pose; retain origin/epoch/goal.
        m.position=tuple(m.core.pose.p);self.reference_speed=0.;self.velocity=(0.,0.,0.)
        m.transition('RETURN_PREP','two_targets_OFF_confirmed_to_lower_window_preparation',i.wall)
        return m.output()
    def filter_geometry(self,m,status):
        if self.leg!=2:return
        metric=status.get('metric_geometry') or {};center=metric.get('center_enu_m')
        if not isinstance(center,(list,tuple)) or len(center)!=3:raise ValueError('second_window_missing_geometry')
        a,b=self.coordinates(m,center);lo,hi=self.p['second_window_forward_range_m'];bottom,top=self.p['second_window_left_range_m']
        if not lo<=a<=hi or not bottom<=b<=top:raise ValueError('geometry_outside_drawing_lower_window_region')
        if norm(sub(center,self.first.snapshot['geometry']['center_enu_m']))<1.3:raise ValueError('first_window_is_not_second_window')
        if (metric.get('pose_epoch'),metric.get('calibration_epoch'))!=tuple(self.first.snapshot['epoch']):raise ValueError('second_window_geometry_epoch_changed')
    def begin_home(self,m,i):
        self.second_pass_complete=True;self.target=(m.origin[0],m.origin[1],m.target_z)
        self.progress.reset()
        self.last_account_wall=i.wall
        self.motion=MotionWindow();self.arrival=None;m.station_velocity=None
        # Start each movement at the fresh measured pose; retain origin/epoch/goal.
        m.position=tuple(m.core.pose.p);self.reference_speed=0.;self.velocity=(0.,0.,0.)
        if not all(self.region(m,p,-1) for p in (m.core.pose.p,m.position,self.target)):
            return m.handover('return_home_outside_left_corridor',i)
        m.transition('RETURN_HOME','second_window_full_body_clear_and_stop_to_frozen_origin',i.wall)
        return m.output()
    def stable(self,m,i,target):
        ready=(norm(sub(m.core.pose.p,target))<=.08 and self.motion.stable(.12,.08))
        if not ready:self.arrival=None;return False
        if self.arrival is None:self.arrival=i.wall
        return i.wall-self.arrival>=.5
    def move_speed(self,phase):
        return self.p['return_speed_mps'] if phase=='RETURN_HOME' else self.p['preparation_speed_mps']
    def recover_move(self,m,i,reason):
        side=-1 if m.phase=='RETURN_HOME' else 1
        if not self.region(m,m.core.pose.p,side) or not self.region(m,self.target,side):return False
        if not m.recovery.take(m,i,'return_move',reason):return False
        m.position=tuple(m.core.pose.p);self.reference_speed=0.;self.velocity=(0.,0.,0.)
        self.motion=MotionWindow();self.arrival=None;self.progress.reset()
        m.reason='recovering_return_reference:'+reason
        return True
    def move(self,m,i,dt,side):
        if not self.region(m,m.core.pose.p,side):return m.fail('return_actual_outside_stage_corridor',i)
        if self.progress.stalled(norm(sub(self.target,m.core.pose.p)),i.wall):
            if self.recover_move(m,i,'no_progress'):return m.output()
        delta=sub(self.target,m.position);length=norm(delta)
        acceleration=self.p['return_acceleration_mps2'] if m.phase=='RETURN_HOME' else self.p['preparation_acceleration_mps2']
        speed=min(self.move_speed(m.phase),self.reference_speed+acceleration*dt,math.sqrt(2*acceleration*length))
        step=min(length,speed*dt)
        direction=tuple(x/length for x in delta) if length>1e-9 else (0.,0.,0.)
        # Limit reference lead without faulting a lagging vehicle. Stage corridor,
        # height, localization and finite-time checks still supervise real motion.
        offset=sub(m.position,m.core.pose.p);along=dot(offset,direction)
        transverse2=max(0.,dot(offset,offset)-along*along)
        available=max(0.,math.sqrt(max(0.,self.p['reference_lead_m']**2-transverse2))-along) if transverse2<=self.p['reference_lead_m']**2 else 0.
        step=min(step,available)
        self.reference_speed=step/dt if dt>0 else 0.
        self.velocity=tuple(self.reference_speed*x if k<2 else 0. for k,x in enumerate(direction)) if step<length else (0.,0.,0.)
        candidate=tuple(m.position[k]+delta[k]*step/length for k in range(3)) if length>1e-9 else self.target
        if not self.region(m,candidate,side):return m.fail('return_reference_outside_stage_corridor',i)
        m.position=candidate
        if norm(sub(m.position,self.target))>1e-6 or not self.stable(m,i,self.target):return m.output()
        if side==1:
            self.reference_speed=0.;self.velocity=(0.,0.,0.)
            self.turn_start=m.heading;self.turn_goal=self.turn_start+math.pi;self.turn_progress=0.
            self.motion=MotionWindow();self.arrival=None
            self.progress.reset();self.turn_extra_s=0.
            m.transition('YAW_TURN','preparation_position_height_and_stop_confirmed',i.wall)
        else:
            self.origin_return_stopped=True;self.light_started=i.wall;self.request_id=m.hover_target.request_id+1
            self.request_on=True;m.hover_target.request_id=self.request_id
            m.transition('LANDING_LIGHT_ON','frozen_takeoff_origin_stop_confirmed_wait_light_ACK',i.wall)
        return m.output()
    def tick(self,m,i,dt):
        c=m.core;self.motion.update(c.pose.stamp,c.pose.p)
        if self.last_account_wall is not None:
            self.active_elapsed += max(0.,i.wall-self.last_account_wall)
        self.last_account_wall=i.wall
        if m.phase in ('RETURN_PREP','YAW_TURN','RETURN_HOME') and self.active_elapsed>self.p['return_timeout_s']+self.return_extra_s:
            if m.recovery.take(m,i,'return_budget','return_active_timeout',maximum=1):self.return_extra_s+=20.
            else:return m.handover('second_window_return_timeout',i)
        if m.phase=='RETURN_PREP':return self.move(m,i,dt,1)
        if m.phase=='RETURN_HOME':return self.move(m,i,dt,-1)
        if m.phase=='YAW_TURN':
            if not self.region(m,c.pose.p,1):return m.fail('yaw_turn_outside_right_corridor',i)
            if i.wall-m.entered>self.p['yaw_timeout_s']+self.turn_extra_s:
                if not m.recovery.take(m,i,'turn','yaw_feedback_timeout'):return m.handover('yaw180_feedback_timeout',i)
                self.turn_extra_s+=self.p['yaw_timeout_s']
                m.position=tuple(c.pose.p)
                m.reason='recovering_remaining_turn_at_reduced_rate'
            # Bound reference lead by measured yaw; a slow response must not
            # accumulate an open-loop 180-degree command ahead of the vehicle.
            observed=self.turn_progress+wrap(yaw(c.pose.q)-m.heading)
            self.turn_progress=max(self.turn_progress,min(math.pi,
                self.turn_progress+(min(self.p['yaw_rate_rad_s'],math.radians(45)) if self.turn_extra_s else self.p['yaw_rate_rad_s'])*dt,observed+self.p['yaw_reference_lead_rad']))
            m.heading=wrap(self.turn_start+self.turn_progress)
            heading_ready=self.turn_progress>=math.pi and abs(wrap(yaw(c.pose.q)-self.turn_goal))<=c.c['yaw_limit_rad']
            if not heading_ready:self.arrival=None
            # Turn drift is recovered from fresh second-opening geometry. Wait
            # for actual motion to settle, without requiring the old hold point.
            if heading_ready and self.stable(m,i,c.pose.p):
                from session_window import Acquisition
                from pass_terminal import TerminalCapture
                # Rebase only the outgoing hold reference, never origin, source
                # or epoch. SEARCH/ADJUST starts at the measured settled pose.
                m.position=tuple(c.pose.p)
                self.leg=2;m.session_window=Acquisition(copy.deepcopy(self.first.identity),m.session_policy['fast_confirmation'],
                    tracking_reserve_m=.01,clearance_relaxation_m=.01,
                    maximize_minimum_clearance=True,horizontal_cone_slack_m=.02)
                m.direct_plan=None;m.direct_pass_completed=False;m.terminal_capture=TerminalCapture();m.pass_velocity=None
                m.adjust_plan=None;m.adjust_completed=False;m.direct_arrival_since=None
                c.metric_input=None;c.metric_received=-1e9
                self.scan_center=m.heading;self.scan_offset=0.;self.scan_direction=-1
                m.search_started_ros=i.ros;self.last_account_wall=i.wall
                m.transition('SEARCH','yaw180_feedback_confirmed_fresh_lower_window_session',i.wall)
            return m.output()
        return self.landing_tick(m,i)
    def light_status(self,m):
        return dict(enabled=True,authorization=AUTH,request_on=self.request_on and m.phase in ('LANDING_LIGHT_ON','LANDING_HANDOVER','AUTO_LANDING'),
            request_id=self.request_id,second_pass_complete=self.second_pass_complete,origin_return_stopped=self.origin_return_stopped,
            manual_landing=False,automatic_landing=True,maximum_on_s=self.p['landing_light_timeout_s'],on_acknowledged=self.on_ack,
            off_acknowledged=self.off_ack,hardware_light_verified=False)
    def stop_light(self,m,i,reason):
        self.request_on=False;self.request_id+=1;m.hover_target.request_id=self.request_id
        if reason:self.failure=reason
        m.transition('LANDING_LIGHT_OFF',reason or 'landed_and_disarmed_wait_OFF_ACK',i.wall)
        return m.output()
    def landing_tick(self,m,i):
        c=m.core
        b=fresh_bridge(i.hover_laser,i.wall,'window_pass_two_targets',allow_fault_off=True)
        if m.phase=='LANDING_LIGHT_OFF':
            if b and b.get('request_id')==self.request_id and b.get('state')=='OFF_ACKNOWLEDGED' and b.get('requested_on') is False:
                self.off_ack=True;self.complete=not self.failure and i.landed and not c.armed
                m.transition('DONE' if self.complete else 'FAULT',self.failure or 'automatic_landing_disarm_and_OFF_ACK_confirmed',i.wall)
            elif i.wall-m.entered>self.p['ack_timeout_s']+getattr(self,'landing_off_extra_s',0.):
                if (not getattr(self,'landing_off_extra_s',0.) and
                        i.wall-m.began<m.o['max_mission_time_s']):
                    # OFF remains the same idempotent request; ON is never retried.
                    self.landing_off_extra_s=self.p['ack_timeout_s']
                    m.reason='retry_wait_same_landing_OFF_request'
                    return m.output()
                self.failure=self.failure or 'landing_OFF_ACK_timeout_hardware_unknown';m.transition('FAULT',self.failure,i.wall)
            return m.output()
        if i.wall-m.began>=m.o['max_mission_time_s']:return self.stop_light(m,i,'whole_mission_illumination_deadline')
        if i.wall-self.light_started>=self.p['landing_light_timeout_s']:return self.stop_light(m,i,'automatic_landing_illumination_timeout')
        if i.landed and not c.armed:return self.stop_light(m,i,None)
        if b and b.get('fault_latched'):return self.stop_light(m,i,'landing_laser_bridge_fault')
        on=bool(b and b.get('request_id')==self.request_id and b.get('state')=='ON_ACKNOWLEDGED' and b.get('requested_on') is True)
        if m.phase=='LANDING_LIGHT_ON':
            if on:
                self.on_ack=True;m.transition('LANDING_HANDOVER','landing_light_ACK_wait_observed_AUTO_LAND',i.wall)
                return m.output('LAND')
            if i.wall-m.entered>self.p['ack_timeout_s']:return self.stop_light(m,i,'landing_ON_ACK_timeout')
        elif m.phase=='LANDING_HANDOVER':
            if not on:return self.stop_light(m,i,'landing_light_continuity_lost')
            if c.mode=='AUTO.LAND':
                self.handover_observed=True;c.authorized=False
                m.transition('AUTO_LANDING','AUTO_LAND_observed_illumination_active',i.wall)
            elif i.wall-m.entered>m.o['mode_timeout_s']:return self.stop_light(m,i,'landing_AUTO_LAND_not_observed')
        elif m.phase=='AUTO_LANDING' and not on:return self.stop_light(m,i,'landing_light_continuity_lost')
        return m.output()
    def handover_descent(self,m,i):
        # Only our already-issued landing handover can accept LANDING before
        # mode feedback. Never reinterpret LANDING as airborne during flight.
        c=m.core
        return bool(m.phase=='LANDING_HANDOVER' and self.request_on and self.on_ack and
                    c.mode in ('OFFBOARD','AUTO.LAND') and
                    (c.armed or (c.mode=='AUTO.LAND' and i.landed)) and
                    (c.mode=='AUTO.LAND' or i.landed_state==4))
    def landing_control_tick(self,m,i):
        # PX4 owns descent after AUTO.LAND. Identity/epoch/FCU/RC stay supervised;
        # the normal flight floor no longer applies during confirmed descent.
        c=m.core
        if not i.state_fresh or not i.landed_fresh or not c.connected:return m.fail('automatic_landing_state_unavailable',i)
        if i.emergency or not i.rc_ok:return m.fail('automatic_landing_RC_or_emergency',i)
        if c.mode!='AUTO.LAND' and not self.handover_descent(m,i):return m.fail('automatic_landing_mode_changed',i)
        if not i.graph_ok or not i.estimator_ok:return m.fail('automatic_landing_input_untrusted',i)
        from session_window import localization_identity
        if localization_identity(i.localization_identity)!=m.session_window.identity or i.localization_epoch!=m.localization_epoch:return m.fail('automatic_landing_localization_changed',i)
        if not c.fresh('pose',i.ros,i.wall,c.c['pose_timeout_s']):return m.fail('automatic_landing_pose_stale',i)
        return self.landing_tick(m,i)
    def status(self):
        return dict(enabled=True,active=self.active,window_leg=self.leg,second_pass_complete=self.second_pass_complete,
            last_corridor_check=copy.deepcopy(getattr(self,'last_region_check',None)),
            home_tilt_fence_slack_m=HOME_TILT_FENCE_SLACK_M,
            second_exit_horizontal_clearance_m=EXIT_HORIZONTAL_CLEARANCE_M,
            preparation_left_bias_m=PREPARATION_LEFT_BIAS_M,
            second_search_scan_amplitude_rad=SEARCH_SCAN_AMPLITUDE_RAD,
            second_search_scan_rate_rad_s=SEARCH_SCAN_RATE_RAD_S,
            second_search_local_timeout_enabled=False,
            origin_return_stopped=self.origin_return_stopped,return_active_elapsed_s=self.active_elapsed,
            return_timeout_s=self.p['return_timeout_s'],movement_tracking_error_abort_enabled=self.p['movement_tracking_error_abort_enabled'],
            return_recovery_extra_s=self.return_extra_s,turn_recovery_extra_s=self.turn_extra_s,
            preparation_speed_mps=self.p['preparation_speed_mps'],return_speed_mps=self.p['return_speed_mps'],yaw_rate_rad_s=self.p['yaw_rate_rad_s'],
            yaw_reference_lead_rad=self.p['yaw_reference_lead_rad'],reference_speed_mps=self.reference_speed,reference_lead_m=self.p['reference_lead_m'],automatic_landing_handover_observed=self.handover_observed,
            automatic_landing_descent_observed=self.descent_observed,
            mission_complete=self.complete,illumination_failure=self.failure,first_session_evidence={k:copy.deepcopy(self.first.snapshot[k]) for k in ('identity','epoch','captured_ros','after_stamp')} if self.first else None,
            drawing_opening='independent_lower_window',precision_acceptance='not_verified')
