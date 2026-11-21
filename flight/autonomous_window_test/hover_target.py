"""Single-target camera correction during an existing takeoff-only mission.
Pure Python; the parent Mission remains the sole source of flight setpoints.
"""
import math
from collections import deque
from window_pass.geometry import norm, sub, yaw

DEFAULTS=dict(paced_visual_servo=False,bounded_visual_aim=False,visual_aim_max_s=1.,timed_visual_aim=False,visual_target_hold=False,fixed_position_timed=False,require_stable_target_before_light=False,enabled=False,offset_x_px=53.25,offset_y_px=15.484375,
              camera_frame_id='circle_usb_optical_frame', image_width=640,image_height=360,
              inner_free_radius_m=.052,max_correction_m=.15,max_speed_mps=.04,
              observation_timeout_s=.35,acquisition_timeout_s=15.,
              settle_time_s=.6,exposure_s=3.,max_light_s=5.,ack_timeout_s=2.,
              centre_radius_ratio=.45,spot_error_limit_px=8.,
              trigger_policy='vision_confirmed',hover_light_delay_s=1.,
              visual_servo_enabled=False,servo_deadband_px=2.,servo_gain=.6,
              visual_calibration_mode='off')


def settings(supplied=None):
    if supplied is None: return None
    if not isinstance(supplied,dict) or set(supplied)-set(DEFAULTS):raise ValueError('invalid hover_target settings')
    p=dict(DEFAULTS,**supplied)
    if type(p['paced_visual_servo']) is not bool:raise ValueError('paced visual servo must be boolean')
    if p['paced_visual_servo'] and not (p['bounded_visual_aim'] and p['visual_servo_enabled']):
        raise ValueError('paced visual servo requires bounded observed-spot aiming')
    if type(p['bounded_visual_aim']) is not bool:raise ValueError('bounded visual aim must be boolean')
    if type(p['visual_aim_max_s']) not in (int,float) or not math.isfinite(p['visual_aim_max_s']) or not 1.<=p['visual_aim_max_s']<=3.:
        raise ValueError('visual aim budget must be one to three seconds')
    if p['bounded_visual_aim'] and not (p['timed_visual_aim'] and p['visual_target_hold']):
        raise ValueError('bounded visual aim requires timed visual target hold')
    if p['bounded_visual_aim'] and p['visual_aim_max_s']!=1.:
        raise ValueError('bounded visual aim requires the one-second laser bridge contract')
    if p['visual_calibration_mode'] not in ('off','learn','reuse'):raise ValueError('invalid visual calibration mode')
    if p['visual_calibration_mode']!='off' and (p['fixed_position_timed'] or not p['visual_servo_enabled'] or p['trigger_policy']!='hover_timed'):
        raise ValueError('visual calibration requires explicit observed-spot servo profile')
    if p['visual_calibration_mode']!='off' and p['exposure_s']!=3.:
        raise ValueError('visual calibration requires three continuous observed seconds')
    if type(p['visual_target_hold']) is not bool:raise ValueError('visual target hold must be boolean')
    if type(p['timed_visual_aim']) is not bool:raise ValueError('timed visual aim must be boolean')
    if p['timed_visual_aim'] and (not p['visual_target_hold'] or p['require_stable_target_before_light'] or p['hover_light_delay_s']!=1. or p['exposure_s']!=3.):
        raise ValueError('timed visual aim requires explicit one-second optional-vision/three-second exposure profile')
    if p['visual_target_hold'] and (p['fixed_position_timed'] or not p['visual_servo_enabled'] or (not p['require_stable_target_before_light'] and not p['timed_visual_aim']) or p['trigger_policy']!='hover_timed' or p['visual_calibration_mode']!='off'):
        raise ValueError('visual target hold requires explicit live visual servo profile')
    if type(p['fixed_position_timed']) is not bool:raise ValueError('fixed position timed switch must be boolean')
    if p['fixed_position_timed'] and (p['visual_servo_enabled'] or p['require_stable_target_before_light'] or p['trigger_policy']!='hover_timed'):raise ValueError('fixed position light requires optional vision and no visual retargeting')
    if type(p['require_stable_target_before_light']) is not bool:raise ValueError('stable target light switch must be boolean')
    if type(p['visual_servo_enabled']) is not bool:raise ValueError('servo enable must be boolean')
    if not .1<=p['servo_gain']<=1 or not .5<=p['servo_deadband_px']<=5:
        raise ValueError('invalid visual servo gain/deadband')
    if type(p['enabled']) is not bool:raise ValueError('hover target enabled must be boolean')
    if not p['enabled']:return None
    if p['trigger_policy'] not in ('vision_confirmed','hover_timed'):
        raise ValueError('invalid laser trigger policy')
    if not .5<=p['hover_light_delay_s']<=3.:raise ValueError('invalid hover light delay')
    if not isinstance(p['camera_frame_id'],str) or not p['camera_frame_id']:raise ValueError('camera frame required')
    for k,v in p.items():
        if k in ('paced_visual_servo','bounded_visual_aim','timed_visual_aim','visual_target_hold','fixed_position_timed','require_stable_target_before_light','enabled','visual_servo_enabled','camera_frame_id','trigger_policy','visual_calibration_mode'):continue
        if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('finite hover setting: '+k)
    for k in ('image_width','image_height'):
        if type(p[k]) is not int or p[k]<=0:raise ValueError('invalid dimensions')
    for k in set(p)-{'paced_visual_servo','bounded_visual_aim','timed_visual_aim','visual_target_hold','fixed_position_timed','require_stable_target_before_light','enabled','visual_servo_enabled','camera_frame_id','trigger_policy','offset_x_px','offset_y_px','image_width','image_height','visual_calibration_mode'}:
        if p[k]<=0:raise ValueError('positive hover setting: '+k)
    if p['max_correction_m']>.15 or p['max_speed_mps']>.04 or p['max_light_s']>5 or p['exposure_s']>=p['max_light_s']:
        raise ValueError('hover target scope exceeded')
    if p['observation_timeout_s']>.35 or p['acquisition_timeout_s']>15 or p['ack_timeout_s']>2:
        raise ValueError('hover target timeout scope exceeded')
    if not 0<p['centre_radius_ratio']<1:raise ValueError('invalid centring ratio')
    if (p['image_width'],p['image_height'])!=(640,360) or p['camera_frame_id']!='circle_usb_optical_frame':
        raise ValueError('this measured hover profile is 640x360 circle_usb_optical_frame only')
    if not (0<=(p['image_width']-1)/2+p['offset_x_px']<p['image_width'] and
            0<=(p['image_height']-1)/2+p['offset_y_px']<p['image_height']):
        raise ValueError('reference outside image')
    return p


def fresh_bridge(row,wall,scope='single_hover_target',allow_fault_off=False):
    if not isinstance(row,tuple) or len(row)!=2:return None
    received,data=row
    if type(received) not in (int,float) or not math.isfinite(received) or not 0<=wall-received<=.5 or not isinstance(data,dict):return None
    if data.get('scope')!=scope:return None
    if data.get('fault_latched') is not False and not (allow_fault_off and data.get('fault_latched') is True and data.get('state')=='OFF_ACKNOWLEDGED' and data.get('requested_on') is False):return None
    return data


class TargetHold:
    def __init__(self,p):
        self.scope='single_hover_target';self.off_confirmed=False
        self.p=settings(p)
        if self.p is None:raise ValueError('explicit hover target enable required')
        self.phase='WAIT_HOVER';self.reason='takeoff_in_progress';self.started=None
        self.request_on=False;self.request_id=0;self.on_request_wall=None
        self.on_ack_wall=None;self.on_ack_ros=None;self.off_request_wall=None;self.settled=None;self.hit_since=None
        self.last_stamp=None;self.error_px=None;self.max_error_px=0.;self.completed=False
        self.last_visual_stamp=None
        from pass_terminal import MotionWindow
        self.stable_motion=MotionWindow()
        self.last_frame_wall=None;self.motion_samples=deque();self.window_speed=None
        self.gates={};self.metrics={};self.correction_dt=0.
        self.last_image_stamp=None;self.observed_hit_frames=0
        self.exposure_completed=False;self.completion_basis=None
        self.stable_exposure_since=None;self.stable_exposure_s=0.
        self.correction_updates=0;self.aim_source=None;self.aim_error_px=None
        self.spot_offset_px=None;self.spot_offset_stamp=None
        self.servo_paused=False;self.servo_best_error=None;self.servo_worse_since=None
        self.servo_path_m=0.;self.servo_diagnostics={}
        self.calibration_candidate=None;self.calibration_samples=[];self.calibration_last_frame=None
        self.calibration_baseline_clear=False;self.calibration_recorded=False
        self.last_correction_reason='not_started' 
        from aim_evidence import AimEvidence
        self.aim_evidence=AimEvidence()
        from observed_servo_pacer import ObservedServoPacer
        self.servo_pacer=ObservedServoPacer()
        self.visual_start_basis=None

    def status(self):
        return dict(bounded_visual_aim=self.p['bounded_visual_aim'],visual_evidence=self.aim_evidence.status(),visual_start_basis=self.visual_start_basis,timed_visual_aim=self.p['timed_visual_aim'],visual_aim_budget_s=(self.p['visual_aim_max_s'] if self.p['bounded_visual_aim'] else self.p['hover_light_delay_s']) if self.p['timed_visual_aim'] else None,visual_target_hold=self.p['visual_target_hold'],scope=self.scope,fixed_position_timed=self.p['fixed_position_timed'],off_confirmed=self.off_confirmed,enabled=True,stage=self.phase,reason=self.reason,
                    visual_aim_budget_elapsed=bool(self.p['timed_visual_aim'] and self.on_request_wall is not None and self.started is not None and self.on_request_wall-self.started>=(self.p['visual_aim_max_s'] if self.p['bounded_visual_aim'] else self.p['hover_light_delay_s'])),
                    exposure_s=self.p['exposure_s'],request_on=self.request_on,request_id=self.request_id,
                    error_px=self.error_px,max_error_px=self.max_error_px,
                    capture_stamp=self.last_visual_stamp,completed=self.completed,
                    offset_px=[self.p['offset_x_px'],self.p['offset_y_px']],
                    image_size_px=[self.p['image_width'],self.p['image_height']],
                    camera_frame_id=self.p['camera_frame_id'],physical_hit_verified=False,
                    laser_on_ever_requested=self.on_request_wall is not None,
                    trigger_policy=self.p['trigger_policy'],exposure_completed=self.exposure_completed,
                    completion_basis=self.completion_basis,observed_hit_frames=self.observed_hit_frames,
                    stable_observed_exposure_s=self.stable_exposure_s,
                    visual_calibration_mode=self.p['visual_calibration_mode'],calibration_baseline_clear=self.calibration_baseline_clear,calibration_recorded=self.calibration_recorded,
                    calibration_candidate=self.calibration_candidate,
                    visual_servo_enabled=self.p['visual_servo_enabled'],correction_updates=self.correction_updates,
                    aim_source=self.aim_source,aim_error_px=self.aim_error_px,
                    correction_reason=self.last_correction_reason,servo_paused=self.servo_paused,
                    servo_path_m=self.servo_path_m,servo_diagnostics=dict(self.servo_diagnostics),
                    paced_visual_servo=self.p['paced_visual_servo'],servo_pacing=self.servo_pacer.status(),
                    gates=dict(self.gates),metrics=dict(self.metrics))

    def off(self,reason,wall):
        if self.phase in ('DONE','ABORT'):return
        self.request_on=False;self.request_id+=1;self.off_request_wall=wall
        self.reason=reason;self.phase='OFF_WAIT';self.settled=None

    def invalid(self,reason,wall):
        self.completed=False;self.off(reason,wall)

    def vision(self,row,ros,wall,search_start):
        if not isinstance(row,tuple) or len(row)!=2:raise ValueError('camera_observation_missing')
        received,d=row
        if type(received) not in (int,float) or not math.isfinite(received):raise ValueError('camera_receipt_invalid')
        if not 0<=wall-received<=self.p['observation_timeout_s'] or not isinstance(d,dict):raise ValueError('camera_status_stale')
        stamp=d.get('capture_stamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not -.05<=ros-stamp<=self.p['observation_timeout_s'] or stamp<=search_start:
            raise ValueError('camera_capture_stale_or_before_hover')
        if (d.get('valid') is not True or d.get('camera_frame_id')!=self.p['camera_frame_id'] or
            d.get('source_image_size_px')!=[self.p['image_width'],self.p['image_height']]):
            raise ValueError('camera_identity_or_target_invalid')
        reference=d.get('reference_point_px');expected=[(self.p['image_width']-1)/2+self.p['offset_x_px'],(self.p['image_height']-1)/2+self.p['offset_y_px']]
        if not isinstance(reference,list) or len(reference)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in reference) or norm(sub(reference,expected))>1e-6:
            raise ValueError('camera_reference_offset_mismatch')
        err=d.get('reference_error_px');axes=d.get('inner_semiaxes_px');angle=d.get('inner_angle_deg')
        for vector in (err,axes):
            if not isinstance(vector,list) or len(vector)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in vector):raise ValueError('invalid_image_geometry')
        if min(axes)<=0 or type(angle) not in (int,float) or not math.isfinite(angle):raise ValueError('invalid_image_ellipse')
        normal=d.get('reference_normalized_radius')
        if type(normal) not in (int,float) or not math.isfinite(normal) or normal<0:raise ValueError('invalid_image_radius')
        if (d.get('observed_basis')=='white_core_local_contrast' or
            d.get('candidate_search_policy')=='calibrated_neighbourhood') and not self.p['bounded_visual_aim']:
            # Legacy/calibration profiles do not have the new OFF-baseline and
            # multi-frame guards. Their spot feedback keeps its old meaning.
            d=dict(d,observed_unique=False,observed_candidates=[],observed_spot_px=None,
                   observed_error_px=None,observed_normalized_radius=None,observed_inside_inner=None)
        new=self.last_stamp is None or stamp>self.last_stamp
        if self.last_stamp is not None and stamp<self.last_stamp:raise ValueError('camera_stamp_reversed')
        self.last_stamp=stamp;self.last_visual_stamp=stamp;self.error_px=list(err)
        self.max_error_px=max(self.max_error_px,norm(err))
        return d,new

    def image_status(self,row,ros,wall):
        # A live image and a detected target are separate facts. A missing ring
        # must not turn a healthy camera into a failed camera in hover_timed.
        if not isinstance(row,tuple) or len(row)!=2:raise ValueError('camera_observation_missing')
        received,d=row
        if type(received) not in (int,float) or not math.isfinite(received) or not 0<=wall-received<=self.p['observation_timeout_s'] or not isinstance(d,dict):
            raise ValueError('camera_status_stale')
        stamp=d.get('capture_stamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not -.05<=ros-stamp<=self.p['observation_timeout_s'] or stamp<=self.hover_ros:
            raise ValueError('camera_capture_stale_or_before_hover')
        if d.get('camera_frame_id')!=self.p['camera_frame_id'] or d.get('source_image_size_px')!=[self.p['image_width'],self.p['image_height']]:
            raise ValueError('camera_identity_invalid')
        reference=d.get('reference_point_px')
        expected=[(self.p['image_width']-1)/2+self.p['offset_x_px'],(self.p['image_height']-1)/2+self.p['offset_y_px']]
        if not isinstance(reference,list) or len(reference)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in reference) or norm(sub(reference,expected))>1e-6:
            raise ValueError('camera_reference_offset_mismatch')
        if self.last_image_stamp is not None and stamp<self.last_image_stamp:raise ValueError('camera_stamp_reversed')
        new=self.last_image_stamp is None or stamp>self.last_image_stamp
        self.last_image_stamp=stamp;self.last_visual_stamp=stamp
        return d,new

    def tick_timed(self,mission,i,dt,bridge):
        p=self.p;c=mission.core
        try:image,new_image=self.image_status(i.hover_visual,i.ros,i.wall)
        except ValueError as exc:
            self.gates={'camera_live':False};self.metrics={'camera_error':str(exc)}
            self.last_frame_wall=None;self.settled=None
            if self.request_on or i.wall-self.started>p['acquisition_timeout_s']:self.invalid(str(exc),i.wall)
            else:self.reason=str(exc)
            return mission.output()
        detection=None;new_target=False
        try:detection,new_target=self.vision(i.hover_visual,i.ros,i.wall,self.hover_ros)
        except ValueError as exc:target_reason=str(exc)
        else:target_reason='valid_target'
        self.gates=dict(camera_live=True,hover_delay_elapsed=i.wall-self.started>=p['hover_light_delay_s'])
        self.metrics=dict(target_detected=detection is not None,target_reason=target_reason,
                          target_required_for_light=False,centering_elapsed_s=i.wall-self.started,
                          hover_light_delay_s=p['hover_light_delay_s'])
        servo_dt=dt
        if p['visual_servo_enabled'] and new_image:
            servo_dt=(min(p['observation_timeout_s'],max(0.,i.wall-self.last_frame_wall))
                      if self.last_frame_wall is not None else dt)
            self.last_frame_wall=i.wall
        if p['visual_servo_enabled'] and self.phase in ('CENTER','EXPOSE'):
            if detection is not None and new_target:
                observed=detection.get('observed_error_px')
                use_spot=(self.phase=='EXPOSE' and detection['capture_stamp']>self.on_ack_ros and
                          isinstance(observed,list) and len(observed)==2 and
                          all(type(v) in (int,float) and math.isfinite(v) for v in observed))
                if use_spot:
                    self.spot_offset_px=[observed[k]-detection['reference_error_px'][k] for k in range(2)]
                    self.spot_offset_stamp=detection['capture_stamp']
                recent_offset=(not use_spot and self.spot_offset_px is not None and
                               0<=detection['capture_stamp']-self.spot_offset_stamp<=.35)
                self.aim_source=('observed_spot_minus_target' if use_spot else
                                 'recent_spot_offset_minus_target' if recent_offset else 'predicted_reference_minus_target')
                self.aim_error_px=(list(observed) if use_spot else
                    [detection['reference_error_px'][k]+self.spot_offset_px[k] for k in range(2)] if recent_offset else
                    list(detection['reference_error_px']))
                self.correct_reference(mission,dict(detection,servo_error_px=self.aim_error_px),servo_dt,i.wall)
            elif detection is None and not self.servo_paused:self.last_correction_reason='target_unavailable_hold_position'
        chain=getattr(mission,'full_chain',None)
        if chain and new_image:
            if detection is not None:chain.target_acquired=True
            current_error=(detection.get('observed_error_px') if detection and self.phase=='EXPOSE' else None)
            if current_error is None and detection:current_error=detection.get('reference_error_px')
            centred=(new_target and isinstance(current_error,list) and norm(current_error)<=p['servo_deadband_px'] and c.pose_speed<=c.c['stable_speed_mps'])
            if centred:
                if chain.aim_since is None:chain.aim_since=detection['capture_stamp']
                chain.aim_converged=detection['capture_stamp']-chain.aim_since>=p['settle_time_s']
            else:chain.aim_since=None;chain.aim_converged=False
        light_ready=self.gates['hover_delay_elapsed']
        if p['require_stable_target_before_light']:
            self.stable_motion.update(c.pose.stamp,c.pose.p)
            aligned=(detection is not None and detection['reference_normalized_radius']<=p['centre_radius_ratio'] and not self.servo_paused)
            stopped=(self.stable_motion.stable(c.c['stable_speed_mps']) and norm(sub(mission.position,c.pose.p))<=.04)
            if aligned and stopped:
                if self.settled is None:self.settled=i.wall
            else:self.settled=None
            held=self.settled is not None and i.wall-self.settled>=p['settle_time_s']
            self.gates.update(target_aligned=aligned,position_stable=stopped,stable_target_held=held)
            self.metrics.update(target_required_for_light=True,net_speed_05s_mps=self.stable_motion.speed)
            light_ready=light_ready and held
            if self.phase=='CENTER' and i.wall-self.started>p['acquisition_timeout_s']:
                self.invalid('stable_target_not_acquired',i.wall);return mission.output()
        if self.phase=='CENTER':
            if not p['visual_servo_enabled'] and detection is not None and new_target and not self.gates['hover_delay_elapsed']:
                if self.last_frame_wall is not None:self.correction_dt=min(p['observation_timeout_s'],max(0.,i.wall-self.last_frame_wall))
                else:self.correction_dt=dt
                self.last_frame_wall=i.wall
                self.correct_reference(mission,detection,self.correction_dt,i.wall)
                if self.phase!='CENTER':return mission.output()
            if light_ready:
                # Hold the existing position reference throughout this fixed
                # competition indicator interval. Do not follow pose drift.
                mission.position=(mission.position[0],mission.position[1],mission.target_z)
                self.request_id+=1;self.request_on=True;self.on_request_wall=i.wall
                self.phase='ON_WAIT';self.reason='hover_delay_elapsed_request_laser_on'
            else:self.reason=('waiting_stable_target_alignment' if p['require_stable_target_before_light'] else 'hover_delay_before_timed_light')
        elif self.phase=='ON_WAIT':
            if bridge.get('state')=='ON_ACKNOWLEDGED' and bridge.get('request_id')==self.request_id:
                self.on_ack_wall=i.wall;self.on_ack_ros=i.ros;self.phase='EXPOSE'
                self.reason='timed_laser_on_vision_for_score_only'
            elif i.wall-self.on_request_wall>p['ack_timeout_s']:self.invalid('laser_on_not_acknowledged',i.wall)
        elif self.phase=='EXPOSE':
            if bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=self.request_id:
                self.invalid('laser_ack_lost',i.wall)
            else:
                if detection is not None and new_target and detection['capture_stamp']>self.on_ack_ros and detection.get('observed_inside_inner') is True:
                    self.observed_hit_frames+=1
                exposure_ready=i.wall-self.on_ack_wall>=p['exposure_s']
                if p['require_stable_target_before_light']:
                    observed=detection.get('observed_error_px') if detection else None
                    spot_centered=(isinstance(observed,list) and len(observed)==2 and all(type(v) in (int,float) and math.isfinite(v) for v in observed) and
                                   norm(observed)/min(detection['inner_semiaxes_px'])<=p['centre_radius_ratio'] and detection.get('observed_inside_inner') is True)
                    stable_observed=spot_centered and self.stable_motion.stable(c.c['stable_speed_mps'])
                    if stable_observed:
                        if self.stable_exposure_since is None:self.stable_exposure_since=i.wall
                        self.stable_exposure_s=i.wall-self.stable_exposure_since
                    else:self.stable_exposure_since=None;self.stable_exposure_s=0.
                    self.gates.update(observed_spot_centered=spot_centered,stable_observed_exposure=stable_observed)
                    exposure_ready=self.stable_exposure_s>=p['exposure_s']
                if exposure_ready:
                    self.exposure_completed=True;self.completed=True
                    self.completion_basis=('stable_observed_spot_exposure_camera_evidence_only' if p['require_stable_target_before_light'] else 'timed_exposure_acknowledged_hit_not_required')
                    self.off('timed_exposure_complete',i.wall)
        if self.request_on and i.wall-self.on_request_wall>=p['max_light_s']-.2:self.invalid('light_exposure_limit',i.wall)
        return mission.output()

    def tick_fixed(self,mission,i,dt,bridge):
        p=self.p;c=mission.core
        self.stable_motion.update(c.pose.stamp,c.pose.p)
        anchor=mission.aim_anchor
        horizontal=norm(sub(c.pose.p,anchor)[:2])
        vertical=abs(c.pose.p[2]-anchor[2])
        pair=self.scope=='window_pass_two_targets'
        exposing=pair and self.phase in ('ON_WAIT','EXPOSE')
        hr,vr,speed,span=(.15,.10,.30,.15) if exposing else (.12,.08,.20,.12) if pair else (.05,.04,.08,.05)
        in_region=horizontal<=hr and vertical<=vr
        stable=in_region and self.stable_motion.stable(speed,span)
        # Optional score evidence never changes the fixed command or inhibits ON.
        try:
            d,new=self.vision(i.hover_visual,i.ros,i.wall,self.hover_ros)
            target=True
            if new and self.phase=='EXPOSE' and d.get('observed_inside_inner') is True:self.observed_hit_frames+=1
        except ValueError:target=False
        self.gates=dict(fixed_position_in_region=in_region,position_stable=stable,vision_required=False,target_detected=target)
        self.metrics=dict(horizontal_error_m=horizontal,vertical_error_m=vertical,net_speed_05s_mps=self.stable_motion.speed,exposure_elapsed_s=i.wall-self.on_ack_wall if self.on_ack_wall else 0.,horizontal_tolerance_m=hr,vertical_tolerance_m=vr,net_speed_limit_mps=speed,approximate_position_mode=pair)
        if self.phase=='CENTER':
            if stable:
                if self.settled is None:self.settled=i.wall
            else:self.settled=None
            if self.settled is not None and i.wall-self.settled>=(.2 if pair else 1.) and i.wall-self.started>=p['hover_light_delay_s']:
                self.request_id+=1;self.request_on=True;self.on_request_wall=i.wall
                self.phase='ON_WAIT';self.reason='fixed_compensated_position_stable_request_on'
            elif i.wall-self.started>p['acquisition_timeout_s']:self.invalid('fixed_position_not_stable',i.wall)
            else:self.reason='waiting_fixed_compensated_position_stability'
        elif self.phase=='ON_WAIT':
            if not stable:self.invalid('fixed_position_unstable_before_on_ack',i.wall)
            elif bridge.get('state')=='ON_ACKNOWLEDGED' and bridge.get('request_id')==self.request_id:
                self.on_ack_wall=i.wall;self.on_ack_ros=i.ros;self.phase='EXPOSE';self.reason='fixed_position_timed_exposure'
            elif i.wall-self.on_request_wall>p['ack_timeout_s']:self.invalid('laser_on_not_acknowledged',i.wall)
        elif self.phase=='EXPOSE':
            if bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=self.request_id:self.invalid('laser_ack_lost',i.wall)
            elif not stable:self.invalid('fixed_position_left_stable_region_during_exposure',i.wall)
            elif i.wall-self.on_ack_wall>=p['exposure_s']:
                self.exposure_completed=True;self.completed=True;self.completion_basis='fixed_compensated_position_stable_3s_hit_not_verified'
                self.off('fixed_position_exposure_complete',i.wall)
        if self.request_on and i.wall-self.on_request_wall>=p['max_light_s']-.2:self.invalid('light_exposure_limit',i.wall)
        return mission.output()

    def correct_reference(self,mission,d,dt,wall):
        p=self.p;c=mission.core
        error=d.get('servo_error_px',d['reference_error_px'])
        paced_dt=None
        if p['paced_visual_servo'] and not self.servo_paused:
            paced_dt=self.servo_pacer.allowed_dt(mission,d,dt,p['max_speed_mps'])
            if self.servo_pacer.filtered_error_px is not None:
                error=self.servo_pacer.filtered_error_px
        if p['visual_servo_enabled']:
            if self.servo_paused:
                self.last_correction_reason='error_increasing_servo_paused';return
            scaled_error=norm(error)/min(d['inner_semiaxes_px'])
            if self.servo_best_error is None or scaled_error<self.servo_best_error:
                self.servo_best_error=scaled_error;self.servo_worse_since=None
            worse=(self.servo_path_m>=.01 and
                   scaled_error>self.servo_best_error+max(.25,self.servo_best_error*.35))
            if worse:
                if self.servo_worse_since is None:self.servo_worse_since=wall
                elif wall-self.servo_worse_since>=.4:
                    self.servo_paused=True
                    self.last_correction_reason='error_increasing_servo_paused';return
            else:self.servo_worse_since=None
            if norm(error)<=p['servo_deadband_px']:
                if p['paced_visual_servo']:self.servo_pacer.hold('within_pixel_deadband')
                self.last_correction_reason='within_pixel_deadband';return
        elif d['reference_normalized_radius']<=p['centre_radius_ratio']:return
        if p['paced_visual_servo']:
            if paced_dt is None:
                self.last_correction_reason=self.servo_pacer.reason;return
            dt=paced_dt
        dx,dy=error;a,b=d['inner_semiaxes_px'];theta=math.radians(d['inner_angle_deg'])
        co,si=math.cos(theta),math.sin(theta)
        u=(co*dx+si*dy)/a*p['inner_free_radius_m'];v=(-si*dx+co*dy)/b*p['inner_free_radius_m']
        px,py=co*u-si*v,si*u+co*v
        heading=yaw(c.pose.q);ch,sh=math.cos(heading),math.sin(heading)
        delta=(ch*py-sh*px,sh*py+ch*px)
        if p['visual_servo_enabled']:
            # Aim from measured position, then rate-limit the reference. This
            # avoids integrating a stale image error into a moving setpoint.
            delta=tuple(c.pose.p[k]+p['servo_gain']*delta[k]-mission.position[k] for k in range(2))
        length=norm(delta)
        step=min(length,p['max_speed_mps']*dt)
        if length<=1e-9 or step<=1e-9:return
        proposed=(mission.position[0]+delta[0]*step/length,mission.position[1]+delta[1]*step/length,mission.target_z)
        calibration=getattr(mission,'visual_calibration',None)
        if (calibration and not calibration.correction_allowed(mission,proposed)) or norm(sub(proposed,getattr(mission,'aim_anchor',None) or mission.origin)[:2])>p['max_correction_m'] or not mission.corridor(proposed) or (getattr(mission,'full_chain',None) and not mission.full_chain.allowed(mission,proposed)):
            # In competition mode a failed aim is a score issue. Hold the
            # existing reference; never move outside the confirmed column.
            self.last_correction_reason='correction_boundary_hold';return
        horizontal_lead=norm(sub(proposed,c.pose.p)[:2])
        old_horizontal_lead=norm(sub(mission.position,c.pose.p)[:2])
        # Altitude is tracked independently by the flight supervisor. Its
        # intentional remaining climb must not consume the planar servo lead.
        # Permit a rate-limited return towards an already exceeded lead bound.
        allowed=(horizontal_lead<=c.c['max_setpoint_lead_m'] or
                 horizontal_lead<old_horizontal_lead-1e-9)
        if p['paced_visual_servo'] and self.servo_pacer.reduce_lead_only:
            allowed=allowed and horizontal_lead<old_horizontal_lead-1e-9
        self.servo_diagnostics=dict(horizontal_lead_m=horizontal_lead,
                                    previous_horizontal_lead_m=old_horizontal_lead,
                                    vertical_tracking_error_m=mission.target_z-c.pose.p[2],
                                    planar_lead_limit_m=c.c['max_setpoint_lead_m'],
                                    proposed_position_local_m=list(proposed),
                                    actual_position_local_m=list(c.pose.p))
        if allowed:
            if p['paced_visual_servo']:self.servo_pacer.applied(mission.position,proposed)
            self.servo_path_m+=norm(sub(proposed,mission.position)[:2])
            mission.position=proposed;self.correction_updates+=1;self.last_correction_reason='visual_correction_applied'
        else:self.last_correction_reason='waiting_for_horizontal_tracking' 

    def tick(self,mission,i,dt):
        p=self.p;c=mission.core
        bridge=fresh_bridge(i.hover_laser,i.wall,self.scope,allow_fault_off=self.phase=='OFF_WAIT')
        if self.phase=='WAIT_HOVER':
            self.started=i.wall;self.hover_ros=i.ros;self.phase='CENTER';self.reason='camera_guided_centering'
        if self.phase=='OFF_WAIT':
            if bridge and bridge.get('state')=='OFF_ACKNOWLEDGED' and bridge.get('request_id')==self.request_id:
                self.off_confirmed=True
                if bridge.get('fault_latched') is True:
                    self.completed=False;self.reason='laser_bridge_fault_but_off_confirmed'
                self.phase='DONE' if self.completed else 'ABORT'
                session=getattr(mission,'visual_calibration',None)
                if self.completed and session and self.calibration_candidate and not self.calibration_recorded:
                    session.accept(mission,self.calibration_candidate);self.calibration_recorded=True
                if self.completed and hasattr(getattr(mission,'full_chain',None),'finish_target'):
                    return mission.full_chain.finish_target(mission,i)
                return mission.handover('hover_target_'+self.reason,i)
            deadline=self.off_request_wall+p['ack_timeout_s']+getattr(self,'off_ack_extra_s',0.)
            if i.wall>deadline:
                if (getattr(mission,'recovery',None) and
                        mission.recovery.take(mission,i,'light_off_ack','awaiting_same_OFF_request',maximum=1)):
                    self.off_ack_extra_s=p['ack_timeout_s']
                    self.reason='retry_wait_same_OFF_request'
                    return mission.output()
                self.phase='ABORT';self.completed=False
                return mission.handover('hover_target_off_ack_timeout_hardware_unknown',i)
            return mission.output()
        if bridge is None:
            self.invalid('laser_bridge_missing_or_fault',i.wall);return mission.output()
        if i.wall-self.started>p['acquisition_timeout_s']+p['max_light_s']:
            self.invalid('target_session_timeout',i.wall);return mission.output()
        if p['visual_calibration_mode']!='off':
            from visual_calibration import tick_observed
            return tick_observed(self,mission,i,dt,bridge)
        if p['visual_target_hold']:
            from visual_target_hold import tick
            return tick(self,mission,i,dt,bridge)
        if p['fixed_position_timed']:return self.tick_fixed(mission,i,dt,bridge)
        if p['trigger_policy']=='hover_timed':return self.tick_timed(mission,i,dt,bridge)
        try:d,new=self.vision(i.hover_visual,i.ros,i.wall,self.hover_ros)
        except ValueError as exc:
            self.settled=None;self.motion_samples.clear();self.window_speed=None;self.last_frame_wall=None
            self.gates={'camera_valid':False};self.metrics={'camera_error':str(exc)}
            if self.request_on:self.invalid(str(exc),i.wall)
            elif i.wall-self.started>p['acquisition_timeout_s']:self.invalid(str(exc),i.wall)
            self.reason=str(exc)
            return mission.output()
        if new:
            # Images arrive at 20Hz while the supervisor runs at 50Hz. Integrate
            # the elapsed time between fresh frames, not one supervisor tick.
            self.correction_dt=(min(p['observation_timeout_s'],max(0.,i.wall-self.last_frame_wall))
                                if self.last_frame_wall is not None else dt)
            self.last_frame_wall=i.wall
            # Finite-difference speed from one odometry tick amplifies jitter.
            # Measure net displacement over 0.3s; keep the same speed threshold.
            self.motion_samples.append((i.wall,tuple(c.pose.p)))
            while len(self.motion_samples)>2 and self.motion_samples[1][0]<=i.wall-.3:
                self.motion_samples.popleft()
            elapsed=i.wall-self.motion_samples[0][0]
            self.window_speed=(norm(sub(c.pose.p,self.motion_samples[0][1]))/elapsed
                               if elapsed>=.3-1e-6 else None)
        # Camera up is body-forward; camera right is body-right. This profile
        # uses a downward camera and a near-horizontal target at the fixed height.
        # Convert the image ellipse to a metric planar residual before yaw rotation.
        dx,dy=d['reference_error_px'];a,b=d['inner_semiaxes_px'];theta=math.radians(d['inner_angle_deg'])
        co,si=math.cos(theta),math.sin(theta)
        u=(co*dx+si*dy)/a*p['inner_free_radius_m'];v=(-si*dx+co*dy)/b*p['inner_free_radius_m']
        px,py=co*u-si*v,si*u+co*v
        body_forward,body_left=py,px
        heading=yaw(c.pose.q);ch,sh=math.cos(heading),math.sin(heading)
        delta=(ch*body_forward-sh*body_left,sh*body_forward+ch*body_left)
        centred=d['reference_normalized_radius']<=p['centre_radius_ratio']
        actual=d.get('observed_spot_px')
        spot_ok=False
        if isinstance(actual,list) and len(actual)==2:
            error=d.get('observed_minus_reference_px')
            if isinstance(error,list) and len(error)==2 and all(type(v) in (int,float) and math.isfinite(v) for v in error):
                spot_ok=norm(error)<=p['spot_error_limit_px'] and d.get('observed_inside_inner') is True
        if self.phase=='CENTER' and not centred and new:
            length=norm(delta);step=min(length,p['max_speed_mps']*self.correction_dt)
            if length>1e-9:
                proposed=(mission.position[0]+delta[0]*step/length,mission.position[1]+delta[1]*step/length,mission.target_z)
                if norm(sub(proposed,getattr(mission,'aim_anchor',None) or mission.origin)[:2])>p['max_correction_m'] or not mission.corridor(proposed) or (getattr(mission,'full_chain',None) and not mission.full_chain.allowed(mission,proposed)):
                    self.invalid('centering_outside_takeoff_column',i.wall);return mission.output()
                if norm(sub(proposed,c.pose.p))<=c.c['max_setpoint_lead_m']:
                    mission.position=proposed
        tracking_error=norm(sub(c.pose.p,mission.position))
        self.gates=dict(camera_valid=True,reference_inside_centre=centred,
                        speed_window_ready=self.window_speed is not None,
                        speed_stable=self.window_speed is not None and self.window_speed<=c.c['stable_speed_mps'],
                        position_tracking=tracking_error<=c.c['align_tolerance_m'])
        stable=all(self.gates.values())
        self.metrics=dict(reference_normalized_radius=d['reference_normalized_radius'],
                          centre_radius_limit=p['centre_radius_ratio'],pose_speed_raw_mps=c.pose_speed,
                          pose_speed_window_mps=self.window_speed,speed_limit_mps=c.c['stable_speed_mps'],
                          tracking_error_m=tracking_error,tracking_tolerance_m=c.c['align_tolerance_m'],
                          centering_elapsed_s=i.wall-self.started,correction_frame_dt_s=self.correction_dt,
                          settled_s=(max(0.,d['capture_stamp']-self.settled) if stable and self.settled is not None else 0.))
        if self.phase=='CENTER':
            if stable and new:
                if self.settled is None:self.settled=d['capture_stamp']
                if d['capture_stamp']-self.settled>=p['settle_time_s']:
                    self.request_id+=1;self.request_on=True;self.on_request_wall=i.wall;self.phase='ON_WAIT';self.reason='centred_request_laser_on'
            elif not stable:self.settled=None
            if i.wall-self.started>p['acquisition_timeout_s']:self.invalid('centering_timeout',i.wall)
        elif self.phase=='ON_WAIT':
            if not stable:self.invalid('stability_lost_before_light_ack',i.wall)
            elif bridge.get('state')=='ON_ACKNOWLEDGED' and bridge.get('request_id')==self.request_id:
                self.on_ack_wall=i.wall;self.on_ack_ros=i.ros;self.phase='EXPOSE';self.reason='laser_on_observe_spot';self.hit_since=None
            elif i.wall-self.on_request_wall>p['ack_timeout_s']:self.invalid('laser_on_not_acknowledged',i.wall)
        elif self.phase=='EXPOSE':
            if not stable or (actual is not None and not spot_ok):self.invalid('spot_or_stability_outside_target',i.wall)
            elif bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=self.request_id:self.invalid('laser_ack_lost',i.wall)
            elif spot_ok and new and d['capture_stamp']>self.on_ack_ros:
                if self.hit_since is None:self.hit_since=d['capture_stamp']
                if d['capture_stamp']-self.hit_since>=p['exposure_s']:
                    self.completed=True;self.off('camera_observed_interval_complete',i.wall)
            elif not spot_ok:
                self.hit_since=None
                if i.wall-self.on_ack_wall>.35:self.invalid('light_spot_not_observed',i.wall)
            if self.request_on and i.wall-self.on_request_wall>=p['max_light_s']-.2:self.invalid('light_exposure_limit',i.wall)
        return mission.output()
