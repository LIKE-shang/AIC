"""Visual aiming with one fixed three-second exposure per selected target.

No ROS or light IO. Mission owns motion; TargetHold owns OFF acknowledgement.
"""
import math
from window_pass.geometry import norm, sub


def reset_exposure(h):
    h.stable_exposure_since=None
    h.stable_exposure_s=0.


def recover_visual_servo(h,m,d,wall):
    """Resume a scoring servo only after fresh evidence of image recovery.

    This is not a mission/pose/light fault reset. Persistent worsening keeps
    holding. Geometry, speed, correction radius and acquisition budget remain.
    """
    count=getattr(h,'visual_recoveries',0)
    h.metrics['servo_recovery_count']=count
    if not h.servo_paused:
        h.visual_recovery_since=None;h.visual_recovery_frames=0
        return
    error=norm(d['reference_error_px'])/min(d['inner_semiaxes_px'])
    best=h.servo_best_error
    lead=norm(sub(m.position,m.core.pose.p)[:2])
    # Return within 15% of the best seen error (stricter than the old 35%
    # worsening trigger). Allow normal hover wobble; do not require zero speed.
    recovered=(best is not None and error<=best+max(.15,best*.15) and
               h.stable_motion.stable(.20,.12) and lead<=.08 and count<3)
    h.metrics.update(servo_recovery_error_ratio=error,servo_recovery_horizontal_lead_m=lead,
                     servo_recovery_limit=3,servo_recovery_candidate=recovered)
    if not recovered:
        h.visual_recovery_since=None;h.visual_recovery_frames=0
        return
    if getattr(h,'visual_recovery_since',None) is None:
        h.visual_recovery_since=d['capture_stamp'];h.visual_recovery_frames=0
    h.visual_recovery_frames+=1
    if h.visual_recovery_frames>=3 and d['capture_stamp']-h.visual_recovery_since>=.15:
        h.servo_paused=False;h.servo_worse_since=None
        h.servo_best_error=min(best,error);h.visual_recoveries=count+1
        h.visual_recovery_since=None;h.visual_recovery_frames=0
        h.metrics['servo_recovery_count']=h.visual_recoveries
        h.last_correction_reason='visual_recovered_bounded_servo_resume'


def acquisition_failure(h):
    # An invalid final image must not erase the preceding valid detections.
    if h.last_visual_stamp is None:return 'visual_target_not_acquired'
    return 'visual_servo_recovery_exhausted' if h.servo_paused and getattr(h,'visual_recoveries',0)>=3 else 'visual_target_not_aligned'


def tick(h,m,i,dt,bridge):
    p,c=h.p,m.core
    h.stable_motion.update(c.pose.stamp,c.pose.p)
    anchor=m.aim_anchor or m.origin
    # Nominal station error remains diagnostic. It is not a camera aim error.
    h.metrics=dict(nominal_horizontal_error_m=norm(sub(c.pose.p,anchor)[:2]),
                   nominal_error_abort_enabled=False,
                   net_speed_05s_mps=h.stable_motion.speed,
                   exposure_elapsed_s=i.wall-h.on_ack_wall if h.on_ack_wall is not None else 0.)
    timed=p.get('timed_visual_aim',False)
    bounded=p.get('bounded_visual_aim',False)
    aim_budget=p['visual_aim_max_s'] if bounded else p['hover_light_delay_s']
    h.gates=dict(target_required_for_light=not timed,actual_spot_required_for_completion=False)
    h.metrics.update(visual_aim_budget_s=aim_budget if timed else None,
                     visual_aim_elapsed_s=i.wall-h.started)
    h.metrics['exposure_timer_policy']='fixed_3s_after_on_ack'
    if h.phase=='ON_WAIT' and bridge.get('state')=='ON_ACKNOWLEDGED' and bridge.get('request_id')==h.request_id:
        h.on_ack_wall=i.wall;h.on_ack_ros=i.ros;h.phase='EXPOSE'
        h.reason='timed_exposure_with_visual_correction';reset_exposure(h)
        h.servo_best_error=None;h.servo_worse_since=None;h.servo_paused=False
    if h.phase=='EXPOSE':
        if bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=h.request_id:
            h.invalid('laser_ack_lost',i.wall);return m.output()
        elapsed=i.wall-h.on_ack_wall
        h.metrics.update(exposure_elapsed_s=elapsed,exposure_remaining_s=max(0.,p['exposure_s']-elapsed))
        # Completion depends on the acknowledged ON interval, never image availability.
        if elapsed>=p['exposure_s']:
            h.exposure_completed=True;h.completed=True
            h.completion_basis='timed_3s_after_on_ack_hit_not_verified'
            h.off('visual_target_timed_exposure_complete',i.wall);return m.output()
    if h.request_on and i.wall-h.on_request_wall>=p['max_light_s']-.2:
        h.invalid('light_exposure_limit',i.wall);return m.output()
    if h.phase=='ON_WAIT' and i.wall-h.on_request_wall>p['ack_timeout_s']:
        h.invalid('laser_on_not_acknowledged',i.wall);return m.output()
    if h.phase=='EXPOSE' and (bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=h.request_id):
        h.invalid('laser_ack_lost',i.wall);return m.output()
    # Keep the original safe height, field and actual/reference corridor checks.
    if abs(c.pose.p[2]-m.target_z)>.10:
        h.invalid('visual_hold_height_outside_region',i.wall);return m.output()
    if not m.corridor(c.pose.p) or (getattr(m,'full_chain',None) and not m.full_chain.allowed(m,c.pose.p)):
        h.invalid('visual_hold_actual_outside_region',i.wall);return m.output()
    # The route supervisor already confirmed arrival. Keep the bridge's
    # one-second contract: gather OFF evidence, then light before spot servo.
    if timed and h.phase=='CENTER' and i.wall-h.started>=aim_budget:
        h.request_id+=1;h.request_on=True;h.on_request_wall=i.wall
        h.phase='ON_WAIT';h.reason='bounded_visual_budget_elapsed_request_on' if bounded else 'one_second_visual_budget_elapsed_request_on'
        h.visual_start_basis='bounded_timeout_visual_unconfirmed' if bounded else 'legacy_one_second_timer'
        h.gates['visual_aim_budget_elapsed']=True
        return m.output()
    try:
        d,new=h.vision(i.hover_visual,i.ros,i.wall,h.hover_ros)
    except ValueError as e:
        if bounded:h.aim_evidence.missing(str(e))
        if p['paced_visual_servo']:h.servo_pacer.hold('visual_unavailable_hold_position')
        reset_exposure(h);h.settled=None
        h.visual_recovery_since=None;h.visual_recovery_frames=0
        h.last_correction_reason='visual_unavailable_hold_position'
        h.metrics['visual_error']=str(e)
        if not timed and not h.request_on and i.wall-h.started>p['acquisition_timeout_s']:
            h.invalid(acquisition_failure(h),i.wall)
        return m.output()
    if not new:
        # Repeated images cannot move, request light or advance exposure.
        return m.output()
    previous=getattr(h,'visual_capture_stamp',None)
    if previous is not None and d['capture_stamp']-previous>p['observation_timeout_s']:
        reset_exposure(h);h.settled=None
        h.visual_recovery_since=None;h.visual_recovery_frames=0
    h.visual_capture_stamp=d['capture_stamp'];h.visual_valid_wall=i.wall
    frame_dt=min(p['observation_timeout_s'],max(0.,i.wall-h.last_frame_wall)) if h.last_frame_wall is not None else dt
    h.last_frame_wall=i.wall
    target_ready,spot_ready=(h.aim_evidence.observe(d,h.phase,h.on_ack_ros,p['observation_timeout_s']) if bounded else (True,True))
    if getattr(m,'full_chain',None):m.full_chain.target_acquired=True
    if h.phase=='CENTER':
        if bounded:
            # A predicted reference is not an observed laser spot. Hold the
            # arrived position while collecting the OFF background; real-spot
            # corrections start only after ON_ACK and temporal confirmation.
            h.last_correction_reason='await_light_hold_arrived_position'
            h.reason='off_background_before_timed_light'
            h.gates.update(prelight_motion_enabled=False,
                           clear_off_baseline=h.aim_evidence.baseline_clear)
            return m.output()
        recover_visual_servo(h,m,d,i.wall)
        # Never start an exposure on an already-present bright reflection.
        baseline=d.get('observed_candidates')==[]
        centred=d['reference_normalized_radius']<=p['centre_radius_ratio']
        if bounded:centred=centred and norm(d['reference_error_px'])<=p['servo_deadband_px']
        if not centred and target_ready:
            h.correct_reference(m,dict(d,servo_error_px=d['reference_error_px']),frame_dt,i.wall)
        stopped=h.stable_motion.stable(.05,.025) if bounded else h.stable_motion.stable(.20,.12)
        aligned=centred and stopped and baseline and not h.servo_paused and target_ready and (not bounded or h.aim_evidence.baseline_clear)
        if aligned:
            if h.settled is None:h.settled=d['capture_stamp']
        else:h.settled=None
        held=h.settled is not None and d['capture_stamp']-h.settled>=p['settle_time_s']
        h.gates.update(target_aligned=centred,position_stable=stopped,baseline_clear=baseline,stable_target_held=held)
        if (not timed or bounded) and held and i.wall-h.started>=p['hover_light_delay_s']:
            h.request_id+=1;h.request_on=True;h.on_request_wall=i.wall
            h.phase='ON_WAIT';h.reason='visual_target_aligned_request_on'
            h.visual_start_basis='stable_visual_alignment'
        elif not timed and i.wall-h.started>p['acquisition_timeout_s']:
            h.invalid(acquisition_failure(h),i.wall)
        else:h.reason='visual_target_centering'
    elif h.phase=='EXPOSE':
        error=d.get('observed_error_px');radius=d.get('observed_normalized_radius')
        spot=(d.get('observed_unique') is True and isinstance(d.get('observed_candidates'),list) and len(d['observed_candidates'])==1 and
              isinstance(error,list) and len(error)==2 and all(type(v) in (int,float) and math.isfinite(v) for v in error) and
              type(radius) in (int,float) and math.isfinite(radius) and radius>=0 and d['capture_stamp']>h.on_ack_ros)
        spot=spot and spot_ready
        centred=spot and d.get('observed_inside_inner') is True and radius<=p['centre_radius_ratio']
        if bounded and centred:centred=norm(error)<=p['servo_deadband_px']
        if spot:
            h.aim_source='observed_spot_minus_target';h.aim_error_px=list(error)
            if not centred:h.correct_reference(m,dict(d,servo_error_px=error,
                                                       servo_capture_age_s=i.ros-d['capture_stamp']),frame_dt,i.wall)
            else:
                h.last_correction_reason='observed_spot_centre_hold'
                if p['paced_visual_servo']:h.servo_pacer.hold('observed_spot_centre_hold')
        else:
            h.last_correction_reason=h.aim_evidence.last_reason if bounded else 'spot_missing_or_ambiguous_hold_position'
            if p['paced_visual_servo']:h.servo_pacer.hold(h.last_correction_reason)
        stopped=h.stable_motion.stable(.30,.15)
        h.gates.update(actual_unique_spot=bool(spot),observed_spot_centered=bool(centred),position_stable=stopped)
        if centred and stopped:
            h.observed_hit_frames+=1
            if h.stable_exposure_since is None:h.stable_exposure_since=d['capture_stamp']
            h.stable_exposure_s=d['capture_stamp']-h.stable_exposure_since
        else:reset_exposure(h)
        h.metrics.update(spot_normalized_radius=radius,stable_observed_exposure_s=h.stable_exposure_s)
    return m.output()
