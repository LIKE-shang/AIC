"""Single-owner automatic flight supervisor. Pure Python; no ROS or hardware IO.
A transport must acknowledge each successful setpoint publication with sent().
Service acceptance is never substituted for observed FCU state.
"""
import math
from dataclasses import dataclass, replace
from window_pass.core import Controller
from window_pass.geometry import add, sub, norm, yaw, tilt, rotate, wrap

TERMINAL = ('DONE', 'REFUSED', 'TAKEOVER', 'FAULT', 'PILOT_CONTROL')

@dataclass
class Inputs:
    wall: float
    ros: float
    state_fresh: bool
    landed_fresh: bool
    landed: bool
    rc_ok: bool
    estimator_ok: bool
    battery_ok: bool
    graph_ok: bool
    emergency: bool = False
    estimator_reason: str = ""
    estimator_evidence: object = None
    localization_identity: object = None
    localization_epoch: object = None
    hover_visual: object = None
    hover_laser: object = None
    # Preserve the MAVROS enum: LANDING (4) is neither IN_AIR (2) nor ON_GROUND (1).
    landed_state: object = None

DEFAULTS = dict(bottom_to_fcu_m=0.245, ground_fcu_height_m=None,
                target_bottom_height_m=0.90, minimum_bottom_height_m=0.80,
                climb_speed_mps=0.10, takeoff_timeout_s=25., search_timeout_s=10.,
                stable_time_s=2., hover_time_s=5., mode_timeout_s=4., landing_timeout_s=25.,
                max_mission_time_s=100., landing_radius_m=0.20, takeoff_horizontal_error_m=0.12,
                minimum_battery_voltage=15.2, ground_z_span_limit_m=0.04,
                companion_battery_monitor_enabled=True, simplified_takeoff_enabled=False, vendor_takeoff_arrival_enabled=False, takeoff_transient_tolerance_enabled=False, takeoff_attitude_checks_enabled=True, takeoff_vertical_velocity_enabled=False, takeoff_position_goal_enabled=False, pilot_landing_enabled=False, second_window_return_enabled=False)


def settings(supplied):
    unknown = set(supplied)-set(DEFAULTS)
    if unknown:
        raise ValueError('unknown mission settings: '+str(sorted(unknown)))
    out = dict(DEFAULTS, **supplied)
    for k, v in out.items():
        if k in ("companion_battery_monitor_enabled", "simplified_takeoff_enabled", "vendor_takeoff_arrival_enabled", "takeoff_transient_tolerance_enabled", "takeoff_attitude_checks_enabled", "takeoff_vertical_velocity_enabled", "takeoff_position_goal_enabled", "pilot_landing_enabled", "second_window_return_enabled"):
            if type(v) is not bool: raise ValueError("monitoring policy must be boolean: "+k)
            continue
        if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
            raise ValueError('measured positive setting required: '+k)
    if not .12 <= out['takeoff_horizontal_error_m'] <= min(.20,out['landing_radius_m']):
        raise ValueError('takeoff horizontal allowance must fit the takeoff column')
    if out['simplified_takeoff_enabled'] and (not out['vendor_takeoff_arrival_enabled'] or not out['takeoff_position_goal_enabled'] or out['takeoff_vertical_velocity_enabled']):
        raise ValueError('simplified takeoff requires vendor arrival and full position goal')
    if out['takeoff_vertical_velocity_enabled'] and out['takeoff_position_goal_enabled']:
        raise ValueError('takeoff control modes are mutually exclusive')
    minimum_allowed=.70 if out['simplified_takeoff_enabled'] else .80
    if not minimum_allowed <= out['minimum_bottom_height_m'] < out['target_bottom_height_m'] <= 1.10:
        raise ValueError('invalid height band')
    if not .02 <= out['climb_speed_mps'] <= .15:
        raise ValueError('invalid climb speed')
    if out['takeoff_vertical_velocity_enabled'] and out['climb_speed_mps']>.10:
        raise ValueError('takeoff velocity test exceeds 0.10 m/s')
    if not 2 <= out['search_timeout_s'] <= 20:
        raise ValueError('invalid search timeout')
    if not .15 <= out['bottom_to_fcu_m'] <= .35:
        raise ValueError('invalid measured body offset')
    if out['ground_fcu_height_m'] < out['bottom_to_fcu_m']-.01:
        raise ValueError('ground measurement places vehicle bottom below floor')
    for k in ('takeoff_timeout_s', 'landing_timeout_s', 'max_mission_time_s'):
        if out[k] > (240 if k=='max_mission_time_s' and out['second_window_return_enabled'] else 150): raise ValueError('excessive timeout: '+k)
    if not 1 <= out['hover_time_s'] <= 10:
        raise ValueError('invalid hover duration')
    if out['stable_time_s'] < 1 or out['mode_timeout_s'] > 5:
        raise ValueError('invalid stability/mode timeout')
    return out


class Mission:
    def __init__(self, control, options, airborne_geometry=False, experiment="traverse", alignment_commissioning=False, straight_commissioning=False, adjustment_policy=None, session_window_policy=None, session_identity=None, hover_target_policy=None, full_chain_policy=None):
        self.o = settings(options)
        if experiment not in ('traverse','takeoff_land','align_only','straight_pass'):raise ValueError('unknown experiment')
        from full_chain import settings as chain_settings,Postpass,SCOPE as CHAIN_SCOPE
        if isinstance(full_chain_policy,dict) and 'selected_target_ids' in full_chain_policy:
            from two_targets import TwoTargetPostpass,SCOPE as PAIR_SCOPE
            self.full_chain=TwoTargetPostpass(full_chain_policy);CHAIN_SCOPE=PAIR_SCOPE
        else:
            chain_policy=chain_settings(full_chain_policy)
            self.full_chain=Postpass(chain_policy) if chain_policy else None
        from second_window_return import Journey
        self.return_journey=Journey(self.full_chain.return_policy) if getattr(self.full_chain,'return_policy',None) else None
        if bool(self.return_journey)!=self.o['second_window_return_enabled']:raise ValueError('return mission authorization mismatch')
        self.aim_anchor=None
        self.takeoff_only=experiment=='takeoff_land'
        from hover_target import settings as hover_settings, TargetHold
        hover_policy=hover_settings(hover_target_policy)
        self.hover_target=TargetHold(hover_policy) if hover_policy else None
        if self.hover_target and ((not self.takeoff_only and not self.full_chain) or not self.o['pilot_landing_enabled']):
            raise ValueError('hover target requires takeoff-only and pilot handover')
        self.align_only=experiment=='align_only'
        self.direct_pass=experiment=='straight_pass'
        if self.full_chain:
            if not self.direct_pass or not self.hover_target or not (session_window_policy or {}).get('enabled'):
                raise ValueError('full chain requires session-window pass and target policy')
            self.hover_target.scope=CHAIN_SCOPE
        if straight_commissioning and not self.direct_pass:raise ValueError('straight commissioning is straight-pass-only')
        if self.direct_pass and (not airborne_geometry or not self.o['pilot_landing_enabled']):raise ValueError('straight passage requires live RGB-D and pilot handover')
        if self.direct_pass:
            from straight_pass import PassageDecision
            self.passage_decision=PassageDecision()
        from prepass_adjustment import settings as adjustment_settings
        self.adjustment_policy=adjustment_settings(adjustment_policy)
        if self.adjustment_policy['enabled'] and not self.direct_pass:raise ValueError('adjustment is straight-pass-only')
        self.adjust_candidate=None;self.adjust_plan=None;self.adjust_count=0;self.adjust_arrival_since=None
        self.adjust_decision=None;self.adjust_reason='disabled' if not self.adjustment_policy['enabled'] else 'not_needed'
        self.adjust_completed=False;self.adjust_path_m=0.;self.adjust_last_position=None
        from session_window import settings as window_settings,Acquisition,localization_identity,alignment_policy
        self.session_policy=window_settings(session_window_policy)
        self.session_alignment_policy=alignment_policy(self.session_policy)
        self.session_window=None;self.localization_epoch=None;self.session_invalidated=None
        if self.session_policy['enabled']:
            if not self.direct_pass or self.adjustment_policy['enabled']:
                raise ValueError('session-window requires straight pass without legacy adjustment')
            if not self.o['simplified_takeoff_enabled']:
                raise ValueError('session-window requires simplified vendor takeoff')
            self.session_window=Acquisition(localization_identity(session_identity),self.session_policy['fast_confirmation'])
        elif self.o['simplified_takeoff_enabled']:
            raise ValueError('simplified takeoff is scoped to session-window profile')
        self.direct_plan=None
        self.pass_velocity=None
        self.station_velocity=None
        self.terminal_capture=None
        if self.session_policy.get('terminal_capture'):
            if not self.full_chain:raise ValueError('terminal capture requires same-owner full chain')
            from pass_terminal import TerminalCapture
            self.terminal_capture=TerminalCapture()
        self.direct_pass_completed=False
        self.direct_arrival_since=None
        if self.o['vendor_takeoff_arrival_enabled'] and not (self.takeoff_only or self.align_only or self.direct_pass):raise ValueError('vendor arrival trial cannot enable traversal')
        if alignment_commissioning and not self.align_only:raise ValueError('alignment commissioning is alignment-only')
        self.alignment_completed=False
        if self.o['takeoff_transient_tolerance_enabled'] and not (self.align_only or self.direct_pass):raise ValueError('transient takeoff tolerance is limited to alignment-only trials')
        if self.align_only and not airborne_geometry:raise ValueError('alignment-only requires fresh airborne RGB-D geometry')
        if self.align_only and not self.o['pilot_landing_enabled']:raise ValueError('alignment-only requires pilot handover')
        if self.align_only and (control.get('enable_traverse') or control.get('pass_stage_approved')):raise ValueError('alignment-only forbids traversal permissions')
        if self.o['takeoff_vertical_velocity_enabled'] and not self.takeoff_only:
            raise ValueError('vertical velocity commissioning is takeoff-only')
        if self.takeoff_only and airborne_geometry:raise ValueError('takeoff test cannot acquire window geometry')
        self.takeoff_hover_completed=False
        self.airborne_geometry=airborne_geometry
        if self.takeoff_only:
            from takeoff import TakeoffCore
            self.core=TakeoffCore(control,attitude_checks_enabled=self.o["takeoff_attitude_checks_enabled"])
        elif self.direct_pass:
            from straight_pass import StraightController
            self.core=StraightController(control,straight_commissioning=straight_commissioning,session_window=bool(self.session_window))
        elif airborne_geometry:
            from live_geometry import AirborneController
            self.core=AirborneController(control,alignment_commissioning=alignment_commissioning,alignment_only=self.align_only)
        else:
            self.core=Controller(control)
        if self.session_window:
            from session_window import decorate
            decorate(self.core.c,self.session_policy)
        self.search_started_ros=None
        self.search_evidence=None
        self.search_events=[]
        required=('enable_motion',) if self.takeoff_only or self.align_only else ('enable_motion','enable_traverse','pass_stage_approved')
        if not all(self.core.c[k] for k in required):
            raise ValueError('explicit verified motion/pass configuration required')
        expected = self.core.c['vehicle_size_m'][2]/2-self.o['bottom_to_fcu_m']
        if abs(self.core.c['envelope_center_body_m'][2]-expected) > .001:
            raise ValueError('envelope centre inconsistent with measured bottom offset')
        self.phase, self.reason = 'IDLE', 'await_explicit_start'
        self.position = self.heading = self.origin = self.ground_z = None
        self.takeoff_heading=None
        self.began = self.entered = self.last_tick = None
        self.stable_since = None
        self.sent_count = 0
        self.last_sent = None
        self.events = []
        self.landing_site = None
        self.command_result = None
        self.last_command_feedback = None
        self.fault_snapshot = None
        self.handover_snapshot = None
        self.startup_checks = None
        self.last_estimator_evidence = None
        self.arm_request_wall = None
        self.takeoff_progress = None
        self.takeoff_stability_resets = 0
        self.takeoff_stable_max_s = 0.
        from mission_recovery import Recovery
        self.recovery = Recovery()

    def transition(self, phase, reason, t):
        if self.session_window and phase in ('FAULT','TAKEOVER'):
            self.session_invalidated=reason
        self.events.append(dict(old=self.phase, new=phase, reason=reason, wall=t))
        self.phase, self.reason, self.entered = phase, reason, t
        self.stable_since = None

    def sent(self, t):
        self.sent_count += 1
        self.last_sent = t
        if self.core.authorized:
            self.core.output_sent(t)

    def command_finished(self, command, accepted, result_code=None, error=None):
        self.command_result = (command, accepted)
        self.last_command_feedback=dict(command=command,accepted=bool(accepted),mav_result=result_code,error=error)

    def simplified_takeoff(self):
        return self.o["simplified_takeoff_enabled"] and (self.phase=="TAKEOFF" or (self.phase=="ARM" and self.core.armed))

    def attitude_checks_enabled(self):
        # This operator-supervised exception never disables traversal heading checks.
        if self.session_window and (self.phase=='HANDOVER_REQUEST' or (self.phase=='LANDING_HANDOVER' and self.core.mode=='AUTO.LAND')):return False
        return not self.simplified_takeoff() and (not self.takeoff_only or self.o["takeoff_attitude_checks_enabled"])

    def hover_takeoff_heading_tolerance(self):
        # Reuse the existing takeoff transient yaw band only for USB target
        # takeoff. Keep tilt, tracking, fence and input checks unchanged.
        return (self.hover_target is not None and self.position_goal_takeoff() and
                self.o['vendor_takeoff_arrival_enabled'] and
                (self.phase == 'TAKEOFF' or (self.phase == 'ARM' and self.core.armed)))

    def heading_error_gate_enabled(self):
        # Operator-requested full competition policy: yaw remains commanded and
        # measured for geometry, but yaw error alone is not a flight gate.
        return not (getattr(self.full_chain,'scope',None)=='window_pass_two_targets' and
                    getattr(self.full_chain,'return_policy',None))

    def heading_limit(self):
        if not self.heading_error_gate_enabled():return None
        guard=getattr(self.full_chain,'transfer_heading',None)
        if guard and guard.scoped(self):return guard.hard_limit_rad
        return math.radians(30) if self.transient_takeoff() or self.hover_takeoff_heading_tolerance() else self.core.c['yaw_limit_rad']

    def hover_heading_ready(self):
        return (not self.heading_error_gate_enabled() or self.hover_target is None or
                abs(wrap(yaw(self.core.pose.q)-self.heading)) <= self.core.c['yaw_limit_rad'])

    def position_goal_takeoff(self):
        return self.o['takeoff_position_goal_enabled']

    def transient_takeoff(self):
        return ((self.align_only or self.direct_pass) and self.o['takeoff_transient_tolerance_enabled'] and
                (self.phase=='TAKEOFF' or (self.phase=='ARM' and self.core.armed)))

    def precise_takeoff_ready(self):
        c=self.core
        return (norm(sub(c.pose.p,self.origin)[:2])<=c.c['max_tracking_error_m'] and
                tilt(c.pose.q)<=c.c['tilt_limit_rad'] and
                (not self.heading_error_gate_enabled() or abs(wrap(yaw(c.pose.q)-self.heading))<=c.c['yaw_limit_rad']))

    def horizontal_tracking_limit(self):
        if self.simplified_takeoff():return None
        if self.transient_takeoff():return .30
        return self.o['takeoff_horizontal_error_m'] if self.takeoff_only else self.core.c['max_tracking_error_m']

    def tracking_snapshot(self):
        actual = self.core.pose.p if self.core.pose else None
        error = sub(self.position,actual) if self.position is not None and actual is not None else None
        turning = bool(self.return_journey and self.phase=='YAW_TURN')
        return_move=bool(self.return_journey and self.phase in ('RETURN_PREP','RETURN_HOME') and not self.return_journey.p['movement_tracking_error_abort_enabled'])
        second_search=bool(self.return_journey and self.return_journey.search_active(self))
        return dict(target_position_local_m=self.position, actual_position_local_m=actual,
                    error_target_minus_actual_m=error,
                    error_norm_m=norm(error) if error is not None else None,
                    horizontal_error_m=norm(error[:2]) if error is not None else None,
                    vertical_error_m=error[2] if error is not None else None,
                    tracking_limit_m=None if turning or return_move or second_search else self.core.c["max_tracking_error_m"],
                    horizontal_tracking_limit_m=None if turning or return_move or second_search else self.horizontal_tracking_limit(),
                    tracking_error_abort_enabled=False if turning or return_move or second_search else None,
                    tracking_check="diagnostic_only_second_window_search" if second_search else "diagnostic_only_return_movement" if return_move else "turn_stage_envelope_then_visual_reacquisition" if turning else "simplified_takeoff_no_xy_abort" if self.simplified_takeoff() else "horizontal_and_height_overshoot" if self.position_goal_takeoff() and self.phase=="TAKEOFF" else "three_dimensional",
                    pose_stamp=self.core.pose.stamp if self.core.pose else None,
                    airborne=self.core.airborne, armed=self.core.armed, mode=self.core.mode)

    def takeoff_vertical_velocity(self):
        if not self.takeoff_only or not self.o['takeoff_vertical_velocity_enabled'] or self.phase!='TAKEOFF':
            return None
        # ROS ENU: positive z velocity climbs. Taper the command to the final
        # altitude; XY remains position controlled throughout the ascent.
        remaining=self.target_z-self.core.pose.p[2]
        return min(self.o['climb_speed_mps'],max(0.,remaining))

    def takeoff_minimum_elapsed(self):
        return 1. if getattr(self.full_chain,'scope',None)=='window_pass_two_targets' else 3.

    def takeoff_arrival(self,i):
        c=self.core
        if not self.hover_heading_ready():return False
        if self.o['vendor_takeoff_arrival_enabled']:
            # Vendor uav_mission.cpp: z within .20 and 3s after last request.
            # ARM time is captured before requesting arming; airborne still
            # must be observed, and no time-only fallback certifies arrival.
            return (abs(c.pose.p[2]-self.target_z)<.20 and c.airborne and
                    self.full_height_bounds(c.pose.p)[0]>=self.o['minimum_bottom_height_m'] and
                    self.full_height_bounds(self.position)[0]>=self.o['minimum_bottom_height_m'] and
                    self.arm_request_wall is not None and i.wall-self.arm_request_wall>self.takeoff_minimum_elapsed())
        return (abs(c.pose.p[2]-self.target_z)<=c.c['align_tolerance_m'] and
                c.pose_speed<=c.c['stable_speed_mps'] and c.airborne and
                (not self.transient_takeoff() or self.precise_takeoff_ready()))

    def update_takeoff_progress(self,i):
        # Diagnostic evidence only; same values/thresholds as TAKEOFF gating.
        c=self.core
        precise=not self.transient_takeoff() or self.precise_takeoff_ready()
        criteria=dict(height=abs(c.pose.p[2]-self.target_z)<=c.c['align_tolerance_m'],
                      speed=c.pose_speed<=c.c['stable_speed_mps'],
                      airborne=bool(c.airborne),precise_position_and_attitude=precise)
        if self.o['vendor_takeoff_arrival_enabled']:
            criteria=dict(height=abs(c.pose.p[2]-self.target_z)<.20,airborne=bool(c.airborne),existing_flight_height_floor=self.full_height_bounds(c.pose.p)[0]>=self.o['minimum_bottom_height_m'] and self.full_height_bounds(self.position)[0]>=self.o['minimum_bottom_height_m'],elapsed_since_arm_request=self.arm_request_wall is not None and i.wall-self.arm_request_wall>self.takeoff_minimum_elapsed())
        if self.hover_target is not None:criteria['hover_heading_recovered']=self.hover_heading_ready()
        elapsed=0. if self.stable_since is None else max(0.,i.wall-self.stable_since)
        if not all(criteria.values()) and self.stable_since is not None:
            self.takeoff_stability_resets+=1
        self.takeoff_stable_max_s=max(self.takeoff_stable_max_s,elapsed)
        self.takeoff_progress=dict(original_target_position_local_m=(self.origin[0],self.origin[1],self.target_z),
            actual_position_local_m=c.pose.p,height_error_m=self.target_z-c.pose.p[2],
            arrival_policy='vendor_height_band_and_elapsed_time' if self.o['vendor_takeoff_arrival_enabled'] else 'precision_stability',
            height_tolerance_m=.20 if self.o['vendor_takeoff_arrival_enabled'] else c.c['align_tolerance_m'],position_difference_speed_mps=c.pose_speed,
            stable_speed_limit_mps=None if self.o['vendor_takeoff_arrival_enabled'] else c.c['stable_speed_mps'],criteria=criteria,
            blocking_criteria=[k for k,v in criteria.items() if not v],
            required_continuous_stable_s=0. if self.o['vendor_takeoff_arrival_enabled'] else self.o['stable_time_s'],
            stable_duration_before_current_check_s=elapsed,maximum_stable_duration_s=self.takeoff_stable_max_s,
            stability_reset_count=self.takeoff_stability_resets,ros=i.ros)

    def record_search_evidence(self,i,geometry_ready,reason):
        c=self.core
        metric=getattr(c,'metric_input',None) or {}
        runtime=metric.get('runtime') or {}
        row=dict(ros=i.ros,reason=reason,geometry_ready=bool(geometry_ready),
                 live_bound=bool(getattr(c,'live_bound',False)),
                 geometry_reason=getattr(c,'live_reason',None),
                 acquisition_confirmations=len(getattr(c,'acquisition_rows',[])),
                 legacy_rgb_target_fresh=c.target_fresh(i.ros,i.wall),
                 legacy_rgb_confirmed=c.confirmed(),legacy_rgb_confirmation_samples=len(c.samples),
                 legacy_rgb_observation_present=c.obs is not None,
                 position_difference_speed_mps=c.pose_speed,stable_speed_limit_mps=c.c['stable_speed_mps'],
                 rgbd_target_count=metric.get('target_count'),rgbd_ready=metric.get('ready'),
                 metric_geometry_ready=metric.get('metric_geometry_ready'),
                 metric_geometry_reason=metric.get('metric_geometry_reason'),
                 result_fresh=runtime.get('result_fresh'),source_stamp=metric.get('source_stamp'))
        if not self.search_evidence or self.search_evidence['reason']!=reason:
            self.search_events.append(dict(row));self.search_events=self.search_events[-60:]
        self.search_evidence=row

    def output(self, action=None):
        position = None if self.phase in TERMINAL or self.phase in ('IDLE','LANDING','AUTO_LANDING','LANDING_LIGHT_OFF') else self.position
        if self.phase=='LANDING_HANDOVER' and self.return_journey and self.return_journey.descent_observed:
            position=None
        hover=(self.hover_target.status() if self.hover_target else None)
        if hover and self.phase!='HOVER':
            hover['request_on']=False
            if self.phase in ('FAULT','TAKEOVER','REFUSED'):hover['completed']=False
        return dict(recovery=self.recovery.status(), landing_illumination=self.return_journey.light_status(self) if self.return_journey else None, full_chain=self.full_chain.status(self) if self.full_chain else None, hover_target=hover, phase=self.phase, reason=self.reason, position=position,
                    yaw=self.heading, action=action,
                    vertical_velocity_mps=self.takeoff_vertical_velocity(),
                    velocity_local_mps=(self.pass_velocity if self.phase=='PASS' else self.station_velocity if self.phase=='POSTPASS' else self.return_journey.velocity if self.return_journey and self.phase in ('RETURN_PREP','RETURN_HOME') else None) if position is not None else None,
                    observed_tilt_rad=tilt(self.core.pose.q) if self.core.pose else None,
                    effective_tilt_limit_rad=self.attitude_limit(),
                    passage_geometry_tilt_limit_rad=self.core.c.get('_session_pass_tilt_limit_rad',self.core.c['tilt_limit_rad']),
                    observed_yaw_rad=yaw(self.core.pose.q) if self.core.pose else None,
                    heading_error_rad=wrap(yaw(self.core.pose.q)-self.heading) if self.core.pose and self.heading is not None else None,
                    attitude_checks_enabled=self.attitude_checks_enabled(),
                    heading_limit_rad=self.heading_limit(),
                    heading_error_gate_enabled=self.heading_error_gate_enabled(),
                    hover_takeoff_heading_tolerance_active=self.hover_takeoff_heading_tolerance(),
                    takeoff_transient_tolerance_active=self.transient_takeoff() and not self.simplified_takeoff(),
                    simplified_takeoff_active=self.simplified_takeoff(),
                    session_window=dict(enabled=bool(self.session_window),localization_epoch=self.localization_epoch,
                        alignment_policy=self.session_alignment_policy,
                        clear_column_radius_m=(None if self.session_policy.get('field_bounded_alignment') else self.session_policy.get('alignment_clear_column_radius_m')),
                        field_bounded_alignment=self.session_policy.get('field_bounded_alignment',False),field_alignment_last_rejection=getattr(self,'field_alignment_last_rejection',None),
                        departure_clearance_gate=getattr(self,'departure_clearance_gate',None),
                        alignment_tracking_limit_m=self.core.c['max_tracking_error_m'],
                        pass_cross_track_limit_m=self.core.c.get('_session_pass_cross_track_limit_m',self.core.c['clearance_tracking_budget_m']),
                        invalidated=self.session_invalidated,
                        acquisition=self.session_window.reason if self.session_window else None,
                        candidate_evidence=self.session_window.candidate_evidence if self.session_window else None,
                        candidate_history=list(self.session_window.candidate_history) if self.session_window and self.phase in TERMINAL else None,
                        frozen_geometry=self.session_window.snapshot if self.session_window else None),
                    takeoff_precise_ready=self.precise_takeoff_ready() if self.origin is not None else False,
                    experiment='takeoff_land' if self.takeoff_only else ('align_only' if self.align_only else ('straight_pass' if self.direct_pass else 'traverse')),
                    alignment_completed=self.alignment_completed,
                    straight_pass_completed=self.direct_pass_completed,
                    passage_decision=(self.session_window.decision.snapshot() if self.session_window else self.passage_decision.snapshot()) if self.direct_pass else None,
                    frozen_straight_route=self.direct_plan,
                    terminal_capture=self.terminal_capture.status() if self.terminal_capture else None,
                    prepass_adjustment=dict(policy=self.session_alignment_policy if self.session_window else self.adjustment_policy,
                        reason=('session_bounded_reference_tracking' if self.session_window else self.adjust_reason),
                        attempts=self.adjust_count,completed=self.adjust_completed,actual_path_m=self.adjust_path_m,
                        actual_displacement_from_start_m=(norm(sub(self.core.pose.p,self.adjust_plan['start'])) if self.adjust_plan and self.core.pose else None),
                        candidate=self.adjust_candidate,plan=self.adjust_plan,
                        confirmation=self.adjust_decision.snapshot() if self.adjust_decision else None),
                    window_traversal_permitted=not self.takeoff_only and not self.align_only,
                    takeoff_hover_completed=self.takeoff_hover_completed,
                    pilot_landing_required=self.o['pilot_landing_enabled'] and self.core.armed and
                        (not self.return_journey or self.phase in ('FAULT','TAKEOVER','HANDOVER_REQUEST','PILOT_CONTROL')),
                    handover_confirmed=self.phase=='PILOT_CONTROL',
                    last_command_feedback=self.last_command_feedback,
                    tracking=self.tracking_snapshot(), fault_snapshot=self.fault_snapshot,
                    height=self.height_snapshot(),handover_snapshot=self.handover_snapshot,
                    startup_checks=self.startup_checks, takeoff_progress=self.takeoff_progress,
                    search_evidence=self.search_evidence,search_events=self.search_events,
                    estimator=self.last_estimator_evidence)

    def height_snapshot(self):
        if self.ground_z is None or self.core.pose is None:return None
        actual=self.full_height_bounds(self.core.pose.p)[0]
        planned=self.full_height_bounds(self.position)[0] if self.position is not None else None
        return dict(ground_z_local_m=self.ground_z,actual_bottom_height_m=actual,
                    planned_bottom_height_m=planned,minimum_bottom_height_m=self.o['minimum_bottom_height_m'],
                    actual_floor_margin_m=actual-self.o['minimum_bottom_height_m'],
                    takeoff_reference_bottom_height_m=self.o['target_bottom_height_m'],
                    takeoff_reference_is_alignment_requirement=False)

    def full_height_bounds(self, p):
        # Lowest/highest rotated corner, not just the horizontal body centre.
        c = self.core.c
        if self.session_window:
            from attitude_envelope import cylinder_axis_support
            axis=rotate(self.core.pose.q,(0.,0.,1.))
            return (p[2]-cylinder_axis_support(c,(0,0,-1),axis)-self.ground_z,
                    p[2]+cylinder_axis_support(c,(0,0,1),axis)-self.ground_z)
        center = c['envelope_center_body_m']
        half = [v/2 for v in c['vehicle_size_m']]
        zs = [p[2]+rotate(self.core.pose.q,add(center,(sx*half[0],sy*half[1],sz*half[2])))[2]
              for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)]
        return min(zs)-self.ground_z, max(zs)-self.ground_z

    def corridor(self, p):
        from envelope_fence import envelope_inside
        control=self.core.c
        if (self.return_journey and self.return_journey.leg==2 and
                self.phase in ('SEARCH','ADJUST','PASS')):
            control=self.session_window.scoped_control(control)
        if self.return_journey:
            return self.return_journey.field_inside(self,control,p)
        return envelope_inside(control,p,self.core.pose.q)

    def attitude_limit(self):
        if self.transient_takeoff():return math.radians(15)
        c=self.core.c
        # Return translation uses the same cylinder cone already included by
        # attitude_envelope.support. Keep the normal hold/turn/landing limit.
        if (self.return_journey and self.return_journey.active and self.session_window and
                '_session_cylinder_radius_m' in c and self.phase in ('RETURN_PREP','RETURN_HOME')):
            return max(c['tilt_limit_rad'],min(math.radians(15),c.get('_session_pass_tilt_limit_rad',c['tilt_limit_rad'])))
        # Passage geometry is computed for this entire cone before commitment.
        if (self.phase=='PASS' or (self.phase=='POSTPASS' and getattr(self.full_chain,'scope',None)=='window_pass_two_targets' and self.full_chain.stage=='STATION_TRANSFER')):return c.get('_session_pass_tilt_limit_rad',c['tilt_limit_rad'])
        return c['tilt_limit_rad']

    def common_error(self, i):
        c = self.core
        if not i.state_fresh or not c.connected or not c.fresh('state',i.ros,i.wall,c.c['state_timeout_s']): return 'state_unavailable'
        if not i.landed_fresh or not c.fresh('extended',i.ros,i.wall,c.c['state_timeout_s']): return 'landed_state_unavailable'
        if not i.rc_ok: return 'rc_unavailable'
        if not i.graph_ok: return 'control_graph_untrusted'
        if not c.fresh('pose',i.ros,i.wall,c.c['pose_timeout_s']): return 'pose_unavailable'
        if not i.estimator_ok: return i.estimator_reason or 'estimator_unhealthy'
        if self.session_window:
            from session_window import localization_identity
            if localization_identity(i.localization_identity)!=self.session_window.identity:
                return 'localization_identity_changed'
            if type(i.localization_epoch) is not int or i.localization_epoch<0:
                return 'localization_epoch_unavailable'
            if self.localization_epoch is not None and i.localization_epoch!=self.localization_epoch:
                return 'localization_epoch_changed'
            if getattr(c,'session_pose_fault',None):return c.session_pose_fault
        tilt_limit=self.attitude_limit()
        yaw_limit=self.heading_limit()
        if self.attitude_checks_enabled() and tilt(c.pose.q)>tilt_limit: return 'attitude_limit'
        guard=getattr(self.full_chain,'transfer_heading',None)
        if self.heading_error_gate_enabled() and guard and guard.scoped(self):
            heading_error=guard.check(self,i)
            if heading_error:return heading_error
        heading_adjustment=bool(self.return_journey and self.return_journey.heading_adjustment(self))
        if self.heading_error_gate_enabled() and self.attitude_checks_enabled() and not heading_adjustment and self.heading is not None and abs(wrap(yaw(c.pose.q)-self.heading))>yaw_limit:
            return 'heading_changed'
        return ''

    def start(self, i):
        if self.phase != 'IDLE': raise ValueError('one_session_only')
        c = self.core
        self.last_estimator_evidence=i.estimator_evidence
        error = self.common_error(i)
        if error or i.emergency or c.armed or not i.landed or c.mode not in ('MANUAL','ALTCTL','POSCTL') or (self.o['companion_battery_monitor_enabled'] and not i.battery_ok):
            self.transition('REFUSED',error or 'ground_preflight_failed',i.wall)
            return self.output()
        if not self.takeoff_only and not c.fresh('image',i.ros,i.wall,c.c['image_timeout_s']):
            self.transition('REFUSED','camera_stream_unavailable',i.wall)
            return self.output()
        if c.pose_speed>c.c['stable_speed_mps']:
            self.transition('REFUSED','ground_not_stationary',i.wall)
            return self.output()
        if self.session_window:self.localization_epoch=i.localization_epoch
        self.origin = c.pose.p
        self.position, self.heading = c.pose.p,yaw(c.pose.q)
        self.takeoff_heading=self.heading
        self.ground_z = c.pose.p[2]-self.o['ground_fcu_height_m']
        self.target_z = self.ground_z+self.o['bottom_to_fcu_m']+self.o['target_bottom_height_m']
        target = (self.origin[0],self.origin[1],self.target_z)
        # Reject unsafe startup geometry before arming, including both endpoint
        # and landing column. Convex fence makes the intervening segment valid.
        self.startup_checks=dict(ascending_target=self.target_z>self.origin[2],
                                 start_envelope_inside=self.corridor(self.origin),
                                 target_envelope_inside=self.corridor(target),
                                 target_control_envelope_inside=c.inside(target))
        if not all(self.startup_checks.values()):
            self.transition('REFUSED','takeoff_column_or_search_position_outside_fence',i.wall)
            return self.output()
        self.began = self.last_tick = i.wall
        self.transition('PRESTREAM','ground_checks_passed',i.wall)
        return self.output()

    def fail(self, reason, i):
        # Losing pose/RC/ownership cannot be repaired by sending a landing mode
        # based on untrusted state. Stop output; verified FCU failsafe takes over.
        if self.fault_snapshot is None:
            self.fault_snapshot=dict(phase=self.phase, reason=reason, wall=i.wall, ros=i.ros,
                                     tracking=self.tracking_snapshot(), estimator=i.estimator_evidence,
                                     observed_tilt_rad=tilt(self.core.pose.q) if self.core.pose else None,
                                     effective_tilt_limit_rad=self.attitude_limit())
        self.core.authorized = False
        if self.session_window:self.session_invalidated=reason
        self.transition('FAULT',reason+'_FCU_failsafe_or_pilot_required',i.wall)
        return self.output()

    def handover(self, reason, i):
        if self.full_chain and self.phase=='HOVER' and self.hover_target.phase not in ('DONE','ABORT'):
            if self.hover_target.phase!='OFF_WAIT':self.hover_target.invalid(reason,i.wall)
            return self.hover_target.tick(self,i,0.)
        c=self.core
        if not c.armed:
            self.transition('REFUSED',reason+'_before_arming',i.wall)
            return self.output()
        if self.handover_snapshot is None:
            self.handover_snapshot=dict(phase=self.phase,reason=reason,ros=i.ros,wall=i.wall,
                height=self.height_snapshot(),tracking=self.tracking_snapshot())
        # Still use position hold while requesting the mode switch. Never
        # claim pilot control merely because the service accepted a request.
        self.position=c.pose.p
        c.authorized=False
        self.transition('HANDOVER_REQUEST',reason+'_pilot_landing_required',i.wall)
        return self.output('POSCTL')

    def land(self, reason, i, completed=False):
        if self.o['pilot_landing_enabled']:
            if not self.corridor(self.core.pose.p):
                return self.fail(reason+'_handover_position_outside_fence',i)
            return self.handover(reason,i)

        c = self.core
        if not c.armed:
            self.transition('REFUSED',reason+'_before_arming',i.wall)
            return self.output()
        # Never initiate descent while inside/near the window on a failed pass.
        if not completed and norm(sub(c.pose.p,self.origin)[:2])>self.o['landing_radius_m']:
            return self.fail(reason+'_outside_verified_takeoff_landing_column',i)
        touchdown=(c.pose.p[0],c.pose.p[1],self.ground_z+self.o['ground_fcu_height_m'])
        if not self.corridor(touchdown):return self.fail(reason+'_landing_column_outside_fence',i)
        self.position = c.pose.p
        self.landing_site = c.pose.p
        self.core.authorized = False
        self.transition('LAND_REQUEST',reason,i.wall)
        return self.output('LAND')

    def consider_adjustment(self,i,status):
        from straight_pass import PassageDecision
        from prepass_adjustment import suggest,validate_motion
        from dataclasses import replace
        c=self.core
        if (not self.adjustment_policy['enabled'] or self.adjust_count or
                (self.passage_decision.final_rejection and self.passage_decision.reason!='fixed_height_heading_box_would_clip_window')):
            return self.handover('straight_route_rejected:'+self.passage_decision.reason,i)
        if self.adjust_candidate is None:
            try:
                self.adjust_candidate=suggest(status,c.c,c.pose,i.ros,i.wall,c.metric_received,self.search_started_ros,
                    self.ground_z,self.o['minimum_bottom_height_m'],self.origin,self.o['landing_radius_m'],self.adjustment_policy)
            except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
                self.adjust_reason=str(exc)
                return self.handover('prepass_adjustment_unavailable:'+str(exc),i)
            self.adjust_decision=PassageDecision();self.adjust_reason='confirming_correction_route_3_of_5'
            # Pause at actual height while checking the proposed translation.
            self.position=c.pose.p
        plan=self.adjust_candidate;heading=plan['yaw_rad']
        q=(0.,0.,math.sin(heading/2),math.cos(heading/2))
        hypothetical=replace(c.pose,p=tuple(plan['target']),q=q)
        ready=self.adjust_decision.update(status,c.c,hypothetical,i.ros,i.wall,c.metric_received,self.search_started_ros)
        if self.adjust_decision.final_rejection:
            self.adjust_reason=self.adjust_decision.reason
            return self.handover('proposed_adjustment_route_rejected:'+self.adjust_reason,i)
        if not ready:return None
        try:
            if self.heading_error_gate_enabled() and abs(wrap(yaw(c.pose.q)-heading))>c.c['yaw_limit_rad']:raise ValueError('adjustment_heading_changed_before_commit')
            validate_motion(plan,c.c,c.pose,status,i.ros,i.wall,c.metric_received,self.search_started_ros,
                self.ground_z,self.o['minimum_bottom_height_m'],self.origin,self.o['landing_radius_m'],self.adjustment_policy)
            distance=norm(sub(plan['target'],c.pose.p))
            if not .002<distance<=min(self.adjustment_policy['max_distance_m'],self.adjustment_policy['max_path_m'],c.c['max_align_distance_m'],c.c['max_align_path_m']):raise ValueError('adjustment_distance_changed_before_commit')
            duration=distance/min(self.adjustment_policy['speed_mps'],c.c['align_speed_mps'])+self.adjustment_policy['settle_s']
            if duration+self.adjust_decision.plan['expected_duration_s']+self.o['search_timeout_s']>=self.o['max_mission_time_s']-(i.wall-self.began):raise ValueError('adjustment_remaining_time_insufficient')
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            self.adjust_reason=str(exc);return self.handover('adjustment_commit_rejected:'+str(exc),i)
        self.adjust_plan=dict(plan,start=list(c.pose.p),length_m=distance)
        self.adjust_count=1;self.adjust_reason='bounded_translation_committed'
        self.adjust_path_m=0.;self.adjust_last_position=c.pose.p
        self.position=c.pose.p;self.heading=heading
        self.transition('ADJUST','one_measured_lateral_vertical_correction',i.wall)
        return self.output()

    def acquire_session_window(self,i,dt=0.):
        c=self.core;acq=self.session_window
        second_search=bool(self.return_journey and self.return_journey.search_active(self))
        if second_search and not all(self.return_journey.region(self,p,1) for p in (c.pose.p,self.position)):
            if self.recovery.begin_prepass(self,i,'second_search_predicted_corridor'):
                return self.output()
            return self.handover('second_search_outside_right_corridor',i)
        if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
            return self.handover('session_search_height_floor',i)
        if (not second_search and self.o['minimum_bottom_height_m']<.8 and self.position[2]>c.pose.p[2] and
                norm(sub(self.position,c.pose.p))>c.c['max_tracking_error_m']):
            if i.wall-self.entered>=self.o['search_timeout_s']:
                return self.handover('search_climb_reference_timeout',i)
            self.reason='continuing_climb_before_lateral_adjustment'
            return self.output()
        if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
            return self.handover('session_search_height_floor',i)
        if not second_search and i.wall-self.entered>=self.o['search_timeout_s']:
            if self.recovery.holding_allowed(self,c.pose.p) and self.recovery.take(self,i,'search','acquisition_timeout'):
                self.search_started_ros=i.ros
                self.transition('SEARCH','retry_fresh_window_acquisition',i.wall)
                return self.output()
            return self.handover('session_acquisition_timeout:'+acq.reason,i)
        try:
            if self.return_journey:self.return_journey.filter_geometry(self,c.metric_input or {})
            self.recovery.check_geometry((c.metric_input or {}).get('metric_geometry') or {})
            ready=acq.update(c.metric_input or {},c.c,c.pose,i.ros,i.wall,c.metric_received,
                self.search_started_ros,self.ground_z,self.o['minimum_bottom_height_m'],self.origin,self.session_policy['alignment_clear_column_radius_m'],
                self.session_alignment_policy,self.position)
        except (ValueError,TypeError,KeyError,IndexError,ZeroDivisionError) as exc:
            acq.reason=str(exc);acq.reject_frame(c.metric_input or {},i.ros,str(exc))
            self.reason='session_acquisition:'+str(exc)
            # Bad/stale/no frames cannot authorize translation or passage.
            # Second-search yaw scanning stays within the whole-mission budget.
            if second_search:self.return_journey.scan(self,dt,i.ros)
            return self.output()
        self.reason=acq.reason
        if not ready:
            if second_search:self.return_journey.scan(self,dt,i.ros)
            return self.output()
        acq.snapshot['localization_epoch']=self.localization_epoch
        self.recovery.expected_geometry=None
        self.adjust_plan=acq.plan;self.adjust_count=1
        self.adjust_path_m=0.;self.adjust_last_position=c.pose.p
        # A confirmed second-opening plan starts at the measured pose, so a
        # post-turn drift does not fault against the old SEARCH hold reference.
        # First-opening acquisition retains the ongoing climb reference.
        if second_search:self.position=tuple(c.pose.p)
        if self.session_policy.get('field_bounded_alignment'):
            from pass_terminal import MotionWindow
            self.field_alignment_motion=MotionWindow();self.field_alignment_replan_stamp=None;self.field_alignment_last_rejection=None
        self.heading=acq.plan['yaw_rad']
        self.transition('ADJUST','one_session_geometry_centering',i.wall)
        return self.output()

    def session_departure_route(self):
        plan=self.session_window.route(self.core.c,self.core.pose,self.position) if self.session_policy.get('fixed_pass_distance_m') else self.session_window.route(self.core.c,self.core.pose)
        if not (self.return_journey and self.return_journey.leg==2 and
                self.session_policy.get('field_bounded_alignment')):
            return plan
        # The 1 cm geometric minimum is not enough reason to abandon a wider
        # feasible alignment. Compare at the current yaw against the same frozen
        # opening, and accept most of the available margin without exact centering.
        from window_pass.geometry import vector
        def margin(route):
            return min(e['clearance_after_budget_m']/norm(e['translation_gradient_local'])
                       for e in route['edge_diagnostics'])
        current=margin(plan)
        gate=dict(current_margin_m=current,required_margin_m=None,available_margin_m=None,
                  policy='eighty_percent_of_feasible_margin_capped_by_cross_track_limit',ready=True)
        self.departure_clearance_gate=gate
        target=(self.adjust_plan or {}).get('target')
        if not vector(target):
            gate['candidate_reason']='alignment_target_unavailable'
            return plan
        try:
            alternative=self.session_window.route(self.core.c,replace(self.core.pose,p=tuple(target)))
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            # An obsolete/unreachable target must not block a valid departure.
            gate['candidate_reason']=str(exc)
            return plan
        available=margin(alternative)
        required=min(plan['live_cross_track_abort_m'],.8*available)
        gate.update(available_margin_m=available,required_margin_m=required,ready=current+1e-9>=required)
        if not gate['ready']:
            raise ValueError('second_window_departure_clearance_not_ready')
        return plan

    def commit_session_route(self,i):
        try:
            plan=self.session_departure_route()
            if plan['expected_duration_s']>=self.o['max_mission_time_s']-(i.wall-self.began):
                raise ValueError('session_remaining_time_insufficient')
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            if str(exc)=='second_window_departure_clearance_not_ready':
                self.reason=str(exc)
                return self.output()
            if (isinstance(exc,ValueError) and str(exc)!='session_remaining_time_insufficient' and
                    self.recovery.begin_prepass(self,i,'departure:'+str(exc))):
                return self.output()
            return self.handover('session_departure_rejected:'+str(exc),i)
        plan['localization_epoch']=self.localization_epoch
        self.direct_plan=plan;self.position=tuple(plan['start']);self.heading=plan['yaw_rad']
        self.adjust_completed=True;self.alignment_completed=True
        self.transition('PASS','session_route_frozen_localization_feedback_only',i.wall)
        return self.output()

    def align_session_window(self,i,dt):
        from session_window import validate_motion,advance_alignment
        c=self.core;policy=self.session_alignment_policy
        if self.return_journey and self.return_journey.leg==2 and not self.return_journey.region(self,c.pose.p,1):
            if self.recovery.begin_prepass(self,i,'second_alignment_predicted_corridor'):
                return self.output()
            return self.handover('second_alignment_left_right_corridor',i)
        if self.return_journey and self.return_journey.leg==2 and policy.get('field_bounded_alignment'):
            # Prefer a feasible measured departure over further precision
            # centering. Recheck both actual and retained-height rays, using
            # the frozen opening, before validating an obsolete target.
            self.field_alignment_motion.update(c.pose.stamp,c.pose.p)
            if (min(self.full_height_bounds(p)[0] for p in (c.pose.p,self.position))>=self.o['minimum_bottom_height_m'] and
                    self.return_journey.region(self,self.position,1) and self.field_alignment_motion.stable(.12,.08)):
                try:self.session_departure_route()
                except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:self.field_alignment_last_rejection=str(exc)
                else:return self.commit_session_route(i)
        self.adjust_path_m+=norm(sub(c.pose.p,self.adjust_last_position));self.adjust_last_position=c.pose.p
        if policy.get('cumulative_path_gate_enabled',True) and self.adjust_path_m>min(policy['max_path_m'],c.c['max_align_path_m']):
            return self.handover('session_alignment_path_limit',i)
        try:
            scoped_c=self.session_window.scoped_control(c.c) if self.return_journey and self.return_journey.leg==2 else c.c
            validate_motion(self.adjust_plan,scoped_c,c.pose,self.ground_z,self.o['minimum_bottom_height_m'],
                self.origin,self.session_policy['alignment_clear_column_radius_m'],policy)
            self.position,arrived=advance_alignment(self.adjust_plan,scoped_c,c.pose,self.position,dt,i.wall-self.entered,policy)
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            if isinstance(exc,ValueError) and self.recovery.begin_prepass(self,i,'alignment:'+str(exc)):
                return self.output()
            return self.handover('session_alignment:'+str(exc),i)
        if self.return_journey and self.return_journey.leg==2 and not self.return_journey.region(self,self.position,1):
            if self.recovery.begin_prepass(self,i,'second_alignment_reference_corridor'):
                return self.output()
            return self.handover('second_alignment_reference_outside_right_corridor',i)
        if min(self.full_height_bounds(p)[0] for p in (c.pose.p,self.position))<self.o['minimum_bottom_height_m']:
            return self.handover('session_alignment_height_floor',i)
        if policy.get('field_bounded_alignment'):
            self.field_alignment_motion.update(c.pose.stamp,c.pose.p)
            if self.field_alignment_motion.stable(.12,.08):
                try:self.session_departure_route()
                except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:self.field_alignment_last_rejection=str(exc)
                else:return self.commit_session_route(i)
            # Keep correcting against the same confirmed opening if feedback
            # has moved. Never publish a second takeoff or reanchor the origin.
            if self.field_alignment_replan_stamp is None or c.pose.stamp-self.field_alignment_replan_stamp>=.3:
                try:
                    self.adjust_plan=self.session_window.retarget(c.c,c.pose,self.position,self.ground_z,self.o['minimum_bottom_height_m'],self.origin,policy)
                    self.field_alignment_replan_stamp=c.pose.stamp
                except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:self.field_alignment_last_rejection=str(exc)
            self.reason='field_geometry_alignment_until_departure_feasible'
            return self.output()
        if self.session_policy.get('smooth_pass') and c.pose_speed>c.c['stable_speed_mps']:
            self.adjust_arrival_since=None
            return self.output()
        if arrived and not policy.get('arrival_speed_gate_enabled',True):return self.commit_session_route(i)
        if arrived and c.pose_speed<=c.c['stable_speed_mps']:
            if self.adjust_arrival_since is None:self.adjust_arrival_since=i.wall
            if i.wall-self.adjust_arrival_since>=policy['settle_s']:return self.commit_session_route(i)
        else:self.adjust_arrival_since=None
        return self.output()

    def tick(self, i):
        if self.phase in TERMINAL or self.phase=='IDLE': return self.output()
        self.last_estimator_evidence=i.estimator_evidence
        c = self.core
        if self.align_only and self.phase=='PASS':return self.fail('alignment_only_illegal_pass_phase',i)
        if self.takeoff_only and self.phase in ('SEARCH','ALIGN','PASS'):
            return self.fail('takeoff_test_illegal_visual_phase',i)
        dt = i.wall-self.last_tick
        self.last_tick = i.wall
        if dt<0 or dt>c.c['max_loop_gap_s']:
            return self.fail('loop_deadline',i)
        if self.return_journey and self.phase in ('AUTO_LANDING','LANDING_LIGHT_OFF'):
            return self.return_journey.landing_control_tick(self,i) if self.phase=='AUTO_LANDING' else self.return_journey.landing_tick(self,i)
        if i.emergency:
            self.transition('TAKEOVER','emergency_latched',i.wall)
            c.authorized=False
            return self.output()
        if i.state_fresh and c.connected:
            allowed = {'PRESTREAM':('MANUAL','ALTCTL','POSCTL'), 'MODE':('MANUAL','ALTCTL','POSCTL','OFFBOARD'),
                       'LAND_REQUEST':('OFFBOARD','AUTO.LAND'), 'LANDING':('AUTO.LAND',),
                       'LANDING_HANDOVER':('OFFBOARD','AUTO.LAND'),
                       'HANDOVER_REQUEST':('OFFBOARD','POSCTL')}.get(self.phase,('OFFBOARD',))
            if c.mode not in allowed:
                self.transition('TAKEOVER','mode_exit_never_reacquire',i.wall)
                c.authorized=False
                return self.output()
        error=self.common_error(i)
        if error:
            if self.session_window and self.phase in ('SEARCH','ADJUST') and error in ('attitude_limit','heading_changed'):
                return self.handover('prepass_operating_attitude_not_ready:'+error,i)
            return self.fail(error,i)
        if self.command_result is not None:
            command,accepted=self.command_result
            self.command_result=None
            if not accepted:
                feedback=self.last_command_feedback or {}
                detail=('service_exception' if feedback.get('error') else
                        ('rejected_mav_result_'+str(feedback['mav_result']) if feedback.get('mav_result') is not None else 'service_rejected_or_failed'))
                return self.fail(command+'_'+detail,i)
        if self.return_journey and self.return_journey.handover_descent(self,i):
            # LANDING may precede AUTO.LAND mode feedback. PX4 has begun descent;
            # keep the light alive without applying airborne/hold-height guards
            # or sending the old altitude target against that descent.
            self.return_journey.descent_observed=True
            return self.return_journey.landing_control_tick(self,i)
        if self.return_journey and self.phase in ('RETURN_PREP','YAW_TURN','RETURN_HOME','LANDING_LIGHT_ON','LANDING_HANDOVER'):
            if not c.armed or not c.airborne:return self.fail('return_unexpected_disarm_or_landed',i)
            if i.wall-self.began>self.o['max_mission_time_s']:return self.fail('return_mission_timeout',i)
            if not self.corridor(c.pose.p) or not self.corridor(self.position):return self.fail('return_outside_field_fence',i)
            tracking_limit=(self.return_journey.p['return_tracking_error_limit_m']
                            if self.phase=='RETURN_HOME' else c.c['max_tracking_error_m'])
            enforce_tracking=self.phase not in ('RETURN_PREP','RETURN_HOME','YAW_TURN') or (self.phase in ('RETURN_PREP','RETURN_HOME') and self.return_journey.p['movement_tracking_error_abort_enabled'])
            if enforce_tracking and norm(sub(self.position,c.pose.p))>tracking_limit:return self.fail('return_tracking_error',i)
            if min(self.full_height_bounds(p)[0] for p in (c.pose.p,self.position))<self.o['minimum_bottom_height_m']:return self.fail('return_height_floor',i)
            return self.return_journey.tick(self,i,dt)
        if self.phase=='HANDOVER_REQUEST':
            if c.mode=='POSCTL':
                self.transition('PILOT_CONTROL','POSCTL_observed_pilot_must_land',i.wall)
                return self.output()
            if i.wall-self.entered>self.o['mode_timeout_s']:
                return self.fail('POSCTL_handover_not_observed',i)
            return self.output()
        if self.phase in ('LAND_REQUEST','LANDING'):
            if i.landed and not c.armed:
                self.transition('DONE','landed_and_disarmed_observed',i.wall)
            elif i.wall-self.entered>self.o['landing_timeout_s']:
                return self.fail('landing_not_completed',i)
            elif c.mode=='AUTO.LAND' and self.phase=='LAND_REQUEST':
                self.transition('LANDING','AUTO_LAND_observed',i.wall)
            elif self.phase=='LAND_REQUEST' and i.wall-self.entered>self.o['mode_timeout_s']:
                return self.fail('landing_mode_not_observed',i)
            return self.output()
        if self.phase in ('TAKEOFF','HOVER','SEARCH','ADJUST','ALIGN','PASS','POSTPASS') and not c.armed:
            return self.fail('unexpected_disarm',i)
        if self.phase in ('HOVER','SEARCH','ADJUST','ALIGN','PASS','POSTPASS') and not c.airborne:
            return self.fail('airborne_state_lost',i)
        if self.phase not in ('PRESTREAM','MODE','ARM') and self.o['companion_battery_monitor_enabled'] and not i.battery_ok:
            return self.land('battery_low',i)
        if i.wall-self.began>self.o['max_mission_time_s']:
            return self.land('mission_timeout',i)
        if self.recovery.active:
            return self.recovery.tick(self,i,dt)
        if (self.phase=='ADJUST' and self.session_window and
                norm(sub(self.position,c.pose.p))>c.c['max_tracking_error_m'] and
                self.recovery.begin_prepass(self,i,'alignment_tracking_lag')):
            return self.output()
        if self.phase=='POSTPASS' and hasattr(self.full_chain,'recover_transfer'):
            limit=.35 if self.full_chain.stage=='STATION_TRANSFER' else c.c['max_tracking_error_m']
            if norm(sub(self.position,c.pose.p))>limit and self.full_chain.recover_transfer(self,i,'tracking_lag'):
                return self.output()
        if self.position is not None:
            if self.position_goal_takeoff() and (self.phase=='TAKEOFF' or self.simplified_takeoff()):
                # Full height gap is intentional. The session profile relies
                # on live input validity and the actual site envelope here.
                if not self.simplified_takeoff() and norm(sub(self.position,c.pose.p)[:2])>self.horizontal_tracking_limit():
                    return self.fail('takeoff_horizontal_tracking_error',i)
                if not self.simplified_takeoff() and c.pose.p[2]-self.target_z>(.20 if self.transient_takeoff() else c.c['max_tracking_error_m']):
                    return self.fail('takeoff_height_overshoot',i)
                if not self.corridor(self.position):
                    return self.fail('planned_takeoff_goal_outside_fence',i)
            elif self.takeoff_only and self.phase=='HOVER' and self.horizontal_tracking_limit()!=c.c['max_tracking_error_m']:
                delta=sub(self.position,c.pose.p)
                if norm(delta[:2])>self.horizontal_tracking_limit() or abs(delta[2])>c.c['max_tracking_error_m']:
                    return self.fail('tracking_error',i)
            elif self.return_journey and self.return_journey.search_active(self):
                # Keep the post-turn hold reference; the surveyed corridor,
                # floor and whole-mission deadline supervise reacquisition.
                pass
            elif (self.session_window and self.o['minimum_bottom_height_m']<.8 and
                  self.phase in ('SEARCH','ADJUST') and c.pose.p[2]<self.position[2]<=self.target_z+1e-9):
                # Vendor arrival permits a remaining upward climb of <20cm.
                # Do not turn that intentional reference into a 12cm 3-D fault.
                if norm(sub(self.position,c.pose.p)[:2])>c.c['max_tracking_error_m'] or self.position[2]-c.pose.p[2]>.20:
                    return self.fail('tracking_error',i)
            elif self.phase=='HOVER' and getattr(self.full_chain,'scope',None)=='window_pass_two_targets':
                if norm(sub(self.position,c.pose.p))>.20:return self.fail('two_target_hover_tracking_error',i)
            elif self.phase=='POSTPASS' and getattr(self.full_chain,'scope',None)=='window_pass_two_targets' and self.full_chain.stage=='STATION_TRANSFER':
                if norm(sub(self.position,c.pose.p))>.35:return self.fail('two_target_transfer_tracking_error',i)
            elif self.phase=='PASS' and self.direct_plan and self.direct_plan.get('smooth_trajectory'):
                from smooth_pass import ALONG_ERROR
                from window_pass.geometry import dot
                from pass_terminal import TERMINAL_ALONG_ERROR
                terminal_clear=bool(self.terminal_capture and self.full_chain.clear_of_window(self,c.pose.p))
                if abs(dot(sub(self.position,c.pose.p),self.direct_plan['direction']))>(TERMINAL_ALONG_ERROR if terminal_clear else ALONG_ERROR):
                    return self.fail('smooth_pass_along_tracking_error',i)
            elif norm(sub(self.position,c.pose.p))>c.c['max_tracking_error_m']:
                return self.fail('tracking_error',i)
        if not self.corridor(c.pose.p): return self.fail('physical_envelope_outside_fence',i)
        if self.full_chain and self.phase in ('POSTPASS','HOVER') and not self.full_chain.allowed(self,c.pose.p):
            return self.fail('postpass_actual_left_confirmed_region',i)
        if self.takeoff_only and not self.simplified_takeoff() and norm(sub(c.pose.p,self.origin)[:2])>self.o['landing_radius_m']:
            return self.fail('takeoff_column_horizontal_limit',i)
        if self.phase=='PRESTREAM':
            if i.wall-self.entered>self.o['mode_timeout_s']+c.c['hold_prestream_s']:
                return self.fail('prestream_publication_timeout',i)
            if c.armed: return self.fail('unexpected_ground_arm',i)
            if i.wall-self.entered>=c.c['hold_prestream_s'] and self.sent_count>=math.ceil(c.c['hold_prestream_s']*c.c['control_hz']) and self.last_sent is not None and i.wall-self.last_sent<=c.c['max_loop_gap_s']:
                self.transition('MODE','prestream_sent_wait_OFFBOARD',i.wall)
                return self.output('OFFBOARD')
        elif self.phase=='MODE':
            if c.armed: return self.fail('armed_before_supervisor_request',i)
            if c.mode=='OFFBOARD':
                self.arm_request_wall=i.wall
                self.transition('ARM','OFFBOARD_observed',i.wall)
                return self.output('ARM')
            if i.wall-self.entered>self.o['mode_timeout_s']:
                return self.fail('OFFBOARD_not_observed',i)
        elif self.phase=='ARM':
            if c.armed:
                if self.position_goal_takeoff():
                    self.position=(self.origin[0],self.origin[1],self.target_z)
                self.transition('TAKEOFF','armed_observed',i.wall)
            elif i.wall-self.entered>self.o['mode_timeout_s']:
                return self.fail('arming_not_observed',i)
        elif self.phase=='TAKEOFF':
            self.update_takeoff_progress(i)
            if i.wall-self.entered>self.o['takeoff_timeout_s']:
                return self.land('takeoff_timeout',i)
            if not self.position_goal_takeoff():
                z=min(self.target_z,self.position[2]+self.o['climb_speed_mps']*dt,c.pose.p[2]+c.c['max_setpoint_lead_m'])
                self.position=(self.origin[0],self.origin[1],max(self.position[2],z))
            if self.takeoff_arrival(i):
                if self.stable_since is None:self.stable_since=i.wall
                if self.o['vendor_takeoff_arrival_enabled'] or i.wall-self.stable_since>=self.o['stable_time_s']:
                    if self.takeoff_only:
                        self.transition('HOVER','takeoff_stable_hold_without_perception',i.wall)
                    else:
                        if self.session_window:
                            # A loose arrival must not lower the climb reference
                            # to feedback near the floor. Keep final height while
                            # acquiring; measured centering may later choose any
                            # feasible height above the configured minimum body-bottom height.
                            self.position=(c.pose.p[0],c.pose.p[1],self.target_z)
                            self.heading=yaw(c.pose.q)
                        self.search_started_ros=i.ros
                        self.transition('SEARCH','takeoff_stable_wait_unique_window',i.wall)
            else:self.stable_since=None
        elif self.phase=='POSTPASS':
            return self.full_chain.tick(self,i,dt)
        elif self.phase=='HOVER':
            if self.hover_target:
                if self.full_height_bounds(c.pose.p)[0]<(self.full_chain.p['minimum_aim_bottom_m'] if self.full_chain else self.o['minimum_bottom_height_m']):
                    if self.hover_target.phase not in ('OFF_WAIT','DONE','ABORT'):
                        self.hover_target.invalid('flight_height_floor',i.wall)
                    return self.hover_target.tick(self,i,dt)
                return self.hover_target.tick(self,i,dt)
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.land('flight_height_floor',i)
            stable=(abs(c.pose.p[2]-self.target_z)<=c.c['align_tolerance_m'] and c.pose_speed<=c.c['stable_speed_mps'])
            if stable:
                if self.stable_since is None:self.stable_since=i.wall
                if i.wall-self.stable_since>=self.o['hover_time_s']:
                    self.takeoff_hover_completed=True
                    return self.land('takeoff_hover_test_complete',i)
            else:self.stable_since=None
            if i.wall-self.entered>self.o['hover_time_s']+10.:
                return self.land('hover_stability_timeout',i)
        elif self.session_window and self.phase=='SEARCH':
            return self.acquire_session_window(i,dt)
        elif self.session_window and self.phase=='ADJUST':
            return self.align_session_window(i,dt)
        elif self.direct_pass and self.phase=='SEARCH':
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.land('flight_height_floor',i)
            if i.wall-self.entered>=self.o['search_timeout_s']:
                return self.land('straight_decision_timeout:'+self.passage_decision.reason,i)
            status=getattr(c,'metric_input',None) or {}
            ready=self.passage_decision.update(status,c.c,c.pose,i.ros,i.wall,c.metric_received,self.search_started_ros)
            self.reason=self.passage_decision.reason
            if not ready and (self.adjust_candidate is not None or self.passage_decision.final_rejection):
                result=self.consider_adjustment(i,status)
                if result is not None:return result
            if ready:
                plan=self.passage_decision.plan
                # Freeze one straight finite path. Never enter ALIGN or retarget
                # on a new window observation once a passage is committed.
                if plan['expected_duration_s']>=self.o['max_mission_time_s']-(i.wall-self.began):
                    return self.land('straight_remaining_time_insufficient',i)
                if self.adjust_candidate is not None:
                    self.adjust_candidate=None;self.adjust_reason='current_route_feasible_without_correction'
                self.direct_plan=plan
                self.position=tuple(plan['start']);self.heading=plan['yaw_rad']
                self.transition('PASS','straight_route_committed_no_alignment',i.wall)
        elif self.direct_pass and self.phase=='ADJUST':
            from prepass_adjustment import validate_motion,advance
            self.adjust_path_m+=norm(sub(c.pose.p,self.adjust_last_position));self.adjust_last_position=c.pose.p
            if self.adjust_path_m>min(self.adjustment_policy['max_path_m'],c.c['max_align_path_m']):
                return self.handover('adjustment_total_path_limit',i)
            try:
                validate_motion(self.adjust_plan,c.c,c.pose,c.metric_input or {},i.ros,i.wall,c.metric_received,
                    self.search_started_ros,self.ground_z,self.o['minimum_bottom_height_m'],self.origin,self.o['landing_radius_m'],self.adjustment_policy)
                self.position,arrived=advance(self.adjust_plan,c.c,c.pose,self.position,dt,i.wall-self.entered,self.adjustment_policy)
            except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
                self.adjust_reason=str(exc)
                return self.handover('prepass_adjustment:'+str(exc),i)
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.handover('adjustment_height_floor',i)
            if arrived and c.pose_speed<=c.c['stable_speed_mps']:
                if self.adjust_arrival_since is None:self.adjust_arrival_since=i.wall
                if i.wall-self.adjust_arrival_since>=self.adjustment_policy['settle_s']:
                    from straight_pass import PassageDecision
                    self.adjust_completed=True;self.adjust_reason='completed_recheck_new_geometry'
                    self.adjust_candidate=None;self.passage_decision=PassageDecision()
                    self.position=tuple(self.adjust_plan['target']);self.search_started_ros=i.ros
                    self.transition('SEARCH','adjustment_complete_fresh_passage_decision_required',i.wall)
            else:self.adjust_arrival_since=None
        elif self.direct_pass and self.phase=='PASS':
            from straight_pass import advance_route
            try:
                elapsed=i.wall-self.entered
                scoped_c=self.session_window.scoped_control(c.c) if self.return_journey and self.return_journey.leg==2 else c.c
                self.position,finished=advance_route(self.direct_plan,c.pose,self.position,scoped_c,dt,elapsed)
                if self.direct_plan.get('smooth_trajectory'):
                    from smooth_pass import sample
                    speed=sample(self.direct_plan['smooth_trajectory'],elapsed)[1]
                    self.pass_velocity=tuple(speed*x for x in self.direct_plan['direction'])
            except ValueError as exc:return self.fail('straight_pass:'+str(exc),i)
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.fail('straight_pass_height_floor',i)
            if self.terminal_capture:
                try:stopped=self.terminal_capture.update(self,i,dt)
                except ValueError as exc:return self.fail('straight_pass:'+str(exc),i)
                if stopped:
                    self.direct_pass_completed=True
                    try:out=self.full_chain.begin(self,i)
                    except ValueError as exc:return self.handover(str(exc),i)
                    # Second-pass completion may request a pilot handover.
                    return out if out is not None else self.output()
                return self.output()
            if finished and c.pose_speed<=c.c['stable_speed_mps']:
                if self.direct_arrival_since is None:self.direct_arrival_since=i.wall
                if i.wall-self.direct_arrival_since>=c.c['stop_stable_time_s']:
                    self.direct_pass_completed=True
                    if self.full_chain:
                        try:out=self.full_chain.begin(self,i)
                        except ValueError as exc:return self.handover(str(exc),i)
                        return out if out is not None else self.output()
                    return self.handover('straight_pass_complete_full_body_clear',i)
            else:self.direct_arrival_since=None
        elif self.phase=='SEARCH':
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.land('flight_height_floor',i)
            if i.wall-self.entered>=self.o['search_timeout_s']:
                return self.land('recognition_timeout',i)
            geometry_ready=True
            if self.airborne_geometry:
                geometry_ready=c.live_bound or c.acquire(i.ros,i.wall,self.search_started_ros)
                if not geometry_ready:self.reason=c.live_reason
            reason=c.live_reason if self.airborne_geometry and not geometry_ready else ('legacy_rgb_target_not_fresh' if not c.target_fresh(i.ros,i.wall) else ('legacy_rgb_target_not_confirmed' if not c.confirmed() else 'request_hold_pending'))
            self.record_search_evidence(i,geometry_ready,reason)
            if geometry_ready and c.target_fresh(i.ros,i.wall) and c.confirmed():
                ok,reason=c.request_hold(i.ros,i.wall)
                if ok:self.transition('ALIGN','stable_window_auto_authorized',i.wall)
                else:
                    self.reason='waiting_geometry:'+reason
                    self.record_search_evidence(i,geometry_ready,'hold_refused:'+reason)
        elif self.phase in ('ALIGN','PASS'):
            if self.airborne_geometry and self.phase=='ALIGN' and not c.live_geometry_current(i.ros,i.wall):
                return self.land('live_geometry_lost:'+c.live_reason,i)
            if self.full_height_bounds(c.pose.p)[0]<self.o['minimum_bottom_height_m']:
                return self.land('flight_height_floor',i)
            if self.phase=='ALIGN' and not c.started:
                ok,reason=c.request_start(i.ros,i.wall)
                if not ok and i.wall-self.entered>c.c['max_hold_wait_s']:
                    return self.land('alignment_start_timeout:'+reason,i)
            c.tick(i.ros,i.wall)
            if c.state in ('ABORT','FAULT','RELEASED'):
                return self.land('pass_controller:'+c.reason,i)
            if c.state=='PREPASS':
                if self.align_only:
                    self.alignment_completed=True
                    return self.handover('alignment_only_complete_no_pass',i)
                ok,reason=c.request_pass(i.ros,i.wall)
                if not ok:return self.land('commit_refused:'+reason,i)
                self.transition('PASS','finite_path_committed',i.wall)
            if c.state=='HOLD_COMPLETE':
                return self.land('pass_complete',i,completed=True)
            output=c.output(i.ros,i.wall)
            if output is None:return self.fail('pass_output_unavailable',i)
            self.position,self.heading=output
        # Check the planned target as well as feedback, BEFORE publication.
        if self.phase in ('HOVER','SEARCH','ADJUST','ALIGN','PASS','POSTPASS') and self.full_height_bounds(self.position)[0]<self.o['minimum_bottom_height_m']:
            return self.land('planned_height_below_floor',i)
        return self.output()

