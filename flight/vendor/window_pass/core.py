"""Deterministic single-window controller. No ROS/network/hardware imports.
A position setpoint is an intention, NOT proof that a vehicle stopped.
"""
from collections import deque
from dataclasses import dataclass
from .config import load_config, blockers
from .geometry import (finite, vector, quaternion, norm, add, sub, scale, dot,
                       yaw, tilt, wrap, qdistance, slerp, rotate, normal, lateral,
                       intersect_ray, opening_polygon, envelope_half_extents)


@dataclass
class Pose:
    stamp: float
    p: tuple
    q: tuple
    frame: str
    child: str


@dataclass
class Observation:
    stamp: float
    width: int
    height: int
    frame: str
    valid: bool
    count: int
    center: tuple
    corners: tuple
    method: str
    geometry: str
    rays: tuple = ()  # centre, then four corner optical rays; z=1, not metres
    camera_hash: str = ''
    profile_hash: str = ''
    algorithm_hash: str = ''


ACTIVE = ('WAIT_TARGET','CONFIRM','ALIGN','STABLE','PREPASS','TRAVERSE')
TERMINAL = ('ABORT','FAULT','RELEASED','HOLD_COMPLETE')


class Controller:
    def __init__(self, supplied=None):
        self.c = load_config(supplied or {})
        self.blockers = blockers(self.c, motion=self.c['enable_motion'])
        if self.c['enable_motion'] and self.blockers:
            raise ValueError('motion configuration refused: '+','.join(self.blockers))
        self.state, self.reason = 'OBSERVE', 'no_human_authorization'
        self.transitions = deque(maxlen=200)
        self.samples, self.poses = deque(maxlen=256), deque(maxlen=256)
        self.streams, self.latest_stamps = {}, {}
        self.pose, self.obs, self.previous_obs = None, None, None
        self.pose_speed = 0.
        self.align_path_m, self.traverse_path_m = 0.,0.
        self.connected, self.armed, self.mode, self.airborne = False, False, '', False
        self.graph_ok, self.graph_wall, self.graph_reason = False, -1e9, 'graph_not_checked'
        self.authorized, self.started, self.used = False, False, False
        self.command, self.command_yaw, self.start_p, self.pass_start, self.endpoint = None, 0., None, None, None
        self.begin_wall = self.hold_wall = self.pass_wall = self.last_tick = None
        self.stable_since = self.arrival_since = None
        self.metric, self.proposal = {}, None
        self.last_output_wall, self.output_count = None, 0
        self.confirmed_offboard = False
        self.fault_wall = None

    def transition(self, state, reason, wall):
        if (self.state, self.reason) != (state, reason):
            self.transitions.append(dict(wall=wall, old=self.state, new=state,
                                         reason=reason, measurements=dict(self.metric)))
        self.state, self.reason = state, reason

    def receive(self, name, stamp, ros, wall, timeout):
        ok = (finite(stamp) and finite(ros) and stamp > 0 and ros > 0 and
              -self.c['future_tolerance_s'] <= ros-stamp <= timeout and
              stamp > self.latest_stamps.get(name, 0))
        if ok:
            self.latest_stamps[name] = stamp
        self.streams[name] = (stamp, wall, ok)
        return ok

    def fresh(self, name, ros, wall, timeout):
        s, w, ok = self.streams.get(name, (0., -1e9, False))
        return (ok and 0 <= wall-w <= timeout and
                -self.c['future_tolerance_s'] <= ros-s <= timeout)

    def image(self, stamp, width, height, ros, wall):
        ok = self.receive('image', stamp, ros, wall, self.c['image_timeout_s'])
        if (width, height) != (self.c['expected_width'], self.c['expected_height']):
            self.streams['image'] = (stamp, wall, False)
            ok = False
        if not ok and (self.authorized or self.started):
            self.stop('invalid_image_latched', ros, wall, fault=True)
        return ok

    def flight_state(self, stamp, connected, armed, mode, ros, wall):
        good = self.receive('state', stamp, ros, wall, self.c['state_timeout_s'])
        self.connected, self.armed, self.mode = connected, armed, mode
        if self.authorized and mode == 'OFFBOARD': self.confirmed_offboard = True
        if self.c['enable_motion'] and (self.authorized or self.started):
            allowed = ('OFFBOARD',) if self.started or self.confirmed_offboard else ('POSCTL','OFFBOARD')
            if not good or not connected or not armed or mode not in allowed:
                self.stop('fcu_state_or_mode_change_latched',ros,wall,fault=True,release=True)

    def extended_state(self, stamp, airborne, ros, wall):
        good = self.receive('extended', stamp, ros, wall, self.c['state_timeout_s'])
        self.airborne = airborne
        if self.c['enable_motion'] and self.authorized and (not good or not airborne):
            self.stop('airborne_state_change_latched',ros,wall,fault=True,release=True)

    def graph(self, ok, reason, wall):
        self.graph_ok, self.graph_reason, self.graph_wall = ok, reason, wall
        if not ok and self.authorized:
            self.authorized = False
            self.transition('RELEASED','publisher_graph_untrusted:'+reason,wall)

    def pose_update(self, p, ros, wall):
        good = self.receive('pose', p.stamp, ros, wall, self.c['pose_timeout_s'])
        good = good and vector(p.p) and quaternion(p.q)
        good = good and (not self.c['local_frame'] or p.frame == self.c['local_frame'])
        good = good and (not self.c['body_frame'] or p.child == self.c['body_frame'])
        reason = 'invalid_or_stale_pose'
        if good and self.pose is not None:
            dt = p.stamp-self.pose.stamp
            jump = norm(sub(p.p, self.pose.p))
            good = dt > 0 and jump <= self.c['max_pose_step_m'] and qdistance(p.q, self.pose.q) <= self.c['max_attitude_step_rad']
            if good:
                self.pose_speed = jump/dt
                good = self.pose_speed <= self.c['max_pose_speed_mps']
            reason = 'pose_jump_or_speed'
        if good:
            if self.pose is not None and self.state in ACTIVE:
                if self.state == 'TRAVERSE': self.traverse_path_m += norm(sub(p.p,self.pose.p))
                else: self.align_path_m += norm(sub(p.p,self.pose.p))
            self.pose = p
            self.poses.append(p)
            if (self.authorized or self.started) and not self.blockers:
                if (not self.inside(p.p) or tilt(p.q) > self.c['tilt_limit_rad'] or
                        abs(wrap(yaw(p.q)-self.c['window_yaw_rad'])) > self.c['yaw_limit_rad']):
                    self.stop('pose_envelope_or_heading_latched',ros,wall,fault=True)
        else:
            self.streams['pose'] = (p.stamp, wall, False)
            if self.authorized or self.started:
                self.stop(reason, ros, wall, fault=True)
        return good

    def observe(self, o, ros, wall):
        good = self.receive('observation', o.stamp, ros, wall, self.c['capture_timeout_s'])
        reason = 'invalid_capture_timestamp'
        try:
            if not good: raise ValueError(reason)
            if not o.valid or o.count != 1: raise ValueError('absent_or_multiple_targets')
            if (o.width, o.height) != (self.c['expected_width'], self.c['expected_height']): raise ValueError('image_dimensions')
            if o.method not in ('checker_texture','checker_contour','nested_frame'): raise ValueError('unknown_method')
            expected = 'contour_inner_opening' if o.method == 'nested_frame' else 'approximate_inner_opening'
            if o.geometry != expected: raise ValueError('not_inner_opening')
            center, area = opening_polygon(o.corners, o.width, o.height)
            if not vector(o.center, 2) or norm(sub(center, o.center)) > 1.: raise ValueError('inconsistent_center')
            prev = self.previous_obs
            if prev is not None:
                _, old_area = opening_polygon(prev.corners, prev.width, prev.height)
                if prev.method != o.method or prev.geometry != o.geometry: raise ValueError('method_changed_reconfirm')
                if norm(sub(o.center, prev.center)) > self.c['target_jump_px']: raise ValueError('target_jump')
                if max(area/old_area, old_area/area) > self.c['target_area_ratio']: raise ValueError('target_scale_jump')
                if o.stamp-prev.stamp > self.c['observation_timeout_s']: self.samples.clear()
            self.obs, self.previous_obs = o, o
            self.samples.append((o.stamp, o.center))
            while len(self.samples) > 1 and o.stamp-self.samples[1][0] > self.c['confirm_time_s']:
                self.samples.popleft()
            return True
        except (ValueError, TypeError, OverflowError) as exc:
            reason = str(exc)
        self.obs, self.previous_obs = None, None
        self.samples.clear()
        self.streams['observation'] = (o.stamp, wall, False)
        # After commit the frame can legitimately leave view. Never retarget.
        if self.state in ('ALIGN','STABLE','PREPASS'):
            self.stop(reason, ros, wall)
        return False

    def target_fresh(self, ros, wall):
        return (self.obs is not None and self.fresh('observation', ros, wall, self.c['capture_timeout_s']) and
                wall-self.streams['observation'][1] <= self.c['observation_timeout_s'])

    def confirmed(self):
        if len(self.samples) < self.c['confirm_frames']: return False
        if self.samples[-1][0]-self.samples[0][0] < self.c['confirm_time_s']-1e-6: return False
        pts = [p for _, p in self.samples]
        return max(max(p[i] for p in pts)-min(p[i] for p in pts) for i in (0,1)) <= self.c['stability_px']

    def capture_pose(self, stamp):
        history = list(self.poses)
        slop = self.c['pose_sync_slop_s']
        for a, b in zip(history, history[1:]):
            if a.stamp <= stamp <= b.stamp and max(stamp-a.stamp, b.stamp-stamp) <= slop:
                t = (stamp-a.stamp)/(b.stamp-a.stamp)
                return add(a.p, scale(sub(b.p,a.p),t)), slerp(a.q,b.q,t)
        if history:
            p = min(history, key=lambda p: abs(p.stamp-stamp))
            if abs(p.stamp-stamp) <= slop: return p.p, p.q
        raise ValueError('no_capture_synchronised_pose')

    def inside(self, p):
        c = self.c
        if not vector(c['geofence_min_m']) or not vector(c['geofence_max_m']): return False
        # Sphere about odometry origin encloses any body attitude and protrusions.
        radius = norm(c['vehicle_size_m'])/2+norm(c['envelope_center_body_m'])+c['clearance_margin_m']+c['position_uncertainty_m']
        return all(lo+radius <= v <= hi-radius for lo,hi,v in zip(c['geofence_min_m'],c['geofence_max_m'],p))

    def health(self, ros, wall, require_mode=True):
        if not self.fresh('pose',ros,wall,self.c['pose_timeout_s']): return 'pose_timeout_or_invalid'
        if not self.fresh('image',ros,wall,self.c['image_timeout_s']): return 'image_timeout_or_invalid'
        if self.c['enable_motion']:
            if not self.graph_ok or wall-self.graph_wall > self.c['graph_timeout_s']: return 'publisher_graph_untrusted:'+self.graph_reason
            if not self.fresh('state',ros,wall,self.c['state_timeout_s']) or not self.connected: return 'fcu_disconnected_or_stale'
            if not self.fresh('extended',ros,wall,self.c['state_timeout_s']) or not self.airborne: return 'not_confirmed_airborne'
            if not self.armed: return 'not_manually_armed'
            if require_mode and self.mode != 'OFFBOARD': return 'mode_exit_or_manual_takeover'
            if not require_mode and self.mode not in ('POSCTL','OFFBOARD'): return 'hold_requires_POSCTL_or_OFFBOARD'
        if self.pose and not self.blockers:
            if not self.inside(self.pose.p): return 'geofence_or_altitude'
            if tilt(self.pose.q) > self.c['tilt_limit_rad']: return 'attitude_tilt'
            if abs(wrap(yaw(self.pose.q)-self.c['window_yaw_rad'])) > self.c['yaw_limit_rad']: return 'heading_not_manually_aligned'
            if self.command is not None and self.state not in ('ABORT','FAULT','RELEASED'):
                if norm(sub(self.command,self.pose.p)) > self.c['max_tracking_error_m']: return 'setpoint_tracking_error'
        return ''

    def measurement(self):
        c, o = self.c, self.obs
        if not o: raise ValueError('no_target')
        # OBSERVE may display metric geometry before motion budgets are measured.
        # Keep every blocker in status; motion still requires ALL of them.
        non_geometric = {'missing:geofence_min_m', 'missing:geofence_max_m',
                         'invalid:geofence', 'missing:position_uncertainty_m',
                         'missing:stopping_margin_m', 'insufficient:worst_case_clearance'}
        required = [b for b in self.blockers
                    if c['enable_motion'] or b not in non_geometric]
        if required: raise ValueError(';'.join(required))
        if o.frame != c['optical_frame']: raise ValueError('optical_frame_mismatch')
        for actual, key in ((o.camera_hash,'camera_info_sha256'),(o.profile_hash,'vision_profile_sha256'),(o.algorithm_hash,'algorithm_sha256')):
            if actual != c[key]: raise ValueError(key+'_mismatch')
        if len(o.rays) != 5 or not all(vector(r) and abs(r[2]-1.) < 1e-6 for r in o.rays): raise ValueError('no_calibrated_optical_rays')
        p, q = self.capture_pose(o.stamp)
        hits = [intersect_ray(p,q,r,c) for r in o.rays]
        world, corners = hits[0], hits[1:]
        if norm(sub(world,c['window_center_local_m'])) > c['center_agreement_m']: raise ValueError('survey_visual_disagreement')
        lr = [dot(sub(p,c['window_center_local_m']),lateral(c['window_yaw_rad'])) for p in corners]
        zz = [p[2] for p in corners]
        sizes = (max(lr)-min(lr),max(zz)-min(zz))
        if any(abs(v-ref)/ref > c['opening_size_error_fraction'] for v,ref in zip(sizes,c['opening_size_m'])): raise ValueError('survey_opening_size_disagreement')
        n = normal(c['window_yaw_rad'])
        body_center = add(self.pose.p,rotate(self.pose.q,c['envelope_center_body_m']))
        error = sub(world,body_center)
        distance = dot(sub(c['window_center_local_m'],body_center),n)
        transverse = sub(error,scale(n,dot(error,n)))
        survey_error = sub(c['window_center_local_m'],body_center)
        survey_error = sub(survey_error,scale(n,dot(survey_error,n)))
        return dict(survey_error_norm_m=norm(survey_error), distance_m=distance, lateral_error_m=dot(transverse,lateral(c['window_yaw_rad'])),
                    vertical_error_m=transverse[2], error_norm_m=norm(transverse),
                    visual_center_local_m=world, correction_local_m=transverse,
                    pose_speed_mps=self.pose_speed, capture_stamp=o.stamp,
                    method=o.method, geometry=o.geometry)

    def request_hold(self, ros, wall):
        if not self.c['enable_motion']: return False, 'shadow_mode_has_no_motion_publisher'
        if self.used or self.authorized or self.state != 'OBSERVE': return False, 'one_session_restart_required'
        error = self.health(ros,wall,False)
        if error: return False,error
        if self.pose_speed > self.c['stable_speed_mps']: return False,'not_stably_hovering'
        if not self.target_fresh(ros,wall) or not self.confirmed(): return False,'fresh_confirmed_target_required'
        try: self.metric = self.measurement()
        except ValueError as exc: return False,str(exc)
        if not self.c['approach_min_m'] <= self.metric['distance_m'] <= self.c['approach_max_m']: return False,'approach_distance'
        self.authorized, self.used, self.hold_wall = True, True, wall
        self.command, self.command_yaw = self.pose.p, yaw(self.pose.q)
        self.transition('PRECHECK','human_authorized_stationary_stream_only',wall)
        return True,self.reason

    def request_start(self, ros, wall):
        if self.started or self.state in TERMINAL: return False,'one_session_restart_required'
        if self.c['enable_motion']:
            if not self.authorized: return False,'authorize_hold_first'
            if wall-self.hold_wall < self.c['hold_prestream_s']: return False,'hold_stream_duration_too_short'
            if self.output_count < int(self.c['hold_prestream_s']*self.c['control_hz']) or self.last_output_wall is None or wall-self.last_output_wall > self.c['max_loop_gap_s']:
                return False,'actual_hold_stream_not_yet_verified'
        elif self.blockers:
            return False,'shadow_metric_test_needs_calibration:'+','.join(self.blockers)
        error = self.health(ros,wall)
        if error: return False,error
        if self.pose_speed > self.c['stable_speed_mps']: return False,'start_requires_stable_hold'
        if not self.target_fresh(ros,wall): return False,'fresh_target_required'
        self.started, self.used, self.begin_wall = True, True, wall
        self.start_p = self.pose.p
        if self.command is None: self.command, self.command_yaw = self.pose.p,yaw(self.pose.q)
        self.samples.clear()  # New confirmation AFTER human start, never consume prestart counter.
        self.transition('WAIT_TARGET','human_started_alignment_only',wall)
        return True,self.reason

    def request_pass(self, ros, wall):
        if not self.c['enable_traverse'] or not self.c['pass_stage_approved']: return False,'traversal_not_enabled_and_accepted'
        if self.state != 'PREPASS': return False,'not_in_prepass'
        error = self.health(ros,wall)
        if error: return False,error
        if not self.target_fresh(ros,wall) or not self.confirmed(): return False,'fresh_stable_target_required'
        try: self.metric = self.measurement()
        except ValueError as exc: return False,str(exc)
        if max(self.metric['error_norm_m'],self.metric['survey_error_norm_m']) > self.c['align_tolerance_m'] or self.pose_speed > self.c['stable_speed_mps']: return False,'alignment_no_longer_stable'
        d = self.metric['distance_m']
        if not self.c['approach_min_m'] <= d <= self.c['approach_max_m']: return False,'approach_distance'
        n = normal(self.c['window_yaw_rad'])
        # stopping_margin_m is the distance kept before the commanded endpoint.
        # It used to be required for motion but consumed nowhere, so changing it
        # had no effect on any decision. It belongs on the approach axis, where
        # it shortens travel rather than inflating the lateral clearance.
        travel = (d+envelope_half_extents(self.c)[0]+self.c['window_thickness_m']/2+
                  self.c['post_plane_clearance_m']+self.c['position_uncertainty_m']+
                  self.c['arrival_tolerance_m']+self.c['stopping_margin_m'])
        if travel > self.c['max_traverse_distance_m']: return False,'trajectory_too_long'
        nominal_time = travel/self.c['traverse_speed_mps']+self.c['stop_stable_time_s']
        if nominal_time >= min(self.c['max_traverse_time_s'],self.c['max_total_time_s']-(wall-self.begin_wall)):
            return False,'insufficient_remaining_time_for_finite_path'
        end = add(self.pose.p,scale(n,travel))
        if not self.inside(end): return False,'endpoint_outside_geofence'
        self.pass_start, self.endpoint, self.pass_wall = self.pose.p,end,wall
        self.command = self.pose.p
        self.transition('TRAVERSE','human_committed_finite_surveyed_path_no_retargeting',wall)
        return True,self.reason

    def stop(self, reason, ros, wall, fault=False, release=False):
        if release:
            self.authorized = False
            self.transition('RELEASED',reason,wall)
            return
        if self.state in ('ABORT','FAULT','RELEASED'): return
        self.fault_wall = wall
        # With valid pose: brake/hold once at current position, never backtrack/land.
        # Without valid pose: freeze last finite setpoint, not a computed hover.
        if self.fresh('pose',ros,wall,self.c['pose_timeout_s']) and self.pose and self.inside(self.pose.p):
            if self.state == 'TRAVERSE' and self.command is not None:
                n = normal(self.c['window_yaw_rad'])
                # No intentional reverse at a frame: freeze the furthest of
                # current measured progress and the already bounded lead point.
                if dot(sub(self.pose.p,self.command),n) >= 0: self.command = self.pose.p
            else: self.command = self.pose.p
        self.transition('RELEASED' if release else ('FAULT' if fault else 'ABORT'),reason,wall)
        if release: self.authorized = False

    def release(self, ros, wall):
        if self.c['enable_motion'] and (not self.fresh('state',ros,wall,self.c['state_timeout_s']) or self.mode == 'OFFBOARD'):
            return False,'first_take_over_and_confirm_fresh_non_OFFBOARD_state'
        self.authorized = False
        self.transition('RELEASED','human_released_after_takeover',wall)
        return True,self.reason

    def output_sent(self, wall):
        self.last_output_wall, self.output_count = wall,self.output_count+1

    def watchdog(self, ros, wall):
        if not (self.authorized or self.started): return
        if self.last_tick is not None and wall-self.last_tick > self.c['max_loop_gap_s']:
            self.stop('control_loop_deadline',ros,wall,fault=True)
        error = self.health(ros,wall,self.started or self.confirmed_offboard)
        if error:
            release = error.startswith(('mode_exit','fcu_','publisher_graph_','not_manually','not_confirmed_airborne'))
            if self.state == 'PRECHECK' and self.mode == 'POSCTL' and error == 'setpoint_tracking_error':
                release = True  # Do not later pull a manually drifting vehicle back.
            if self.state in ('ABORT','FAULT') and release:
                self.authorized = False
                self.transition('RELEASED',error,wall)
            else: self.stop(error,ros,wall,fault=not release,release=release)
        if self.mode == 'OFFBOARD' and self.authorized: self.confirmed_offboard = True
        if self.state in ('ALIGN','STABLE','PREPASS') and not self.target_fresh(ros,wall): self.stop('target_watchdog_expired',ros,wall)
        if self.state == 'PRECHECK' and wall-self.hold_wall > self.c['max_hold_wait_s']: self.stop('hold_wait_expired',ros,wall)
        if (self.state in ('ABORT','FAULT') and self.fault_wall is not None and
                not self.fresh('pose',ros,wall,self.c['pose_timeout_s']) and
                wall-self.fault_wall >= self.c['degraded_hold_timeout_s']):
            self.stop('pose_unavailable_handoff_to_verified_FCU_failsafe_not_hover',ros,wall,release=True)

    def tick(self, ros, wall):
        dt = 1./self.c['control_hz'] if self.last_tick is None else wall-self.last_tick
        self.watchdog(ros,wall)
        self.last_tick = wall
        self.proposal = None
        if self.target_fresh(ros,wall) and self.fresh('pose',ros,wall,self.c['pose_timeout_s']):
            try: self.metric = self.measurement()
            except ValueError as exc:
                self.metric = dict(measurement_rejected=str(exc), capture_stamp=self.obs.stamp)
                if self.state in ('ALIGN','STABLE','PREPASS'): self.stop(str(exc),ros,wall)
        else: self.metric = dict(measurement_rejected='no_fresh_target')
        if self.state == 'OBSERVE' and 'correction_local_m' in self.metric:
            e = self.metric['correction_local_m']
            self.proposal = scale(e,min(self.c['align_gain_per_s'],self.c['align_speed_mps']/max(norm(e),1e-9)))
        if self.state not in ACTIVE: return self.snapshot(ros,wall)
        if wall-self.begin_wall > self.c['max_total_time_s']:
            self.stop('total_time_limit',ros,wall)
        elif self.state != 'TRAVERSE' and wall-self.begin_wall > self.c['max_align_time_s']:
            self.stop('alignment_time_limit',ros,wall)
        elif self.state != 'TRAVERSE' and (norm(sub(self.pose.p,self.start_p)) > self.c['max_align_distance_m'] or self.align_path_m > self.c['max_align_path_m']):
            self.stop('alignment_distance_limit',ros,wall)
        if self.state not in ACTIVE: return self.snapshot(ros,wall)
        if self.state in ('WAIT_TARGET','CONFIRM'):
            if not self.target_fresh(ros,wall): self.transition('WAIT_TARGET','waiting_for_new_target',wall)
            elif not self.confirmed(): self.transition('CONFIRM','association_and_time_stability',wall)
            elif 'correction_local_m' not in self.metric: self.transition('CONFIRM','metric_geometry_not_ready',wall)
            else: self.transition('ALIGN','confirmed_same_geometry_target',wall)
        if self.state in ('ALIGN','STABLE','PREPASS'):
            if not self.c['approach_min_m'] <= self.metric['distance_m'] <= self.c['approach_max_m']:
                self.stop('approach_distance',ros,wall)
            elif self.metric['error_norm_m'] > self.c['align_tolerance_m']:
                self.stable_since = None
                self.transition('ALIGN','correcting_transverse_only',wall)
                e = self.metric['correction_local_m']
                v = scale(e,min(self.c['align_gain_per_s'],self.c['align_speed_mps']/max(norm(e),1e-9)))
                self.proposal = v
                candidate = add(self.command,scale(v,max(0.,dt)))
                # Hold approach-plane normal coordinate; no forward alignment creep.
                n = normal(self.c['window_yaw_rad'])
                candidate = sub(candidate,scale(n,dot(sub(candidate,self.start_p),n)))
                self.advance(candidate,ros,wall)
            elif self.pose_speed <= self.c['stable_speed_mps'] and self.confirmed() and self.metric['survey_error_norm_m'] <= self.c['align_tolerance_m']:
                if self.stable_since is None:
                    self.stable_since = wall
                    self.command = self.pose.p
                if wall-self.stable_since >= self.c['stable_time_s']:
                    self.transition('PREPASS','stable_holding_waiting_separate_pass_authorization',wall)
                else: self.transition('STABLE','timed_stability_hold',wall)
            else:
                self.stable_since = None
                self.transition('ALIGN','waiting_for_low_speed_and_visual_stability',wall)
        elif self.state == 'TRAVERSE':
            n = normal(self.c['window_yaw_rad'])
            delta = sub(self.pose.p,self.pass_start)
            progress, length = dot(delta,n),norm(sub(self.endpoint,self.pass_start))
            cross = norm(sub(delta,scale(n,progress)))
            self.metric = dict(progress_m=progress,length_m=length,cross_track_m=cross,pose_speed_mps=self.pose_speed)
            if (wall-self.pass_wall > self.c['max_traverse_time_s'] or
                    progress < -self.c['max_tracking_error_m'] or progress > length+self.c['arrival_tolerance_m'] or
                    max(norm(delta),self.traverse_path_m) > self.c['max_traverse_distance_m'] or cross > self.c['align_tolerance_m']):
                self.stop('finite_path_distance_time_or_cross_track_limit',ros,wall)
            else:
                commanded = dot(sub(self.command,self.pass_start),n)
                step = min(length,commanded+self.c['traverse_speed_mps']*max(0.,dt),progress+self.c['max_setpoint_lead_m'])
                # If feedback falls behind, HOLD previous setpoint rather than retreat.
                step = max(commanded,step)
                self.advance(add(self.pass_start,scale(n,step)),ros,wall)
                if self.state != 'TRAVERSE': return self.snapshot(ros,wall)
                self.proposal = scale(n,self.c['traverse_speed_mps']) if step < length else (0.,0.,0.)
                if norm(sub(self.pose.p,self.endpoint)) <= self.c['arrival_tolerance_m'] and self.pose_speed <= self.c['stable_speed_mps']:
                    if self.arrival_since is None: self.arrival_since = wall
                    if wall-self.arrival_since >= self.c['stop_stable_time_s']:
                        self.command = self.endpoint
                        self.transition('HOLD_COMPLETE','past_plane_and_low_speed_hold_observed',wall)
                else: self.arrival_since = None
        return self.snapshot(ros,wall)

    def advance(self, candidate, ros, wall):
        if norm(sub(self.command,self.pose.p)) > self.c['max_tracking_error_m']:
            self.stop('setpoint_tracking_error',ros,wall)
            return
        if not self.inside(candidate):
            self.stop('setpoint_geofence',ros,wall)
            return
        if self.state != 'TRAVERSE' and norm(sub(candidate,self.start_p)) > self.c['max_align_distance_m']:
            self.stop('command_alignment_distance',ros,wall)
            return
        if norm(sub(candidate,self.pose.p)) <= self.c['max_setpoint_lead_m']:
            self.command = candidate

    def output(self, ros, wall):
        if not self.c['enable_motion'] or not self.authorized or self.command is None: return None
        if not self.graph_ok or wall-self.graph_wall > self.c['graph_timeout_s']: return None
        if not self.fresh('state',ros,wall,self.c['state_timeout_s']) or not self.connected or not self.armed: return None
        if self.mode not in ('POSCTL','OFFBOARD') or (self.confirmed_offboard and self.mode != 'OFFBOARD'): return None
        return self.command,self.command_yaw

    def snapshot(self, ros, wall):
        return dict(state=self.state,reason=self.reason,enable_motion=self.c['enable_motion'],
                    authorized_hold=self.authorized,started=self.started,blockers=self.blockers,
                    analysis_phase=('WAIT_TARGET' if not self.target_fresh(ros,wall) else ('TARGET_CONFIRMED' if self.confirmed() else 'TARGET_CONFIRMING')),
                    align_path_m=self.align_path_m,traverse_path_m=self.traverse_path_m,
                    measurement=self.metric,confirmed=self.confirmed() and self.target_fresh(ros,wall),suggested_velocity_local=self.proposal,
                    commanded_position_local=self.command,actual_output_permitted=self.output(ros,wall) is not None,
                    hold_pose_fresh=self.fresh('pose',ros,wall,self.c['pose_timeout_s']),hold_guaranteed=False,
                    accepted_flight=False,accepted_real_window_pass=False)
