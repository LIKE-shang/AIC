"""Opt-in two-run observed-spot learning. No ROS, motion publisher or light IO.

Records are evidence, never a replacement for current-session authorization.
Only relative planar position is reused; the current safe height/yaw stay fixed.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import uuid
from window_pass.geometry import norm, sub, yaw, wrap

SCHEMA = 'original_car_observed_spot_calibration_v1'


def finite_vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size and
            all(type(v) in (int, float) and math.isfinite(v) for v in value))


def fingerprint(config):
    chain = copy.deepcopy(config.get('full_chain', {}))
    for key in ('selected_target_ids', 'second_window_return', 'station_offset_forward_left_m', 'station_evidence',
                'station_relation_confirmed', 'station_area_confirmed'):
        chain.pop(key, None)
    h = config['hover_target']
    camera = {k: h.get(k) for k in ('camera_frame_id', 'image_width', 'image_height',
                                  'offset_x_px', 'offset_y_px', 'inner_free_radius_m')}
    control=copy.deepcopy(config.get('control',{}))
    for key in ('evidence_id','geofence_min_m','geofence_max_m','enable_motion','enable_traverse','pass_stage_approved'):
        control.pop(key,None)
    data = dict(chain=chain, camera=camera, control=control,
                mission=config.get('mission'), session_window=config.get('session_window'))
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


class CalibrationSession:
    def __init__(self, config, identity):
        h = config.get('hover_target') or {}
        self.mode = h.get('visual_calibration_mode', 'off')
        if self.mode == 'off':
            raise ValueError('explicit visual calibration mode required')
        p = config.get('visual_calibration')
        if not isinstance(p, dict) or set(p) - {'scene_key', 'seed'}:
            raise ValueError('visual calibration scene and optional seed required')
        self.scene = p.get('scene_key')
        if not isinstance(self.scene, str) or not self.scene.strip():
            raise ValueError('explicit unchanged physical scene key required')
        self.profile = fingerprint(config)
        self.identity = copy.deepcopy(identity)
        self.pending = {}
        self.nominal_stations = {}
        self.seed = {}
        seed = p.get('seed')
        if self.mode == 'learn' and seed is not None:
            raise ValueError('learning must not preload a previous point')
        if self.mode == 'reuse':
            if (not isinstance(seed, dict) or seed.get('schema') != SCHEMA or
                    seed.get('scene_key') != self.scene or seed.get('profile_sha256') != self.profile):
                raise ValueError('calibration scene/profile mismatch')
            self.seed = copy.deepcopy(seed.get('targets'))
            ids = config.get('full_chain', {}).get('selected_target_ids', [1])
            if not isinstance(self.seed, dict) or any(str(x) not in self.seed for x in ids):
                raise ValueError('calibration missing selected target')
            for key, record in self.seed.items():
                if (key not in ('1', '2', '3') or not isinstance(record, dict) or
                        not finite_vector(record.get('position_forward_left_up_m'), 3) or
                        not finite_vector(record.get('nominal_delta_forward_left_m'), 2) or
                        norm(record['nominal_delta_forward_left_m']) > .15 + 1e-9 or
                        type(record.get('yaw_relative_rad')) not in (int, float) or
                        not math.isfinite(record['yaw_relative_rad']) or
                        record.get('off_acknowledged') is not True or
                        type(record.get('stable_exposure_s')) not in (int,float) or
                        not math.isfinite(record['stable_exposure_s']) or
                        record['stable_exposure_s'] < h['exposure_s'] or
                        type(record.get('distinct_pose_samples')) is not int or record['distinct_pose_samples'] < 2 or
                        not finite_vector(record.get('actual_position_local_m'),3) or
                        not isinstance(record.get('source_identity'), dict) or
                        not all(isinstance(record['source_identity'].get(k),str) and record['source_identity'][k]
                                for k in ('boot_id','run_id'))):
                    raise ValueError('invalid calibration evidence:' + str(key))

    @staticmethod
    def target_id(m):
        chain = getattr(m, 'full_chain', None)
        return str(chain.selected[chain.index]) if hasattr(chain, 'selected') else '1'

    def station(self, m, nominal, target_id):
        self.nominal_stations[str(target_id)]=tuple(nominal)
        if self.mode != 'reuse':
            return nominal
        record = self.seed[str(target_id)]
        a, b, _ = record['position_forward_left_up_m']
        co, si = math.cos(m.takeoff_heading), math.sin(m.takeoff_heading)
        point = (m.origin[0] + co*a - si*b, m.origin[1] + si*a + co*b, nominal[2])
        # Reused evidence cannot extend the original aiming disk, change height,
        # or silently rotate the craft to a different learned orientation.
        if norm(sub(point, nominal)[:2]) > .15 + 1e-9:
            raise ValueError('calibrated_point_outside_original_aim_region')
        if abs(wrap(m.takeoff_heading + record['yaw_relative_rad'] - m.heading)) > m.core.c['yaw_limit_rad']:
            raise ValueError('calibrated_heading_incompatible')
        return point

    def correction_allowed(self,m,point):
        nominal=self.nominal_stations.get(self.target_id(m),getattr(m,'aim_anchor',None) or m.origin)
        return norm(sub(point,nominal)[:2])<=m.hover_target.p['max_correction_m']+1e-9

    def accept(self, m, candidate):
        if not (m.hover_target.completed and m.hover_target.off_confirmed):
            raise ValueError('calibration requires completed exposure and OFF_ACK')
        row = copy.deepcopy(candidate)
        row.update(source_identity=copy.deepcopy(self.identity), off_acknowledged=True)
        self.pending[self.target_id(m)] = row

    def document(self):
        # Failed/missing targets never overwrite a previous successful record.
        records = copy.deepcopy(self.seed)
        records.update(copy.deepcopy(self.pending))
        return dict(schema=SCHEMA, scene_key=self.scene, profile_sha256=self.profile,
                    targets=records, physical_hit_verified=False)

    def save(self, directory):
        if not self.pending:
            return None
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        target = directory / ('visual_aim_' + uuid.uuid4().hex + '.json')
        temporary = target.with_suffix('.tmp')
        try:
            with temporary.open('x', encoding='utf-8') as stream:
                json.dump(self.document(), stream, indent=2, allow_nan=False)
                stream.write('\n');stream.flush();os.fsync(stream.fileno())
            os.replace(str(temporary), str(target))
        finally:
            if temporary.exists():
                temporary.unlink()
        return str(target)


def reset_stability(h):
    h.stable_exposure_since = None
    h.stable_exposure_s = 0.
    h.calibration_samples = []


def tick_observed(h, m, i, dt, bridge):
    """Laser-assisted fine aiming; never integrates absent/predicted spot data."""
    p, c = h.p, m.core
    if h.request_on and i.wall-h.on_request_wall >= p['max_light_s']-.2:
        h.invalid('light_exposure_limit',i.wall);return m.output()
    if h.phase=='ON_WAIT' and i.wall-h.on_request_wall>p['ack_timeout_s']:
        h.invalid('laser_on_not_acknowledged',i.wall);return m.output()
    if h.phase=='EXPOSE' and (bridge.get('state')!='ON_ACKNOWLEDGED' or bridge.get('request_id')!=h.request_id):
        h.invalid('laser_ack_lost',i.wall);return m.output()
    h.stable_motion.update(c.pose.stamp, c.pose.p)
    try:
        d, new = h.vision(i.hover_visual, i.ros, i.wall, h.hover_ros)
    except ValueError as exc:
        reset_stability(h)
        h.last_correction_reason = 'visual_unavailable_hold_position'
        if h.request_on:
            h.invalid(str(exc), i.wall)
        else:
            h.reason = str(exc)
        return m.output()
    # Equal/replayed images cannot accumulate stable exposure or issue corrections.
    if not new:
        return m.output()
    stamp = d['capture_stamp']
    if h.calibration_last_frame is not None and stamp-h.calibration_last_frame > p['observation_timeout_s']:
        reset_stability(h)
    h.calibration_last_frame = stamp
    error = d.get('observed_error_px')
    spot = (d.get('observed_unique') is True and
            isinstance(d.get('observed_candidates'), list) and len(d['observed_candidates']) == 1 and
            finite_vector(error, 2) and finite_vector(d.get('observed_spot_px'), 2) and
            type(d.get('observed_normalized_radius')) in (int, float) and
            math.isfinite(d['observed_normalized_radius']) and d['observed_normalized_radius'] >= 0)
    stopped = h.stable_motion.stable(c.c['stable_speed_mps'])
    anchor=getattr(m,'aim_anchor',None) or m.origin
    h.gates=dict(position_stable=bool(stopped),fixed_position_in_region=norm(sub(c.pose.p,anchor)[:2])<=p['max_correction_m'] and abs(c.pose.p[2]-m.target_z)<=.1)
    if h.phase == 'CENTER':
        # Do not learn a stationary reflection already visible with our light OFF.
        h.calibration_baseline_clear = not d.get('observed_candidates')
        if (h.calibration_baseline_clear and stopped and
                i.wall-h.started >= p['hover_light_delay_s']):
            h.request_id += 1;h.request_on = True;h.on_request_wall = i.wall
            h.phase = 'ON_WAIT';h.reason = 'observed_spot_probe_wait_on_ack'
        return m.output()
    if h.phase == 'ON_WAIT':
        if bridge.get('state') == 'ON_ACKNOWLEDGED' and bridge.get('request_id') == h.request_id:
            h.on_ack_wall = i.wall;h.on_ack_ros = i.ros;h.phase = 'EXPOSE'
            reset_stability(h)
        elif i.wall-h.on_request_wall > p['ack_timeout_s']:
            h.invalid('laser_on_not_acknowledged', i.wall)
    elif h.phase == 'EXPOSE':
        if bridge.get('state') != 'ON_ACKNOWLEDGED' or bridge.get('request_id') != h.request_id:
            h.invalid('laser_ack_lost', i.wall);return m.output()
        post_on = spot and stamp > h.on_ack_ros
        centred = post_on and d.get('observed_inside_inner') is True and d['observed_normalized_radius'] <= p['centre_radius_ratio']
        h.gates.update(actual_unique_spot=bool(post_on), observed_spot_centered=bool(centred))
        if post_on:
            h.aim_source = 'observed_spot_minus_target';h.aim_error_px = list(error)
            # Reuse keeps its reference untouched while inside the central region.
            # No predicted-reference or cached-spot fallback is allowed here.
            if not centred:
                h.correct_reference(m, dict(d, servo_error_px=list(error)), min(dt, p['observation_timeout_s']), i.wall)
            else:
                h.last_correction_reason = 'observed_centre_hold_no_correction'
        else:
            h.last_correction_reason = 'spot_missing_or_ambiguous_hold_position'
        session=getattr(m,'visual_calibration',None)
        stable = (centred and stopped and h.gates['fixed_position_in_region'] and
                  (not session or session.correction_allowed(m,c.pose.p)) and
                  norm(sub(m.position, c.pose.p)[:2]) <= c.c['max_setpoint_lead_m'])
        if stable:
            h.observed_hit_frames += 1
            if h.stable_exposure_since is None:
                h.stable_exposure_since = stamp
            h.stable_exposure_s = stamp-h.stable_exposure_since
            # One measurement per new pose and image; never save command position.
            if not h.calibration_samples or c.pose.stamp > h.calibration_samples[-1][0]:
                h.calibration_samples.append((c.pose.stamp, tuple(c.pose.p), yaw(c.pose.q), list(error),d['observed_normalized_radius']))
        else:
            reset_stability(h)
        h.metrics = dict(stable_observed_exposure_s=h.stable_exposure_s, exposure_elapsed_s=i.wall-h.on_ack_wall,
                         spot_normalized_radius=d.get('observed_normalized_radius'))
        pose_span=(h.calibration_samples[-1][0]-h.calibration_samples[0][0]) if len(h.calibration_samples)>=2 else 0.
        if h.stable_exposure_s >= p['exposure_s'] and pose_span >= p['exposure_s']:
            rows = h.calibration_samples
            actual = [sum(row[1][k] for row in rows)/len(rows) for k in range(3)]
            co, si = math.cos(m.takeoff_heading), math.sin(m.takeoff_heading)
            dx, dy, dz = sub(actual, m.origin)
            session=getattr(m,'visual_calibration',None)
            anchor=(session.nominal_stations.get(session.target_id(m)) if session else None) or getattr(m, 'aim_anchor', None) or m.origin
            ax, ay = sub(actual, anchor)[:2]
            heading = math.atan2(sum(math.sin(row[2]) for row in rows), sum(math.cos(row[2]) for row in rows))
            h.calibration_candidate = dict(position_forward_left_up_m=[co*dx+si*dy, -si*dx+co*dy, dz],
                nominal_delta_forward_left_m=[co*ax+si*ay, -si*ax+co*ay],
                yaw_relative_rad=wrap(heading-m.takeoff_heading), actual_position_local_m=actual,
                stable_exposure_s=h.stable_exposure_s, distinct_pose_samples=len(rows),
                actual_xyz_span_m=[max(row[1][k] for row in rows)-min(row[1][k] for row in rows) for k in range(3)],
                mean_spot_error_px=[sum(row[3][k] for row in rows)/len(rows) for k in range(2)],
                maximum_spot_normalized_radius=max(row[4] for row in rows),
                origin_local_m=list(m.origin),takeoff_heading_rad=m.takeoff_heading,
                localization_epoch=getattr(m,'localization_epoch',None),
                image_stamp=stamp, pose_stamp=rows[-1][0], physical_hit_verified=False)
            h.exposure_completed = True;h.completed = True
            h.completion_basis = 'stable_observed_spot_3s_camera_evidence_only'
            h.off('observed_spot_calibration_complete', i.wall)
    if h.request_on and i.wall-h.on_request_wall >= p['max_light_s']-.2:
        h.invalid('light_exposure_limit', i.wall)
    return m.output()
