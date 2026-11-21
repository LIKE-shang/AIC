#!/usr/bin/env python3
"""Read-only preflight by default. --execute is explicit one-shot motion intent.
Never start alongside trial, air, the old motion node, or a mission publisher.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
import queue
from pathlib import Path
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mission import Mission, Inputs, settings, TERMINAL
from window_pass.config import load_config, blockers
from window_pass.core import Controller, Pose, Observation
from window_pass.graph import check_graph, OUTPUT
from live_geometry import flight_blockers
from takeoff import TakeoffCore, takeoff_blockers, check_takeoff_graph, EVIDENCE as TAKEOFF_EVIDENCE

EVIDENCE=('body_origin_verified','ground_height_verified','takeoff_column_verified',
          'takeoff_landing_site_verified','postpass_landing_site_verified',
          'fcu_failsafe_verified','auto_takeoff_land_verified','simulation_verified')
PARAMETERS=('RC_MAP_KILL_SW','RC_KILLSWITCH_TH','RC6_REV','RC_MAP_OFFB_SW',
            'COM_RC_OVERRIDE','COM_OF_LOSS_T','COM_OBL_RC_ACT','COM_OBL_ACT',
            'NAV_RCL_ACT','EKF2_HGT_MODE','EKF2_EV_POS_X','EKF2_EV_POS_Y','EKF2_EV_POS_Z',
            'MPC_LAND_SPEED','COM_LOW_BAT_ACT','COM_RC_LOSS_T','COM_RCL_EXCEPT',
            'COM_RC_STICK_OV','BAT1_N_CELLS','MPC_Z_VEL_MAX_UP','MPC_ACC_UP_MAX')


def verify_experiment(config,required=None):
    experiment=config.get('experiment','traverse')
    if experiment not in ('traverse','takeoff_land','align_only','straight_pass'):raise ValueError('unknown experiment')
    if required is not None and experiment!=required:raise ValueError('entry/config experiment mismatch')
    return experiment


COMMISSIONING_DEFERRALS=frozenset(('body_origin_verified','fcu_failsafe_verified','simulation_verified'))

def commissioning_deferrals(config,requested):
    validate_flight_policy(config)
    if not requested:return frozenset()
    if config.get('hover_target',{}).get('enabled'):
        raise ValueError('takeoff-only commissioning deferrals cannot authorize hover visual correction')
    if verify_experiment(config)!='takeoff_land':raise ValueError('commissioning exception is takeoff-only')
    declaration=config.get('commissioning_authorization',{})
    if declaration.get('scope')!='supervised_takeoff_hover_land_only' or declaration.get('operator_accepts_unverified_tests') is not True:
        raise ValueError('explicit commissioning declaration required')
    if set(declaration.get('deferred_checks',[]))!=COMMISSIONING_DEFERRALS:
        raise ValueError('commissioning cannot skip live safety checks')
    options=settings(config.get('mission',{}))
    if options['target_bottom_height_m']>.90 or options['climb_speed_mps']>.10 or options['hover_time_s']>5 or options['landing_radius_m']>.20:
        raise ValueError('commissioning limits exceed the scoped low-speed test')
    return COMMISSIONING_DEFERRALS


def validate_flight_policy(config):
    # Hardware policy applies to every entry, including non-commissioning flights.
    # This validation grants no motion permission and defers no acceptance gate.
    options=settings(config.get('mission',{}))
    if options['takeoff_position_goal_enabled']:
        expected=config.get('expected_fcu_parameters',{})
        for key,limit in [('MPC_Z_VEL_MAX_UP',.4),('MPC_ACC_UP_MAX',.4)]:
            pair=expected.get(key)
            if not isinstance(pair,list) or len(pair)!=2 or pair[0]!=0 or type(pair[1]) not in (int,float) or not math.isfinite(pair[1]) or not 0<pair[1]<=limit+1e-6:
                raise ValueError('position goal takeoff requires bounded FCU parameter: '+key)
    if options['pilot_landing_enabled'] and config.get('expected_fcu_parameters',{}).get('COM_OBL_RC_ACT')!=[0,0.]:
        raise ValueError('pilot landing requires declared RC-online offboard-loss Position policy')


def geometry_identity(path, master, ros):
    raw=path.read_bytes()
    meta=json.loads(path.with_suffix('.session.json').read_text())
    ident=meta['identity']
    if hashlib.sha256(raw).hexdigest()!=meta['sha256']:raise ValueError('geometry_hash_changed')
    if ident['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('geometry_from_previous_boot')
    if ident['run_id']!=ros.get_param('/window_pass_session_launch/run_id',''):raise ValueError('geometry_from_previous_session')
    pubs=dict(master.getSystemState()[0])
    for key,topic in [('mavros','/mavros/state'),('lio','/window_pass_lio/Odometry'),('pose','/window_pass_competition/status')]:
        names=pubs.get(topic,[])
        if len(names)!=1 or ident[key]!=[names[0],master.lookupNode(names[0])]:raise ValueError('geometry_source_changed:'+key)
    return json.loads(raw)


def wait_readonly_timesync(read, wait, stopped, clock=time.monotonic, initial=None):
    """Keep the same ground observer for a bounded startup sync convergence.

    Every poll rechecks the full report. Only the observed offset instability
    may wait; no blocker is removed and no clock or control request is made.
    """
    first = read() if initial is None else initial
    if first.get('ready') or first.get('blockers') != ['pose_time_guard:timesync_offset_unstable']:
        return first
    started = clock()
    deadline = started + 15.
    checks = 1
    report = first
    print('PRECHECK_WAIT timesync_offset_unstable; max_extra_wait_s=15; same_session=true', flush=True)
    while True:
        if stopped():
            raise RuntimeError('readonly_preflight_interrupted')
        remaining = deadline - clock()
        if report.get('ready') or report.get('blockers') != first['blockers'] or remaining <= 0:
            result = dict(report)
            result['startup_timesync_wait'] = dict(
                elapsed_s=clock()-started, max_extra_wait_s=15., reports=checks,
                initial_guard=first.get('pose_time_guard'),
                outcome='ready' if report.get('ready') else 'timeout' if remaining <= 0 else 'other_blocker')
            return result
        wait(min(.1, remaining))
        if stopped():
            raise RuntimeError('readonly_preflight_interrupted')
        report = read()
        checks += 1


class NonBlockingLog:
    """Bounded best-effort text logs; no ROS control/status traffic here.

    A slow console or ROS logging handler must never hold the control lock.
    Items contain already formatted immutable text arguments. Shutdown never
    waits for a blocked logger; the worker is a daemon.
    """
    def __init__(self, logger, capacity=64):
        self.logger=logger
        self.queue=queue.Queue(maxsize=capacity)
        self.closed=threading.Event()
        self.dropped=0
        self.errors=0
        self.worker=threading.Thread(target=self._work,daemon=True)

    def __enter__(self):
        self.worker.start()
        return self

    def info(self, *args):
        if self.closed.is_set():return
        try:self.queue.put_nowait(args)
        except queue.Full:self.dropped+=1

    def _work(self):
        while not self.closed.is_set():
            try:args=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:self.logger(*args)
            except Exception:self.errors+=1

    def __exit__(self, *exc):
        self.closed.set()


class ThreadLocalMaster:
    """XML-RPC transports are owned by one thread, never shared concurrently."""
    def __init__(self,factory):
        self.factory=factory
        self.local=threading.local()

    def client(self):
        if not hasattr(self.local,'master'):
            self.local.master=self.factory()
        return self.local.master

    def getSystemState(self):return self.client().getSystemState()
    def lookupNode(self,name):return self.client().lookupNode(name)


class Runtime:
    def __init__(self, config, execute, commissioning=False, alignment_commissioning=False, straight_commissioning=False, hover_commissioning=False, full_chain_commissioning=False):
        import rospy,rosgraph
        from mavros_msgs.msg import State,ExtendedState,RCIn,EstimatorStatus,PositionTarget,StatusText,TimesyncStatus
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image,BatteryState
        from std_msgs.msg import String
        if config.get('experiment','traverse')!='takeoff_land':
            from window_pass_test.msg import WindowObservation
        self.ros=rospy
        self.types=dict(PositionTarget=PositionTarget,String=String)
        self.master=ThreadLocalMaster(lambda:rosgraph.Master(rospy.get_name()))
        self.cfg=config
        self.full_chain_enabled=bool(config.get('full_chain',{}).get('enabled'))
        self.chain_scope=('window_pass_two_targets' if 'selected_target_ids' in config.get('full_chain',{}) else 'window_pass_single_target')
        from full_chain_policy import validate as validate_full_chain
        chain_deferrals=validate_full_chain(config,full_chain_commissioning)
        from hover_target import settings as hover_settings
        self.hover_policy=hover_settings(config.get('hover_target'))
        self.visual_calibration=None
        if self.hover_policy and self.hover_policy.get('visual_calibration_mode','off')!='off':
            from visual_calibration import CalibrationSession
            self.visual_calibration=CalibrationSession(config,config.get('site_identity',{}))
        if self.hover_policy and self.ros.get_name()!='/autonomous_window_test':
            raise ValueError('hover supervisor name must match execution bridge')
        if self.hover_policy and ((config.get('experiment')!='takeoff_land' and not self.full_chain_enabled) or not config.get('mission',{}).get('pilot_landing_enabled')):
            raise ValueError('hover targeting requires takeoff-only and pilot handover')
        self.hover_visual_row=None;self.hover_laser_row=None
        from session_window import settings as window_settings
        self.session_policy=window_settings(config.get('session_window'))
        self.localization=None
        self.current_localization_identity=None
        self.execute=execute
        if sum(map(bool,(commissioning,alignment_commissioning,straight_commissioning,hover_commissioning,full_chain_commissioning)))>1:raise ValueError('commissioning scopes are mutually exclusive')
        from alignment_policy import validate as validate_alignment_policy
        from straight_pass_policy import validate as validate_straight_policy
        self.hover_commissioning=bool(hover_commissioning)
        self.straight_commissioning=bool(straight_commissioning or full_chain_commissioning)
        self.alignment_commissioning=bool(alignment_commissioning)
        self.deferred_checks=commissioning_deferrals(config,commissioning) | validate_alignment_policy(config,alignment_commissioning) | validate_straight_policy(config,straight_commissioning)
        from hover_trial_policy import validate as validate_hover_policy
        self.deferred_checks |= validate_hover_policy(config,hover_commissioning) | chain_deferrals
        self.experiment=verify_experiment(config)
        self.takeoff_only=self.experiment=='takeoff_land'
        self.align_only=self.experiment=='align_only'
        self.direct_pass=self.experiment=='straight_pass'
        if self.direct_pass and (config.get('acquisition')!='airborne_rgbd' or not config.get('mission',{}).get('pilot_landing_enabled')):raise ValueError('straight pass requires airborne RGB-D and pilot handover')
        if self.align_only:
            if config.get('acquisition')!='airborne_rgbd':raise ValueError('alignment-only requires airborne RGB-D')
            if not config.get('mission',{}).get('pilot_landing_enabled'):raise ValueError('alignment-only requires pilot handover')
        self.passive_rgbd=config.get('acquisition')=='takeoff_rgbd_observe'
        if self.passive_rgbd and not self.takeoff_only:raise ValueError('passive RGB-D observation is takeoff-only')
        self.airborne_geometry=config.get('acquisition')=='airborne_rgbd'
        self.observation_rows=deque(maxlen=100)
        self.observation_started_ros=None
        if self.takeoff_only and self.airborne_geometry:raise ValueError('takeoff test cannot use airborne acquisition')
        self.rgbd_row=None
        self.depth_row=None
        self.source_identity=None
        self.lock=threading.RLock()
        self.stop=threading.Event()
        self.motion=None
        self.mission=None
        self.params={}
        self.params_wall=0.
        self.params_error='not_checked'
        self.rows={}
        self.fcu_text=deque(maxlen=30)
        self.history=[]
        self.graph_wall=0
        self.graph_reason='not_checked'
        self.graph_ok=False
        self.control=None
        self.errors=[]
        try:
            if self.takeoff_only:
                base=dict(config.get('control',{}))
                try:self.source_identity=self.identity()
                except Exception as exc:self.errors.append(str(exc))
                if config.get('site_identity')!=self.source_identity:
                    self.errors.append('takeoff_site_not_bound_to_current_session')
            elif self.airborne_geometry:
                base=dict(config['control']) if 'control' in config else json.loads(Path(config['control_base']).expanduser().read_text())
                # Installation profile only. Never read the old geometry pointer.
                base['window_center_local_m']=[]
                base['window_yaw_rad']=0.
                base['geometry_verified']=False
                base['opening_geometry_verified']=False
                try:self.source_identity=self.identity()
                except Exception as exc:self.errors.append(str(exc))
                if (self.align_only or self.direct_pass) and config.get('site_identity')!=self.source_identity:self.errors.append('motion_site_not_bound_to_current_session' if self.direct_pass else 'alignment_site_not_bound_to_current_session')
            else:
                chosen=config.get('geometry_file') or (Path.home()/'window_pass_current_geometry.path').read_text().strip()
                config['geometry_file']=chosen
                path=Path(chosen).expanduser().resolve(strict=True)
                base=geometry_identity(path,self.master,rospy)
            base.update(config.get('control_overrides',{}))
            if self.align_only and (base.get('enable_traverse') or base.get('pass_stage_approved')):raise ValueError('alignment-only forbids traversal permissions')
            self.control=load_config(base)
            if self.airborne_geometry and self.control['window_center_local_m']:
                raise ValueError('airborne mode refuses saved window coordinates')
        except Exception as exc:
            self.errors.append(str(exc))
            self.control=load_config({})
        # Observer core has no flight authority until every preflight is passed.
        self.core=Controller(dict(self.control,enable_motion=False,enable_traverse=False,pass_stage_approved=False))
        from pose_time_guard import PoseTimeGuard
        guard_policy=config.get('pose_time_guard',{})
        if not isinstance(guard_policy,dict) or set(guard_policy)-{'enabled'} or type(guard_policy.get('enabled',False)) is not bool:
            raise ValueError('invalid pose_time_guard policy')
        self.pose_time_guard=PoseTimeGuard(self.control,guard_policy.get('enabled',False))
        if self.session_policy['enabled'] or self.hover_policy:
            from session_localization import Continuity
            from session_window import localization_identity
            if self.full_chain_enabled:
                from full_chain_localization import RouteContinuity
                self.localization=RouteContinuity(self.control)
            elif self.hover_policy:
                from hover_localization import HoverContinuity
                self.localization=HoverContinuity(self.control)
            else:self.localization=Continuity(self.control)
            self.current_localization_identity=localization_identity(self.source_identity)
        self.diag=rospy.Publisher('/autonomous_window_test/status',String,queue_size=2)
        self.subs=[rospy.Subscriber('/mavros/state',State,self.state,queue_size=10),
                   rospy.Subscriber('/mavros/extended_state',ExtendedState,self.ext,queue_size=10),
                   rospy.Subscriber('/mavros/local_position/odom',Odometry,self.pose,queue_size=100)]
        if self.session_policy['enabled'] or self.hover_policy:
            self.subs.extend([
                rospy.Subscriber('/window_pass_lio/Odometry',Odometry,self.lio_pose,queue_size=10),
                rospy.Subscriber('/window_pass_competition/status',String,self.bridge_status,queue_size=2)])
        if self.hover_policy:
            self.subs.extend([
                rospy.Subscriber('/circle_pixel_preview/status',String,self.hover_visual,queue_size=1),
                rospy.Subscriber('/hover_target_laser/status',String,self.hover_laser,queue_size=1)])
        self.subs.append(rospy.Subscriber('/mavros/statustext/recv',StatusText,self.statustext,queue_size=30))
        self.subs.append(rospy.Subscriber('/mavros/timesync_status',TimesyncStatus,self.timesync,queue_size=40))
        if not self.takeoff_only and not self.direct_pass:
            self.subs.extend([
                rospy.Subscriber('/camera/rgb/image_raw',Image,self.image,queue_size=1,buff_size=16777216),
                rospy.Subscriber('/window_pass_test/observation',WindowObservation,self.obs,queue_size=10)])
        if self.direct_pass:self.subs.append(rospy.Subscriber('/camera/rgb/image_raw',Image,self.image,queue_size=1,buff_size=16777216))
        if self.airborne_geometry or self.passive_rgbd:
            self.subs.append(rospy.Subscriber('/window_live/status',String,self.rgbd,queue_size=2))
            self.subs.append(rospy.Subscriber('/camera/depth/image_raw',Image,self.depth,queue_size=1,buff_size=16777216))
        for topic,typ in [('/mavros/rc/in',RCIn),('/mavros/estimator_status',EstimatorStatus),('/mavros/battery',BatteryState)]:
            if topic=='/mavros/battery' and not config.get('mission',{}).get('companion_battery_monitor_enabled',True):continue
            self.subs.append(rospy.Subscriber(topic,typ,lambda m,t=topic:self.store(t,m),queue_size=10))
        threading.Thread(target=self.parameter_worker,daemon=True).start()
        threading.Thread(target=self.graph_worker,daemon=True).start()
        threading.Thread(target=self.watchdog,daemon=True).start()

    def identity(self,include_perception=True):
        pubs=dict(self.master.getSystemState()[0])
        result={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'run_id':self.ros.get_param('/window_pass_session_launch/run_id','')}
        if not result['run_id']:raise ValueError('localization_session_id_missing')
        topics=['/mavros/state','/window_pass_lio/Odometry','/window_pass_competition/status']
        if include_perception and (self.airborne_geometry or getattr(self,'passive_rgbd',False)):topics.append('/window_live/status')
        for topic in topics:
            names=pubs.get(topic,[])
            if len(names)!=1:raise ValueError('localization_source_not_unique:'+topic)
            result[topic]=[names[0],self.master.lookupNode(names[0])]
        return result

    def visual_required(self):
        mission=getattr(self,'mission',None)
        return not (mission and getattr(mission,'session_window',None) and mission.session_window.snapshot is not None)

    def target_visual_optional(self):
        policy=getattr(self,'hover_policy',None) or {}
        return bool(policy.get('fixed_position_timed') or policy.get('timed_visual_aim'))

    def lio_pose(self,msg):
        with self.lock:
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            expected=(self.source_identity or {}).get('/window_pass_lio/Odometry',[None])[0]
            self.localization.observe(Pose(msg.header.stamp.to_sec(),(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),
                msg.header.frame_id,msg.child_frame_id),msg.header.seq,*self.now(),
                (getattr(msg,'_connection_header',{}) or {}).get('callerid'),expected)

    def bridge_status(self,msg):
        with self.lock:
            try:status=json.loads(msg.data)
            except (ValueError,TypeError):self.localization.invalidate('pose_bridge_status_invalid');return
            if not isinstance(status,dict):self.localization.invalidate('pose_bridge_status_invalid');return
            expected=(self.source_identity or {}).get('/window_pass_competition/status',[None])[0]
            self.localization.observe_bridge(status,time.monotonic(),
                (getattr(msg,'_connection_header',{}) or {}).get('callerid'),expected)

    def rgbd(self,msg):
        with self.lock:
            if (getattr(msg,'_connection_header',{}) or {}).get('callerid')!='/window_live_vision':return
            try:status=json.loads(msg.data)
            except (ValueError,TypeError):status={}
            if not isinstance(status,dict):status={}
            self.rgbd_row=(time.monotonic(),status)
            if getattr(self,'passive_rgbd',False) and self.mission and self.mission.phase=='HOVER':
                from live_geometry import validate_metric
                if self.observation_started_ros is None:self.observation_started_ros=self.ros.Time.now().to_sec()
                stamp=status.get('source_stamp')
                if not self.observation_rows or self.observation_rows[-1].get('source_stamp')!=stamp:
                    row={k:status.get(k) for k in ('source_stamp','target_count','reason','calibration_valid','metric_geometry_ready','metric_geometry_reason','metric_geometry')}
                    try:
                        row['validated_geometry']=validate_metric(status,self.control,self.ros.Time.now().to_sec(),time.monotonic(),self.rgbd_row[0],self.observation_started_ros)
                        row['metric_validation']='ok'
                    except (ValueError,TypeError,KeyError) as exc:row['metric_validation']=str(exc)
                    self.observation_rows.append(row)
            if self.mission and self.airborne_geometry:self.core.ingest_metric(status,time.monotonic())

    def hover_visual(self,msg):
        if (getattr(msg,'_connection_header',{}) or {}).get('callerid')!='/circle_pixel_preview':return
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):data={}
        with self.lock:self.hover_visual_row=(time.monotonic(),data if isinstance(data,dict) else {})

    def hover_laser(self,msg):
        if (getattr(msg,'_connection_header',{}) or {}).get('callerid')!='/hover_target_laser':return
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):data={}
        with self.lock:self.hover_laser_row=(time.monotonic(),data if isinstance(data,dict) else {})

    def statustext(self,msg):
        if (getattr(msg,'_connection_header',{}) or {}).get('callerid')!='/mavros':return
        row=dict(received_wall=time.monotonic(),source_stamp=msg.header.stamp.to_sec(),severity=msg.severity,text=msg.text)
        with self.lock:self.fcu_text.append(row)
        self.ros.logwarn('FCU_TEXT severity=%s: %s',msg.severity,msg.text)

    def depth(self,msg):
        with self.lock:
            if msg.encoding not in ('16UC1','32FC1') or msg.width<=0 or msg.height<=0:return
            self.depth_row=(time.monotonic(),msg.header.stamp.to_sec())

    def watchdog(self):
        while not self.stop.wait(.02):
            with self.lock:
                m=self.mission
                if m and m.phase not in TERMINAL and m.phase!='IDLE' and m.last_tick is not None:
                    if time.monotonic()-m.last_tick>self.control['max_loop_gap_s']:
                        m.core.authorized=False
                        m.transition('FAULT','independent_watchdog_deadline_FCU_failsafe_required',time.monotonic())

    def now(self):return self.ros.Time.now().to_sec(),time.monotonic()
    def store(self,topic,m):
        with self.lock:
            self.rows[topic]=(time.monotonic(),m)
            mission=self.mission
            if mission and mission.phase not in TERMINAL and mission.phase!='IDLE':
                if topic=='/mavros/rc/in' and (len(m.channels)<7 or not 900<=m.channels[6]<=1200):
                    mission.core.authorized=False
                    mission.transition('TAKEOVER','RC_switch_change_latched',time.monotonic())
                if topic=='/mavros/estimator_status' and not (m.pos_horiz_rel_status_flag and m.pos_vert_abs_status_flag):
                    mission.core.authorized=False
                    mission.transition('FAULT','estimator_failure_latched_FCU_failsafe_required',time.monotonic())
    def state(self,m):
        with self.lock:
            self.store('/mavros/state',m)
            self.core.flight_state(m.header.stamp.to_sec(),m.connected,m.armed,m.mode,*self.now())
            mission=self.mission
            if mission and mission.phase not in TERMINAL and mission.phase!='IDLE':
                allowed={'PRESTREAM':('MANUAL','ALTCTL','POSCTL'),'MODE':('MANUAL','ALTCTL','POSCTL','OFFBOARD'),
                         'LAND_REQUEST':('OFFBOARD','AUTO.LAND'),'LANDING':('AUTO.LAND',),
                         'LANDING_HANDOVER':('OFFBOARD','AUTO.LAND'),'AUTO_LANDING':('AUTO.LAND',),'LANDING_LIGHT_OFF':('OFFBOARD','AUTO.LAND','POSCTL'),
                         'HANDOVER_REQUEST':('OFFBOARD','POSCTL')}.get(mission.phase,('OFFBOARD',))
                if not m.connected or m.system_status not in (3,4) or m.mode not in allowed or (mission.phase in ('PRESTREAM','MODE') and m.armed):
                    mission.core.authorized=False
                    mission.transition('TAKEOVER','FCU_state_or_mode_pulse_latched',time.monotonic())
    def ext(self,m):
        with self.lock:
            self.store('/mavros/extended_state',m)
            self.core.extended_state(m.header.stamp.to_sec(),m.landed_state==2,*self.now())
            if self.mission and self.mission.phase in ('HOVER','SEARCH','ADJUST','ALIGN','PASS','POSTPASS') and m.landed_state!=2:
                self.core.authorized=False
                self.mission.transition('FAULT','airborne_state_loss_latched',time.monotonic())
    def timesync(self,m):
        with self.lock:
            ros,wall=self.now()
            self.pose_time_guard.timesync(ros,wall,m.header.stamp.to_sec(),m.observed_offset_ns,m.estimated_offset_ns,m.round_trip_time_ms,(m._connection_header or {}).get('callerid'))

    def pose(self,m):
        arrival=self.now()
        with self.lock:
            p,q=m.pose.pose.position,m.pose.pose.orientation
            pose=Pose(m.header.stamp.to_sec(),(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),m.header.frame_id,m.child_frame_id)
            prior=self.core.pose;latest=self.core.latest_stamps.get('pose',0.)
            ros,wall=self.now()
            caller=(getattr(m,'_connection_header',None) or {}).get('callerid')
            if caller==self.core.c.get('fcu_node') and pose==prior:
                # Keep the original accepted sample and its expiry. Never
                # interpret an identical MAVROS retransmission as new feedback.
                self.duplicate_pose_messages=getattr(self,'duplicate_pose_messages',0)+1
                return
            valid=self.core.pose_update(pose,ros,wall)
            guard=getattr(self,'pose_time_guard',None)
            if guard is None:
                from pose_time_guard import PoseTimeGuard
                guard=PoseTimeGuard(self.core.c);self.pose_time_guard=guard
            guard.pose(pose,ros,wall,prior,latest,valid,getattr(getattr(self,'mission',None),'phase',None),arrival=arrival)
            if getattr(self,'hover_policy',None) and getattr(self,'localization',None) is not None:
                self.localization.observe_fcu(Pose(m.header.stamp.to_sec(),(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),m.header.frame_id,m.child_frame_id))
            if valid:
                self.history.append((time.monotonic(),p.z))
                self.history=self.history[-300:]
            elif self.mission and self.mission.phase not in TERMINAL and self.mission.phase!='IDLE':
                self.core.authorized=False
                self.mission.transition('FAULT','invalid_pose_pulse_latched',time.monotonic())
    def image(self,m):
        with self.lock:
            stamp=m.header.stamp.to_sec() if m.encoding in ('rgb8','bgr8') and (not self.core.c['optical_frame'] or m.header.frame_id==self.core.c['optical_frame']) else 0
            self.core.image(stamp,m.width,m.height,*self.now())
    def obs(self,m):
        with self.lock:
            rays=tuple((r.x,r.y,r.z) for r in m.optical_rays) if m.calibration_valid else ()
            self.core.observe(Observation(m.header.stamp.to_sec(),m.image_width,m.image_height,m.header.frame_id,
                m.valid and m.calibration_valid and m.input_encoding in ('rgb8','bgr8'),m.target_count,(m.center_px.x,m.center_px.y),
                tuple((p.x,p.y) for p in m.opening_corners_px),m.detection_method,m.geometry_kind,rays,
                m.camera_info_sha256,m.vision_profile_sha256,m.algorithm_sha256),*self.now())

    def fresh(self,topic,age=1.):
        row=self.rows.get(topic)
        if row is None:return None
        wall,m=row
        stamp=m.header.stamp.to_sec()
        if (getattr(m,'_connection_header',{}) or {}).get('callerid')!='/mavros':return None
        if not 0<=time.monotonic()-wall<=age or not -.02<=self.ros.Time.now().to_sec()-stamp<=age or stamp<=0:return None
        return m

    def estimator_health(self):
        # ESTIMATOR_STATUS is normally 1 Hz. One second is its period, not a
        # valid expiry margin. Keep a finite deadline and retain fault latching.
        topic='/mavros/estimator_status';limit=1.5
        row=getattr(self,'rows',{}).get(topic)
        ros,wall=self.now()
        evidence=dict(timeout_s=limit,receipt_age_s=None,source_age_s=None,
                      horizontal_valid=None,vertical_valid=None,reason='estimator_data_missing')
        if row is not None:
            received,msg=row
            evidence.update(receipt_age_s=wall-received,source_age_s=ros-msg.header.stamp.to_sec(),
                            horizontal_valid=bool(msg.pos_horiz_rel_status_flag),
                            vertical_valid=bool(msg.pos_vert_abs_status_flag))
            source=(getattr(msg,'_connection_header',{}) or {}).get('callerid')
            if source!='/mavros':evidence['reason']='estimator_source_untrusted'
            elif self.fresh(topic,limit) is None:evidence['reason']='estimator_data_stale_or_invalid_stamp'
            elif not msg.pos_horiz_rel_status_flag:evidence['reason']='estimator_horizontal_invalid'
            elif not msg.pos_vert_abs_status_flag:evidence['reason']='estimator_vertical_invalid'
            else:evidence['reason']='ok'
        return evidence['reason']=='ok',evidence

    def inputs(self):
        ros,wall=self.now()
        s=self.fresh('/mavros/state',1.5);e=self.fresh('/mavros/extended_state',1.5)
        rc=self.fresh('/mavros/rc/in')
        estimator_ok,estimator_evidence=self.estimator_health()
        monitor=self.cfg.get('mission',{}).get('companion_battery_monitor_enabled',True)
        bat=self.fresh('/mavros/battery',2.) if monitor else None
        rc_ok=rc is not None and len(rc.channels)>=7 and 900<=rc.channels[6]<=1200
        # Do not infer kill state from CH6 alone; PX4 state is authoritative.
        # Any RC offboard-switch activation also cancels this mission.
        emergency=bool(s and s.system_status not in (3,4)) or bool(rc and len(rc.channels)>=7 and rc.channels[6]>1200)
        battery_ok=not monitor or bool(bat and math.isfinite(bat.voltage) and bat.voltage>=self.cfg['mission']['minimum_battery_voltage'])
        localization=getattr(self,'localization',None)
        if localization is not None:
            local_ok,local_reason=localization.health(ros,wall)
            estimator_evidence['localization_continuity']=dict(ok=local_ok,reason=local_reason,epoch=localization.epoch)
            if not local_ok:estimator_ok=False;estimator_evidence['reason']=local_reason
        return Inputs(wall,ros,s is not None,e is not None,bool(e and e.landed_state==1),
                      rc_ok,estimator_ok,
                      battery_ok,self.graph_ok and wall-self.graph_wall<=.6,emergency,
                      estimator_evidence['reason'] if not estimator_ok else '',estimator_evidence,
                      getattr(self,'current_localization_identity',None),localization.epoch if localization else None,
                      getattr(self,'hover_visual_row',None),getattr(self,'hover_laser_row',None),
                      landed_state=e.landed_state if e is not None else None)

    def parameter_names(self):
        from session_window import settings as window_settings
        smooth=window_settings(self.cfg.get('session_window')).get('smooth_pass',False)
        extra=('MPC_XY_VEL_MAX','MPC_ACC_HOR','MPC_ACC_HOR_MAX','MPC_JERK_AUTO','MPC_TILTMAX_AIR') if smooth else ()
        if 'MPC_YAWRAUTO_MAX' in self.cfg.get('expected_fcu_parameters',{}):extra+=('MPC_YAWRAUTO_MAX',)
        return PARAMETERS+extra

    def parameters_ok(self,wall):
        expected=self.cfg.get('expected_fcu_parameters',{})
        if not 0<=wall-self.params_wall<=5. or set(expected)!=set(self.parameter_names()):return False
        for key,wanted in expected.items():
            actual=self.params.get(key)
            if not isinstance(wanted,list) or len(wanted)!=2 or not isinstance(actual,list) or len(actual)!=2:return False
            if type(wanted[0]) is not int or type(actual[0]) is not int or wanted[0]!=actual[0]:return False
            if any(type(v) not in (float,int) or not math.isfinite(v) for v in (wanted[1],actual[1])):return False
            if not math.isclose(wanted[1],actual[1],rel_tol=1e-6,abs_tol=1e-6):return False
        return True

    def parameter_worker(self):
        from mavros_msgs.srv import ParamGet
        while not self.stop.is_set():
            try:
                getter=self.ros.ServiceProxy('/mavros/param/get',ParamGet)
                current={}
                for name in self.parameter_names():
                    response=getter(param_id=name)
                    if not response.success:raise IOError('parameter_read_unavailable:'+name)
                    current[name]=[response.value.integer,response.value.real]
                with self.lock:
                    self.params=current;self.params_wall=time.monotonic();self.params_error=''
            except Exception as exc:
                # Preserve the last successful timestamp. parameters_ok still
                # expires it after 5 seconds; a real mismatched read is not cached away.
                with self.lock:self.params_error=str(exc)
            self.stop.wait(2.)

    def graph_worker(self):
        while not self.stop.is_set():
            try:
                graph=self.master.getSystemState()
                checker=check_takeoff_graph if self.takeoff_only or getattr(self,'direct_pass',False) else check_graph
                ok,reason=checker(graph,self.ros.get_name(),self.control,self.motion is not None)
                pubs=dict(graph[0])
                for t in ('/mavros/rc/in','/mavros/estimator_status','/mavros/battery'):
                    if t=='/mavros/battery' and not self.cfg.get('mission',{}).get('companion_battery_monitor_enabled',True):continue
                    if pubs.get(t)!=['/mavros']:ok,reason=False,'unexpected_source:'+t
                for t in ('/window_pass_hold/status','/window_pass_align/status','/window_pass_test/status'):
                    if pubs.get(t):ok,reason=False,'old_control_session_running:'+t
                for t in ('/setpoint_position/tf/listen','/setpoint_attitude/tf/listen','/vision_pose/tf/listen'):
                    if self.ros.get_param('/mavros'+t,None) is not False:ok,reason=False,'TF_input_unverified:'+t
                if self.ros.get_param('/use_sim_time',False):ok,reason=False,'real_execution_refuses_sim_time'
                if (self.airborne_geometry or getattr(self,'passive_rgbd',False)) and self.visual_required():
                    if pubs.get('/window_live/status')!=['/window_live_vision']:ok,reason=False,'rgbd_node_missing_or_ambiguous'
                    if getattr(self,'direct_pass',False) and len(pubs.get('/camera/rgb/image_raw',[]))!=1:ok,reason=False,'rgb_source_missing_or_ambiguous'
                    if len(pubs.get('/camera/depth/image_raw',[]))!=1:ok,reason=False,'depth_source_missing_or_ambiguous'
                    if self.source_identity is not None and self.identity()!=self.source_identity:ok,reason=False,'localization_session_changed'
                    if self.ros.get_param('/window_live_vision/camera_to_body_xyz',[.08,0.,-.06])!=self.control['camera_translation_body_m']:ok,reason=False,'rgbd_camera_translation_mismatch'
                    if self.ros.get_param('/window_live_vision/camera_to_body_quaternion',[-.5,.5,-.5,.5])!=self.control['camera_rotation_body_from_optical_xyzw']:ok,reason=False,'rgbd_camera_rotation_mismatch'
                if getattr(self,'localization',None) is not None:
                    from session_window import localization_identity
                    current=self.identity(include_perception=False)
                    self.current_localization_identity=current
                    if current!=localization_identity(self.source_identity):
                        self.localization.invalidate('localization_session_changed')
                        ok,reason=False,'localization_session_changed'
                if getattr(self,'hover_policy',None):
                    for topic,node in [('/circle_pixel_preview/status','/circle_pixel_preview'),('/hover_target_laser/status','/hover_target_laser')]:
                        optional=topic=='/circle_pixel_preview/status' and self.target_visual_optional()
                        if (pubs.get(topic) not in (None,[],[node]) if optional else pubs.get(topic)!=[node]):ok,reason=False,'hover_source_missing_or_ambiguous:'+topic
                    if pubs.get('/circle_laser_test/decision') and self.ros.get_param('/circle_laser_bridge/enable_laser',False):
                        ok,reason=False,'ground_laser_bridge_must_be_stopped'
                if self.takeoff_only and self.source_identity is not None and self.identity()!=self.source_identity:
                    ok,reason=False,'localization_session_changed'
                if self.execute and not self.parameters_ok(time.monotonic()):
                    ok,reason=False,'FCU_parameter_manifest_missing_or_changed'
                with self.lock:
                    self.graph_ok,self.graph_reason,self.graph_wall=ok,reason,time.monotonic()
                    self.core.graph(ok,reason,self.graph_wall)
            except Exception as exc:
                with self.lock:
                    # A query transport failure cannot refresh a successful result.
                    # inputs() keeps the original 0.6 s expiry. Known identity
                    # violations remain immediately fatal.
                    if isinstance(exc,ValueError):self.graph_ok=False
                    self.graph_reason=str(exc)
            self.stop.wait(.2)

    def report(self):
        i=self.inputs()
        errors=list(self.errors)
        guard=getattr(self,'pose_time_guard',None)
        time_check=guard.ground(i.ros,i.wall) if guard is not None and guard.enabled else {'enabled':False}
        if time_check['enabled'] and not time_check['ready']:
            errors.extend('pose_time_guard:'+x for x in time_check['blockers'])
        try:options=settings(self.cfg.get('mission',{}))
        except ValueError as exc:errors.append(str(exc));options=None
        if getattr(self,'hover_policy',None):
            from hover_target import fresh_bridge
            bridge=fresh_bridge(getattr(self,'hover_laser_row',None),i.wall,getattr(self,'chain_scope','window_pass_single_target') if getattr(self,'full_chain_enabled',False) else 'single_hover_target')
            if self.cfg.get('mission',{}).get('second_window_return_enabled') and (not bridge or bridge.get('landing_illumination_authorized') is not True or bridge.get('landing_illumination_used') is not False):errors.append('landing_illumination_bridge_not_fresh_or_authorized')
            if self.hover_policy.get('visual_calibration_mode','off')!='off' and (not bridge or bridge.get('visual_calibration_authorization')!='observed_spot_calibration_20261026'):errors.append('visual_calibration_bridge_version_mismatch')
            if not bridge or bridge.get('state')!='OFF_ACKNOWLEDGED':errors.append('hover_laser_initial_off_not_acknowledged')
            if getattr(self,'chain_scope','')=='window_pass_two_targets' and bridge and (bridge.get('exposure_count')!=0 or bridge.get('maximum_exposures')!=2):errors.append('two_target_laser_episode_count_invalid')
            if getattr(self,'full_chain_enabled',False) and bridge and (bridge.get('exposure_used') is not False or bridge.get('request_id')!=0):errors.append('full_chain_laser_episode_already_used_or_unknown')
            row=getattr(self,'hover_visual_row',None)
            if self.target_visual_optional():pass
            elif not row or not 0<=i.wall-row[0]<=.5:errors.append('hover_camera_status_missing')
            elif row[1].get('reason')=='pixel_reference_camera_or_size_mismatch':errors.append('hover_camera_identity_or_size_mismatch')
            # Complete ground rings are deliberately NOT a preflight requirement.
        required_evidence=TAKEOFF_EVIDENCE if self.takeoff_only else EVIDENCE
        if getattr(self,'align_only',False):required_evidence=tuple(k for k in EVIDENCE if k not in ('auto_takeoff_land_verified','postpass_landing_site_verified'))
        if getattr(self,'direct_pass',False):required_evidence=tuple(k for k in EVIDENCE if k!='auto_takeoff_land_verified')
        deferred=getattr(self,'deferred_checks',frozenset())
        for k in required_evidence:
            if self.cfg.get('acceptance',{}).get(k) is not True and k not in deferred:errors.append('unverified:'+k)
        errors+=(takeoff_blockers(self.control) if self.takeoff_only else (flight_blockers(self.control,getattr(self,'alignment_commissioning',False),getattr(self,'straight_commissioning',False),__import__('session_window').settings(self.cfg.get('session_window'))['enabled']) if self.airborne_geometry else blockers(self.control,motion=True)))
        if self.airborne_geometry:
            if self.cfg.get('acceptance',{}).get('rgbd_geometry_verified') is not True and 'rgbd_geometry_verified' not in deferred:errors.append('unverified:rgbd_geometry_verified')
            row=self.rgbd_row
            if not row or not 0<=i.wall-row[0]<=1. or row[1].get('calibration_valid') is not True:errors.append('rgbd_stream_or_calibration_not_ready')
            if self.cfg.get('session_window',{}).get('fast_confirmation',False):
                conf=(row[1].get('configuration') or {}) if row else {}
                if not (conf.get('fast_confirmation') is True and conf.get('metric_required_frames')==3 and
                        conf.get('metric_minimum_span_s')==.35 and conf.get('max_track_gap_s')==.6 and
                        conf.get('max_image_age_s')==.5):errors.append('fast_confirmation_perception_profile_mismatch')
            d=self.depth_row
            if not d or not 0<=i.wall-d[0]<=.5 or not -.02<=i.ros-d[1]<=.5:errors.append('depth_stream_not_fresh')
        if getattr(self,'passive_rgbd',False):
            row=self.rgbd_row;d=self.depth_row
            if not row or not 0<=i.wall-row[0]<=.5 or row[1].get('calibration_valid') is not True:errors.append('passive_rgbd_stream_or_calibration_not_ready')
            if not d or not 0<=i.wall-d[0]<=.5 or not -.02<=i.ros-d[1]<=.5:errors.append('passive_depth_stream_not_fresh')
        required_switches=('enable_motion',) if self.takeoff_only or getattr(self,'align_only',False) else ('enable_motion','enable_traverse','pass_stage_approved')
        for k in required_switches:
            if self.control[k] is not True:errors.append('disabled:'+k)
        if not self.cfg.get('evidence_id'):errors.append('missing:mission_evidence_id')
        if not i.state_fresh or self.core.armed or not i.landed or self.core.mode not in ('MANUAL','ALTCTL','POSCTL') or i.emergency:errors.append('ground_state_not_ready')
        if not i.rc_ok:errors.append('rc_not_ready')
        if not i.battery_ok:errors.append('battery_not_ready')
        if not i.estimator_ok:errors.append('estimator_not_ready:'+i.estimator_reason)
        if not i.graph_ok:errors.append('graph:'+self.graph_reason)
        if not self.core.fresh('pose',i.ros,i.wall,.15):errors.append('pose_not_fresh')
        if not self.takeoff_only and not self.core.fresh('image',i.ros,i.wall,.25):errors.append('image_not_fresh')
        samples=[z for t,z in self.history if i.wall-t<=3]
        if len(samples)<50:errors.append('insufficient_ground_samples')
        elif options and max(samples)-min(samples)>options['ground_z_span_limit_m']:errors.append('ground_height_unstable')
        startup_envelope=None
        if (getattr(self,'align_only',False) or getattr(self,'direct_pass',False)) and options and self.core.pose is not None:
            from envelope_fence import envelope_inside,envelope_margins
            p=self.core.pose.p;q=self.core.pose.q
            from session_window import decorate,settings as window_settings
            envelope_control=decorate(dict(self.control),window_settings(self.cfg.get('session_window')))
            target=(p[0],p[1],p[2]-options['ground_fcu_height_m']+options['bottom_to_fcu_m']+options['target_bottom_height_m'])
            if not envelope_inside(envelope_control,p,q) or not envelope_inside(envelope_control,target,q):errors.append('takeoff_envelope_outside_fence')
            try:startup_envelope={'model':('horizontal_cylinder_plus_rigid_clearance' if '_session_cylinder_radius_m' in envelope_control else 'rotated_box_plus_existing_reserve'),'start':envelope_margins(envelope_control,p,q),'target':envelope_margins(envelope_control,target,q)}
            except (ValueError,TypeError,KeyError):errors.append('invalid_startup_envelope')
        # The declaration must be checked by a person against the installed FCU,
        # never copied automatically from the current parameter readback.
        expected=self.cfg.get('expected_fcu_parameters',{})
        if not self.parameters_ok(i.wall):errors.append('FCU_parameter_manifest_missing_changed_or_stale')
        return dict(ready=not errors,blockers=sorted(set(errors)),fcu_parameters=self.params,
                    pose_time_guard=time_check,
                    startup_envelope=startup_envelope,
                    runtime_build=('20261025_full_chain_v6_1_transfer_heading_recovery' if self.cfg.get('mission',{}).get('second_window_return_enabled') else '20261025_full_chain_v5_3_field_geometry_alignment' if getattr(self,'full_chain_enabled',False) else '20261023_single_hover_target_v7_planar_servo' if getattr(self,'hover_policy',None) else '20261021_session_window_v24_8_faster_pass'),
                    session_window=__import__('session_window').settings(self.cfg.get('session_window')),
                    terminal_capture_enabled=self.cfg.get('session_window',{}).get('terminal_capture',False),
                    fixed_position_timed=self.cfg.get('hover_target',{}).get('fixed_position_timed',False),
                    stable_target_required_for_light=self.cfg.get('hover_target',{}).get('require_stable_target_before_light',False),
                    passage_geometry_tilt_limit_rad=self.cfg.get('session_window',{}).get('pass_tilt_limit_rad') or self.control['tilt_limit_rad'],
                    pass_command_speed_mps=(1.5 if self.cfg.get('session_window',{}).get('smooth_pass') else self.control['traverse_speed_mps']),
                    pass_maximum_forward_lead_m=(.35 if self.cfg.get('session_window',{}).get('smooth_pass') else .12 if self.cfg.get('session_window',{}).get('faster_pass',False) else self.control['max_setpoint_lead_m']),
                    minimum_bottom_height_m=options['minimum_bottom_height_m'] if options else None,
                    hover_takeoff_heading_tolerance_rad=(math.radians(30) if getattr(self,'hover_policy',None) and options and options['takeoff_position_goal_enabled'] and options['vendor_takeoff_arrival_enabled'] else None),
                    simplified_takeoff_enabled=bool(options and options['simplified_takeoff_enabled']),
                    takeoff_extra_xy_yaw_tilt_overshoot_gates_enabled=not bool(options and options['simplified_takeoff_enabled']),
                    prepass_adjustment=__import__('prepass_adjustment').settings(self.cfg.get('prepass_adjustment')),
                    estimator=i.estimator_evidence,
                    takeoff_arrival_policy='vendor_height_20cm_after_arm_request_3s' if options and options['vendor_takeoff_arrival_enabled'] else 'precision_stability',
                    takeoff_stability_speed_gate_enabled=bool(options and not options['vendor_takeoff_arrival_enabled']),
                    passive_rgbd_observation_enabled=getattr(self,'passive_rgbd',False),
                    takeoff_horizontal_error_limit_m=(None if options and options['simplified_takeoff_enabled'] else (.30 if options and options['takeoff_transient_tolerance_enabled'] else (options['takeoff_horizontal_error_m'] if options and self.takeoff_only else self.control['max_tracking_error_m']))),
                    pilot_landing_enabled=self.cfg.get("mission",{}).get("pilot_landing_enabled",False),
                    takeoff_vertical_control='final_height_position' if self.cfg.get('mission',{}).get('takeoff_position_goal_enabled',False) else ('velocity' if self.takeoff_only and self.cfg.get('mission',{}).get('takeoff_vertical_velocity_enabled',False) else 'position'),
                    takeoff_position_goal_enabled=self.cfg.get('mission',{}).get('takeoff_position_goal_enabled',False),
                    fcu_climb_speed_limit_mps=(self.params.get('MPC_Z_VEL_MAX_UP') or [0,None])[1],
                    fcu_climb_acceleration_limit_mps2=(self.params.get('MPC_ACC_UP_MAX') or [0,None])[1],
                    takeoff_vertical_velocity_enabled=self.takeoff_only and self.cfg.get('mission',{}).get('takeoff_vertical_velocity_enabled',False),
                    attitude_checks_enabled=not self.takeoff_only or self.cfg.get('mission',{}).get('takeoff_attitude_checks_enabled',True),
                    companion_battery_monitor_enabled=self.cfg.get('mission',{}).get('companion_battery_monitor_enabled',True),
                    fcu_battery_parameters_modified=False,
                    ready_scope='companion_checks_only_FCU_arming_not_yet_requested',
                    actual_motion_enabled=False,mode=self.core.mode,armed=self.core.armed,
                    experiment='takeoff_land' if self.takeoff_only else ('align_only' if getattr(self,'align_only',False) else ('straight_pass' if getattr(self,'direct_pass',False) else 'traverse')),
                    acquisition='airborne_rgbd_then_usb_target' if getattr(self,'full_chain_enabled',False) else 'usb_hover_target' if getattr(self,'hover_policy',None) else 'takeoff_rgbd_observe' if getattr(self,'passive_rgbd',False) else ('none' if self.takeoff_only else ('airborne_rgbd' if self.airborne_geometry else 'surveyed')),
                    ground_window_detection_required=not(self.airborne_geometry or self.takeoff_only),
                    window_traversal_permitted=not self.takeoff_only and not getattr(self,'align_only',False) and not errors and all(self.control[k] for k in ('enable_motion','enable_traverse','pass_stage_approved')),
                    commissioning_trial=bool(deferred),
                    hover_commissioning=getattr(self,'hover_commissioning',False),
                    hover_target_policy=getattr(self,'hover_policy',None),
                    full_chain_policy=self.cfg.get('full_chain'),
                    full_chain_horizontal_abort_enabled=bool(getattr(self,'full_chain_enabled',False) and options and not options['simplified_takeoff_enabled']),
                    straight_commissioning=getattr(self,'straight_commissioning',False),
                    alignment_commissioning=getattr(self,'alignment_commissioning',False),
                    unverified_control_checks_accepted_for_alignment=sorted(k for k in __import__('alignment_policy').CONTROL_DEFERRALS if not self.control[k]) if getattr(self,'alignment_commissioning',False) else [],
                    unverified_checks_accepted_for_this_trial=sorted(k for k in deferred if self.cfg.get('acceptance',{}).get(k) is not True),
                    ground_samples=len(samples),z_span_m=max(samples)-min(samples) if samples else None)

    def request(self,action):
        def worker():
            from mavros_msgs.srv import SetMode,CommandBool
            try:
                with self.lock:
                    phase={'OFFBOARD':'MODE','ARM':'ARM','LAND':'LAND_REQUEST','POSCTL':'HANDOVER_REQUEST'}[action]
                    i=self.inputs()
                    landing_handover=(action=='LAND' and self.mission.phase=='LANDING_HANDOVER' and
                        getattr(self.mission,'return_journey',None) is not None and
                        self.mission.return_journey.on_ack and self.mission.return_journey.request_on)
                    if (self.mission.phase!=phase and not landing_handover) or self.mission.common_error(i) or i.emergency: return
                    if action in ('OFFBOARD','ARM') and (not i.landed or self.core.armed or not i.battery_ok):return
                if action=='ARM':r=self.ros.ServiceProxy('/mavros/cmd/arming',CommandBool)(value=True);accepted=r.success
                else:r=self.ros.ServiceProxy('/mavros/set_mode',SetMode)(base_mode=0,custom_mode={'LAND':'AUTO.LAND','OFFBOARD':'OFFBOARD','POSCTL':'POSCTL'}[action]);accepted=r.mode_sent
                result_code=getattr(r,'result',None) if action=='ARM' else None
                with self.lock:self.mission.command_finished(action,accepted,result_code=result_code)
                self.ros.loginfo('FCU_COMMAND action=%s accepted=%s mav_result=%s',action,accepted,result_code)
            except Exception as exc:
                with self.lock:self.mission.command_finished(action,False,error=type(exc).__name__+': '+str(exc))
                self.ros.logerr('Command failed: %s',exc)
        threading.Thread(target=worker,daemon=True).start()

    def publish(self,out):
        if out['position'] is None:return
        if self.motion.get_num_connections()!=1:raise RuntimeError('setpoint_subscriber_not_exclusive')
        P=self.types['PositionTarget'];m=P()
        m.header.stamp=self.ros.Time.now();m.header.frame_id=self.control['local_frame']
        m.coordinate_frame=P.FRAME_LOCAL_NED  # MAVROS converts ROS ENU to NED.
        m.type_mask=P.IGNORE_VX|P.IGNORE_VY|P.IGNORE_VZ|P.IGNORE_AFX|P.IGNORE_AFY|P.IGNORE_AFZ|P.IGNORE_YAW_RATE
        velocity=out.get('velocity_local_mps')
        if velocity is not None:
            from window_pass.geometry import vector,norm
            pair_transfer=(out.get('phase')=='POSTPASS' and getattr(getattr(self.mission,'full_chain',None),'scope',None)=='window_pass_two_targets')
            pass_feedforward=(out.get('phase')=='PASS' and self.mission.session_policy.get('smooth_pass'))
            return_feedforward=(out.get('phase') in ('RETURN_PREP','RETURN_HOME') and getattr(self.mission,'return_journey',None) is not None and self.mission.return_journey.active)
            limit=(1.0 if getattr(self.mission.full_chain,'index',0)>0 else .35) if pair_transfer else 1.5
            if return_feedforward:limit=self.mission.return_journey.move_speed(out['phase'])
            if not (pair_transfer or pass_feedforward or return_feedforward) or not vector(velocity) or norm(velocity)>limit+1e-9 or abs(velocity[2])>1e-9:
                raise ValueError('invalid smooth passage velocity feedforward')
            m.type_mask &= ~(P.IGNORE_VX|P.IGNORE_VY|P.IGNORE_VZ)
            m.velocity.x,m.velocity.y,m.velocity.z=velocity
        vz=out.get('vertical_velocity_mps')
        if vz is not None:
            if not self.takeoff_only or out.get('phase')!='TAKEOFF' or type(vz) not in (int,float) or not math.isfinite(vz) or not 0<=vz<=.10:
                raise ValueError('invalid takeoff vertical velocity command')
            m.type_mask=(m.type_mask & ~P.IGNORE_VZ) | P.IGNORE_PZ
            m.velocity.z=vz
        m.position.x,m.position.y,m.position.z=out['position'];m.yaw=out['yaw']
        self.motion.publish(m);self.mission.sent(time.monotonic())
        guard=getattr(self,'pose_time_guard',None)
        if guard is not None:
            guard.setpoint=dict(stamp=m.header.stamp.to_sec(),published_wall=time.monotonic(),position_local_m=list(out['position']),yaw=out['yaw'],velocity_local_mps=out.get('velocity_local_mps'),type_mask=m.type_mask,coordinate_frame=m.coordinate_frame,evidence='submitted_to_ROS_publisher_not_FCU_receipt')

    def prepare_ground_posctl(self):
        """Explicitly reset commander intent before creating any setpoint publisher.

        A failsafe POSCTL report can hide an internal OFFBOARD main state.
        Read-only preflight never calls this method; only --execute does.
        """
        from mavros_msgs.srv import SetMode
        def guard():
            i=self.inputs()
            if (not i.state_fresh or not self.core.connected or self.core.armed or
                not i.landed_fresh or not i.landed or not i.rc_ok or i.emergency or
                not i.estimator_ok or not i.graph_ok or not i.battery_ok or
                self.core.mode not in ('MANUAL','ALTCTL','POSCTL') or
                not self.parameters_ok(i.wall)):
                raise RuntimeError('ground_POSCTL_prepare_conditions_changed')
            if self.motion is not None:raise RuntimeError('ground_prepare_requires_no_output')
            if self.source_identity is not None and self.identity()!=self.source_identity:
                raise RuntimeError('localization_session_changed')
        with self.lock:guard()
        self.ros.wait_for_service('/mavros/set_mode',timeout=3)
        with self.lock:
            guard()
            before=self.rows['/mavros/state'][0]
            response=self.ros.ServiceProxy('/mavros/set_mode',SetMode)(base_mode=0,custom_mode='POSCTL')
            if not response.mode_sent:raise RuntimeError('ground_POSCTL_prepare_rejected')
        deadline=time.monotonic()+4
        observed=0;last=before
        while time.monotonic()<deadline:
            with self.lock:
                guard()
                received,msg=self.rows['/mavros/state']
                if received>last:
                    last=received
                    observed=observed+1 if msg.mode=='POSCTL' else 0
                    if observed>=2:
                        self.ros.loginfo('GROUND_POSCTL_INTENT_RESET: explicit request and two new state confirmations; no arm or setpoint sent')
                        return
            self.stop.wait(.05)
        raise RuntimeError('ground_POSCTL_prepare_not_observed')

    def preflight_report(self):
        # A periodic OFF refresh briefly publishes OFF_PENDING. Wait for a NEW
        # fresh OFF_ACK; never use an earlier ACK or treat pending as success.
        from hover_target import fresh_bridge
        deadline=time.monotonic()+2.
        while True:
            with self.lock:
                report=self.report()
                bridge=fresh_bridge(getattr(self,'hover_laser_row',None),time.monotonic(),getattr(self,'chain_scope','window_pass_single_target'))
                pending=(getattr(self,'full_chain_enabled',False) and
                    report.get('blockers')==['hover_laser_initial_off_not_acknowledged'] and
                    bridge and bridge.get('state')=='OFF_PENDING' and
                    bridge.get('startup_off_acked') is True and bridge.get('requested_on') is False and
                    bridge.get('exposure_used') is False and bridge.get('request_id')==0)
            if not pending or time.monotonic()>=deadline:return report
            self.stop.wait(.05)

    def run(self):
        time.sleep(3.5)
        if not self.execute and getattr(self,'full_chain_enabled',False):
            def read():
                with self.lock:return self.report()
            # Initial report retains the existing OFF refresh handling. Later
            # polls recheck every blocker without extending the sync deadline.
            report=wait_readonly_timesync(read,self.stop.wait,
                lambda:self.stop.is_set() or self.ros.is_shutdown(),
                initial=self.preflight_report())
        else:
            report=self.preflight_report()
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        if not self.execute:return 0 if report['ready'] else 2
        if not report['ready']:raise RuntimeError('PREFLIGHT_BLOCKED')
        # Revalidate the geometry/session immediately before taking ownership.
        if self.airborne_geometry or self.takeoff_only:
            if self.identity()!=self.source_identity:raise RuntimeError('localization_session_changed')
        else:
            geometry_identity(Path(self.cfg['geometry_file']).expanduser(),self.master,self.ros)
        self.prepare_ground_posctl()
        with self.lock:
            self.mission=Mission(self.control,self.cfg['mission'],airborne_geometry=self.airborne_geometry,experiment=self.experiment,alignment_commissioning=getattr(self,'alignment_commissioning',False),straight_commissioning=getattr(self,'straight_commissioning',False),adjustment_policy=self.cfg.get('prepass_adjustment'),session_window_policy=self.cfg.get('session_window'),session_identity=self.source_identity,hover_target_policy=self.cfg.get('hover_target'),full_chain_policy=self.cfg.get('full_chain'))
            if getattr(self,'visual_calibration',None):
                self.visual_calibration.identity=dict(self.source_identity)
                self.mission.visual_calibration=self.visual_calibration
            # Retain fresh data, not authority, from the read-only observer.
            old=self.core;new=self.mission.core
            for key in ('pose','poses','pose_speed','obs','previous_obs','samples','streams','latest_stamps',
                        'connected','armed','mode','airborne','graph_ok','graph_wall','graph_reason'):
                setattr(new,key,getattr(old,key))
            self.core=new
            self.motion=self.ros.Publisher(OUTPUT,self.types['PositionTarget'],queue_size=1,latch=False)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if self.motion.get_num_connections()==1 and self.graph_ok and self.graph_wall>deadline-3:break
            time.sleep(.02)
        else:raise RuntimeError('exclusive_output_handshake_failed')
        with self.lock:out=self.mission.start(self.inputs())
        self.publish(out)
        last_phase=None
        last_search_reason=None
        last_hover_log=0.;last_hover_stage=None
        last_decision_reason=None
        terminal=None
        with NonBlockingLog(self.ros.loginfo) as run_log:
            while not self.ros.is_shutdown():
                began=time.monotonic()
                with self.lock:
                    out=self.mission.tick(self.inputs())
                    self.publish(out)
                    out['supervisor_ros_stamp']=self.ros.Time.now().to_sec()
                    out['pose_input_diagnostics']=self.pose_time_guard.evidence()
                    out['duplicate_pose_messages']=getattr(self,'duplicate_pose_messages',0)
                    out['control_tick_duration_s']=time.monotonic()-began
                    if out['phase']!=last_phase:
                        run_log.info('AUTO_WINDOW %s: %s',out['phase'],out['reason'])
                        if out['phase']=='ADJUST':run_log.info('PREPASS_ADJUSTMENT %s',json.dumps(out.get('prepass_adjustment')))
                        last_phase=out['phase']
                    if out.get('hover_target'):
                        hover=out['hover_target'];stage=hover['stage']
                        if stage!=last_hover_stage or (out['phase']=='HOVER' and began-last_hover_log>=.5):
                            run_log.info('HOVER_TARGET %s',json.dumps(hover))
                            last_hover_log=began;last_hover_stage=stage
                    if out['phase']=='SEARCH' and out.get('search_evidence'):
                        search_reason=out['search_evidence']['reason']
                        if search_reason!=last_search_reason:
                            run_log.info('SEARCH_GATE %s',json.dumps(out['search_evidence']))
                            last_search_reason=search_reason
                    if out.get('passage_decision'):
                        decision_reason=out['passage_decision']['reason']
                        if decision_reason!=last_decision_reason:
                            run_log.info('PASSAGE_DECISION %s',json.dumps(out['passage_decision']))
                            last_decision_reason=decision_reason
                    if out['action']:self.request(out['action'])
                    if out['phase'] in TERMINAL:
                        terminal=out
                # Serialization and ROS diagnostic output must not hold the
                # same lock required by pose callbacks and the watchdog.
                self.diag.publish(self.types['String'](data=json.dumps(out)))
                if terminal is not None:break
                self.stop.wait(max(0.,1./self.control['control_hz']-(time.monotonic()-began)))
        if terminal is not None:
            if terminal['phase']=='PILOT_CONTROL':
                self.ros.logwarn('PILOT_CONTROL: POSCTL observed; pilot must land and disarm; no automatic landing')
            return self.finish_terminal(terminal)
        return 3

    def finish_terminal(self,terminal):
        # Unregister before waiting, with no lock held while late text arrives.
        with self.lock:
            if self.motion:self.motion.unregister();self.motion=None
        if terminal['phase']!='DONE':self.stop.wait(2.)
        with self.lock:
            if getattr(self,'passive_rgbd',False):
                terminal['rgbd_observation']={'motion_scope':'fixed_takeoff_column_no_alignment_no_traversal',
                    'samples':list(self.observation_rows),'unique_samples':len(self.observation_rows),
                    'validated_metric_samples':sum(r.get('metric_validation')=='ok' for r in self.observation_rows),
                    'not_a_geometry_accuracy_acceptance':True}
            if getattr(self,'pose_time_guard',None) is not None:
                terminal['pose_input_diagnostics']=self.pose_time_guard.evidence()
            terminal['recent_fcu_text']=list(self.fcu_text)
            terminal['fcu_state_at_exit']=dict(mode=self.core.mode,armed=self.core.armed,connected=self.core.connected)
        calibration=getattr(self,'visual_calibration',None)
        if calibration:
            try:
                terminal['visual_calibration']={'record_file':calibration.save('/home/abot/visual_aim_records'),
                                                'accepted_target_ids':sorted(calibration.pending),
                                                'physical_hit_verified':False}
            except (OSError,ValueError) as exc:
                terminal['visual_calibration']={'save_error':str(exc),'accepted_target_ids':sorted(calibration.pending)}
        print(json.dumps(terminal),flush=True)
        if getattr(self,'full_chain_enabled',False):return 0 if (terminal.get('full_chain') or {}).get('mission_complete') else 3
        return 0 if terminal['phase']=='DONE' or (terminal['phase']=='PILOT_CONTROL' and (terminal.get('takeoff_hover_completed') or terminal.get('alignment_completed') or terminal.get('straight_pass_completed') or (terminal.get('hover_target') or {}).get('completed'))) else 3


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--commissioning-takeoff',action='store_true',help='Explicit supervised low-speed takeoff test with recorded unperformed tests; never traversal')
    p.add_argument('--commissioning-alignment',action='store_true',help='Explicit separately accepted bounded alignment trial; never traversal')
    p.add_argument('--commissioning-full-chain',action='store_true',help='One supervisor: window pass, postpass hold, one USB target and confirmed POSCTL')
    p.add_argument('--commissioning-hover-target',action='store_true',help='Explicit single-target hover trial; no passage or multi-target flight')
    p.add_argument('--commissioning-straight-pass',action='store_true',help='Separately authorized finite straight-passage test')
    p.add_argument('--require-experiment',choices=['takeoff_land','traverse','align_only','straight_pass'])
    a=p.parse_args()
    cfg=json.loads(a.config.read_text())
    verify_experiment(cfg,a.require_experiment)
    from full_chain_policy import validate as validate_full_chain
    validate_full_chain(cfg,a.commissioning_full_chain)
    commissioning_deferrals(cfg,a.commissioning_takeoff)
    from alignment_policy import validate as validate_alignment_policy
    validate_alignment_policy(cfg,a.commissioning_alignment)
    from straight_pass_policy import validate as validate_straight_policy
    validate_straight_policy(cfg,a.commissioning_straight_pass)
    from hover_trial_policy import validate as validate_hover_policy
    validate_hover_policy(cfg,a.commissioning_hover_target)
    if sum(map(bool,(a.commissioning_takeoff,a.commissioning_alignment,a.commissioning_straight_pass,a.commissioning_hover_target,a.commissioning_full_chain)))>1:raise ValueError('commissioning scopes are mutually exclusive')
    import fcntl
    import rospy
    # Prevent this entry and existing hold/align from coexisting.
    lock=(Path.home()/'.window_pass_competition_hold.lock').open('a') if a.execute else None
    if lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rospy.init_node('autonomous_window_test',disable_signals=False)
    runtime=Runtime(cfg,a.execute,commissioning=a.commissioning_takeoff,alignment_commissioning=a.commissioning_alignment,straight_commissioning=a.commissioning_straight_pass,hover_commissioning=a.commissioning_hover_target,full_chain_commissioning=a.commissioning_full_chain)
    try:return runtime.run()
    finally:
        runtime.stop.set()
        if runtime.motion:runtime.motion.unregister()
        if lock:lock.close()
        # No shutdown arm/disarm/mode calls. FCU loss handling must be verified.

if __name__=='__main__':sys.exit(main())
