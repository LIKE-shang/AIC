#!/usr/bin/env python3
"""Prepare targets early; explicit one-shot run owns ground then one flight supervisor."""
import argparse,hashlib,json,math,os,signal,subprocess,sys,time,xmlrpc.client
from pathlib import Path
from policy import chosen,ground_done,same_start,validate_plan,Handoff,ReturnHandoff,flight_failure_reason
ROOT=Path(__file__).resolve().parent
FLIGHT=ROOT.parent/'flight'
MAP=Path('/home/abot/abot_ws/ground_map_aOrQKfU7')
GROUND=ROOT/'ground_stage_v6_yaw015.yaml'
PLAN=ROOT/'prepared.json'
ENTRY_BUILD='20261101_aic234471_ground_lidar_preflight'

def configure_ros_package_path():
    preferred=[str(FLIGHT/'rgbd'),str(FLIGHT/'vision_ws/src'),
               '/home/abot/window_pass_session_20261013']
    duplicates=set(preferred+['/home/abot/full_chain_20261030_lidar_recovery/vision_ws/src'])
    remaining=[]
    for path in os.environ.get('ROS_PACKAGE_PATH','').split(os.pathsep):
        if not path or path in duplicates:continue
        duplicates.add(path)
        remaining.append(path)
    os.environ['ROS_PACKAGE_PATH']=os.pathsep.join(preferred+remaining)

def validate_entry_binding(d):
    if d.get('entry_build')!=ENTRY_BUILD or d.get('entry_manifest_sha256')!=digest(ROOT/'manifest.json'):raise ValueError('prepared_entry_changed_reprepare_required')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):
    tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(d,ensure_ascii=False,indent=2));tmp.replace(p)
def invoke(args):
    subprocess.run(args,check=True)

def prompt_target_selection(values=None,input_fn=None,output_fn=None):
    if values is not None:return chosen(values)
    input_fn=input if input_fn is None else input_fn
    output_fn=print if output_fn is None else output_fn
    while True:
        output_fn('TARGET_PROMPT:请选择两个不同靶位，输入两个数字：1、2、3（例如13）')
        try:raw=input_fn()
        except EOFError:raise ValueError('target_selection_required')
        compact=''.join(raw.split()).replace('，',',').replace('、',',').replace(',','')
        if len(compact)==2 and all(c in '123' for c in compact) and compact[0]!=compact[1]:
            return chosen([int(compact[0]),int(compact[1])])
        output_fn('TARGET_PROMPT:输入无效，请选择两个不同的靶位编号。')
def spawn(label,args):
    log=(ROOT/(label+'.log')).open('ab')
    p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
    save(ROOT/(label+'.process.json'),{'pid':p.pid,'command':args,'boot':Path('/proc/sys/kernel/random/boot_id').read_text().strip()});return p

def lidar_publishers(master):
    return list(dict(master.getSystemState()[0]).get('/livox/lidar',[]))

def wait_for_unique_lidar(master,timeout=25,sleep=time.sleep,clock=time.monotonic):
    deadline=clock()+timeout;last_error=None
    while clock()<deadline:
        try:
            publishers=lidar_publishers(master)
            if len(publishers)>1:raise ValueError('multiple /livox/lidar publishers; inspect before LIO recovery')
            if len(publishers)==1:return publishers[0]
        except ValueError:raise
        except Exception as exc:last_error=exc
        sleep(.1)
    if last_error:raise ValueError('ROS master/MID360 startup timed out; inspect sensor power, network, and driver log: '+str(last_error))
    raise ValueError('MID360 did not publish /livox/lidar; inspect sensor power, network, and driver log')

def ensure_unique_lidar(master,node_names,start_driver,timeout=25,sleep=time.sleep,clock=time.monotonic):
    deadline=clock()+timeout;last_error=None;driver_started=False
    while clock()<deadline:
        try:
            publishers=lidar_publishers(master)
            if len(publishers)>1:raise ValueError('multiple /livox/lidar publishers; inspect before LIO recovery')
            if len(publishers)==1:return publishers[0]
            nodes=node_names()
        except ValueError:raise
        except Exception as exc:
            last_error=exc;sleep(.1);continue
        if not driver_started and not any('livox' in name.lower() for name in nodes):
            start_driver();driver_started=True
        sleep(.1)
    if last_error:raise ValueError('ROS master/MID360 startup timed out; inspect sensor power, network, and driver log: '+str(last_error))
    raise ValueError('MID360 did not publish /livox/lidar; inspect sensor power, network, and driver log')

def wait_for_unique_topic(master,topic,timeout=20.0,sleep=time.sleep,clock=time.monotonic):
    deadline=clock()+timeout
    latest=[]
    while clock()<deadline:
        try:
            latest=list(dict(master.getSystemState()[0]).get(topic,[]))
        except Exception:
            latest=[]
        if len(latest)>1:raise ValueError('PREPARE_CLASS=duplicate_status_publishers; topic='+topic)
        if len(latest)==1:return latest[0]
        sleep(.1)
    raise ValueError('PREPARE_CLASS=startup_component_not_ready; topic='+topic)

def validate_laser_bridge_status(bridge):
    required={'fault_latched':False,'startup_off_acked':True,'requested_on':False,'fcu_connected':True,'fcu_armed':False}
    bad={key:bridge.get(key) for key,value in required.items() if bridge.get(key) is not value}
    if bad:
        details={k:bridge.get(k) for k in ('state','fault_latched','startup_off_acked','requested_on','fcu_connected','fcu_armed','reason','error')}
        raise ValueError('PREPARE_CLASS=laser_bridge_unhealthy; '+json.dumps(details,sort_keys=True))

def classify_binding_failure(output):
    if 'missing telemetry: /mavros/local_position/odom' in output:
        return 'PREPARE_CLASS=fcu_local_odom_missing; FCU did not publish fused local odometry; inspect estimator fusion flags and vision_pose input'
    return 'PREPARE_CLASS=flight_binding_failed'

class StartupProbeTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host)
        connection.timeout=2
        return connection

def probe_startup_node(uri):
    with xmlrpc.client.ServerProxy(uri,transport=StartupProbeTransport()) as node:
        code,message,pid=node.getPid('/competition_prepare_bootstrap')
    if code!=1 or type(pid) is not int or pid<=0:
        raise ValueError('invalid node PID reply: '+repr((code,message,pid)))
    return pid

def local_lio_processes():
    root=Path('/proc')
    if not root.is_dir():return []
    found=[]
    for process in root.iterdir():
        if not process.name.isdigit():continue
        try:
            args=[x.decode(errors='replace') for x in (process/'cmdline').read_bytes().split(bytes([0])) if x]
        except FileNotFoundError:continue
        if args and Path(args[0]).name in ('fastlio_mapping','laserMapping'):
            found.append({'pid':int(process.name),'argv':args})
    return found

def require_prepare_sources(master,node_names,probe=probe_startup_node):
    pubs=dict(master.getSystemState()[0]);issues=[]
    controllers={topic:pubs[topic] for topic in ('/mavros/setpoint_raw/local','/mavros/setpoint_position/local',
                 '/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_raw/attitude') if pubs.get(topic)}
    if controllers:issues.append({'class':'flight_controller_present','publishers':controllers})
    mavros=pubs.get('/mavros/state',[])
    if mavros and mavros!=['/mavros']:
        issues.append({'class':'mavros_source_conflict','publishers':mavros})
    if '/mavros' in node_names or mavros==['/mavros']:
        uri=None
        try:
            uri=master.lookupNode('/mavros');probe(uri)
        except Exception as exc:
            issues.append({'class':'mavros_registration_unreachable','node':'/mavros','uri':uri,
                           'error':str(exc),'next_step':'inspect the registered node and current processes before removing stale registration; no automatic cleanup'})
    foreign={topic:names for topic,names in pubs.items()
             if topic.rsplit('/',1)[-1] in ('Odometry','odometry') and names
             and topic!='/window_pass_lio/Odometry'}
    other_nodes=sorted(n for n in node_names if n.rsplit('/',1)[-1]=='laserMapping' and n!='/window_pass_lio/laserMapping')
    own_pid=None
    if pubs.get('/window_pass_lio/Odometry')==['/window_pass_lio/laserMapping']:
        try:own_pid=probe(master.lookupNode('/window_pass_lio/laserMapping'))
        except Exception as exc:issues.append({'class':'competition_lio_registration_unreachable','error':str(exc)})
    processes=[p for p in local_lio_processes() if p['pid']!=own_pid]
    if foreign or other_nodes or processes:
        issues.append({'class':'external_lio_present','publishers':foreign,'nodes':other_nodes,'processes':processes,
                       'next_step':'end the other localization session on trusted ground before using the competition localization chain; no automatic stop or second LIO'})
    if issues:raise ValueError('PREPARE_CLASS=startup_source_conflict; '+json.dumps(issues,sort_keys=True))
    return pubs

def prepare_support_after_mavros(master,node_names,start_mavros,wait_mavros,prepare_support,probe=probe_startup_node):
    pubs=require_prepare_sources(master,node_names,probe)
    if '/mavros' not in node_names and not pubs.get('/mavros/state'):
        start_mavros()
    # An advertised topic is not evidence of fresh telemetry, including after
    # startup. This wait is read-only; existing ground gates remain in force.
    wait_mavros()
    pubs=dict(master.getSystemState()[0])
    if pubs.get('/hover_target_laser/status'):
        prepare_support()
    else:
        print('PREPARE_SUPPORT=ABSENT; ground restore will start the OFF bridge',flush=True)

def prepare(ids,area,recover_landed_mode=False):
    if not area:raise ValueError('explicit current complete ground and flight area confirmation required')
    import fcntl
    prepare_lock=(ROOT/'run.lock').open('a')
    fcntl.flock(prepare_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Stop the previous robot ROS session only after independent ground proof.
    # This helper returns only once its exact audited process list has exited.
    invoke([sys.executable,'-B',str(ROOT/'prepare_restart.py')])
    # All slow cold-start operations occur here, before the official run.
    # The old restorer starts a base with default MAVROS and may race its own
    # missing-node fallback. Give it a base with those two children disabled.
    import rosgraph,rosnode,uuid
    master=rosgraph.Master('/competition_prepare_bootstrap')
    if not rosgraph.is_master_online():
        base='/home/abot/autonomous_window_test_20261019_v14/base_10hz_reboot.launch'
        spawn('base_shared',['roslaunch',base,'run_id:='+uuid.uuid4().hex,'start_mavros:=false','start_lio:=false','start_camera:=false','start_detector:=false','start_pose:=false'])
        wait_for_unique_lidar(master)
    else:
        ensure_unique_lidar(master,rosnode.get_node_names,
                            lambda:spawn('lidar_driver',['roslaunch','livox_ros_driver2','msg_MID360.launch']))
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState,RCIn
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String
    rospy.init_node('competition_prepare_readonly',anonymous=True)
    sys.path[:0]=[str(FLIGHT/'autonomous_window_test'),str(FLIGHT/'autonomous_window_test/scripts')]
    from ground_snapshot import await_mavros_startup
    from full_chain_ground import spawn as spawn_ground_component
    prepare_support_after_mavros(master,set(rosnode.get_node_names()),
        lambda:spawn_ground_component('mavros',['python3','/home/abot/window_pass_session_20261013/scripts/session.py','_mavros']),
        lambda:await_mavros_startup(rospy,State,timeout=15),
        lambda:invoke(['python3',str(FLIGHT/'autonomous_window_test/scripts/prepare_two_target_support.py'),'--targets']+[str(x) for x in ids]))
    from prepare_mode import prepare_startup_mode
    try:
        mode_evidence=prepare_startup_mode(rospy,master,authorized=recover_landed_mode)
    except Exception as exc:
        save(ROOT/'prepare_mode_recovery.json',{'ready':False,'authorized':recover_landed_mode,'error':str(exc)})
        raise
    save(ROOT/'prepare_mode_recovery.json',dict(mode_evidence,ready=True,authorized=recover_landed_mode))
    invoke(['bash',str(FLIGHT/'ground.bash'),'--restore'])
    bridge_master=rosgraph.Master(rospy.get_name())
    bridge_node=wait_for_unique_topic(bridge_master,'/hover_target_laser/status')
    if bridge_node!='/hover_target_laser':
        raise ValueError('PREPARE_CLASS=laser_bridge_source_mismatch; publisher='+repr(bridge_node))
    try:
        bridge=json.loads(rospy.wait_for_message('/hover_target_laser/status',String,timeout=3).data)
    except Exception as exc:
        raise ValueError('PREPARE_CLASS=laser_bridge_status_unavailable; '+str(exc))
    validate_laser_bridge_status(bridge)
    sys.path.insert(0,str(FLIGHT/'autonomous_window_test'))
    from ground_snapshot import latest_messages,ground_rejection,snapshot_details
    master=rosgraph.Master(rospy.get_name())
    def guard():
        graph=dict(master.getSystemState()[0])
        if graph.get('/mavros/setpoint_raw/local'):raise ValueError('flight controller already present')
        topics=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]
        s,e,r=latest_messages(rospy,topics,timeout=3,max_age=1.5,reject=ground_rejection)
        now=rospy.Time.now().to_sec()
        if any((m._connection_header or {}).get('callerid')!='/mavros' or not -.02<=now-m.header.stamp.to_sec()<1.5 for m in (s,e,r)) or not s.connected or s.armed or s.system_status!=3 or e.landed_state!=1 or len(r.channels)<7 or any(not 900<=r.channels[k]<=1200 for k in (5,6)):
            raise ValueError('trusted unarmed landed CH6 released CH7 off required; '+snapshot_details(rospy,topics,(s,e,r),1.5))
        return graph
    graph=guard()
    nodes=set(__import__('rosnode').get_node_names())
    if '/rosserial_starrobot' not in nodes:
        spawn('chassis',['roslaunch','starrobot_bringup','minimal.launch'])
    # This node has no second LIO. All stages use /window_pass_lio/Odometry.
    if not graph.get('/localization'):
        if '/global_localization' in nodes or '/transform_fusion' in nodes:raise ValueError('partial localization exists; inspect before duplicating')
        guard()
        spawn('shared_localization',['roslaunch',str(ROOT/'shared_localization.launch')])
        pub=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1,latch=True)
        deadline=time.monotonic()+60
        while not rospy.is_shutdown() and time.monotonic()<deadline and not pub.get_num_connections():time.sleep(.1)
        if not pub.get_num_connections():raise ValueError('global localization did not subscribe initial pose')
        # User-approved map start is (0,0), held heading is map +x.
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=rospy.Time.now();m.pose.pose.orientation.w=1.;pub.publish(m)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            try:
                loc=rospy.wait_for_message('/localization',Odometry,timeout=2)
                if 0<=rospy.Time.now().to_sec()-loc.header.stamp.to_sec()<.5:break
            except rospy.ROSException:pass
        else:raise ValueError('map localization cold initialization not ready')
    loc=rospy.wait_for_message('/localization',Odometry,timeout=3)
    if loc.header.frame_id!='map' or math.hypot(loc.pose.pose.position.x,loc.pose.pose.position.y)>.10:raise ValueError('prepare requires existing map origin; do not assume a new map origin')
    graph=guard()
    if graph.get('/cmd_vel'):
        if graph.get('/cmd_vel')!=['/ground_straight_nav']:raise ValueError('cmd_vel owner exists; no competing controller')
        uri=master.lookupNode('/ground_straight_nav');_,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        if str(ROOT/'ground_straight_nav/scripts/straight_nav_node.py') not in Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode():raise ValueError('unknown ground controller')
    else:
        spawn('ground_nav',['python3',str(ROOT/'ground_straight_nav/scripts/straight_nav_node.py'),'--config',str(GROUND),'--pose-topic','/localization'])
    # Reload our inactive node before binding: READY may still hold an older
    # in-memory profile even when the disk manifest has already been updated.
    status=json.loads(rospy.wait_for_message('/ground_straight_nav/status',String,timeout=5).data)
    if status.get('state') in ('READY','ABORT','DONE'):
        if status.get('dry_run') or status.get('command')!={'vx':0.,'vy':0.,'wz':0.}:raise ValueError('own ground controller must be inactive and zero before reload')
        guard()
        import rosnode
        rosnode.kill_nodes(['/ground_straight_nav'])
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if not dict(master.getSystemState()[0]).get('/cmd_vel'):break
            time.sleep(.1)
        else:raise ValueError('old own ground controller did not stop')
        spawn('ground_nav',['python3',str(ROOT/'ground_straight_nav/scripts/straight_nav_node.py'),'--config',str(GROUND),'--pose-topic','/localization'])
    rospy.wait_for_service('/ground_straight_nav/start',timeout=10)
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        status=json.loads(rospy.wait_for_message('/ground_straight_nav/status',String,timeout=2).data)
        if status.get('state')=='READY':break
    else:raise ValueError('ground navigator not idle:'+str(status))
    # Binder performs original 8s stationary sampling and full read-only preflight HERE.
    result=subprocess.run(['bash',str(FLIGHT/'select_two.bash'),'--targets']+[str(x) for x in ids]+['--current-flight-area-confirmed'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
    (ROOT/'prepare_flight.log').write_text(result.stdout)
    if result.returncode:raise ValueError(classify_binding_failure(result.stdout)+'; see prepare_flight.log')
    trial=next((line.split('=',1)[1] for line in result.stdout.splitlines() if line.startswith('TRIAL_DIRECTORY=')),None)
    cfg=json.loads((Path(trial)/'config.json').read_text())
    graph=guard();sources={}
    for topic in ('/localization','/ground_straight_nav/status','/window_pass_lio/Odometry'):
        names=graph.get(topic,[])
        if len(names)!=1:raise ValueError('unique source required:'+topic)
        sources[topic]=[names[0],master.lookupNode(names[0])]
    invoke(['python3',str(ROOT/'zero_message_check.py'),'--trial',trial])
    d={'schema':'competition_20261101_aic234471','entry_build':ENTRY_BUILD,'entry_manifest_sha256':digest(ROOT/'manifest.json'),'ready':True,'boot_id':cfg['site_identity']['boot_id'],'selected_target_ids':ids,'flight_manifest_sha256':digest(FLIGHT/'MANIFEST_SHA256.json'),'ground_config_sha256':digest(GROUND),'flight_config':str(Path(trial)/'config.json'),'flight_config_sha256':digest(Path(trial)/'config.json'),'sources':sources,'snapshot':cfg['site_evidence']['snapshot'],'prepared_wall':time.time(),'physical_hit_verified':False}
    save(PLAN,d);print('PRECHECK_READY targets='+','.join(str(x) for x in ids));print('NEXT_COMMAND=bash '+str(ROOT/'competition.bash')+' run --execute')

def run(execute):
    if not execute:raise ValueError('run requires explicit --execute; no motion sent')
    import fcntl
    lock=(ROOT/'run.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    flight_lock=(Path.home()/'.window_pass_competition_hold.lock').open('a');fcntl.flock(flight_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    d=json.loads(PLAN.read_text());validate_plan(d,Path('/proc/sys/kernel/random/boot_id').read_text().strip(),digest(FLIGHT/'MANIFEST_SHA256.json'),digest(GROUND));validate_entry_binding(d)
    used=ROOT/('run.used.'+digest(PLAN)+'.json')
    if used.exists():raise ValueError('competition episode already started; prepare a fresh session')
    cfgpath=Path(d['flight_config'])
    if digest(cfgpath)!=d['flight_config_sha256']:raise ValueError('prepared flight config changed')
    sys.path[:0]=[str(FLIGHT/'autonomous_window_test'),str(FLIGHT/'autonomous_window_test/scripts')]
    import rospy,rosgraph
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from auto_window import Runtime
    from run_recorded_hover_trial import TOPICS
    rospy.init_node('autonomous_window_test',disable_signals=False)
    runtime=Runtime(json.loads(cfgpath.read_text()),True,full_chain_commissioning=True)
    master=rosgraph.Master(rospy.get_name());rows={};record=None;log=None;ground_started=False;stage='PRESTART'
    episode=str(time.monotonic_ns());recordbase=ROOT/('competition_telemetry_'+episode)
    def cb(msg):rows['ground']=(time.monotonic(),(msg._connection_header or {}).get('callerid'),json.loads(msg.data))
    sub=rospy.Subscriber('/ground_straight_nav/status',String,cb,queue_size=2)
    def sources():
        graph=dict(master.getSystemState()[0])
        for topic,expected in d['sources'].items():
            if graph.get(topic)!=[expected[0]] or master.lookupNode(expected[0])!=expected[1]:raise ValueError('prepared source changed:'+topic)
        if runtime.identity()!=d['snapshot']['identity']:raise ValueError('flight localization identity changed')
        return graph
    def ground_guard():
        i=runtime.inputs()
        if not i.state_fresh or runtime.core.armed or not i.landed_fresh or not i.landed or not i.rc_ok or i.emergency:raise ValueError('ground state/RC changed')
        if not i.estimator_ok or not i.graph_ok:
            evidence={'estimator_ok':i.estimator_ok,'estimator_reason':i.estimator_reason,
                      'estimator_evidence':i.estimator_evidence,'graph_ok':i.graph_ok,
                      'graph_reason':runtime.graph_reason,'graph_age_s':i.wall-runtime.graph_wall,
                      'battery_policy':'reference_only'}
            raise ValueError('localization/control graph invalid during ground stage: '+json.dumps(evidence))
        sources()
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            time.sleep(.1)
            if rows and runtime.preflight_report()['ready']:break
        else:raise ValueError('competition preflight not ready:'+str(runtime.preflight_report()))
        same_start(d['snapshot'],{'identity':runtime.identity(),'position_local_m':runtime.core.pose.p})
        sources();ground_guard()
        row=rows.get('ground')
        if not row or row[1]!=d['sources']['/ground_straight_nav/status'][0] or time.monotonic()-row[0]>.5 or row[2].get('dry_run') or row[2].get('state') not in ('READY',):raise ValueError('ground navigator not fresh and idle')
        save(used,{'selected_target_ids':d['selected_target_ids'],'boot_id':d['boot_id'],'started':time.time(),'flight_authority_activated':False})
        log=recordbase.with_suffix('.log').open('ab')
        record=subprocess.Popen(['rosbag','record','-O',str(recordbase.with_suffix('.bag')),'--duration=780']+TOPICS+['/ground_straight_nav/status','/localization','/cmd_vel'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        time.sleep(.5)
        if record.poll() is not None:raise ValueError('competition recorder failed')
        rospy.wait_for_service('/ground_straight_nav/start',timeout=3)
        response=rospy.ServiceProxy('/ground_straight_nav/start',Trigger)()
        if not response.success:raise ValueError('ground start rejected:'+response.message)
        ground_started=True;stage='GROUND';deadline=time.monotonic()+480;handoff=Handoff()
        print('COMPETITION GROUND_RUNNING targets='+str(d['selected_target_ids']),flush=True)
        while time.monotonic()<deadline and not rospy.is_shutdown():
            ground_guard();row=rows.get('ground')
            if not row or row[1]!=d['sources']['/ground_straight_nav/status'][0] or time.monotonic()-row[0]>.5:raise ValueError('ground status stale or source changed')
            if handoff.observe(row[2],time.monotonic()-row[0],row[1],d['sources']['/ground_straight_nav/status'][0]):break
            time.sleep(.05)
        else:raise ValueError('ground timeout')
        stage='HANDOFF'
        return_gate=ReturnHandoff(d['snapshot']);deadline=time.monotonic()+return_gate.maximum_wait_s
        while time.monotonic()<deadline and not rospy.is_shutdown():
            ground_guard();row=rows.get('ground')
            if not row or row[1]!=d['sources']['/ground_straight_nav/status'][0] or time.monotonic()-row[0]>.5:raise ValueError('return ground status stale or source changed')
            pose=runtime.core.pose
            if pose is None or not runtime.core.fresh('pose',rospy.Time.now().to_sec(),time.monotonic(),runtime.control['pose_timeout_s']):raise ValueError('return FCU pose stale')
            if return_gate.observe({'identity':runtime.identity(),'position_local_m':pose.p},row[2],pose.stamp,time.monotonic()):break
            time.sleep(.02)
        else:
            save(ROOT/'return_handoff.json',return_gate.status())
            raise ValueError('return_handoff_timeout:'+return_gate.reason)
        save(ROOT/'return_handoff.json',return_gate.status())
        from window_pass.geometry import yaw,wrap
        if abs(wrap(yaw(runtime.core.pose.q)-yaw(d['snapshot']['orientation_xyzw'])))>.0872665:raise ValueError('returned flight heading changed')
        # Only the recorded own ground controller may be stopped; LIO/pose remain alive.
        import rosnode
        name=d['sources']['/ground_straight_nav/status'][0]
        uri=master.lookupNode(name);_,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        command=Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
        if str(ROOT/'ground_straight_nav/scripts/straight_nav_node.py') not in command:raise ValueError('unrecognized ground controller process')
        rospy.ServiceProxy('/ground_straight_nav/stop',Trigger)();rosnode.kill_nodes([name]);ground_started=False
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if not dict(master.getSystemState()[0]).get('/cmd_vel'):break
            time.sleep(.05)
        else:raise ValueError('ground command owner not released')
        sub.unregister()
        # Drop only our own ICP/map workers after the chassis has stopped. The
        # LIO, MAVROS, guarded flight pose and cameras keep the same processes.
        marker=ROOT/'shared_localization.process.json'
        if marker.exists():
            own=json.loads(marker.read_text());parent=own['pid']
            if own['boot']==d['boot_id'] and Path('/proc/%d/cmdline'%parent).exists():
                cmd=Path('/proc/%d/cmdline'%parent).read_bytes().replace(b'\0',b' ').decode()
                if str(ROOT/'shared_localization.launch') not in cmd or os.getpgid(parent)!=parent:raise ValueError('map worker group ownership changed')
                os.killpg(parent,signal.SIGINT)
        print('COMPETITION GROUND_RETURN_CONFIRMED -> existing flight supervisor; no second command needed',flush=True)
        # SAME Runtime has watched LIO/FCU during the full ground stage. Its normal
        # fresh report, POSCTL intent reset, OFFBOARD prestream and one ARM still run.
        stage='FLIGHT'
        result=runtime.run()
        terminal=runtime.mission.output() if runtime.mission is not None else {}
        report={'exit_code':result,'selected_target_ids':d['selected_target_ids'],'ground_return_confirmed':True,'physical_hit_verified':False,'pilot_landing_required':terminal.get('phase')!='DONE','automatic_landing_enabled':True,'flight_terminal':terminal,'telemetry_bag':str(recordbase.with_suffix('.bag'))}
        if result:
            report.update(stage='FLIGHT',reason=flight_failure_reason(terminal))
            save(ROOT/'latest_failure.json',report)
            print('COMPETITION_ABORT '+json.dumps({'stage':'FLIGHT','reason':report['reason'],'telemetry_bag':report['telemetry_bag']}),flush=True)
        save(ROOT/'result.json',report)
        return result
    except BaseException as exc:
        # Save before STOP service can overwrite the live ground status.
        failure={'stage':stage,'error_type':type(exc).__name__,'reason':str(exc),'selected_target_ids':d['selected_target_ids'],'boot_id':d['boot_id'],'ground_snapshot':rows.get('ground',[None,None,None])[2],'telemetry_bag':str(recordbase.with_suffix('.bag')),'flight_started':runtime.mission is not None,'return_handoff':return_gate.status() if 'return_gate' in locals() else None,'physical_hit_verified':False}
        save(ROOT/('failure_'+episode+'.json'),failure);save(ROOT/'latest_failure.json',failure)
        print('COMPETITION_ABORT '+json.dumps(failure,ensure_ascii=False),flush=True)
        raise
    finally:
        if ground_started:
            try:rospy.ServiceProxy('/ground_straight_nav/stop',Trigger)()
            except Exception:pass
        runtime.stop.set()
        if runtime.motion:runtime.motion.unregister()
        if record and record.poll() is None:
            os.killpg(record.pid,signal.SIGINT)
            try:record.wait(timeout=10)
            except subprocess.TimeoutExpired:pass
        if log:log.close()

def check():
    d=json.loads(PLAN.read_text());validate_plan(d,Path('/proc/sys/kernel/random/boot_id').read_text().strip(),digest(FLIGHT/'MANIFEST_SHA256.json'),digest(GROUND));validate_entry_binding(d)
    if digest(Path(d['flight_config']))!=d['flight_config_sha256']:raise ValueError('prepared flight config changed')
    sys.path[:0]=[str(FLIGHT/'autonomous_window_test'),str(FLIGHT/'autonomous_window_test/scripts')]
    import rospy
    from auto_window import Runtime
    rospy.init_node('autonomous_window_test',disable_signals=False)
    r=Runtime(json.loads(Path(d['flight_config']).read_text()),False,full_chain_commissioning=True)
    try:
        time.sleep(3.5)
        same_start(d['snapshot'],{'identity':r.identity(),'position_local_m':r.core.pose.p})
        graph=dict(r.master.getSystemState()[0])
        for topic,expected in d['sources'].items():
            if graph.get(topic)!=[expected[0]] or r.master.lookupNode(expected[0])!=expected[1]:raise ValueError('prepared source changed:'+topic)
        report=r.preflight_report()
        result={'selected_target_ids':d['selected_target_ids'],'flight_readonly_preflight':report,'setpoints_sent':0,'ground_start_calls':0,'arm_calls':0,'laser_on_requests':0,'ready':report['ready'],'whole_competition_physically_verified':False}
        save(ROOT/'ground_check.json',result)
        if result['ready']:
            print('PRECHECK_READY stage=flight_readonly')
            return 0
        print('PRECHECK_FAILED stage=flight_readonly')
        return 2
    finally:r.stop.set()

def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='mode',required=True)
    a=s.add_parser('prepare');a.add_argument('--targets',nargs='+',type=int);a.add_argument('--current-flight-area-confirmed',action='store_true')
    a.add_argument('--recover-landed-mode',action='store_true',help='explicitly permit one disarmed-ground AUTO.LAND to ALTCTL recovery before pose startup')
    a=s.add_parser('run');a.add_argument('--execute',action='store_true')
    s.add_parser('check')
    a=p.parse_args()
    configure_ros_package_path()
    if a.mode=='prepare':prepare(prompt_target_selection(a.targets,output_fn=lambda m:print(m,file=sys.stderr,flush=True)),a.current_flight_area_confirmed,recover_landed_mode=a.recover_landed_mode);return 0
    if a.mode=='check':return check()
    return run(a.execute)
if __name__=='__main__':sys.exit(main())
