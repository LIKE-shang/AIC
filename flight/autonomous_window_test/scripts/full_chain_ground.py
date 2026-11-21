#!/usr/bin/env python3
"""Ground-only source inventory and optional missing-node recovery. Never arms or sends a flight setpoint."""
import argparse,json,os,subprocess,sys,time,uuid,xmlrpc.client
from pathlib import Path
BUNDLE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(BUNDLE/'autonomous_window_test'))
from ground_snapshot import latest_messages,ground_rejection,snapshot_details,await_mavros_startup
BASE=Path('/home/abot/autonomous_window_test_20261019_v14/base_10hz_reboot.launch')

def spawn(name,args):
    log=BUNDLE/'ground_logs';log.mkdir(exist_ok=True)
    marker=log/(name+'.pid.json')
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if marker.exists():
        old=json.loads(marker.read_text())
        if old.get('boot_id')==boot and Path('/proc/%s'%old.get('pid')).exists():
            raise ValueError('component process already started; inspect before retry:'+name)
    stream=(log/(name+'.log')).open('ab')
    p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    marker.write_text(json.dumps({'pid':p.pid,'boot_id':boot,'command':args}))
    stream.close();print(json.dumps({'started':name,'pid':p.pid,'command':args}),flush=True)
    return p

def start_ground_pose(ground, start, graph, describe_state):
    # Re-sample after cold LIO startup. Never reuse the earlier mode or graph.
    state=ground()
    pubs=graph()
    evidence=describe_state(state)
    if pubs.get('/mavros/setpoint_raw/local'):
        raise ValueError('flight setpoint publisher present; no recovery; '+evidence)
    if len(pubs.get('/window_pass_lio/Odometry',[]))!=1:
        raise ValueError('LIO source ambiguous before pose startup; '+evidence)
    if pubs.get('/window_pass_competition/status') or pubs.get('/mavros/vision_pose/pose'):
        raise ValueError('pose bridge appeared during startup; inspect before restoration; '+
                         json.dumps({'status_publishers':pubs.get('/window_pass_competition/status',[]),
                                     'vision_publishers':pubs.get('/mavros/vision_pose/pose',[])},sort_keys=True))
    if state.mode!='ALTCTL':
        raise ValueError('manually select ALTCTL before starting the vision-pose bridge; observed='+state.mode+'; '+evidence)
    print('POSE_STARTUP_SNAPSHOT '+evidence,flush=True)
    return start('guarded_pose',['python3',str(BUNDLE/'autonomous_window_test/scripts/start_full_chain_pose.py'),'--run'])


def pose_bridge_evidence(pubs, read_status):
    """Explain a refused partial graph; status never grants recovery/readiness."""
    nodes=pubs.get('/window_pass_competition/status',[])
    evidence={'status_publishers':nodes,
              'vision_publishers':pubs.get('/mavros/vision_pose/pose',[]),
              'diagnostic_only':True}
    if len(nodes)==1:
        try:
            msg=read_status()
            caller=(msg._connection_header or {}).get('callerid','')
            evidence['status_caller']=caller
            if caller!=nodes[0]:
                evidence['status_error']='untrusted status source'
            else:
                data=json.loads(msg.data)
                fields=('state','real_output_state','real_output_active','real_fault',
                        'source_age_s','ground_startup_mode','horizontal_flags_valid',
                        'local_odom_received','hover_raw_lio_guard','raw_lio_guard_scope')
                evidence['status']={key:data[key] for key in fields if key in data}
        except Exception as exc:
            evidence['status_error']=str(exc)
    return json.dumps(evidence,sort_keys=True)



def require_existing_pose_bridge(pubs, read_status):
    nodes=pubs.get('/window_pass_competition/status',[])
    if len(nodes)!=1 or pubs.get('/mavros/vision_pose/pose')!=nodes:
        raise ValueError('pose bridge partial/ambiguous; inspect before restoration; '+
                         pose_bridge_evidence(pubs,read_status))
    msg=read_status()
    caller=(msg._connection_header or {}).get('callerid','')
    if caller!=nodes[0]:
        raise ValueError('untrusted pose status source; expected='+nodes[0]+'; observed='+str(caller))
    status=json.loads(msg.data)
    if status.get('raw_lio_motion_limits')!={'max_pose_step_m':.35,'max_pose_speed_mps':2.2}:
        raise ValueError('pose limits require explicit --restore --upgrade-pose')
    if status.get('raw_lio_guard_scope')!='RouteContinuity':
        raise ValueError('existing bridge is not full-chain guarded source; inspect/replace on ground')
    return status


def latched_dead_bridge(pubs,read_status):
    # Recognise only a stopped, latched bridge: status alive, no vision publisher, output inactive, fault set.
    nodes=pubs.get('/window_pass_competition/status',[])
    if len(nodes)!=1 or pubs.get('/mavros/vision_pose/pose'):return None
    if not nodes[0].startswith('/window_pass_competition_pose_'):return None
    msg=read_status()
    if (msg._connection_header or {}).get('callerid','')!=nodes[0]:return None
    status=json.loads(msg.data)
    if status.get('real_output_active') is not False or not status.get('real_fault'):return None
    if status.get('raw_lio_guard_scope')!='RouteContinuity':return None
    return nodes[0],status


def replace_latched_pose_bridge(pubs,read_status,ground,graph,node_process,kill,pid_alive,sleep=time.sleep,clock=time.monotonic):
    # Stop a latched bridge on trusted disarmed ground so a fresh one can be started. The latch is never cleared in place.
    found=latched_dead_bridge(pubs,read_status)
    if found is None:return False
    node,status=found
    ground()
    current=graph()
    if current.get('/mavros/setpoint_raw/local'):raise ValueError('flight setpoint publisher present; latched bridge not replaced')
    if len(current.get('/window_pass_lio/Odometry',[]))!=1:raise ValueError('LIO source ambiguous; latched bridge not replaced')
    pid,command=node_process(node)
    if str(BUNDLE/'autonomous_window_test/scripts/start_full_chain_pose.py') not in command:
        raise ValueError('latched bridge is not this bundle process; no replacement; pid=%s'%pid)
    print('LATCHED_BRIDGE_REPLACE '+json.dumps({'node':node,'pid':pid,'real_fault':status.get('real_fault'),'real_published':status.get('real_published')},sort_keys=True),flush=True)
    kill([node])
    deadline=clock()+8
    while clock()<deadline:
        now=graph()
        if not now.get('/window_pass_competition/status') and not now.get('/mavros/vision_pose/pose') and not pid_alive(pid):return True
        sleep(.1)
    raise ValueError('latched pose bridge did not exit; inspect pid %s'%pid)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--restore',action='store_true');p.add_argument('--upgrade-pose',action='store_true');a=p.parse_args()
    if a.upgrade_pose and not a.restore:p.error('--upgrade-pose requires --restore')
    if a.restore:
        import fcntl
        lock=(BUNDLE/'.ground_restore.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    import rosgraph
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    print('CURRENT_BOOT_ID='+boot,flush=True)
    if boot=='523c469a-4806-441e-af45-fd0d53fb95aa':raise ValueError('historical boot is not a fresh power-on session')
    if not rosgraph.is_master_online():
        if not a.restore:print('ROS_MASTER=OFFLINE');return 2
        if not BASE.is_file():raise ValueError('known 10Hz base entry missing; inspect instead of falling back to 50Hz')
        run=uuid.uuid4().hex
        spawn('base_10hz',['roslaunch',str(BASE),'run_id:='+run,'start_camera:=false','start_detector:=false','start_pose:=false'])
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and not rosgraph.is_master_online():time.sleep(.2)
        if not rosgraph.is_master_online():raise ValueError('base startup did not create ROS master; inspect ground_logs')
    import rospy
    from mavros_msgs.msg import State,ExtendedState,RCIn
    from std_msgs.msg import String
    rospy.init_node('full_chain_ground_recovery_readonly',anonymous=True,disable_signals=True)
    master=rosgraph.Master(rospy.get_name())
    def graph():return dict(master.getSystemState()[0])
    def ground():
        pubs=graph()
        if pubs.get('/mavros/setpoint_raw/local'):raise ValueError('flight setpoint publisher present; no recovery')
        topics=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]
        values=latest_messages(rospy,topics,timeout=4,max_age=2,reject=ground_rejection)
        s,e,r=values;now=rospy.Time.now().to_sec()
        if any((m._connection_header or {}).get('callerid')!='/mavros' or not -.02<=now-m.header.stamp.to_sec()<2 for m in values):raise ValueError('fresh trusted MAVROS required; '+snapshot_details(rospy,topics,values,2))
        if not s.connected or s.armed or s.system_status!=3 or e.landed_state!=1 or len(r.channels)<7 or not all(900<=r.channels[k]<=1200 for k in (5,6)):
            raise ValueError('connected disarmed ground, kill released, A off required; '+snapshot_details(rospy,topics,values,2))
        return s
    if a.restore:
        import rosnode
        def await_publisher(topic,seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                if graph().get(topic):return True
                time.sleep(.2)
            return False
        pubs=graph();names=set(rosnode.get_node_names())
        if not pubs.get('/mavros/state'):
            if '/mavros' not in names:
                spawn('mavros',['python3','/home/abot/window_pass_session_20261013/scripts/session.py','_mavros'])
                await_mavros_startup(rospy,State,timeout=15)
            elif not await_publisher('/mavros/state',15):raise ValueError('MAVROS startup missing; inspect ground_logs/mavros.log')
        ground();pubs=graph();names=set(rosnode.get_node_names())
        if not pubs.get('/window_pass_lio/Odometry'):
            if '/window_pass_lio/laserMapping' not in names:
                if len(pubs.get('/livox/lidar',[]))!=1:raise ValueError('unique existing lidar required before LIO recovery')
                spawn('lio',['python3','/home/abot/window_pass_session_20261013/scripts/session.py','_lio'])
            if not await_publisher('/window_pass_lio/Odometry',15):raise ValueError('LIO startup missing; inspect ground_logs/lio.log')
        pubs=graph()  # LIO startup may have changed other publishers as well.
        if len(pubs.get('/window_pass_lio/Odometry',[]))!=1:raise ValueError('LIO source ambiguous')
        if a.upgrade_pose and pubs.get('/window_pass_competition/status'):
            ground()
            nodes=pubs['/window_pass_competition/status']
            if len(nodes)!=1 or pubs.get('/mavros/vision_pose/pose')!=nodes:raise ValueError('pose source ambiguous')
            data=json.loads(rospy.wait_for_message('/window_pass_competition/status',String,timeout=3).data)
            desired=json.loads((BUNDLE/'autonomous_window_test/config/full_chain_single_target.json').read_text())['control']
            limits={k:desired[k] for k in ('max_pose_step_m','max_pose_speed_mps')}
            if data.get('raw_lio_motion_limits')!=limits:
                node=nodes[0]
                if data.get('raw_lio_guard_scope')!='RouteContinuity' or not node.startswith('/window_pass_competition_pose_'):raise ValueError('unexpected pose source; no replacement')
                uri=master.lookupNode(node);code,text,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
                command=Path('/proc/%s/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
                if not any('/home/abot/'+name+'/autonomous_window_test/scripts/start_full_chain_pose.py' in command for name in ('full_chain_20261024_v1_1','full_chain_20261024_v2')):raise ValueError('unrecognized previous bridge; no replacement')
                rosnode.kill_nodes([node]);deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    pubs=graph()
                    if not pubs.get('/window_pass_competition/status') and not pubs.get('/mavros/vision_pose/pose'):break
                    time.sleep(.1)
                else:raise ValueError('old pose bridge did not stop')
                print('Previous recognized pose bridge stopped on disarmed ground; starting scoped speed guard.',flush=True)
        pubs=graph()
        def read_pose_status():
            return rospy.wait_for_message('/window_pass_competition/status',String,timeout=3)
        def bridge_process(node):
            uri=master.lookupNode(node);_,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
            return pid,Path('/proc/%s/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if replace_latched_pose_bridge(pubs,read_pose_status,ground,graph,bridge_process,rosnode.kill_nodes,lambda pid:Path('/proc/%s'%pid).exists()):
            pubs=graph()
        if not pubs.get('/window_pass_competition/status') and not pubs.get('/mavros/vision_pose/pose'):
            start_ground_pose(ground,spawn,graph,
                lambda state:snapshot_details(rospy,[('/mavros/state',State)],[state],2))
        else:
            require_existing_pose_bridge(pubs,
                lambda:rospy.wait_for_message('/window_pass_competition/status',String,timeout=3))
        ground();pubs=graph()
        usb=pubs.get('/usb_cam/image_raw',[]);visual=pubs.get('/circle_pixel_preview/status',[]);laser=pubs.get('/hover_target_laser/status',[])
        if not visual and not laser:
            if len(usb)>1:raise ValueError('USB source ambiguous')
            spawn('target_support',['roslaunch','full_chain_vision_20261025_v6_1','hover_target_support.launch','enable_laser:=true','operator_pwm_verified:=true','scope:=window_pass_two_targets','enable_landing_illumination:=true',
                  'start_camera:='+('false' if usb else 'true'),'video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'])
        else:
            if len(visual)!=1 or len(laser)!=1:raise ValueError('target support partial/ambiguous; inspect group instead of duplicating nodes')
            status=json.loads(rospy.wait_for_message('/hover_target_laser/status',String,timeout=3).data)
            if status.get('scope')!='window_pass_two_targets' or status.get('exposure_used') is not False:raise ValueError('target support wrong scope or spent episode; restore only on ground')
        ground();pubs=graph()
        if not pubs.get('/camera/rgb/image_raw') and not pubs.get('/window_live/status'):
            spawn('rgbd',['roslaunch',str(BUNDLE/'autonomous_window_test/launch/perception.launch')])
        elif len(pubs.get('/camera/rgb/image_raw',[]))!=1 or len(pubs.get('/window_live/status',[]))!=1:
            raise ValueError('RGBD partially running; inspect missing camera/detector before adding nodes')
        print('Restoration dispatched. Wait for localization and sensors; rerun status, then bind. No flight executed.',flush=True)
    # Read identity, rates and source paths. Never subscribe to control input.
    pubs=graph();state=rospy.wait_for_message('/mavros/state',State,timeout=4)
    data={'boot_id':boot,'run_id':rospy.get_param('/window_pass_session_launch/run_id',''),'fcu':{'mode':state.mode,'armed':state.armed,'connected':state.connected,'system_status':state.system_status},'publishers':pubs,'source_processes':{}}
    import socket
    socket.setdefaulttimeout(2)
    wanted={'/mavros','/window_pass_lio/laserMapping','/window_live_vision','/circle_usb_camera','/circle_pixel_preview','/hover_target_laser'}
    for topic in ('/window_pass_competition/status','/mavros/vision_pose/pose','/camera/rgb/image_raw'):
        wanted.update(pubs.get(topic,[]))
    for node in sorted(wanted):
        try:
            uri=master.lookupNode(node);code,text,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
            data['source_processes'][node]={'uri':uri,'pid':pid,'cmdline':Path('/proc/%s/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode(errors='replace')}
        except Exception as exc:data['source_processes'][node]={'error':str(exc)}
    from nav_msgs.msg import Odometry
    stamps=[]
    def got(m):stamps.append(m.header.stamp.to_sec())
    sub=rospy.Subscriber('/window_pass_lio/Odometry',Odometry,got,queue_size=100)
    time.sleep(3);sub.unregister()
    rate=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else None
    data['lio_sample_rate_hz']=rate
    if a.restore and (rate is None or not 8<=rate<=12):raise ValueError('expected 10Hz LIO, actual='+str(rate))
    folder=BUNDLE/'ground_logs';folder.mkdir(exist_ok=True)
    (folder/('snapshot_'+boot+'.json')).write_text(json.dumps(data,indent=2))
    print(json.dumps(data,indent=2));return 0

if __name__=='__main__':sys.exit(main())
