#!/usr/bin/env python3
"""Rebuild the local robot ROS session, only after fresh disarmed-ground proof."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse
import xmlrpc.client
from competition import ROOT,FLIGHT,StartupProbeTransport
SIGNAL_KILL=getattr(signal,'SIGKILL',9)


def processes():
    rows={}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            argv=[v.decode(errors='replace') for v in (path/'cmdline').read_bytes().split(bytes([0])) if v]
            rows[int(path.name)]={'pid':int(path.name),'ppid':int(fields[1]),'start':fields[19],'argv':argv}
        except FileNotFoundError:continue
    return rows


def is_ros_process(row):
    args=row['argv']
    return any(Path(v).name in ('roslaunch','roscore','rosmaster') for v in args[:3]) or bool(args and
        (args[0].startswith('/opt/ros/noetic/lib/') or Path(args[0]).name in ('fastlio_mapping','laserMapping')))


def robot_processes(rows,node_pids=()):
    selected={pid for pid,row in rows.items() if is_ros_process(row)}
    for pid in node_pids:
        row=rows.get(pid)
        if row and any(a.startswith(('/home/abot/','/opt/ros/')) for a in row['argv'][:2]):selected.add(pid)
    while True:
        children={pid for pid,row in rows.items() if row['ppid'] in selected}
        if children<=selected:break
        selected|=children
    if os.getpid() in selected:raise RuntimeError('refuse: reset helper included in stop inventory')
    return {pid:rows[pid] for pid in selected}


def same_process(row):
    current=processes().get(row['pid'])
    return bool(current and current['start']==row['start'] and current['argv']==row['argv'])


def signal_process(row,number):
    if same_process(row):
        try:os.kill(row['pid'],number)
        except ProcessLookupError:pass


def stop_inventory(rows,phases=((signal.SIGINT,10),(signal.SIGTERM,5),(SIGNAL_KILL,3))):
    for number,budget in phases:
        alive=[row for row in rows.values() if same_process(row)]
        if not alive:return
        for row in alive:signal_process(row,number)
        deadline=time.monotonic()+budget
        while time.monotonic()<deadline:
            if not any(same_process(row) for row in rows.values()):return
            time.sleep(.2)
    remaining=[row for row in rows.values() if same_process(row)]
    if remaining:raise RuntimeError('robot processes did not exit: '+json.dumps(remaining))


def master():
    return xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11311'),transport=StartupProbeTransport())


def master_online():
    try:return master().getPid('/competition_prepare_reset')[0]==1
    except OSError:return False


def mavros_bootstrap_decision(mavros_alive,rows):
    if mavros_alive:return 'reuse_registered_mavros'
    if any(r['argv'] and Path(r['argv'][0]).name=='mavros_node' for r in rows.values()):
        return 'verify_existing_mavros_telemetry'
    return 'start_minimal_mavros'


def spawn_bootstrap(args):
    with (ROOT/'prepare_reset_bootstrap.log').open('ab') as log:
        child=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    print('PREPARE_RESET_BOOTSTRAP '+json.dumps({'pid':child.pid,'argv':args}),flush=True)


def ground_snapshot():
    sys.path.insert(0,str(FLIGHT/'autonomous_window_test'))
    import rospy
    from mavros_msgs.msg import State,ExtendedState,RCIn
    from ground_snapshot import latest_messages,ground_rejection,snapshot_details,await_mavros_startup
    rospy.init_node('competition_prepare_reset_ground_proof',anonymous=True,disable_signals=True)
    await_mavros_startup(rospy,State,timeout=15)
    topics=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]
    values=latest_messages(rospy,topics,timeout=15,max_age=2,reject=ground_rejection)
    s,e,r=values;now=rospy.Time.now().to_sec()
    if (any((v._connection_header or {}).get('callerid')!='/mavros' or not -.02<=now-v.header.stamp.to_sec()<2 for v in values)
        or not s.connected or s.armed or s.system_status!=3 or e.landed_state!=1
        or len(r.channels)<7 or any(not 900<=r.channels[k]<=1200 for k in (5,6))):
        raise ValueError('PREPARE_RESET_REFUSED: fresh connected disarmed landed CH6/CH7 off required; '+snapshot_details(rospy,topics,values,2))
    print(json.dumps({'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'connected':s.connected,'armed':s.armed,'landed_state':e.landed_state,'mode':s.mode,
        'ch6':r.channels[5],'ch7':r.channels[6],'details':snapshot_details(rospy,topics,values,2)}),flush=True)


def registered_local_pids():
    api=master();caller='/competition_prepare_reset'
    code,msg,state=api.getSystemState(caller)
    if code!=1:raise RuntimeError(msg)
    hosts={'localhost','127.0.0.1',os.environ.get('ROS_IP','')}
    found=[]
    for name in {name for group in state for topic,names in group for name in names}:
        code,msg,uri=api.lookupNode(caller,name)
        if code!=1 or urlparse(uri).hostname not in hosts:continue
        try:
            with xmlrpc.client.ServerProxy(uri,transport=StartupProbeTransport()) as node:
                code,msg,pid=node.getPid(caller)
            if code==1 and type(pid) is int:found.append(pid)
        except OSError:continue
    return found


def sample_ground():
    result=subprocess.run([sys.executable,'-B',str(Path(__file__).resolve()),'--sample-ground'],
                          stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=40)
    print(result.stdout,flush=True)
    if result.returncode:raise RuntimeError('PREPARE_RESET_REFUSED: no fresh ground proof; no old processes stopped')
    return result.stdout


def reset_proven_session():
    proof=sample_ground()
    selected=robot_processes(processes(),registered_local_pids())
    audit=ROOT/'prepare_reset_audit.json'
    audit.write_text(json.dumps({'phase':'audited_before_stop','ground_proof':proof,'processes':list(selected.values())},indent=2)+'\n')
    sample_ground()
    stop_inventory(selected)
    remaining=robot_processes(processes())
    if remaining:raise RuntimeError('robot processes appeared during reset; no replacement chain started: '+json.dumps(list(remaining.values())))
    audit.write_text(json.dumps({'phase':'stopped_verified','ground_proof':proof,'stopped_processes':list(selected.values())},indent=2)+'\n')
    print('PREPARE_RESET=STOPPED_VERIFIED; rebuilding the robot ROS session next',flush=True)


def main():
    if sys.argv[1:]==['--sample-ground']:
        ground_snapshot();return
    if sys.argv[1:]:raise ValueError('unsupported reset arguments')
    rows=processes()
    if not master_online():
        if any(any(Path(a).name in ('rosmaster','roscore') for a in r['argv'][:3]) for r in rows.values()):
            raise RuntimeError('ROS master process is unresponsive; cannot prove safe ground for reset')
        spawn_bootstrap(['roscore','-p','11311'])
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and not master_online():time.sleep(.2)
        if not master_online():raise RuntimeError('minimal ROS master bootstrap failed; no reset performed')
    mavros_alive=False
    api=master();code,msg,uri=api.lookupNode('/competition_prepare_reset','/mavros')
    if code==1:
        try:
            with xmlrpc.client.ServerProxy(uri,transport=StartupProbeTransport()) as node:
                mavros_alive=node.getPid('/competition_prepare_reset')[0]==1
        except OSError:pass
    mavros_decision=mavros_bootstrap_decision(mavros_alive,processes())
    if mavros_decision=='verify_existing_mavros_telemetry':
        print('PREPARE_RESET_MAVROS=UNREACHABLE; requiring fresh ground telemetry before reset',flush=True)
    elif mavros_decision=='start_minimal_mavros':
        spawn_bootstrap(['python3','/home/abot/window_pass_session_20261013/scripts/session.py','_mavros'])
    # The proof runs in a short-lived subscriber child. The reset supervisor
    # never becomes a ROS node, so restarting Master cannot invalidate it.
    reset_proven_session()


if __name__=='__main__':main()
