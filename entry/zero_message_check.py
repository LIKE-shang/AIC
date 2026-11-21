#!/usr/bin/env python3
"""Ground transport handshake with bag recording; advertises but publishes ZERO flight targets."""
import importlib.util,json,os,signal,subprocess,time
from pathlib import Path
import rospy,rosgraph
from mavros_msgs.msg import State,ExtendedState,RCIn,PositionTarget
B=Path(__file__).resolve().parent.parent/'flight'
import argparse
a=argparse.ArgumentParser();a.add_argument('--trial',type=Path,required=True);T=a.parse_args().trial.resolve()
assert T.parent==Path('/home/abot') and T.name.startswith('full_chain_trial_')
import sys
sys.path.insert(0,str(B/'autonomous_window_test'))
from takeoff import check_takeoff_graph
from ground_snapshot import latest_messages,ground_rejection,snapshot_details
rospy.init_node('full_chain_transport_precheck',disable_signals=True)
master=rosgraph.Master(rospy.get_name())
def ground():
 topics=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]
 s,e,r=latest_messages(rospy,topics,timeout=3,max_age=2,reject=lambda t,m:ground_rejection(t,m,allowed_modes=('MANUAL','ALTCTL','POSCTL')))
 now=rospy.Time.now().to_sec()
 assert all((m._connection_header or {}).get('callerid')=='/mavros' and -.02<=now-m.header.stamp.to_sec()<2 for m in (s,e,r)),snapshot_details(rospy,topics,(s,e,r),2)
 assert s.connected and not s.armed and s.system_status==3 and s.mode in ('MANUAL','ALTCTL','POSCTL') and e.landed_state==1,snapshot_details(rospy,topics,(s,e,r),2)
 assert len(r.channels)>=7 and all(900<=r.channels[k]<=1200 for k in (5,6)),snapshot_details(rospy,topics,(s,e,r),2)
 return s
before=ground()
assert not dict(master.getSystemState()[0]).get('/mavros/setpoint_raw/local')
spec=importlib.util.spec_from_file_location('record_policy',B/'autonomous_window_test/scripts/run_recorded_hover_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
assert '/mavros/setpoint_raw/local' not in module.TOPICS
folder=T/'ground_checks';folder.mkdir(exist_ok=True)
stream=(folder/'recorder_handshake.log').open('ab')
rec=subprocess.Popen(['rosbag','record','-O',str(folder/'recorder_handshake.bag'),'--duration=5']+module.TOPICS,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
pub=None
try:
 time.sleep(.7);assert rec.poll() is None
 pub=rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1,latch=False)
 deadline=time.monotonic()+3
 while time.monotonic()<deadline and pub.get_num_connections()!=1:time.sleep(.05)
 graph=master.getSystemState();ok,reason=check_takeoff_graph(graph,rospy.get_name(),json.loads((T/'config.json').read_text())['control'],True)
 assert ok and pub.get_num_connections()==1,(reason,pub.get_num_connections())
 assert dict(graph[1]).get('/mavros/setpoint_raw/local')==['/mavros']
 after=ground();assert after.mode==before.mode
 report={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'run_id':rospy.get_param('/window_pass_session_launch/run_id'),
  'trial':str(T),'graph_ok':ok,'graph_reason':reason,'output_connections':pub.get_num_connections(),
  'output_subscribers':dict(graph[1]).get('/mavros/setpoint_raw/local'),'setpoints_published':0,'arming_calls':0,'mode_calls':0,
  'laser_on_requests':0,'bag_running_during_check':rec.poll() is None,'recorder_topics':module.TOPICS,'mode_before':before.mode,'mode_after':after.mode,'armed_after':after.armed}
finally:
 if pub:pub.unregister()
 if rec.poll() is None:
  os.killpg(rec.pid,signal.SIGINT)
  try:rec.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(rec.pid,signal.SIGTERM);rec.wait(timeout=3)
 stream.close()
assert not dict(master.getSystemState()[0]).get('/mavros/setpoint_raw/local')
report['output_publisher_removed']=True
(folder/'recorder_handshake_verified.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
