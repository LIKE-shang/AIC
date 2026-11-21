#!/usr/bin/env python3
"""Switch to the scoped two-target OFF bridge on trusted unarmed ground only."""
import argparse,json,os,signal,sys,time,xmlrpc.client
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from two_targets import selection
from ground_snapshot import latest_messages,ground_rejection,snapshot_details

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',nargs='+',type=int,required=True);p.add_argument('--visual-calibration',action='store_true');a=p.parse_args();ids=selection(a.targets)
 import rospy,rosgraph
 from mavros_msgs.msg import State,ExtendedState,RCIn
 from std_msgs.msg import String
 rospy.init_node('prepare_selected_two_target_support_ground',anonymous=True,disable_signals=True);master=rosgraph.Master(rospy.get_name())
 def guard():
  pubs=dict(master.getSystemState()[0])
  if pubs.get('/mavros/setpoint_raw/local'):raise ValueError('flight supervisor present; no support switch')
  topics=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]
  rows=latest_messages(rospy,topics,timeout=3,max_age=2,reject=ground_rejection)
  s,e,r=rows;now=rospy.Time.now().to_sec()
  if any((v._connection_header or {}).get('callerid')!='/mavros' or not -.02<=now-v.header.stamp.to_sec()<2 for v in rows):raise ValueError('fresh trusted MAVROS required; '+snapshot_details(rospy,topics,rows,2))
  if not s.connected or s.armed or s.system_status!=3 or e.landed_state!=1 or len(r.channels)<7 or not all(900<=r.channels[k]<=1200 for k in (5,6)):raise ValueError('disarmed landed CH6 released CH7 off required; '+snapshot_details(rospy,topics,rows,2))
  return pubs
 pubs=guard();nodes=pubs.get('/hover_target_laser/status',[])
 if not nodes:print('No actuator bridge; ground restore will start OFF bridge.');return 0
 if nodes!=['/hover_target_laser']:raise ValueError('unexpected laser publisher')
 data=json.loads(rospy.wait_for_message('/hover_target_laser/status',String,timeout=3).data)
 if data.get('requested_on') is not False:raise ValueError('laser requested ON; no support migration')
 uri=master.lookupNode('/hover_target_laser');_,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
 cmd=Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
 known=('/home/abot/full_chain_20261024_v4_1/vision_ws/','/home/abot/full_chain_20261024_v5/vision_ws/','/home/abot/full_chain_20261025_v5_1/vision_ws/','/home/abot/full_chain_20261025_v5_2/vision_ws/','/home/abot/full_chain_20261025_v5_3/vision_ws/','/home/abot/full_chain_20261025_v6/vision_ws/','/home/abot/full_chain_20261025_v6_1/vision_ws/','/home/abot/full_chain_20261030_lidar_recovery/vision_ws/')
 if not any(x in cmd for x in known) or 'full_chain_target_laser_' not in cmd:raise ValueError('unrecognized bridge binary')
 if (not (a.visual_calibration or os.environ.get('ORIGINAL_CAR_VISUAL_CALIBRATION','off')!='off') or data.get('visual_calibration_authorization')=='observed_spot_calibration_20261026') and str(ROOT.parent/'vision_ws') in cmd and (not (ROOT/'config/full_chain_second_window_return.json').exists() or (data.get('landing_illumination_authorized') is True and data.get('landing_illumination_authorization')=='supervised_second_window_return_auto_landing_20261026')) and data.get('scope')=='window_pass_two_targets' and data.get('exposure_count')==0 and data.get('fault_latched') is False and data.get('startup_off_acked') is True:
  print('Fresh current-version two-target bridge retained; selected order='+str(ids));return 0
 parent=int(next(row.split(':',1)[1] for row in Path('/proc/%d/status'%pid).read_text().splitlines() if row.startswith('PPid:')))
 pgid=os.getpgid(parent);launch=Path('/proc/%d/cmdline'%parent).read_bytes().replace(b'\0',b' ').decode()
 if pgid!=parent or 'roslaunch' not in launch or 'hover_target_support.launch' not in launch or not any(name in launch for name in ('full_chain_vision_20261024_v4_1','full_chain_vision_20261024_v5','full_chain_vision_20261025_v5_1','full_chain_vision_20261025_v5_2','full_chain_vision_20261025_v5_3','full_chain_vision_20261025_v6','full_chain_vision_20261025_v6_1')):raise ValueError('unrecognized target support group')
 guard();os.killpg(pgid,signal.SIGINT)
 deadline=time.monotonic()+18
 while time.monotonic()<deadline:
  pubs=dict(master.getSystemState()[0])
  if not any(pubs.get(t) for t in ('/hover_target_laser/status','/circle_pixel_preview/status','/usb_cam/image_raw')):break
  time.sleep(.2)
 else:raise ValueError('old support did not stop; inspect before restoring')
 print(json.dumps({'stopped_ground_support_group':pgid,'selected_target_order':ids,'next_step':'ground.bash --restore starts scoped OFF bridge; then select_two.bash --targets ... binds','flight_executed':False,'on_requests':0}));return 0
if __name__=='__main__':sys.exit(main())
