#!/usr/bin/env python3
"""Ground-only authorized horizontal speed cap change. Never arms or sends setpoints."""
import json,math,time
from pathlib import Path

def main():
 import rospy,rosgraph
 from mavros_msgs.msg import State,ExtendedState,RCIn,ParamValue
 from mavros_msgs.srv import ParamGet,ParamSet
 rospy.init_node('full_chain_prepare_speed_ground',anonymous=True,disable_signals=True)
 master=rosgraph.Master(rospy.get_name())
 def ground():
  pubs=dict(master.getSystemState()[0])
  if pubs.get('/mavros/setpoint_raw/local'):raise ValueError('flight controller present')
  rows=[rospy.wait_for_message(t,k,timeout=4) for t,k in [('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn)]]
  s,e,r=rows;now=rospy.Time.now().to_sec()
  if any((m._connection_header or {}).get('callerid')!='/mavros' or not -.02<=now-m.header.stamp.to_sec()<2 for m in rows):raise ValueError('fresh trusted MAVROS required')
  if not s.connected or s.armed or s.system_status!=3 or e.landed_state!=1 or s.mode not in ('MANUAL','POSCTL') or len(r.channels)<7 or not all(900<=r.channels[k]<=1200 for k in (5,6)):raise ValueError('disarmed ground, kill released and CH7 off required')
 get=rospy.ServiceProxy('/mavros/param/get',ParamGet);put=rospy.ServiceProxy('/mavros/param/set',ParamSet)
 expected={'MPC_XY_VEL_MAX':1.5,'MPC_ACC_HOR':1.,'MPC_ACC_HOR_MAX':1.,'MPC_JERK_AUTO':4.,'MPC_TILTMAX_AIR':15.}
 def read():
  values={}
  for name in expected:
   r=get(param_id=name)
   if not r.success or r.value.integer!=0 or not math.isfinite(r.value.real):raise ValueError('parameter read failed:'+name)
   values[name]=r.value.real
  return values
 ground();before=read()
 for name,value in expected.items():
  if name!='MPC_XY_VEL_MAX' and not math.isclose(before[name],value,abs_tol=1e-6):raise ValueError('unexpected existing parameter:'+name)
 speed=before['MPC_XY_VEL_MAX']
 if not any(math.isclose(speed,x,abs_tol=1e-6) for x in (.7,1.5)):raise ValueError('unexpected horizontal speed baseline')
 root=Path(__file__).resolve().parents[2];folder=root/'ground_logs';folder.mkdir(exist_ok=True)
 backup=folder/('speed_backup_'+str(time.monotonic_ns())+'.json')
 backup.write_text(json.dumps({'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'before':before,'authorized_max_speed_mps':1.5,'flight_executed':False},indent=2))
 changed=False
 if not math.isclose(speed,1.5,abs_tol=1e-6):
  ground();r=put(param_id='MPC_XY_VEL_MAX',value=ParamValue(integer=0,real=1.5))
  if not r.success:raise ValueError('speed cap change rejected; inspect readback')
  changed=True
 after=read();ground()
 if any(not math.isclose(after[k],v,abs_tol=1e-6) for k,v in expected.items()):raise ValueError('speed readback mismatched')
 report={'before':before,'after':after,'changed_parameters':['MPC_XY_VEL_MAX'] if changed else [],'backup':str(backup),'flight_executed':False,'laser_on':False}
 (folder/'speed_prepared.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
 return 0
if __name__=='__main__':raise SystemExit(main())
