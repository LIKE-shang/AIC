#!/usr/bin/env python3
"""Reboot preparation for the previously authorized takeoff-only trial."""
import argparse,copy,json,os,subprocess,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from auto_window import commissioning_deferrals


def manual():
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState,RCIn
    from mavros_msgs.srv import SetMode
    from std_msgs.msg import String
    rospy.init_node('reboot_ground_mode_prepare',anonymous=True)
    e=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=15)
    rc=rospy.wait_for_message('/mavros/rc/in',RCIn,timeout=5)
    s=rospy.wait_for_message('/mavros/state',State,timeout=5)
    now=rospy.Time.now().to_sec()
    if not(s.connected and not s.armed and s.system_status==3 and e.landed_state==1 and len(rc.channels)>=7 and 900<=rc.channels[6]<=1200):
        raise RuntimeError('Require landed, disarmed, system_status=3, A off; do not change switches blindly')
    if not all(0<=now-m.header.stamp.to_sec()<2 and (getattr(m,'_connection_header',{}) or {}).get('callerid')=='/mavros' for m in (s,e,rc)):
        raise RuntimeError('fresh MAVROS telemetry required')
    pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    if any(pubs.get(t) for t in ('/mavros/setpoint_raw/local','/window_pass_align/status','/window_pass_hold/status','/window_pass_test/status')):
        raise RuntimeError('existing controller; no mode request sent')
    if s.mode!='MANUAL':
        rospy.wait_for_service('/mavros/set_mode',timeout=3)
        response=rospy.ServiceProxy('/mavros/set_mode',SetMode)(base_mode=0,custom_mode='MANUAL')
        if not response.mode_sent:raise RuntimeError('MANUAL request rejected')
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        s=rospy.wait_for_message('/mavros/state',State,timeout=2)
        if s.armed or s.system_status!=3:raise RuntimeError('ground state changed')
        if s.connected and s.mode=='MANUAL' and 0<=rospy.Time.now().to_sec()-s.header.stamp.to_sec()<1.5:break
        time.sleep(.2)
    else:raise RuntimeError('MANUAL not observed')
    print('MANUAL_CONFIRMED_DISARMED; waiting for continuous localization',flush=True)
    deadline=time.monotonic()+25
    while time.monotonic()<deadline:
        try:m=rospy.wait_for_message('/window_pass_competition/status',String,timeout=3)
        except rospy.ROSException:continue
        d=json.loads(m.data)
        if (d.get('real_output_state')=='FORWARDING_CONTINUOUS_POSE' and not d.get('real_fault')
            and d.get('horizontal_flags_valid') is True and (d.get('real_output_rate_hz') or 0)>=8):
            print('BASE_READY: continuous localization and horizontal estimate available',flush=True)
            return
    raise RuntimeError('localization not ready; keep base terminal and report its error')


def template_for_binding(cfg):
    c=copy.deepcopy(cfg)
    commissioning_deferrals(c,True)
    if c['control']['enable_traverse'] or c['control']['pass_stage_approved']:
        raise ValueError('takeoff-only profile required')
    c['control']['enable_motion']=False
    c['control']['geofence_min_m']=[];c['control']['geofence_max_m']=[]
    c['site_identity']=None
    c.pop('site_evidence',None)
    return c


def trial(args):
    if not args.same_start_confirmed:raise ValueError('--same-start-confirmed required: same physical start and heading as surveyed')
    cfg=template_for_binding(json.loads(args.template.read_text()))
    directory=Path(tempfile.mkdtemp(prefix='takeoff_trial_',dir='/home/abot'))
    base=directory/'binding_template.json';out=directory/'config.json'
    base.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+chr(10))
    subprocess.run([sys.executable,str(ROOT/'scripts/bind_takeoff_site.py'),'--config',str(base),
                    '--output',str(out),'--same-start-confirmed'],check=True)
    bound=json.loads(out.read_text());commissioning_deferrals(bound,True)
    if bound['control']['enable_motion'] is not False:raise RuntimeError('binding unexpectedly enabled motion')
    bound['control']['enable_motion']=True
    bound['commissioning_authorization']['reboot_rebound_from']=str(args.template)
    # Existing narrowly scoped authorization is reused; no verified flags added.
    out.write_text(json.dumps(bound,ensure_ascii=False,indent=2)+chr(10))
    command=['bash',str(ROOT/'run_takeoff.bash'),'--config',str(out),'--commissioning-takeoff']
    result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (directory/'preflight.txt').write_text(result.stdout)
    print(result.stdout,flush=True)
    print('TRIAL_DIRECTORY='+str(directory),flush=True)
    if result.returncode!=0:raise SystemExit(result.returncode)
    pointer=Path('/home/abot/autonomous_window_current_trial.path')
    tmp=directory/'current_trial.path';tmp.write_text(str(out)+chr(10))
    os.replace(str(tmp),str(pointer))
    print('TRIAL_READY: current_takeoff.bash --execute is the separate flight command',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['manual','trial'])
    p.add_argument('--same-start-confirmed',action='store_true')
    p.add_argument('--template',type=Path,default=Path('/home/abot/takeoff_test_aOaGrX/config.commissioning_20261019.json'))
    args=p.parse_args()
    if args.action=='manual':manual()
    else:trial(args)

if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit('REBOOT_PREPARE_STOP: '+str(exc))
