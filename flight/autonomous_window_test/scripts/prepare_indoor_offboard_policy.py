"""Prepare indoor offboard-loss policy. Read-only unless --apply is explicit."""
import argparse,json,time,math
from pathlib import Path

def blockers(state,extended,rc,estimator,pubs,parameters,now):
    errors=[]
    for name,m in [('state',state),('landed',extended),('rc',rc),('estimator',estimator)]:
        if (getattr(m,'_connection_header',{}) or {}).get('callerid')!='/mavros':errors.append('source:'+name)
        if not 0<=now-m.header.stamp.to_sec()<=1.5:errors.append('stale:'+name)
    if not state.connected or state.armed or state.system_status!=3 or state.mode not in ('MANUAL','POSCTL'):errors.append('healthy_disarmed_ground_mode_required')
    if extended.landed_state!=1:errors.append('landed_required')
    if len(rc.channels)<7 or not 900<=rc.channels[6]<=1200:errors.append('A_off_required')
    if not estimator.pos_horiz_rel_status_flag or not estimator.pos_vert_abs_status_flag:errors.append('valid_local_position_required')
    if pubs.get('/mavros/setpoint_raw/local') or pubs.get('/mavlink/to'):errors.append('existing_controller')
    if pubs.get('/mavros/vision_pose/pose')!=['/window_pass_competition_pose_session_20261013']:errors.append('verified_pose_bridge_required')
    for name,wanted in [('COM_OBL_RC_ACT',4),('COM_OBL_ACT',0),('NAV_RCL_ACT',3),('COM_RC_OVERRIDE',3)]:
        if parameters.get(name)!=[wanted,0.]:errors.append('unexpected_parameter:'+name)
    return errors

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--apply',action='store_true');a=p.parse_args()
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState,RCIn,EstimatorStatus,ParamValue
    from mavros_msgs.srv import ParamGet,ParamSet
    rospy.init_node('indoor_offboard_loss_ground_prepare',anonymous=True)
    kinds=[('/mavros/state',State),('/mavros/extended_state',ExtendedState),('/mavros/rc/in',RCIn),('/mavros/estimator_status',EstimatorStatus)]
    latest={}
    subs=[rospy.Subscriber(t,k,lambda m,t=t:latest.__setitem__(t,m)) for t,k in kinds]
    end=time.monotonic()+4
    while time.monotonic()<end:
        if all(t in latest for t,k in kinds):break
        time.sleep(.05)
    if not all(t in latest for t,k in kinds):raise RuntimeError('required_telemetry_missing')
    getter=rospy.ServiceProxy('/mavros/param/get',ParamGet);parameters={}
    for name in ['COM_OBL_RC_ACT','COM_OBL_ACT','NAV_RCL_ACT','COM_RC_OVERRIDE']:
        r=getter(param_id=name);parameters[name]=[r.value.integer,r.value.real] if r.success else None
    master=rosgraph.Master(rospy.get_name())
    def check():return blockers(*(latest[t] for t,k in kinds),dict(master.getSystemState()[0]),parameters,rospy.Time.now().to_sec())
    errors=check();report={'ready':not errors,'blockers':errors,'proposed_change':{'COM_OBL_RC_ACT':{'from':4,'to':0}},'before':parameters,'arm_requests_sent':0,'applied':False}
    if not a.apply:print(json.dumps(report,indent=2));return 0 if not errors else 2
    if errors:raise RuntimeError('GROUND_CHECKS_BLOCKED:'+str(errors))
    template=Path('/home/abot/takeoff_test_aOaGrX/config.commissioning_20261019.json')
    raw=template.read_bytes();cfg=json.loads(raw)
    if cfg['experiment']!='takeoff_land' or cfg['expected_fcu_parameters']['COM_OBL_RC_ACT']!=[4,0.]:raise RuntimeError('unexpected_template')
    folder=Path('/home/abot')/('indoor_offboard_policy_'+str(time.time_ns()));folder.mkdir()
    (folder/'template.before.json').write_bytes(raw);(folder/'parameters.before.json').write_text(json.dumps(parameters,indent=2))
    # Invalidate the old trial before changing the expected flight-control policy.
    pointer=Path('/home/abot/autonomous_window_current_trial.path')
    if pointer.exists():
        profile=Path(pointer.read_text().strip());old=profile.read_bytes();trial=json.loads(old)
        (folder/'trial.before.json').write_bytes(old);trial['control']['enable_motion']=False
        profile.write_text(json.dumps(trial,indent=2)+chr(10))
    errors=check()
    if errors:raise RuntimeError('GROUND_STATE_CHANGED:'+str(errors))
    setter=rospy.ServiceProxy('/mavros/param/set',ParamSet)
    result=setter(param_id='COM_OBL_RC_ACT',value=ParamValue(integer=0,real=0.))
    if not result.success:raise RuntimeError('parameter_write_rejected; backups='+str(folder))
    readback=getter(param_id='COM_OBL_RC_ACT')
    if not readback.success or readback.value.integer!=0 or not math.isclose(readback.value.real,0.):raise RuntimeError('readback_not_confirmed; backups='+str(folder))
    cfg['expected_fcu_parameters']['COM_OBL_RC_ACT']=[0,0.]
    cfg['mission']['pilot_landing_enabled']=True
    cfg['control']['enable_motion']=False
    template.write_text(json.dumps(cfg,indent=2)+chr(10))
    report.update(applied=True,readback=[readback.value.integer,readback.value.real],backups=str(folder),next_step='rebind_and_preflight_no_automatic_flight')
    (folder/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
