"""Synthetic two distinct openings and automatic touchdown; no hardware actions."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from test_two_targets import PairSim
from test_session_window import IDENTITY
from test_airborne_geometry import DynamicSim
from mission import Mission
from second_window_return import DEFAULTS,settings
ROOT=Path(__file__).resolve().parents[1]
class ReturnSim(PairSim):
 def __init__(self,ids=(1,3)):
  super().__init__(list(ids));old=self.m
  chain=json.loads((ROOT/'config/full_chain_two_targets.json').read_text())['full_chain'];chain['selected_target_ids']=sorted(ids);chain['second_window_return']=copy.deepcopy(DEFAULTS)
  c={k:copy.deepcopy(v) for k,v in old.core.c.items() if not k.startswith('_session_')}
  self.m=Mission(c,dict(old.o,second_window_return_enabled=True,max_mission_time_s=240.),True,'straight_pass',session_window_policy=old.session_policy,session_identity=IDENTITY,hover_target_policy=old.hover_target.p,full_chain_policy=chain)
  self.f.ctrl=self.m.core;self.f.cfg['window_center_local_m']=[1.65,0.,1.05];self.follow_yaw=True;self.lower=True
  for _ in range(30):self.update()
 def update(self,follow=True):
  if self.out and self.out.get('yaw') is not None and self.follow_yaw:
   h=self.out['yaw'];self.f.q=(0.,0.,math.sin(h/2),math.cos(h/2))
  if getattr(self.m.return_journey,'leg',1)==2 and self.lower:self.f.cfg['window_center_local_m']=[1.65,-2.65,1.05]
  if self.m.phase in ('RETURN_PREP','YAW_TURN','RETURN_HOME','LANDING_LIGHT_ON','LANDING_HANDOVER'):DynamicSim.update(self,follow)
  else:super().update(follow)
 def inputs(self):
  i=super().inputs();j=self.m.return_journey
  if j.request_id:
   on=j.request_on and self.allow_on
   if not j.request_on and not self.allow_off:on=True
   i=replace(i,hover_laser=(i.wall,dict(scope='window_pass_two_targets',fault_latched=False,request_id=j.request_id,requested_on=on,state='ON_ACKNOWLEDGED' if on else 'OFF_ACKNOWLEDGED')))
  return i
 def step(self,follow=True,respond=True):
  out=super().step(follow,respond)
  if respond and out['action']=='POSCTL':self.mode='POSCTL'
  return out
class ReturnTests(unittest.TestCase):
 def test_return_timeout_pauses_during_second_window_search_and_pass(self):
  s=ReturnSim();s.start()
  for _ in range(4500):
   s.step()
   if s.m.phase=='SEARCH' and s.m.return_journey.leg==2:break
  self.assertEqual(s.m.phase,'SEARCH');self.assertEqual(s.m.return_journey.leg,2)
  j=s.m.return_journey
  before=j.active_elapsed
  s.until('RETURN_HOME',limit=4500)
  self.assertAlmostEqual(j.active_elapsed,before,delta=.2)
  s.step();self.assertGreater(j.active_elapsed,before)
 def test_return_reference_advances_continuously_under_tracking_lag(self):
  s=ReturnSim();s.start();s.until('RETURN_PREP');j=s.m.return_journey
  initial=tuple(s.m.position)
  s.step(False)
  self.assertGreater(math.dist(s.m.position,initial),0.)
  self.assertEqual(j.move_speed('RETURN_PREP'),.5)
  self.assertEqual(j.move_speed('RETURN_HOME'),1.5)

 def test_return_home_keeps_corridor_with_diagnostic_tracking(self):
  s=ReturnSim();s.start();s.until('RETURN_HOME');j=s.m.return_journey
  self.assertFalse(j.p['movement_tracking_error_abort_enabled'])
  self.assertIsNone(s.out['tracking']['tracking_limit_m'])
  self.assertEqual(s.m.core.c['max_tracking_error_m'],.12)
  self.assertTrue(j.region(s.m,s.f.p,-1))
  self.assertEqual(s.m.phase,'RETURN_HOME')
 def test_turn_drift_reacquires_from_settled_actual_pose_without_changing_origin(self):
  for delta in ((-.07222514,-.09651835,-.03160086),(-.20,-.15,-.03)):
   s=ReturnSim();s.start();s.until('YAW_TURN');origin=s.m.origin;owner=s.m.core
   identity=copy.deepcopy(s.m.session_window.identity);epoch=s.m.localization_epoch
   hold=tuple(s.m.position);start=tuple(s.f.p)
   for n in range(1,11):
    s.f.p=tuple(start[k]+(hold[k]+delta[k]-start[k])*n/10 for k in range(3));s.step(False)
   actual=tuple(s.f.p)
   self.assertEqual(s.m.phase,'YAW_TURN',s.m.reason)
   self.assertGreater(s.out['tracking']['error_norm_m'],.12)
   self.assertIsNone(s.out['tracking']['tracking_limit_m'])
   self.assertEqual(tuple(s.m.position),hold)
   for _ in range(400):
    s.step(False)
    if s.m.phase!='YAW_TURN':break
   self.assertEqual(s.m.phase,'SEARCH',s.m.reason)
   self.assertEqual(tuple(s.m.position),actual);self.assertEqual(s.m.origin,origin)
   self.assertIs(s.m.core,owner);self.assertEqual(s.m.session_window.identity,identity)
   self.assertEqual(s.m.localization_epoch,epoch);self.assertIsNone(s.m.session_window.snapshot)
   self.assertEqual(s.actions,['OFFBOARD','ARM'])
   s.step(False);self.assertNotEqual(s.m.phase,'FAULT',s.m.reason)
   s.until('PASS');self.assertEqual(s.m.return_journey.leg,2)
   self.assertIsNot(s.m.session_window,s.m.return_journey.first)

 def test_turn_drift_does_not_bypass_actual_stage_corridor(self):
  s=ReturnSim();s.start();s.until('YAW_TURN')
  s.f.p=(1.65,-2.65,s.m.target_z);s.step(False)
  self.assertEqual(s.m.phase,'FAULT');self.assertEqual(s.m.return_journey.leg,1)
  self.assertFalse(s.m.return_journey.second_pass_complete)

 def test_return_movement_tracking_error_is_diagnostic_and_corridor_still_enforced(self):
  for phase in ('RETURN_PREP','RETURN_HOME'):
   s=ReturnSim();s.start();s.until(phase,limit=4500)
   s.m.position=(s.f.p[0]+.25,s.f.p[1],s.f.p[2]);s.step(False)
   self.assertNotEqual(s.m.phase,'FAULT',s.m.reason)
   self.assertGreater(s.out['tracking']['error_norm_m'],.12)
   self.assertIsNone(s.out['tracking']['tracking_limit_m'])
   self.assertFalse(s.out['tracking']['tracking_error_abort_enabled'])
   s.f.p=(1.65,-2.65,s.m.target_z);s.step(False)
   self.assertEqual(s.m.phase,'FAULT')
   self.assertFalse(s.m.reason.startswith('return_tracking_error'))

 def test_lag_freezes_reference_without_error_abort_and_still_times_out(self):
  from window_pass.geometry import norm,sub
  s=ReturnSim();s.start();s.until('RETURN_PREP');fixed=tuple(s.f.p)
  s.m.return_journey.target=(fixed[0]+.5,fixed[1],fixed[2])
  for _ in range(100):
   s.step(False)
   self.assertEqual(s.m.phase,'RETURN_PREP',s.m.reason)
   self.assertLessEqual(norm(sub(s.m.position,fixed)),.35+1e-8)
  self.assertGreater(s.out['tracking']['error_norm_m'],.12)
  s.m.return_journey.active_elapsed=86.;s.step(False)
  self.assertEqual(s.m.phase,'RETURN_PREP')
  self.assertEqual(s.m.return_journey.return_extra_s,20.)
  s.m.return_journey.active_elapsed=106.;s.step(False)
  self.assertIn(s.m.phase,('HANDOVER_REQUEST','PILOT_CONTROL'))

 def test_return_home_acceleration_and_feedforward_reach_1p5_then_stop(self):
  from window_pass.geometry import norm
  s=ReturnSim();s.start();s.until('RETURN_HOME',limit=4500)
  previous=0.;peak=0.
  while s.m.phase=='RETURN_HOME':
   s.step();speed=s.m.return_journey.reference_speed
   self.assertLessEqual(speed,1.5+1e-8)
   self.assertLessEqual(speed-previous,1.*.05+1e-8)
   if s.m.phase=='RETURN_HOME':
    self.assertIsNotNone(s.out['velocity_local_mps'])
    self.assertLessEqual(norm(s.out['velocity_local_mps']),1.5+1e-8)
   previous=speed;peak=max(peak,speed)
  self.assertGreaterEqual(peak,1.49)
  self.assertTrue(s.m.return_journey.origin_return_stopped)
  self.assertIsNone(s.out['velocity_local_mps'])

 def test_return_feedforward_mavlink_mask_and_phase_limits(self):
  import types
  from unittest.mock import Mock
  from test_auto_transport import app
  class P:
   FRAME_LOCAL_NED=1;IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32;IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256;IGNORE_YAW_RATE=2048;IGNORE_PZ=4
   def __init__(self):self.header=types.SimpleNamespace();self.position=types.SimpleNamespace();self.velocity=types.SimpleNamespace()
  r=app.Runtime.__new__(app.Runtime);r.control={'local_frame':'map'};r.types={'PositionTarget':P};r.motion=Mock();r.motion.get_num_connections.return_value=1;r.takeoff_only=False;r.ros=Mock()
  r.mission=types.SimpleNamespace(session_policy={},full_chain=None,return_journey=types.SimpleNamespace(active=True,move_speed=lambda phase:.5 if phase=='RETURN_PREP' else 1.5),sent=Mock())
  out=dict(position=[0,0,1],yaw=0,phase='RETURN_HOME',velocity_local_mps=[1.5,0,0])
  r.publish(out);message=r.motion.publish.call_args[0][0]
  self.assertEqual(message.type_mask & (8|16|32),0)
  self.assertEqual(message.velocity.x,1.5)
  with self.assertRaises(ValueError):r.publish(dict(out,phase='RETURN_PREP'))
  with self.assertRaises(ValueError):r.publish(dict(out,velocity_local_mps=[1.51,0,0]))
  with self.assertRaises(ValueError):r.publish(dict(out,phase='YAW_TURN'))

 def test_second_pass_waits_for_alignment_target_before_frozen_route(self):
  s=ReturnSim();s.start();s.until('YAW_TURN');s.until('SEARCH');s.until('ADJUST')
  target=s.m.adjust_plan['target']
  s.until('PASS')
  self.assertLessEqual(math.dist(target,s.m.direct_plan['start']),.06+1e-8)

 def test_all_pairs_complete_two_distinct_passes_origin_then_automatic_landing_light(self):
  for ids in ((1,2),(1,3),(2,3)):
   s=ReturnSim(ids);s.start();origin=s.m.origin;ground=s.m.ground_z;owner=s.m.core
   s.until('AUTO_LANDING',limit=4500);j=s.m.return_journey
   self.assertEqual(s.actions,['OFFBOARD','ARM','LAND']);self.assertTrue(j.request_on)
   self.assertEqual(s.m.origin,origin);self.assertEqual(s.m.ground_z,ground);self.assertIs(owner,s.m.core)
   self.assertLess(math.dist(s.f.p[:2],origin[:2]),.081);self.assertTrue(j.second_pass_complete)
   self.assertEqual(sum(e['new']=='SEARCH' for e in s.m.events),2);self.assertEqual(sum(e['new']=='TAKEOFF' for e in s.m.events),1)
   self.assertIsNot(j.first,s.m.session_window);self.assertEqual(j.first.identity,s.m.session_window.identity)
   self.assertGreater(s.m.session_window.snapshot['after_stamp'],j.first.snapshot['captured_ros'])
   self.assertFalse(s.out['full_chain']['mission_complete']);self.assertIsNone(s.out['position'])
   # FCU performs descent; companion issues no further position setpoints.
   s.until('DONE');self.assertTrue(s.out['full_chain']['mission_complete']);self.assertTrue(j.off_ack)
 def test_no_yaw_feedback_never_reacquires_second_window(self):
  s=ReturnSim();s.start();s.until('YAW_TURN');s.follow_yaw=False
  s.until('PILOT_CONTROL');self.assertTrue(any(e['reason'].startswith('yaw180_feedback_timeout') for e in s.m.events));self.assertEqual(s.m.return_journey.leg,1);self.assertFalse(s.m.return_journey.second_pass_complete)
 def test_turn_with_lagged_yaw_feedback_completes_without_reference_runaway(self):
  from window_pass.geometry import yaw,wrap
  s=ReturnSim();s.start();s.until('YAW_TURN');s.follow_yaw=False
  start=s.f.wall;original=s.update
  def delayed(follow=True):
   actual=yaw(s.f.q);delta=wrap(s.m.heading-actual)*.22
   s.f.q=(0.,0.,math.sin((actual+delta)/2),math.cos((actual+delta)/2))
   original(follow)
  s.update=delayed
  while s.m.phase=='YAW_TURN':
   prior=s.m.heading;s.step()
   self.assertLessEqual(abs(wrap(s.m.heading-prior)),math.radians(150)*.05+1e-8)
   self.assertLessEqual(abs(wrap(s.m.heading-yaw(s.f.q))),math.radians(15)+1e-8)
  self.assertEqual(s.m.phase,'SEARCH',s.m.reason);self.assertLess(s.f.wall-start,20.)
 def test_ideal_turn_is_faster_and_second_search_yaw_is_not_an_abort_gate(self):
  from window_pass.geometry import yaw
  s=ReturnSim();s.start();s.until('YAW_TURN');start=s.f.wall
  s.until('SEARCH');self.assertLess(s.f.wall-start,3.)
  s.follow_yaw=False;h=s.m.heading+math.radians(6)
  s.f.q=(0.,0.,math.sin(h/2),math.cos(h/2));s.step()
  self.assertNotEqual(s.m.phase,'FAULT',s.m.reason)
 def test_second_actual_pass_has_no_standalone_yaw_gate(self):
  s=ReturnSim();s.start();s.until('YAW_TURN');s.until('SEARCH');s.until('PASS')
  s.follow_yaw=False;h=s.m.heading+math.radians(6)
  s.f.q=(0.,0.,math.sin(h/2),math.cos(h/2));s.step()
  self.assertNotEqual(s.m.phase,'FAULT',s.m.reason);self.assertIsNone(s.out['heading_limit_rad'])
 def test_fresh_first_window_cannot_authorize_second_pass(self):
  s=ReturnSim();s.start();s.until('YAW_TURN');s.lower=False
  s.until('SEARCH')
  for _ in range(240):s.step()
  self.assertEqual(s.m.phase,'SEARCH')
  s.until('PILOT_CONTROL',limit=5000);self.assertEqual(sum(e['new']=='PASS' for e in s.m.events),1)
  self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_epoch_change_after_turn_is_fault_no_light(self):
  s=ReturnSim();s.start();s.until('YAW_TURN');s.local_epoch+=1;s.step()
  self.assertEqual(s.m.phase,'FAULT');self.assertFalse(s.out['landing_illumination']['request_on'])
 def test_on_ack_timeout_never_reports_success_or_requests_auto_land(self):
  s=ReturnSim();s.start();s.until('LANDING_LIGHT_ON',limit=4500);s.allow_on=False;s.until('FAULT')
  self.assertFalse(s.out['full_chain']['mission_complete']);self.assertNotIn('LAND',s.actions)
 def test_landing_timeout_forces_off_without_rearming(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500)
  for _ in range(1000):
   s.f.p=(s.f.p[0],s.f.p[1],s.f.p[2]+.02);s.step(False)
   if s.m.phase in ('FAULT','DONE'):break
  self.assertEqual(s.m.phase,'FAULT');self.assertTrue(s.m.return_journey.off_ack);self.assertFalse(s.out['landing_illumination']['request_on']);self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
 def test_landed_but_armed_continues_illumination(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.airborne=False
  s.mode='OFFBOARD';s.update(False);s.mode='AUTO.LAND'
  s.m.core.flight_state(s.f.ros,True,True,'AUTO.LAND',s.f.ros,s.f.wall)
  s.out=s.m.tick(s.inputs());self.assertTrue(s.out['landing_illumination']['request_on']);self.assertEqual(s.m.phase,'AUTO_LANDING')
 def test_missing_off_ack_never_marks_complete(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.allow_off=False;s.airborne=False;s.armed=False
  s.until('FAULT');self.assertFalse(s.out['full_chain']['mission_complete']);self.assertIn('OFF_ACK_timeout',s.m.reason)
 def test_pilot_mode_change_stops_light_and_never_reacquires(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.mode='ALTCTL';s.step()
  self.assertEqual(s.m.phase,'FAULT');self.assertFalse(s.out['landing_illumination']['request_on']);self.assertIsNone(s.out['position'])
 def test_new_template_binds_all_pairs_and_preserves_frozen_origin(self):
  from test_full_chain_binding import FullChainBinding
  from test_two_targets import binder
  from full_chain_policy import validate
  for ids in ([1,2],[1,3],[2,3]):
   _,snap,bounds=FullChainBinding().data();cfg=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())
   bound=binder.bind(cfg,snap,bounds,ids)
   self.assertTrue(validate(bound,True));self.assertEqual(bound['mission']['max_mission_time_s'],240.)
   self.assertEqual(bound['site_evidence']['snapshot']['identity'],snap['identity'])
   self.assertTrue(bound['second_window_return_evidence']['automatic_landing'])
 def test_real_outside_return_corridor_rejects_before_motion(self):
  s=ReturnSim();s.start();s.until('RETURN_PREP');j=s.m.return_journey
  self.assertFalse(j.region(s.m,(1.65,-2.65,s.m.target_z),1));self.assertFalse(j.region(s.m,(3.15,-2.65,s.m.target_z),-1))
 def test_whole_mission_deadline_also_applies_during_auto_landing(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.m.began=s.f.wall-240.
  s.step(False);self.assertEqual(s.m.phase,'LANDING_LIGHT_OFF');self.assertFalse(s.out['landing_illumination']['request_on'])
 def test_stale_bridge_heartbeat_stops_landing_illumination(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);original=s.inputs
  s.inputs=lambda:replace(original(),hover_laser=(s.f.wall-1.,original().hover_laser[1]))
  s.step(False);self.assertEqual(s.m.phase,'LANDING_LIGHT_OFF');self.assertFalse(s.out['landing_illumination']['request_on'])
 def test_policy_requires_exact_drawing_and_automatic_landing_scope(self):
  for key,value in [('preparation_forward_left_m',[3.15,-.05]),('landing_light_timeout_s',500.),('authorization','old')]:
   p=copy.deepcopy(DEFAULTS);p[key]=value
   with self.assertRaises(ValueError):settings(p)
 def test_auto_land_mode_must_be_observed_before_releasing_position_authority(self):
  s=ReturnSim();s.start();s.until('LANDING_LIGHT_ON',limit=4500)
  out=s.step(respond=False);self.assertEqual(out['action'],'LAND');self.assertEqual(s.mode,'OFFBOARD')
  self.assertIsNotNone(out['position']);self.assertTrue(out['landing_illumination']['request_on'])
  s.until('FAULT');self.assertFalse(s.out['full_chain']['mission_complete'])
  self.assertFalse(s.out['landing_illumination']['request_on'])
 def test_auto_landing_pilot_override_never_reacquires_or_rearms(self):
  s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.mode='POSCTL';before=list(s.actions)
  s.step(False);self.assertIsNone(s.out['position']);self.assertFalse(s.out['landing_illumination']['request_on'])
  self.assertEqual(s.actions,before);self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_auto_descent_input_loss_still_stops_light(self):
  for key in ('state_fresh','landed_fresh','rc_ok','estimator_ok','graph_ok'):
   s=ReturnSim();s.start();s.until('AUTO_LANDING',limit=4500);s.healthy[key]=False;s.step(False)
   self.assertEqual(s.m.phase,'FAULT',key);self.assertIsNone(s.out['position']);self.assertFalse(s.out['landing_illumination']['request_on'])
if __name__=='__main__':unittest.main()
