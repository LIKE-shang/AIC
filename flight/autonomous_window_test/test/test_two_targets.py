"""Two-target lifecycle, order, off-before-move and scope regression; no ROS IO."""
import copy,json,math,unittest,importlib.util
from pathlib import Path
from dataclasses import replace
from test_full_chain import ChainSim
from test_session_window import IDENTITY
from mission import Mission
from two_targets import selection,TARGETS,SCOPE
from full_chain_policy import validate
from test_full_chain_binding import FullChainBinding
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('pair_bind',ROOT/'scripts/bind_two_target_trial.py');binder=importlib.util.module_from_spec(spec);spec.loader.exec_module(binder)

class PairSim(ChainSim):
 def __init__(self,ids):
  super().__init__();cfg=json.loads((ROOT/'config/full_chain_two_targets.json').read_text());cfg['full_chain']['selected_target_ids']=selection(ids)
  c={k:copy.deepcopy(v) for k,v in self.m.core.c.items() if not k.startswith('_session_')}
  for k in ('traverse_speed_mps','max_pose_step_m','max_pose_speed_mps','tilt_limit_rad'):c[k]=cfg['control'][k]
  c.update(max_traverse_distance_m=3.,max_traverse_time_s=55.,max_total_time_s=120.,geofence_min_m=[-4.,-4.,-2.],geofence_max_m=[5.,5.,4.])
  self.m=Mission(c,dict(self.options,max_mission_time_s=120.,minimum_bottom_height_m=.7),True,'straight_pass',session_window_policy=cfg['session_window'],session_identity=IDENTITY,hover_target_policy=cfg['hover_target'],full_chain_policy=cfg['full_chain']);self.f.ctrl=self.m.core
  for _ in range(30):self.update()
 def update(self,follow=True):
  if getattr(self.m,'phase',None)=='POSTPASS' and self.m.full_chain.stage=='STATION_TRANSFER':
   from test_airborne_geometry import DynamicSim
   DynamicSim.update(self,follow)
  else:super().update(follow)
 def inputs(self):
  i=super().inputs()
  if self.m.hover_target:
   b=dict(i.hover_laser[1],scope=self.m.hover_target.scope)
   i=replace(i,hover_laser=(i.wall,b))
  return i

class TwoTargetTests(unittest.TestCase):
 def test_all_three_pairs_complete_in_ascending_order_one_arm(self):
  for ids in ([2,1],[3,1],[3,2]):
   s=PairSim(ids);s.start();ids=sorted(ids);on=[];prior=False;last_id=-1
   for _ in range(2400):
    s.step();h=s.m.hover_target
    self.assertGreaterEqual(h.request_id,last_id);last_id=h.request_id
    if h.request_on and not prior:on.append(s.m.full_chain.selected[s.m.full_chain.index])
    prior=h.request_on
    if s.m.phase=='PILOT_CONTROL':break
    self.assertNotIn(s.m.phase,('FAULT','TAKEOVER','REFUSED'),(ids,s.m.reason))
   self.assertEqual(s.m.phase,'PILOT_CONTROL');self.assertEqual(on,ids)
   self.assertEqual(s.out['full_chain']['completed_target_ids'],ids)
   self.assertTrue(s.out['full_chain']['mission_complete']);self.assertFalse(s.out['full_chain']['physical_hit_verified'])
   self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertEqual(sum(e['new']=='TAKEOFF' for e in s.m.events),1)
 def test_off_not_confirmed_never_moves_to_second_target(self):
  s=PairSim([1,2]);s.start();s.until('HOVER')
  while s.m.hover_target.phase!='EXPOSE':s.step()
  s.allow_off=False;s.until('PILOT_CONTROL')
  self.assertEqual(s.m.full_chain.index,0);self.assertEqual(s.m.full_chain.results,[])
  self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_each_target_gets_new_stability_and_exposure_interval(self):
  s=PairSim([1,3]);s.start();s.until('HOVER');first_station=s.m.position
  while s.m.full_chain.index==0:s.step()
  self.assertEqual(s.m.phase,'POSTPASS');self.assertFalse(s.m.hover_target.request_on)
  self.assertIsNone(s.m.hover_target.started);self.assertFalse(s.m.full_chain.hold_complete)
  self.assertTrue(s.m.full_chain.results[0]['laser_off_confirmed'])
  s.until('HOVER');self.assertAlmostEqual(math.dist(first_station,s.m.position),2.6,places=5)
  self.assertIsNone(s.m.hover_target.on_ack_wall)
 def test_missing_camera_does_not_gate_pair_fixed_mode(self):
  s=PairSim([2,3]);original=s.inputs
  s.inputs=lambda:replace(original(),hover_visual=None)
  s.start();s.until('PILOT_CONTROL',limit=2400)
  self.assertTrue(s.out['full_chain']['mission_complete'])
 def test_second_target_failure_is_not_two_target_success(self):
  s=PairSim([1,2]);s.start()
  while s.m.full_chain.index==0:s.step()
  s.until('HOVER');s.allow_on=False;s.until('PILOT_CONTROL')
  self.assertEqual(s.out['full_chain']['completed_target_ids'],[1]);self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_invalid_selection_rejected_and_no_random_defaults(self):
  for ids in (None,[],[1],[1,1],[0,2],[1,4],[1,2,3],[True,2],['1',2]):
   with self.assertRaises(ValueError):selection(ids)
  self.assertEqual(selection([3,1]),[1,3])
 def test_all_pairs_bind_drawing_positions_and_compensation_without_precision_claim(self):
  for ids in ([1,2],[1,3],[2,3]):
   _,snap,bounds=FullChainBinding().data();cfg=json.loads((ROOT/'config/full_chain_two_targets.json').read_text())
   result=binder.bind(cfg,snap,bounds,ids);self.assertEqual(result['full_chain']['selected_target_ids'],ids)
   self.assertEqual(result['mission']['max_mission_time_s'],120.)
   self.assertFalse(result['two_target_evidence']['second_target_accuracy_verified']);self.assertFalse(result['site_evidence']['flight_acceptance'])
   self.assertTrue(validate(result,True))
 def test_intertarget_one_mps_curve_respects_accel_jerk_and_rest_endpoints(self):
  from smooth_pass import trajectory,sample
  for length in (1.3,2.6):
   plan=trajectory(length,1.,.6,1.5);self.assertLess(plan['duration_s'],6.)
   self.assertLessEqual(plan['peak_speed_mps'],1.)
   self.assertEqual(sample(plan,0.),(0.,0.,0.));self.assertEqual(sample(plan,plan['duration_s']+1),(length,0.,0.))
   old=sample(plan,0.);dt=.001
   for n in range(1,int(plan['duration_s']/dt)+1):
    row=sample(plan,n*dt);self.assertLessEqual(row[1],1.+1e-8);self.assertLessEqual(abs(row[2]),.6+1e-8)
    self.assertLessEqual(abs(row[2]-old[2])/dt,1.5+1e-6);old=row
 def test_first_station_point35_second_transfer_one_then_zero(self):
  s=PairSim([1,3]);s.start();first=[];second=[]
  for _ in range(2300):
   s.step();v=s.out.get('velocity_local_mps')
   if s.m.phase=='POSTPASS' and v is not None:(second if s.m.full_chain.index else first).append(math.sqrt(sum(x*x for x in v)))
   if s.m.phase=='HOVER':self.assertIsNone(v)
   if s.m.phase=='PILOT_CONTROL':break
  self.assertTrue(first and second);self.assertLessEqual(max(first),.35+1e-8)
  self.assertGreater(max(second),.99);self.assertLessEqual(max(second),1.+1e-8)
  self.assertEqual(s.m.phase,'PILOT_CONTROL')
 def test_transfer_feedforward_is_scoped_to_pair_phase_and_index_speed_limit(self):
  import types
  from unittest.mock import Mock
  from test_auto_transport import app
  class P:
   FRAME_LOCAL_NED=1;IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32;IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256;IGNORE_YAW_RATE=2048;IGNORE_PZ=4
   def __init__(self):self.header=types.SimpleNamespace();self.position=types.SimpleNamespace();self.velocity=types.SimpleNamespace()
  r=app.Runtime.__new__(app.Runtime);r.control={'local_frame':'map'};r.types={'PositionTarget':P};r.motion=Mock();r.motion.get_num_connections.return_value=1;r.takeoff_only=False
  r.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:123.))
  r.mission=types.SimpleNamespace(full_chain=types.SimpleNamespace(scope=SCOPE,index=1),session_policy={'smooth_pass':True},sent=Mock())
  out=dict(position=[3.,-1.,1.],yaw=0.,phase='POSTPASS',velocity_local_mps=[0.,-1.,0.])
  r.publish(out);m=r.motion.publish.call_args[0][0];self.assertEqual(m.type_mask & (8|16|32),0);self.assertEqual(m.velocity.y,-1.)
  for velocity in ([0.,-1.01,0.],[0.,0.,.1],[float('nan'),0.,0.]):
   with self.assertRaises(ValueError):r.publish(dict(out,velocity_local_mps=velocity))
  r.mission.full_chain.index=0
  with self.assertRaises(ValueError):r.publish(out)
  r.publish(dict(out,velocity_local_mps=[0.,-.08,0.]))
  r.mission.full_chain.scope='window_pass_single_target'
  with self.assertRaises(ValueError):r.publish(dict(out,velocity_local_mps=[0.,-.08,0.]))
 def test_approximate_arrival_opens_at_8cm_without_camera_and_without_two_second_delay(self):
  s=PairSim([1,3]);s.start();s.until('HOVER');anchor=s.m.position;original=s.inputs
  s.inputs=lambda:replace(original(),hover_visual=None)
  s.f.p=(anchor[0]+.08,anchor[1],anchor[2]+.05);entered=s.f.wall
  for _ in range(30):
   s.step(follow=False)
   if s.m.hover_target.request_on:break
  self.assertTrue(s.m.hover_target.request_on,s.m.hover_target.reason)
  self.assertLess(s.f.wall-entered,1.1)
  self.assertFalse(s.m.hover_target.gates['vision_required'])
  self.assertEqual(s.m.position,anchor)
 def test_small_exposure_motion_does_not_cancel_three_seconds(self):
  s=PairSim([1,3]);s.start();s.until('HOVER')
  while s.m.hover_target.phase!='EXPOSE':s.step()
  anchor=s.m.position;h=s.m.hover_target
  s.f.p=(anchor[0]+.08,anchor[1],anchor[2]+.04)
  for _ in range(100):
   s.step(follow=False)
   if h.phase in ('OFF_WAIT','DONE'):break
  self.assertTrue(h.exposure_completed,h.reason)
  self.assertGreaterEqual(h.off_request_wall-h.on_ack_wall,3.)
  self.assertFalse(h.request_on)
 def test_faulted_bridge_off_ack_is_recognized_but_never_advances_target(self):
  s=PairSim([1,3]);s.start();s.until('HOVER');h=s.m.hover_target
  h.invalid('simulated_bridge_failure',s.f.wall);original=s.inputs
  def inputs():
   i=original();return replace(i,hover_laser=(i.wall,dict(i.hover_laser[1],fault_latched=True,requested_on=False)))
  s.inputs=inputs;s.step()
  self.assertTrue(h.off_confirmed);self.assertEqual(s.m.full_chain.index,0)
  self.assertEqual(s.m.phase,'HANDOVER_REQUEST');self.assertFalse(h.completed)
  self.assertNotIn('off_ack_timeout',s.m.reason)
 def test_approximate_profile_retains_real_outside_region_rejection(self):
  s=PairSim([1,3]);s.start();s.until('HOVER');anchor=s.m.position
  s.f.p=(anchor[0]+.17,anchor[1],anchor[2])
  for _ in range(30):s.step(follow=False)
  self.assertFalse(s.m.hover_target.request_on)
 def test_only_pair_shortens_minimum_takeoff_timer(self):
  s=PairSim([1,3]);self.assertEqual(s.m.takeoff_minimum_elapsed(),1.)
  self.assertEqual(ChainSim().m.takeoff_minimum_elapsed(),3.)
 def test_old_single_authorization_cannot_authorize_pair(self):
  _,snap,bounds=FullChainBinding().data();cfg=json.loads((ROOT/'config/full_chain_two_targets.json').read_text());bound=binder.bind(cfg,snap,bounds,[1,2])
  bound['full_chain_authorization']['target_count']=1
  with self.assertRaises(ValueError):validate(bound,True)
 def test_target3_fence_cannot_be_silently_enlarged(self):
  _,snap,bounds=FullChainBinding().data();cfg=json.loads((ROOT/'config/full_chain_two_targets.json').read_text());bounds['boundary_distances_m']['right']=2.7
  with self.assertRaisesRegex(ValueError,'outside current ground fence'):binder.bind(cfg,snap,bounds,[1,3])

if __name__=='__main__':unittest.main()
