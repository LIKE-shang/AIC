import copy,unittest
from policy import chosen,ground_done,same_start,validate_plan,Handoff,flight_failure_reason
D={'state':'DONE','reason':'ALL_WAYPOINTS_REACHED','waypoint_index':4,'waypoint_count':4,'pose':[.01,-.02,.01],'command':{'vx':0.,'vy':0.,'wz':0.}}
class PolicyTests(unittest.TestCase):
 def test_all_pairs_sort_and_invalid_selection_rejects(self):
  for pair in ([3,1],[2,1],[3,2]):self.assertEqual(chosen(pair),sorted(pair))
  for pair in ([],[1],[1,1],[True,2],[0,3],[1,4],[1,2,3]):
   with self.assertRaises(ValueError):chosen(pair)
 def test_only_real_four_point_origin_stop_is_completion(self):
  self.assertTrue(ground_done(D))
  for key,value in [('state','DWELL'),('reason','ARRIVED:3'),('waypoint_index',3),('waypoint_count',3),('pose',[.2,0.,0.]),('pose',[0.,0.,.2]),('pose',[float('nan'),0.,0.]),('command',{'vx':.01,'vy':0.,'wz':0.}),('dry_run',True)]:
   self.assertFalse(ground_done(dict(D,**{key:value})),(key,value))
 def test_boot_run_sources_and_origin_must_match(self):
  a={'identity':{'boot':'a','run':'b','source':'c'},'position_local_m':[0.,0.,0.]}
  same_start(a,copy.deepcopy(a))
  for key in ('boot','run','source'):
   b=copy.deepcopy(a);b['identity'][key]='changed'
   with self.assertRaises(ValueError):same_start(a,b)
  b=copy.deepcopy(a);b['position_local_m'][0]=.11
  with self.assertRaises(ValueError):same_start(a,b)
 def test_prepared_choice_requires_same_build_and_ground_route(self):
  for schema in ('competition_20261025_v1','competition_20261101_aic234471'):
   d={'schema':schema,'ready':True,'selected_target_ids':[1,3],'boot_id':'b','flight_manifest_sha256':'m','ground_config_sha256':'g'}
   with self.subTest(schema=schema):validate_plan(d,'b','m','g')
  for args in [('new','m','g'),('b','new','g'),('b','m','new')]:
   with self.assertRaises(ValueError):validate_plan(d,*args)
  for key,value in [('ready',False),('selected_target_ids',[3,1]),('schema','old')]:
   with self.assertRaises(ValueError):validate_plan(dict(d,**{key:value}),'b','m','g')
class FlightResultRegression(unittest.TestCase):
 def test_posctl_terminal_does_not_hide_original_alignment_rejection(self):
  self.assertEqual(flight_failure_reason({'phase':'PILOT_CONTROL','reason':'POSCTL_observed_pilot_must_land','handover_snapshot':{'phase':'ADJUST','reason':'session_alignment:session_alignment_outside_confirmed_column'}}),'session_alignment:session_alignment_outside_confirmed_column')
 def test_fault_snapshot_has_precedence_over_generic_terminal_reason(self):
  self.assertEqual(flight_failure_reason({'reason':'generic','fault_snapshot':{'reason':'tracking_error'}}),'tracking_error')

class GroundAbortRegression(unittest.TestCase):
 def test_recorded_6p1degree_drift_is_inside_new_ground_bound_but_9degrees_stops(self):
  import math
  from ground_straight_nav.core import StraightRun
  r=StraightRun({'yaw_abort_rad':.15,'max_speed_mps':.4,'waypoint_timeout_s':120.})
  r.pose_update(.117,-2.65,.0344317294,0.);r.start();r.begin([{'map':[0.,-2.9]},{'map':[0.,0.]}])
  r.pose_update(.117,-2.65,.0344317294-.1065253547,.05);r.update(.05)
  self.assertNotEqual(r.state,'ABORT')
  r.pose_update(.117,-2.65,.0344317294-.16,.1);self.assertEqual(r.update(.1),(0.,0.))
  self.assertEqual(r.state,'ABORT');reason=r.reason;self.assertIn('YAW_DRIFT_ABORT',reason)
  r.abort('STOP_REQUESTED');r.abort('NODE_SHUTDOWN');self.assertEqual(r.reason,reason)
 def test_handoff_reports_actual_ground_cause(self):
  h=Handoff()
  with self.assertRaisesRegex(ValueError,'YAW_DRIFT_ABORT:-0.107'):
   h.observe(dict(D,state='ABORT',reason='YAW_DRIFT_ABORT:-0.107',waypoint_index=2),0.,'own','own')

class EntryBoundaryTests(unittest.TestCase):
 def test_no_execute_never_imports_ros_or_calls_transport(self):
  import competition
  from unittest.mock import patch
  with patch.object(competition.subprocess,'Popen',side_effect=AssertionError('unexpected process')),patch.object(competition.subprocess,'run',side_effect=AssertionError('unexpected process')):
   with self.assertRaisesRegex(ValueError,'explicit --execute'):competition.run(False)
 def test_prepare_requires_area_before_any_transport(self):
  import competition
  from unittest.mock import patch
  with patch.object(competition.subprocess,'run',side_effect=AssertionError('unexpected process')):
   with self.assertRaisesRegex(ValueError,'area confirmation'):competition.prepare([1,3],False)
 def test_prepare_rejects_unacknowledged_or_faulted_laser_bridge(self):
  import competition
  healthy={'fault_latched':False,'startup_off_acked':True,'requested_on':False,'fcu_connected':True,'fcu_armed':False}
  competition.validate_laser_bridge_status(healthy)
  for key,value in [('fault_latched',True),('startup_off_acked',False),('requested_on',True),('fcu_connected',False),('fcu_armed',True)]:
   with self.subTest(key=key),self.assertRaisesRegex(ValueError,'PREPARE_CLASS=laser_bridge_unhealthy'):
    competition.validate_laser_bridge_status(dict(healthy,**{key:value}))
 def test_binding_failure_classifies_missing_fcu_local_odometry(self):
  import competition
  message=competition.classify_binding_failure('ValueError: missing telemetry: /mavros/local_position/odom')
  self.assertIn('PREPARE_CLASS=fcu_local_odom_missing',message)
  self.assertIn('estimator fusion flags',message)

class HandoffTests(unittest.TestCase):
 def test_partial_stale_dryrun_old_done_do_not_start_flight(self):
  with self.assertRaises(ValueError):Handoff().observe(D,0.,'own','own')
  for row,age,source in [(D,.6,'own'),(D,0.,'other'),(dict(D,dry_run=True),0.,'own'),(dict(D,waypoint_count=3),0.,'own')]:
   h=Handoff();h.observe({'state':'RUNNING'},0.,'own','own')
   with self.assertRaises(ValueError):h.observe(row,age,source,'own')
 def test_ground_full_loop_then_existing_flight_each_pair(self):
  import math
  from ground_straight_nav.core import StraightRun
  from test_two_targets import PairSim
  for ids in ([1,2],[1,3],[2,3]):
   r=StraightRun({'max_speed_mps':.4,'accel_limit_mps2':.3,'decel_limit_mps2':.3,'arrive_tolerance_m':.05,'arrive_cycles':5,'dwell_s':.2,'max_run_distance_m':4.,'waypoint_timeout_s':120.})
   r.pose_update(0.,0.,0.,0.);r.start();r.begin([{'map':[0.,-.85]},{'map':[.65,-1.85]},{'map':[0.,-2.9]},{'map':[0.,0.]}]);h=Handoff();x=y=0.
   for n in range(1,5000):
    t=n*.05;r.pose_update(x,y,0.,t);vx,vy=r.update(t);x+=vx*.05;y+=vy*.05
    if h.observe(r.summary(),0.,'own','own'):break
   self.assertTrue(h.done);self.assertLess(math.hypot(x,y),.1)
   with self.assertRaises(ValueError):h.observe(r.summary(),0.,'own','own')
   f=PairSim(ids);f.start();f.until('PILOT_CONTROL',limit=2400)
   self.assertTrue(f.out['full_chain']['mission_complete']);self.assertEqual(f.actions,['OFFBOARD','ARM','POSCTL'])
   self.assertEqual(f.out['full_chain']['completed_target_ids'],ids)
if __name__=='__main__':unittest.main()
