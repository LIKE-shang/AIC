"""Legacy finite recovery component tests, explicitly opted in; no hardware IO.
Current competition disables this gate; see test_flight_heading_policy.
"""
import copy,math,unittest
from dataclasses import replace
from test_second_window_return import ReturnSim
from test_two_targets import PairSim
from window_pass.geometry import wrap

def q(angle):return (0.,0.,math.sin(angle/2),math.cos(angle/2))
def second_transfer():
 s=ReturnSim();s.m.heading_error_gate_enabled=lambda:True;s.start()
 for _ in range(2000):
  s.step()
  if s.m.full_chain.index==1 and s.m.full_chain.stage=='STATION_TRANSFER':return s
 raise AssertionError(s.m.reason)
def angle(s,degrees):
 s.follow_yaw=False;s.f.q=q(s.m.heading+math.radians(degrees))

class TransferHeadingTests(unittest.TestCase):
 def test_recorded_excursion_completes_second_exposure_only_after_recovery(self):
  s=second_transfer();heading=s.m.heading;origin=s.m.origin;owner=s.m.core;station=s.m.full_chain.station
  # Reach the final deceleration interval where the recorded yaw drift occurred.
  for _ in range(75):s.step()
  for degrees in (-3.9,-4.3,-4.85,-5.062215856823332,-6.45):
   angle(s,degrees);s.step();self.assertEqual(s.m.phase,'POSTPASS',s.m.reason);self.assertFalse(s.m.hover_target.request_on)
  for _ in range(15):s.step()
  self.assertEqual(s.m.phase,'POSTPASS');self.assertFalse(s.m.hover_target.request_on)
  angle(s,2.)
  for _ in range(8):s.step();self.assertFalse(s.m.hover_target.request_on)
  s.follow_yaw=True;s.until('HOVER');self.assertTrue(s.m.full_chain.transfer_heading.recovered)
  self.assertEqual(s.m.heading,heading);self.assertEqual(s.m.origin,origin);self.assertIs(s.m.core,owner);self.assertEqual(s.m.position,station)
  while not s.m.hover_target.exposure_completed:s.step()
  self.assertEqual(s.m.full_chain.index,1);self.assertEqual(s.actions,['OFFBOARD','ARM'])
 def test_full_two_window_flow_continues_after_recovery_one_arm(self):
  s=second_transfer();angle(s,6.5)
  for _ in range(10):s.step()
  angle(s,0.);s.follow_yaw=True;s.until('AUTO_LANDING',limit=4500)
  self.assertEqual(s.actions,['OFFBOARD','ARM','LAND']);self.assertEqual(s.m.full_chain.selected,[1,3]);self.assertEqual(len(s.m.full_chain.results),2)
  self.assertTrue(s.m.return_journey.second_pass_complete)
 def test_over10deg_still_faults_without_light(self):
  s=second_transfer();angle(s,10.1);s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIn('over10deg',s.m.reason);self.assertFalse(s.out['hover_target']['request_on'])
 def test_persistent_six_degrees_times_out_without_second_ON(self):
  s=second_transfer();angle(s,6.)
  for _ in range(90):
   s.step()
   if s.m.phase=='FAULT':break
  self.assertEqual(s.m.phase,'FAULT');self.assertIn('recovery_timeout',s.m.reason);self.assertFalse(s.m.hover_target.status()['laser_on_ever_requested'])
 def test_chatter_below5_but_above3_does_not_reset_deadline(self):
  s=second_transfer();angle(s,6.);s.step();angle(s,4.)
  for _ in range(90):
   s.step()
   if s.m.phase=='FAULT':break
  self.assertEqual(s.m.phase,'FAULT');self.assertFalse(s.m.hover_target.status()['laser_on_ever_requested'])
 def test_epoch_change_during_recovery_still_faults(self):
  s=second_transfer();angle(s,6.);s.step();s.local_epoch+=1;s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIn('epoch',s.m.reason)
 def test_mode_exit_never_reacquires_during_recovery(self):
  s=second_transfer();angle(s,6.);s.step();s.mode='POSCTL';s.step();self.assertEqual(s.m.phase,'TAKEOVER');self.assertFalse(s.out['hover_target']['request_on'])
 def test_pass_yaw_limit_unchanged(self):
  s=ReturnSim();s.m.heading_error_gate_enabled=lambda:True;s.start();s.until('PASS');angle(s,6.);s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIn('heading_changed',s.m.reason)
 def test_first_target_transfer_yaw_limit_unchanged(self):
  s=ReturnSim();s.m.heading_error_gate_enabled=lambda:True;s.start();s.until('POSTPASS');angle(s,6.);s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIn('heading_changed',s.m.reason)
 def test_second_target_HOVER_yaw_limit_unchanged(self):
  s=second_transfer();s.until('HOVER');angle(s,6.);s.step();self.assertEqual(s.m.phase,'FAULT');self.assertFalse(s.out['hover_target']['request_on'])
 def test_yaw180_turn_uses_feedback_instead_of_station_transfer_guard(self):
  s=ReturnSim();s.m.heading_error_gate_enabled=lambda:True;s.start();s.until('YAW_TURN');s.follow_yaw=False;angle(s,6.);s.step();self.assertEqual(s.m.phase,'YAW_TURN')
 def test_region_and_cylinder_scope_required(self):
  s=second_transfer();guard=s.m.full_chain.transfer_heading
  self.assertTrue(guard.scoped(s.m));p=s.m.core.pose
  s.m.core.pose=replace(p,p=(99.,0.,1.));self.assertFalse(guard.scoped(s.m));s.m.core.pose=p
  s.m.direct_pass_completed=False;self.assertFalse(guard.scoped(s.m));s.m.direct_pass_completed=True
  s.m.core.c.pop('_session_cylinder_radius_m');self.assertFalse(guard.scoped(s.m))
 def test_legacy_pair_profile_not_changed(self):
  s=PairSim([1,3]);s.start()
  while s.m.full_chain.index==0:s.step()
  self.assertFalse(s.m.full_chain.transfer_heading.scoped(s.m));self.assertAlmostEqual(s.m.heading_limit(),math.radians(5),places=6)
 def test_duplicate_pose_cannot_finish_recovery(self):
  s=second_transfer();g=s.m.full_chain.transfer_heading;angle(s,6.);s.step();angle(s,0.);s.update();i=s.inputs()
  g.check(s.m,i)
  for n in range(1,15):g.check(s.m,replace(i,wall=i.wall+n*.05))
  self.assertFalse(g.recovered)
 def test_unchanged_reference_trajectory_has_no_restart(self):
  s=second_transfer();s.step();plan=copy.deepcopy(s.m.full_chain.transfer_plan);start=s.m.full_chain.transfer_started;angle(s,6.)
  for _ in range(12):s.step()
  self.assertEqual(s.m.full_chain.transfer_plan,plan);self.assertEqual(s.m.full_chain.transfer_started,start)
if __name__=='__main__':unittest.main()
