"""Actual terminal overrun and same-owner, stable-target light regressions. No ROS IO."""
import json,copy,math,unittest
from pathlib import Path
from dataclasses import replace
import test_pass_tilt as tilttest
from pass_terminal import MotionWindow,TerminalCapture
from straight_pass import advance_route
from smooth_pass import trajectory,sample
from test_prepass_adjustment import euler
from session_window import settings,decorate
from window_pass.core import Pose

ROOT=Path(__file__).resolve().parents[1]
FIX=json.loads(Path(__file__).with_name('terminal_failure_fixture.json').read_text(encoding='utf-8-sig'))

class TerminalAndTarget(unittest.TestCase):
 def sim(self):
  s=tilttest.PassTiltRegression().sim();s.m.hover_target.p.update(fixed_position_timed=False,require_stable_target_before_light=True,visual_servo_enabled=True)
  return s
 def test_real_16cm_overrun_does_not_abort_before_settling(self):
  cfg=json.loads((ROOT/'config/full_chain_single_target.json').read_text());c=cfg['control'];c.update(geofence_min_m=[-3.,-3.,-2.],geofence_max_m=[5.,5.,4.]);c=decorate(c,settings(cfg['session_window']))
  p=copy.deepcopy(FIX['frozen_straight_route']);p.update(terminal_capture_enabled=True,smooth_trajectory=trajectory(2.625))
  actual=FIX['tracking']['actual_position_local_m'];pose=Pose(1024.946873821,tuple(actual),euler(p['yaw_rad'],FIX['observed_tilt_rad'],0.),'map','base_link')
  command,arrived=advance_route(p,pose,FIX['tracking']['target_position_local_m'],c,.033,3.2)
  self.assertFalse(arrived);self.assertAlmostEqual(sample(p['smooth_trajectory'],3.5)[1],0.)
  self.assertLessEqual(math.dist(command,p['start']),3.)
 def test_no_target_never_lights_and_returns_posctl_after_off_ack(self):
  s=self.sim();s.until('HOVER');s.detected=False;s.until('PILOT_CONTROL')
  self.assertIsNone(s.m.hover_target.on_request_wall);self.assertFalse(s.out['full_chain']['mission_complete'])
  self.assertTrue(s.out['full_chain']['laser_off_confirmed']);self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
 def test_new_chain_pass_stops_and_acquires_target_before_one_light(self):
  s=self.sim();self.assertTrue(s.m.terminal_capture)
  s.until('POSTPASS');self.assertIsNotNone(s.m.terminal_capture.stop_position)
  self.assertEqual(s.m.position,s.m.terminal_capture.stop_position)
  self.assertFalse(s.m.hover_target.request_on);s.until('HOVER');s.until('PILOT_CONTROL')
  self.assertTrue(s.out['full_chain']['mission_complete']);self.assertTrue(s.m.hover_target.gates['stable_target_held'])
  self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertFalse(s.out['full_chain']['physical_hit_verified'])
 def test_predicted_alignment_without_observed_spot_cannot_complete_exposure(self):
  s=self.sim();s.until('HOVER')
  original=s.inputs
  def inputs():
   i=original();row=dict(i.hover_visual[1]);row['observed_error_px']=None;row['observed_inside_inner']=False
   return replace(i,hover_visual=(i.hover_visual[0],row))
  s.inputs=inputs;s.until('PILOT_CONTROL')
  self.assertIsNotNone(s.m.hover_target.on_request_wall)
  self.assertFalse(s.out['full_chain']['mission_complete']);self.assertFalse(s.m.hover_target.exposure_completed)
  self.assertTrue(s.out['full_chain']['laser_off_confirmed'])
 def test_valid_target_outside_alignment_region_does_not_request_light(self):
  s=self.sim();s.until('HOVER');original=s.inputs
  def inputs():
   i=original();row=dict(i.hover_visual[1]);row.update(reference_error_px=[100.,100.],reference_normalized_radius=6.8)
   return replace(i,hover_visual=(i.hover_visual[0],row))
  s.inputs=inputs;s.until('PILOT_CONTROL')
  self.assertIsNone(s.m.hover_target.on_request_wall);self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_unconfirmed_off_never_becomes_completed_single_target(self):
  s=self.sim();s.until('HOVER')
  while s.m.hover_target.phase!='EXPOSE':s.step()
  s.allow_off=False;s.until('PILOT_CONTROL')
  self.assertFalse(s.out['full_chain']['mission_complete']);self.assertFalse(s.out['full_chain']['laser_off_confirmed'])
 def test_fast_segment_stops_short_of_nominal_end(self):
  s=self.sim();p=s.m.direct_plan
  self.assertEqual(p['length_m'],3.);self.assertEqual(p['smooth_trajectory']['length_m'],2.625)
  self.assertEqual(p['smooth_trajectory']['peak_speed_mps'],1.5)
  self.assertAlmostEqual(p['smooth_trajectory']['duration_s'],3.5)
 def test_jitter_does_not_prevent_net_motion_stop_but_large_sweep_does(self):
  w=MotionWindow()
  for n in range(30):w.update(n*.05,(.002*(-1)**n,0.,1.))
  self.assertTrue(w.stable(.04));self.assertLess(w.span,.005)
  w=MotionWindow()
  for n in range(30):w.update(n*.05,(.03*(-1)**n,0.,1.))
  self.assertFalse(w.stable(.04))
 def test_fast_feedback_is_not_mislabelled_stopped(self):
  w=MotionWindow()
  for n in range(30):w.update(n*.05,(n*.05*.15,0.,1.))
  self.assertFalse(w.stable());self.assertAlmostEqual(w.speed,.15)
 def test_stalled_terminal_times_out_without_light(self):
  s=self.sim();s.m.terminal_capture.stage='TERMINAL_CAPTURE';s.m.entered=s.f.wall-16.
  s.step();self.assertEqual(s.m.phase,'FAULT');self.assertIsNone(s.m.hover_target.on_request_wall)
 def test_full_body_clear_latched_and_reentry_rejected(self):
  s=self.sim();s.until('POSTPASS');t=s.m.terminal_capture;t.clear=True
  s.m.phase='PASS';s.m.entered=s.f.wall-4.;p=s.m.core.pose
  s.m.core.pose=replace(p,p=tuple(s.m.direct_plan['start']))
  with self.assertRaisesRegex(ValueError,'reentered'):t.update(s.m,s.inputs(),.033)
 def test_overshoot_fence_is_still_enforced(self):
  s=self.sim();p=s.m.direct_plan;start=p['start'];direction=p['direction'];pos=tuple(start[i]+3.51*direction[i] for i in range(3))
  pose=replace(s.m.core.pose,p=pos)
  with self.assertRaisesRegex(ValueError,'outside_route'):advance_route(p,pose,pos,s.m.core.c,.033,4.)

if __name__=='__main__':unittest.main()
