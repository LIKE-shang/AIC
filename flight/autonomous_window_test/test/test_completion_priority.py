"""Bounded second-opening changes; recorded geometry and synthetic transport."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from test_second_window_return import ReturnSim
from session_window import Acquisition,decorate
from attitude_envelope import enclosed,support
from envelope_fence import envelope_inside,envelope_margins
from unittest.mock import patch

class CompletionPriorityTests(unittest.TestCase):
 def recorded(self):
  p=Path(__file__).resolve().parent/'fixtures/second_finish_211024508216.json';d=json.loads(p.read_text())
  s=ReturnSim((1,2));s.start();s.until('YAW_TURN');s.until('SEARCH');s.until('ADJUST')
  m=s.m;j=m.return_journey;cfg=d['config'];r=d['handover'];z=d['pose'];a=d['adjust']
  m.core.c=decorate(copy.deepcopy(cfg['control']),cfg['session_window']);m.heading=r['yaw']
  m.core.pose=replace(m.core.pose,p=tuple(z['p']),q=tuple(z['q']),stamp=z['stamp'])
  m.origin=tuple(r['full_chain']['takeoff_origin_local_m']);m.ground_z=r['height']['ground_z_local_m']
  m.position=tuple(r['handover_snapshot']['tracking']['target_position_local_m'])
  j.first_plan=copy.deepcopy(d['first']['frozen_straight_route'])
  acq=m.session_window;acq.snapshot=copy.deepcopy(a['session_window']['frozen_geometry'])
  acq.plan=dict(start=list(z['p']),target=list(a['session_window']['candidate_evidence']['selected_target_local_m']),yaw_rad=r['yaw'],epoch=[0,0],normal=acq.snapshot['geometry']['normal_enu'],proposed_route={})
  # retarget creates the same bounded plan from the recorded immutable geometry.
  m.adjust_plan=acq.retarget(m.core.c,m.core.pose,m.position,m.ground_z,m.o['minimum_bottom_height_m'],m.origin,m.session_alignment_policy)
  m.entered=s.f.wall-.1;m.field_alignment_motion.rows.clear()
  for k in range(12):m.field_alignment_motion.update(z['stamp']-.55+.05*k,z['p'])
  return s,d

 def test_recorded_actual_and_reference_corridors_and_departure_route_pass(self):
  s,d=self.recorded();m=s.m;c=m.session_window.scoped_control(m.core.c);p=m.core.pose.p
  strict=dict(c);strict.pop('_session_horizontal_cone_slack_m')
  self.assertFalse(enclosed(strict,p,m.heading));self.assertTrue(enclosed(c,p,m.heading))
  self.assertGreater(envelope_margins(c,p,m.core.pose.q)['lower_m'][1],.05)
  for point in (p,m.position):self.assertTrue(m.return_journey.region(m,point,1))
  plan=m.session_window.route(m.core.c,m.core.pose,m.position)
  self.assertEqual(plan['departure_tracking_reserve_m'],.01)
  self.assertEqual(plan['horizontal_cone_slack_m'],.02)
  for e in plan['edge_diagnostics']:self.assertGreater(e['clearance_after_budget_m'],.01)
  # The same recorded departure used to be rejected by the extra 3cm gate.
  m.session_window.tracking_reserve_m=.03
  with self.assertRaisesRegex(ValueError,'second_window_departure_tracking_reserve:left'):
   m.session_window.route(m.core.c,m.core.pose,m.position)

 def test_recorded_alignment_enters_pass_without_waiting_for_precise_target(self):
  s,d=self.recorded();m=s.m
  # Represent an earlier feasible point, short of the selected centering goal.
  target=list(m.adjust_plan['target']);target[1]-=.01;m.adjust_plan['target']=target
  out=m.align_session_window(s.inputs(),.02)
  self.assertEqual(out['phase'],'PASS',out['reason']);self.assertIsNone(out['action'])
  self.assertFalse(m.return_journey.second_pass_complete)
  self.assertEqual(s.actions,['OFFBOARD','ARM'])

 def test_horizontal_slack_is_bounded_vertical_actual_body_and_divider_stay_hard(self):
  s,d=self.recorded();m=s.m;c=m.session_window.scoped_control(m.core.c);h=m.heading
  self.assertNotIn('_session_horizontal_cone_slack_m',m.core.c)
  for k in range(3):
   w=tuple(-float(i==k) for i in range(3));p=list(m.core.pose.p)
   strict=c['geofence_min_m'][k]+support(c,h,w)+c['_session_field_fence_margin_m']
   p[k]=strict-(.021 if k<2 else .001)
   self.assertFalse(enclosed(c,p,h))
  for k in range(3):
   p=list(m.core.pose.p);p[k]=c['geofence_min_m'][k]
   self.assertFalse(envelope_inside(c,p,m.core.pose.q))
  self.assertFalse(m.return_journey.region(m,m.return_journey.first_plan['geometry_center'],1))

 def test_first_opening_has_no_slack_and_normal_second_creation_is_scoped(self):
  s=ReturnSim();s.start();a=s.m.session_window
  self.assertEqual(a.horizontal_cone_slack_m,0.);self.assertEqual(a.tracking_reserve_m,0.)
  s.until('YAW_TURN');s.until('SEARCH');a=s.m.session_window
  self.assertEqual(a.horizontal_cone_slack_m,.02);self.assertEqual(a.tracking_reserve_m,.01)
  self.assertEqual(a.scoped_control(s.m.core.c)['_session_aperture_clearance_margin_m'],.03)
  for slack in (-.01,.021,float('nan')):
   with self.assertRaises(ValueError):Acquisition(a.identity,horizontal_cone_slack_m=slack)

if __name__=='__main__':unittest.main(verbosity=2)
