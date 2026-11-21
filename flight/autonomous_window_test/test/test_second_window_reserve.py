"""Captured second opening: retain original geometry, field and live guards."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from session_window import Acquisition,settings,decorate,alignment_policy
from attitude_envelope import enclosed,support
from straight_pass import route_from_geometry,advance_route
from window_pass.core import Pose
from window_pass.geometry import norm,sub,yaw
from test_second_window_return import ReturnSim

class SecondWindowReserveTests(unittest.TestCase):
 def data(self,reserve=True):
  d=json.loads(Path(__file__).with_name('second_window_edge_case_20261027.json').read_text())
  c=decorate(d['control'],settings(d['profile']));p=d['departure_route'];h=p['yaw_rad']
  pose=Pose(p['departure_pose_stamp'],tuple(p['actual_departure_position_local_m']),
            (0.,0.,math.sin(h/2),math.cos(h/2)),'map','base_link')
  a=Acquisition(d['snapshot']['identity'],True,c['clearance_tracking_budget_m'] if reserve else 0.,
                clearance_relaxation_m=.01 if reserve else 0.,maximize_minimum_clearance=reserve)
  a.snapshot=copy.deepcopy(d['snapshot']);a.plan=copy.deepcopy(d['last_adjustment']);a.plan['yaw_rad']=h
  return d,c,pose,a,alignment_policy(settings(d['profile']))

 def test_recorded_departure_is_rejected_for_reserve_before_pass(self):
  d,c,p,a,policy=self.data()
  with self.assertRaisesRegex(ValueError,'departure_tracking_reserve:top'):
   a.route(c,p,tuple(d['departure_route']['start']))
  # The old first-window policy remains unchanged.
  a.tracking_reserve_m=0.
  self.assertGreater(a.route(c,p,tuple(d['departure_route']['start']))['edge_diagnostics'][0]['clearance_after_budget_m'],0.)

 def test_replan_from_recorded_geometry_fits_all_edges_and_fixed_endpoint(self):
  d,c,p,a,policy=self.data();frozen=copy.deepcopy(a.snapshot)
  plan=a.retarget(c,p,tuple(d['departure_route']['start']),d['ground_z'],.7,d['origin'],policy)
  target=tuple(plan['target']);arrived=replace(p,p=target)
  route=a.route(c,arrived,target)
  self.assertEqual(a.snapshot,frozen)
  self.assertLess(target[2],d['departure_route']['height_local_m']-.03)
  self.assertEqual(route['length_m'],3.)
  for e in route['edge_diagnostics']:
   self.assertGreaterEqual(e['clearance_after_budget_m'],.03*norm(e['translation_gradient_local'])-1e-9)
  # A 2cm vertical deviation must no longer exhaust the recorded top edge.
  advanced=replace(arrived,p=(target[0]+route['direction'][0]*.2,target[1]+route['direction'][1]*.2,target[2]+.02))
  advance_route(route,advanced,target,c,.03,.3)
  # The unchanged live guard still refuses a sufficiently large drift.
  outside=replace(advanced,p=(advanced.p[0],advanced.p[1],target[2]+.081))
  with self.assertRaisesRegex(ValueError,'cross_track'):
   advance_route(route,outside,target,c,.03,.3)

 def test_retarget_does_not_ratchet_height_up_with_reference(self):
  d,c,p,a,policy=self.data();heights=[]
  for _ in range(8):
   plan=a.retarget(c,p,(p.p[0],p.p[1],.8781092912540677),d['ground_z'],.7,d['origin'],policy)
   heights.append(plan['target'][2]);p=replace(p,p=tuple(plan['target']))
  self.assertLess(max(heights)-min(heights),1e-8)

 def test_original_fence_is_not_relaxed_to_force_a_fit(self):
  d,c,p,a,policy=self.data();c['geofence_max_m'][2]=.9
  with self.assertRaises(ValueError):
   a.retarget(c,p,tuple(d['departure_route']['start']),d['ground_z'],.7,d['origin'],policy)

 def test_maximin_selection_improves_weakest_edge_with_scoped_fence_relaxation(self):
  d,c,p,a,policy=self.data();frozen=copy.deepcopy(a.snapshot)
  selected=a.retarget(c,p,tuple(d['departure_route']['start']),d['ground_z'],.7,d['origin'],policy)
  selected_route=a.route(c,replace(p,p=tuple(selected['target'])),tuple(selected['target']))
  nearest=Acquisition(d['snapshot']['identity'],True,c['clearance_tracking_budget_m'],
                     clearance_relaxation_m=.01,maximize_minimum_clearance=False)
  nearest.snapshot=copy.deepcopy(frozen);nearest.plan=copy.deepcopy(a.plan)
  nearest_target=nearest.retarget(c,p,tuple(d['departure_route']['start']),d['ground_z'],.7,d['origin'],policy)
  nearest_route=nearest.route(c,replace(p,p=tuple(nearest_target['target'])),tuple(nearest_target['target']))
  self.assertEqual(c['clearance_margin_m'],.04)
  self.assertEqual(a.scoped_control(c)['_session_field_fence_margin_m'],.03)
  self.assertEqual(selected_route['aperture_clearance_m'],.03)
  self.assertEqual(selected_route['aperture_clearance_relaxation_m'],.01)
  self.assertEqual(selected_route['target_selection_policy'],'maximin_minimum_four_edge_clearance')
  self.assertGreater(min(selected_route['edge_clearance_after_budgets_m']),
                     min(nearest_route['edge_clearance_after_budgets_m'])+1e-7)
  self.assertEqual(a.snapshot,frozen)

 def test_one_centimetre_field_fence_relaxation_is_scoped_to_second_window(self):
  d,c,p,a,policy=self.data()
  h=0.;point=[sum(c['geofence_min_m'][i:i+1]+c['geofence_max_m'][i:i+1])/2 for i in range(3)]
  point[0]=c['geofence_max_m'][0]-support(c,h,(1.,0.,0.))-.035
  scoped=a.scoped_control(c)
  self.assertFalse(enclosed(c,point,h))
  self.assertTrue(enclosed(scoped,point,h))
  first=Acquisition(d['snapshot']['identity'])
  self.assertIs(first.scoped_control(c),c)
  self.assertEqual(scoped['_session_field_fence_margin_m'],.03)

 def test_both_windows_still_complete_one_arm_then_auto_landing(self):
  s=ReturnSim();s.start();first=s.m.session_window
  self.assertEqual(first.tracking_reserve_m,0.)
  s.until('YAW_TURN');s.until('SEARCH');second=s.m.session_window
  self.assertEqual(second.tracking_reserve_m,.01)
  s.until('PASS');self.assertEqual(s.m.direct_plan['departure_tracking_reserve_m'],second.tracking_reserve_m)
  s.until('DONE',limit=4500)
  self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
  self.assertTrue(s.m.return_journey.second_pass_complete)

if __name__=='__main__':unittest.main()
