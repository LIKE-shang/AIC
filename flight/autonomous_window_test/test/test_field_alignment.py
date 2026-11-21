import copy,unittest
from dataclasses import replace
from test_two_targets import PairSim
from session_window import validate_motion
class FieldAlignmentTests(unittest.TestCase):
 def test_adjusts_beyond20cm_until_actual_ray_passes(self):
  for offset in (.45,.70):
   s=PairSim([1,3]);s.f.cfg['window_center_local_m']=[1.6,offset,1.05]
   s.start();s.until('ADJUST');snapshot=copy.deepcopy(s.m.session_window.snapshot)
   self.assertGreater(s.m.adjust_plan['length_m'],.2)
   self.assertIsNone(s.out['session_window']['clear_column_radius_m'])
   s.until('PASS');self.assertGreater(abs(s.m.core.pose.p[1]-s.m.origin[1]),.2)
   self.assertEqual(s.m.session_window.snapshot,snapshot);self.assertEqual(s.actions,['OFFBOARD','ARM'])
 def test_field_mode_removes_origin_and_total20cm_displacement_checks(self):
  s=PairSim([1,3]);s.start();s.until('ADJUST');p=s.m.core.pose;plan=copy.deepcopy(s.m.adjust_plan)
  plan['start']=[p.p[0]-.4,p.p[1],p.p[2]]
  origin=(p.p[0]-.4,p.p[1],s.m.origin[2])
  validate_motion(plan,s.m.core.c,p,s.m.ground_z,.7,origin,.2,s.m.session_alignment_policy)
 def test_still_rejects_real_vehicle_outside_field(self):
  s=PairSim([1,3]);s.start();s.until('ADJUST');p=s.m.core.pose
  with self.assertRaisesRegex(ValueError,'outside_fence'):
   validate_motion(s.m.adjust_plan,s.m.core.c,replace(p,p=(99.,p.p[1],p.p[2])),s.m.ground_z,.7,s.m.origin,.2,s.m.session_alignment_policy)
 def test_frozen_geometry_can_retarget_after_feedback_motion(self):
  s=PairSim([1,3]);s.f.cfg['window_center_local_m']=[1.6,.6,1.05];s.start();s.until('ADJUST')
  old=copy.deepcopy(s.m.session_window.snapshot);p=s.m.core.pose
  moved=replace(p,p=(p.p[0]+.04,p.p[1]-.08,p.p[2]))
  plan=s.m.session_window.retarget(s.m.core.c,moved,s.m.position,s.m.ground_z,.7,s.m.origin,s.m.session_alignment_policy)
  self.assertEqual(s.m.session_window.snapshot,old);self.assertEqual(plan['start'],list(moved.p))
  self.assertGreater(plan['target'][1],moved.p[1])
 def test_no_aperture_fit_within_field_does_not_authorize_pass(self):
  s=PairSim([1,3]);s.f.cfg['window_center_local_m']=[1.6,1.3,1.05];s.m.core.c['geofence_max_m'][1]=1.
  s.start();s.until('PILOT_CONTROL');self.assertFalse(s.out['straight_pass_completed'])
  self.assertNotIn('PASS',[e['new'] for e in s.m.events])
if __name__=='__main__':unittest.main()
