import copy,unittest
from dataclasses import replace
from session_window import validate_motion,column_tracking_budget
from test_two_targets import PairSim
class ColumnRegression(unittest.TestCase):
 def fixture(self):
  s=PairSim([1,3]);s.start();s.until('ADJUST')
  c=s.m.core.c;policy=dict(s.m.session_alignment_policy,field_bounded_alignment=False,max_distance_m=.2,max_path_m=.3)
  pose=replace(s.m.core.pose,p=(.10286660492420197,-.024164192378520966,.7880614399909973))
  plan=copy.deepcopy(s.m.adjust_plan)
  plan.update(start=[.05238790065050125,-.04396626725792885,.7561471462249756],target=[.05581874097342025,.0316901691560524,.820571468770504],yaw_rad=-.04531658702432432)
  plan['normal']=[1.,0.,0.];plan['proposed_route']['geometry_center']=[1.49,0.,.66]
  origin=(-.06265710294246674,.014718827791512012,-.079428531229496)
  return s,c,policy,pose,plan,origin
 def test_recorded_17cm_center_not_rejected_by_extra3cm_budget(self):
  s,c,policy,pose,plan,origin=self.fixture();self.assertEqual(column_tracking_budget(c),0.)
  validate_motion(plan,c,pose,-.324428531229496,.7,origin,.20,policy)
 def test_actual20cm_boundary_remains_enforced(self):
  s,c,policy,pose,plan,origin=self.fixture()
  point=(origin[0]+.201,origin[1],pose.p[2]);plan['start']=point
  with self.assertRaisesRegex(ValueError,'outside_confirmed_column'):
   validate_motion(plan,c,replace(pose,p=point),-.324428531229496,.7,origin,.20,policy)
 def test_nonrigid_profiles_keep_existing_tracking_budget(self):
  self.assertEqual(column_tracking_budget({'clearance_tracking_budget_m':.03}),.03)
 def test_displacement_limit_is_still_checked(self):
  s,c,policy,pose,plan,origin=self.fixture();plan['start']=[pose.p[0]-.201,pose.p[1],pose.p[2]]
  with self.assertRaisesRegex(ValueError,'displacement_limit'):
   validate_motion(plan,c,pose,-.324428531229496,.7,origin,.20,policy)
if __name__=='__main__':unittest.main()
