import unittest,math,copy
from dataclasses import replace
from fixed_aim import compensation,shift
from test_pass_tilt import PassTiltRegression
from test_full_chain import ChainSim
from full_chain import settings

class FixedAimTests(unittest.TestCase):
 def sim(self):return PassTiltRegression().sim()
 def test_total_7cm_opposite_rear_right_once(self):
  f,l=compensation();self.assertAlmostEqual(math.hypot(f,l),.07);self.assertGreater(f,0);self.assertGreater(l,0)
  p=shift((3.15,-.05,1.),0.,(f,l));self.assertAlmostEqual(p[0],3.169545151,places=3);self.assertGreater(p[1],0)
  q=shift((0.,0.,1.),math.pi/2,(f,l));self.assertAlmostEqual(q[0],-l);self.assertAlmostEqual(q[1],f)
 def test_no_camera_or_target_still_timed_3s_and_off_posctl(self):
  s=self.sim();s.until('HOVER');anchor=s.m.position;original=s.inputs
  s.inputs=lambda:replace(original(),hover_visual=None)
  s.until('PILOT_CONTROL');self.assertEqual(s.m.position,anchor)
  self.assertTrue(s.out['full_chain']['mission_complete']);self.assertTrue(s.out['full_chain']['laser_off_confirmed'])
  self.assertGreaterEqual(s.m.hover_target.off_request_wall-s.m.hover_target.on_ack_wall,3.)
  self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL']);self.assertFalse(s.out['full_chain']['physical_hit_verified'])
 def test_camera_error_never_retargets_reference(self):
  s=self.sim();s.until('HOVER');anchor=s.m.position;original=s.inputs
  def inputs():
   i=original();d=dict(i.hover_visual[1]);d['reference_error_px']=[500.,-400.];return replace(i,hover_visual=(i.hover_visual[0],d))
  s.inputs=inputs;s.until('PILOT_CONTROL');self.assertEqual(s.m.position,anchor);self.assertEqual(s.m.hover_target.correction_updates,0)
 def test_motion_out_of_region_during_light_requests_off(self):
  s=self.sim();s.until('HOVER')
  while s.m.hover_target.phase!='EXPOSE':s.step()
  s.f.p=(s.f.p[0]+.06,s.f.p[1],s.f.p[2]);s.step(follow=False)
  self.assertFalse(s.m.hover_target.request_on);self.assertFalse(s.m.hover_target.completed)
 def test_off_ack_still_required(self):
  s=self.sim();s.until('HOVER')
  while s.m.hover_target.phase!='EXPOSE':s.step()
  s.allow_off=False;s.until('PILOT_CONTROL');self.assertFalse(s.out['full_chain']['mission_complete'])
 def test_compensation_scope_rejects_nonfinite_or_large(self):
  from test_full_chain import CHAIN
  for v in ([float('nan'),0.],[.11,0.],[0.,0.,0.]):
   with self.assertRaises(ValueError):settings(dict(CHAIN,fixed_aim_compensation_forward_left_m=v))

if __name__=='__main__':unittest.main()
