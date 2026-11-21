import unittest,math
from test_full_chain import ChainSim,CHAIN
from full_chain import settings
from fixed_aim import shift

class EmpiricalAimTrim(unittest.TestCase):
 def test_rear_left_error_adds_front_right_to_mechanical_once(self):
  step=.116/math.sqrt(2);mechanical=[.01954546347899043,.06721588247870776]
  s=ChainSim();s.m.full_chain.p.update(fixed_aim_compensation_forward_left_m=mechanical,empirical_aim_trim_forward_left_m=[step,-step])
  s.start();s.until('HOVER');target=s.m.full_chain.target_center
  expected=shift(shift(target,s.m.heading,mechanical),s.m.heading,[step,-step])
  for a,b in zip(s.m.full_chain.station,expected):self.assertAlmostEqual(a,b)
  self.assertAlmostEqual(math.hypot(step,-step),.116)
  self.assertEqual(s.m.full_chain.target_center,target)
 def test_trim_is_not_mechanical_offset_and_has_own_cap(self):
  for trim in ([float('nan'),0.],[.151,0.],[True,0.],[0.,0.,0.]):
   with self.assertRaises(ValueError):settings(dict(CHAIN,fixed_aim_compensation_forward_left_m=[.02,.067],empirical_aim_trim_forward_left_m=trim))
  with self.assertRaises(ValueError):settings(dict(CHAIN,empirical_aim_trim_forward_left_m=[.08,-.08]))
  p=settings(dict(CHAIN,fixed_aim_compensation_forward_left_m=[.02,.067],empirical_aim_trim_forward_left_m=[.082,-.082]))
  self.assertEqual(p['station_offset_forward_left_m'],[0.,0.])
 def test_zero_trim_keeps_historical_station(self):
  s=ChainSim();s.m.full_chain.p['empirical_aim_trim_forward_left_m']=[0.,0.];s.start();s.until('HOVER')
  self.assertEqual(s.m.full_chain.station,s.m.full_chain.target_center)

if __name__=='__main__':unittest.main()
