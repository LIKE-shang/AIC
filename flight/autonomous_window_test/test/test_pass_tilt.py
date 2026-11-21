"""Regression of the operator's actual 8.23deg fault, including frozen aperture."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from mission import Mission
from test_full_chain import ChainSim,CHAIN,IDENTITY
from session_window import settings,decorate
from window_pass.core import Pose
from straight_pass import route_from_geometry,advance_route
from attitude_envelope import support,cylinder_axis_support
from window_pass.geometry import rotate
from test_prepass_adjustment import euler

ROOT=Path(__file__).resolve().parents[1]
FIX=json.loads((Path(__file__).with_name('pass_tilt_v2_log_fixture.json')).read_text(encoding='utf-8-sig'))

def replay(angle):
 cfg=json.loads((ROOT/'config/full_chain_single_target.json').read_text());c=cfg['control']
 c.update(geofence_min_m=[-3.,-3.,-2.],geofence_max_m=[5.,5.,4.])
 profile=settings(dict(cfg['session_window'],pass_tilt_limit_rad=angle));c=decorate(c,profile)
 metric=FIX['metric_geometry'];g=dict(center=metric['center_enu_m'],stamp=metric['source_stamp'],epoch=[0,0])
 h=FIX['frozen_heading_rad'];pose=Pose(6208.461406018,tuple(FIX['frozen_start']),(0.,0.,math.sin(h/2),math.cos(h/2)),'map','base_link')
 return c,pose,route_from_geometry(c,pose,g,metric)

class PassTiltRegression(unittest.TestCase):
 def sim(self):
  s=ChainSim();cfg=json.loads((ROOT/'config/full_chain_single_target.json').read_text())
  c={k:copy.deepcopy(v) for k,v in s.m.core.c.items() if not k.startswith('_session_')}
  c.update({k:cfg['control'][k] for k in ('max_pose_step_m','max_pose_speed_mps','tilt_limit_rad')});c.update(max_traverse_distance_m=3.,max_traverse_time_s=55.)
  s.m=Mission(c,s.options,True,'straight_pass',session_window_policy=cfg['session_window'],session_identity=IDENTITY,hover_target_policy=cfg['hover_target'],full_chain_policy=CHAIN);s.f.ctrl=s.m.core
  for _ in range(30):s.update()
  s.start();s.until('PASS');return s
 def test_logged_8p23deg_is_accepted_and_15p1deg_stops_with_evidence(self):
  s=self.sim();s.f.q=euler(s.m.heading,FIX['observed_tilt_rad'],0.);s.step()
  self.assertEqual(s.m.phase,'PASS',s.m.reason)
  self.assertAlmostEqual(s.out['effective_tilt_limit_rad'],math.radians(15))
  s.f.q=euler(s.m.heading,math.radians(15.1),0.);s.step()
  self.assertEqual(s.m.phase,'FAULT');self.assertEqual(s.m.fault_snapshot['reason'],'attitude_limit')
  self.assertAlmostEqual(s.m.fault_snapshot['effective_tilt_limit_rad'],math.radians(15))
  self.assertIsNone(s.out['velocity_local_mps'])
 def test_other_phases_keep_eight_degree_limit(self):
  s=self.sim()
  for phase in ('IDLE','PRESTREAM','SEARCH','ADJUST','POSTPASS','HOVER','HANDOVER_REQUEST'):
   s.m.phase=phase;self.assertAlmostEqual(s.m.attitude_limit(),math.radians(8))
 def test_logged_aperture_recomputed_for_15deg_retains_positive_four_cm_clearance(self):
  c,p,new=replay(math.radians(15));old=replay(math.radians(8))[2]
  self.assertGreater(min(new['edge_clearance_after_budgets_m']),0.)
  self.assertLess(min(new['edge_clearance_after_budgets_m']),min(old['edge_clearance_after_budgets_m']))
  self.assertEqual(new['passage_geometry_tilt_limit_rad'],math.radians(15))
  self.assertEqual(new['command_speed_mps'],1.5)
  # Replay feedback at the fault location with its observed tilt and a nearby old command.
  p=replace(p,p=tuple(FIX['fault_pose']),q=euler(FIX['frozen_heading_rad'],FIX['observed_tilt_rad'],0.))
  advance_route(new,p,p.p,c,.033,.64)
 def test_expanded_support_encloses_actual_tilt_for_every_axis(self):
  c,p,plan=replay(math.radians(15))
  for a in (0.,8.23,12.,15.):
   for az in range(0,360,15):
    axis=rotate(euler(math.radians(az),math.radians(a),0.),(0.,0.,1.))
    for w in ((0,0,1),(0,0,-1),(1,0,0),(-1,0,0),(0,1,0),(0,-1,0)):
     self.assertGreaterEqual(support(c,0.,w)+1e-10,cylinder_axis_support(c,w,axis))
 def test_actual_aperture_violation_not_accepted_by_expanded_angle(self):
  c,p,plan=replay(math.radians(15));bad=list(p.p);bad[2]+=.04
  with self.assertRaisesRegex(ValueError,'aperture'):
   advance_route(plan,replace(p,p=tuple(bad)),p.p,c,.033,0.)
 def test_invalid_scope_angles_rejected(self):
  cfg=json.loads((ROOT/'config/full_chain_single_target.json').read_text())
  for angle in (math.radians(15.01),float('nan'),True):
   with self.assertRaises(ValueError):settings(dict(cfg['session_window'],pass_tilt_limit_rad=angle))
  with self.assertRaises(ValueError):settings(dict(cfg['session_window'],smooth_pass=False))

if __name__=='__main__':unittest.main()
