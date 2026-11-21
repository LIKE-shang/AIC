"""Meaningful S-curve, tracking, continuity, and publication boundary checks; offline only."""
import math,types,unittest,copy
from dataclasses import replace
from unittest.mock import Mock
from smooth_pass import trajectory,sample
from test_full_chain import ChainSim
from test_auto_transport import app
from window_pass.core import Pose
from full_chain_localization import RouteContinuity
from session_window import settings,decorate
from straight_pass import advance_route

class SmoothTrajectory(unittest.TestCase):
 def test_three_metre_rest_to_rest_reaches_requested_speed_and_respects_dynamics(self):
  p=trajectory(3.);self.assertAlmostEqual(p['peak_speed_mps'],1.5);self.assertAlmostEqual(p['duration_s'],3.75)
  old=sample(p,0.);dt=.001
  for n in range(1,3751):
   row=sample(p,n*dt);self.assertGreaterEqual(row[0],old[0]-1e-9)
   self.assertLessEqual(row[1],1.5+1e-8);self.assertLessEqual(abs(row[2]),1.+1e-8)
   self.assertLessEqual(abs(row[2]-old[2])/dt,4.+1e-6);old=row
  self.assertEqual(sample(p,100),(3.,0.,0.))
 def test_short_routes_reduce_peak_without_instant_velocity_step(self):
  for length in (.01,.1,.6,1.,2.,3.):
   p=trajectory(length);last=sample(p,p['duration_s']-1e-6)
   self.assertAlmostEqual(last[0],length,places=8);self.assertLess(last[1],1e-6)
   self.assertEqual(sample(p,0),(0.,0.,0.));self.assertLessEqual(p['peak_speed_mps'],1.5)
 def test_phase_boundaries_continuous_position_velocity_acceleration(self):
  p=trajectory(3.);t=0.
  for duration,j in p['phases']:
   t+=duration;a=sample(p,max(0.,t-1e-7));b=sample(p,t+1e-7)
   for x,y in zip(a,b):self.assertLess(abs(x-y),1e-6)
 def test_ten_hz_pose_at_1p5_accepted_but_reset_stays_latched(self):
  s=ChainSim();c={k:v for k,v in s.m.core.c.items() if not k.startswith('_session_')};c.update(max_pose_step_m=.35,max_pose_speed_mps=2.2)
  g=RouteContinuity(c)
  for n in range(25):
   p=Pose(100+n*.1,(n*.15,0.,0.),(0.,0.,0.,1.),'camera_init','body')
   self.assertTrue(g.observe(p,n+1,p.stamp,p.stamp,'lio','lio'),g.reason)
  self.assertFalse(g.observe(replace(p,stamp=p.stamp+.1,p=(12.,0.,0.)),26,p.stamp+.1,p.stamp+.1,'lio','lio'))
  self.assertFalse(g.observe(replace(p,stamp=p.stamp+.2),27,p.stamp+.2,p.stamp+.2,'lio','lio'))
 def test_longitudinal_lag_and_cross_track_remain_distinct(self):
  s=ChainSim();s.start();s.until('PASS');plan=copy.deepcopy(s.m.direct_plan)
  plan['smooth_trajectory']=trajectory(plan['length_m']);c=s.m.core.c;pose=s.m.core.pose
  # 15cm along-track is accepted at the matching time while lateral 9cm is rejected.
  t=1.;x=sample(plan['smooth_trajectory'],t)[0];start=plan['start'];direction=plan['direction']
  p=tuple(start[i]+direction[i]*(x-.15) for i in range(3))
  advance_route(plan,replace(pose,p=p),c=c,command=p,dt=.033,elapsed=t)
  with self.assertRaisesRegex(ValueError,'along_tracking'):
   p=tuple(start[i]+direction[i]*(x-.36) for i in range(3));advance_route(plan,replace(pose,p=p),p,c,.033,t)
  p=list(start);p[0]+=x;p[1]+=.09
  with self.assertRaisesRegex(ValueError,'cross_track|aperture'):
   advance_route(plan,replace(pose,p=p),p,c,.033,t)
 def test_velocity_feedforward_mask_and_phase_scope(self):
  class P:
   FRAME_LOCAL_NED=1;IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32;IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256;IGNORE_YAW_RATE=2048;IGNORE_PZ=4
   def __init__(self):
    self.header=types.SimpleNamespace();self.position=types.SimpleNamespace();self.velocity=types.SimpleNamespace()
  r=app.Runtime.__new__(app.Runtime);r.control={'local_frame':'map'};r.types={'PositionTarget':P};r.motion=Mock();r.motion.get_num_connections.return_value=1;r.takeoff_only=False
  r.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:123.));r.mission=types.SimpleNamespace(session_policy={'smooth_pass':True},sent=Mock())
  out=dict(position=[1.,0.,1.],yaw=0.,phase='PASS',velocity_local_mps=[1.5,0.,0.])
  r.publish(out);m=r.motion.publish.call_args[0][0]
  self.assertEqual(m.type_mask & (8|16|32),0);self.assertEqual(m.type_mask & 7,0);self.assertEqual(m.velocity.x,1.5)
  for change in [dict(phase='HOVER'),dict(velocity_local_mps=[1.51,0.,0.]),dict(velocity_local_mps=[0.,0.,.1]),dict(velocity_local_mps=[float('nan'),0.,0.])]:
   with self.assertRaises(ValueError):r.publish(dict(out,**change))
  r.publish(dict(out,phase='POSTPASS',velocity_local_mps=None));self.assertEqual(r.motion.publish.call_args[0][0].type_mask & (8|16|32),8|16|32)

if __name__=='__main__':unittest.main()
