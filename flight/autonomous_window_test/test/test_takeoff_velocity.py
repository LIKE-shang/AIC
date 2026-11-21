"""Bounded vertical velocity takeoff with lagged synthetic plant; no hardware."""
import math,types,unittest
from unittest.mock import Mock
from test_takeoff import TakeoffSim
from test_mission import Sim
from test_auto_transport import app
from mission import Mission,settings
from window_pass.core import Pose

class VelocitySim(TakeoffSim):
    def __init__(self):
        super().__init__();self.options['takeoff_vertical_velocity_enabled']=True
        self.m=Mission(self.m.core.c,self.options,experiment='takeoff_land');self.f.ctrl=self.m.core
        self.vz=0.;self.spool=0.;self.freeze=False
        for _ in range(30):self.update()
    def update(self,follow=True):
        # Parent construction reaches this override before plant initialization.
        if not hasattr(self,'vz'):return super().update(follow)
        f=self.f;c=self.m.core;f.wall+=.05;f.ros+=.05;self.elapsed+=.05
        if self.armed:self.spool+=.05
        if follow and self.out and self.out['position'] is not None and not self.freeze:
            command=self.out.get('vertical_velocity_mps')
            if command is None:command=max(-.1,min(.1,2*(self.out['position'][2]-f.p[2])))
            if self.spool<1.:command=0.
            self.vz+=(command-self.vz)*.05/.4
            f.p=(f.p[0],f.p[1],max(.245,f.p[2]+self.vz*.05))
        if self.mode=='AUTO.LAND':
            self.vz=0.;f.p=(f.p[0],f.p[1],max(.245,f.p[2]-.015))
            if f.p[2]<=.245:self.airborne=False;self.armed=False
        c.pose_update(Pose(f.ros,f.p,f.q,'map','base_link'),f.ros,f.wall)
        c.flight_state(f.ros,True,self.armed,self.mode,f.ros,f.wall)
        c.extended_state(f.ros,self.airborne,f.ros,f.wall);c.graph(True,'synthetic',f.wall)
        if self.armed and f.p[2]>.30:self.airborne=True

class TakeoffVelocity(unittest.TestCase):
    def test_lagged_climb_hover_and_land(self):
        s=VelocitySim();s.start();commands=[]
        for _ in range(1600):
            out=s.step();commands.append(out)
            if out['phase']=='DONE':break
        self.assertEqual(s.m.phase,'DONE',s.m.reason)
        self.assertTrue(s.m.takeoff_hover_completed)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])
        speeds=[r['vertical_velocity_mps'] for r in commands if r['vertical_velocity_mps'] is not None]
        self.assertTrue(speeds);self.assertLessEqual(max(speeds),.1);self.assertGreaterEqual(min(speeds),0.)
        self.assertTrue(any(0<v<.1 for v in speeds))
        self.assertTrue(all(r['vertical_velocity_mps'] is None for r in commands if r['phase']!='TAKEOFF'))
    def test_false_airborne_without_climb_cannot_pass(self):
        s=VelocitySim();s.freeze=True;s.start();s.until('TAKEOFF');s.airborne=True
        s.until('DONE');self.assertFalse(s.m.takeoff_hover_completed)
        self.assertIn('takeoff_timeout',[e['reason'] for e in s.m.events])
    def test_lateral_error_still_stops(self):
        s=VelocitySim();s.start();s.until('TAKEOFF')
        for _ in range(30):
            s.f.p=(s.f.p[0]+.005,s.f.p[1],s.f.p[2])
            r=s.step(follow=False)
            if r['phase']=='FAULT':break
        self.assertEqual(r['phase'],'FAULT');self.assertIn('tracking_error',r['reason'])
        self.assertIsNone(r['vertical_velocity_mps'])
    def test_overshoot_does_not_command_more_ascent(self):
        s=VelocitySim();s.start();s.until('TAKEOFF')
        p=s.m.core.pose;s.m.core.pose=Pose(p.stamp,(p.p[0],p.p[1],s.m.target_z+.01),p.q,p.frame,p.child)
        self.assertEqual(s.m.takeoff_vertical_velocity(),0.)
    def test_no_velocity_authority_before_arm(self):
        s=VelocitySim();s.start();self.assertIsNone(s.m.output()['vertical_velocity_mps'])
        s.until('ARM');self.assertIsNone(s.m.output()['vertical_velocity_mps'])
    def test_traversal_and_excess_speed_refused(self):
        s=Sim()
        with self.assertRaisesRegex(ValueError,'takeoff-only'):Mission(s.m.core.c,dict(s.options,takeoff_vertical_velocity_enabled=True))
        with self.assertRaises(ValueError):settings(dict(s.options,takeoff_vertical_velocity_enabled=True,climb_speed_mps=.11))

class PositionTarget:
    FRAME_LOCAL_NED=1
    IGNORE_PZ=4;IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32
    IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256;IGNORE_YAW_RATE=2048
    def __init__(self):
        self.header=types.SimpleNamespace();self.position=types.SimpleNamespace();self.velocity=types.SimpleNamespace()

class VelocityTransport(unittest.TestCase):
    def runtime(self):
        r=app.Runtime.__new__(app.Runtime);r.takeoff_only=True
        r.motion=Mock();r.motion.get_num_connections.return_value=1
        r.types={'PositionTarget':PositionTarget};r.control={'local_frame':'map'};r.mission=Mock()
        r.ros=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:10.))
        return r
    def test_xy_position_z_velocity_mask_and_enu_sign(self):
        r=self.runtime();r.publish({'position':(1.,2.,3.),'yaw':.2,'phase':'TAKEOFF','vertical_velocity_mps':.1})
        m=r.motion.publish.call_args[0][0]
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_PZ)
        self.assertFalse(m.type_mask & PositionTarget.IGNORE_VZ)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_VX)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_VY)
        self.assertEqual(m.velocity.z,.1);self.assertEqual((m.position.x,m.position.y),(1.,2.))
    def test_hover_returns_to_position_hold(self):
        r=self.runtime();r.publish({'position':(1.,2.,3.),'yaw':.2,'phase':'HOVER','vertical_velocity_mps':None})
        m=r.motion.publish.call_args[0][0]
        self.assertFalse(m.type_mask & PositionTarget.IGNORE_PZ)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_VZ)
    def test_invalid_or_wrong_phase_never_published(self):
        for value,phase in [(.2,'TAKEOFF'),(-.1,'TAKEOFF'),(float('nan'),'TAKEOFF'),(.1,'HOVER')]:
            r=self.runtime()
            with self.assertRaises(ValueError):r.publish({'position':(0.,0.,1.),'yaw':0.,'phase':phase,'vertical_velocity_mps':value})
            r.motion.publish.assert_not_called()
