"""Takeoff-only operator-observed attitude, with remaining pose checks intact."""
import math,unittest
from test_takeoff import TakeoffSim
from test_mission import Sim
from mission import Mission,settings
from window_pass.core import Pose

class AttitudePolicy(unittest.TestCase):
    def sim(self,enabled):
        s=TakeoffSim();s.options['takeoff_attitude_checks_enabled']=enabled
        s.m=Mission(s.m.core.c,s.options,experiment='takeoff_land');s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('HOVER');return s
    def quaternion(self,axis,angle):
        q=[0.,0.,0.,math.cos(angle/2)];q[axis]=math.sin(angle/2);return tuple(q)
    def test_three_axes_and_angular_jump_exempt(self):
        for axis in range(3):
            s=self.sim(False);s.f.q=self.quaternion(axis,.5);s.update()
            self.assertEqual(s.m.common_error(s.inputs()),'')
            self.assertFalse(s.m.output()['attitude_checks_enabled'])
    def test_yaw_drift_can_complete_hover_land(self):
        s=self.sim(False);s.f.q=self.quaternion(2,.3);s.until('DONE')
        self.assertTrue(s.m.takeoff_hover_completed)
    def test_enabled_checks_still_reject_tilt_and_yaw(self):
        for axis,reason in [(0,'attitude_limit'),(1,'attitude_limit'),(2,'heading_changed')]:
            s=self.sim(True);s.f.q=self.quaternion(axis,.1);s.update()
            self.assertEqual(s.m.common_error(s.inputs()),reason)
    def test_invalid_quaternion_and_position_jump_still_rejected(self):
        for pos,q in [((0.,0.,1.),(0.,0.,0.,0.)),((100.,0.,1.),(0.,0.,0.,1.))]:
            s=self.sim(False);s.f.wall+=.05;s.f.ros+=.05
            ok=s.m.core.pose_update(Pose(s.f.ros,pos,q,'map','base_link'),s.f.ros,s.f.wall)
            self.assertFalse(ok)
    def test_traverse_never_uses_exception(self):
        s=Sim();s.options['takeoff_attitude_checks_enabled']=False
        m=Mission(s.m.core.c,s.options)
        self.assertTrue(m.attitude_checks_enabled())
        self.assertLess(m.core.c['max_attitude_step_rad'],math.pi)
    def test_invalid_policy_rejected(self):
        s=TakeoffSim()
        with self.assertRaises(ValueError):settings(dict(s.options,takeoff_attitude_checks_enabled='false'))
