"""Full mission with fixed-height ascent and observed pilot handover; no hardware IO."""
import unittest
from dataclasses import replace
from test_mission import Sim
from mission import Mission
from test_auto_transport import app

class TraversePilotSim(Sim):
    def __init__(self,target=True):
        super().__init__(target)
        self.options.update(takeoff_position_goal_enabled=True,pilot_landing_enabled=True,
                            takeoff_attitude_checks_enabled=False)
        self.m=Mission(self.m.core.c,self.options);self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def update(self,follow=True):
        # A full-height setpoint must drive a bounded synthetic plant rather
        # than teleport feedback through the position-jump guard.
        if follow and self.out and self.out['position'] is not None:
            original=self.out
            limited=tuple(p+max(-.01,min(.01,t-p)) for p,t in zip(self.f.p,self.out['position']))
            self.out=dict(self.out,position=limited)
            super().update(follow)
            self.out=original
        else:super().update(follow)
    def step(self,follow=True,respond=True):
        out=super().step(follow,respond)
        if out['action']=='POSCTL' and respond:self.mode='POSCTL'
        return out

class TraversePilotPolicy(unittest.TestCase):
    def test_full_mission_hands_over_after_pass_without_land(self):
        s=TraversePilotSim();s.start();s.until('ARM')
        self.assertEqual(s.m.position,s.m.origin)
        s.until('TAKEOFF');self.assertAlmostEqual(s.m.position[2]-s.m.origin[2],.9)
        s.until('PILOT_CONTROL')
        self.assertIn('PASS',[e['new'] for e in s.m.events])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
        self.assertGreater(s.f.p[0],1.5);self.assertTrue(s.armed)
        self.assertIsNone(s.out['position']);self.assertTrue(s.out['handover_confirmed'])
    def test_recognition_timeout_hands_over(self):
        s=TraversePilotSim(False);s.start();s.until('PILOT_CONTROL')
        self.assertNotIn('PASS',[e['new'] for e in s.m.events])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
    def test_handover_requires_actual_mode(self):
        s=TraversePilotSim(False);s.start();s.until('SEARCH')
        out=s.m.land('recognition_timeout',s.inputs());self.assertEqual(out['action'],'POSCTL')
        s.m.command_finished('POSCTL',True)
        for _ in range(3):s.step(respond=False)
        self.assertEqual(s.m.phase,'HANDOVER_REQUEST')
        s.mode='POSCTL';s.step();self.assertEqual(s.m.phase,'PILOT_CONTROL')
    def test_real_input_fault_retains_fault_handling(self):
        s=TraversePilotSim();s.start();s.until('SEARCH');s.healthy['estimator_ok']=False;s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('POSCTL',s.actions)
    def test_traverse_attitude_exception_cannot_leak(self):
        s=TraversePilotSim();self.assertTrue(s.m.attitude_checks_enabled())
        s.start();s.until('TAKEOFF');s.m.core.pose=replace(s.m.core.pose,q=(0,0,.2,.979795897))
        out=s.m.tick(s.inputs());self.assertEqual(out['phase'],'FAULT')
    def test_flight_policy_enforced_without_commissioning(self):
        c={'mission':dict(TraversePilotSim().options),'expected_fcu_parameters':{}}
        with self.assertRaises(ValueError):app.commissioning_deferrals(c,False)
        c['expected_fcu_parameters'].update(MPC_Z_VEL_MAX_UP=[0,.4],MPC_ACC_UP_MAX=[0,.4],COM_OBL_RC_ACT=[0,0.])
        self.assertFalse(app.commissioning_deferrals(c,False))
        c['expected_fcu_parameters']['COM_OBL_RC_ACT']=[4,0.]
        with self.assertRaises(ValueError):app.commissioning_deferrals(c,False)

    def test_rejected_posctl_after_pass_is_fault_without_land_or_retry(self):
        s=TraversePilotSim();s.start();s.until('HANDOVER_REQUEST')
        s.m.command_finished('POSCTL',False);s.step(respond=False)
        self.assertEqual(s.m.phase,'FAULT')
        self.assertNotIn('LAND',s.actions);self.assertEqual(s.actions.count('POSCTL'),1)
    def test_airborne_rgbd_geometry_still_required_with_new_policies(self):
        from test_airborne_geometry import DynamicSim
        s=DynamicSim(False);s.options.update(pilot_landing_enabled=True,takeoff_position_goal_enabled=True)
        s.m=Mission(s.m.core.c,s.options,airborne_geometry=True);s.f.ctrl=s.m.core
        for _ in range(30):s.update()
        s.start();s.until('TAKEOFF')
        self.assertFalse(s.m.core.live_bound)
        self.assertEqual(s.m.core.c['window_center_local_m'],[])
        self.assertNotIn('PASS',[e['new'] for e in s.m.events])
