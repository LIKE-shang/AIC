"""Pilot landing handover must be observed; never autonomously land or rearm."""
import unittest,dataclasses,types
from test_position_goal_takeoff import GoalSim
from test_auto_transport import app
import test_auto_transport as transport_tests
from mission import Mission
from test_mission import Sim

class PilotSim(GoalSim):
    def __init__(self):
        super().__init__();self.options['pilot_landing_enabled']=True
        self.m=Mission(self.m.core.c,self.options,experiment='takeoff_land');self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def step(self,follow=True,respond=True):
        out=super().step(follow,respond)
        if out.get('action')=='POSCTL' and respond:self.mode='POSCTL'
        return out

class PilotLanding(unittest.TestCase):
    def test_complete_hover_hands_over_without_land(self):
        s=PilotSim();s.start();s.until('PILOT_CONTROL',limit=1700)
        self.assertTrue(s.m.takeoff_hover_completed)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
        self.assertTrue(s.armed);self.assertTrue(s.airborne)
        self.assertTrue(s.out['pilot_landing_required']);self.assertTrue(s.out['handover_confirmed'])
        self.assertIsNone(s.out['position'])
    def test_handover_cannot_complete_on_acceptance_alone(self):
        s=PilotSim();s.start();s.until('HOVER');s.out=s.m.handover('test',s.inputs())
        s.m.command_finished('POSCTL',True)
        for _ in range(4):s.step(respond=False)
        self.assertEqual(s.m.phase,'HANDOVER_REQUEST')
        self.assertFalse(s.out['handover_confirmed']);self.assertIsNotNone(s.out['position'])
    def test_timeout_hands_over_not_land(self):
        s=PilotSim();s.freeze=True;s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.m.takeoff_hover_completed)
        self.assertNotIn('LAND',s.actions)
    def test_rejected_handover_does_not_loop_or_land(self):
        s=PilotSim();s.start();s.until('HOVER');s.m.handover('test',s.inputs())
        s.m.command_finished('POSCTL',False);s.step(respond=False)
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('LAND',s.actions)
    def test_missing_mode_confirmation_times_out(self):
        s=PilotSim();s.start();s.until('HOVER');s.m.handover('test',s.inputs())
        for _ in range(100):
            s.step(respond=False)
            if s.m.phase=='FAULT':break
        self.assertEqual(s.m.phase,'FAULT');self.assertIn('handover_not_observed',s.m.reason)
    def test_no_reacquire_after_handover(self):
        s=PilotSim();s.start();s.until('PILOT_CONTROL',limit=1700);before=list(s.actions);s.mode='ALTCTL'
        for _ in range(10):s.step()
        self.assertEqual(s.actions,before);self.assertIsNone(s.out['position'])
    def test_transport_requests_posctl_not_disarm(self):
        s=PilotSim();s.start();s.until('HOVER');s.m.handover('test',s.inputs())
        b=transport_tests.Boundary();r=b.runtime(s);b.request(r,'POSCTL')
        self.assertEqual(r.calls,[('/mavros/set_mode',{'base_mode':0,'custom_mode':'POSCTL'})])
    def test_existing_pilot_mode_exit_is_respected(self):
        s=PilotSim();s.start();s.until('HOVER');s.mode='ALTCTL';s.step()
        self.assertEqual(s.m.phase,'TAKEOVER');self.assertNotIn('LAND',s.actions)
    def test_traverse_permissions_still_required(self):
        s=Sim();cfg=dict(s.m.core.c,enable_traverse=False,pass_stage_approved=False)
        with self.assertRaises(ValueError):Mission(cfg,dict(s.options,pilot_landing_enabled=True))
    def test_failsafe_expectation_required(self):
        from test_position_goal_takeoff import PositionGoal
        c=PositionGoal().config();c['mission']['pilot_landing_enabled']=True
        c['expected_fcu_parameters']['COM_OBL_RC_ACT']=[4,0.]
        with self.assertRaises(ValueError):app.commissioning_deferrals(c,True)
        c['expected_fcu_parameters']['COM_OBL_RC_ACT']=[0,0.]
        self.assertTrue(app.commissioning_deferrals(c,True))
