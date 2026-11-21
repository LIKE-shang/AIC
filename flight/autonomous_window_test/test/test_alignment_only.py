"""Stop at alignment, with no traversal permission or commit call."""
import unittest,copy
from unittest.mock import Mock
from test_airborne_geometry import DynamicSim
from mission import Mission
from test_auto_transport import app

class AlignOnlySim(DynamicSim):
    def __init__(self,target=True):
        super().__init__(target)
        cfg=copy.deepcopy(self.m.core.c)
        cfg.update(enable_traverse=False,pass_stage_approved=False)
        self.options.update(takeoff_position_goal_enabled=True,pilot_landing_enabled=True)
        self.m=Mission(cfg,self.options,airborne_geometry=True,experiment='align_only')
        self.f.ctrl=self.m.core
        for _ in range(30):self.update()
    def update(self,follow=True):
        if follow and self.out and self.out['position'] is not None:
            old=self.out;limited=tuple(p+max(-.01,min(.01,t-p)) for p,t in zip(self.f.p,old['position']))
            self.out=dict(old,position=limited);super().update(follow);self.out=old
        else:super().update(follow)
    def step(self,follow=True,respond=True):
        out=super().step(follow,respond)
        if out['action']=='POSCTL' and respond:self.mode='POSCTL'
        return out

class AlignmentOnly(unittest.TestCase):
    def test_alignment_completes_and_never_calls_commit(self):
        s=AlignOnlySim();s.m.core.request_pass=Mock(side_effect=AssertionError('must never commit'))
        s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.out['alignment_completed']);self.assertTrue(s.out['handover_confirmed'])
        self.assertFalse(s.out['window_traversal_permitted'])
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
        self.assertNotIn('PASS',[e['new'] for e in s.m.events]);s.m.core.request_pass.assert_not_called()
        self.assertLess(abs(s.f.p[0]),.01)
        self.assertTrue(s.armed);self.assertIsNone(s.out['position'])
    def test_no_target_timeout_hands_over_without_alignment_success(self):
        s=AlignOnlySim(False);s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.out['alignment_completed']);self.assertNotIn('LAND',s.actions)
    def test_never_accepts_pass_permissions(self):
        s=AlignOnlySim()
        for flag in ('enable_traverse','pass_stage_approved'):
            with self.subTest(flag=flag),self.assertRaisesRegex(ValueError,'forbids traversal'):
                Mission(dict(s.m.core.c,**{flag:True}),s.options,airborne_geometry=True,experiment='align_only')
    def test_cannot_skip_installation_or_motion_gates(self):
        s=AlignOnlySim()
        with self.assertRaises(ValueError):Mission(dict(s.m.core.c,camera_verified=False),s.options,airborne_geometry=True,experiment='align_only')
        with self.assertRaises(ValueError):Mission(dict(s.m.core.c,enable_motion=False),s.options,airborne_geometry=True,experiment='align_only')
    def test_commissioning_takeoff_exception_does_not_expand(self):
        s=AlignOnlySim()
        cfg={'experiment':'align_only','mission':s.options,'expected_fcu_parameters':{'MPC_Z_VEL_MAX_UP':[0,.4],'MPC_ACC_UP_MAX':[0,.4],'COM_OBL_RC_ACT':[0,0.]}}
        self.assertFalse(app.commissioning_deferrals(cfg,False))
        with self.assertRaisesRegex(ValueError,'takeoff-only'):app.commissioning_deferrals(cfg,True)
    def test_untrusted_pose_stops_without_blind_handover(self):
        s=AlignOnlySim();s.start();s.until('ALIGN');s.healthy['estimator_ok']=False;s.step()
        self.assertEqual(s.m.phase,'FAULT');self.assertNotIn('POSCTL',s.actions)
    def test_requires_live_geometry_and_pilot_policy(self):
        s=AlignOnlySim()
        with self.assertRaises(ValueError):Mission(s.m.core.c,s.options,airborne_geometry=False,experiment='align_only')
        with self.assertRaises(ValueError):Mission(s.m.core.c,dict(s.options,pilot_landing_enabled=False),airborne_geometry=True,experiment='align_only')
