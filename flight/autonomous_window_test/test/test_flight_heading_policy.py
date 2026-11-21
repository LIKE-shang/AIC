"""Current full competition: no standalone yaw-error abort or recovery gate."""
import math
import unittest
from dataclasses import replace
from test_second_window_return import ReturnSim
from window_pass.geometry import yaw


def drift(s,degrees):
    s.follow_yaw=False;h=s.m.heading+math.radians(degrees)
    s.f.q=(0.,0.,math.sin(h/2),math.cos(h/2))


class FlightHeadingPolicy(unittest.TestCase):
    def test_recorded_preparation_drift_no_longer_aborts(self):
        s=ReturnSim();s.start();s.until('RETURN_PREP')
        heading=s.m.heading;origin=s.m.origin;owner=s.m.core
        for degrees in (4.90,4.98,5.05,6.,11.):
            drift(s,degrees);s.step()
            self.assertEqual(s.m.phase,'RETURN_PREP',s.m.reason)
            self.assertEqual(s.m.heading,heading)
            self.assertIsNone(s.out['heading_limit_rad'])
            self.assertFalse(s.out['heading_error_gate_enabled'])
        s.follow_yaw=True;s.until('AUTO_LANDING',limit=4500)
        self.assertIs(s.m.core,owner);self.assertEqual(s.m.origin,origin)
        self.assertEqual(s.actions,['OFFBOARD','ARM','LAND'])

    def test_pass_hover_and_transfer_have_no_yaw_only_fault(self):
        for phase in ('PASS','POSTPASS','HOVER'):
            s=ReturnSim();s.start();s.until(phase);drift(s,11.);s.step()
            self.assertNotEqual(s.m.phase,'FAULT',s.m.reason)
            self.assertIsNone(s.out['heading_limit_rad'])

    def test_second_target_transfer_has_no_recovery_timer_or_heading_wait(self):
        s=ReturnSim();s.start()
        for _ in range(2000):
            s.step()
            if s.m.full_chain.index==1 and s.m.phase=='POSTPASS':break
        self.assertEqual(s.m.full_chain.index,1)
        drift(s,8.);s.until('HOVER')
        self.assertIsNone(s.m.full_chain.transfer_heading.started)
        self.assertFalse(s.out['full_chain']['transfer_heading_recovery']['enabled'])

    def test_faster_turn_still_uses_actual_yaw_and_bounded_reference_lead(self):
        s=ReturnSim();s.start();s.until('YAW_TURN');start=s.f.wall
        s.until('SEARCH')
        self.assertLess(s.f.wall-start,3.)
        self.assertEqual(s.m.return_journey.p['yaw_rate_rad_s'],math.radians(150))

    def test_epoch_and_pilot_takeover_apply_while_return_tracking_is_diagnostic(self):
        s=ReturnSim();s.start();s.until('RETURN_PREP');drift(s,6.)
        s.local_epoch+=1;s.step();self.assertEqual(s.m.phase,'FAULT')
        self.assertIn('epoch',s.m.reason)
        s=ReturnSim();s.start();s.until('RETURN_PREP');drift(s,6.)
        s.mode='POSCTL';s.step();self.assertEqual(s.m.phase,'TAKEOVER')
        s=ReturnSim();s.start();s.until('RETURN_PREP')
        s.update(False);p=s.m.core.pose
        s.m.core.pose=replace(p,p=(p.p[0],p.p[1]+.3,p.p[2]))
        s.out=s.m.tick(s.inputs())
        self.assertEqual(s.m.phase,'RETURN_PREP',s.m.reason)
        self.assertFalse(s.out['tracking']['tracking_error_abort_enabled'])
        self.assertGreater(s.out['tracking']['error_norm_m'],.12)
