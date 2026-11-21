"""ALTCTL ground preparation preserves freshness and in-flight pilot takeover."""
import unittest
from types import SimpleNamespace as NS
from hover_pose_guard import guarded_bridge
from test_mission import Sim


class ReadinessParent:
    def __init__(self):
        self.reason = 'STARTUP_MODE_NOT_MANUAL'
        self.real_published = 0
        self.state = NS(connected=True, armed=False, mode='ALTCTL')
        self.gate = NS(last_stamp_ns=100)
        self.gm = NS(MAX_FUTURE_NS=5, MAX_AGE_NS=20)

    def _real_readiness(self, now, wall):
        return self.reason

    def _fresh_state(self, now, wall):
        return self.state


class AltctlPreparationTests(unittest.TestCase):
    def bridge(self, mode='ALTCTL'):
        return guarded_bridge(NS(ContinuousPoseBridge=ReadinessParent),
                              Sim().m.core.c, startup_mode=mode)()

    def test_altctl_ground_runs_original_lifecycle(self):
        s = Sim(); s.mode = 'ALTCTL'
        for _ in range(30): s.update()
        s.start(); s.until('DONE')
        self.assertEqual(s.actions, ['OFFBOARD', 'ARM', 'LAND'])

    def test_altctl_in_flight_is_still_pilot_takeover(self):
        s = Sim(); s.start(); s.until('SEARCH')
        s.mode = 'ALTCTL'; s.step()
        self.assertEqual(s.m.phase, 'TAKEOVER')
        self.assertIsNone(s.out['position'])

    def test_bridge_accepts_fresh_disarmed_altctl(self):
        self.assertEqual(self.bridge()._real_readiness(100, 1), '')

    def test_bridge_preserves_original_failures(self):
        b = self.bridge()
        for reason in ('CLOSED', 'GRAPH_CHECK_STALE_BEFORE_START', 'LIO_GUARD_FAULT',
                       'LIO_GUARD_CLOCK_JUMP', 'WAITING_FRESH_INPUT',
                       'MAVROS_STATE_NOT_FRESH', 'STARTUP_ARMED_REFUSED'):
            b.reason = reason
            self.assertEqual(b._real_readiness(100, 1), reason)

    def test_bridge_rechecks_state_and_input_after_mode_branch(self):
        for state, stamp, expected in (
                (None, 100, 'MAVROS_STATE_NOT_FRESH'),
                (NS(connected=False), 100, 'MAVROS_NOT_CONNECTED'),
                (NS(connected=True, armed=True), 100, 'STARTUP_ARMED_REFUSED'),
                (NS(connected=True, armed=False, mode='OFFBOARD'), 100, 'STARTUP_MODE_NOT_ALTCTL'),
                (NS(connected=True, armed=False, mode='ALTCTL'), None, 'WAITING_INPUT'),
                (NS(connected=True, armed=False, mode='ALTCTL'), 79, 'WAITING_FRESH_INPUT'),
                (NS(connected=True, armed=False, mode='ALTCTL'), 106, 'WAITING_FRESH_INPUT')):
            b = self.bridge(); b.state = state; b.gate.last_stamp_ns = stamp
            self.assertEqual(b._real_readiness(100, 1), expected)

    def test_default_bridge_still_uses_original_mode(self):
        self.assertEqual(self.bridge(None)._real_readiness(100, 1), 'STARTUP_MODE_NOT_MANUAL')

    def test_ground_preparation_requires_manual_altctl_selection(self):
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        source = (root/'scripts/full_chain_ground.py').read_text()
        self.assertIn('manually select ALTCTL before starting the vision-pose bridge', source)
        self.assertNotIn('mavros_msgs.srv import SetMode', source)
        self.assertNotIn("ServiceProxy('/mavros/set_mode'", source)


    def test_explicit_altctl_rejects_manual_accepted_by_parent(self):
        b = self.bridge(); b.reason = ''; b.state.mode = 'MANUAL'
        self.assertEqual(b._real_readiness(100, 1), 'STARTUP_MODE_NOT_ALTCTL')

    def test_mode_is_rechecked_until_first_actual_publication(self):
        b = self.bridge()
        self.assertEqual(b._real_readiness(100, 1), '')
        # A ready advertisement is not a real publication. A change while the
        # TCPROS connection is pending must still enforce the selected policy.
        b.reason = ''; b.state.mode = 'MANUAL'
        self.assertEqual(b.real_published, 0)
        self.assertEqual(b._real_readiness(100, 1), 'STARTUP_MODE_NOT_ALTCTL')

    def test_started_bridge_keeps_pose_during_mode_and_arming_changes(self):
        for mode, armed in (('OFFBOARD', True), ('ALTCTL', True), ('MANUAL', False)):
            with self.subTest(mode=mode, armed=armed):
                b = self.bridge(); b.real_published = 1; b.reason = ''
                b.state.mode = mode; b.state.armed = armed
                def no_startup_recheck(*args):
                    self.fail('startup-only state gate must not run after real publication')
                b._fresh_state = no_startup_recheck
                self.assertEqual(b._real_readiness(100, 1), '')

    def test_started_bridge_preserves_parent_time_and_lio_failures(self):
        b = self.bridge(); b.real_published = 1
        for reason in ('CLOSED', 'LIO_GUARD_CLOCK_JUMP', 'LIO_GUARD_FAULT',
                       'WAITING_INPUT', 'WAITING_FRESH_INPUT'):
            with self.subTest(reason=reason):
                b.reason = reason
                self.assertEqual(b._real_readiness(100, 1), reason)

    def test_default_policy_keeps_parent_manual_acceptance(self):
        b = self.bridge(None); b.reason = ''; b.state.mode = 'MANUAL'
        self.assertEqual(b._real_readiness(100, 1), '')

    def test_parent_failure_precedes_explicit_startup_mode(self):
        b = self.bridge(); b.state.mode = 'MANUAL'
        for reason in ('CLOSED', 'GRAPH_CHECK_STALE_BEFORE_START',
                       'LIO_GUARD_CLOCK_JUMP', 'MAVROS_STATE_NOT_FRESH'):
            with self.subTest(reason=reason):
                b.reason = reason
                self.assertEqual(b._real_readiness(100, 1), reason)


if __name__ == '__main__': unittest.main()
