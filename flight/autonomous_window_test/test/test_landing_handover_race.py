"""Landing telemetry order regression, with fake FCU/bridge inputs only."""
import copy
import json
import types
import unittest
from dataclasses import replace
from pathlib import Path

import test_auto_transport as transport
from test_second_window_return import ReturnSim


class LandingHandoverRace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        s = ReturnSim()
        s.start()
        s.until('LANDING_LIGHT_ON', limit=4500)
        s.step(respond=False)
        assert s.m.phase == 'LANDING_HANDOVER'
        assert s.out['action'] == 'LAND'
        s.m.command_finished('LAND', True)
        cls.ready = s

    def sim(self):
        return copy.deepcopy(self.ready)

    def sample(self, s, raw=4, mode='OFFBOARD', armed=True, dz=0., **changes):
        s.mode = mode
        s.armed = armed
        s.f.p = (s.f.p[0], s.f.p[1], s.f.p[2] + dz)
        s.update(False)
        # A distinct ExtendedState message follows the simulated pose/state.
        s.f.ros += .001
        s.f.wall += .001
        s.m.core.extended_state(s.f.ros, raw == 2, s.f.ros, s.f.wall)
        i = replace(s.inputs(), landed=raw == 1, **changes)
        i.landed_state = raw
        s.out = s.m.tick(i)
        return s.out

    def assert_lit(self, s, request):
        self.assertTrue(s.out['landing_illumination']['request_on'], s.m.reason)
        self.assertEqual(s.out['landing_illumination']['request_id'], request)
        self.assertIsNone(s.out['position'])
        self.assertIsNone(s.out['action'])

    def test_recorded_landing_before_mode_keeps_light_until_touchdown(self):
        evidence = json.loads((Path(__file__).parent / 'fixtures/landing_158619422586.json').read_text())
        self.assertGreater(evidence['mode_feedback_delay_after_landing_s'], .6)
        s = self.sim()
        request = s.m.return_journey.request_id
        # Replay the recorded gap at control-loop cadence, including descent
        # through the normal flight floor while mode feedback is still old.
        for _ in range(int(evidence['mode_feedback_delay_after_landing_s'] / .051) + 1):
            self.sample(s, dz=-.016)
            self.assertEqual(s.m.phase, 'LANDING_HANDOVER', s.m.reason)
            self.assertFalse(s.m.return_journey.handover_observed)
            self.assert_lit(s, request)
        self.assertLess(s.m.full_height_bounds(s.f.p)[0], s.m.o['minimum_bottom_height_m'])
        self.sample(s, mode='AUTO.LAND')
        self.assertEqual(s.m.phase, 'AUTO_LANDING', s.m.reason)
        self.assertTrue(s.m.return_journey.handover_observed)
        self.assert_lit(s, request)
        s.until('DONE')
        self.assertTrue(s.m.return_journey.complete)
        self.assertTrue(s.m.return_journey.off_ack)
        self.assertEqual(s.actions, ['OFFBOARD', 'ARM', 'LAND'])

    def test_mode_before_landing_state_bypasses_obsolete_hold_height(self):
        s = self.sim()
        request = s.m.return_journey.request_id
        for _ in range(14):
            self.sample(s, raw=2, mode='AUTO.LAND', dz=-.016)
            self.assertEqual(s.m.phase, 'AUTO_LANDING', s.m.reason)
            self.assert_lit(s, request)
        self.assertLess(s.m.full_height_bounds(s.f.p)[0], s.m.o['minimum_bottom_height_m'])

    def test_neither_observation_keeps_hold_and_requires_mode_confirmation(self):
        s = self.sim()
        out = self.sample(s, raw=2)
        self.assertEqual(s.m.phase, 'LANDING_HANDOVER')
        self.assertIsNotNone(out['position'])
        self.assertFalse(s.m.return_journey.handover_observed)
        s.until('FAULT')
        self.assertEqual(s.m.reason, 'landing_AUTO_LAND_not_observed')

    def test_landing_state_does_not_replace_mode_confirmation_or_reset_timeout(self):
        s = self.sim()
        request = s.m.return_journey.request_id
        while s.f.wall - s.m.entered <= s.m.o['mode_timeout_s'] + .051:
            self.sample(s)
            if s.m.phase != 'LANDING_HANDOVER':
                break
            self.assert_lit(s, request)
        self.assertEqual(s.m.phase, 'LANDING_LIGHT_OFF', s.m.reason)
        self.assertEqual(s.m.reason, 'landing_AUTO_LAND_not_observed')
        self.assertFalse(s.m.return_journey.handover_observed)
        self.assertFalse(s.out['landing_illumination']['request_on'])

    def test_other_extended_states_cannot_authorize_handover_descent(self):
        for state in (None, 0, 1, 3):
            with self.subTest(state=state):
                s = self.sim()
                self.sample(s, raw=state)
                self.assertEqual(s.m.phase, 'FAULT')
                self.assertEqual(s.m.fault_snapshot['reason'], 'return_unexpected_disarm_or_landed')

    def test_disarm_before_confirmed_auto_landing_still_faults(self):
        s = self.sim()
        self.sample(s, armed=False)
        self.assertEqual(s.m.phase, 'FAULT')
        self.assertFalse(s.out['landing_illumination']['request_on'])

    def test_unacknowledged_landing_light_cannot_authorize_descent(self):
        s = self.sim()
        s.m.return_journey.on_ack = False
        self.sample(s)
        self.assertEqual(s.m.phase, 'FAULT')

    def test_rejected_land_command_is_not_hidden_by_landing_state(self):
        s = self.sim()
        s.m.command_finished('LAND', False, result_code=2)
        self.sample(s)
        self.assertEqual(s.m.phase, 'FAULT')
        self.assertEqual(s.m.fault_snapshot['reason'], 'LAND_rejected_mav_result_2')

    def test_landing_signal_never_relaxes_return_flight_guard(self):
        s = self.sim()
        s.m.phase = 'RETURN_HOME'
        self.sample(s)
        self.assertEqual(s.m.phase, 'FAULT')
        self.assertEqual(s.m.fault_snapshot['reason'], 'return_unexpected_disarm_or_landed')

    def test_health_and_emergency_guards_remain_active_in_delayed_handover(self):
        for changes in (dict(state_fresh=False), dict(landed_fresh=False),
                        dict(rc_ok=False), dict(graph_ok=False), dict(estimator_ok=False),
                        dict(localization_epoch=1), dict(emergency=True)):
            with self.subTest(changes=changes):
                s = self.sim()
                self.sample(s, **changes)
                self.assertIn(s.m.phase, ('FAULT', 'TAKEOVER'))
                self.assertFalse(s.out['landing_illumination']['request_on'])

    def test_stale_pose_still_faults_in_delayed_handover(self):
        s = self.sim()
        s.m.core.streams['pose'] = (s.f.ros - 3., s.f.wall - 3., True)
        i = s.inputs()
        i.landed_state = 4
        s.m.core.airborne = False
        s.out = s.m.tick(i)
        self.assertEqual(s.m.phase, 'FAULT')
        self.assertEqual(s.m.fault_snapshot['reason'], 'pose_unavailable')

    def test_bridge_loss_still_stops_light_in_delayed_handover(self):
        s = self.sim()
        old = s.inputs().hover_laser
        self.sample(s, hover_laser=(s.f.wall - 3., old[1]))
        self.assertEqual(s.m.phase, 'LANDING_LIGHT_OFF')
        self.assertEqual(s.m.reason, 'landing_light_continuity_lost')

    def test_ground_and_disarm_without_auto_land_are_not_reported_as_success(self):
        s = self.sim()
        self.sample(s, raw=1, armed=False)
        self.assertEqual(s.m.phase, 'FAULT')
        self.assertFalse(s.m.return_journey.complete)
        self.assertFalse(s.out['landing_illumination']['request_on'])
        self.assertEqual(s.actions, ['OFFBOARD', 'ARM', 'LAND'])

    def test_runtime_preserves_raw_landing_enum_without_changing_airborne(self):
        s = self.sim()
        r = transport.Boundary().runtime(s)
        r.rows = {}
        s.f.ros += .001
        s.f.wall += .001
        r.now = lambda: (s.f.ros, s.f.wall)
        msg = types.SimpleNamespace(landed_state=4, header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: s.f.ros)))
        r.ext(msg)
        self.assertFalse(r.core.airborne)
        self.assertEqual(s.m.phase, 'LANDING_HANDOVER')
        state = types.SimpleNamespace(system_status=4)
        rc = types.SimpleNamespace(channels=[1500] * 6 + [1050])
        r.fresh = lambda topic, *args: {
            '/mavros/state': state, '/mavros/extended_state': msg, '/mavros/rc/in': rc}.get(topic)
        r.cfg = {'mission': {'companion_battery_monitor_enabled': False}}
        r.estimator_health = lambda: (True, {})
        r.graph_ok = True
        r.graph_wall = s.f.wall
        r.current_localization_identity = s.m.session_window.identity
        r.localization = types.SimpleNamespace(epoch=s.m.localization_epoch,
                                                health=lambda *args: (True, 'ok'))
        r.hover_laser_row = s.inputs().hover_laser
        i = transport.app.Runtime.inputs(r)
        self.assertEqual(i.landed_state, 4)
        self.assertFalse(i.landed)
        s.out = s.m.tick(i)
        self.assert_lit(s, s.m.return_journey.request_id)
        r.fresh = lambda *args: None
        self.assertIsNone(transport.app.Runtime.inputs(r).landed_state)


if __name__ == '__main__':
    unittest.main()
