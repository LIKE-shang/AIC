"""Synthetic integration with existing Mission; does not modify its runtime."""
from dataclasses import replace
import unittest
from test_position_goal_takeoff import GoalSim
from mission import Mission
from competition_rehearsal import Rehearsal, Sample


class HoverPlant(GoalSim):
    def __init__(self):
        super().__init__()
        self.options.update(pilot_landing_enabled=True, takeoff_position_goal_enabled=True,
                            vendor_takeoff_arrival_enabled=False, hover_time_s=10.)
        self.m = Mission(self.m.core.c, self.options, experiment='takeoff_land')
        self.f.ctrl = self.m.core
        self.out = None
        for _ in range(30): self.update()

    def step(self, follow=True, respond=True):
        out = super().step(follow, respond)
        if respond and out.get('action') == 'POSCTL': self.mode = 'POSCTL'
        return out


class ExistingFlightRehearsal(unittest.TestCase):
    def run_flow(self, lose_spot=False):
        plant = HoverPlant(); plant.start(); plant.until('HOVER')
        flow = Rehearsal((1,))
        handover_sent = False
        for frame in range(800):
            flight = plant.step()
            row = flow.tick(Sample(
                time=plant.f.wall, hover_ready=plant.m.phase == 'HOVER',
                observed_mode=plant.mode, target_id=1, observation_id=frame,
                observation_time=plant.f.wall, target_valid=True, centred=True,
                stable=True, spot_inside=not lose_spot,
                ack_token=flow.token, ack_on=flow.light_intent, ack_time=plant.f.wall))
            if row['phase'] in ('HANDOVER', 'ABORT') and not handover_sent:
                self.assertIs(row['simulated_light_request'], False)
                if row['phase'] == 'HANDOVER':
                    self.assertTrue(row['simulated_light_off_confirmed'])
                flight = plant.m.handover('offline_competition_test', plant.inputs())
                self.assertEqual(flight['action'], 'POSCTL')
                plant.actions.append('POSCTL')
                plant.mode = 'POSCTL'
                handover_sent = True
            if plant.m.phase == 'PILOT_CONTROL':
                break
        return plant, flow

    def test_single_target_reuses_takeoff_and_observed_pilot_handover(self):
        plant, flow = self.run_flow()
        self.assertEqual(plant.actions, ['OFFBOARD', 'ARM', 'POSCTL'])
        self.assertEqual(plant.m.phase, 'PILOT_CONTROL')
        self.assertTrue(plant.armed)
        self.assertAlmostEqual(plant.m.target_z - plant.m.origin[2], .9)
        self.assertEqual(flow.phase, 'DONE')
        self.assertEqual(flow.completed, [1])
        self.assertEqual(plant.f.p[:2], plant.m.origin[:2])

    def test_lost_spot_does_not_complete_and_returns_to_pilot(self):
        plant, flow = self.run_flow(lose_spot=True)
        self.assertEqual(flow.phase, 'ABORT')
        self.assertEqual(flow.completed, [])
        self.assertIs(flow.light_intent, False)
        self.assertEqual(plant.m.phase, 'PILOT_CONTROL')
        self.assertNotIn('LAND', plant.actions)

    def test_existing_pilot_takeover_remains_terminal(self):
        plant = HoverPlant(); plant.start(); plant.until('HOVER')
        plant.mode = 'ALTCTL'; plant.step()
        self.assertEqual(plant.m.phase, 'TAKEOVER')
        actions = list(plant.actions)
        plant.mode = 'OFFBOARD'
        for _ in range(20): plant.step()
        self.assertEqual(plant.actions, actions)
        self.assertIsNone(plant.out['position'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
