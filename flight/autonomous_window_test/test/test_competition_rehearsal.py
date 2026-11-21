"""Standard-library tests; no real flight, ROS connection or light output."""
import ast
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from competition_rehearsal import Rehearsal, Sample, select_targets
spec = importlib.util.spec_from_file_location('rehearse_competition', ROOT/'tools/rehearse_competition.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class Plant:
    def __init__(self, sequence=(1,)):
        self.flow = Rehearsal(sequence)
        self.now = 0.
        self.frame = 0
        self.last = None

    def step(self, **patch):
        self.now = round(self.now + .05, 6)
        self.frame += 1
        sample = Sample(time=self.now, hover_ready=True,
                        observed_mode='POSCTL' if self.flow.phase == 'HANDOVER' else 'OFFBOARD',
                        target_id=self.flow.target, observation_id=self.frame,
                        observation_time=self.now, target_valid=True, centred=True,
                        stable=True, at_requested_target=True, spot_inside=True,
                        ack_token=self.flow.token, ack_on=self.flow.light_intent,
                        ack_time=self.now)
        self.last = replace(sample, **patch)
        return self.flow.tick(self.last)

    def until(self, phase):
        for _ in range(2000):
            if self.flow.phase == phase:
                return
            out = self.step()
            if out['phase'] == 'ABORT':
                self.fail = out
                break
        raise AssertionError((phase, self.flow.output()))


class CompetitionTests(unittest.TestCase):
    def test_single_target(self):
        report = runner.simulate((1,))
        self.assertEqual(report['final']['phase'], 'DONE')
        self.assertEqual(report['final']['completed_targets'], [1])
        self.assertNotIn('MOVE', [r['phase'] for r in report['transitions']])
        self.assertFalse(report['hardware_commands_sent'])
        self.assertFalse(report['physical_hit_verified'])

    def test_every_ordered_pair(self):
        for first in (1, 2, 3):
            for second in (1, 2, 3):
                if first == second:
                    continue
                report = runner.simulate((first, second))
                self.assertEqual(report['final']['phase'], 'DONE')
                self.assertEqual(report['final']['completed_targets'], [first, second])
                for row in report['transitions']:
                    if row['phase'] == 'MOVE':
                        self.assertIs(row['simulated_light_request'], False)
                        self.assertTrue(row['simulated_light_off_confirmed'])

    def test_random_is_distinct_reproducible_and_balanced_over_seeds(self):
        results = {select_targets(random_two=True, seed=i) for i in range(100)}
        self.assertEqual(len(results), 6)
        self.assertEqual(select_targets(random_two=True, seed=7), select_targets(random_two=True, seed=7))
        with self.assertRaises(ValueError):
            select_targets((1,), True)

    def test_invalid_targets(self):
        for seq in [(), (0,), (4,), (1, 1), (True,), ('1',), (1.,)]:
            with self.subTest(seq=seq), self.assertRaises(ValueError):
                Rehearsal(seq)

    def test_invalid_timing(self):
        for val in [0, -1, float('nan'), float('inf'), True, '3']:
            with self.subTest(val=val), self.assertRaises(ValueError):
                Rehearsal(timing={'light_time_s': val})
        with self.assertRaises(ValueError):
            Rehearsal(timing={'made_up': 1})

    def test_startup_requires_current_off_ack(self):
        p = Plant()
        for _ in range(50):
            out = p.step(ack_token=None, ack_on=None, ack_time=None)
        self.assertEqual(out['phase'], 'ABORT')
        self.assertEqual(out['reason'], 'startup_off_not_confirmed')
        self.assertIs(out['simulated_light_request'], False)

    def test_wrong_target_does_not_light(self):
        p = Plant()
        p.until('ACQUIRE')
        for _ in range(230):
            out = p.step(target_id=2)
        self.assertEqual(out['phase'], 'ABORT')
        self.assertEqual(out['completed_targets'], [])
        self.assertFalse(any(e['phase'] == 'ON_WAIT' for e in p.flow.events))

    def test_no_accumulation_across_unstable_periods(self):
        p = Plant()
        p.until('ACQUIRE')
        for _ in range(15): p.step()
        p.step(stable=False)
        for _ in range(15): p.step()
        self.assertEqual(p.flow.phase, 'ACQUIRE')

    def test_duplicate_frames_do_not_advance_confirmation(self):
        p = Plant()
        p.until('ACQUIRE')
        p.step()
        old = p.last
        for _ in range(30):
            p.step(observation_id=old.observation_id, observation_time=old.observation_time)
        self.assertEqual(p.flow.phase, 'ACQUIRE')
        self.assertIs(p.flow.light_intent, False)

    def test_on_ack_timeout(self):
        p = Plant(); p.until('ON_WAIT')
        for _ in range(50): out = p.step(ack_token=-1)
        self.assertEqual(out['phase'], 'ABORT')
        self.assertEqual(out['reason'], 'light_on_ack_timeout')

    def test_ack_does_not_count_as_physical_hit(self):
        p = Plant(); p.until('LIGHTING')
        out = p.step(spot_inside=False)
        self.assertEqual(out['phase'], 'ABORT')
        self.assertFalse(out['physical_hit_verified'])
        self.assertEqual(out['completed_targets'], [])

    def test_light_loss_cases_switch_off(self):
        cases = [dict(target_valid=False), dict(centred=False), dict(stable=False),
                 dict(pose_ok=False), dict(pilot_override=True), dict(target_id=3),
                 dict(observation_time=-1), dict(observation_time=9999),
                 dict(observation_id=-1), dict(observed_mode='ALTCTL'),
                 dict(ack_token=-1)]
        for patch in cases:
            p = Plant(); p.until('LIGHTING')
            out = p.step(**patch)
            self.assertEqual(out['phase'], 'ABORT', patch)
            self.assertIs(out['simulated_light_request'], False)
            self.assertIsNone(out['simulated_move_to_target'])

    def test_off_ack_required_before_target_completion_and_move(self):
        p = Plant((1, 2)); p.until('OFF_WAIT')
        old_token = p.flow.token - 1
        for _ in range(50): out = p.step(ack_token=old_token, ack_on=False)
        self.assertEqual(out['phase'], 'ABORT')
        self.assertEqual(out['completed_targets'], [])
        self.assertIsNone(out['simulated_move_to_target'])

    def test_move_requires_arrival_and_new_target_evidence(self):
        p = Plant((1, 2)); p.until('MOVE')
        for _ in range(8): p.step(at_requested_target=False)
        self.assertEqual(p.flow.phase, 'MOVE')
        p.step(target_id=1)
        self.assertEqual(p.flow.phase, 'MOVE')
        p.step()
        self.assertEqual(p.flow.phase, 'ACQUIRE')
        self.assertIs(p.flow.light_intent, False)

    def test_move_rejects_changed_or_missing_off_feedback(self):
        for patch in [dict(ack_on=True), dict(ack_token=-1), dict(ack_time=-1)]:
            p = Plant((1, 2)); p.until('MOVE')
            out = p.step(**patch)
            self.assertEqual(out['phase'], 'ABORT')
            self.assertIsNone(out['simulated_move_to_target'])

    def test_handover_checks_off_feedback(self):
        p = Plant(); p.until('HANDOVER')
        out = p.step(ack_on=True)
        self.assertEqual(out['phase'], 'ABORT')

    def test_handover_requires_actual_mode_readback(self):
        p = Plant(); p.until('HANDOVER')
        for _ in range(20): p.step(observed_mode='OFFBOARD')
        self.assertEqual(p.flow.phase, 'HANDOVER')
        self.assertEqual(p.step()['phase'], 'DONE')

    def test_handover_timeout(self):
        p = Plant(); p.until('HANDOVER')
        for _ in range(90): out = p.step(observed_mode='OFFBOARD')
        self.assertEqual(out['phase'], 'ABORT')
        self.assertEqual(out['reason'], 'handover_not_observed')

    def test_abort_never_reacquires(self):
        p = Plant(); p.until('LIGHTING'); out = p.step(pilot_override=True)
        for _ in range(100): self.assertEqual(p.step(), out)

    def test_clock_errors_switch_off(self):
        for val in [float('nan'), float('inf'), -1, 9999]:
            p = Plant(); p.until('LIGHTING')
            self.assertEqual(p.step(time=val)['phase'], 'ABORT')

    def test_malformed_boolean_does_not_enable_light(self):
        p = Plant(); p.until('ACQUIRE')
        self.assertEqual(p.step(centred='true')['phase'], 'ABORT')

    def test_operator_measurements_remain_unfabricated(self):
        cfg = json.loads((ROOT/'config/competition_rehearsal.json').read_text(encoding='utf-8-sig'))
        data = cfg['operator_observations']
        self.assertEqual(data['laser_displacement_magnitude_m'], .07)
        self.assertEqual(data['target_bottom_height_m'], .9)
        self.assertEqual((data['pwm_off_percent'], data['pwm_on_percent']), (0, 100))
        self.assertTrue(all(v is None for v in cfg['measurements_not_supplied'].values()))

    def test_no_hardware_transport_imports(self):
        for path in (ROOT/'competition_rehearsal.py', ROOT/'tools/rehearse_competition.py'):
            tree = ast.parse(path.read_text(encoding='utf-8-sig'), feature_version=(3, 8))
            imports = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import): imports.extend(x.name.split('.')[0] for x in node.names)
                if isinstance(node, ast.ImportFrom): imports.append(node.module.split('.')[0])
            self.assertFalse(set(imports) & {'rospy', 'roslib', 'serial', 'ctypes', 'subprocess', 'socket'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
