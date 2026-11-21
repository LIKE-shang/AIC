#!/usr/bin/env python3
"""Run synthetic competition sequences. No --execute mode and no hardware IO."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from competition_rehearsal import Rehearsal, Sample, select_targets


def simulate(sequence, timing=None):
    flow = Rehearsal(sequence, timing)
    rows = []
    now = 0.
    for frame in range(6000):
        now = round(now + .05, 6)
        # Every observation and ACK here is deliberately fabricated. The tool
        # never imports rospy, opens a camera, loads a vendor library or flies.
        sample = Sample(time=now, hover_ready=True,
                        observed_mode='POSCTL' if flow.phase == 'HANDOVER' else 'OFFBOARD',
                        target_id=flow.target, observation_id=frame,
                        observation_time=now, target_valid=True, centred=True,
                        stable=True, spot_inside=True, at_requested_target=True,
                        ack_token=flow.token, ack_on=flow.light_intent,
                        ack_time=now)
        row = flow.tick(sample)
        if not rows or (row['phase'], row['target_id']) != (rows[-1]['phase'], rows[-1]['target_id']):
            rows.append(dict(time=now, **row))
        if row['phase'] in ('DONE', 'ABORT'):
            break
    return dict(execution='offline_rehearsal_only', synthetic_inputs=True,
                hardware_commands_sent=False, physical_hit_verified=False,
                flight_integration_implemented=False,
                selected_targets=list(sequence), transitions=rows,
                final=flow.output(), events=flow.events)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--targets', nargs='+', type=int)
    group.add_argument('--random-two', action='store_true')
    parser.add_argument('--seed', type=int)
    parser.add_argument('--config', type=Path, default=ROOT/'config/competition_rehearsal.json')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    cfg = json.loads(args.config.read_text(encoding='utf-8-sig'))
    if cfg.get('execution') != 'offline_rehearsal_only':
        raise ValueError('this tool supports offline rehearsal only')
    explicit = args.targets if args.targets is not None else (None if args.random_two else cfg['sequence'])
    sequence = select_targets(explicit, args.random_two, args.seed)
    report = simulate(sequence, cfg['timing'])
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        # Exclusive creation prevents silently replacing any existing result.
        with args.output.open('x', encoding='utf-8') as file:
            file.write(text + '\n')
    print(text)
    return 0 if report['final']['phase'] == 'DONE' else 1


if __name__ == '__main__':
    sys.exit(main())
