#!/usr/bin/env python3
"""Offline profile builder. Creates files only; never connects to or starts ROS."""
import argparse
import copy
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'window_pass_test/src')]
from hover_target import settings
from visual_calibration import CalibrationSession


def build(base, mode, scene_key, seed=None):
    config = copy.deepcopy(base)
    if mode not in ('learn', 'reuse'):
        raise ValueError('explicit learn/reuse mode required')
    config['hover_target'].update(paced_visual_servo=False, bounded_visual_aim=False, timed_visual_aim=False, visual_target_hold=False, fixed_position_timed=False, visual_servo_enabled=True,
        visual_calibration_mode=mode, trigger_policy='hover_timed',
        require_stable_target_before_light=True, exposure_s=3.)
    settings(config['hover_target'])
    config['visual_calibration'] = dict(scene_key=scene_key)
    if seed is not None:
        config['visual_calibration']['seed'] = copy.deepcopy(seed)
    CalibrationSession(config, config.get('site_identity', {}))
    return config


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, required=True)
    p.add_argument('--mode', choices=('learn', 'reuse'), required=True)
    p.add_argument('--scene-key', required=True, help='Same physical start, target layout and camera mount')
    p.add_argument('--records', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    config = build(json.loads(a.base.read_text(encoding='utf-8-sig')), a.mode, a.scene_key,
                   json.loads(a.records.read_text(encoding='utf-8-sig')) if a.records else None)
    with a.output.open('x', encoding='utf-8') as stream:
        json.dump(config, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print('PROFILE_WRITTEN_ONLY=' + str(a.output))


if __name__ == '__main__':
    main()
