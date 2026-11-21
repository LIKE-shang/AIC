#!/usr/bin/env python3
"""Replay/benchmark one RGB file with the real window algorithm; no ROS/hardware."""
import argparse
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from robot_vision_test.vision import detect_windows, annotate_windows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--config', type=Path, default=PACKAGE/'config/window_test.yaml')
    parser.add_argument('--output-dir', type=Path, default=PACKAGE/'test/artifacts')
    parser.add_argument('--runs', type=int, default=30)
    parser.add_argument('--threads', type=int, default=None,
                        help='Optional OpenCV worker count (otherwise leave runtime default).')
    args = parser.parse_args()
    if not 1 <= args.runs <= 10000 or (args.threads is not None and args.threads < 0):
        parser.error('runs must be 1..10000; threads must be nonnegative')
    if args.threads is not None:
        cv2.setNumThreads(args.threads)
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None:
        parser.error('Cannot read image: '+str(args.image))
    parameters = yaml.safe_load(args.config.read_text(encoding='utf-8-sig'))
    for _ in range(3):
        detect_windows(frame, parameters)
    milliseconds = []
    for _ in range(args.runs):
        start = time.perf_counter()
        found, edges = detect_windows(frame, parameters)
        milliseconds.append((time.perf_counter()-start)*1000.0)
    old = dict(parameters, checker_fallback=False)
    old_count = len(detect_windows(frame, old)[0])
    targets = [dict(method=t['method'], outline_kind=t['outline_kind'],
                    contour=t['contour'].reshape(4, 2).tolist(),
                    center_px=t['center'].tolist(), width_px=t['width'], height_px=t['height'],
                    area_px=t['area'], checker_support=t.get('checker_support')) for t in found]
    report = dict(scope='OFFLINE ONLY - no ROS, no camera, no hardware',
                  image=str(args.image.resolve()), opencv=cv2.__version__, numpy=np.__version__,
                  opencv_threads=cv2.getNumThreads(), image_shape=list(frame.shape), runs=args.runs,
                  nested_only_count=old_count, count=len(found), targets=targets,
                  algorithm_ms=dict(median=float(np.median(milliseconds)),
                                    p95=float(np.percentile(milliseconds, 95)),
                                    maximum=float(max(milliseconds))))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, pixels in [('window_checker_result.png', annotate_windows(frame, found)),
                         ('window_checker_edges.png', edges)]:
        if not cv2.imwrite(str(args.output_dir/name), pixels):
            raise IOError('Could not save '+name)
    text = json.dumps(report, indent=2, ensure_ascii=False)
    (args.output_dir/'window_checker_replay.json').write_text(text+'\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
