#!/usr/bin/env python3
"""Check straight-run waypoints against an occupancy map before driving.

Reads a map_server style YAML + PGM (no ROS, no numpy) and reports, for every
waypoint and for the straight segments between consecutive waypoints, whether
the cells are free.  A chassis that must never rotate drives straight lines
between waypoints, so the segments matter as much as the endpoints.

Exit code 0 when every sampled cell is free, 1 otherwise.
"""
import argparse
import math
import os
import sys


def read_pgm(path):
    with open(path, 'rb') as handle:
        data = handle.read()
    index, tokens = 0, []
    while len(tokens) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index >= len(data):
            raise ValueError('truncated PGM header')
        if data[index:index + 1] == b'#':
            end = data.find(b'\n', index)
            if end < 0:
                raise ValueError('unterminated PGM comment')
            index = end + 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    magic, width, height, maxval = tokens
    if magic != b'P5' or maxval != b'255':
        raise ValueError('only 8-bit binary PGM (P5, maxval 255) supported')
    if data[index:index + 2] == b'\r\n':
        index += 2
    elif data[index:index + 1].isspace():
        index += 1
    else:
        raise ValueError('missing PGM raster separator')
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise ValueError('invalid PGM dimensions')
    pixels = data[index:index + width * height]
    if len(pixels) != width * height:
        raise ValueError('truncated PGM: ' + path)
    return width, height, pixels


def default_map_yaml():
    """Locate fast_lio's scans.yaml the same way slam_location.launch does.

    ``slam_location.launch`` passes ``$(find fast_lio)/PCD/scans.yaml`` to
    map_server.  Prefer rospack when a catkin environment is sourced, then
    fall back to the workspace layouts this robot actually uses.
    """
    try:
        import subprocess
        found = subprocess.run(['rospack', 'find', 'fast_lio'],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                               timeout=5).stdout.decode().strip()
        if found:
            candidate = os.path.join(found, 'PCD', 'scans.yaml')
            if os.path.exists(candidate):
                return candidate
    except (OSError, subprocess.SubprocessError):
        pass
    here = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    for candidate in (os.path.join(here, 'FAST_LIO', 'PCD', 'scans.yaml'),
                      os.path.join(here, 'fast_lio', 'PCD', 'scans.yaml'),
                      '/home/abot/abot_ws/src/FAST_LIO/PCD/scans.yaml'):
        if os.path.exists(candidate):
            return candidate
    return ''


def read_map_yaml(path):
    """Return (doc, resolved_image_path, note).

    ``note`` is non-empty when the yaml's declared image path was missing and
    a same-named neighbour was used instead.  That substitution changes the
    map extent, so the caller must surface it rather than trust the verdict.
    """
    try:
        import yaml
    except ImportError:
        raise SystemExit('PyYAML required')
    with open(path, 'r') as handle:
        doc = yaml.safe_load(handle)
    declared = doc['image']
    image = declared
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    note = ''
    if not os.path.exists(image):
        alt = os.path.join(os.path.dirname(os.path.abspath(path)),
                           os.path.basename(declared))
        if os.path.exists(alt):
            note = ('yaml declares %s which does not exist here; used the '
                    'same-named neighbour %s instead.  Its extent may differ '
                    'from what the robot loads.' % (declared, alt))
            image = alt
        else:
            raise SystemExit('map image missing: %s (declared in %s)'
                             % (declared, path))
    return doc, image, note


class Occupancy(object):
    FREE, OCCUPIED, UNKNOWN = 0, 100, -1

    def __init__(self, doc, image):
        self.res = float(doc['resolution'])
        self.ox, self.oy = [float(v) for v in doc['origin'][:2]]
        self.yaw = float(doc['origin'][2])
        if not all(math.isfinite(v) for v in (self.res, self.ox, self.oy, self.yaw)) or self.res <= 0:
            raise ValueError('invalid map geometry')
        self.negate = int(doc.get('negate', 0))
        self.occ_th = float(doc.get('occupied_thresh', 0.65))
        self.free_th = float(doc.get('free_thresh', 0.196))
        self.width, self.height, self.pixels = read_pgm(image)

    def value(self, wx, wy):
        dx, dy = wx - self.ox, wy - self.oy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        col = int(math.floor((c * dx + s * dy) / self.res))
        row_from_bottom = int(math.floor((-s * dx + c * dy) / self.res))
        if not (0 <= col < self.width and 0 <= row_from_bottom < self.height):
            return None
        row = self.height - 1 - row_from_bottom  # PGM row 0 is top = max y
        pixel = self.pixels[row * self.width + col] / 255.0
        # negate: 0 means white is free, which is what map_saver writes:
        # 254 free, 0 occupied, 205 unknown.  These yaml files pick
        # free_thresh 0.196 precisely because 1 - 205/255 == 0.196 puts the
        # unknown grey just outside the free band.
        occ = pixel if self.negate else 1.0 - pixel
        if occ > self.occ_th:
            return self.OCCUPIED
        if occ < self.free_th:
            return self.FREE
        return self.UNKNOWN

    def footprint_clear(self, wx, wy, radius):
        # Conservative circle: include all cells intersecting the footprint.
        dx, dy = wx - self.ox, wy - self.oy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        mx, my = c * dx + s * dy, -s * dx + c * dy
        margin = radius + self.res / math.sqrt(2.0)
        for ix in range(math.floor((mx-margin)/self.res), math.floor((mx+margin)/self.res)+1):
            for iy in range(math.floor((my-margin)/self.res), math.floor((my+margin)/self.res)+1):
                cx, cy = (ix+0.5)*self.res, (iy+0.5)*self.res
                if math.hypot(cx-mx, cy-my) <= margin:
                    if self.value(self.ox+c*cx-s*cy, self.oy+s*cx+c*cy) != self.FREE:
                        return False
        return True

    def label(self, value):
        return {self.FREE: 'free', self.OCCUPIED: 'OCCUPIED',
                self.UNKNOWN: 'unknown', None: 'OUTSIDE'}[value]

    def bounds(self):
        return (self.ox, self.ox + self.width * self.res,
                self.oy, self.oy + self.height * self.res)


def sample_segment(start, end, step):
    span = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(span / step)))
    for index in range(count + 1):
        t = index / float(count)
        yield (start[0] + (end[0] - start[0]) * t,
               start[1] + (end[1] - start[1]) * t)


def resolve_waypoints(entries, start_xy, start_yaw):
    out = []
    for index, entry in enumerate(entries):
        keys = set(entry) & {'map', 'relative'}
        if len(keys) != 1:
            raise SystemExit('waypoint %d needs exactly one of map/relative' % index)
        key = keys.pop()
        a, b = entry[key]
        if key == 'map':
            out.append((float(a), float(b)))
        else:
            if start_xy is None:
                raise SystemExit('relative waypoint %d needs --start-x/--start-y/'
                                 '--start-yaw' % index)
            c, s = math.cos(start_yaw), math.sin(start_yaw)
            out.append((start_xy[0] + c * a - s * b,
                        start_xy[1] + s * a + c * b))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', default='',
                        help='map_server yaml (default: slam_navigation/PCD/scans.yaml)')
    parser.add_argument('--config', required=True, help='straight_run.yaml')
    parser.add_argument('--start-x', type=float, required=True)
    parser.add_argument('--start-y', type=float, required=True)
    parser.add_argument('--start-yaw', type=float, required=True)
    parser.add_argument('--robot-radius', type=float, required=True,
                        help='measured circumscribed chassis radius plus safety clearance, metres')
    parser.add_argument('--step', type=float, default=0.05,
                        help='segment sampling step in metres')
    args = parser.parse_args(argv)
    if not all(math.isfinite(v) for v in (args.start_x, args.start_y, args.start_yaw, args.step, args.robot_radius)) or args.step <= 0 or args.robot_radius <= 0:
        parser.error('finite pose and positive step/robot-radius required')

    try:
        import yaml
    except ImportError:
        raise SystemExit('PyYAML required')
    with open(args.config, 'r') as handle:
        cfg = yaml.safe_load(handle) or {}
    entries = cfg.get('waypoints') or []
    if not entries:
        raise SystemExit('config has no waypoints')

    start_xy = None
    if args.start_x is not None and args.start_y is not None:
        start_xy = (args.start_x, args.start_y)
    points = resolve_waypoints(entries, start_xy, args.start_yaw)

    map_yaml = args.map_yaml or default_map_yaml()
    if not map_yaml:
        raise SystemExit('cannot locate fast_lio/PCD/scans.yaml; pass '
                         '--map-yaml explicitly (on the robot: '
                         '$(rospack find fast_lio)/PCD/scans.yaml)')
    doc, image, note = read_map_yaml(map_yaml)
    occ = Occupancy(doc, image)
    x0, x1, y0, y1 = occ.bounds()
    print('map yaml : %s' % map_yaml)
    print('map image: %s  (%dx%d cells @ %.3f m)' % (image, occ.width,
                                                     occ.height, occ.res))
    print('extent   : x %.2f..%.2f  y %.2f..%.2f' % (x0, x1, y0, y1))
    if note:
        print()
        print('WARNING  : %s' % note)
        print('WARNING  : verify with the robot-side run before driving.')
    if start_xy:
        print('start    : (%.3f, %.3f) yaw %.3f rad' % (start_xy[0], start_xy[1],
                                                        args.start_yaw))

    failures = 1 if note else 0
    step = min(args.step, occ.res / 2.0)
    radius = args.robot_radius + step / 2.0
    previous = start_xy
    origin = 'start' if start_xy else None
    for index, point in enumerate(points):
        value = occ.value(point[0], point[1])
        ok = occ.footprint_clear(point[0], point[1], radius)
        failures += 0 if ok else 1
        if not ok:
            print('  footprint intersects occupied/unknown/outside cells')
        print('waypoint %d (%7.3f, %7.3f) : %s' % (index, point[0], point[1],
                                                  occ.label(value)))
        if previous is not None:
            samples = list(sample_segment(previous, point, step))
            bad = [p for p in samples if not occ.footprint_clear(p[0], p[1], radius)]
            if bad:
                failures += 1
                print('  segment %s->%d blocked: %d/%d samples, first at '
                      '(%.3f, %.3f) %s'
                      % (origin if origin else str(index - 1), index, len(bad),
                         len(samples), bad[0][0], bad[0][1],
                         occ.label(occ.value(*bad[0]))))
            else:
                print('  segment %s->%d: clear (%d samples)'
                      % (origin if origin else str(index - 1), index,
                         len(samples)))
        else:
            print('  segment before waypoint %d: unverified (no start given)'
                  % index)
        previous = point
        origin = None

    print()
    print('RESULT: %s' % ('ALL CLEAR' if failures == 0 else
                          '%d PROBLEM(S) - do not drive this path' % failures))
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
