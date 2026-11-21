"""ROS-free shape algorithms, adapted from shape_detector.py's OpenCV pipeline.
All sizes are image pixels, NOT physical dimensions. No motion or hardware I/O.
"""
import math
from itertools import chain
import cv2
import numpy as np

WINDOW_DEFAULTS = dict(
    blur_kernel=5, canny_low=50, canny_high=150, epsilon_ratio=0.02,
    min_area=1800.0, max_area_fraction=0.9, min_side=25.0,
    min_aspect_ratio=1.0, max_aspect_ratio=3.5, max_angle_cosine=0.4,
    max_opposite_side_ratio=1.6, min_rectangularity=0.72,
    require_inner_frame=True, min_inner_area_ratio=0.30, max_inner_area_ratio=0.90,
    max_inner_center_offset=0.16, max_inner_aspect_difference=0.4,
    min_frame_gap=3.0, min_inner_parallel_cosine=0.94, border_margin=3,
    dedup_center_ratio=0.15,
    # A checkerboard frame often gives a clean quadrilateral for the opening,
    # but not one outer contour.  The fallback requires texture evidence in
    # all four side bands; it never disables the normal nested-frame checks.
    checker_fallback=True, checker_outer_scale=1.15,
    checker_min_area_fraction=0.08,
    checker_min_side_edge_fraction=0.18, checker_min_side_std=25.0,
    checker_dark_threshold=100, checker_light_threshold=140,
    checker_min_dark_fraction=0.12, checker_min_light_fraction=0.10,
    checker_min_cell_px=2, checker_max_cell_px=24,
    checker_min_alternation=0.25, checker_min_repeat=0.25,
    checker_texture_candidates=True, checker_texture_kernels=(7, 11),
    checker_texture_thresholds=(45.0, 55.0),
    checker_morph_open=7, checker_morph_close=5)
CIRCLE_DEFAULTS = dict(
    blur_kernel=5, canny_low=60, canny_high=160, min_area=450.0,
    min_radius=14.0, max_radius=160.0, min_circularity=0.80,
    min_circle_fill=0.78, max_radial_cv=0.13, min_axis_ratio=0.83,
    require_ring=False, min_ring_radius_ratio=0.25, max_ring_radius_ratio=0.90,
    concentric_center_ratio=0.22, dedup_center_ratio=0.32,
    border_margin=3, use_hough=False, hough_dp=1.2, hough_min_dist=25.0,
    hough_param1=160.0, hough_param2=30.0)


def checked_parameters(defaults, supplied):
    p = dict(defaults)
    p.update({k: supplied[k] for k in defaults if k in supplied})
    if not isinstance(p['blur_kernel'], int) or not 3 <= p['blur_kernel'] <= 51 or p['blur_kernel'] % 2 != 1:
        raise ValueError('blur_kernel must be an odd integer in [3, 51]')
    for key, value in p.items():
        if isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError('%s must be finite' % key)
    if not 0 <= p['canny_low'] < p['canny_high'] or p['min_area'] <= 0:
        raise ValueError('invalid Canny thresholds / min_area')
    if p['border_margin'] < 0 or not 0 < p['dedup_center_ratio'] <= 1:
        raise ValueError('invalid border / deduplication limits')
    if 'epsilon_ratio' in p:
        if not 0 < p['epsilon_ratio'] < 0.15:
            raise ValueError('epsilon_ratio must be in (0, 0.15)')
        if not 1 <= p['min_aspect_ratio'] <= p['max_aspect_ratio']:
            raise ValueError('aspect ratio is long side / short side, must be >= 1')
        if not 0 < p['min_inner_area_ratio'] < p['max_inner_area_ratio'] < 1:
            raise ValueError('invalid inner/outer frame area ratios')
        if not 0 < p['max_angle_cosine'] < 1 or p['max_opposite_side_ratio'] < 1:
            raise ValueError('invalid rectangle perspective filters')
        for key in ('max_area_fraction', 'min_rectangularity', 'max_inner_center_offset', 'min_inner_parallel_cosine'):
            if not 0 < p[key] <= 1:
                raise ValueError('invalid window threshold '+key)
        if p['min_side'] <= 0 or p['min_frame_gap'] <= 0 or p['max_inner_aspect_difference'] < 0:
            raise ValueError('invalid window frame geometry')
        for key in ('checker_fallback', 'checker_texture_candidates'):
            if type(p[key]) is not bool:
                raise ValueError(key+' must be boolean')
        if not 1.01 <= p['checker_outer_scale'] <= 2.0:
            raise ValueError('checker_outer_scale must be in [1.01, 2.0]')
        if not 0 < p['checker_min_area_fraction'] <= p['max_area_fraction']:
            raise ValueError('invalid checker_min_area_fraction')
        if not 0 < p['checker_min_side_edge_fraction'] <= 1 or p['checker_min_side_std'] < 0:
            raise ValueError('invalid checker side edge/std thresholds')
        if not 0 <= p['checker_dark_threshold'] < p['checker_light_threshold'] <= 255:
            raise ValueError('invalid checker intensity thresholds')
        if not 0 <= p['checker_min_dark_fraction'] <= 1 or not 0 <= p['checker_min_light_fraction'] <= 1:
            raise ValueError('invalid checker dark/light fractions')
        if (type(p['checker_min_cell_px']) is not int or type(p['checker_max_cell_px']) is not int or
                not 2 <= p['checker_min_cell_px'] <= p['checker_max_cell_px'] <= 64):
            raise ValueError('checker cell pixel range must be integers in [2, 64]')
        for key in ('checker_min_alternation', 'checker_min_repeat'):
            if not 0 < p[key] <= 1:
                raise ValueError('invalid checker periodicity threshold '+key)
        for key in ('checker_morph_open', 'checker_morph_close'):
            if type(p[key]) is not int or not 1 <= p[key] <= 51 or p[key] % 2 != 1:
                raise ValueError(key+' must be an odd integer in [1, 51]')
        for key in ('checker_texture_kernels', 'checker_texture_thresholds'):
            if not isinstance(p[key], (tuple, list)) or not 1 <= len(p[key]) <= 4:
                raise ValueError(key+' must have 1 to 4 values')
        if any(type(k) is not int or not 3 <= k <= 51 or k % 2 != 1
               for k in p['checker_texture_kernels']):
            raise ValueError('checker_texture_kernels must be odd integers in [3, 51]')
        if any(type(t) not in (int, float) or not math.isfinite(t) or not 0 < t <= 127.5
               for t in p['checker_texture_thresholds']):
            raise ValueError('checker texture std thresholds must be finite in (0, 127.5]')
    else:
        if not 0 < p['min_radius'] < p['max_radius']:
            raise ValueError('invalid circle radius limits')
        for key in ('min_circularity', 'min_circle_fill', 'min_axis_ratio'):
            if not 0 < p[key] <= 1:
                raise ValueError('%s must be in (0, 1]' % key)
        if not 0 < p['min_ring_radius_ratio'] < p['max_ring_radius_ratio'] < 1:
            raise ValueError('invalid ring radius ratios')
        if not 0 < p['max_radial_cv'] < 1 or not 0 < p['concentric_center_ratio'] <= 1:
            raise ValueError('invalid circle radial/concentric filters')
        if any(p[k] <= 0 for k in ('hough_dp','hough_min_dist','hough_param1','hough_param2')):
            raise ValueError('Hough parameters must be positive')
    return p


def depth_to_gray(depth, encoding, near=0.2, far=5.0, uint16_scale=0.001):
    """Explicit fixed metric range, never pass uint16/float directly to Canny.
    16UC1 defaults to millimetres; operator must verify the driver's depth unit.
    32FC1 convention is metres. Invalid/zero/NaN/Inf become black.
    """
    if not (all(math.isfinite(x) for x in (near, far, uint16_scale)) and 0 <= near < far and uint16_scale > 0):
        raise ValueError('invalid depth display range/scale')
    if depth.ndim != 2:
        raise ValueError('depth input must be a single-channel image')
    if encoding.upper() == '16UC1' and depth.dtype == np.uint16:
        metres = depth.astype(np.float32) * uint16_scale
    elif encoding.upper() == '32FC1' and depth.dtype == np.float32:
        metres = depth.copy()
    else:
        raise ValueError('depth requires 16UC1/uint16 or 32FC1/float32, got %s/%s' %
                         (encoding, depth.dtype))
    valid = np.isfinite(metres) & (metres > 0)
    out = np.zeros(metres.shape, np.uint8)
    out[valid] = np.clip(255.0 * (far - metres[valid]) / (far - near), 1, 255).astype(np.uint8)
    return out


def _edges(frame, p):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    if gray.dtype != np.uint8:
        raise ValueError('OpenCV detector input must be uint8')
    blurred = cv2.GaussianBlur(gray, (p['blur_kernel'], p['blur_kernel']), 0)
    return blurred, cv2.Canny(blurred, p['canny_low'], p['canny_high'])


def _contours(edges):
    # OpenCV 3 and 4/5 have different tuple lengths.
    result = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return result[-2], result[-1]


def _quad(contour, shape, p):
    area = abs(cv2.contourArea(contour))
    if not p['min_area'] <= area <= shape[0] * shape[1] * p['max_area_fraction']:
        return None
    perimeter = cv2.arcLength(contour, True)
    q = cv2.approxPolyDP(contour, p['epsilon_ratio'] * perimeter, True)
    if len(q) != 4 or not cv2.isContourConvex(q):
        return None
    pts = q.reshape(4, 2).astype(np.float32)
    m = p['border_margin']
    if np.any(pts[:, 0] < m) or np.any(pts[:, 1] < m) or np.any(pts[:, 0] >= shape[1]-m) or np.any(pts[:, 1] >= shape[0]-m):
        return None
    sides = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    if np.min(sides) < p['min_side']:
        return None
    cosines = []
    for i in range(4):
        a, b = pts[(i-1) % 4]-pts[i], pts[(i+1) % 4]-pts[i]
        cosines.append(abs(float(np.dot(a, b))) / (np.linalg.norm(a)*np.linalg.norm(b)))
    if max(cosines) > p['max_angle_cosine']:
        return None
    if max(sides[0]/sides[2], sides[2]/sides[0], sides[1]/sides[3], sides[3]/sides[1]) > p['max_opposite_side_ratio']:
        return None
    width, height = float((sides[0]+sides[2])/2), float((sides[1]+sides[3])/2)
    aspect = max(width, height)/min(width, height)
    rw, rh = cv2.minAreaRect(q)[1]
    if not p['min_aspect_ratio'] <= aspect <= p['max_aspect_ratio'] or area/max(1, rw*rh) < p['min_rectangularity']:
        return None
    # Label the more horizontal pair W; contour start/winding can change.
    # Both dimensions remain image pixels, not physical measurements.
    vectors = np.roll(pts, -1, axis=0) - pts
    horizontal = np.abs(vectors[:, 0]) / sides
    if horizontal[0]+horizontal[2] < horizontal[1]+horizontal[3]:
        width, height = height, width
    return dict(contour=q, center=np.mean(pts, axis=0), area=area,
                width=width, height=height, aspect=aspect)


def _checker_period(gray, side, p):
    """Check alternating cells and repetition along each rectified side band.

    Adjacent checker rows have opposite phase: correlate pixels without first
    averaging rows. Kernel/lag limits bound the work at large input resolutions.
    """
    length = max(8, int(np.linalg.norm(side[1] - side[0])))
    thickness = max(4, int(np.linalg.norm((side[2]+side[3]-side[0]-side[1])/2)))
    length = min(length, 1024)
    thickness = min(thickness, 64)
    dest = np.array([[0, 0], [length-1, 0], [length-1, thickness-1],
                     [0, thickness-1]], np.float32)
    transform = cv2.getPerspectiveTransform(side, dest)
    patch = cv2.warpPerspective(gray, transform, (length, thickness)).astype(np.float32)
    signal = patch - cv2.GaussianBlur(patch, (0, 0), 8.0)
    max_cell = min(p['checker_max_cell_px'], (length-1)//8)
    correlations = {}
    for lag in range(p['checker_min_cell_px'], 2*max_cell+2):
        a, b = signal[:, :-lag], signal[:, lag:]
        denom = math.sqrt(float(np.sum(a*a))*float(np.sum(b*b)))
        correlations[lag] = float(np.sum(a*b))/max(denom, 1e-8)
    for cell in range(p['checker_min_cell_px'], max_cell+1):
        alternation = -correlations[cell]
        repeat = max(correlations.get(lag, -1.0) for lag in (2*cell-1, 2*cell, 2*cell+1))
        if alternation >= p['checker_min_alternation'] and repeat >= p['checker_min_repeat']:
            return dict(cell_px=cell, alternation=alternation, repeat=repeat)
    return None


def _checkerboard_support(frame, edges, candidate, p):
    """Return texture evidence for a checkerboard border around an opening.

    The candidate is the visible inner opening.  Each side band is measured
    independently so a textured background on only one side cannot authorize
    the fallback.  This is deliberately conservative and does not run when a
    valid nested outer/inner frame pair was found at this candidate's position.
    """
    height, width = frame.shape[:2]
    if candidate['area'] < p['checker_min_area_fraction'] * height * width:
        return None
    points = candidate['contour'].reshape(4, 2).astype(np.float32)
    center = points.mean(axis=0)
    expanded = center + (points - center) * p['checker_outer_scale']
    margin = float(p['border_margin'])
    if (np.any(expanded[:, 0] < margin) or np.any(expanded[:, 1] < margin) or
            np.any(expanded[:, 0] >= width - margin) or np.any(expanded[:, 1] >= height - margin)):
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    side_scores = []
    for index in range(4):
        side = np.array((points[index], points[(index + 1) % 4],
                         expanded[(index + 1) % 4], expanded[index]), dtype=np.float32)
        x, y, w, h = cv2.boundingRect(np.round(side).astype(np.int32))
        w, h = min(w, width-x), min(h, height-y)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(side - [x, y]).astype(np.int32), 255)
        pixels = mask > 0
        if int(np.count_nonzero(pixels)) < 20:
            return None
        values = gray[y:y+h, x:x+w][pixels]
        edge_fraction = float(np.count_nonzero(edges[y:y+h, x:x+w][pixels])) / float(values.size)
        std = float(np.std(values))
        dark_fraction = float(np.mean(values <= p['checker_dark_threshold']))
        light_fraction = float(np.mean(values >= p['checker_light_threshold']))
        if (edge_fraction < p['checker_min_side_edge_fraction'] or
                std < p['checker_min_side_std'] or
                dark_fraction < p['checker_min_dark_fraction'] or
                light_fraction < p['checker_min_light_fraction']):
            return None
        periodicity = _checker_period(gray, side, p)
        if periodicity is None:
            return None
        side_scores.append(dict(edge_fraction=edge_fraction, std=std,
                                dark_fraction=dark_fraction, light_fraction=light_fraction,
                                **periodicity))
    return dict(scale=p['checker_outer_scale'], sides=side_scores,
                score=min(score['edge_fraction'] for score in side_scores))


def _texture_openings(frame, p):
    """Find holes inside locally high-contrast frame material, NOT whole boards.

    Checker cells fragment Canny contours. Local variance groups those cells;
    morphology removes thin reflections. A convex hull only smooths indentations
    if the *original hole* fills >= min_rectangularity of that hull. The result
    is still just a candidate: every side must pass _checkerboard_support.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    gray = gray.astype(np.float32)
    square = gray*gray
    open_kernel = np.ones((p['checker_morph_open'],)*2, np.uint8)
    close_kernel = np.ones((p['checker_morph_close'],)*2, np.uint8)
    min_area = max(p['min_area'], p['checker_min_area_fraction']*gray.size)
    for kernel in p['checker_texture_kernels']:
        mean = cv2.boxFilter(gray, -1, (kernel, kernel))
        std = np.sqrt(np.maximum(cv2.boxFilter(square, -1, (kernel, kernel))-mean*mean, 0))
        for threshold in p['checker_texture_thresholds']:
            mask = (std >= threshold).astype(np.uint8)*255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            contours, hierarchy = _contours(mask)
            if hierarchy is None:
                continue
            for index, contour in enumerate(contours):
                # Only odd-depth contours are holes in foreground components.
                parent, depth = int(hierarchy[0][index][3]), 0
                while parent >= 0:
                    depth += 1
                    parent = int(hierarchy[0][parent][3])
                area = abs(cv2.contourArea(contour))
                if depth % 2 != 1 or area < min_area:
                    continue
                candidate = _quad(cv2.convexHull(contour), frame.shape, p)
                if candidate is None or area/candidate['area'] < p['min_rectangularity']:
                    continue
                candidate['method'] = 'checker_texture'
                candidate['texture_hole_contour'] = contour
                yield candidate


def _texture_rectangle_candidate(candidate, shape, p):
    """Bounded alternative for an already validated texture-hole quadrilateral.

    An intrusive background edge can pull one approximated corner into the
    opening. A minimum-area rectangle is only a new candidate, not acceptance:
    the original texture hole must occupy >= min_rectangularity of it, and
    detect_windows rechecks ALL four checker side bands with unchanged limits.
    This remains approximate image geometry, not a measured aperture boundary.
    """
    contour = candidate['texture_hole_contour']
    points = cv2.boxPoints(cv2.minAreaRect(contour))
    box = np.rint(points).astype(np.int32).reshape(4, 1, 2)
    rectangle = _quad(box, shape, p)
    if rectangle is None:
        return None
    if abs(cv2.contourArea(contour))/rectangle['area'] < p['min_rectangularity']:
        return None
    rectangle['method'] = 'checker_texture'
    rectangle['candidate_geometry'] = 'bounded_min_area_rectangle'
    return rectangle


def detect_windows(frame, parameters=None):
    p = checked_parameters(WINDOW_DEFAULTS, parameters or {})
    _, edges = _edges(frame, p)
    contours, hierarchy = _contours(edges)
    quads = {i: q for i, c in enumerate(contours) for q in [_quad(c, frame.shape, p)] if q}
    found = []
    for i, outer in quads.items():
        inner_match = None
        if hierarchy is not None:
            for j, inner in quads.items():
                ancestor = hierarchy[0][j][3]
                while ancestor >= 0 and ancestor != i:
                    ancestor = hierarchy[0][ancestor][3]
                if ancestor != i:
                    continue
                ratio = inner['area']/outer['area']
                if not p['min_inner_area_ratio'] <= ratio <= p['max_inner_area_ratio']:
                    continue  # rejects the two almost identical Canny edges of a solid box
                if np.linalg.norm(inner['center']-outer['center']) > p['max_inner_center_offset']*math.sqrt(outer['area']):
                    continue
                if abs(inner['aspect']-outer['aspect']) > p['max_inner_aspect_difference']:
                    continue
                ip = inner['contour'].reshape(4, 2).astype(np.float32)
                if any(cv2.pointPolygonTest(outer['contour'], tuple(map(float, pt)), True) < p['min_frame_gap'] for pt in ip):
                    continue
                op = outer['contour'].reshape(4, 2).astype(np.float32)
                oe, ie = np.roll(op, -1, axis=0)-op, np.roll(ip, -1, axis=0)-ip
                oe /= np.linalg.norm(oe, axis=1)[:, None]
                ie /= np.linalg.norm(ie, axis=1)[:, None]
                if np.min(np.max(np.abs(ie @ oe.T), axis=1)) < p['min_inner_parallel_cosine']:
                    continue
                inner_match = inner['contour']
                break
        if p['require_inner_frame'] and inner_match is None:
            continue
        outer['checker_fallback'] = False
        outer['method'] = 'nested_frame' if inner_match is not None else 'quad_only'
        outer['outline_kind'] = 'outer_frame'
        outer['inner_contour'] = inner_match
        found.append(outer)

    if p['require_inner_frame'] and p['checker_fallback']:
        # Prefer texture-derived opening geometry over a noisy Canny approximation.
        # Accepted frames at other positions remain independently detectable.
        candidates = _texture_openings(frame, p) if p['checker_texture_candidates'] else ()
        for candidate in chain(candidates, sorted(quads.values(), key=lambda q: q['area'], reverse=True)):
            if any(np.linalg.norm(candidate['center']-a['center']) <
                   p['dedup_center_ratio']*math.sqrt(a['area']) for a in found):
                continue
            support = _checkerboard_support(frame, edges, candidate, p)
            if support is None and candidate.get('method') == 'checker_texture':
                rectangle = _texture_rectangle_candidate(candidate, frame.shape, p)
                if rectangle is not None:
                    support = _checkerboard_support(frame, edges, rectangle, p)
                    if support is not None:
                        candidate = rectangle
            if support is None:
                continue
            target = dict(candidate, checker_fallback=True, checker_support=support,
                          inner_contour=None, outline_kind='opening')
            target.setdefault('method', 'checker_contour')
            found.append(target)
    unique = []
    for q in sorted(found, key=lambda x: x['area'], reverse=True):
        if not any(np.linalg.norm(q['center']-a['center']) < p['dedup_center_ratio']*math.sqrt(a['area']) for a in unique):
            unique.append(q)
    return unique, edges


def annotate_windows(frame, found):
    """Shared offline/live overlay. Checker fallback outlines the inner opening."""
    out = frame.copy()
    for target in found:
        cv2.drawContours(out, [target['contour']], -1, (0, 230, 0), 2)
        if target['inner_contour'] is not None:
            cv2.drawContours(out, [target['inner_contour']], -1, (255, 180, 0), 1)
        center = tuple(int(round(x)) for x in target['center'])
        cv2.drawMarker(out, center, (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        kind = 'OPENING' if target['outline_kind'] == 'opening' else 'FRAME'
        label = '%s W=%.0f H=%.0f px' % (kind, target['width'], target['height'])
        cv2.putText(out, label, (max(0, center[0]-125), max(44, center[1]-14)),
                    cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 255, 255), 1)
        cv2.putText(out, target['method'],
                    (max(0, center[0]-100), min(out.shape[0]-8, center[1]+24)),
                    cv2.FONT_HERSHEY_SIMPLEX, .44, (0, 255, 255), 1)
    cv2.putText(out, 'WINDOW: %s (%d) | RGB ONLY | NO CONTROL' %
                ('FOUND' if found else 'NOT FOUND', len(found)), (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 220, 0) if found else (0, 180, 255), 1)
    return out


def detect_circles(frame, parameters=None):
    p = checked_parameters(CIRCLE_DEFAULTS, parameters or {})
    blurred, edges = _edges(frame, p)
    contours, _ = _contours(edges)
    candidates = []
    for c in contours:
        area, perimeter = abs(cv2.contourArea(c)), cv2.arcLength(c, True)
        if area < p['min_area'] or perimeter <= 0 or len(c) < 5:
            continue
        (x, y), radius = cv2.minEnclosingCircle(c)
        if not p['min_radius'] <= radius <= p['max_radius']:
            continue
        m = p['border_margin']
        if x-radius < m or y-radius < m or x+radius >= frame.shape[1]-m or y+radius >= frame.shape[0]-m:
            continue
        circularity = 4*math.pi*area/(perimeter*perimeter)
        if circularity < p['min_circularity'] or area/(math.pi*radius*radius) < p['min_circle_fill']:
            continue
        (_, _), axes, _ = cv2.fitEllipse(c)
        if min(axes)/max(axes) < p['min_axis_ratio']:
            continue
        distances = np.linalg.norm(c.reshape(-1, 2)-np.array([x, y]), axis=1)
        if np.std(distances)/max(np.mean(distances), 1) > p['max_radial_cv']:
            continue
        candidates.append(dict(center=np.array([x, y]), radius=radius, contour=c, circularity=circularity, ring=False))
    for a in candidates:
        a['ring'] = any(p['min_ring_radius_ratio'] <= b['radius']/a['radius'] <= p['max_ring_radius_ratio']
                        and np.linalg.norm(a['center']-b['center']) <= p['concentric_center_ratio']*a['radius']
                        for b in candidates if b is not a)
    if p['require_ring']:
        candidates = [a for a in candidates if a['ring']]
    if p['use_hough']:
        hough = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, p['hough_dp'], p['hough_min_dist'],
                                 param1=p['hough_param1'], param2=p['hough_param2'],
                                 minRadius=int(p['min_radius']), maxRadius=int(p['max_radius']))
        circles = [] if hough is None else hough[0]
        # Hough alone never authorizes ON: require the contour shape checks above too.
        candidates = [a for a in candidates if any(np.linalg.norm(a['center']-b[:2]) < 0.25*a['radius']
                      and abs(a['radius']-b[2]) < 0.25*a['radius'] for b in circles)]
    unique = []
    for a in sorted(candidates, key=lambda c: c['radius'], reverse=True):
        if not any(np.linalg.norm(a['center']-b['center']) <= p['dedup_center_ratio']*max(a['radius'], b['radius']) for b in unique):
            unique.append(a)
    return unique, edges
