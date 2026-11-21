"""Three-ring target, calibrated prediction and optional colour-spot evidence.
ROS-free; NO hardware I/O. Closed paired boundaries only: never invent an
occluded ring. Ellipse geometry is a bounded perspective approximation, not 3D.
"""
import copy
import itertools
import math
import cv2
import numpy as np

AIR_DEFAULTS = dict(
    diameters_m=[0.35, 0.24, 0.12], ring_width_m=0.008,
    diameter_definition='unconfirmed', dimensions_confirmed=False,
    blur_kernel=3, min_contrast=45.0, min_ring_radius_px=14.0,
    min_axis_ratio=0.55, ellipse_residual=0.035, ellipse_fill_tolerance=0.10,
    center_tolerance=0.055, ratio_tolerance=0.055, axis_ratio_tolerance=0.07,
    angle_tolerance_deg=10.0, width_relative_tolerance=0.65,
    width_pixel_tolerance=1.5, max_inner_obstruction_fraction=0.02, border_margin=4, polarity='dark',
    safety_margin_m=0.005, effective_radius_cap_m=0.045,
    track_center_ratio=0.18, track_scale_ratio=0.10,
    working_distance_m=0.0, operating_envelope_confirmed=False,
    live_camera_verified=False, prediction_only_acknowledged=False,
    feedback_mode='prediction_only',
    calibration=dict(confirmed=False, calibration_id='', camera_frame_id='',
                     image_width=0, image_height=0, offset_px=[], distance_m=0.0,
                     distance_tolerance_m=0.02, outer_major_px=0.0,
                     scale_tolerance=0.05, axis_ratio=0.0, axis_tolerance=0.04,
                     angle_deg=0.0, angle_tolerance_deg=5.0,
                     spot_radius_m=-1.0, error_bound_m=-1.0),
    spot=dict(validated=False, color='unconfirmed', min_value=210,
              min_saturation=100, min_area_px=2.0, max_area_px=120.0,
              max_aspect=2.0, min_fill=0.45))


def air_parameters(supplied=None):
    p = copy.deepcopy(AIR_DEFAULTS)
    for k, v in (supplied or {}).items():
        if k in ('calibration', 'spot'):
            if not isinstance(v, dict):
                raise ValueError(k+' must be a mapping')
            unknown = set(v)-set(p[k])
            if unknown:
                raise ValueError('unknown '+k+' keys: '+str(sorted(unknown)))
            p[k].update(v)
        elif k in p:
            p[k] = v
    def numeric(v):
        if isinstance(v, bool):
            return
        if isinstance(v, (int, float)) and not math.isfinite(v):
            raise ValueError('nonfinite air-target parameter')
        if isinstance(v, dict):
            for item in v.values(): numeric(item)
        if isinstance(v, (list, tuple)):
            for item in v: numeric(item)
    numeric(p)
    for obj, names in [(p, ('dimensions_confirmed', 'operating_envelope_confirmed',
                            'live_camera_verified', 'prediction_only_acknowledged')),
                       (p['calibration'], ('confirmed',)), (p['spot'], ('validated',))]:
        for name in names:
            if type(obj[name]) is not bool: raise ValueError(name+' must be boolean')
    d = p['diameters_m']
    if len(d) != 3 or not (d[0] > d[1] > d[2] > 4*p['ring_width_m'] > 0):
        raise ValueError('need three descending physical diameters and a separate line width')
    if p['diameter_definition'] not in ('unconfirmed', 'outer', 'centerline', 'inner'):
        raise ValueError('diameter_definition must be outer/centerline/inner/unconfirmed')
    if p['dimensions_confirmed'] and p['diameter_definition'] == 'unconfirmed':
        raise ValueError('confirm the diameter definition first')
    if type(p['blur_kernel']) is not int or p['blur_kernel'] not in (3, 5, 7):
        raise ValueError('blur_kernel must be 3,5,7')
    if p['polarity'] not in ('dark', 'bright') or p['feedback_mode'] not in ('prediction_only', 'camera'):
        raise ValueError('invalid polarity/feedback_mode')
    for key in ('min_axis_ratio', 'ellipse_residual', 'ellipse_fill_tolerance',
                'center_tolerance', 'ratio_tolerance', 'axis_ratio_tolerance',
                'width_relative_tolerance', 'max_inner_obstruction_fraction', 'track_center_ratio', 'track_scale_ratio'):
        if not 0 < p[key] < 1: raise ValueError('invalid '+key)
    for key in ('min_contrast', 'min_ring_radius_px', 'angle_tolerance_deg',
                'width_pixel_tolerance', 'effective_radius_cap_m'):
        if p[key] <= 0: raise ValueError('invalid '+key)
    if p['safety_margin_m'] < 0 or p['border_margin'] < 0 or p['working_distance_m'] < 0:
        raise ValueError('negative physical/geometry setting')
    c, s = p['calibration'], p['spot']
    for key in ('distance_tolerance_m', 'scale_tolerance', 'axis_tolerance', 'angle_tolerance_deg'):
        if c[key] <= 0: raise ValueError('invalid calibration '+key)
    if c['confirmed']:
        if (not c['calibration_id'] or not c['camera_frame_id'] or
                c['image_width'] <= 0 or c['image_height'] <= 0 or
                len(c['offset_px']) != 2 or c['distance_m'] <= 0 or c['outer_major_px'] <= 0 or
                not 0 < c['axis_ratio'] <= 1 or c['spot_radius_m'] <= 0 or c['error_bound_m'] <= 0):
            raise ValueError('confirmed calibration must contain measured identity/geometry/error/spot size')
    if s['color'] not in ('unconfirmed', 'red', 'green'):
        raise ValueError('spot color must be measured red/green or unconfirmed')
    if s['validated'] and s['color'] == 'unconfirmed': raise ValueError('validate measured spot color first')
    if not (0 <= s['min_value'] <= 255 and 0 <= s['min_saturation'] <= 255 and
            0 < s['min_area_px'] < s['max_area_px'] and s['max_aspect'] >= 1 and 0 < s['min_fill'] <= 1):
        raise ValueError('invalid spot thresholds')
    return p


def physical_radii(p):
    """Returns centre-line radii and unpainted inner radius. Unknown = preview only."""
    w = p['ring_width_m']
    correction = {'outer': -w, 'centerline': 0, 'inner': w, 'unconfirmed': 0}[p['diameter_definition']]
    radii = (np.asarray(p['diameters_m'], float)+correction)/2
    return radii, float(radii[2]-w/2)


def angle_difference(a, b):
    return abs((a-b+90) % 180-90)


def ellipse_points(e, points):
    theta = math.radians(e['angle'])
    rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return (np.asarray(points, float)-e['center']) @ rotation / e['axes']


def ellipse_norm(e, point):
    return float(np.linalg.norm(ellipse_points(e, np.asarray(point))))


def _ellipse(contour, p):
    if len(contour) < 16: return None
    (x, y), (a, b), angle = cv2.fitEllipse(contour)
    a, b = a/2, b/2
    if a < b: a, b, angle = b, a, angle+90
    if b <= 2 or b/a < p['min_axis_ratio']: return None
    e = dict(center=np.array([x, y]), axes=np.array([a, b]), angle=angle % 180)
    residual = float(np.quantile(np.abs(np.linalg.norm(ellipse_points(e, contour[:, 0, :]), axis=1)-1), .95))
    fill = cv2.contourArea(contour)/(math.pi*a*b)
    if residual > p['ellipse_residual'] or abs(fill-1) > p['ellipse_fill_tolerance']: return None
    e['residual'] = residual
    return e


def detect_air_target(frame, supplied=None):
    """Deduplicate each painted ring's two boundaries, then group EXACTLY three.
    Additional plausible rings (including incomplete second targets) veto ON.
    Open/occluded boundaries cannot qualify. No Hough hallucination of missing rings.
    """
    p = air_parameters(supplied)
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError('air target requires uint8 BGR camera frame')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (p['blur_kernel'],)*2, 0)
    if float(gray.max())-float(gray.min()) < p['min_contrast']:
        return dict(targets=[], rings=[], valid=False, reason='low_contrast'), np.zeros_like(gray)
    flag = cv2.THRESH_BINARY_INV if p['polarity'] == 'dark' else cv2.THRESH_BINARY
    _, mask = cv2.threshold(gray, 0, 255, flag | cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]
    rings = []
    h, w = gray.shape
    clipped = False
    if hierarchy is not None:
        tree = hierarchy[0]
        for i, contour in enumerate(contours):
            # Even nesting depth is foreground material, odd depth is its hole.
            depth, ancestor = 0, int(tree[i][3])
            while ancestor >= 0:
                depth += 1; ancestor = int(tree[ancestor][3])
            child = int(tree[i][2])
            if depth % 2 or child < 0: continue
            outer, inner = _ellipse(contour, p), _ellipse(contours[child], p)
            if outer is None or inner is None or outer['axes'][1] < p['min_ring_radius_px']: continue
            if tree[child][0] >= 0: continue  # multiple holes / occlusion / confusing texture
            thickness = outer['axes']-inner['axes']
            if np.any(thickness <= 0) or np.max(thickness/outer['axes']) > .30: continue
            if np.linalg.norm(outer['center']-inner['center']) > p['center_tolerance']*outer['axes'][1]: continue
            x, y, bw, bh = cv2.boundingRect(contour)
            m = p['border_margin']
            if x <= m or y <= m or x+bw >= w-m or y+bh >= h-m:
                clipped = True; continue
            ring = dict(center=(outer['center']+inner['center'])/2,
                        axes=(outer['axes']+inner['axes'])/2,
                        angle=outer['angle'], outer=outer, inner=inner,
                        thickness=thickness, contour_id=i)
            # RETR_TREE already pairs line edges; geometric dedup handles coincident candidates.
            if not any(np.linalg.norm(ring['center']-r['center']) < 2 and
                       np.max(np.abs(ring['axes']-r['axes'])) < 2 for r in rings):
                rings.append(ring)
    rings.sort(key=lambda r: -r['axes'][0])
    radii, _ = physical_radii(p)
    expected = radii/radii[0]
    targets = []
    # Bound pathological noisy frames before combinatorial grouping.
    if len(rings) > 12:
        return dict(targets=[], rings=rings, valid=False, reason='excess_ring_candidates'), mask
    for triple in itertools.combinations(rings, 3):
        outer = triple[0]
        ratio_error = 0.0
        ok = True
        for j, r in enumerate(triple):
            ratios = r['axes']/outer['axes']
            err = float(np.max(np.abs(ratios-expected[j])))
            ratio_error = max(ratio_error, err)
            width_expected = r['axes'] * (p['ring_width_m']/radii[j])
            if (err > p['ratio_tolerance'] or
                np.linalg.norm(r['center']-outer['center']) > p['center_tolerance']*outer['axes'][1] or
                abs(r['axes'][1]/r['axes'][0]-outer['axes'][1]/outer['axes'][0]) > p['axis_ratio_tolerance'] or
                (outer['axes'][1]/outer['axes'][0] < .92 and angle_difference(r['angle'], outer['angle']) > p['angle_tolerance_deg']) or
                np.any(np.abs(r['thickness']-width_expected) > p['width_relative_tolerance']*width_expected+p['width_pixel_tolerance'])):
                ok = False; break
        if not ok: continue
        # Fitted containment is required; contour ordering alone is not topology.
        if any(ellipse_norm(triple[k], triple[k+1]['center']) +
               max(triple[k+1]['outer']['axes']/triple[k]['inner']['axes']) >= 1 for k in range(2)):
            continue
        # Visible obstruction inside the otherwise closed inner ring must also veto.
        # This cannot distinguish an occluder identical to the background; use controlled optics.
        inner=triple[2]['inner']
        region=np.zeros_like(mask)
        axes=tuple(float(max(1., a-3)) for a in inner['axes'])
        cv2.ellipse(region, (tuple(map(float, inner['center'])), tuple(2*a for a in axes), float(inner['angle'])), 255, -1)
        obstruction=float(np.count_nonzero(mask[region>0]))/max(1,np.count_nonzero(region))
        if obstruction > p['max_inner_obstruction_fraction']: continue
        targets.append(dict(rings=list(triple), inner_obstruction_fraction=obstruction, center=np.mean([r['center'] for r in triple], axis=0),
                            radius=float(outer['axes'][0]),
                            confidence=max(0., 1-ratio_error/p['ratio_tolerance']),
                            ratio_error=ratio_error))
    valid = len(targets) == 1 and len(rings) == 3 and not clipped
    reason = ('three_rings_unique' if valid else 'ambiguous_multiple_targets' if len(targets) > 1 else
              'extra_or_clipped_rings' if clipped or len(rings) > 3 else 'missing_or_inconsistent_rings')
    return dict(targets=targets, rings=rings, valid=valid, reason=reason), mask


def detect_spot(frame, supplied=None):
    p = air_parameters(supplied)['spot']
    if p['color'] == 'unconfirmed':
        return dict(valid=False, reason='spot_color_unconfirmed', candidates=[])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, sat, val = [hsv[:, :, i] for i in range(3)]
    color = ((hue <= 12) | (hue >= 168)) if p['color'] == 'red' else ((hue >= 35) & (hue <= 90))
    mask = (color & (sat >= p['min_saturation']) & (val >= p['min_value'])).astype(np.uint8)*255
    n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    candidates, uncertain = [], False
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < p['min_area_px']: continue
        if (area > p['max_area_px'] or max(w, h)/max(1, min(w, h)) > p['max_aspect'] or
                area/(w*h) < p['min_fill']):
            uncertain = True; continue
        candidates.append(dict(center=centers[i].tolist(), radius_px=max(w, h)/2, area_px=int(area)))
    valid = len(candidates) == 1 and not uncertain
    return dict(valid=valid, reason='unique_color_spot' if valid else 'spot_missing_or_ambiguous',
                candidates=candidates, uncertain=uncertain)


def assess_aim(target, shape, frame_id, supplied=None):
    p = air_parameters(supplied); c = p['calibration']; h, w = shape[:2]
    result = dict(calibration_valid=False, aim_valid=False, predicted_point=None,
                  effective_radius_m=None, effective_ellipse=None, reason='calibration_unconfirmed')
    if not p['dimensions_confirmed']:
        result['reason'] = 'diameter_definition_unconfirmed'; return result
    if not c['confirmed']: return result
    if (w != c['image_width'] or h != c['image_height'] or frame_id != c['camera_frame_id']):
        result['reason'] = 'calibration_camera_mismatch'; return result
    point = np.array([(w-1)/2, (h-1)/2])+np.asarray(c['offset_px'])
    result['predicted_point'] = point.tolist()
    if not p['operating_envelope_confirmed'] or abs(p['working_distance_m']-c['distance_m']) > c['distance_tolerance_m']:
        result['reason'] = 'distance_or_pose_not_confirmed'; return result
    outer = target['rings'][0]
    if (abs(outer['axes'][0]/c['outer_major_px']-1) > c['scale_tolerance'] or
            abs(outer['axes'][1]/outer['axes'][0]-c['axis_ratio']) > c['axis_tolerance'] or
            (c['axis_ratio'] < .92 and angle_difference(outer['angle'], c['angle_deg']) > c['angle_tolerance_deg'])):
        result['reason'] = 'outside_calibrated_scale_or_pose'; return result
    _, free = physical_radii(p)
    effective = min(p['effective_radius_cap_m'], free-c['spot_radius_m']-c['error_bound_m']-p['safety_margin_m'])
    if effective <= 0:
        result['reason'] = 'no_safe_inner_area'; return result
    inner = target['rings'][2]['inner']
    e = dict(center=inner['center'], axes=inner['axes']*(effective/free), angle=inner['angle'])
    error = point-inner['center']
    result.update(calibration_valid=True, aim_valid=ellipse_norm(e, point) < 1,
                  effective_radius_m=effective, effective_ellipse=e,
                  error_px=error.tolist(), normalized_error=ellipse_norm(e, point),
                  reason='predicted_inside' if ellipse_norm(e, point) < 1 else 'predicted_outside')
    return result


def assess_spot(spot, aim, target, supplied=None):
    p = air_parameters(supplied)
    if not spot['valid'] or not aim['calibration_valid'] or not p['spot']['validated']: return False
    s = spot['candidates'][0]
    _, free = physical_radii(p)
    metres_per_px = free/target['rings'][2]['inner']['axes'][1]
    # Size and disagreement with prediction both invalidate claimed spot identity.
    size_ok = s['radius_px']*metres_per_px <= p['calibration']['spot_radius_m']
    error_ok = np.linalg.norm(np.asarray(s['center'])-aim['predicted_point'])*metres_per_px <= p['calibration']['error_bound_m']
    return bool(size_ok and error_ok and ellipse_norm(aim['effective_ellipse'], s['center']) < 1)


def draw_air_overlay(frame, detection, aim, spot, lines=()):
    def ellipse(e, color, thickness=1):
        cv2.ellipse(frame, (tuple(map(float, e['center'])), tuple(map(float, 2*np.asarray(e['axes']))), float(e['angle'])), color, thickness)
    for target in detection['targets']:
        for r, label, color, text_angle in zip(target['rings'], ('OUTER', 'MIDDLE', 'INNER'), ((0,255,0),(255,170,0),(255,0,255)), (-20, 35, 85)):
            ellipse(r['outer'], color); ellipse(r['inner'], color)
            theta = math.radians(text_angle)
            q = tuple(np.rint(r['center']+r['axes']*np.array([math.cos(theta), math.sin(theta)])+np.array([5, 5])).astype(int))
            cv2.putText(frame, label, q, cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        cc = tuple(np.rint(target['center']).astype(int))
        cv2.drawMarker(frame, cc, (0,180,0), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(frame, 'CENTER', (cc[0]-65, cc[1]+18), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,140,0), 1)
    if aim and aim.get('effective_ellipse') is not None:
        e = aim['effective_ellipse']; ellipse(e, (0,255,255), 2)
        pt = tuple(np.rint(e['center']-np.array([e['axes'][0]+25, e['axes'][1]+12])).astype(int))
        cv2.putText(frame, 'SAFE AREA', pt, cv2.FONT_HERSHEY_SIMPLEX, .4, (0,150,180), 1)
    if aim and aim.get('predicted_point') is not None:
        pt = tuple(np.rint(aim['predicted_point']).astype(int))
        cv2.drawMarker(frame, pt, (255,0,0), cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.putText(frame, 'PREDICTED', (pt[0]+8, pt[1]-8), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,0,0), 1)
    for s in spot.get('candidates', []):
        pt = tuple(np.rint(s['center']).astype(int))
        cv2.circle(frame, pt, max(4, int(s['radius_px'])), (0,0,255), 1)
        cv2.putText(frame, 'OBSERVED candidate', (pt[0]+8, pt[1]+25), cv2.FONT_HERSHEY_SIMPLEX, .4, (0,0,255), 1)
    for i, text in enumerate(lines):
        cv2.putText(frame, str(text), (8, 20+i*20), cv2.FONT_HERSHEY_SIMPLEX, .43, (0,120,220), 1, cv2.LINE_AA)
    return frame


def jsonable(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [jsonable(v) for v in value]
    return value
