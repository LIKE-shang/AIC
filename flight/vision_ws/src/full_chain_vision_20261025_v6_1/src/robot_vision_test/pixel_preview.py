"""Image-only measured reference preview. No flight/PWM or hit certification."""
import copy
import math
import cv2
import numpy as np
from .air_target import ellipse_norm

DEFAULTS = dict(enabled=True, image_width=640, image_height=360,
                camera_frame_id='circle_usb_optical_frame',
                offset_x_px=53.25, offset_y_px=15.484375,
                inner_radius_ratio=1.0,
                pink_r_minus_g_min=20, pink_b_minus_g_min=10,
                pink_r_min=160, core_min_brightness=220.,
                core_peak_band=10., halo_area_min=2, halo_area_max=100,
                white_core_fallback=False, local_spot_search=False, white_core_brightness=235.,
                white_core_contrast=35., white_core_max_area=25,
                white_core_search_radius_px=8.)


def parameters(supplied=None):
    supplied = supplied or {}
    if not isinstance(supplied, dict) or set(supplied)-set(DEFAULTS):
        raise ValueError('invalid pixel preview parameters')
    out = dict(DEFAULTS, **supplied)
    if type(out['enabled']) is not bool:
        raise ValueError('enabled must be boolean')
    if type(out['white_core_fallback']) is not bool:
        raise ValueError('white_core_fallback must be boolean')
    if type(out['local_spot_search']) is not bool:
        raise ValueError('local_spot_search must be boolean')
    if not isinstance(out['camera_frame_id'], str) or not out['camera_frame_id'].strip():
        raise ValueError('camera_frame_id required')
    for key, value in out.items():
        if key in ('enabled', 'white_core_fallback', 'local_spot_search', 'camera_frame_id'): continue
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('finite number required: '+key)
    for key in ('image_width', 'image_height', 'halo_area_min', 'halo_area_max'):
        if type(out[key]) is not int or out[key] <= 0:
            raise ValueError('positive integer required: '+key)
    if not 0 < out['inner_radius_ratio'] <= 1:
        raise ValueError('inner_radius_ratio must be in (0,1]')
    if out['halo_area_min'] > out['halo_area_max']:
        raise ValueError('invalid halo area interval')
    for key in ('pink_r_minus_g_min', 'pink_b_minus_g_min', 'pink_r_min', 'core_min_brightness'):
        if not 0 < out[key] <= 255: raise ValueError('invalid colour threshold: '+key)
    if not 0 < out['core_peak_band'] < 255: raise ValueError('invalid core peak band')
    if not 220 <= out['white_core_brightness'] <= 255 or not 20 <= out['white_core_contrast'] <= 100:
        raise ValueError('invalid white core brightness/contrast')
    if type(out['white_core_max_area']) is not int or not 1 <= out['white_core_max_area'] <= 25 or not 1 <= out['white_core_search_radius_px'] <= 12:
        raise ValueError('invalid white core search bounds')
    point = reference_point(out)
    if not (0 <= point[0] < out['image_width'] and 0 <= point[1] < out['image_height']):
        raise ValueError('reference point outside image')
    return out


def reference_point(p):
    return [(p['image_width']-1)/2+p['offset_x_px'],
            (p['image_height']-1)/2+p['offset_y_px']]


def unavailable(reason):
    return dict(valid=False, reason=reason, preview_only=True,
                reference_point_px=None, target_centre_px=None,
                reference_error_px=None, reference_inside_inner=None,
                reference_normalized_radius=None, observed_candidates=[],
                observed_spot_px=None, observed_error_px=None,
                observed_inside_inner=None, observed_minus_reference_px=None,
                observed_normalized_radius=None, observed_unique=False,
                physical_hit_verified=False)


def candidate_in_region(point, outer, p):
    # Spot identity is independent of the scoring ring. A two-ring detection
    # contains the middle ring first, not the outside edge of the target.
    if p['local_spot_search']:
        return np.linalg.norm(np.asarray(point)-reference_point(p)) <= p['white_core_search_radius_px']
    return ellipse_norm(outer, point) < 1


def pink_candidates(frame, outer, p):
    """Candidate preview tuned on the saved burst, not validated light identity."""
    b, g, r = [frame[:, :, i].astype(np.int16) for i in range(3)]
    mask = ((r-g >= p['pink_r_minus_g_min']) & (b-g >= p['pink_b_minus_g_min']) &
            (r >= p['pink_r_min'])).astype(np.uint8)
    n, labels, stats, centres = cv2.connectedComponentsWithStats(mask, 8)
    # Discard colour noise before doing work proportional to component count.
    selected = [i for i in range(1, n)
                if p['halo_area_min'] <= stats[i, cv2.CC_STAT_AREA] <= p['halo_area_max']
                and candidate_in_region(centres[i], outer, p)]
    if len(selected) > 32: return [], 'excess_colour_candidates'
    result = []
    for index in selected:
        x, y, width, height, area = map(int, stats[index])
        x0, x1 = max(0, x-3), min(frame.shape[1], x+width+3)
        y0, y1 = max(0, y-3), min(frame.shape[0], y+height+3)
        brightness = frame[y0:y1, x0:x1].astype(float).mean(axis=2)
        peak = float(brightness.max())
        if peak < p['core_min_brightness']: continue
        ys, xs = np.where(brightness >= max(p['core_min_brightness'], peak-p['core_peak_band']))
        if not len(xs): continue
        centre = [float(xs.mean()+x0), float(ys.mean()+y0)]
        if not candidate_in_region(centre, outer, p): continue
        box = [int(xs.min()+x0), int(ys.min()+y0), int(xs.max()+x0), int(ys.max()+y0)]
        same = next((c for c in result if c['box_px'] == box and
                     np.allclose(c['centre_px'], centre, atol=1e-9)), None)
        if same is None:
            result.append(dict(centre_px=centre, box_px=box, halo_pixels=area,
                               halo_components=1, core_pixels=int(len(xs))))
        else:
            same['halo_components'] += 1
            same['halo_pixels'] += area
    return result, ('unique_bright_candidate' if len(result) == 1 else 'bright_candidate_missing_or_multiple')


def evaluate(frame, frame_id, detection, supplied=None):
    p = parameters(supplied)
    result = unavailable('disabled')
    if not p['enabled']: return result
    if frame is None or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        return unavailable('requires_uint8_BGR_image')
    if frame.shape[:2] != (p['image_height'], p['image_width']) or frame_id != p['camera_frame_id']:
        return unavailable('pixel_reference_camera_or_size_mismatch')
    result['reference_point_px'] = reference_point(p)
    if not detection.get('valid') or len(detection.get('targets', [])) != 1:
        result['reason'] = 'await_unique_three_ring_target'
        return result
    target = detection['targets'][0]
    inner = target['aiming_inner'] if 'aiming_inner' in target else target['rings'][2]['inner']
    opening = dict(inner, axes=np.asarray(inner['axes'])*p['inner_radius_ratio'])
    point = np.asarray(result['reference_point_px'])
    centre = np.asarray(inner['center'])
    normal = ellipse_norm(opening, point)
    result.update(target_detection_mode=detection.get('hover_detection_mode','three_ring_original'),
                  detector_reason=detection.get('reason',''),
                  valid=True, reason='reference_inside_inner' if normal < 1 else 'reference_outside_inner',
                  target_centre_px=centre.tolist(), reference_error_px=(point-centre).tolist(),
                  reference_inside_inner=bool(normal < 1), reference_normalized_radius=normal,
                  inner_radius_ratio=p['inner_radius_ratio'],
                  inner_semiaxes_px=np.asarray(inner['axes']).tolist(), inner_angle_deg=float(inner['angle']))
    candidates, reason = pink_candidates(frame, target['rings'][0]['outer'], p)
    result['observed_basis'] = 'colour_halo'
    if p['local_spot_search'] and p['white_core_fallback']:
        # Inspect the same local region both OFF and ON. Do not let one pink
        # reflection hide an independent white core (or vice versa).
        white = white_core_candidates(frame, target['rings'][0]['outer'], p)
        for core in white:
            box = core['box_px']
            same = next((c for c in candidates if
                         np.linalg.norm(np.asarray(c['centre_px'])-core['centre_px']) <= 1.5 and
                         max(box[0],c['box_px'][0]) <= min(box[2],c['box_px'][2]) and
                         max(box[1],c['box_px'][1]) <= min(box[3],c['box_px'][3])), None)
            if same is None: candidates.append(core)
        reason = 'unique_local_candidate' if len(candidates) == 1 else 'local_candidate_missing_or_multiple'
        if len(candidates) == 1 and candidates[0].get('basis') == 'white_core_local_contrast':
            result['observed_basis'] = 'white_core_local_contrast'
    elif p['white_core_fallback'] and not candidates:
        candidates = white_core_candidates(frame, target['rings'][0]['outer'], p)
        reason = 'unique_white_core_candidate' if len(candidates) == 1 else 'white_core_missing_or_multiple'
        result['observed_basis'] = 'white_core_local_contrast'
    result['candidate_search_policy'] = 'calibrated_neighbourhood' if p['local_spot_search'] else 'measured_ring'
    result['observed_candidates'], result['observed_reason'] = candidates, reason
    if len(candidates) == 1:
        spot = np.asarray(candidates[0]['centre_px'])
        result.update(observed_spot_px=spot.tolist(), observed_error_px=(spot-centre).tolist(),
                      observed_inside_inner=bool(ellipse_norm(opening, spot) < 1),
                      observed_normalized_radius=float(ellipse_norm(opening, spot)),observed_unique=True,
                      observed_minus_reference_px=(spot-point).tolist())
    return result


def white_core_candidates(frame, outer, p):
    """Candidate evidence only; the controller must check OFF/ON and continuity.

    Tiny saturated spots may lose their colour halo. Search only the calibrated
    image neighbourhood, with bounded area and measured local contrast.
    """
    point = reference_point(p)
    radius = p['white_core_search_radius_px']
    x0, x1 = max(0, int(point[0]-radius)-1), min(frame.shape[1], int(point[0]+radius)+2)
    y0, y1 = max(0, int(point[1]-radius)-1), min(frame.shape[0], int(point[1]+radius)+2)
    bright = frame[y0:y1, x0:x1].astype(float).mean(axis=2)
    mask = (bright >= p['white_core_brightness']).astype(np.uint8)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    result = []
    for i in range(1, count):
        x, y, w, h, area = map(int, stats[i])
        center = centers[i]+[x0, y0]
        if not 2 <= area <= p['white_core_max_area'] or max(w,h) > 3*min(w,h): continue
        if x == 0 or y == 0 or x+w == bright.shape[1] or y+h == bright.shape[0]: continue
        if np.linalg.norm(center-point) > radius or not candidate_in_region(center, outer, p): continue
        rx0, rx1 = max(0,x-4), min(bright.shape[1],x+w+4)
        ry0, ry1 = max(0,y-4), min(bright.shape[0],y+h+4)
        ring = bright[ry0:ry1, rx0:rx1]
        ring_mask = labels[ry0:ry1, rx0:rx1] != i
        if not ring_mask.any(): continue
        contrast = float(bright[labels == i].mean()-np.median(ring[ring_mask]))
        if contrast < p['white_core_contrast']: continue
        result.append(dict(centre_px=center.tolist(), box_px=[x+x0,y+y0,x+x0+w-1,y+y0+h-1],
                           core_pixels=area, local_contrast=contrast, basis='white_core_local_contrast'))
    return result


def draw(frame, detection, result):
    """Draw concise preview labels; no automatic pass or physical hit claim."""
    for target in detection.get('targets', []):
        for ring, colour in zip(target['rings'], ((0,255,0),(255,180,0),(255,0,255))):
            e = ring['inner']
            cv2.ellipse(frame, (tuple(map(float,e['center'])),tuple(map(float,2*np.asarray(e['axes']))),
                        float(e['angle'])), colour, 1)
    def mark(point, colour, kind):
        if point is not None:
            cv2.drawMarker(frame, tuple(np.rint(point).astype(int)), colour, kind, 11, 1)
    mark(result['target_centre_px'], (0,255,0), cv2.MARKER_CROSS)
    mark(result['reference_point_px'], (0,255,255), cv2.MARKER_TILTED_CROSS)
    mark(result['observed_spot_px'], (0,0,255), cv2.MARKER_CROSS)
    lines = ['PIXEL PREVIEW ONLY | no flight/PWM output', result['reason']]
    if result['reference_error_px'] is not None:
        dx, dy = result['reference_error_px']
        lines.append('REF dx=%+.1f dy=%+.1f px | radius=%.2f (<1 inside)' %
                     (dx,dy,result['reference_normalized_radius']))
    if result['observed_minus_reference_px'] is not None:
        dx,dy = result['observed_minus_reference_px']
        lines.append('SPOT-REF dx=%+.1f dy=%+.1f px | spot %s' %
                     (dx,dy,'INSIDE' if result['observed_inside_inner'] else 'OUTSIDE'))
    else:
        lines.append('SPOT candidate unavailable / ambiguous')
    for i,line in enumerate(lines):
        cv2.putText(frame,line,(6,16+16*i),cv2.FONT_HERSHEY_SIMPLEX,.36,(0,180,255),1,cv2.LINE_AA)
    return frame
