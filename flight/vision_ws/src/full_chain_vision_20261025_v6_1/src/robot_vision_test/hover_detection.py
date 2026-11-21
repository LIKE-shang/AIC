"""Competition-only concentric target centre; no laser enable decision."""
import itertools
import numpy as np
from .air_target import detect_air_target,ellipse_norm


def detect_hover_target(frame,supplied=None):
    strict,mask=detect_air_target(frame,supplied)
    if strict['valid']:
        strict['hover_detection_mode']='three_ring_original'
        return strict,mask
    relaxed=dict(supplied or {})
    relaxed.update(min_contrast=25.,min_ring_radius_px=9.,ellipse_residual=.08,
                   ellipse_fill_tolerance=.18,center_tolerance=.10,ratio_tolerance=.09,
                   axis_ratio_tolerance=.12,angle_tolerance_deg=18.,
                   width_relative_tolerance=.85,max_inner_obstruction_fraction=.15)
    result,mask=detect_air_target(frame,relaxed)
    result['original_reason']=strict['reason']
    if len(result['targets'])==1:
        # Unrelated extra/clipped rings do not invalidate the one complete
        # matched target in this assigned single-target hover experiment.
        result.update(valid=True,reason='hover_unique_three_ring_center',
                      hover_detection_mode='three_ring_relaxed')
        return result,mask
    if len(result['targets'])>1:
        result.update(valid=False,reason='hover_multiple_targets',hover_detection_mode='unavailable')
        return result,mask
    if len(result['rings'])>12:
        result.update(valid=False,reason='hover_excess_ring_candidates',hover_detection_mode='unavailable')
        return result,mask
    pairs=[]
    diameters=relaxed.get('diameters_m',[.35,.24,.12]);width=relaxed.get('ring_width_m',.008)
    expected=(diameters[2]-width)/(diameters[1]-width)
    for big,small in itertools.combinations(result['rings'],2):
        ratios=small['axes']/big['axes']
        if np.max(np.abs(ratios-expected))>.065:continue
        if np.linalg.norm(big['center']-small['center'])>max(2.5,.08*big['axes'][1]):continue
        if abs(big['axes'][1]/big['axes'][0]-small['axes'][1]/small['axes'][0])>.10:continue
        if ellipse_norm(big['inner'],small['center'])+max(small['outer']['axes']/big['inner']['axes'])>=1:continue
        pairs.append(dict(rings=[big,small],aiming_inner=small['inner'],center=small['inner']['center'],
                          radius=float(big['axes'][0]),confidence=.5))
    result.update(targets=pairs if len(pairs)==1 else [],valid=len(pairs)==1,
                  reason=('hover_concentric_middle_inner_pair' if len(pairs)==1 else 'hover_center_missing_or_ambiguous'),
                  hover_detection_mode=('two_ring_measured_inner' if len(pairs)==1 else 'unavailable'))
    return result,mask
