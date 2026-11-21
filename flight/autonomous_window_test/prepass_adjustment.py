"""One bounded pre-window translation, then reacquire before straight passage."""
import math
from dataclasses import replace
from window_pass.geometry import add,sub,scale,dot,norm,yaw
from straight_pass import straight_route,RouteRejection
from attitude_envelope import support,enclosed
from live_geometry import validate_metric

LIMITS=dict(max_distance_m=.20,max_vertical_m=.15,max_path_m=.30,speed_mps=.08,
            timeout_s=8.,arrival_tolerance_m=.015,settle_s=.5,target_extra_margin_m=.01)


def settings(policy=None):
    if policy is None:return dict(LIMITS,enabled=False)
    if not isinstance(policy,dict) or set(policy)-set(LIMITS)-{'enabled','operator_authorized','operator_statement'}:
        raise ValueError('invalid prepass adjustment declaration')
    if type(policy.get('enabled')) is not bool:raise ValueError('explicit adjustment switch required')
    if policy['enabled'] and (policy.get('operator_authorized') is not True or not isinstance(policy.get('operator_statement'),str) or not policy['operator_statement'].strip()):
        raise ValueError('explicit bounded adjustment authorization required')
    result=dict(LIMITS,**policy)
    for k,maximum in LIMITS.items():
        value=result[k]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError('invalid adjustment limit:'+k)
        if k in ('settle_s','target_extra_margin_m'):
            if value!=maximum:raise ValueError('fixed adjustment policy:'+k)
        elif value>maximum:raise ValueError('adjustment scoped maximum exceeded:'+k)
    return result


def suggest(status,c,pose,ros,wall,received,after_stamp,ground_z,minimum_bottom,origin,column_radius,policy):
    """Find nearest feasible lateral/vertical translation of the current route.

    Each aperture edge is a half-plane in lateral/height displacement. Its
    closest point is the origin, a perpendicular projection, or intersection
    of two boundaries. Every candidate is independently rechecked by the full
    route planner, fence, height floor, and surveyed takeoff-column limits.
    """
    try:straight_route(status,c,pose,ros,wall,received,after_stamp)
    except RouteRejection as exc:e=exc.diagnostics
    else:raise ValueError('adjustment_not_needed')
    heading=yaw(pose.q);left=(-math.sin(heading),math.cos(heading),0.);up=(0.,0.,1.)
    constraints=[]
    for edge in e['edges']:
        w=edge['translation_gradient_local']
        constraints.append((dot(w,left),dot(w,up),policy['target_extra_margin_m']-edge['clearance_after_budget_m']))
    m=policy['max_distance_m'];v=policy['max_vertical_m']
    constraints.extend(((1.,0.,-m),(-1.,0.,-m),(0.,1.,-v),(0.,-1.,-v)))
    # Target floor is enclosed for the entire allowed attitude range plus the
    # existing tracking budget; current feedback retains the mission floor check.
    floor_z=ground_z+minimum_bottom+support(c,heading,(0.,0.,-1.))+c['clearance_tracking_budget_m']
    constraints.append((0.,1.,floor_z-pose.p[2]))
    reserve=c['position_uncertainty_m']+c['clearance_tracking_budget_m']+c['clearance_margin_m']
    for i in range(3):
        axis=tuple(float(j==i) for j in range(3))
        lower=c['geofence_min_m'][i]+support(c,heading,scale(axis,-1.))+reserve-pose.p[i]
        upper=c['geofence_max_m'][i]-support(c,heading,axis)-reserve-pose.p[i]
        constraints.extend(((left[i],up[i],lower),(-left[i],-up[i],-upper)))
    candidates=bounded_candidates(constraints)
    for lateral,vertical in sorted(candidates,key=lambda p:math.hypot(*p)):
        if any(a*lateral+b*vertical<d-1e-9 for a,b,d in constraints):continue
        delta=add(scale(left,lateral),scale(up,vertical));distance=norm(delta)
        if not .002<distance<=min(policy['max_distance_m'],policy['max_path_m'],c['max_align_distance_m']):continue
        target=add(pose.p,delta)
        if norm(sub(target,origin)[:2])+c['clearance_tracking_budget_m']>column_radius:continue
        if not enclosed(c,pose.p,heading) or not enclosed(c,target,heading):continue
        q=(0.,0.,math.sin(heading/2),math.cos(heading/2))
        try:route=straight_route(status,c,replace(pose,p=target,q=q),ros,wall,received,after_stamp)
        except (ValueError,TypeError,KeyError):continue
        duration=distance/min(policy['speed_mps'],c['align_speed_mps'])+policy['settle_s']
        if duration>=min(policy['timeout_s'],c['max_align_time_s']):continue
        return dict(kind='ONE_BOUNDED_LATERAL_VERTICAL_ADJUSTMENT',start=list(pose.p),target=list(target),
            lateral_m=lateral,vertical_m=vertical,length_m=distance,yaw_rad=heading,
            geometry_stamp=route['geometry_stamp'],epoch=route['epoch'],geometry_center=route['geometry_center'],
            geometry_corners=status['metric_geometry']['corners_enu_m'],proposed_route=route,
            expected_duration_s=duration,actual_output_permitted=False)
    raise ValueError('no_feasible_bounded_adjustment')


def validate_motion(plan,c,pose,status,ros,wall,received,after_stamp,ground_z,minimum_bottom,origin,column_radius,policy):
    g=validate_metric(status,dict(c,window_yaw_rad=math.atan2(status['metric_geometry']['normal_enu'][1],status['metric_geometry']['normal_enu'][0])),ros,wall,received,after_stamp)
    if list(g['epoch'])!=plan['epoch']:raise ValueError('adjustment_geometry_epoch_changed')
    normal=status['metric_geometry']['normal_enu']
    for point in (pose.p,plan['target']):
        distance=abs(dot(sub(g['center'],point),normal))
        if not c['approach_min_m']<=distance<=c['approach_max_m']:raise ValueError('adjustment_approach_distance_changed')
    if norm(sub(g['center'],plan['geometry_center']))>c['center_agreement_m']:raise ValueError('adjustment_window_moved')
    if any(norm(sub(a,b))>c['center_agreement_m'] for a,b in zip(status['metric_geometry']['corners_enu_m'],plan['geometry_corners'])):raise ValueError('adjustment_corners_changed')
    if norm(sub(pose.p,plan['start']))>policy['max_distance_m']+c['clearance_tracking_budget_m']:raise ValueError('adjustment_displacement_limit')
    if norm(sub(pose.p,origin)[:2])+c['clearance_tracking_budget_m']>column_radius:raise ValueError('adjustment_outside_clear_takeoff_column')
    if not enclosed(c,pose.p,plan['yaw_rad']) or not enclosed(c,plan['target'],plan['yaw_rad']):raise ValueError('adjustment_envelope_outside_fence')
    target_z=plan['target'][2]-support(c,plan['yaw_rad'],(0.,0.,-1.))-c['clearance_tracking_budget_m']-ground_z
    if target_z<minimum_bottom-1e-9:raise ValueError('adjustment_target_height_floor')
    if norm(sub(plan['target'],pose.p))>policy['max_distance_m']+c['clearance_tracking_budget_m']:raise ValueError('adjustment_remaining_distance_limit')
    return g


def advance(plan,c,pose,command,dt,elapsed,policy):
    if not 0<=dt<=c['max_loop_gap_s']:raise ValueError('loop_deadline')
    if elapsed>=min(policy['timeout_s'],c['max_align_time_s']):raise ValueError('adjustment_timeout')
    delta=sub(plan['target'],plan['start']);length=norm(delta);direction=scale(delta,1/length)
    progress=dot(sub(pose.p,plan['start']),direction)
    if norm(sub(sub(pose.p,plan['start']),scale(direction,progress)))>c['clearance_tracking_budget_m']:raise ValueError('adjustment_cross_track')
    old=dot(sub(command,plan['start']),direction)
    new=max(old,min(length,old+min(policy['speed_mps'],c['align_speed_mps'])*dt,progress+c['max_setpoint_lead_m']))
    target=add(plan['start'],scale(direction,new))
    if not enclosed(c,target,plan['yaw_rad']):raise ValueError('adjustment_setpoint_outside_fence')
    arrived=new>=length-1e-9 and norm(sub(pose.p,plan['target']))<=policy['arrival_tolerance_m']
    return target,arrived


def bounded_candidates(constraints,preferred=(0.,0.),radius=None):
    """Boundary candidates for nearest point in 2-D halfplanes and a disk.

    Feasibility is checked by the caller. Include line projections, vertices,
    disk projection, and line/disk intersections; no coarse grid search.
    """
    px,py=preferred;candidates=[preferred]
    for a,b,d in constraints:
        sq=a*a+b*b
        if sq>1e-15:
            delta=(d-a*px-b*py)/sq
            candidates.append((px+delta*a,py+delta*b))
            if radius is not None:
                base=(d*a/sq,d*b/sq);remaining=radius*radius-d*d/sq
                if remaining>=-1e-12:
                    step=math.sqrt(max(0.,remaining)/sq)
                    candidates.extend(((base[0]-b*step,base[1]+a*step),(base[0]+b*step,base[1]-a*step)))
    for i,(a,b,d) in enumerate(constraints):
        for x,y,z in constraints[i+1:]:
            determinant=a*y-b*x
            if abs(determinant)>1e-12:
                candidates.append(((d*y-b*z)/determinant,(a*z-d*x)/determinant))
    if radius is not None:
        length=math.hypot(px,py)
        if length>0:candidates.append((px*radius/length,py*radius/length))
    return candidates
