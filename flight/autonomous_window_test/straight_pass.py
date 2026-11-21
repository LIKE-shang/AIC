"""Fixed-height, fixed-heading straight passage. Pure Python; no hardware IO.

The whole box is swept along the current heading, projected through BOTH
faces of the measured frame slab, then checked against every polygon edge.
A decision never authorizes a controller or claims unknown space is free.
"""
import math
from collections import deque
from window_pass.geometry import add,sub,scale,dot,norm,rotate,yaw,quaternion,vector
from envelope_fence import envelope_inside
from live_geometry import validate_metric
from attitude_envelope import support,nominal_support,enclosed,spatial_budget as geometry_spatial_budget


def cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


class RouteRejection(ValueError):
    """Keep the geometric evidence without changing the rejection condition."""
    def __init__(self,reason,diagnostics):
        super().__init__(reason)
        self.diagnostics=diagnostics


def straight_route(status,control,pose,ros,wall,received,after_stamp):
    c=control
    if not -.02<=ros-pose.stamp<=c['pose_timeout_s']:raise ValueError('straight_pose_stale')
    if not quaternion(pose.q) or not vector(pose.p):raise ValueError('invalid_route_pose')
    # Same measured geometry gates as the existing RGB-D pipeline; it already
    # confirms at least five depth frames. No second legacy RGB vote is used.
    metric=status.get('metric_geometry') or {}
    if not isinstance(metric,dict):raise ValueError('invalid_metric_geometry')
    raw_normal=metric.get('normal_enu')
    window_heading=math.atan2(raw_normal[1],raw_normal[0]) if vector(raw_normal) else yaw(pose.q)
    # The old alignment-facing angle is not a straight-route requirement.
    # Obliqueness is checked by the actual swept projection below.
    g=validate_metric(status,dict(c,window_yaw_rad=window_heading),ros,wall,received,after_stamp)
    return route_from_geometry(c,pose,g,metric)


def route_from_geometry(c,pose,g,metric):
    """Pure sweep of already validated geometry (live or session snapshot).

    Caller must enforce current pose/session health. This function does not
    alter timestamps or label captured geometry as a current observation.
    """
    if not quaternion(pose.q) or not vector(pose.p):raise ValueError('invalid_route_pose')
    center=tuple(g['center']);n=tuple(metric['normal_enu'])
    heading=yaw(pose.q);direction=(math.cos(heading),math.sin(heading),0.)
    n=scale(n,1./norm(n))
    if dot(n,direction)<0:n=scale(n,-1.)
    denominator=dot(n,direction)
    if denominator<c['min_ray_plane_cos']:raise ValueError('straight_heading_oblique_to_window')
    u=cross((0.,0.,1.),n);u=scale(u,1./norm(u));v=cross(n,u)
    polygon=[(dot(sub(p,center),u),dot(sub(p,center),v)) for p in metric['corners_enu_m']]
    area=sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(polygon,polygon[1:]+polygon[:1]))
    if area<0:polygon.reverse()
    half=[x/2 for x in c['vehicle_size_m']]
    level_heading_q=(0.,0.,math.sin(heading/2.),math.cos(heading/2.))
    offsets=[rotate(level_heading_q,add(c['envelope_center_body_m'],(a*half[0],b*half[1],d*half[2])))
             for a in (-1,1) for b in (-1,1) for d in (-1,1)]
    # Project each edge normal through the current travel direction. The
    # directional attitude support contains the entire retained yaw/tilt range;
    # it no longer charges the largest spherical motion to every window edge.
    spatial_budget=geometry_spatial_budget(c)
    slab=c['window_thickness_m']/2.
    distance=dot(sub(center,pose.p),n)
    normal_extent=max(support(c,heading,n),support(c,heading,scale(n,-1.)))+spatial_budget
    if not c['approach_min_m']<=distance<=c['approach_max_m']:raise ValueError('straight_approach_distance')
    if distance<=normal_extent+slab:raise ValueError('already_inside_window_slab')
    margins=[];edge_details=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        edge=(b[0]-a[0],b[1]-a[1]);length=math.hypot(*edge)
        if length<1e-6:raise ValueError('degenerate_aperture_edge')
        inward=(-edge[1]/length,edge[0]/length)
        edge_world=add(scale(u,inward[0]),scale(v,inward[1]))
        effective=sub(edge_world,scale(n,dot(edge_world,direction)/denominator))
        center_gap=min(dot(edge_world,sub(add(pose.p,scale(direction,(distance+face)/denominator)),center))
                       -inward[0]*a[0]-inward[1]*a[1] for face in (-slab,slab))
        negative=scale(effective,-1.)
        nominal=nominal_support(c,heading,negative)
        attitude_extra=max(0.,support(c,heading,negative)-nominal)
        aperture_clearance=c.get('_session_aperture_clearance_margin_m',c['clearance_margin_m'])
        projected_budget=spatial_budget*norm(effective)+attitude_extra+aperture_clearance
        raw_clearance=center_gap-nominal
        margin=raw_clearance-projected_budget
        margins.append(margin)
        side=('left' if inward[0]>0 else 'right') if abs(inward[0])>abs(inward[1]) else ('bottom' if inward[1]>0 else 'top')
        edge_details.append(dict(side_in_window_basis=side,edge_start_uv_m=list(a),edge_end_uv_m=list(b),
            translation_gradient_local=list(effective),attitude_inflation_m=attitude_extra,
            position_and_tracking_projection_m=spatial_budget*norm(effective),
            raw_box_clearance_m=raw_clearance,required_budget_m=projected_budget,
            clearance_after_budget_m=margin))
    projected_budget=max(e['required_budget_m'] for e in edge_details)
    attitude_inflation=max(e['attitude_inflation_m'] for e in edge_details)
    if min(margins)<=0:
        raise RouteRejection('fixed_height_heading_box_would_clip_window',dict(
            geometry_stamp=g['stamp'],pose_stamp=pose.stamp,pose_local_m=list(pose.p),
            pose_quaternion_xyzw=list(pose.q),window_center_local_m=list(center),
            window_corners_local_m=metric['corners_enu_m'],window_normal=list(n),
            window_basis_u=list(u),window_basis_v=list(v),direction=list(direction),
            position_uncertainty_m=(0. if '_session_cylinder_radius_m' in c else c['position_uncertainty_m']),
            tracking_budget_m=(0. if '_session_cylinder_radius_m' in c else c['clearance_tracking_budget_m']),
            configured_position_uncertainty_m=c['position_uncertainty_m'],live_cross_track_abort_m=c.get('_session_pass_cross_track_limit_m',c['clearance_tracking_budget_m']),
            attitude_inflation_m=attitude_inflation,
            clearance_margin_m=c.get('_session_aperture_clearance_margin_m',c['clearance_margin_m']),
            ray_plane_cos=denominator,projected_error_budget_m=projected_budget,
            edges=edge_details,minimum_clearance_after_budget_m=min(margins),
            input_accuracy_verified=metric.get('metric_accuracy_verified',False)))
    travel=(distance+normal_extent+slab+c['post_plane_clearance_m']+c['stopping_margin_m'])/denominator
    minimum_clear_travel=travel
    requested=c.get('_session_fixed_pass_distance_m')
    if requested is not None:
        if requested<minimum_clear_travel:raise ValueError('requested_distance_insufficient_to_clear_window')
        travel=requested
    if travel>c['max_traverse_distance_m']:raise ValueError('straight_route_too_long')
    endpoint=add(pose.p,scale(direction,travel))
    # A convex fence encloses the full uncertainty-expanded segment.
    for p in (pose.p,endpoint):
        if not envelope_inside(c,p,pose.q):raise ValueError('straight_swept_box_outside_fence')
        if not enclosed(c,p,heading):raise ValueError('straight_uncertainty_envelope_outside_fence')
    from smooth_pass import trajectory
    from pass_terminal import BRAKE_RESERVE
    fast_length=travel-BRAKE_RESERVE if c.get('_session_terminal_capture') else travel
    smooth=trajectory(fast_length) if c.get('_session_smooth_pass') else None
    duration=(smooth['duration_s']+(BRAKE_RESERVE/.15+12. if c.get('_session_terminal_capture') else 0.) if smooth else travel/c['traverse_speed_mps'])+c['stop_stable_time_s']
    if duration>=c['max_traverse_time_s']:raise ValueError('straight_route_time_budget')
    return dict(kind='FIXED_HEIGHT_HEADING_FINITE_ROUTE',start=list(pose.p),endpoint=list(endpoint),
        direction=list(direction),height_local_m=pose.p[2],yaw_rad=heading,length_m=travel,
        terminal_capture_enabled=bool(c.get('_session_terminal_capture')),
        passage_geometry_tilt_limit_rad=c.get('_session_pass_tilt_limit_rad',c['tilt_limit_rad']),
        smooth_trajectory=smooth,
        command_speed_mps=smooth['peak_speed_mps'] if smooth else c['traverse_speed_mps'],maximum_forward_lead_m=.35 if smooth else c.get('_session_forward_lead_m',c['max_setpoint_lead_m']),
        minimum_clear_travel_m=minimum_clear_travel,requested_distance_m=requested,
        plane_distance_m=distance,epoch=list(g['epoch']),geometry_center=list(center),
        window_normal_local_m=list(n),minimum_center_exit_signed_distance_m=normal_extent+slab+c.get('_session_aperture_clearance_margin_m',c['clearance_margin_m']),
        geometry_stamp=g['stamp'],edge_clearance_after_budgets_m=margins,edge_diagnostics=edge_details,
        envelope_method=('horizontal_cylinder_continuous_tilt_plus_rigid_clearance' if '_session_cylinder_radius_m' in c else 'directional_continuous_yaw_tilt_support'),
        uncertainty_budget_verified=False,
        clearance_is_accuracy_bound=False,
        spatial_error_budget_m=spatial_budget,projected_error_budget_m=projected_budget,
        live_cross_track_abort_m=c.get('_session_pass_cross_track_limit_m',c['clearance_tracking_budget_m']),
        attitude_inflation_m=attitude_inflation,expected_duration_s=duration,
        corridor_verified=c['corridor_verified'],metric_accuracy_verified=metric.get('metric_accuracy_verified',False),
        decision_is_conditional=True,actual_output_permitted=False)


class PassageDecision:
    """Three feasible distinct frames among at most five, within one second.

    Missing detections do not erase recent good frames. The CURRENT frame must
    still be feasible; a current obstacle or bad route cannot be outvoted.
    """
    def __init__(self,required_frames=3,window_s=1.):
        if required_frames not in (2,3) or window_s not in (1.,1.2):raise ValueError('invalid vote policy')
        self.required_frames=required_frames;self.window_s=window_s
        self.rows=deque(maxlen=5);self.last_stamp=None;self.epoch=None
        self.reason='waiting_for_post_takeoff_geometry';self.plan=None;self.latest_safe=False
        self.events=deque(maxlen=60)
        self.final_rejection=False
        self.geometry_diagnostics=None

    def update(self,status,c,pose,ros,wall,received,after_stamp):
        self.plan=None;self.latest_safe=False;self.final_rejection=False
        while self.rows and ros-self.rows[0]['stamp']>self.window_s:self.rows.popleft()
        stamp=status.get('source_stamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):
            self.reason='no_current_metric_frame';return False
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.reason='duplicate_or_reversed_frame';return False
        self.last_stamp=stamp
        self.geometry_diagnostics=None
        current_epoch=((status.get('metric_geometry') or {}).get('pose_epoch'),(status.get('metric_geometry') or {}).get('calibration_epoch'))
        if all(type(e) is int and e>=0 for e in current_epoch):
            if self.epoch is not None and self.epoch!=current_epoch:self.rows.clear()
            self.epoch=current_epoch
        try:
            plan=straight_route(status,c,pose,ros,wall,received,after_stamp)
            # Current route is independently rechecked against older geometry
            # centers so votes cannot accumulate while the target moves.
            if any(row['safe'] and norm(sub(row['center'],plan['geometry_center']))>c['center_agreement_m'] for row in self.rows):self.rows.clear()
            self.rows.append(dict(stamp=stamp,safe=True,center=plan['geometry_center']))
            self.latest_safe=True
            if sum(row['safe'] for row in self.rows)>=self.required_frames:
                self.plan=plan;self.reason='straight_route_confirmed_'+str(self.required_frames)+'_of_5';return True
            self.reason='confirming_straight_route_'+str(self.required_frames)+'_of_5'
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            self.rows.append(dict(stamp=stamp,safe=False,center=None));self.reason=str(exc)
            self.geometry_diagnostics=getattr(exc,'diagnostics',None)
            self.final_rejection=self.reason in ('fixed_height_heading_box_would_clip_window','straight_heading_oblique_to_window','straight_approach_distance','already_inside_window_slab','straight_route_too_long','straight_swept_box_outside_fence','straight_uncertainty_envelope_outside_fence','straight_route_time_budget','opening_depth_evidence_required','window_not_facing_vehicle')
        finally:self.events.append(dict(ros=ros,reason=self.reason,safe=self.latest_safe))
        return False

    def snapshot(self):
        return dict(required_frames=self.required_frames,window_s=self.window_s,reason=self.reason,final_rejection=self.final_rejection,window_samples=len(self.rows),safe_samples=sum(r['safe'] for r in self.rows),
            latest_frame_safe=self.latest_safe,plan=self.plan,geometry_diagnostics=self.geometry_diagnostics,events=list(self.events))


def advance_route(plan,pose,command,c,dt,elapsed):
    if not 0<=dt<=c['max_loop_gap_s']:raise ValueError('loop_deadline')
    if elapsed>c['max_traverse_time_s']:raise ValueError('finite_path_timeout')
    start=tuple(plan['start']);direction=tuple(plan['direction']);length=plan['length_m']
    delta=sub(pose.p,start);progress=dot(delta,direction)
    cross_track=norm(sub(delta,scale(direction,progress)))
    limit=min(c.get('_session_pass_cross_track_limit_m',c['clearance_tracking_budget_m']),
              plan.get('live_cross_track_abort_m',c['clearance_tracking_budget_m']))
    from pass_terminal import END_OVERSHOOT,TERMINAL_ALONG_ERROR,CAPTURE_RADIUS,FINISH_REFERENCE_SPEED
    terminal=bool(c.get('_session_terminal_capture') and plan.get('terminal_capture_enabled'))
    body_clear=dot(sub(pose.p,plan['geometry_center']),plan['window_normal_local_m'])>=plan['minimum_center_exit_signed_distance_m']
    after_clear=terminal and body_clear
    if after_clear:limit=CAPTURE_RADIUS
    if cross_track>limit:raise ValueError('straight_cross_track_exceeds_geometry_budget')
    if '_session_cylinder_radius_m' in c and not after_clear:
        # The frozen aperture halfplanes apply to the translated travel ray.
        # Their gradients are orthogonal to travel, so along-route progress
        # does not consume sideways clearance. Each margin already includes
        # the whole allowed cylinder attitude range and 4cm rigid clearance.
        error=sub(delta,scale(direction,progress))
        edges=plan.get('edge_diagnostics')
        if not isinstance(edges,list) or len(edges)!=4:raise ValueError('frozen_aperture_edges_missing')
        for edge in edges:
            w=edge.get('translation_gradient_local');margin=edge.get('clearance_after_budget_m')
            if not vector(w) or type(margin) not in (int,float) or not math.isfinite(margin):
                raise ValueError('frozen_aperture_edge_invalid')
            if margin+dot(w,error)<=0:raise ValueError('straight_live_aperture_clearance_exhausted')
    if progress< -c['max_tracking_error_m'] or progress>length+(END_OVERSHOOT if terminal else .15 if plan.get('smooth_trajectory') else c['arrival_tolerance_m']):raise ValueError('straight_progress_outside_route')
    if not envelope_inside(c,pose.p,pose.q):raise ValueError('actual_body_outside_fence')
    if plan.get('smooth_trajectory'):
        from smooth_pass import sample,ALONG_ERROR
        target,speed,acceleration=sample(plan['smooth_trajectory'],elapsed)
        if terminal and elapsed>=plan['smooth_trajectory']['duration_s']:
            old=dot(sub(command,start),direction)
            target=min(length,old+FINISH_REFERENCE_SPEED*dt)
        allowed_error=TERMINAL_ALONG_ERROR if after_clear else ALONG_ERROR
        if abs(target-progress)>allowed_error:raise ValueError('smooth_pass_along_tracking_error')
        candidate=add(start,scale(direction,target))
        if not envelope_inside(c,candidate,pose.q):raise ValueError('target_body_outside_fence')
        arrived=elapsed>=plan['smooth_trajectory']['duration_s'] and norm(sub(pose.p,plan['endpoint']))<=c['arrival_tolerance_m']
        return candidate,arrived
    old=dot(sub(command,start),direction)
    speed=min(c['traverse_speed_mps'],plan.get('command_speed_mps',c['traverse_speed_mps']))
    lead=min(c.get('_session_forward_lead_m',c['max_setpoint_lead_m']),plan.get('maximum_forward_lead_m',c['max_setpoint_lead_m']))
    # Forward lead and lateral/vertical error share the unchanged 3-D
    # tracking allowance. Do not request a target that exceeds it merely
    # because both separate directional limits would individually fit.
    if '_session_forward_lead_m' in c:
        available=math.sqrt(max(0.,c['max_tracking_error_m']**2-cross_track**2))
        lead=min(lead,max(0.,available-1e-9))
    target=max(old,min(length,old+speed*dt,progress+lead))
    candidate=add(start,scale(direction,target))
    if not envelope_inside(c,candidate,pose.q):raise ValueError('target_body_outside_fence')
    arrived=target>=length-1e-9 and norm(sub(pose.p,plan['endpoint']))<=c['arrival_tolerance_m']
    return candidate,arrived


from live_geometry import AirborneController

class StraightController(AirborneController):
    """Use RGB-D only. The legacy RGB observer cannot start/stop this mode."""
    def __init__(self,control,straight_commissioning=False,session_window=False):
        super().__init__(control,straight_commissioning=straight_commissioning,session_window=session_window)

    def pose_update(self,p,ros,wall):
        good=super().pose_update(p,ros,wall)
        if not good and '_session_cylinder_radius_m' in self.c:
            self.session_pose_fault='session_pose_discontinuity_latched'
        return good

    def inside(self,p):
        return self.pose is not None and envelope_inside(self.c,p,self.pose.q)

    def request_hold(self,*args):return False,'straight_pass_has_no_alignment_hold'
    def request_start(self,*args):return False,'straight_pass_has_no_alignment_start'
    def request_pass(self,*args):return False,'straight_pass_commits_only_validated_fixed_route'
