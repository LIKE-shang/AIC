"""One acquisition and centering translation in a single localization session.

No IO and no saved-coordinate loader. The snapshot is held only in memory.
"""
import copy
import math
from collections import deque
from dataclasses import replace
from window_pass.geometry import add, sub, scale, dot, norm, yaw, rotate, vector
from live_geometry import validate_metric
from straight_pass import route_from_geometry, PassageDecision, RouteRejection
from attitude_envelope import enclosed, support, spatial_budget
from prepass_adjustment import settings as adjustment_settings, bounded_candidates


def settings(value=None):
    if value is None:return dict(enabled=False)
    required={'enabled','horizontal_radius_m','rigid_clearance_m'}
    optional={'aperture_clearance_m','field_bounded_alignment','alignment_max_distance_m','alignment_clear_column_radius_m','pass_cross_track_limit_m','fast_confirmation','relaxed_alignment','fixed_pass_distance_m','faster_pass','smooth_pass','pass_tilt_limit_rad','terminal_capture'}
    if not isinstance(value,dict) or not required<=set(value) or set(value)-required-optional:
        raise ValueError('explicit session-window profile required')
    if type(value['enabled']) is not bool:raise ValueError('invalid session-window switch')
    if value['horizontal_radius_m']!=.31 or value['rigid_clearance_m']!=.04:
        raise ValueError('session-window requires reported 31cm radius and 4cm clearance')
    aperture=value.get('aperture_clearance_m',value['rigid_clearance_m'])
    if type(aperture) not in (int,float) or not math.isfinite(aperture) or not .03<=aperture<=.04:
        raise ValueError('session aperture clearance must be 3 to 4cm')
    result=dict(field_bounded_alignment=False,terminal_capture=False,pass_tilt_limit_rad=None,smooth_pass=False,faster_pass=False,fixed_pass_distance_m=None,relaxed_alignment=False,fast_confirmation=False,alignment_max_distance_m=.20,alignment_clear_column_radius_m=.20,pass_cross_track_limit_m=.03);result.update(value)
    for key,upper in [('alignment_max_distance_m',.50),('alignment_clear_column_radius_m',.60)]:
        number=result[key]
        if type(number) not in (int,float) or not math.isfinite(number) or not .20<=number<=upper:
            raise ValueError('invalid session alignment limit:'+key)
    if type(result['field_bounded_alignment']) is not bool:raise ValueError('field alignment must be boolean')
    if result['field_bounded_alignment'] and not (result['smooth_pass'] and result['relaxed_alignment']):raise ValueError('field alignment requires relaxed smooth session')
    if type(result['smooth_pass']) is not bool:raise ValueError('smooth pass must be boolean')
    if result['smooth_pass'] and (not result['faster_pass'] or result['fixed_pass_distance_m']!=3.):raise ValueError('smooth pass requires explicit 3m session profile')
    if type(result['terminal_capture']) is not bool or (result['terminal_capture'] and not result['smooth_pass']):raise ValueError('terminal capture requires smooth session')
    angle=result['pass_tilt_limit_rad']
    if angle is not None:
        if not result['smooth_pass'] or type(angle) not in (int,float) or not math.isfinite(angle) or not math.radians(8)<=angle<=math.radians(15):raise ValueError('scoped pass tilt must be 8 to 15 degrees in smooth session')
    if type(result['faster_pass']) is not bool:raise ValueError('faster pass must be boolean')
    if type(result['relaxed_alignment']) is not bool:raise ValueError('relaxed alignment must be boolean')
    if type(result['fast_confirmation']) is not bool:raise ValueError('fast confirmation must be boolean')
    if result['fixed_pass_distance_m'] is not None and (type(result['fixed_pass_distance_m']) not in (int,float) or result['fixed_pass_distance_m']!=3.):
        raise ValueError('fixed pass profile must explicitly request 3 metres')
    limit=result['pass_cross_track_limit_m']
    if type(limit) not in (int,float) or not math.isfinite(limit) or not .03<=limit<=.08:
        raise ValueError('invalid session pass cross-track limit')
    return result


def alignment_policy(profile):
    p=adjustment_settings()
    distance=profile.get('alignment_max_distance_m',.20)
    p.update(enabled=True,max_distance_m=distance,
        max_path_m=.75 if distance>.20 else .30,timeout_s=15. if distance>.20 else 8.)
    p['cumulative_path_gate_enabled']=not profile.get('relaxed_alignment',False)
    p['arrival_speed_gate_enabled']=not profile.get('relaxed_alignment',False)
    if profile.get('relaxed_alignment',False):p.update(arrival_tolerance_m=.06,settle_s=0.)
    if profile.get('field_bounded_alignment'):
        p.update(field_bounded_alignment=True,max_distance_m=None,max_path_m=None,timeout_s=30.,cumulative_path_gate_enabled=False)
    return p


def decorate(c,profile):
    # Called only AFTER the shared controller has validated its normal config.
    if profile['enabled']:
        if any(abs(x)>1e-9 for x in c['envelope_center_body_m'][:2]):
            raise ValueError('cylinder requires centered horizontal body offset')
        if profile.get('faster_pass',False):c['_session_forward_lead_m']=.12
        if profile.get('smooth_pass',False):c['_session_smooth_pass']=True
        if profile.get('terminal_capture',False):c['_session_terminal_capture']=True
        if profile.get('pass_tilt_limit_rad') is not None:
            c['_session_pass_tilt_limit_rad']=profile['pass_tilt_limit_rad']
        if profile.get('fixed_pass_distance_m') is not None:c['_session_fixed_pass_distance_m']=profile['fixed_pass_distance_m']
        if profile.get('fast_confirmation',False):c['_session_fast_confirmation']=True
        c['_session_cylinder_radius_m']=profile['horizontal_radius_m']
        c['_session_pass_cross_track_limit_m']=profile.get('pass_cross_track_limit_m',.03)
        c['clearance_margin_m']=profile['rigid_clearance_m']
        # Aperture-only reserve; keep the field fence and vehicle envelope unchanged.
        c['_session_aperture_clearance_margin_m']=profile.get('aperture_clearance_m',profile['rigid_clearance_m'])
    return c


class Acquisition:
    def __init__(self,identity,fast_confirmation=False,tracking_reserve_m=0.,clearance_relaxation_m=0.,maximize_minimum_clearance=False,horizontal_cone_slack_m=0.):
        if not isinstance(identity,dict) or not identity.get('boot_id') or not identity.get('run_id'):
            raise ValueError('session-window requires current boot/run identity')
        if type(tracking_reserve_m) not in (int,float) or not math.isfinite(tracking_reserve_m) or tracking_reserve_m<0:
            raise ValueError('invalid session alignment tracking reserve')
        if type(clearance_relaxation_m) not in (int,float) or not math.isfinite(clearance_relaxation_m) or not 0.<=clearance_relaxation_m<=.01:
            raise ValueError('invalid scoped aperture clearance relaxation')
        if type(maximize_minimum_clearance) is not bool or (clearance_relaxation_m and not tracking_reserve_m):
            raise ValueError('aperture clearance relaxation requires second-window tracking reserve')
        if type(horizontal_cone_slack_m) not in (int,float) or not math.isfinite(horizontal_cone_slack_m) or not 0.<=horizontal_cone_slack_m<=.02:
            raise ValueError('invalid horizontal cone slack')
        if horizontal_cone_slack_m and not (tracking_reserve_m and clearance_relaxation_m):
            raise ValueError('horizontal cone slack requires second-window scope')
        self.horizontal_cone_slack_m=horizontal_cone_slack_m
        self.tracking_reserve_m=tracking_reserve_m
        self.maximize_minimum_clearance=maximize_minimum_clearance
        self.clearance_relaxation_m=clearance_relaxation_m
        self.identity=copy.deepcopy(identity);self.fast_confirmation=fast_confirmation;self.decision=self.new_decision()
        self.snapshot=None;self.plan=None;self.reason='waiting_for_trusted_geometry'
        self.candidate_evidence=None;self.candidate_history=deque(maxlen=40)

    def scoped_control(self,c):
        if not self.clearance_relaxation_m:return c
        scoped=dict(c)
        margin=max(0.,c['clearance_margin_m']-self.clearance_relaxation_m)
        scoped['_session_aperture_clearance_margin_m']=margin
        scoped['_session_field_fence_margin_m']=margin
        # Only the hypothetical tilt cone gets horizontal tolerance.
        # Actual-attitude field checks and aperture support keep their budgets.
        if self.horizontal_cone_slack_m:
            scoped['_session_horizontal_cone_slack_m']=self.horizontal_cone_slack_m
        return scoped

    def new_decision(self):
        return PassageDecision(required_frames=2,window_s=1.2) if self.fast_confirmation else PassageDecision()

    def update(self,status,c,pose,ros,wall,received,after,ground,minimum,origin,column,policy=None,reference=None):
        """Vote on a feasible route nearest the measured centre within the scope."""
        if self.snapshot is not None:raise ValueError('session_geometry_already_acquired')
        stamp=status.get('source_stamp')
        if type(stamp) in (int,float) and self.decision.last_stamp is not None and stamp<=self.decision.last_stamp:
            return False
        metric=status.get('metric_geometry') or {}
        normal=metric.get('normal_enu') if isinstance(metric,dict) else None
        angle=math.atan2(normal[1],normal[0]) if vector(normal) else yaw(pose.q)
        g=validate_metric(status,dict(c,window_yaw_rad=angle),ros,wall,received,after)
        heading=yaw(pose.q);left=(-math.sin(heading),math.cos(heading),0.)
        offset=rotate((0,0,math.sin(heading/2),math.cos(heading/2)),c['envelope_center_body_m'])
        delta=sub(g['center'],add(pose.p,offset))
        target=add(pose.p,add(tuple(dot(delta,left)*x for x in left),(0.,0.,delta[2])))
        policy=policy or alignment_policy({})
        self.candidate_evidence=dict(source_stamp=g['stamp'],pose_stamp=pose.stamp,
            pose_local_m=list(pose.p),pose_quaternion_xyzw=list(pose.q),
            ground_z_local_m=ground,minimum_bottom_height_m=minimum,
            geometry=copy.deepcopy(metric),center_height_above_ground_m=g['center'][2]-ground,
            exact_center_target_local_m=list(target),
            exact_center_worst_bottom_height_m=target[2]-support(c,heading,(0,0,-1))-ground,
            exact_center_delta_m=list(sub(target,pose.p)),
            rigid_clearance_m=c['clearance_margin_m'],
            aperture_clearance_m=self.scoped_control(c).get('_session_aperture_clearance_margin_m',c['clearance_margin_m']),
            aperture_clearance_relaxation_m=self.clearance_relaxation_m,
            effective_spatial_budget_m=spatial_budget(c))
        preferred=target if reference is None or self.tracking_reserve_m else (target[0],target[1],reference[2])
        self.candidate_evidence['previous_command_local_m']=list(reference) if reference is not None else None
        scoped_c=self.scoped_control(c)
        try:
            target=select_target(scoped_c,pose,g,metric,preferred,ground,minimum,origin,column,policy,self.candidate_evidence,self.tracking_reserve_m,
                maximize_minimum_clearance=self.maximize_minimum_clearance)
        except (ValueError,TypeError,KeyError,ZeroDivisionError) as exc:
            self.candidate_evidence['reason']=str(exc)
            raise
        finally:
            if not self.candidate_history or self.candidate_history[-1]['source_stamp']!=g['stamp']:
                self.candidate_history.append(copy.deepcopy(self.candidate_evidence))
        distance=norm(sub(target,pose.p))
        # Stable corners and normal, not only a stable centre, are needed for
        # a single trusted acquisition. Epoch changes reset the votes.
        previous=getattr(self,'previous_geometry',None)
        if previous is not None and (previous['pose_epoch']!=metric['pose_epoch'] or
                previous['calibration_epoch']!=metric['calibration_epoch'] or
                any(norm(sub(a,b))>c['center_agreement_m'] for a,b in zip(previous['corners_enu_m'],metric['corners_enu_m'])) or
                abs(dot(previous['normal_enu'],metric['normal_enu']))<math.cos(c['yaw_limit_rad'])):
            self.decision=self.new_decision()
        self.previous_geometry=copy.deepcopy(metric)
        hypothetical=replace(pose,p=target)
        if not self.decision.update(status,scoped_c,hypothetical,ros,wall,received,after):
            self.reason=self.decision.reason
            if self.decision.final_rejection:raise ValueError(self.reason)
            return False
        # The latest, independently feasible frame supplies all snapshot data.
        self.snapshot=dict(identity=copy.deepcopy(self.identity),epoch=list(g['epoch']),
            geometry=copy.deepcopy(status['metric_geometry']),validated_geometry=copy.deepcopy(g),status=copy.deepcopy(status),
            captured_ros=ros,captured_wall=wall,after_stamp=after,received=received)
        self.plan=dict(start=list(pose.p),target=list(target),yaw_rad=heading,length_m=distance,
            proposed_route=self.decision.plan,epoch=list(g['epoch']),normal=list(metric['normal_enu']),command_start=list(reference) if reference is not None else list(pose.p),selection=copy.deepcopy(self.candidate_evidence))
        self.reason='session_geometry_frozen_after_'+str(self.decision.required_frames)+'_of_5'
        return True

    def retarget(self,c,pose,reference,ground,minimum,origin,policy):
        if not policy.get('field_bounded_alignment') or self.snapshot is None:raise ValueError('field retarget requires acquired snapshot')
        g=self.snapshot['validated_geometry'];metric=self.snapshot['geometry']
        heading=self.plan['yaw_rad'];q=(0.,0.,math.sin(heading/2),math.cos(heading/2))
        held=replace(pose,q=q);left=(-math.sin(heading),math.cos(heading),0.)
        offset=rotate(q,c['envelope_center_body_m']);delta=sub(g['center'],add(pose.p,offset))
        preferred=add(pose.p,tuple(dot(delta,left)*x for x in left))
        preferred=(preferred[0],preferred[1],pose.p[2]+delta[2] if self.tracking_reserve_m else reference[2])
        evidence=dict(exact_center_target_local_m=list(preferred))
        scoped_c=self.scoped_control(c)
        target=select_target(scoped_c,held,g,metric,preferred,ground,minimum,origin,None,policy,evidence,self.tracking_reserve_m,
            maximize_minimum_clearance=self.maximize_minimum_clearance)
        self.plan=dict(self.plan,start=list(pose.p),target=list(target),length_m=norm(sub(target,pose.p)),selection=evidence)
        return self.plan

    def reject_frame(self,status,ros,reason):
        """Failed acquisition frames occupy the same latest-five vote window."""
        d=self.decision;stamp=status.get('source_stamp')
        while d.rows and ros-d.rows[0]['stamp']>d.window_s:d.rows.popleft()
        if type(stamp) not in (int,float) or not math.isfinite(stamp):return
        if d.last_stamp is not None and stamp<=d.last_stamp:return
        d.last_stamp=stamp;d.plan=None;d.latest_safe=False;d.reason=reason
        d.rows.append(dict(stamp=stamp,safe=False,center=None))
        d.events.append(dict(ros=ros,reason=reason,safe=False))

    def route(self,c,pose,reference=None):
        """Validate the actual departure ray against the immutable snapshot.

        Current pose freshness and localization continuity are checked by
        Mission. Captured geometry timestamps remain historical evidence.
        """
        s=self.snapshot
        scoped_c=self.scoped_control(c)
        result=route_from_geometry(scoped_c,pose,s['validated_geometry'],s['geometry'])
        self.check_departure_reserve(result)
        if reference is not None:
            # Validate actual departure AND retained height ray; keep XY at the
            # current feedback so no sudden horizontal return is requested.
            planned=replace(pose,p=(pose.p[0],pose.p[1],reference[2]))
            result=route_from_geometry(scoped_c,planned,s['validated_geometry'],s['geometry'])
            self.check_departure_reserve(result)
            from straight_pass import advance_route
            advance_route(result,pose,planned.p,c,0.,0.)
            result['actual_departure_position_local_m']=list(pose.p)
            result['retained_height_reference_m']=reference[2]
        result['session_identity']=copy.deepcopy(s['identity'])
        result['aperture_clearance_m']=scoped_c.get('_session_aperture_clearance_margin_m',c['clearance_margin_m'])
        result['aperture_clearance_relaxation_m']=self.clearance_relaxation_m
        result['horizontal_cone_slack_m']=self.horizontal_cone_slack_m
        result['target_selection_policy']='maximin_minimum_four_edge_clearance' if self.maximize_minimum_clearance else 'nearest_feasible_to_window_center'
        result['geometry_epoch']=list(s['epoch'])
        result['geometry_source']='immutable_session_snapshot'
        result['departure_pose_stamp']=pose.stamp
        return result

    def check_departure_reserve(self,plan):
        if self.tracking_reserve_m:
            for edge in plan['edge_diagnostics']:
                required=self.tracking_reserve_m*norm(edge['translation_gradient_local'])
                if edge['clearance_after_budget_m']<required-1e-9:
                    raise ValueError('second_window_departure_tracking_reserve:'+edge['side_in_window_basis'])
            plan['departure_tracking_reserve_m']=self.tracking_reserve_m


def localization_identity(identity):
    if not isinstance(identity,dict):return None
    return {k:v for k,v in identity.items() if k!='/window_live/status'}


def column_tracking_budget(c):
    # Rigid session geometry explicitly has no extra unverified spatial error
    # budget (see attitude_envelope.spatial_budget). The actual 20cm centre
    # column remains enforced; do not silently shrink it to17cm only here.
    return 0. if '_session_cylinder_radius_m' in c else c['clearance_tracking_budget_m']


def validate_motion(plan,c,pose,ground,minimum,origin,column,policy):
    """Use the captured opening only; no live camera input during centering."""
    if not policy.get('field_bounded_alignment') and norm(sub(pose.p,plan['start']))>policy['max_distance_m']+(c['clearance_tracking_budget_m'] if policy.get('cumulative_path_gate_enabled',True) else 0.):
        raise ValueError('session_alignment_displacement_limit')
    if not policy.get('field_bounded_alignment') and norm(sub(pose.p,origin)[:2])+column_tracking_budget(c)>column:
        raise ValueError('session_alignment_outside_confirmed_column')
    if not enclosed(c,pose.p,plan['yaw_rad']) or not enclosed(c,plan['target'],plan['yaw_rad']):
        raise ValueError('session_alignment_outside_fence')
    if plan['target'][2]-support(c,plan['yaw_rad'],(0,0,-1))-ground<minimum-1e-10:
        raise ValueError('session_alignment_height_floor')
    route=plan['proposed_route'];n=plan['normal'];center=route['geometry_center']
    for point in (pose.p,plan['target']):
        if not c['approach_min_m']<=abs(dot(sub(center,point),n))<=c['approach_max_m']:
            raise ValueError('session_alignment_approach_distance')


def select_target(c,pose,g,metric,preferred,ground,minimum,origin,column,policy,evidence,tracking_reserve_m=0.,maximize_minimum_clearance=False):
    """Keep the centre as a preference, with the floor and aperture as bounds."""
    heading=yaw(pose.q);left=(-math.sin(heading),math.cos(heading),0.);up=(0.,0.,1.)
    if not enclosed(c,pose.p,heading):raise ValueError('alignment_start_outside_fence')
    # Existing sweep provides the aperture halfplanes, including obliqueness,
    # thickness, cylinder tilt occupancy, and the unchanged rigid clearance.
    try:
        current=route_from_geometry(c,pose,g,metric)
        edges=current['edge_diagnostics']
    except RouteRejection as exc:
        edges=exc.diagnostics['edges']
    evidence['current_ray_edges']=copy.deepcopy(edges)
    preferred_delta=sub(preferred,pose.p)
    preference=(dot(preferred_delta,left),preferred_delta[2])
    constraints=[]
    for edge in edges:
        w=edge['translation_gradient_local']
        # Second opening reserves the existing tracking budget in every edge
        # normal. A numerical epsilon alone cannot absorb real feedback error.
        required=tracking_reserve_m*norm(w)+1e-6
        constraints.append((dot(w,left),w[2],required-edge['clearance_after_budget_m']))
    vertical=policy['max_vertical_m']
    constraints.extend(((0.,1.,-vertical),(0.,-1.,-vertical)))
    floor_z=ground+minimum+support(c,heading,(0.,0.,-1.))
    constraints.append((0.,1.,floor_z-pose.p[2]))
    reserve=spatial_budget(c)+c.get('_session_field_fence_margin_m',c['clearance_margin_m'])
    for i in range(3):
        axis=tuple(float(j==i) for j in range(3))
        lower=c['geofence_min_m'][i]+support(c,heading,tuple(-x for x in axis))+reserve-pose.p[i]
        upper=c['geofence_max_m'][i]-support(c,heading,axis)-reserve-pose.p[i]
        slack=c.get('_session_horizontal_cone_slack_m',0.) if i<2 else 0.
        lower-=slack;upper+=slack
        constraints.extend(((left[i],up[i],lower),(-left[i],-up[i],-upper)))
        if tracking_reserve_m and c.get('_session_fixed_pass_distance_m'):
            # Include the fixed route endpoint in candidate generation, not
            # only in the final validator. Otherwise a valid interior point
            # can be missed when the preferred centre exits the field fence.
            shift=c['_session_fixed_pass_distance_m']*(math.cos(heading),math.sin(heading),0.)[i]
            constraints.extend(((left[i],up[i],lower-shift),(-left[i],-up[i],-upper+shift)))
    # Project the unchanged surveyed horizontal clear-column disk into the
    # lateral translation axis (no forward motion or heading change).
    delta=sub(pose.p,origin);forward=(math.cos(heading),math.sin(heading),0.)
    if not policy.get('field_bounded_alignment'):
        usable=column-column_tracking_budget(c)
        forward_offset=dot(delta,forward);lateral_offset=dot(delta,left)
        if usable<=0 or abs(forward_offset)>usable:raise ValueError('alignment_outside_confirmed_column')
        lateral_span=math.sqrt(max(0.,usable*usable-forward_offset*forward_offset))
        constraints.extend(((1.,0.,-lateral_span-lateral_offset),(-1.,0.,lateral_offset-lateral_span)))
        radius=min(policy['max_distance_m'],policy['max_path_m'],c['max_align_distance_m'],
            min(policy['speed_mps'],c['align_speed_mps'])*(min(policy['timeout_s'],c['max_align_time_s'])-policy['settle_s']))
    else:
        # Every candidate still satisfies the full-body fence/aperture/floor
        # halfplanes above. No extra circle about the takeoff origin is needed.
        radius=norm(sub(c['geofence_max_m'],c['geofence_min_m']))
    selection_policy='maximin_minimum_four_edge_clearance' if maximize_minimum_clearance else 'nearest_feasible_to_window_center'
    evidence.update(selection_policy=selection_policy,planned_floor_z_local_m=floor_z,
        maximum_translation_m=radius,maximum_vertical_translation_m=vertical,
        exact_center_is_required=False,search_candidates=0)
    if tracking_reserve_m:evidence['departure_tracking_reserve_m']=tracking_reserve_m
    objective_edges=edges if maximize_minimum_clearance else []
    objective_lines=[]
    for i,first in enumerate(objective_edges):
        wi=first['translation_gradient_local'];ai=dot(wi,left);bi=wi[2];mi=first['clearance_after_budget_m']
        for second in objective_edges[i+1:]:
            wj=second['translation_gradient_local'];aj=dot(wj,left);bj=wj[2];mj=second['clearance_after_budget_m']
            objective_lines.append((ai-aj,bi-bj,mj-mi))
    candidates=bounded_candidates(constraints+objective_lines,preference,radius)
    def candidate_score(xy):
        lateral,vertical=xy
        return min(edge['clearance_after_budget_m']+
            dot(edge['translation_gradient_local'],add(scale(left,lateral),(0.,0.,vertical))) for edge in objective_edges)
    key=(lambda xy:(-candidate_score(xy),math.hypot(xy[0]-preference[0],xy[1]-preference[1]))) if maximize_minimum_clearance else (lambda xy:math.hypot(xy[0]-preference[0],xy[1]-preference[1]))
    for lateral,vertical in sorted(candidates,key=key):
        if any(a*lateral+b*vertical<d-1e-10 for a,b,d in constraints):continue
        distance=math.hypot(lateral,vertical)
        if distance>radius+1e-10:continue
        target=add(pose.p,add(tuple(lateral*x for x in left),(0.,0.,vertical)))
        if target[2]-support(c,heading,(0,0,-1))-ground<minimum-1e-10:continue
        if not enclosed(c,target,heading):continue
        evidence['search_candidates']+=1
        try:plan=route_from_geometry(c,replace(pose,p=target),g,metric)
        except (ValueError,TypeError,KeyError,ZeroDivisionError):continue
        evidence.update(reason='feasible_aperture_target',selected_target_local_m=list(target),
            selected_worst_bottom_height_m=target[2]-support(c,heading,(0,0,-1))-ground,
            selected_delta_m=list(sub(target,pose.p)),selected_edges=plan['edge_diagnostics'],
            selected_minimum_edge_clearance_m=min(edge['clearance_after_budget_m'] for edge in plan['edge_diagnostics']),
            offset_from_exact_center_m=norm(sub(target,evidence['exact_center_target_local_m'])),
            offset_from_preferred_target_m=norm(sub(target,preferred)))
        return target
    raise ValueError('no_feasible_aperture_target_within_floor_and_scope')


def advance_alignment(plan,c,pose,command,dt,elapsed,policy):
    """Rate-limit a bounded point adjustment from the preceding command.

    The clear column contains the motion. Alignment has the existing 12cm
    target tracking limit, not the 3cm straight PASS cross-track limit.
    """
    if not 0<=dt<=c['max_loop_gap_s']:raise ValueError('loop_deadline')
    if elapsed>=min(policy['timeout_s'],c['max_align_time_s']):raise ValueError('adjustment_timeout')
    if norm(sub(command,pose.p))>c['max_tracking_error_m']:raise ValueError('adjustment_target_tracking_error')
    delta=sub(plan['target'],command);length=norm(delta)
    step=min(length,min(policy['speed_mps'],c['align_speed_mps'])*dt)
    candidate=command if length<1e-12 else add(command,tuple(x*step/length for x in delta))
    # If a newly moved reference gets ahead of the available tracking allowance,
    # hold the preceding command rather than snapping it back to feedback.
    if norm(sub(candidate,pose.p))>c['max_tracking_error_m']:candidate=command
    if not enclosed(c,candidate,plan['yaw_rad']):raise ValueError('adjustment_setpoint_outside_fence')
    arrived=(norm(sub(candidate,plan['target']))<1e-9 and
             norm(sub(pose.p,plan['target']))<=policy['arrival_tolerance_m'])
    return candidate,arrived

