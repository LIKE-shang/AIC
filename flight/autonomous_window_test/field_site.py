"""Convert operator-measured boundaries to conservative local coordinates."""
import math
from window_pass.geometry import vector,quaternion,rotate,add

def bind_bounds(measurements,position,orientation,options,control):
    if not vector(position) or not quaternion(orientation):raise ValueError('invalid site pose')
    d=measurements['boundary_distances_m']
    if set(d)!={'front','back','left','right'}:raise ValueError('four boundary distances required')
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in d.values()):raise ValueError('invalid boundary distances')
    ceiling=measurements['ceiling_above_ground_m']
    if type(ceiling) not in (float,int) or not math.isfinite(ceiling) or ceiling<=0:raise ValueError('invalid physical ceiling')
    x,y,z=position
    forward=rotate(orientation,(1.,0.,0.))
    heading=math.atan2(forward[1],forward[0]);c,s=math.cos(heading),math.sin(heading)
    hx,hy=(d['front']+d['back'])/2,(d['left']+d['right'])/2
    cx,cy=(d['front']-d['back'])/2,(d['left']-d['right'])/2
    center=(x+c*cx-s*cy,y+s*cx+c*cy)
    # An enclosing world AABB would include unmeasured airspace. Use an
    # INSCRIBED AABB; all four corners satisfy the measured oriented rectangle.
    k=min(hx/(abs(c)*hx+abs(s)*hy),hy/(abs(s)*hx+abs(c)*hy))
    low=[center[0]-hx*k,center[1]-hy*k]
    high=[center[0]+hx*k,center[1]+hy*k]
    ground=z-options['ground_fcu_height_m']
    reserve=control['position_uncertainty_m']+control['clearance_margin_m']
    half=[v/2 for v in control['vehicle_size_m']]
    offsets=[rotate(orientation,add(control['envelope_center_body_m'],(a*half[0],b*half[1],t*half[2])))
             for a in (-1,1) for b in (-1,1) for t in (-1,1)]
    if '_session_cylinder_radius_m' in control:
        from attitude_envelope import cylinder_axis_support
        axis=rotate(orientation,(0.,0.,1.));reserve=control['clearance_margin_m']
        axes=[tuple(float(i==j) for j in range(3)) for i in range(3)]
        lows=[-cylinder_axis_support(control,tuple(-v for v in w),axis) for w in axes]
        highs=[cylinder_axis_support(control,w,axis) for w in axes]
        offsets=[(a,b,c) for a in (lows[0],highs[0]) for b in (lows[1],highs[1]) for c in (lows[2],highs[2])]
    # Ground binding accepts a finite stationary height span; the contact
    # allowance must cover that same bounded span plus numerical clearance.
    # This does not change the flight-height floor or command a lower target.
    contact_tolerance=options["ground_z_span_limit_m"]+.01
    low.append(min(ground,min(z+v[2] for v in offsets))-reserve-contact_tolerance)
    high.append(ground+min(ceiling,2.0))
    target=(x,y,ground+options['bottom_to_fcu_m']+options['target_bottom_height_m'])
    for reference in (position,target):
        for offset in offsets:
            p=add(reference,offset)
            if any(not low[i]+reserve<=p[i]<=high[i]-reserve for i in range(3)):
                raise ValueError('takeoff column does not fit the conservative measured fence')
    return dict(geofence_min_m=low,geofence_max_m=high,ground_z_local_m=ground,
                target_fcu_position_local_m=list(target),heading_rad=heading,
                physical_ceiling_above_ground_m=ceiling,software_envelope_ceiling_above_ground_m=min(ceiling,2.0),
                floor_bound_includes_ground_contact_uncertainty=True,
                floor_contact_tolerance_m=contact_tolerance)
