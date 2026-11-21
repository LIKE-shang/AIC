"""Attitude-aware full vehicle box in the unchanged ENU fence."""
from window_pass.geometry import vector,quaternion,add,rotate

def envelope_margins(control,position,orientation):
    if not vector(position) or not quaternion(orientation):raise ValueError('invalid envelope pose')
    lo,hi=control['geofence_min_m'],control['geofence_max_m']
    if not vector(lo) or not vector(hi) or any(a>=b for a,b in zip(lo,hi)):raise ValueError('invalid envelope fence')
    if '_session_cylinder_radius_m' in control:
        from attitude_envelope import cylinder_axis_support
        axis=rotate(orientation,(0.,0.,1.));reserve=control.get('_session_field_fence_margin_m',control['clearance_margin_m'])
        directions=[tuple(float(i==j) for j in range(3)) for i in range(3)]
        return {'lower_m':[position[i]-cylinder_axis_support(control,tuple(-x for x in w),axis)-lo[i]-reserve for i,w in enumerate(directions)],
                'upper_m':[hi[i]-position[i]-cylinder_axis_support(control,w,axis)-reserve for i,w in enumerate(directions)]}
    reserve=control['position_uncertainty_m']+control.get('_session_field_fence_margin_m',control['clearance_margin_m'])
    half=[v/2 for v in control['vehicle_size_m']]
    corners=[add(position,rotate(orientation,add(control['envelope_center_body_m'],(a*half[0],b*half[1],c*half[2]))))
             for a in (-1,1) for b in (-1,1) for c in (-1,1)]
    return {'lower_m':[min(p[i] for p in corners)-lo[i]-reserve for i in range(3)],
            'upper_m':[hi[i]-max(p[i] for p in corners)-reserve for i in range(3)]}

def envelope_inside(control,position,orientation):
    try:return all(v>=0 for values in envelope_margins(control,position,orientation).values() for v in values)
    except (ValueError,TypeError,KeyError):return False
