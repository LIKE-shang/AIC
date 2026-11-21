"""Directional enclosure of every box attitude allowed by yaw/tilt checks.

For Rz(psi) Ry(theta) Rx(phi), cos(theta)*cos(phi)>=cos(tilt).
Let a=cos(theta), b=cos(phi). The residual rotation trace is a+b+ab.
AM-GM gives trace>=2*sqrt(cos(tilt))+cos(tilt). This gives a cone
containing the full allowed residual rotation, including roll/pitch twist.
Maximize its support after the bounded yaw, separately for each query normal.
No discrete attitude sampling is used to authorize a route.
"""
import math
from window_pass.geometry import add,dot,norm,rotate


def body_corners(c):
    h=[x/2 for x in c['vehicle_size_m']]
    return [add(c['envelope_center_body_m'],(i*h[0],j*h[1],k*h[2]))
            for i in (-1,1) for j in (-1,1) for k in (-1,1)]


def support(c,heading,direction):
    """Upper bound max dot(direction, rotated body corner), in metres."""
    if '_session_cylinder_radius_m' in c:return cylinder_tilt_support(c,direction)
    co,si=math.cos(heading),math.sin(heading)
    w=(co*direction[0]+si*direction[1],-si*direction[0]+co*direction[1],direction[2])
    wn=norm(w)
    if wn==0:return 0.
    ct=math.cos(c['tilt_limit_rad'])
    beta=math.acos(max(-1.,min(1.,(2*math.sqrt(ct)+ct-1)/2)))
    cb,sb=math.cos(beta),math.sin(beta);y=c['yaw_limit_rad'];maximum=-math.inf
    for o in body_corners(c):
        a=w[0]*o[0]+w[1]*o[1];b=-w[0]*o[1]+w[1]*o[0];z=w[2]*o[2]
        angles=[-y,0.,y];optimum=math.atan2(b,a)
        if -y<=optimum<=y:angles.append(optimum)
        v=max(a*math.cos(t)+b*math.sin(t)+z for t in angles)
        radius=wn*norm(o)
        bound=radius if v>=radius*cb else v*cb+math.sqrt(max(0.,radius*radius-v*v))*sb
        maximum=max(maximum,bound)
    return maximum+1e-12


def nominal_support(c,heading,w):
    if "_session_cylinder_radius_m" in c:return cylinder_axis_support(c,w,(0.,0.,1.))
    q=(0.,0.,math.sin(heading/2),math.cos(heading/2))
    return max(dot(w,rotate(q,o)) for o in body_corners(c))


def enclosed(c,p,heading):
    reserve=spatial_budget(c)+c.get('_session_field_fence_margin_m',c['clearance_margin_m'])
    for i in range(3):
        axis=tuple(float(j==i) for j in range(3))
        slack=c.get('_session_horizontal_cone_slack_m',0.) if i<2 else 0.
        if p[i]+support(c,heading,axis)+reserve>c['geofence_max_m'][i]+slack:return False
        if p[i]-support(c,heading,tuple(-v for v in axis))-reserve<c['geofence_min_m'][i]-slack:return False
    return True


def spatial_budget(c):
    """Rigid session profile has no extra position/tracking geometry inflation.

    The live tracking limit remains a controller abort threshold, NOT a proven
    uncertainty bound. Four cm is physical clearance, not an accuracy claim.
    """
    return 0. if '_session_cylinder_radius_m' in c else c['position_uncertainty_m']+c['clearance_tracking_budget_m']


def cylinder_axis_support(c, direction, axis):
    axial=dot(direction,axis)
    return (c['envelope_center_body_m'][2]*axial + c['vehicle_size_m'][2]/2*abs(axial)
            + c['_session_cylinder_radius_m']*math.sqrt(max(0.,dot(direction,direction)-axial*axial)))


def cylinder_tilt_support(c,direction):
    """Exact support over all cylinder axes within the allowed vertical cone."""
    length=norm(direction)
    if length==0:return 0.
    z=direction[2];horizontal=math.hypot(*direction[:2]);angle=max(c['tilt_limit_rad'],c.get('_session_pass_tilt_limit_rad',c['tilt_limit_rad']))
    co,si=math.cos(angle),math.sin(angle)
    hi=length if z/length>=co else z*co+horizontal*si
    lo=-length if -z/length>=co else z*co-horizontal*si
    radius=c['_session_cylinder_radius_m'];half=c['vehicle_size_m'][2]/2
    values=[]
    for end in (c['envelope_center_body_m'][2]-half,c['envelope_center_body_m'][2]+half):
        optimum=max(lo,min(hi,length*end/math.hypot(radius,end)))
        values.append(end*optimum+radius*math.sqrt(max(0.,length*length-optimum*optimum)))
    return max(values)+1e-12
