"""ROS-free geometry. ENU world, FLU body, RDF camera optical frame."""
import math


def finite(v):
    return type(v) in (float, int) and math.isfinite(v)


def vector(v, n=3):
    return isinstance(v, (list, tuple)) and len(v) == n and all(finite(x) for x in v)


def add(a, b): return tuple(x+y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def scale(a, k): return tuple(x*k for x in a)
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return scale(a, 1.0/norm(a))
def clamp(x, a, b): return max(a, min(b, x))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))


def quaternion(q):
    return vector(q, 4) and abs(norm(q)-1.0) <= 0.01


def rotate(q, v):
    # Normalise validated quaternion; input and output are not handedness-flipped.
    x, y, z, w = unit(q)
    return ((1-2*(y*y+z*z))*v[0]+2*(x*y-z*w)*v[1]+2*(x*z+y*w)*v[2],
            2*(x*y+z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-x*w)*v[2],
            2*(x*z-y*w)*v[0]+2*(y*z+x*w)*v[1]+(1-2*(x*x+y*y))*v[2])


def yaw(q):
    f = rotate(q, (1., 0., 0.))
    return math.atan2(f[1], f[0])


def tilt(q):
    return math.acos(clamp(rotate(q, (0., 0., 1.))[2], -1., 1.))


def qdistance(a, b):
    return 2*math.acos(clamp(abs(dot(unit(a), unit(b))), -1., 1.))


def slerp(a, b, t):
    a, b = unit(a), unit(b)
    d = dot(a, b)
    if d < 0:
        b, d = scale(b, -1.), -d
    if d > .9995:
        return unit(add(scale(a, 1-t), scale(b, t)))
    angle = math.acos(clamp(d, -1., 1.))
    return add(scale(a, math.sin((1-t)*angle)/math.sin(angle)),
               scale(b, math.sin(t*angle)/math.sin(angle)))


def normal(window_yaw): return (math.cos(window_yaw), math.sin(window_yaw), 0.)
def lateral(window_yaw): return (-math.sin(window_yaw), math.cos(window_yaw), 0.)


def intersect_ray(position, q, ray, cfg):
    origin = add(position, rotate(q, cfg['camera_translation_body_m']))
    direction = rotate(q, rotate(cfg['camera_rotation_body_from_optical_xyzw'], ray))
    n = normal(cfg['window_yaw_rad'])
    den = dot(n, unit(direction))
    if den < cfg['min_ray_plane_cos']:
        raise ValueError('ray_plane_angle')
    distance = dot(sub(cfg['window_center_local_m'], origin), n)/den
    if not 0.05 < distance < cfg['max_ray_range_m']:
        raise ValueError('ray_plane_range')
    return add(origin, scale(unit(direction), distance))


def envelope_half_extents(cfg):
    """Half extents of the swept box, centred on the envelope centre.

    The box is rotated by yaw_limit_rad about the vertical axis and by
    tilt_limit_rad about the lateral axis, so the horizontal footprint grows by
    the projection of the box, not by a corner-radius allowance:

        x: hx
        y: hy*cos(yaw) + hz*sin(tilt) composed through the actual rotation
        z: hz*cos(tilt) + ...

    The attitude allowance used to be ``2*r*sin((yaw+tilt)/2)`` with
    ``r = norm(size)/2 + 2*norm(centre)``. That inflated each axis by the
    radius of the worst-case corner and added the *norm* of the envelope-centre
    offset, so a purely vertical 9 cm offset widened the lateral extent by
    18 cm. For 0.62x0.62x0.35 at 5 deg this overstated the lateral half extent
    by 8.8 cm and the vertical by 6.2 cm.

    The centre offset is deliberately NOT added here: the controller aligns the
    envelope centre (``measurement()`` computes ``pose + rotate(q, centre)``)
    to the opening, so the offset is already handled by the alignment target.
    """
    half = [value / 2.0 for value in cfg['vehicle_size_m']]
    yaw, tilt = cfg['yaw_limit_rad'], cfg['tilt_limit_rad']
    cy, sy = math.cos(yaw), math.sin(yaw)
    ct, st = math.cos(tilt), math.sin(tilt)
    hx, hy, hz = half
    # Yaw mixes the two body horizontal axes in the ground plane.
    plan_x = hx * cy + hy * sy
    plan_y = hx * sy + hy * cy
    # Tilt then mixes the vertical axis into the lateral one.
    return (plan_x, plan_y * ct + hz * st, hz * ct + plan_y * st)


def opening_polygon(points, width, height):
    if len(points) != 4 or not all(vector(p, 2) for p in points):
        raise ValueError('four_pixel_corners_required')
    if any(not (0 <= x < width and 0 <= y < height) for x, y in points):
        raise ValueError('corners_outside_image')
    cross = []
    for i in range(4):
        a, b = sub(points[(i+1)%4], points[i]), sub(points[(i+2)%4], points[(i+1)%4])
        cross.append(a[0]*b[1]-a[1]*b[0])
        if norm(a) < 8:
            raise ValueError('opening_side_too_small')
    if not (all(c > 0 for c in cross) or all(c < 0 for c in cross)):
        raise ValueError('nonconvex_or_crossed_opening')
    area = abs(sum(points[i][0]*points[(i+1)%4][1]-points[(i+1)%4][0]*points[i][1] for i in range(4)))/2
    if area < 400:
        raise ValueError('opening_area_too_small')
    # Intersection of diagonals, not outer-frame centre or vertex average.
    a, b = points[0], points[1]
    u, v = sub(points[2], a), sub(points[3], b)
    den = u[0]*v[1]-u[1]*v[0]
    if abs(den) < 1e-8:
        raise ValueError('degenerate_diagonals')
    d = sub(b, a)
    t = (d[0]*v[1]-d[1]*v[0])/den
    return add(a, scale(u, t)), area
