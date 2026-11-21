"""Rest-to-rest analytic jerk limited passage; no IO or flight authorization."""
import math

SPEED=1.5
ACCELERATION=1.0
JERK=4.0
ALONG_ERROR=.35
END_OVERSHOOT=.15

def trajectory(length,max_speed=SPEED,acceleration=ACCELERATION,jerk=JERK):
    if not math.isfinite(length) or length<=0:raise ValueError('invalid_smooth_length')
    if any(type(x) not in (int,float) or not math.isfinite(x) or x<=0 for x in (max_speed,acceleration,jerk)):raise ValueError('invalid_trajectory_dynamics')
    # Distance of both symmetric acceleration lobes is v*(v/a+a/j).
    v=max_speed
    def ramps(v):
        tj=min(acceleration/jerk,math.sqrt(v/jerk))
        return tj,max(0.,v/(jerk*tj)-tj)
    tj,ta=ramps(v)
    if v*(2*tj+ta)>length:
        lo,hi=0.,v
        for _ in range(60):
            mid=(lo+hi)/2;aj,at=ramps(mid)
            if mid*(2*aj+at)>length:hi=mid
            else:lo=mid
        v=lo;tj,ta=ramps(v)
    tc=max(0.,length/v-(2*tj+ta))
    phases=[(tj,jerk),(ta,0.),(tj,-jerk),(tc,0.),(tj,-jerk),(ta,0.),(tj,jerk)]
    return dict(length_m=length,peak_speed_mps=v,acceleration_limit_mps2=acceleration,
                jerk_limit_mps3=jerk,phases=phases,duration_s=sum(t for t,j in phases))

def sample(plan,elapsed):
    if not math.isfinite(elapsed) or elapsed<0:raise ValueError('invalid_smooth_time')
    if elapsed>=plan['duration_s']:return plan['length_m'],0.,0.
    x=v=a=0.;remaining=elapsed
    for duration,j in plan['phases']:
        t=min(duration,remaining)
        x+=v*t+a*t*t/2+j*t*t*t/6
        v+=a*t+j*t*t/2;a+=j*t;remaining-=t
        if remaining<=1e-12:break
    return x,max(0.,v),a
