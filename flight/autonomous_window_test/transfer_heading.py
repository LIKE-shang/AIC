"""Finite heading recovery solely in the cleared inter-target corridor.
The original world position trajectory and yaw reference never change.
No ROS, actuator IO, yaw reset, or second takeoff.
"""
import math
from window_pass.geometry import yaw,wrap

class TransferHeading:
    hard_limit_rad=math.radians(10.)
    recovered_limit_rad=math.radians(3.)
    timeout_s=4.
    stable_s=.5
    def __init__(self):
        self.started=None;self.recovered_since=None;self.recovered_stamp=None
        self.last_stamp=None;self.recovered=False;self.maximum_error=0.;self.error=0.
    def scoped(self,m):
        chain=m.full_chain
        return bool(getattr(chain,'return_policy',None) and m.phase=='POSTPASS' and chain.index==1 and
                    chain.stage=='STATION_TRANSFER' and m.direct_pass_completed and
                    '_session_cylinder_radius_m' in m.core.c and m.direct_plan and
                    chain.allowed(m,m.core.pose.p) and chain.allowed(m,m.position))
    def check(self,m,i):
        error=abs(wrap(yaw(m.core.pose.q)-m.heading));self.error=error
        self.maximum_error=max(self.maximum_error,error)
        if error>self.hard_limit_rad:return 'inter_target_heading_over10deg'
        if error>m.core.c['yaw_limit_rad']:
            if self.started is None:self.started=i.wall
            self.recovered=False;self.recovered_since=None;self.recovered_stamp=None
        if self.started is None:return ''
        # Only distinct fresh poses spanning real wall time establish recovery.
        stamp=m.core.pose.stamp
        if error>self.recovered_limit_rad:
            self.recovered=False;self.recovered_since=None;self.recovered_stamp=None
        elif stamp!=self.last_stamp:
            if self.recovered_since is None:self.recovered_since=i.wall;self.recovered_stamp=stamp
            if i.wall-self.recovered_since>=self.stable_s and stamp-self.recovered_stamp>=self.stable_s:
                self.recovered=True
        self.last_stamp=stamp
        if not self.recovered and i.wall-self.started>=self.timeout_s:return 'inter_target_heading_recovery_timeout'
        return ''
    def ready(self,m):
        error=abs(wrap(yaw(m.core.pose.q)-m.heading))
        return error<=m.core.c['yaw_limit_rad'] and (self.started is None or self.recovered)
    def status(self):
        return dict(trigger_rad=math.radians(5.),hard_limit_rad=self.hard_limit_rad,
                    recovered_limit_rad=self.recovered_limit_rad,timeout_s=self.timeout_s,stable_s=self.stable_s,
                    started_wall=self.started,recovered=self.recovered,maximum_error_rad=self.maximum_error,
                    reference_yaw_unchanged=True,scoring_light_forbidden_until_recovered=True)
