"""Route-scoped continuity: anchor raw LIO once; never reset it in flight."""
from session_localization import Continuity
from window_pass.geometry import norm,sub,vector

class RouteContinuity(Continuity):
    def __init__(self,control):
        super().__init__(control)
        self.anchor=None
        self.expected_motion_limits={k:control[k] for k in ('max_pose_step_m','max_pose_speed_mps')} if control.get('max_pose_speed_mps')==2.2 else None
        # One 3m passage + 20cm alignment + <=1m station + aiming and reserve.
        # Not the single-hover trial's 5m absolute-origin assumption.
        self.maximum_displacement_m=control.get('max_traverse_distance_m',3.)+2.5
    def observe(self,pose,sequence,ros,wall,source,expected_source):
        if not vector(pose.p):return self.invalidate('route_raw_position_invalid')
        if self.anchor is None and norm(pose.p)>self.maximum_displacement_m:
            return self.invalidate('route_initial_raw_origin_not_plausible_for_fresh_lio')
        if self.anchor is not None and norm(sub(pose.p,self.anchor))>self.maximum_displacement_m:
            return self.invalidate('route_raw_displacement_exceeded')
        ok=super().observe(pose,sequence,ros,wall,source,expected_source)
        if ok and self.anchor is None:self.anchor=pose.p
        return ok
    def observe_bridge(self,status,wall,source,expected_source):
        if self.expected_motion_limits and status.get('raw_lio_motion_limits')!=self.expected_motion_limits:
            return self.invalidate('route_bridge_speed_limits_mismatch')
        return super().observe_bridge(status,wall,source,expected_source)
    def observe_fcu(self,pose):
        # FCU envelope and speed/jump checks are enforced in the existing core;
        # raw/FCU origins are not assumed equal or reset on phase changes.
        if not vector(pose.p) or norm(pose.p)>2*self.maximum_displacement_m:return self.invalidate('route_fcu_position_invalid')
        return not self.reason
