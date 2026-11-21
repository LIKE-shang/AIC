"""Local-origin guard for the single-target, fresh-LIO hover trial only."""
from session_localization import Continuity
from window_pass.geometry import norm,vector

# This trial starts over target1 in a newly initialized local LIO frame.
# It does not support map-relocalized or arbitrary-offset world coordinates.
LOCAL_ORIGIN_RADIUS_M=5.

def require_local_origin(position):
    if not vector(position) or norm(position)>LOCAL_ORIGIN_RADIUS_M:
        raise ValueError('hover_local_origin_not_reset_or_diverged')

class HoverContinuity(Continuity):
    def observe(self,pose,sequence,ros,wall,source,expected_source):
        try:require_local_origin(pose.p)
        except ValueError:return self.invalidate('hover_lio_origin_not_reset_or_diverged')
        return super().observe(pose,sequence,ros,wall,source,expected_source)

    def observe_fcu(self,pose):
        try:require_local_origin(pose.p)
        except ValueError:return self.invalidate('hover_fcu_origin_not_reset_or_diverged')
        return not self.reason
