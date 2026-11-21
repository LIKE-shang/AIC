"""Independent live localization continuity; no visual or flight-control IO."""
import math
from window_pass.core import Controller


class Continuity:
    def __init__(self,control):
        # Reuse the existing pose validator for FAST-LIO source coordinates.
        self.raw=Controller(dict(control,enable_motion=False,enable_traverse=False,
            pass_stage_approved=False,local_frame='camera_init',body_frame='body',pose_timeout_s=.2))
        self.epoch=0;self.reason='';self.last_seq=None;self.bridge=None;self.evidence=None
        self.duplicate_messages=0

    def _evidence(self,pose,sequence,ros):
        # Read-only measurement of the frame about to be judged; never changes the decision.
        def finite(v):return v if isinstance(v,(int,float)) and math.isfinite(v) else None
        try:
            e={'sequence':sequence,'stamp':finite(pose.stamp),'lag_s':finite(ros-pose.stamp)}
            previous=self.raw.pose
            if previous is not None:
                dt=pose.stamp-previous.stamp
                step=math.dist(pose.p,previous.p)
                dot=min(1.,abs(sum(a*b for a,b in zip(pose.q,previous.q))))
                e.update(dt_s=finite(dt),step_m=finite(step),speed_mps=(finite(step/dt) if dt>0 else None),attitude_step_rad=finite(2*math.acos(dot)))
            return e
        except Exception as exc:
            return {'sequence':sequence,'error':'evidence_unavailable:'+type(exc).__name__}

    def invalidate(self,reason):
        if not self.reason:self.epoch+=1;self.reason=reason
        return False

    def observe(self,pose,sequence,ros,wall,source,expected_source):
        if self.reason:return False
        if not expected_source or source!=expected_source:return self.invalidate('lio_source_changed')
        if type(sequence) is not int or sequence<0:return self.invalidate('lio_sequence_invalid')
        if sequence==self.last_seq and self.raw.pose==pose:
            # Identical retransmission is not new motion or new freshness.
            self.duplicate_messages+=1
            return True
        if self.last_seq is not None and sequence<=self.last_seq and not(self.last_seq>0xffff0000 and sequence<0x10000):
            return self.invalidate('lio_sequence_reset_or_duplicate')
        previous=self.raw.pose
        if previous is not None and pose.stamp-previous.stamp>.5:return self.invalidate('lio_continuity_gap')
        evidence=self._evidence(pose,sequence,ros)
        if not self.raw.pose_update(pose,ros,wall):
            if not self.reason:self.evidence=evidence
            return self.invalidate('lio_pose_discontinuity')
        self.last_seq=sequence
        return True

    def observe_bridge(self,status,wall,source,expected_source):
        if not expected_source or source!=expected_source:return self.invalidate('pose_bridge_source_changed')
        count=status.get('real_published')
        if type(count) is not int or count<=0:return self.invalidate('pose_bridge_count_invalid')
        if self.bridge and count<self.bridge[1]['real_published']:return self.invalidate('pose_bridge_restarted')
        if status.get('real_fault') or status.get('real_output_state')!='FORWARDING_CONTINUOUS_POSE':
            return self.invalidate('pose_bridge_not_forwarding')
        if status.get('horizontal_flags_valid') is not True:return self.invalidate('pose_bridge_horizontal_invalid')
        self.bridge=(wall,dict(status));return not self.reason

    def health(self,ros,wall):
        if self.reason:return False,self.reason
        if not self.raw.fresh('pose',ros,wall,.2):return False,'lio_pose_unavailable'
        if self.bridge is None or not 0<=wall-self.bridge[0]<=1.5:return False,'pose_bridge_status_stale'
        return True,'ok'
