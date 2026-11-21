"""Add the hover raw-LIO guard before the unchanged vendor pose conversion."""
from hover_localization import HoverContinuity
from window_pass.core import Pose
import time

def guarded_bridge(legacy,control,guard_type=HoverContinuity,startup_mode=None):
    class GuardedPoseBridge(legacy.ContinuousPoseBridge):
        def __init__(self,*args,**kwargs):
            self.hover_guard=guard_type(control)
            super().__init__(*args,**kwargs)
        def _real_readiness(self,now,wall):
            reason=super()._real_readiness(now,wall)
            # The legacy policy can already accept MANUAL. An explicit ALTCTL
            # policy must therefore cover both that success and its mode refusal,
            # until the first real publish; later mode/arming changes must not
            # interrupt continuous localization. Other parent failures retain priority.
            if startup_mode is None or self.real_published!=0:return reason
            if reason not in ('','STARTUP_MODE_NOT_MANUAL'):return reason
            if startup_mode!='ALTCTL':return 'UNSUPPORTED_GROUND_STARTUP_MODE'
            state=self._fresh_state(now,wall)
            if state is None:return 'MAVROS_STATE_NOT_FRESH'
            if not state.connected:return 'MAVROS_NOT_CONNECTED'
            if state.armed:return 'STARTUP_ARMED_REFUSED'
            if state.mode!='ALTCTL':return 'STARTUP_MODE_NOT_ALTCTL'
            if self.gate.last_stamp_ns is None:return 'WAITING_INPUT'
            if not -self.gm.MAX_FUTURE_NS<=now-self.gate.last_stamp_ns<=self.gm.MAX_AGE_NS:
                return 'WAITING_FRESH_INPUT'
            return ''
        def on_odometry(self,msg):
            with self.lock:
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                valid=self.hover_guard.observe(Pose(msg.header.stamp.to_sec(),
                    (p.x,p.y,p.z),(q.x,q.y,q.z,q.w),msg.header.frame_id,msg.child_frame_id),
                    msg.header.seq,self.ros.Time.now().to_sec(),time.monotonic(),
                    (getattr(msg,'_connection_header',{}) or {}).get('callerid'),self.gm.SOURCE_NODE)
                if not valid:
                    self._disable_real('HOVER_RAW_LIO_'+self.hover_guard.reason)
                    return
                super().on_odometry(msg)
        def snapshot(self,now,wall):
            d=super().snapshot(now,wall)
            if startup_mode:d['ground_startup_mode']=startup_mode
            d['raw_lio_guard_scope']=guard_type.__name__
            d['raw_lio_motion_limits']={k:control[k] for k in ('max_pose_step_m','max_pose_speed_mps')}
            d['hover_raw_lio_guard']=dict(enabled=True,latched_reason=self.hover_guard.reason)
            if getattr(self.hover_guard,'evidence',None):d['hover_raw_lio_guard']['latch_evidence']=self.hover_guard.evidence
            return d
    return GuardedPoseBridge
