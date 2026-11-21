"""Bounded terminal capture after one fast pass; no flight IO."""
import math
from collections import deque
from window_pass.geometry import dot,sub,norm,add,scale,tilt
from smooth_pass import trajectory,sample

BRAKE_RESERVE=.375
END_OVERSHOOT=.50
TERMINAL_ALONG_ERROR=.65
CAPTURE_RADIUS=.30
FINISH_REFERENCE_SPEED=.15

class MotionWindow:
    def __init__(self):self.rows=deque();self.speed=None;self.span=None
    def update(self,stamp,position):
        if self.rows and stamp<=self.rows[-1][0]:return self.speed
        self.rows.append((stamp,tuple(position)))
        while len(self.rows)>2 and self.rows[1][0]<=stamp-.5:self.rows.popleft()
        elapsed=stamp-self.rows[0][0]
        self.speed=norm(sub(position,self.rows[0][1]))/elapsed if elapsed>=.45 else None
        self.span=max(norm(sub(p,position)) for t,p in self.rows)
        return self.speed
    def stable(self,limit=.04,span=.04):return self.speed is not None and self.speed<=limit and self.span<=span

class TerminalCapture:
    def __init__(self):
        self.motion=MotionWindow();self.stable_since=None;self.stop_position=None
        self.stage='FAST_PASS';self.clear=False;self.last_elapsed=0.
        self.capture_deadline=None;self.completion_basis=None
    def update(self,m,i,dt):
        p=m.direct_plan;c=m.core;elapsed=i.wall-m.entered
        self.last_elapsed=elapsed;self.motion.update(c.pose.stamp,c.pose.p)
        clear=m.full_chain.clear_of_window(m,c.pose.p)
        if self.clear and not clear:raise ValueError('terminal_reentered_window_clearance')
        self.clear=self.clear or clear
        if elapsed>=p['smooth_trajectory']['duration_s']:
            self.stage='TERMINAL_CAPTURE'
            if self.capture_deadline is None:self.capture_deadline=p['smooth_trajectory']['duration_s']+12.
            if elapsed>self.capture_deadline:
                if (clear and m.corridor(c.pose.p) and norm(sub(c.pose.p,p['endpoint']))<=CAPTURE_RADIUS and
                        m.recovery.take(m,i,'terminal_capture','capture_settling_timeout')):
                    self.capture_deadline=elapsed+12.
                else:raise ValueError('terminal_capture_timeout')
        if self.stage=='TERMINAL_CAPTURE':
            ref_done=norm(sub(m.position,p['endpoint']))<1e-6
            in_capture=norm(sub(c.pose.p,p['endpoint']))<=CAPTURE_RADIUS
            from mission_recovery import enabled
            actual_stop=enabled(m)
            settled=(ref_done and in_capture and self.clear and (self.motion.stable(.08,.05) if (m.hover_target.p.get('fixed_position_timed') or m.hover_target.p.get('visual_target_hold')) else self.motion.stable(c.c['stable_speed_mps'])) and
                     (actual_stop or norm(sub(c.pose.p,m.position))<=c.c['max_tracking_error_m']) and
                     m.corridor(c.pose.p) and tilt(c.pose.q)<=c.c['tilt_limit_rad'])
            if settled:
                if self.stable_since is None:self.stable_since=i.wall
                if i.wall-self.stable_since>=(.2 if getattr(m.full_chain,'scope',None)=='window_pass_two_targets' else 1. if (m.hover_target.p.get('fixed_position_timed') or m.hover_target.p.get('visual_target_hold')) else c.c['stop_stable_time_s']):
                    self.stop_position=tuple(c.pose.p);self.stage='STOP_CONFIRMED'
                    self.completion_basis='full_body_clear_actual_stop' if actual_stop else 'reference_tracking_stop'
                    return True
            else:self.stable_since=None
        return False
    def status(self):
        return dict(stage=self.stage,full_body_clear=self.clear,brake_reserve_m=BRAKE_RESERVE,
                    finish_reference_speed_mps=FINISH_REFERENCE_SPEED,overshoot_limit_m=END_OVERSHOOT,
                    net_speed_05s_mps=self.motion.speed,position_window_span_m=self.motion.span,
                    stable_since=self.stable_since,stop_position_local_m=self.stop_position,
                    capture_deadline_s=self.capture_deadline,completion_basis=self.completion_basis)
