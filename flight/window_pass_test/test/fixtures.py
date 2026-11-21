"""Synthetic data only: never import this module into a flight launch."""
import math
from window_pass.config import DEFAULTS, ATTESTATIONS
from window_pass.core import Controller, Pose, Observation
from window_pass.geometry import add, sub, scale, rotate, normal, lateral


def calibration(motion=False):
    c = dict(DEFAULTS)
    c.update(enable_motion=motion, enable_traverse=True, pass_stage_approved=True,
             evidence_id='SYNTHETIC_UNIT_TEST_NOT_FLIGHT', local_frame='map',body_frame='base_link',optical_frame='camera_optical',
             camera_info_sha256='a'*64,vision_profile_sha256='b'*64,algorithm_sha256='c'*64,
             envelope_center_body_m=[0.,0.,0.], camera_translation_body_m=[0.,0.,0.],
             camera_rotation_body_from_optical_xyzw=[-.5,.5,-.5,.5],
             window_center_local_m=[1.,0.,1.2], window_thickness_m=.10,
             geofence_min_m=[-3.,-3.,0.],geofence_max_m=[5.,3.,4.],
             position_uncertainty_m=.01,stopping_margin_m=.01,clearance_margin_m=.02,
             yaw_limit_rad=.035,tilt_limit_rad=.035,align_tolerance_m=.02,
             max_tracking_error_m=.04,max_setpoint_lead_m=.03,post_plane_clearance_m=.30)
    c.update({k:True for k in ATTESTATIONS})
    return c


def conjugate(q): return (-q[0],-q[1],-q[2],q[3])


class Feed:
    def __init__(self, motion=False, cfg=None):
        self.cfg = cfg or calibration(motion)
        self.ctrl = Controller(self.cfg)
        self.wall, self.ros = 10.,1010.
        self.p = (0.,0.,1.2)
        y = self.cfg['window_yaw_rad']
        self.q = (0.,0.,math.sin(y/2),math.cos(y/2))
        self.mode = 'OFFBOARD'
        self.observe = True

    def observation(self):
        c = self.cfg
        center = c['window_center_local_m']
        left = lateral(c['window_yaw_rad'])
        pts = [add(center,add(scale(left,x),(0.,0.,z))) for x,z in ((.5,.375),(-.5,.375),(-.5,-.375),(.5,-.375))]
        rays = []
        for point in [center]+pts:
            body = sub(rotate(conjugate(self.q),sub(point,self.p)),c['camera_translation_body_m'])
            cam = rotate(conjugate(c['camera_rotation_body_from_optical_xyzw']),body)
            if cam[2] <= .05:
                return Observation(self.ros,640,480,'camera_optical',False,0,(0.,0.),(),'', '')
            rays.append(tuple(v/cam[2] for v in cam))
        pixels = [(320.+400.*r[0],240.+400.*r[1]) for r in rays]
        return Observation(self.ros,640,480,'camera_optical',self.observe,1 if self.observe else 0,
                           pixels[0],tuple(pixels[1:]),'checker_texture','approximate_inner_opening',tuple(rays),'a'*64,'b'*64,'c'*64)

    def step(self, dt=.05, follow=False, image=True, observation=True, pose=True, graph=True):
        self.wall += dt
        self.ros += dt
        c = self.ctrl
        if follow and c.command is not None: self.p = c.command
        if pose: c.pose_update(Pose(self.ros,self.p,self.q,'map','base_link'),self.ros,self.wall)
        if image: c.image(self.ros,640,480,self.ros,self.wall)
        c.flight_state(self.ros,True,True,self.mode,self.ros,self.wall)
        c.extended_state(self.ros,True,self.ros,self.wall)
        if graph: c.graph(True,'synthetic_exclusive',self.wall)
        if observation: c.observe(self.observation(),self.ros,self.wall)
        data = c.tick(self.ros,self.wall)
        if c.output(self.ros,self.wall): c.output_sent(self.wall)
        return data

    def prime(self):
        for _ in range(25): self.step()

    def start(self):
        self.prime()
        if self.cfg['enable_motion']:
            ok,why = self.ctrl.request_hold(self.ros,self.wall)
            assert ok,why
            for _ in range(40): self.step()
        ok,why = self.ctrl.request_start(self.ros,self.wall)
        assert ok,why

    def ready(self):
        self.start()
        for _ in range(90): self.step(follow=True)
        assert self.ctrl.state == 'PREPASS', (self.ctrl.state,self.ctrl.reason,self.ctrl.metric)


def cleanup_tree(path):
    """Remove a temporary tree even when patched scripts left files read-only.

    ``TemporaryDirectory.cleanup()`` and even ``rm -rf`` fail on files a patch
    tool chmod'ed to 0o400, which is exactly what the guarded patcher does. The
    standard library only exposes ``ignore_cleanup_errors`` from 3.10, while the
    target machine is Python 3.8, so the retry has to be explicit.
    """
    import os
    import shutil
    import stat

    def force(func, target, _info):
        try:
            os.chmod(target, stat.S_IWRITE | stat.S_IREAD)
        except OSError:
            pass
        try:
            func(target)
        except OSError:
            # Last resort: chmod the parent too, then retry once.
            try:
                os.chmod(os.path.dirname(str(target)), stat.S_IRWXU)
                func(target)
            except OSError:
                pass

    if path is None:
        return
    target = str(path)
    if not os.path.exists(target):
        return
    shutil.rmtree(target, onerror=force)
