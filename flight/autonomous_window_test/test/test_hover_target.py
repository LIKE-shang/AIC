"""Single-target real-policy tests with synthetic plant, no ROS or PWM IO."""
import copy
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from test_position_goal_takeoff import GoalSim
from mission import Mission
from hover_target import DEFAULTS,settings,TargetHold


class HoverSim(GoalSim):
    def __init__(self,offset=(0.,0.)):
        super().__init__()
        self.options.update(pilot_landing_enabled=True)
        p=dict(DEFAULTS,enabled=True,settle_time_s=.2,exposure_s=.4,max_light_s=2.)
        self.m=Mission(self.m.core.c,self.options,experiment='takeoff_land',hover_target_policy=p)
        self.f.ctrl=self.m.core;self.out=None;self.offset=offset;self.allow_on=True;self.allow_off=True
        self.spot=True;self.force_error=None;self.visual_fresh=True;self.corrected=False
        for _ in range(30):self.update()
    def update(self,follow=True):
        if follow and getattr(self,'out',None) and self.out.get('position') and hasattr(self,'offset'):
            target=self.out['position'];p=self.f.p
            self.f.p=(p[0]+max(-.002,min(.002,target[0]-p[0])),p[1]+max(-.002,min(.002,target[1]-p[1])),p[2])
        super().update(follow)
    def inputs(self):
        i=super().inputs();flow=self.m.hover_target
        error=[self.offset[0]-(self.f.p[1]-self.m.origin[1])*400 if self.m.origin else 0,
               self.offset[1]-(self.f.p[0]-self.m.origin[0])*400 if self.m.origin else 0]
        if self.force_error is not None:error=list(self.force_error)
        stamp=self.f.ros if self.visual_fresh else self.f.ros-2
        p=flow.p
        d=dict(valid=True,capture_stamp=stamp,camera_frame_id=p['camera_frame_id'],
               source_image_size_px=[640,360],reference_point_px=[372.75,194.984375],
               reference_error_px=error,inner_semiaxes_px=[20.8,20.8],inner_angle_deg=0.,
               reference_normalized_radius=math.hypot(*error)/20.8,
               observed_spot_px=[372.75,194.984375] if self.spot else None,
               observed_minus_reference_px=[0.,0.] if self.spot else None,
               observed_inside_inner=math.hypot(*error)<20.8 if self.spot else None)
        on=flow.request_on and self.allow_on
        if not flow.request_on and not self.allow_off:on=True
        bridge=dict(scope='single_hover_target',fault_latched=False,
                    state='ON_ACKNOWLEDGED' if on else 'OFF_ACKNOWLEDGED',
                    request_id=flow.request_id)
        return replace(i,hover_visual=(i.wall,d),hover_laser=(i.wall,bridge))
    def step(self,follow=True,respond=True):
        out=super().step(follow,respond)
        if respond and out.get('action')=='POSCTL':self.mode='POSCTL'
        return out


class HoverTargetTests(unittest.TestCase):
    def trial(self):
        s=HoverSim()
        s.m.o['vendor_takeoff_arrival_enabled']=True
        s.m.core.c['yaw_limit_rad']=math.radians(5)
        return s

    def yaw(self,s,degrees):
        angle=math.radians(degrees)
        s.f.q=(0.,0.,math.sin(angle/2),math.cos(angle/2))

    def test_reported_ground_yaw_transient_does_not_abort_climb(self):
        s=self.trial();s.start();s.until('TAKEOFF')
        s.freeze=True
        self.yaw(s,5.069)
        for _ in range(28):s.step(follow=False)
        self.assertEqual(s.m.phase,'TAKEOFF',s.m.reason)
        self.assertAlmostEqual(s.f.p[2],.245)
        self.assertTrue(s.out['hover_takeoff_heading_tolerance_active'])
        self.assertAlmostEqual(s.out['heading_limit_rad'],math.radians(30))
        self.assertFalse(s.m.hover_target.request_on)
        self.assertFalse(s.m.takeoff_arrival(s.inputs()))
        s.freeze=False;self.yaw(s,0)
        s.until('PILOT_CONTROL')
        self.assertTrue(s.m.hover_target.completed)

    def test_heading_must_recover_before_hover_and_laser(self):
        s=self.trial();s.start();s.until('TAKEOFF');self.yaw(s,5.069)
        for _ in range(240):s.step()
        self.assertEqual(s.m.phase,'TAKEOFF',s.m.reason)
        self.assertIn('hover_heading_recovered',s.m.takeoff_progress['blocking_criteria'])
        self.assertFalse(s.m.hover_target.request_on)
        self.yaw(s,0);s.until('HOVER');s.step()
        self.assertFalse(s.out['hover_takeoff_heading_tolerance_active'])
        self.assertAlmostEqual(s.out['heading_limit_rad'],math.radians(5))

    def test_hover_retains_five_degree_heading_check(self):
        s=self.trial();s.start();s.until('HOVER');self.yaw(s,5.069);s.step()
        self.assertEqual(s.m.phase,'FAULT')
        self.assertIn('heading_changed',s.m.reason)
        self.assertFalse(s.out['hover_target']['request_on'])

    def test_takeoff_yaw_tolerance_is_bounded_and_requires_hover_policy(self):
        s=self.trial();s.start();s.until('TAKEOFF');self.yaw(s,30.1);s.step()
        self.assertEqual(s.m.phase,'FAULT')
        s=GoalSim();s.m.core.c['yaw_limit_rad']=math.radians(5)
        s.start();s.until('TAKEOFF');self.yaw(s,5.069);s.step()
        self.assertEqual(s.m.phase,'FAULT')

    def test_takeoff_transient_keeps_tracking_and_emergency_checks(self):
        s=self.trial();s.start();s.until('TAKEOFF');self.yaw(s,5.069)
        s.f.p=(.15,0.,s.f.p[2]);s.step(follow=False)
        self.assertEqual(s.m.phase,'FAULT')
        s=self.trial();s.start();s.until('TAKEOFF');self.yaw(s,5.069)
        s.healthy['emergency']=True;s.step(follow=False)
        self.assertEqual(s.m.phase,'TAKEOVER')
        self.assertFalse(s.out['hover_target']['request_on'])

    def center_replay(self,offset=0.,raw_speed=0.,drift=0.):
        p=dict(DEFAULTS,enabled=True)
        flow=TargetHold(p)
        core=SimpleNamespace(pose=SimpleNamespace(p=(0.,0.,1.),q=(0.,0.,0.,1.)),
            pose_speed=raw_speed,c=dict(stable_speed_mps=.05,align_tolerance_m=.12,
                                       max_setpoint_lead_m=.04))
        m=SimpleNamespace(core=core,origin=(0.,0.,1.),position=(0.,0.,1.),target_z=1.,
                          corridor=lambda pos:True,output=flow.status)
        for n in range(61):
            wall=n*.02;stamp=100.+math.floor((wall+1e-8)*20)/20.
            core.pose.p=(m.position if offset else (drift*wall,0.,1.))
            d=dict(valid=True,capture_stamp=stamp,camera_frame_id=p['camera_frame_id'],
                   source_image_size_px=[640,360],reference_point_px=[372.75,194.984375],
                   reference_error_px=[offset,0.],inner_semiaxes_px=[20.8,20.8],
                   inner_angle_deg=0.,reference_normalized_radius=abs(offset)/20.8)
            bridge=dict(scope='single_hover_target',fault_latched=False,state='OFF_ACKNOWLEDGED',request_id=0)
            i=SimpleNamespace(wall=wall,ros=100.+wall,hover_visual=(wall,d),hover_laser=(wall,bridge))
            flow.tick(m,i,.02)
            if flow.request_on:break
        return flow,m

    def test_twenty_hz_camera_uses_elapsed_frames_at_fifty_hz_control(self):
        flow,m=self.center_replay(offset=100.)
        # 1.14s between accepted frames plus one 0.02s initial tick.
        self.assertAlmostEqual(m.position[1],.04*1.16,places=6)
        self.assertFalse(flow.request_on)
        self.assertAlmostEqual(flow.correction_dt,.04)

    def test_hover_jitter_speed_uses_position_window_and_records_gates(self):
        flow,m=self.center_replay(raw_speed=.12)
        self.assertTrue(flow.request_on)
        self.assertTrue(flow.status()['laser_on_ever_requested'])
        self.assertTrue(all(flow.gates.values()))
        self.assertEqual(flow.window_speed,0.)
        self.assertEqual(flow.metrics['pose_speed_raw_mps'],.12)

    def test_real_drift_still_blocks_laser_even_when_camera_says_centred(self):
        flow,m=self.center_replay(drift=.08)
        self.assertFalse(flow.request_on)
        self.assertTrue(flow.gates['position_tracking'])
        self.assertFalse(flow.gates['speed_stable'])
        self.assertAlmostEqual(flow.window_speed,.08)

    def test_full_single_target_lifecycle(self):
        s=HoverSim();s.start();s.until('PILOT_CONTROL',limit=2000)
        self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
        self.assertTrue(s.m.hover_target.completed)
        self.assertFalse(s.out['hover_target']['request_on'])
        self.assertEqual(s.m.hover_target.phase,'DONE')
        self.assertAlmostEqual(s.m.target_z-s.m.origin[2],.9)
        self.assertTrue(s.airborne)
    def test_right_down_error_moves_left_forward(self):
        s=HoverSim((12.,12.));s.start();s.until('HOVER')
        origin=s.m.origin
        for _ in range(50):
            out=s.step()
            if s.m.hover_target.request_on:break
        self.assertGreater(s.m.position[0],origin[0])
        self.assertGreater(s.m.position[1],origin[1])
        self.assertAlmostEqual(s.m.position[2],s.m.target_z)
    def test_fence_and_displacement_bound_are_checked(self):
        s=HoverSim((200.,200.));s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
        self.assertFalse(s.m.hover_target.request_on)
        self.assertLessEqual(math.hypot(s.m.position[0]-s.m.origin[0],s.m.position[1]-s.m.origin[1]),.15)
    def test_no_full_target_on_ground_needed(self):
        s=HoverSim();s.visual_fresh=False;s.start()
        self.assertEqual(s.m.phase,'PRESTREAM')
    def test_no_target_after_takeoff_hands_back_without_on(self):
        s=HoverSim();s.start();s.until('HOVER');s.visual_fresh=False
        s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
        self.assertEqual(s.m.hover_target.on_request_wall,None)
    def test_camera_loss_during_exposure_requests_off_then_handover(self):
        s=HoverSim();s.start();s.until('HOVER')
        for _ in range(200):
            s.step()
            if s.m.hover_target.phase=='EXPOSE':break
        self.assertEqual(s.m.hover_target.phase,'EXPOSE')
        s.visual_fresh=False;s.step()
        self.assertFalse(s.out['hover_target']['request_on'])
        s.until('PILOT_CONTROL');self.assertFalse(s.m.hover_target.completed)
    def test_pilot_mode_exit_immediately_suppresses_on(self):
        s=HoverSim();s.start();s.until('HOVER')
        for _ in range(200):
            s.step()
            if s.m.hover_target.request_on:break
        self.assertTrue(s.m.hover_target.request_on)
        s.mode='ALTCTL';s.step()
        self.assertEqual(s.m.phase,'TAKEOVER')
        self.assertFalse(s.out['hover_target']['request_on'])
    def test_spot_missing_cannot_complete(self):
        s=HoverSim();s.spot=False;s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
    def test_offset_mismatch_rejected(self):
        s=HoverSim();s.start();s.until('HOVER');s.step();s.update();i=s.inputs()
        i.hover_visual[1]['reference_point_px']=[1,2]
        s.m.tick(i)
        self.assertEqual(s.m.hover_target.reason,'camera_reference_offset_mismatch')
    def test_on_ack_required(self):
        s=HoverSim();s.allow_on=False;s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
    def test_off_ack_timeout_is_not_success(self):
        s=HoverSim();s.start();s.until('HOVER')
        for _ in range(300):
            s.step()
            if s.m.hover_target.phase=='OFF_WAIT':break
        s.allow_off=False;s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
        self.assertIn('off_ack_timeout',s.m.handover_snapshot['reason'])
    def test_geometry_includes_current_ellipse_scale(self):
        s=HoverSim();s.start();s.until('HOVER');d,new=s.m.hover_target.vision(s.inputs().hover_visual,s.f.ros,s.f.wall,s.f.ros-.1)
        self.assertTrue(new);self.assertEqual(d['inner_semiaxes_px'],[20.8,20.8])
    def test_invalid_config(self):
        for patch in [dict(max_correction_m=.5),dict(max_speed_mps=.3),dict(max_light_s=9),dict(offset_x_px=float('nan'))]:
            with self.assertRaises(ValueError):settings(dict(enabled=True,**patch))


if __name__=='__main__':unittest.main(verbosity=2)
