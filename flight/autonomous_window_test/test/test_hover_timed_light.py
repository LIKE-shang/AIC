"""Competition: score/target detection cannot veto timed light output."""
import unittest
from dataclasses import replace
from test_hover_target import HoverSim

class CompetitionSim(HoverSim):
    def __init__(self,detected=True):
        self.detected=detected
        super().__init__()
        self.m.hover_target.p.update(trigger_policy='hover_timed',hover_light_delay_s=1.)
    def inputs(self):
        i=super().inputs()
        if not self.detected:
            d=i.hover_visual[1]
            d.update(valid=False,reason='await_unique_three_ring_target',reference_error_px=None,
                     reference_normalized_radius=None,observed_spot_px=None,
                     observed_inside_inner=None,inner_semiaxes_px=None)
        return i

class TimedLightTests(unittest.TestCase):
    def test_reported_missing_target_still_requests_light_then_off(self):
        s=CompetitionSim(False);s.start();s.until('PILOT_CONTROL')
        h=s.m.hover_target
        self.assertTrue(h.completed);self.assertTrue(h.exposure_completed)
        self.assertIsNotNone(h.on_request_wall);self.assertGreaterEqual(h.on_request_wall-h.started,1.)
        self.assertEqual(h.observed_hit_frames,0)
        self.assertEqual(h.completion_basis,'timed_exposure_acknowledged_hit_not_required')
        self.assertFalse(s.out['hover_target']['physical_hit_verified'])
        self.assertFalse(h.request_on)
    def test_off_centre_and_missing_spot_do_not_veto_exposure(self):
        s=CompetitionSim();s.force_error=[100.,100.];s.spot=False
        s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.m.hover_target.completed)
        self.assertEqual(s.m.hover_target.observed_hit_frames,0)
    def test_missing_image_still_prevents_on(self):
        s=CompetitionSim(False);s.visual_fresh=False;s.start();s.until('PILOT_CONTROL')
        self.assertIsNone(s.m.hover_target.on_request_wall)
        self.assertFalse(s.m.hover_target.completed)
    def test_camera_loss_while_on_requests_off(self):
        s=CompetitionSim(False);s.start();s.until('HOVER')
        for _ in range(200):
            s.step()
            if s.m.hover_target.phase=='EXPOSE':break
        self.assertEqual(s.m.hover_target.phase,'EXPOSE')
        s.visual_fresh=False;s.step()
        self.assertFalse(s.m.hover_target.request_on)
        s.until('PILOT_CONTROL');self.assertFalse(s.m.hover_target.completed)
    def test_on_and_off_ack_still_required(self):
        s=CompetitionSim(False);s.allow_on=False;s.start();s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
        s=CompetitionSim(False);s.start();s.until('HOVER')
        for _ in range(200):
            s.step()
            if s.m.hover_target.phase=='OFF_WAIT':break
        s.allow_off=False;s.until('PILOT_CONTROL')
        self.assertFalse(s.m.hover_target.completed)
    def test_mode_takeover_turns_light_off(self):
        s=CompetitionSim(False);s.start();s.until('HOVER')
        for _ in range(200):
            s.step()
            if s.m.hover_target.request_on:break
        self.assertTrue(s.m.hover_target.request_on)
        s.mode='POSCTL';s.step()
        self.assertEqual(s.m.phase,'TAKEOVER')
        self.assertFalse(s.out['hover_target']['request_on'])
    def test_valid_camera_but_bad_identity_never_requests_light(self):
        s=CompetitionSim(False);s.start();s.until('HOVER')
        for _ in range(340):
            s.update();i=s.inputs();i.hover_visual[1]['camera_frame_id']='wrong'
            s.out=s.m.tick(i)
            if s.out['position'] is not None:s.m.sent(s.f.wall)
            if s.out['action']=='POSCTL':s.mode='POSCTL'
            if s.m.phase=='PILOT_CONTROL':break
        self.assertIsNone(s.m.hover_target.on_request_wall)

if __name__=='__main__':unittest.main()
