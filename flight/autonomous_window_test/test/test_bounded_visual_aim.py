"""Camera dropout, false spots and bounded complete-course regressions."""
import math
import unittest
from dataclasses import replace
from hover_target import settings
from test_visual_target_hold import VisualReturnSim


class BoundedSim(VisualReturnSim):
    def __init__(self):
        super().__init__()
        self.m.hover_target.p.update(bounded_visual_aim=True, visual_aim_max_s=1.,
                                    timed_visual_aim=True, require_stable_target_before_light=False,
                                    hover_light_delay_s=1.)

    def inputs(self):
        i = super().inputs()
        if not hasattr(self, 'reference_error'):
            return i
        d = dict(i.hover_visual[1])
        center = [a-b for a,b in zip(d['reference_point_px'], d['reference_error_px'])]
        if d['observed_unique']:
            spot = [a+b for a,b in zip(center, self.spot_error)]
            d.update(observed_spot_px=spot, observed_candidates=[dict(centre_px=spot)])
        return replace(i, hover_visual=(i.wall,d))


class BoundedVisualAimTests(unittest.TestCase):
    def test_profile_rejects_unbounded_or_wrong_branch(self):
        p=BoundedSim().m.hover_target.p
        for patch in (dict(visual_aim_max_s=0),dict(visual_aim_max_s=3.1),
                      dict(visual_aim_max_s=float('nan')),dict(visual_aim_max_s=3.),dict(bounded_visual_aim=1),
                      dict(timed_visual_aim=False)):
            with self.assertRaises(ValueError):settings(dict(p,**patch))

    def test_first_light_starts_at_one_second(self):
        s=BoundedSim();s.expose();h=s.m.hover_target
        self.assertGreaterEqual(h.on_request_wall-h.started,1.)
        self.assertLess(h.on_request_wall-h.started,1.1)
        self.assertEqual(h.visual_start_basis,'bounded_timeout_visual_unconfirmed')
        self.assertTrue(h.aim_evidence.baseline_clear)

    def test_real_success_camera_failure_finishes_both_targets_and_lands(self):
        # The saved match had no valid target in 90/93 frames. Exercise the
        # same unavailable-camera condition through the entire supervisor.
        s=BoundedSim();s.draw_target=False;s.start();seen=[]
        for _ in range(7000):
            s.step();h=s.m.hover_target
            if h.phase=='ON_WAIT' and all(old is not h for old in seen):seen.append(h)
            if s.m.phase=='AUTO_LANDING':s.airborne=False;s.armed=False;s.mode='AUTO.LAND'
            if s.m.phase in ('DONE','FAULT','TAKEOVER','PILOT_CONTROL'):break
        self.assertEqual(s.m.phase,'DONE',s.m.reason)
        self.assertEqual(len(seen),2)
        for h in seen:
            self.assertGreaterEqual(h.on_request_wall-h.started,1.)
            self.assertLess(h.on_request_wall-h.started,1.1)
            self.assertEqual(h.correction_updates,0)
            self.assertEqual(h.visual_start_basis,'bounded_timeout_visual_unconfirmed')
            self.assertTrue(h.off_confirmed)
            self.assertGreaterEqual(h.off_request_wall-h.on_ack_wall,3.)
            self.assertLess(h.off_request_wall-h.on_ack_wall,3.1)
        self.assertTrue(s.m.return_journey.complete)

    def test_large_prediction_error_cannot_move_or_delay_first_light(self):
        s=BoundedSim();s.reference_error=[35.,25.];s.start();s.until('HOVER')
        while s.m.hover_target.started is None:s.step()
        h=s.m.hover_target;arrived=s.m.position
        while h.phase=='CENTER':
            self.assertEqual(s.m.position,arrived)
            self.assertEqual(h.correction_updates,0)
            s.step()
        self.assertEqual(h.phase,'ON_WAIT')
        self.assertTrue(h.request_on)
        self.assertLess(h.on_request_wall-h.started,1.1)
        self.assertEqual(h.status()['visual_aim_budget_s'],1.)

    def test_prelight_bright_reflection_does_not_veto_first_light(self):
        s=BoundedSim();s.preexisting_spot=True;s.expose();h=s.m.hover_target
        self.assertLess(h.on_request_wall-h.started,1.1)
        self.assertFalse(h.aim_evidence.baseline_clear)
        self.assertEqual(h.correction_updates,0)

    def test_real_spot_requires_three_consistent_post_ack_frames(self):
        s=BoundedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        # The ACK frame is not a post-ACK capture. Three later frames qualify.
        initial=h.correction_updates;s.step()
        self.assertEqual(h.correction_updates,initial)
        s.step();self.assertEqual(h.correction_updates,initial)
        before=s.m.position;s.step()
        self.assertGreater(h.correction_updates,initial)
        self.assertEqual(h.aim_source,'observed_spot_minus_target')
        self.assertLessEqual(math.dist(before,s.m.position),.04*.051)
        self.assertFalse(h.status()['physical_hit_verified'])

    def test_preexisting_bright_reflection_cannot_steer_after_timeout(self):
        s=BoundedSim();s.preexisting_spot=True;s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        for _ in range(20):s.step()
        self.assertFalse(h.aim_evidence.baseline_clear)
        self.assertEqual(h.correction_updates,0)
        self.assertEqual(h.aim_evidence.qualified_frames,0)

    def test_dropout_discards_spot_track_without_changing_exposure_deadline(self):
        s=BoundedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        for _ in range(5):s.step()
        self.assertGreater(h.correction_updates,0)
        s.draw_target=False;before=h.correction_updates
        for _ in range(100):
            s.step()
            if s.m.full_chain.index==1:break
        self.assertEqual(h.correction_updates,before)
        self.assertTrue(h.off_confirmed)
        self.assertLess(h.off_request_wall-h.on_ack_wall,3.1)

    def test_duplicate_frames_cannot_create_consensus_or_change_position(self):
        s=BoundedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        s.freeze_capture=s.f.ros;before=h.correction_updates
        for _ in range(12):s.step()
        self.assertEqual(h.correction_updates,before)

    def test_single_outlier_cannot_steer(self):
        s=BoundedSim();s.expose();h=s.m.hover_target
        s.spot_error=[30.,0.];before=h.correction_updates;s.step()
        s.spot_present=False;s.step()
        self.assertEqual(h.correction_updates,before)

    def test_inner_ring_margin_does_not_stop_two_pixel_servo_early(self):
        s=BoundedSim();s.spot_error=[4.,0.];s.expose();h=s.m.hover_target
        for _ in range(4):s.step()
        self.assertGreater(h.correction_updates,0)
        self.assertFalse(h.gates['observed_spot_centered'])
        s.spot_error=[1.,0.]
        for _ in range(4):s.step()
        before=h.correction_updates;s.step()
        self.assertEqual(h.correction_updates,before)
        self.assertTrue(h.gates['observed_spot_centered'])

    def test_white_core_feedback_is_not_enabled_for_unguarded_legacy_mode(self):
        s=BoundedSim();s.expose();h=s.m.hover_target;h.p['bounded_visual_aim']=False
        i=s.inputs();d=dict(i.hover_visual[1],observed_basis='white_core_local_contrast',capture_stamp=i.ros+.01)
        result,_=h.vision((i.wall,d),i.ros+.01,i.wall,h.hover_ros)
        self.assertFalse(result['observed_unique']);self.assertEqual(result['observed_candidates'],[])

    def test_one_missing_ring_frame_requires_reconfirmation_but_preserves_off_baseline(self):
        s=BoundedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        for _ in range(4):s.step()
        self.assertGreater(h.aim_evidence.qualified_frames,0)
        s.draw_target=False;s.step();before=h.correction_updates
        s.draw_target=True
        for _ in range(2):s.step()
        self.assertEqual(h.correction_updates,before)
        s.step();self.assertGreater(h.correction_updates,before)

    def test_long_visual_gap_discards_old_off_baseline(self):
        s=BoundedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        s.draw_target=False
        for _ in range(9):s.step()
        s.draw_target=True;before=h.correction_updates
        for _ in range(5):s.step()
        self.assertFalse(h.aim_evidence.baseline_clear)
        self.assertEqual(h.correction_updates,before)


if __name__=='__main__':unittest.main()
