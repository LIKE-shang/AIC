"""Lagged motion and fixed light deadlines for the paced observed-spot servo."""
import math
from types import SimpleNamespace as NS
import unittest
from hover_target import TargetHold, settings
from test_bounded_visual_aim import BoundedSim


class PacedSim(BoundedSim):
    def __init__(self):
        super().__init__()
        self.m.hover_target.p['paced_visual_servo']=True


class PacedServoTests(unittest.TestCase):
    def fixture(self):
        h=TargetHold(dict(enabled=True,bounded_visual_aim=True,timed_visual_aim=True,
            visual_target_hold=True,visual_servo_enabled=True,trigger_policy='hover_timed',
            paced_visual_servo=True))
        m=NS(core=NS(pose=NS(p=(0.,0.,.85),q=(0.,0.,0.,1.)),
                     c=dict(max_setpoint_lead_m=.04)),
             origin=(0.,0.,.85),aim_anchor=(0.,0.,.85),position=(0.,0.,.85),
             target_z=.85,corridor=lambda p:True,hover_target=h)
        return h,m

    def frame(self,h,m,t,error=(20.,0.),age=.04):
        d=dict(reference_error_px=list(error),servo_error_px=list(error),
               reference_normalized_radius=1.,inner_semiaxes_px=[20.,20.],
               inner_angle_deg=0.,capture_stamp=t,servo_capture_age_s=age)
        h.correct_reference(m,d,.05,t)

    def test_lagged_aircraft_cannot_receive_a_second_unanswered_pulse(self):
        h,m=self.fixture()
        for n in range(40):self.frame(h,m,n*.05)
        self.assertAlmostEqual(m.position[1],.01)
        self.assertAlmostEqual(h.servo_path_m,.01)
        self.assertEqual(h.servo_pacer.pulses,1)
        self.assertEqual(h.servo_pacer.reason,'await_motion_response_before_next_pulse')
        before=h.correction_updates
        m.core.pose.p=(0.,.01,.85)
        for n in range(40,48):self.frame(h,m,n*.05,error=(16.,0.))
        self.assertGreater(h.correction_updates,before)
        self.assertEqual(h.servo_pacer.pulses,2)
        self.assertLessEqual(m.position[1],.02+1e-9)

    def test_stale_images_and_moving_aircraft_wait_without_retargeting(self):
        for stale,moving in ((True,False),(False,True)):
            h,m=self.fixture()
            for n in range(15):
                if moving:m.core.pose.p=(0.,.008*(n%2),.85)
                self.frame(h,m,n*.05,error=(20.+8*(n%2) if moving else 20.,0.),age=.2 if stale else .04)
            self.assertEqual(h.correction_updates,0)

    def test_delayed_plant_reduces_error_without_exceeding_rate_or_pulse_budget(self):
        h,m=self.fixture();queue=[];initial=20.
        for n in range(60):
            t=n*.05
            queue.append(tuple(m.position))
            if len(queue)>6:m.core.pose.p=queue.pop(0)
            error=initial-m.core.pose.p[1]*400.
            before=m.position
            self.frame(h,m,t,error=(error,0.))
            self.assertLessEqual(math.dist(before,m.position),.04*.05+1e-9)
            self.assertLessEqual(h.servo_pacer.path,.01+1e-9)
            self.assertLessEqual(math.dist(m.position,m.aim_anchor),.15)
        self.assertLess(abs(error),initial*.6)
        self.assertGreater(h.servo_pacer.pulses,1)

    def test_lost_target_after_partial_pulse_does_not_forget_pending_motion(self):
        h,m=self.fixture()
        for n in range(5):self.frame(h,m,n*.05)
        before=m.position;self.assertGreater(h.correction_updates,0)
        h.servo_pacer.hold('spot_missing')
        for n in range(5,25):self.frame(h,m,n*.05)
        self.assertEqual(m.position,before)
        self.assertTrue(h.servo_pacer.pending)

    def test_reflection_does_not_steer_and_cannot_delay_first_light(self):
        s=PacedSim();s.preexisting_spot=True;s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
        for _ in range(15):s.step()
        self.assertEqual(h.correction_updates,0)
        self.assertLess(h.on_request_wall-h.started,1.1)

    def test_all_precision_waits_end_at_fixed_exposure_deadline(self):
        for scenario in ('no_target','unanswered_motion','oscillating_error'):
            s=PacedSim();s.spot_error=[20.,0.];s.expose();h=s.m.hover_target
            if scenario=='no_target':s.draw_target=False
            for n in range(100):
                if scenario=='oscillating_error':s.spot_error=[20.+8*(n%2),0.]
                s.step(follow=scenario!='unanswered_motion')
                if s.m.full_chain.index==1:break
            self.assertTrue(h.off_confirmed,scenario)
            self.assertGreaterEqual(h.off_request_wall-h.on_ack_wall,3.)
            self.assertLess(h.off_request_wall-h.on_ack_wall,3.1)
            self.assertFalse(h.status()['physical_hit_verified'])

    def test_profile_requires_guarded_observed_mode(self):
        p=PacedSim().m.hover_target.p
        for patch in (dict(paced_visual_servo=1),dict(bounded_visual_aim=False),
                      dict(visual_servo_enabled=False)):
            with self.assertRaises(ValueError):settings(dict(p,**patch))

    def test_worsening_guard_remains_active_while_waiting_for_tracking(self):
        h,m=self.fixture();h.servo_path_m=.02
        self.frame(h,m,0.,error=(20.,0.))
        for t in (.1,.3,.6):self.frame(h,m,t,error=(40.,0.))
        self.assertTrue(h.servo_paused)
        count=h.correction_updates
        self.frame(h,m,.7,error=(40.,0.))
        self.assertEqual(h.correction_updates,count)

    def test_eight_cm_existing_lead_can_be_reduced_without_unanswered_second_pulse(self):
        h,m=self.fixture();m.position=(0.,.08,.85)
        for n in range(30):self.frame(h,m,n*.05,error=(20.,0.))
        self.assertGreater(h.correction_updates,0)
        self.assertAlmostEqual(m.position[1],.07)
        self.assertAlmostEqual(h.servo_path_m,.01)
        self.assertEqual(h.servo_pacer.pulses,1)
        self.assertEqual(h.servo_pacer.reason,'await_motion_response_before_next_pulse')

    def test_excessive_lead_cannot_grow_and_rejected_proposal_is_not_a_pulse(self):
        h,m=self.fixture();m.position=(0.,.08,.85)
        for n in range(20):self.frame(h,m,n*.05,error=(60.,0.))
        self.assertEqual(h.correction_updates,0)
        self.assertEqual(h.servo_pacer.pulses,0)
        self.assertEqual(m.position,(0.,.08,.85))

    def test_smooth_observed_drift_can_start_while_large_oscillations_cannot(self):
        for coherent in (True,False):
            h,m=self.fixture()
            for n in range(10):
                error=20.+2.*n if coherent else 20.+8.*(n%2)
                self.frame(h,m,n*.05,error=(error,0.))
            if coherent:
                self.assertGreater(h.correction_updates,0)
                self.assertLess(h.servo_pacer.diagnostics['trend_residual_px'],1e-9)
            else:
                self.assertEqual(h.correction_updates,0)

    def test_twenty_mm_position_span_is_bounded(self):
        h,m=self.fixture()
        for n in range(12):
            m.core.pose.p=(.01*n,0.,.85)
            self.frame(h,m,n*.05)
        self.assertEqual(h.correction_updates,0)
        self.assertEqual(h.servo_pacer.reason,'await_slow_position_motion')

    def test_trend_filter_estimates_newest_capture_without_future_extrapolation(self):
        h,m=self.fixture()
        for n in range(4):self.frame(h,m,n*.05,error=(20.+2*n,10.-n))
        self.assertAlmostEqual(h.servo_pacer.filtered_error_px[0],26.)
        self.assertAlmostEqual(h.servo_pacer.filtered_error_px[1],7.)

    def test_repeated_or_future_frame_cannot_accumulate_motion(self):
        h,m=self.fixture()
        for _ in range(10):self.frame(h,m,0.,error=(20.,0.))
        self.assertEqual(h.correction_updates,0)
        for n in range(10):self.frame(h,m,n*.05,age=-.01)
        self.assertEqual(h.correction_updates,0)

    def test_followed_large_lead_allows_only_further_lead_reduction(self):
        h,m=self.fixture();m.position=(0.,.08,.85)
        for n in range(8):self.frame(h,m,n*.05,error=(20.,0.))
        self.assertEqual(h.servo_pacer.pulses,1)
        m.core.pose.p=(0.,-.004,.85)
        before=m.position
        for n in range(8,20):self.frame(h,m,n*.05,error=(20.,0.))
        self.assertGreater(h.servo_pacer.pulses,1)
        self.assertLess(m.position[1],before[1])
        self.assertLessEqual(h.servo_path_m,.02+1e-9)

    def test_precommand_drift_does_not_satisfy_response_gate(self):
        h,m=self.fixture();m.position=(0.,.08,.85)
        for n in range(8):self.frame(h,m,n*.05,error=(60.,0.))
        self.assertEqual(h.correction_updates,0)
        m.core.pose.p=(0.,-.004,.85)
        for n in range(8,35):self.frame(h,m,n*.05,error=(20.,0.))
        self.assertEqual(h.servo_pacer.pulses,1)
        self.assertAlmostEqual(h.servo_path_m,.01)


if __name__=='__main__':unittest.main()
