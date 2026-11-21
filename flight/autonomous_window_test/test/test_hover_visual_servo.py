"""Synthetic plant: actual spot feedback corrects a biased predicted offset."""
import math,unittest
from test_hover_timed_light import CompetitionSim

class ServoSim(CompetitionSim):
    def __init__(self,detected=True,bias=(12.,8.)):
        self.bias=bias
        super().__init__(detected)
        self.m.hover_target.p.update(visual_servo_enabled=True,exposure_s=3.,max_light_s=5.,
                                    servo_deadband_px=2.,servo_gain=.6,hover_light_delay_s=2.)
    def inputs(self):
        i=super().inputs();d=i.hover_visual[1]
        if self.detected and self.m.hover_target.request_on:
            err=[d['reference_error_px'][k]+self.bias[k] for k in range(2)]
            d.update(observed_error_px=err,observed_spot_px=[372.75+self.bias[0],194.984375+self.bias[1]],
                     observed_minus_reference_px=list(self.bias),observed_inside_inner=math.hypot(*err)<20.8)
        return i
    def remaining_error(self):
        return (self.bias[0]-(self.f.p[1]-self.m.origin[1])*400,
                self.bias[1]-(self.f.p[0]-self.m.origin[0])*400)

class ServoTests(unittest.TestCase):
    def test_actual_spot_bias_converges_during_light(self):
        s=ServoSim();s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.m.hover_target.completed)
        self.assertGreater(s.m.hover_target.correction_updates,0)
        self.assertEqual(s.m.hover_target.aim_source,'observed_spot_minus_target')
        self.assertLess(math.hypot(*s.remaining_error()),2.5)
        self.assertGreater(s.m.hover_target.observed_hit_frames,0)
    def test_correction_continues_after_on_ack(self):
        s=ServoSim();s.start();s.until('HOVER')
        for _ in range(250):
            s.step()
            if s.m.hover_target.phase=='EXPOSE':break
        before=s.m.position;updates=s.m.hover_target.correction_updates
        for _ in range(15):s.step()
        self.assertTrue(s.m.hover_target.request_on)
        self.assertGreater(s.m.hover_target.correction_updates,updates)
        self.assertNotEqual(before,s.m.position)
    def test_no_target_preserves_timed_light_and_never_guesses_correction(self):
        s=ServoSim(False);s.start();s.until('PILOT_CONTROL')
        self.assertTrue(s.m.hover_target.completed)
        self.assertEqual(s.m.hover_target.correction_updates,0)
        self.assertEqual(s.m.hover_target.last_correction_reason,'target_unavailable_hold_position')
    def test_scope_and_rate_bound_remain(self):
        s=ServoSim(bias=(4000.,4000.));s.start();last=None
        for _ in range(1800):
            s.step()
            if s.m.phase=='HOVER':
                p=s.m.position
                self.assertLessEqual(math.hypot(p[0]-s.m.origin[0],p[1]-s.m.origin[1]),.15+1e-9)
                if last:self.assertLessEqual(math.hypot(p[0]-last[0],p[1]-last[1]),.04*.05+1e-9)
                last=p
            if s.m.phase=='PILOT_CONTROL':break
        self.assertEqual(s.m.phase,'PILOT_CONTROL')
        self.assertTrue(s.m.hover_target.completed)
    def make_replay(self):
        from types import SimpleNamespace as NS
        from hover_target import TargetHold,DEFAULTS
        h=TargetHold(dict(DEFAULTS,enabled=True,visual_servo_enabled=True))
        c=NS(pose=NS(p=(.0444,-.0033,.7678),q=(0.,0.,0.,1.)),
             c=dict(max_setpoint_lead_m=.04))
        m=NS(core=c,origin=(.0138,.0128,-.05),position=(.0138,.0128,.8499),
             target_z=.8499,corridor=lambda pos:True)
        d=dict(reference_error_px=[20.375,15.279],reference_normalized_radius=1.4,
               inner_semiaxes_px=[18.72,18.60],inner_angle_deg=0.)
        return h,m,d

    def test_reported_vertical_gap_no_longer_blocks_planar_correction(self):
        h,m,d=self.make_replay();before=m.position
        h.correct_reference(m,d,.05,0.)
        self.assertEqual(h.correction_updates,1)
        self.assertNotEqual(m.position[:2],before[:2])
        self.assertEqual(m.position[2],m.target_z)
        self.assertGreater(abs(m.target_z-m.core.pose.p[2]),.04)
        self.assertLess(h.servo_diagnostics['horizontal_lead_m'],.04)

    def test_existing_horizontal_lag_can_recover_but_not_increase(self):
        h,m,d=self.make_replay()
        m.position=(.10,0.,m.target_z);m.core.pose.p=(0.,0.,m.target_z)
        d['reference_error_px']=[0.,1.] # above deadband below gets replaced
        d['reference_error_px']=[0.,3.]
        h.correct_reference(m,d,.05,0.)
        self.assertEqual(h.correction_updates,1)
        self.assertLess(m.position[0],.1)
        h,m,d=self.make_replay()
        m.position=(.04,0.,m.target_z);m.core.pose.p=(0.,0.,m.target_z)
        d['reference_error_px']=[0.,100.]
        h.correct_reference(m,d,.05,0.)
        self.assertEqual(h.correction_updates,0)
        self.assertEqual(h.last_correction_reason,'waiting_for_horizontal_tracking')

    def test_sustained_worsening_pauses_additional_corrections(self):
        h,m,d=self.make_replay();h.servo_path_m=.02
        h.correct_reference(m,d,.05,0.)
        d['reference_error_px']=[20.,50.]
        h.correct_reference(m,d,.05,.1)
        h.correct_reference(m,d,.05,.6)
        self.assertTrue(h.servo_paused)
        count=h.correction_updates;before=m.position
        d['reference_error_px']=[30.,70.]
        h.correct_reference(m,d,.05,.7)
        self.assertEqual(count,h.correction_updates)
        self.assertEqual(before,m.position)

    def test_servo_deadband_and_gain_validation(self):
        from hover_target import settings
        for patch in [dict(servo_gain=2.),dict(servo_deadband_px=0),dict(visual_servo_enabled='true')]:
            with self.assertRaises(ValueError):settings(dict(enabled=True,**patch))

if __name__=='__main__':unittest.main()
