"""New live-vision policy and complete same-owner return lifecycle. No IO."""
import copy,json,math,unittest
from pathlib import Path
from dataclasses import replace
from test_second_window_return import ReturnSim
from hover_target import settings
from visual_target_hold import tick,reset_exposure,recover_visual_servo,acquisition_failure

ROOT=Path(__file__).resolve().parents[1]

class ServoRecoveryTests(unittest.TestCase):
    def fixture(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        h=SimpleNamespace(servo_paused=True,servo_best_error=3.,servo_worse_since=1.,
            stable_motion=SimpleNamespace(stable=Mock(return_value=True)),metrics={},
            servo_path_m=.0194,request_on=False,last_visual_stamp=100.,last_correction_reason='error_increasing_servo_paused')
        m=SimpleNamespace(position=(3.26,.04,.85),core=SimpleNamespace(pose=SimpleNamespace(p=(3.24,.08,.85))))
        d=dict(reference_error_px=[46.,0.],inner_semiaxes_px=[17.,16.],capture_stamp=100.)
        return h,m,d
    def recover(self,h,m,d):
        for dt in (0.,.05,.10,.20):
            recover_visual_servo(h,m,dict(d,capture_stamp=d['capture_stamp']+dt),100.+dt)
    def test_recovers_after_fresh_improved_target_with_hover_wobble(self):
        h,m,d=self.fixture();self.recover(h,m,d)
        self.assertFalse(h.servo_paused);self.assertEqual(h.visual_recoveries,1)
        self.assertEqual(h.servo_path_m,.0194);self.assertFalse(h.request_on)
        h.stable_motion.stable.assert_called_with(.20,.12)
    def test_persistent_wrong_direction_stays_paused(self):
        h,m,d=self.fixture();d['reference_error_px']=[82.,0.];self.recover(h,m,d)
        self.assertTrue(h.servo_paused);self.assertFalse(h.request_on)
    def test_single_or_repeated_frame_cannot_resume(self):
        h,m,d=self.fixture()
        for _ in range(20):recover_visual_servo(h,m,d,100.)
        self.assertTrue(h.servo_paused)
    def test_moving_or_far_from_reference_cannot_resume(self):
        h,m,d=self.fixture();h.stable_motion.stable.return_value=False;self.recover(h,m,d)
        self.assertTrue(h.servo_paused)
        h,m,d=self.fixture();m.position=(3.5,.04,.85);self.recover(h,m,d)
        self.assertTrue(h.servo_paused)
    def test_recoveries_bounded_and_diagnostic_reason_precise(self):
        h,m,d=self.fixture();h.visual_recoveries=3;self.recover(h,m,d)
        self.assertTrue(h.servo_paused);self.assertEqual(acquisition_failure(h),'visual_servo_recovery_exhausted')
        h.visual_recoveries=0;self.assertEqual(acquisition_failure(h),'visual_target_not_aligned')
        h.last_visual_stamp=None;self.assertEqual(acquisition_failure(h),'visual_target_not_acquired')

class VisualReturnSim(ReturnSim):
    def __init__(self):
        super().__init__()
        self.m.hover_target.p.update(visual_target_hold=True,fixed_position_timed=False,visual_servo_enabled=True,require_stable_target_before_light=True,exposure_s=3.,max_light_s=5.,settle_time_s=.2,hover_light_delay_s=.5)
        self.reference_error=[0.,0.];self.spot_error=[0.,0.];self.spot_present=True;self.spot_unique=True
        self.freeze_capture=None;self.draw_target=True;self.preexisting_spot=False
    def inputs(self):
        i=super().inputs()
        if not hasattr(self,'reference_error'):return i
        h=self.m.hover_target;d=dict(i.hover_visual[1]);on=h.phase=='EXPOSE'
        present=(self.spot_present and on) or (self.preexisting_spot and not on)
        d.update(valid=self.draw_target,reference_error_px=list(self.reference_error),reference_normalized_radius=math.hypot(*self.reference_error)/20.8,
                 observed_candidates=[{'centre_px':[373.,195.]}] if present else [],observed_unique=self.spot_unique and present,
                 observed_error_px=list(self.spot_error) if present else None,observed_normalized_radius=math.hypot(*self.spot_error)/20.8 if present else None,
                 observed_inside_inner=present and math.hypot(*self.spot_error)<20.8,observed_spot_px=[373.,195.] if present else None)
        if self.freeze_capture is not None:d['capture_stamp']=self.freeze_capture
        return replace(i,hover_visual=(i.wall,d))
    def expose(self):
        self.start();self.until('HOVER')
        for _ in range(400):
            self.step()
            if self.m.hover_target.phase=='EXPOSE':return
        raise AssertionError((self.m.phase,self.m.reason,self.m.hover_target.status()))

class VisualHoldTests(unittest.TestCase):
    def test_timed_visual_aim_starts_after_one_second_despite_wobble_or_lost_target(self):
        for scenario in ('servo_paused','no_target'):
            s=VisualReturnSim();s.start();s.until('HOVER');h=s.m.hover_target
            while h.started is None:s.step()
            h.p.update(timed_visual_aim=True,require_stable_target_before_light=False,
                       hover_light_delay_s=1.,exposure_s=3.)
            if scenario=='no_target':s.draw_target=False
            else:
                h.servo_paused=True;h.visual_recoveries=3;s.reference_error=[75.,0.]
            started=h.started
            for _ in range(80):
                if h.phase=='ON_WAIT':break
                s.step()
            self.assertEqual(h.phase,'ON_WAIT',scenario)
            self.assertGreaterEqual(h.on_request_wall-started,1.)
            self.assertLess(h.on_request_wall-started,1.1)
            self.assertTrue(h.status()['visual_aim_budget_elapsed'],scenario)
            for _ in range(3):s.step()
            self.assertEqual(h.phase,'EXPOSE',scenario)
            self.assertTrue(h.status()['visual_aim_budget_elapsed'],scenario)
            self.assertEqual(h.status()['exposure_s'],3.)
            self.assertFalse(h.p['require_stable_target_before_light'])
            self.assertEqual(h.p['exposure_s'],3.)
    def test_timed_profile_rejects_changed_light_duration_or_visual_budget(self):
        p=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())['hover_target']
        self.assertTrue(settings(p)['timed_visual_aim'])
        for bad in (dict(hover_light_delay_s=.9),dict(exposure_s=2.)):
            with self.assertRaises(ValueError):settings(dict(p,**bad))
    def test_official_profile_has_explicit_visual_policy_and_preserves_return(self):
        cfg=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())
        p=settings(cfg['hover_target']);self.assertTrue(p['visual_target_hold']);self.assertTrue(p['timed_visual_aim']);self.assertFalse(p['fixed_position_timed'])
        self.assertFalse(p['require_stable_target_before_light']);self.assertEqual(p['hover_light_delay_s'],1.)
        self.assertEqual(p['exposure_s'],3.);self.assertEqual(p['max_light_s'],5.)
        self.assertEqual(cfg['control']['future_tolerance_s'],.02)
        self.assertEqual(cfg['full_chain']['second_window_return']['return_speed_mps'],1.5)
        self.assertEqual(cfg['hover_target']['exposure_s'],3.)
    def test_incompatible_visual_profiles_rejected(self):
        p=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())['hover_target']
        for bad in [dict(fixed_position_timed=True),dict(visual_servo_enabled=False),dict(visual_calibration_mode='learn'),dict(visual_target_hold=1)]:
            with self.assertRaises(ValueError):settings(dict(p,**bad))
    def test_official_prepare_binding_validates_without_running_prepare(self):
        from test_full_chain_binding import FullChainBinding
        from test_two_targets import binder
        from full_chain_policy import validate
        _,snap,bounds=FullChainBinding().data()
        cfg=json.loads((ROOT/'config/full_chain_second_window_return.json').read_text())
        bound=binder.bind(cfg,snap,bounds,[1,3])
        self.assertTrue(validate(bound,True));self.assertTrue(bound['hover_target']['visual_target_hold'])
        self.assertTrue(bound['mission']['second_window_return_enabled'])
        self.assertEqual(bound['mission']['max_mission_time_s'],240.)
    def test_targets_one_three_second_window_return_complete_off_before_moves(self):
        s=VisualReturnSim();s.start();core=s.m.core;origin=None;ons=[];prior=False;last=-1;stamps={};exposure=[]
        for _ in range(7000):
            s.step();h=s.m.hover_target
            if origin is None and s.m.origin is not None:origin=s.m.origin
            if h.request_on and not prior:ons.append(s.m.full_chain.selected[s.m.full_chain.index])
            prior=h.request_on
            self.assertGreaterEqual(h.request_id,last);last=h.request_id
            self.assertIs(s.m.core,core)
            if origin is not None:self.assertEqual(s.m.origin,origin)
            if s.m.phase in ('POSTPASS','RETURN_PREP','YAW_TURN','SEARCH','PASS','RETURN_HOME'):self.assertFalse(h.request_on)
            if h.phase=='EXPOSE':stamps.setdefault(s.m.full_chain.index,[]).append(h.stable_exposure_s)
            if s.m.phase=='AUTO_LANDING':
                s.airborne=False;s.armed=False;s.mode='AUTO.LAND'
            if s.m.phase in ('DONE','PILOT_CONTROL','FAULT','TAKEOVER'):break
        self.assertEqual(s.m.phase,'DONE',s.m.reason)
        self.assertEqual(ons,[1,3]);self.assertEqual(s.out['full_chain']['completed_target_ids'],[1,3])
        self.assertTrue(s.m.return_journey.complete)
        for result in s.m.full_chain.results:
            self.assertTrue(result['laser_off_confirmed']);self.assertEqual(result['completion_basis'],'timed_3s_after_on_ack_hit_not_verified')
        self.assertEqual(sum(e['new']=='TAKEOFF' for e in s.m.events),1)
    def test_15cm_nominal_error_with_valid_aim_does_not_abort(self):
        s=VisualReturnSim();s.expose();anchor=tuple(s.m.aim_anchor)
        for n in range(57):
            s.f.p=(anchor[0]+.16*min(n/55.,1),anchor[1],s.m.target_z);s.step(False)
            self.assertNotIn(s.m.phase,('FAULT','TAKEOVER','PILOT_CONTROL'))
        self.assertGreater(s.m.hover_target.metrics['nominal_horizontal_error_m'],.15)
        self.assertTrue(s.m.hover_target.request_on)
    def test_missing_or_ambiguous_actual_spot_still_finishes_timed_exposure(self):
        for name in ('spot_present','spot_unique'):
            s=VisualReturnSim();setattr(s,name,False);s.expose();s.until('RETURN_PREP',limit=1800)
            self.assertEqual(s.out['full_chain']['completed_target_ids'],[1,3])
            self.assertTrue(s.m.hover_target.off_confirmed)
            self.assertFalse(s.out['full_chain']['physical_hit_verified'])
    def test_spot_outside_corrects_but_does_not_delay_target_transition(self):
        s=VisualReturnSim();s.spot_error=[20.,0.];s.expose();s.until('RETURN_PREP',limit=1800)
        self.assertEqual(s.out['full_chain']['completed_target_ids'],[1,3]);self.assertGreater(s.m.hover_target.correction_updates,0)
    def test_target_lost_after_on_still_closes_after_three_seconds_and_advances(self):
        s=VisualReturnSim();s.expose();old=s.m.hover_target;ack=old.on_ack_wall;s.draw_target=False
        for _ in range(100):
            s.step()
            if s.m.full_chain.index==1:break
        self.assertEqual(s.m.full_chain.index,1);self.assertTrue(old.off_confirmed)
        self.assertGreaterEqual(old.off_request_wall-ack,3.);self.assertLess(old.off_request_wall-ack,3.1)
    def test_replayed_images_do_not_move_but_cannot_extend_fixed_exposure(self):
        s=VisualReturnSim();s.expose();s.step();s.freeze_capture=s.f.ros;old=s.m.hover_target;count=old.correction_updates
        for _ in range(100):
            s.step()
            if s.m.full_chain.index==1:break
        self.assertEqual(s.m.full_chain.index,1);self.assertEqual(old.correction_updates,count);self.assertTrue(old.completed)
    def test_off_ack_missing_does_not_start_target_three(self):
        s=VisualReturnSim();s.expose();s.allow_off=False;s.until('PILOT_CONTROL',limit=900)
        self.assertEqual(s.m.full_chain.index,0);self.assertEqual(s.m.full_chain.results,[])
        self.assertFalse(s.m.hover_target.off_confirmed)
    def test_no_target_before_on_does_not_light(self):
        s=VisualReturnSim();s.draw_target=False;s.start();s.until('HOVER');s.until('PILOT_CONTROL',limit=1100)
        self.assertIsNone(s.m.hover_target.on_request_wall);self.assertEqual(s.m.full_chain.results,[])
    def test_last_moment_spot_leaves_centre_does_not_restart_three_seconds(self):
        s=VisualReturnSim();s.expose();old=s.m.hover_target
        while s.f.wall-old.on_ack_wall<2.9:s.step()
        s.spot_error=[20.,0.]
        while s.m.full_chain.index==0:s.step()
        self.assertTrue(old.completed);self.assertGreaterEqual(old.off_request_wall-old.on_ack_wall,3.)
        self.assertLess(old.off_request_wall-old.on_ack_wall,3.1)
    def test_missing_image_in_on_wait_does_not_hide_on_ack_or_extend_exposure(self):
        s=VisualReturnSim();s.start();s.until('HOVER')
        while s.m.hover_target.phase!='ON_WAIT':s.step()
        old=s.m.hover_target;s.draw_target=False
        for _ in range(120):
            s.step()
            if s.m.full_chain.index==1:break
        self.assertEqual(s.m.full_chain.index,1);self.assertTrue(old.completed)
        self.assertLess(old.off_request_wall-old.on_ack_wall,3.1)
    def test_light_ack_loss_still_aborts_even_with_three_second_policy(self):
        s=VisualReturnSim();s.expose();s.allow_on=False;s.until('PILOT_CONTROL',limit=900)
        self.assertEqual(s.m.full_chain.results,[]);self.assertIn('laser_ack_lost',s.m.hover_target.reason)
    def test_preexisting_bright_spot_blocks_probe(self):
        s=VisualReturnSim();s.preexisting_spot=True;s.start();s.until('HOVER');s.until('PILOT_CONTROL',limit=1100)
        self.assertIsNone(s.m.hover_target.on_request_wall)
    def test_good_alignment_uses_existing_half_second_not_extra_fixed_wait(self):
        s=VisualReturnSim();s.expose();h=s.m.hover_target
        self.assertLessEqual(h.on_request_wall-h.started,.8)
    def test_offcentre_frame_resets_continuous_exposure(self):
        s=VisualReturnSim();s.expose()
        for _ in range(25):s.step()
        self.assertGreater(s.m.hover_target.stable_exposure_s,1.)
        s.spot_error=[20.,0.];s.step();self.assertEqual(s.m.hover_target.stable_exposure_s,0.)
        s.spot_error=[0.,0.]
        for _ in range(25):s.step()
        self.assertFalse(s.m.hover_target.completed)
    def test_correction_is_rate_limited_and_bounded_without_moving_origin(self):
        s=VisualReturnSim();s.expose();s.spot_error=[15.,0.];origin=s.m.origin;before=s.m.position
        s.step();self.assertLessEqual(math.dist(s.m.position,before),.04*.05)
        self.assertLessEqual(math.dist(s.m.position[:2],s.m.aim_anchor[:2]),.15)
        self.assertEqual(s.m.origin,origin)

if __name__=='__main__':unittest.main()
