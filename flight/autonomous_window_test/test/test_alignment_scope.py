"""Explicit limited alignment acceptance never modifies verification flags."""
import unittest,copy,math
from test_alignment_only import AlignOnlySim
from mission import Mission
from alignment_policy import validate,SCOPE,ACCEPTANCE_DEFERRALS,CONTROL_DEFERRALS
from test_auto_transport import app

class AlignmentScope(unittest.TestCase):
    def config(self):
        s=AlignOnlySim();c=copy.deepcopy(s.m.core.c)
        c.update(max_align_path_m=.30,max_align_time_s=20.,max_total_time_s=60.,camera_translation_body_m=[.08,0.,-.06],camera_rotation_body_from_optical_xyzw=[-.5,.5,-.5,.5])
        for k in CONTROL_DEFERRALS:c[k]=False
        o=dict(s.options,max_mission_time_s=75.,takeoff_attitude_checks_enabled=True)
        return {'experiment':'align_only','acquisition':'airborne_rgbd','control':c,'mission':o,
         'expected_fcu_parameters':{'MPC_Z_VEL_MAX_UP':[0,.4],'MPC_ACC_UP_MAX':[0,.4],'COM_OBL_RC_ACT':[0,0.]},
         'installation_confirmation':{'operator_confirmed':True,'camera_translation_body_m':[.08,0.,-.06],'camera_rotation_body_from_optical_xyzw':[-.5,.5,-.5,.5]},
         'acceptance':{k:False for k in ACCEPTANCE_DEFERRALS},
         'alignment_commissioning_authorization':{'scope':SCOPE,'operator_accepts_unverified_alignment_tests':True,'deferred_acceptance_checks':sorted(ACCEPTANCE_DEFERRALS),'deferred_control_checks':sorted(CONTROL_DEFERRALS)}}
    def test_preserves_all_false_flags(self):
        c=self.config();before=copy.deepcopy(c)
        self.assertEqual(validate(c,True),ACCEPTANCE_DEFERRALS);self.assertEqual(c,before)
    def test_no_implicit_acceptance(self):
        c=self.config();self.assertFalse(validate(c,False))
        c['alignment_commissioning_authorization']['operator_accepts_unverified_alignment_tests']=False
        with self.assertRaises(ValueError):validate(c,True)
    def test_cannot_allow_pass_or_other_experiment(self):
        for mutation in ('pass','traverse','experiment','acquisition'):
            c=self.config()
            if mutation=='pass':c['control']['pass_stage_approved']=True
            elif mutation=='traverse':c['control']['enable_traverse']=True
            elif mutation=='experiment':c['experiment']='traverse'
            else:c['acquisition']='none'
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):validate(c,True)
    def test_cannot_skip_live_checks(self):
        for key in ('rc_ok','estimator_ok','single_controller_verified_extra','ground_height_verified'):
            c=self.config();c['alignment_commissioning_authorization']['deferred_acceptance_checks'].append(key)
            with self.subTest(key=key),self.assertRaises(ValueError):validate(c,True)
    def test_caps_enforced(self):
        for key,value in [('align_speed_mps',.081),('max_align_distance_m',.201),('max_align_path_m',.301),('max_tracking_error_m',.121),('pose_timeout_s',.151),('image_timeout_s',.251),('tilt_limit_rad',.1),('approach_min_m',.79)]:
            c=self.config();c['control'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):validate(c,True)
    def test_attitude_and_manual_landing_required(self):
        for key in ('pilot_landing_enabled','takeoff_attitude_checks_enabled','takeoff_position_goal_enabled'):
            c=self.config();c['mission'][key]=False
            with self.subTest(key=key),self.assertRaises(ValueError):validate(c,True)
    def test_camera_declaration_must_match(self):
        c=self.config();c['installation_confirmation']['camera_translation_body_m']=[0.,0.,0.]
        with self.assertRaises(ValueError):validate(c,True)
    def test_scoped_core_defers_only_explicit_flags_and_cannot_pass(self):
        c=self.config();validate(c,True)
        m=Mission(c['control'],c['mission'],airborne_geometry=True,experiment='align_only',alignment_commissioning=True)
        self.assertEqual(m.core.request_pass(),(False,'alignment_commissioning_cannot_commit_pass'))
        self.assertTrue(all(m.core.c[k] is False for k in CONTROL_DEFERRALS))
        bad=dict(c['control'],position_uncertainty_m=0.)
        with self.assertRaises(ValueError):Mission(bad,c['mission'],airborne_geometry=True,experiment='align_only',alignment_commissioning=True)
    def test_complete_scoped_sim_hands_over_without_pass(self):
        c=self.config();s=AlignOnlySim();s.options=c['mission'];s.m=Mission(c['control'],s.options,airborne_geometry=True,experiment='align_only',alignment_commissioning=True);s.f.ctrl=s.m.core
        s.f.cfg.update(camera_translation_body_m=c['control']['camera_translation_body_m'],camera_rotation_body_from_optical_xyzw=c['control']['camera_rotation_body_from_optical_xyzw'])
        for _ in range(30):s.update()
        s.start();s.until('PILOT_CONTROL');self.assertTrue(s.out['alignment_completed'])
        self.assertNotIn('PASS',[e['new'] for e in s.m.events]);self.assertEqual(s.actions,['OFFBOARD','ARM','POSCTL'])
