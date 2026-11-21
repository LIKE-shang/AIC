"""New scope and fresh-site binding checks, no real telemetry or control IO."""
import copy,importlib.util,json,unittest
from test_mission import ROOT
from hover_trial_policy import validate,DEFERRALS,SCOPE
from window_pass.core import Pose
from takeoff import TakeoffCore
spec=importlib.util.spec_from_file_location('bind_hover_target_trial',ROOT/'scripts/bind_hover_target_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class HoverScopeTests(unittest.TestCase):
    def setUp(self):
        self.base=json.loads((ROOT/'config/hover_target_from_traverse.json').read_text())
        self.snapshot=dict(identity={'boot_id':'current_boot','run_id':'current_run'},position_local_m=[.1,-.2,.26],orientation_xyzw=[0,0,0,1],
            pose_samples=120,xyz_span_m=[.01,.01,.006],local_frame='map',body_frame='base_link')
        self.snapshot['raw_lio']=dict(samples=120,position_local_m=[.1,-.2,0.],xyz_span_m=[.01,.01,.006])
        self.cfg=module.bind(self.base,self.snapshot)
    def test_bound_identity_and_fence_use_current_snapshot(self):
        self.assertEqual(self.cfg['site_identity'],self.snapshot['identity'])
        self.assertFalse(self.cfg['control']['enable_traverse'])
        self.assertTrue(self.cfg['control']['enable_motion'])
        self.assertTrue(self.cfg['control']['geofence_min_m'][0]<.1<self.cfg['control']['geofence_max_m'][0])
    def test_acceptance_deferrals_leave_evidence_false(self):
        self.assertEqual(validate(self.cfg,True),DEFERRALS)
        for k in DEFERRALS:self.assertFalse(self.cfg['acceptance'][k])
        self.assertEqual(validate(self.cfg,False),frozenset())
    def test_wrong_scope_or_extra_deferrals_refused(self):
        for key,val in [('scope','straight_pass'),('operator_requested_test',False),('current_target1_heading_and_area_confirmed',False),('deferred_checks',list(DEFERRALS)+['takeoff_column_verified'])]:
            c=copy.deepcopy(self.cfg);c['hover_trial_authorization'][key]=val
            with self.assertRaises(ValueError):validate(c,True)
    def test_passage_or_large_motion_refused(self):
        for field,value in [('max_correction_m',.3),('max_speed_mps',.2)]:
            c=copy.deepcopy(self.cfg);c['hover_target'][field]=value
            with self.assertRaises(ValueError):validate(c,True)
        c=copy.deepcopy(self.cfg);c['control']['enable_traverse']=True
        with self.assertRaises(ValueError):validate(c,True)
    def test_unstable_or_mismatched_samples_refused(self):
        for key,val in [('pose_samples',10),('xyz_span_m',[.05,.01,.01]),('local_frame','other'),('orientation_xyzw',[1,0,0,0])]:
            s=copy.deepcopy(self.snapshot);s[key]=val
            with self.assertRaises(ValueError):module.bind(self.base,s)
    def test_takeoff_ground_and_final_box_fit_without_fictitious_subfloor_sphere(self):
        core=TakeoffCore(self.cfg['control'])
        core.pose=Pose(1.,tuple(self.snapshot['position_local_m']),(0,0,0,1),'map','base_link')
        self.assertTrue(core.inside(core.pose.p))
        self.assertTrue(core.inside((.1,-.2,1.16)))
        self.assertFalse(core.inside((10,10,10)))
    def test_reported_diverged_origin_refused_despite_small_span(self):
        s=copy.deepcopy(self.snapshot)
        s['position_local_m']=[1306.937,-1083.729,-7661.233]
        with self.assertRaisesRegex(ValueError,'origin'):module.bind(self.base,s)

    def test_raw_lio_required_and_checked_independently(self):
        for raw in [None,dict(samples=120,position_local_m=[0,0,-7661],xyz_span_m=[.01]*3),
                    dict(samples=120,position_local_m=[0,0,0],xyz_span_m=[.1,0,0])]:
            s=copy.deepcopy(self.snapshot);s['raw_lio']=raw
            with self.assertRaises(ValueError):module.bind(self.base,s)

    def test_takeoff_deferral_option_cannot_authorize_hover(self):
        from test_auto_transport import app
        with self.assertRaises(ValueError):app.commissioning_deferrals(self.cfg,True)

if __name__=='__main__':unittest.main(verbosity=2)
