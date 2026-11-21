import copy,importlib.util,sys,unittest
from test_mission import ROOT,DEFAULTS
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('reboot_setup_tested',ROOT/'scripts/reboot_setup.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class RebootPreparation(unittest.TestCase):
    def config(self):
        return {'experiment':'takeoff_land','mission':dict(DEFAULTS,ground_fcu_height_m=.245),
                'control':{'enable_motion':True,'enable_traverse':False,'pass_stage_approved':False,
                           'geofence_min_m':[-1,-1,-1],'geofence_max_m':[2,2,2]},
                'site_identity':{'boot_id':'old'},'site_evidence':{'old':True},
                'acceptance':{'body_origin_verified':False,'simulation_verified':False,'fcu_failsafe_verified':False,'ground_height_verified':True},
                'commissioning_authorization':{'scope':'supervised_takeoff_hover_land_only','operator_accepts_unverified_tests':True,
                    'deferred_checks':['body_origin_verified','simulation_verified','fcu_failsafe_verified']}}
    def test_rebind_clears_coordinates_and_keeps_unverified_status(self):
        cfg=self.config();before=copy.deepcopy(cfg);bound=module.template_for_binding(cfg)
        self.assertEqual(cfg,before)
        self.assertFalse(bound['control']['enable_motion'])
        self.assertEqual(bound['control']['geofence_min_m'],[])
        self.assertEqual(bound['control']['geofence_max_m'],[])
        self.assertIsNone(bound['site_identity']);self.assertNotIn('site_evidence',bound)
        self.assertEqual(bound['acceptance'],before['acceptance'])
    def test_rebind_cannot_invent_authorization(self):
        cfg=self.config();cfg.pop('commissioning_authorization')
        with self.assertRaises(ValueError):module.template_for_binding(cfg)
    def test_rebind_refuses_traverse_enabled(self):
        cfg=self.config();cfg['control']['enable_traverse']=True
        with self.assertRaises(ValueError):module.template_for_binding(cfg)

if __name__=='__main__':unittest.main(verbosity=2)
