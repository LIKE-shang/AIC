"""Ground binding cannot reuse old identity or claim flight acceptance."""
import copy,importlib.util,json,unittest
from pathlib import Path
from test_mission import ROOT
from test_session_window import IDENTITY
spec=importlib.util.spec_from_file_location('full_chain_bind',ROOT/'scripts/bind_full_chain_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class FullChainBinding(unittest.TestCase):
    def data(self):
        c=json.loads((ROOT/'config/full_chain_single_target.json').read_text())
        identity=copy.deepcopy(IDENTITY);identity['/window_live/status']=['/window_live_vision','synthetic_uri']
        snapshot=dict(identity=identity,position_local_m=[.01,.02,-.05],orientation_xyzw=[0.,0.,0.,1.],pose_samples=160,xyz_span_m=[.001]*3,
                      raw_lio=dict(samples=80,position_local_m=[0.,0.,0.],xyz_span_m=[.001]*3),local_frame='map',body_frame='base_link')
        bounds=json.loads((ROOT/'FIELD_BOUNDARY_CORRECTED_20261018.json').read_text())
        return c,snapshot,bounds
    def test_new_binding_preserves_false_flight_flags_and_inputs(self):
        c,s,b=self.data();before=copy.deepcopy(c)
        result=module.bind(c,s,b,[.2,-.1],'SYNTHETIC_ONLY')
        self.assertEqual(c,before);self.assertEqual(result['site_identity'],s['identity'])
        for key in ('simulation_verified','body_origin_verified','fcu_failsafe_verified','rgbd_geometry_verified'):
            self.assertFalse(result['acceptance'][key])
        self.assertFalse(result['site_evidence']['flight_acceptance']);self.assertFalse(result['site_evidence']['precision_acceptance'])
        self.assertNotEqual(result['control']['geofence_min_m'],c['control']['geofence_min_m'])
    def test_historical_boot_missing_rgbd_and_moving_raw_stream_rejected(self):
        c,s,b=self.data();s['identity']['boot_id']=module.HISTORICAL_BOOT
        with self.assertRaises(ValueError):module.bind(c,s,b,[0.,0.],'SYNTHETIC_ONLY')
        c,s,b=self.data();s['identity'].pop('/window_live/status')
        with self.assertRaises(ValueError):module.bind(c,s,b,[0.,0.],'SYNTHETIC_ONLY')
        c,s,b=self.data();s['raw_lio']['xyz_span_m']=[.041,0.,0.]
        with self.assertRaises(ValueError):module.bind(c,s,b,[0.,0.],'SYNTHETIC_ONLY')
    def test_already_enabled_template_rejected(self):
        c,s,b=self.data();c['control']['enable_motion']=True
        with self.assertRaises(ValueError):module.bind(c,s,b,[0.,0.],'SYNTHETIC_ONLY')

if __name__=='__main__':unittest.main()
