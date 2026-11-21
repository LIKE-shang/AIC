"""Check reuse of existing passage settings without copying a historical site."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
from test_mission import ROOT
spec=importlib.util.spec_from_file_location('prepare_hover_profile',ROOT/'scripts/prepare_hover_target.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class HoverProfileReuse(unittest.TestCase):
    def setUp(self):
        self.raw=(ROOT/'config/session_window.json').read_bytes()
        self.source=json.loads(self.raw.decode('utf-8-sig'))
    def convert(self):
        return module.from_traverse_template(self.source,hashlib.sha256(self.raw).hexdigest())
    def test_reuses_position_takeoff_height_and_handover(self):
        c=self.convert()
        self.assertEqual(c['mission']['target_bottom_height_m'],.9)
        for key in ('bottom_to_fcu_m','ground_fcu_height_m','takeoff_position_goal_enabled',
                    'vendor_takeoff_arrival_enabled','pilot_landing_enabled'):
            self.assertEqual(c['mission'][key],self.source['mission'][key])
        self.assertEqual(c['expected_fcu_parameters'],self.source['expected_fcu_parameters'])
    def test_never_reuses_old_site_or_passage_permissions(self):
        self.source['site_identity']={'boot_id':'old'}
        self.source['control'].update(enable_motion=True,enable_traverse=True,
                                      pass_stage_approved=True,geofence_min_m=[-9]*3,geofence_max_m=[9]*3)
        c=self.convert()
        self.assertIsNone(c['site_identity'])
        self.assertEqual(c['control']['geofence_min_m'],[])
        self.assertFalse(c['control']['enable_motion'])
        self.assertFalse(c['control']['enable_traverse'])
        self.assertFalse(c['control']['pass_stage_approved'])
        self.assertFalse(c['acceptance']['takeoff_column_verified'])
        self.assertNotIn('straight_pass_authorization',c)
        self.assertNotIn('session_window',c)
    def test_window_specific_exception_not_silently_reused(self):
        c=self.convert()
        self.assertFalse(c['mission']['simplified_takeoff_enabled'])
        self.assertEqual(c['mission']['minimum_bottom_height_m'],.8)
        self.assertFalse(c['source_reuse']['window_only_takeoff_exception_reused'])
    def test_pixel_reference_preserved_and_input_unchanged(self):
        before=copy.deepcopy(self.source)
        c=self.convert()
        self.assertEqual(self.source,before)
        self.assertEqual(c['hover_target']['offset_x_px'],53.25)
        self.assertEqual(c['hover_target']['offset_y_px'],15.484375)
    def test_wrong_source_rejected(self):
        self.source['experiment']='traverse'
        with self.assertRaises(ValueError):self.convert()

if __name__=='__main__':unittest.main(verbosity=2)
