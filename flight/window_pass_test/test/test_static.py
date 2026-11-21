#!/usr/bin/env python3
import ast
import hashlib
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from window_pass.config import DEFAULTS


class StaticTests(unittest.TestCase):
    def test_all_python38_syntax(self):
        for p in ROOT.rglob('*.py'):
            ast.parse(p.read_text(encoding='utf-8-sig'),filename=str(p),feature_version=(3,8))
    def test_xml(self):
        for p in list((ROOT/'launch').glob('*.launch'))+[ROOT/'package.xml']:ET.parse(p)
    def test_no_missions_or_hardware_in_launches(self):
        for p in (ROOT/'launch').glob('*.launch'):
            root=ET.parse(p).getroot()
            for node in root.iter('node'):self.assertEqual(node.get('pkg'),'window_pass_test')
            for include in root.iter('include'):self.assertEqual(include.get('file'),'$(find astra_camera)/launch/gemini.launch')
    def test_observe_forces_both_permissions_false(self):
        node=ET.parse(ROOT/'launch/observe.launch').find('node')
        overrides={p.get('name'):p.get('value') for p in node.findall('param')}
        self.assertEqual(overrides['control/enable_motion'],'false')
        self.assertEqual(overrides['control/enable_traverse'],'false')
    def test_motion_defaults_false_and_no_autostart(self):
        args={a.get('name'):a.get('default') for a in ET.parse(ROOT/'launch/motion.launch').findall('arg')}
        self.assertEqual(args['enable_motion'],'false');self.assertEqual(args['enable_traverse'],'false')
        self.assertIsNone(args['config_file'])
    def test_no_hardware_clients_or_libraries(self):
        for sub in ('src','scripts'):
            for p in (ROOT/sub).rglob('*.py'):
                text=p.read_text(encoding='utf-8')
                for forbidden in ('ServiceProxy(', 'ctypes.', 'CDLL(', 'lib_pwm_control', 'CommandLong(', 'CommandBool(', 'SetMode('):
                    self.assertNotIn(forbidden,text,str(p))
    def test_pinned_window_candidate_revision_not_new_ring_import(self):
        p=ROOT/'src/window_pass/vision_frozen.py'
        self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),'c585040f506561888c8cb6098aaf15676ad8a842c9f099e7d690300bf717f658')
        self.assertNotIn('air_target',p.read_text())
    def test_original_rgb_fixture_integrity(self):
        self.assertEqual(hashlib.sha256((ROOT/'test/window_checker.png').read_bytes()).hexdigest(),'e7ca4d3b5920864301805bd0048261c3d0f1dbd695f7438dc71420cbc8e66f8a')
    def test_full_frozen_yaml_byte_identity(self):
        self.assertEqual(hashlib.sha256((ROOT/'config/vision_frozen.yaml').read_bytes()).hexdigest(),'254b0a0b5d5d52bf672c59b6c8a6f8ea9631362760aa300ccd564b6ab1d71ff8')
    def test_no_unrecorded_default_values(self):
        import json
        cfg={}
        for line in (ROOT/'config/control.yaml').read_text().splitlines():
            if line and not line.startswith('#'):
                k,v=line.split(': ',1);cfg[k]=json.loads(v)
        self.assertEqual(cfg,DEFAULTS)
    def test_generated_message_and_python_install_wiring(self):
        cm=(ROOT/'CMakeLists.txt').read_text()
        for part in ('WindowObservation.msg','message_runtime','catkin_python_setup()','scripts/window_observer.py','scripts/window_pass_node.py'):
            self.assertIn(part,cm)
        msg=(ROOT/'msg/WindowObservation.msg').read_text()
        for field in ('std_msgs/Header header','opening_corners_px','detection_method','geometry_kind','target_count','camera_info_sha256'):
            self.assertIn(field,msg)


if __name__=='__main__':unittest.main(verbosity=2)
