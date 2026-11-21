#!/usr/bin/env python3
"""Exact known-version repair and profile tests; no ROS or OpenCV needed."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from window_pass.config import DEFAULTS, load_config
from window_pass.core import Controller
sys.path.insert(0,str(ROOT/'test'))
from fixtures import cleanup_tree
spec=importlib.util.spec_from_file_location('window_candidate_fix',ROOT/'tools/fix_window_candidate_20261012_v1.py')
fix=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fix)


class WindowCandidateInstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'test')
        self.addCleanup(cleanup_tree, self.temp.name)
        self.root=Path(self.temp.name).resolve()
        self.root.relative_to((ROOT/'test').resolve())
        self.current=(ROOT/'src/window_pass/vision_frozen.py').read_bytes()
        self.old=self.current
        for old,new in reversed(fix.REPLACEMENTS):
            self.assertEqual(self.old.count(new.encode()),1)
            self.old=self.old.replace(new.encode(),old.encode())
        self.assertEqual(fix.sha256(self.old),fix.OLD_SHA256)
        self.source=self.root/'vision_frozen.py'
        self.source.write_bytes(self.old)
        self.base=self.root/'base.json'
        self.output=self.root/'new_base.json'
        self.config=copy.deepcopy(DEFAULTS)
        self.config.update(algorithm_sha256=fix.OLD_SHA256,vision_profile_sha256=fix.PROFILE_SHA256)
        self.base.write_text(json.dumps(self.config),encoding='utf-8')
        self.backup=self.source.with_name(self.source.name+fix.BACKUP_SUFFIX)

    def test_exact_repair_backup_and_new_observe_base(self):
        before=self.base.read_bytes()
        result=fix.repair(self.source,self.base,self.output)
        self.assertEqual(result['result'],'WINDOW_CANDIDATE_FIX_OK')
        self.assertTrue(result['changed'])
        self.assertEqual(self.source.read_bytes(),self.current)
        self.assertEqual(self.backup.read_bytes(),self.old)
        self.assertEqual(self.base.read_bytes(),before)
        output=load_config(json.loads(self.output.read_text(encoding='utf-8')))
        self.assertEqual(output['algorithm_sha256'],fix.NEW_SHA256)
        self.assertFalse(output['enable_motion'])
        self.assertFalse(output['enable_traverse'])
        self.assertIsNone(Controller(output).output(0,0))

    def test_idempotent_reapply_preserves_backup(self):
        fix.repair(self.source,self.base,self.output)
        result=fix.repair(self.source,self.base,self.output)
        self.assertFalse(result['changed'])
        self.assertEqual(self.backup.read_bytes(),self.old)

    def test_unknown_detector_rejected_before_files_are_created(self):
        self.source.write_bytes(self.old+b'\n# other change\n')
        with self.assertRaisesRegex(ValueError,'UNKNOWN_DETECTOR'):
            fix.repair(self.source,self.base,self.output)
        self.assertFalse(self.output.exists());self.assertFalse(self.backup.exists())
        self.assertTrue(self.source.read_bytes().endswith(b'# other change\n'))

    def test_unknown_base_hash_rejected_before_source_is_changed(self):
        for key in ('algorithm_sha256','vision_profile_sha256'):
            with self.subTest(key=key):
                cfg=dict(self.config,**{key:'f'*64})
                self.base.write_text(json.dumps(cfg),encoding='utf-8')
                with self.assertRaisesRegex(ValueError,'MISMATCH'):
                    fix.repair(self.source,self.base,self.output)
                self.assertEqual(self.source.read_bytes(),self.old)
                self.assertFalse(self.backup.exists())

    def test_conflicting_output_refused_before_source_changes(self):
        self.output.write_bytes(b'keep this')
        with self.assertRaisesRegex(ValueError,'DIFFERENT_OUTPUT'):
            fix.repair(self.source,self.base,self.output)
        self.assertEqual(self.source.read_bytes(),self.old)
        self.assertEqual(self.output.read_bytes(),b'keep this')
        self.assertFalse(self.backup.exists())

    def test_conflicting_backup_refused(self):
        self.backup.write_bytes(b'keep backup')
        with self.assertRaisesRegex(ValueError,'DIFFERENT_DETECTOR_BACKUP'):
            fix.repair(self.source,self.base,self.output)
        self.assertEqual(self.source.read_bytes(),self.old)
        self.assertEqual(self.backup.read_bytes(),b'keep backup')

    def test_overwriting_old_base_is_refused(self):
        with self.assertRaisesRegex(ValueError,'NEW_BASE_PATH'):
            fix.repair(self.source,self.base,self.base)
        self.assertEqual(self.source.read_bytes(),self.old)

    def test_cannot_inherit_motion_authorization_or_change_budgets(self):
        cfg=copy.deepcopy(self.config)
        cfg.update(enable_motion=True,enable_traverse=True,pass_stage_approved=True)
        for key in cfg:
            if key.endswith('_verified'):cfg[key]=True
        before=copy.deepcopy(cfg)
        result=fix.observe_profile(cfg)
        self.assertEqual(cfg,before)
        for key,value in result.items():
            if key.endswith('_verified') or key in ('enable_motion','enable_traverse','pass_stage_approved'):
                self.assertFalse(value,key)
            elif key != 'algorithm_sha256':
                self.assertEqual(value,cfg[key],key)

    def test_original_pinned_algorithm_reconstruction_is_byte_exact(self):
        self.assertEqual(fix.patched_bytes(self.old),self.current)
        self.assertEqual(hashlib.sha256(self.current).hexdigest(),fix.NEW_SHA256)


if __name__=='__main__':
    unittest.main(verbosity=2)
