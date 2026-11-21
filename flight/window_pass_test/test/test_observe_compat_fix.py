"""File-patch/profile tests. No ROS or hardware is accessed."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from window_pass.config import DEFAULTS
from window_pass.core import Controller

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "observe_compat_fix_test", ROOT/"tools/fix_observe_compat_20261011_v1.py")
fix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fix)


class ObserveCompatibilityFixTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.graph = Path(self.temp.name)/"graph.py"
        self.profile = Path(self.temp.name)/fix.PROFILE_NAME
        self.current = (ROOT/"src/window_pass/graph.py").read_bytes()
        self.assertEqual(self.current.count(fix.NEW), 1)
        self.original = self.current.replace(fix.NEW, fix.OLD)
        self.graph.write_bytes(self.original)
        self.backup = self.graph.with_name(self.graph.name+".before_desired_fix_20261011_v1")

    def test_exact_patch_and_known_metadata_only(self):
        result = fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertTrue(result["changed"])
        self.assertFalse(result["ROS_started"])
        self.assertEqual(self.graph.read_bytes(), self.current)
        self.assertEqual(self.backup.read_bytes(), self.original)
        cfg = json.loads(self.profile.read_text(encoding="utf-8"))
        self.assertFalse(cfg["enable_motion"])
        self.assertFalse(cfg["enable_traverse"])
        self.assertTrue(all(v is False for k,v in cfg.items() if k.endswith("_verified")))
        for name, value in fix.OBSERVED.items():
            self.assertEqual(cfg[name], value)
        removed = {"missing:"+k for k in fix.OBSERVED}
        self.assertEqual(set(Controller(cfg).blockers), set(Controller().blockers)-removed)
        self.assertIn("missing:window_center_local_m", Controller(cfg).blockers)
        # The profile this fix writes carries no measured uncertainty, so the
        # clearance sum still uses placeholder zeros. What matters here is that
        # the aperture itself is judged against real geometry, so a correctly
        # sized opening must not be blocked by the reserve any more.
        self.assertNotIn("insufficient:worst_case_clearance", Controller(cfg).blockers)

    def test_idempotent_without_changing_original_backup(self):
        fix.repair(self.graph, self.profile, DEFAULTS)
        payload = self.profile.read_bytes()
        result = fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertFalse(result["changed"])
        self.assertEqual(self.backup.read_bytes(), self.original)
        self.assertEqual(self.profile.read_bytes(), payload)

    def test_unknown_source_is_not_overwritten(self):
        source = b"# custom version with no known allowlist\n"
        self.graph.write_bytes(source)
        with self.assertRaisesRegex(ValueError, "Unrecognized"):
            fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertEqual(self.graph.read_bytes(), source)
        self.assertFalse(self.backup.exists())
        self.assertFalse(self.profile.exists())

    def test_ambiguous_source_is_not_overwritten(self):
        source = self.original+b"\n# "+fix.OLD+b"\n"
        self.graph.write_bytes(source)
        with self.assertRaises(ValueError):
            fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertEqual(self.graph.read_bytes(), source)
        self.assertFalse(self.backup.exists())

    def test_existing_profile_is_preserved_and_graph_unchanged(self):
        payload = b'{"user_field_config": true}\n'
        self.profile.write_bytes(payload)
        with self.assertRaisesRegex(ValueError, "profile already"):
            fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertEqual(self.profile.read_bytes(), payload)
        self.assertEqual(self.graph.read_bytes(), self.original)
        self.assertFalse(self.backup.exists())

    def test_existing_different_backup_is_preserved(self):
        payload = b"# another saved version\n"
        self.backup.write_bytes(payload)
        with self.assertRaisesRegex(ValueError, "Backup"):
            fix.repair(self.graph, self.profile, DEFAULTS)
        self.assertEqual(self.backup.read_bytes(), payload)
        self.assertEqual(self.graph.read_bytes(), self.original)
        self.assertFalse(self.profile.exists())

    def test_defaults_are_not_mutated_and_attestations_not_granted(self):
        supplied = copy.deepcopy(DEFAULTS)
        supplied.update(enable_motion=True, enable_traverse=True,
                        pass_stage_approved=True, camera_verified=True)
        snapshot = copy.deepcopy(supplied)
        fix.repair(self.graph, self.profile, supplied)
        self.assertEqual(supplied, snapshot)
        cfg = json.loads(self.profile.read_text(encoding="utf-8"))
        for key in ("enable_motion", "enable_traverse", "pass_stage_approved", "camera_verified"):
            self.assertFalse(cfg[key])
        self.assertIsNone(Controller(cfg).output(10., 10.))

    def test_compiled_module_is_refused(self):
        compiled = self.graph.with_suffix(".pyc")
        compiled.write_bytes(self.original)
        with self.assertRaisesRegex(ValueError, "compiled"):
            fix.repair(compiled, self.profile, DEFAULTS)
        self.assertEqual(compiled.read_bytes(), self.original)
        self.assertFalse(self.profile.exists())


if __name__ == "__main__":
    unittest.main()
