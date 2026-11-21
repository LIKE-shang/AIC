"""No ROS execution: verify recovery cannot start legacy camera/control paths."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


RECOVERY = load('recovery_test', ROOT/'scripts/recover_base.py')
LEGACY = load('legacy_recovery_test', ROOT.parent/'window_pass_session_launch/scripts/session.py')


class RecoveryTests(unittest.TestCase):
    def test_cold_boot_starts_only_localization(self):
        flags = RECOVERY.recovery_plan(LEGACY, {})
        self.assertEqual({key for key, value in flags.items() if value},
                         {'start_mavros','start_lidar','start_lio','start_pose'})

    def test_existing_localization_is_reused(self):
        publishers = {topic: [expected or '/existing_'+key] for key,(topic,expected) in LEGACY.COMPONENTS.items()}
        self.assertFalse(any(RECOVERY.recovery_plan(LEGACY, publishers).values()))

    def test_started_nodes_without_first_output_not_duplicated(self):
        nodes=['/mavros','/window_pass_lio/laserMapping','/livox_lidar_publisher2',
               '/window_pass_competition_pose_session_20261013']
        self.assertFalse(any(RECOVERY.recovery_plan(LEGACY, {}, nodes).values()))

    def test_conflicting_localization_sources_refused(self):
        for topic, nodes in (('/mavros/state',['/mavros','/other']),
                             ('/window_pass_lio/Odometry',['/wrong'])):
            with self.subTest(topic=topic), self.assertRaisesRegex(RuntimeError,'SOURCE_CONFLICT'):
                RECOVERY.recovery_plan(LEGACY,{topic:nodes})

    def test_existing_controller_or_unidentified_pose_refused(self):
        for topic in ('/window_pass_align/status','/window_pass_hold/status','/window_pass_test/status')+RECOVERY.CONTROL_TOPICS+('/mavros/vision_pose/pose',):
            with self.subTest(topic=topic), self.assertRaises(RuntimeError):
                RECOVERY.recovery_plan(LEGACY,{topic:['/existing']})

    def test_launch_command_cannot_enable_legacy_vision(self):
        flags=RECOVERY.recovery_plan(LEGACY,{})
        command=RECOVERY.recovery_command(RECOVERY.LEGACY,flags,'abc123')
        self.assertIn('start_camera:=false',command)
        self.assertIn('start_detector:=false',command)
        self.assertNotIn('trial.launch',' '.join(command))
        for key in ('start_camera','start_detector'):
            invalid=dict(flags);invalid[key]=True
            with self.assertRaises(RuntimeError):
                RECOVERY.recovery_command(RECOVERY.LEGACY,invalid,'abc123')

    def test_deployed_contract_hashes_and_missing_files(self):
        RECOVERY.verify_legacy(ROOT.parent/'window_pass_session_launch')
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(RuntimeError,'EXISTING_SESSION_FILE_MISSING'):
                RECOVERY.verify_legacy(Path(folder))

    def test_modified_legacy_contract_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)
            for name in RECOVERY.LEGACY_HASHES:
                path=target/name;path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes((ROOT.parent/'window_pass_session_launch'/name).read_bytes())
            changed=target/'scripts/session.py'
            changed.write_bytes(changed.read_bytes()+b'\n# changed\n')
            with self.assertRaisesRegex(RuntimeError,'EXISTING_SESSION_VERSION_MISMATCH'):
                RECOVERY.verify_legacy(target)


if __name__=='__main__':unittest.main()
