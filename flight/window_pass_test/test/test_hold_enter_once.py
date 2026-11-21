"""Offline entry-helper regression: never imports ROS or contacts a vehicle."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'tools/hold_enter_once.py'
spec = importlib.util.spec_from_file_location('hold_enter_once_test_app', SCRIPT)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)


def status(**kw):
    result = dict(version='20261011_v1', phase='OBSERVE', input_ready=True,
                  connected=True, armed=True, airborne=True, output_enabled=True,
                  startup_disarmed_seen=True, stop_reason='', graph_error='',
                  graph_fresh=True, mode='POSCTL', capture_ready=True,
                  prestream_ready=False, output_active=False)
    result.update(kw)
    return result


def prestream(**kw):
    result = status(phase='PRESTREAM', capture_ready=False, output_active=True)
    result.update(kw)
    return result


def holding(**kw):
    result = status(phase='HOLDING', mode='OFFBOARD', capture_ready=False,
                    output_active=True, prestream_ready=False)
    result.update(kw)
    return result


class Fixture:
    def __init__(self, messages=None):
        self.messages = list(messages if messages is not None else
                             [status(), prestream(), prestream(prestream_ready=True), holding()])
        self.wall = 100.0
        self.captures = self.requests = 0
        self.log = []
        self.capture_ok = self.mode_sent = True
        self.capture_delay = self.request_delay = 0.0

    def read(self, timeout):
        if not self.messages:
            self.wall += timeout
            raise TimeoutError('no new status')
        message = self.messages.pop(0)
        if isinstance(message, tuple):
            delay, message = message
        else:
            delay = min(.2, timeout)
        self.wall += delay
        if isinstance(message, Exception):
            raise message
        return copy.deepcopy(message)

    def capture(self):
        self.captures += 1
        self.wall += self.capture_delay
        return NS(success=self.capture_ok, message='test capture response')

    def request(self):
        self.requests += 1
        self.wall += self.request_delay
        return NS(mode_sent=self.mode_sent)

    def run(self):
        return app.attempt(self.read, self.capture, self.request,
                           clock=lambda: self.wall, emit=self.log.append)


class EntryTests(unittest.TestCase):
    def test_healthy_prestream_switches_once_then_requires_actual_holding(self):
        f = Fixture()
        result = f.run()
        self.assertEqual(result['phase'], 'HOLDING')
        self.assertEqual((f.captures, f.requests), (1, 1))
        self.assertTrue(any('OFFBOARD_OBSERVED' in line for line in f.log))

    def test_ground_wrong_mode_stale_or_disabled_never_captures(self):
        for change in (dict(armed=False), dict(airborne=False), dict(mode='AUTO.LAND'),
                       dict(mode='OFFBOARD'), dict(input_ready=False), dict(output_enabled=False),
                       dict(connected=False), dict(graph_fresh=False), dict(graph_error='busy'),
                       dict(startup_disarmed_seen=False), dict(capture_ready=False),
                       dict(phase='STOPPED'), dict(version='unknown'), dict(stop_reason='fault')):
            with self.subTest(change=change):
                f = Fixture([status(**change)])
                with self.assertRaisesRegex(RuntimeError, 'NOT_READY'):
                    f.run()
                self.assertEqual((f.captures, f.requests), (0, 0))

    def test_capture_failure_never_requests_mode(self):
        f = Fixture()
        f.capture_ok = False
        with self.assertRaisesRegex(RuntimeError, 'CAPTURE_REJECTED'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_capture_delayed_past_deadline_never_requests_mode(self):
        f = Fixture()
        f.capture_delay = 8.0
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_NOT_READY'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_readiness_late_return_never_requests_mode(self):
        f = Fixture([status(), (8.0, prestream(prestream_ready=True))])
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_NOT_READY'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_pre_capture_status_is_not_permission_to_switch(self):
        f = Fixture([status(), status(prestream_ready=True), prestream(),
                     prestream(prestream_ready=True), holding()])
        f.run()
        self.assertEqual(f.requests, 1)

    def test_no_readiness_within_7s_no_request(self):
        f = Fixture([status()] + [(1.0, prestream())]*8)
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_NOT_READY'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_stopped_disarmed_mode_exit_or_graph_fault_cannot_request_mode(self):
        for change in (dict(phase='STOPPED'), dict(phase='RELEASED'), dict(armed=False),
                       dict(airborne=False), dict(mode='ALTCTL'), dict(mode='OFFBOARD'),
                       dict(input_ready=False), dict(output_active=False),
                       dict(stop_reason='PRESTREAM_EXPIRED_NO_OFFBOARD_OBSERVED'),
                       dict(graph_error='busy')):
            with self.subTest(change=change):
                f = Fixture([status(), prestream(prestream_ready=True, **change)])
                with self.assertRaises(RuntimeError):
                    f.run()
                self.assertEqual(f.requests, 0)

    def test_stale_graph_readiness_does_not_request(self):
        f = Fixture([status()] + [(1.0, prestream(prestream_ready=True, graph_fresh=False))]*8)
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_NOT_READY'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_request_not_sent_aborts_without_retry(self):
        f = Fixture()
        f.mode_sent = False
        with self.assertRaisesRegex(RuntimeError, 'DID_NOT_SEND'):
            f.run()
        self.assertEqual(f.requests, 1)

    def test_mode_sent_is_not_confirmation(self):
        f = Fixture([status(), prestream(prestream_ready=True)] + [(1.0, prestream())]*6)
        with self.assertRaisesRegex(RuntimeError, 'OFFBOARD_NOT_OBSERVED'):
            f.run()
        self.assertEqual(f.requests, 1)
        self.assertFalse(any('OFFBOARD_OBSERVED' in line for line in f.log))

    def test_mode_response_too_late_is_not_confirmation(self):
        f = Fixture()
        f.request_delay = 6.0
        with self.assertRaisesRegex(RuntimeError, 'OFFBOARD_NOT_OBSERVED'):
            f.run()
        self.assertEqual(f.requests, 1)

    def test_after_request_exit_fault_or_stopped_never_reenters(self):
        for change in (dict(phase='RELEASED', mode='POSCTL'), dict(phase='STOPPED'),
                       dict(mode='ALTCTL'), dict(armed=False), dict(input_ready=False),
                       dict(output_active=False), dict(stop_reason='pilot takeover')):
            with self.subTest(change=change):
                f = Fixture([status(), prestream(prestream_ready=True), holding(**change)])
                with self.assertRaises(RuntimeError):
                    f.run()
                self.assertEqual((f.captures, f.requests), (1, 1))

    def test_missing_input_no_request(self):
        f = Fixture([status(), TimeoutError('status timeout')])
        with self.assertRaises(TimeoutError):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_default_and_check_do_not_import_ros(self):
        for args in ([], ['--check']):
            result = subprocess.run([sys.executable, str(SCRIPT)] + args,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('No ROS started', result.stdout)

    def test_nano_python38_syntax_and_no_flight_output_or_cleanup(self):
        text = SCRIPT.read_text(encoding='utf-8')
        ast.parse(text, feature_version=(3, 8))
        for forbidden in ('/mavros/cmd/arming', 'custom_mode=\'AUTO.LAND\'',
                          'rospy.Publisher(', '/mavros/param/set', '/window_pass_hold/release',
                          'kill(', 'Popen(', 'os.system('):
            self.assertNotIn(forbidden, text)
        self.assertEqual(text.count("custom_mode='OFFBOARD'"), 1)



class FirstSendRegressionTests(unittest.TestCase):
    def first_send(self):
        return json.loads((ROOT/'test/fixtures/hold_first_send_20261011.json').read_text(encoding='utf-8'))

    def test_exact_user_capture_snapshot_waits_instead_of_aborting(self):
        f = Fixture([status(), self.first_send(), prestream(output_sent_count=1),
                     prestream(output_sent_count=33, prestream_ready=True), holding(output_sent_count=34)])
        result = f.run()
        self.assertEqual(result['phase'], 'HOLDING')
        self.assertEqual((f.captures, f.requests), (1, 1))

    def test_only_zero_send_snapshots_expire_without_any_mode_request(self):
        f = Fixture([status()] + [(1.0, self.first_send())]*8)
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_NOT_READY_WITHIN_7S'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_zero_send_is_never_itself_permission_for_offboard(self):
        s = self.first_send()
        s['prestream_ready'] = True
        f = Fixture([status(), s])
        with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_ABORT'):
            f.run()
        self.assertEqual(f.requests, 0)

    def test_missing_or_invalid_count_does_not_bypass_inactive_output(self):
        for value in (None, False, True, -1, '0', 0.0, 1, 20):
            with self.subTest(count=value):
                s = self.first_send()
                s['output_sent_count'] = value
                f = Fixture([status(), s])
                with self.assertRaisesRegex(RuntimeError, 'PRESTREAM_ABORT'):
                    f.run()
                self.assertEqual(f.requests, 0)

    def test_faults_during_zero_send_wait_still_abort(self):
        for change in (dict(phase='STOPPED'), dict(phase='RELEASED'), dict(armed=False),
                       dict(mode='ALTCTL'), dict(input_ready=False), dict(output_enabled=False),
                       dict(stop_reason='MAVROS_SETPOINT_CONNECTION_LOST'), dict(graph_error='busy')):
            with self.subTest(change=change):
                s = self.first_send()
                s.update(change)
                f = Fixture([status(), s])
                with self.assertRaises(RuntimeError):
                    f.run()
                self.assertEqual(f.requests, 0)

    def test_zero_send_then_real_transport_stop_never_requests_mode(self):
        s = self.first_send()
        stopped = dict(s, phase='STOPPED', stop_reason='MAVROS_SETPOINT_CONNECTION_LOST')
        f = Fixture([status(), s, s, stopped])
        with self.assertRaisesRegex(RuntimeError, 'MAVROS_SETPOINT_CONNECTION_LOST'):
            f.run()
        self.assertEqual(f.requests, 0)


_hold_spec = importlib.util.spec_from_file_location(
    'hold_lifecycle_fixture', ROOT/'test/test_competition_hold.py')
_hold = importlib.util.module_from_spec(_hold_spec)
_hold_spec.loader.exec_module(_hold)


class CoupledLifecycle:
    """Uses production HoldNode.once/service status, not handwritten ready booleans."""
    def __init__(self, connection_delay=0.25, disconnect_after=None, accept_mode=True):
        self.ros, self.graph = _hold.Ros(), _hold.Graph()
        self.node = _hold.app.HoldNode(self.ros, self.graph, _hold.TYPES, True, lambda: self.ros.w)
        self.mode = 'POSCTL'
        self.capture_time = None
        self.connection_delay, self.disconnect_after = connection_delay, disconnect_after
        self.accept_mode = accept_mode
        self.requests = 0
        self.history, self.log = [], []
        self.advance(armed=False)
        for _ in range(27):
            self.advance()

    def advance(self, armed=True):
        self.ros.r += .05
        self.ros.w += .05
        if self.capture_time is not None and self.node.pub is not None:
            elapsed = self.ros.w - self.capture_time
            connected = elapsed >= self.connection_delay
            if self.disconnect_after is not None and elapsed >= self.disconnect_after:
                connected = False
            self.node.pub.connections = int(connected)
        for name, msg in _hold.messages(self.ros.r, armed=armed, mode=self.mode,
                                       landed=2 if armed else 1).items():
            self.node.receive(name, msg)
        self.node.core.graph_update('', self.ros.w)
        self.node.once()
        s = json.loads(self.ros.pubs[_hold.app.STATUS].messages[-1].data)
        self.history.append(s)
        return s

    def read(self, timeout):
        return self.advance()

    def capture(self):
        result = self.node.service('capture')
        self.capture_time = self.ros.w
        self.node.pub.connections = 0
        return result

    def request(self):
        s = self.history[-1]
        assert s['phase'] == 'PRESTREAM' and s['prestream_ready'] and s['output_active']
        assert s['output_sent_count'] >= 25
        self.requests += 1
        if self.accept_mode:
            self.mode = 'OFFBOARD'
        return NS(mode_sent=True)

    def run(self):
        return app.attempt(self.read, self.capture, self.request,
                           clock=lambda: self.ros.w, emit=self.log.append)


class CoupledLifecycleTests(unittest.TestCase):
    def test_production_status_with_delayed_subscriber_reaches_holding(self):
        for delay in (0.0, 0.1, 0.3, 1.0, 2.9):
            with self.subTest(delay=delay):
                f = CoupledLifecycle(connection_delay=delay)
                self.addCleanup(f.node.close)
                result = f.run()
                self.assertEqual(result['phase'], 'HOLDING')
                self.assertEqual(f.requests, 1)
                if delay >= 0.1:
                    self.assertTrue(any(s['phase'] == 'PRESTREAM' and s['output_sent_count'] == 0
                                        and not s['output_active'] for s in f.history))
                self.assertTrue(f.node.core.active)  # Helper exit does not stop hold/localization.

    def test_controller_connection_timeout_remains_a_real_fault(self):
        f = CoupledLifecycle(connection_delay=3.5)
        self.addCleanup(f.node.close)
        with self.assertRaisesRegex(RuntimeError, 'MAVROS_SETPOINT_CONNECTION_LOST'):
            f.run()
        self.assertEqual(f.node.core.phase, 'STOPPED')
        self.assertEqual(f.requests, 0)

    def test_connection_loss_after_first_send_is_not_reclassified_as_startup(self):
        f = CoupledLifecycle(connection_delay=.1, disconnect_after=.7)
        self.addCleanup(f.node.close)
        with self.assertRaisesRegex(RuntimeError, 'MAVROS_SETPOINT_CONNECTION_LOST'):
            f.run()
        self.assertGreater(f.node.core.count, 0)
        self.assertEqual(f.requests, 0)

    def test_actual_controller_without_offboard_transition_is_not_success(self):
        f = CoupledLifecycle(connection_delay=.2, accept_mode=False)
        self.addCleanup(f.node.close)
        with self.assertRaisesRegex(RuntimeError, 'OFFBOARD_NOT_OBSERVED_WITHIN_5S'):
            f.run()
        self.assertEqual(f.requests, 1)
        self.assertNotEqual(f.node.core.phase, 'HOLDING')


_patch_spec = importlib.util.spec_from_file_location(
    'first_send_patch_test', ROOT/'tools/fix_hold_first_send_20261011.py')
_patch = importlib.util.module_from_spec(_patch_spec)
_patch_spec.loader.exec_module(_patch)


class PatchTests(unittest.TestCase):
    def temporary_file(self, content):
        directory = tempfile.TemporaryDirectory(prefix='hold_fix_test_', dir=ROOT.parent/'_deploy')
        # Any recursive fixture cleanup is confined to this checked workspace child.
        assert (ROOT.parent/'_deploy').resolve() in Path(directory.name).resolve().parents
        self.addCleanup(directory.cleanup)
        path = Path(directory.name)/'hold_enter_once.py'
        path.write_bytes(content)
        return path

    def original(self):
        updated = SCRIPT.read_bytes()
        return updated.replace(_patch.NEW.encode('utf-8'), _patch.OLD.encode('utf-8'), 1)

    def test_guarded_patch_produces_exact_tested_bytes_and_backup(self):
        old = self.original()
        path = self.temporary_file(old)
        self.assertTrue(_patch.apply(path))
        self.assertEqual(path.read_bytes(), SCRIPT.read_bytes())
        self.assertEqual(path.with_name(path.name + '.before_first_send_fix').read_bytes(), old)
        self.assertIn(_patch.NEW_SHA, path.with_name(path.name + '.sha256').read_text())
        self.assertFalse(_patch.apply(path))
        self.assertEqual(path.read_bytes(), SCRIPT.read_bytes())

    def test_unknown_version_is_not_modified(self):
        unknown = self.original() + b'\n# unrelated local edit\n'
        path = self.temporary_file(unknown)
        with self.assertRaisesRegex(RuntimeError, 'version mismatch'):
            _patch.apply(path)
        self.assertEqual(path.read_bytes(), unknown)
        self.assertFalse(path.with_name(path.name + '.before_first_send_fix').exists())

    def test_existing_different_backup_is_preserved(self):
        old = self.original()
        path = self.temporary_file(old)
        backup = path.with_name(path.name + '.before_first_send_fix')
        backup.write_bytes(b'prior unrelated backup')
        with self.assertRaisesRegex(RuntimeError, 'Backup differs'):
            _patch.apply(path)
        self.assertEqual(path.read_bytes(), old)
        self.assertEqual(backup.read_bytes(), b'prior unrelated backup')

    def test_patch_is_python38_compatible_and_offline(self):
        text = (ROOT/'tools/fix_hold_first_send_20261011.py').read_text(encoding='utf-8')
        ast.parse(text, feature_version=(3, 8))
        for forbidden in ('import rospy', 'ServiceProxy', 'subprocess', 'os.system', 'unlink('):
            self.assertNotIn(forbidden, text)

if __name__ == '__main__':
    unittest.main()
