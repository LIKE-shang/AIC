"""Standard-library/Fake ROS hold-entry tests; no ROS network, SITL or flight."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/competition_hold.py'
spec = importlib.util.spec_from_file_location('competition_hold_test_app', SCRIPT)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)


def pose(p=(3.5, -2.0, -0.92), yaw=0.2, speed=0., tilt=0.):
    return dict(p=tuple(p), yaw=yaw, speed=speed, tilt=tilt)


class Fixture:
    def __init__(self, enabled=True):
        self.c = app.HoldCore(enabled)
        self.r, self.w = 100., 10.
        self.feed(armed=False, landed=1)

    def feed(self, dt=.05, armed=True, mode='POSCTL', landed=2, p=None,
             estimator=True, graph=True, send=False, tick=True, omit=()):
        self.r += dt
        self.w += dt
        data = dict(state=dict(connected=True, armed=armed, mode=mode),
                    extended=landed, pose=p or pose(), estimator=estimator)
        for name, value in data.items():
            if name not in omit:
                self.c.update(name, self.r, value, self.r, self.w)
        if graph:
            self.c.graph_update('', self.w)
        if tick:
            self.c.tick(self.r, self.w)
        if send and self.c.active:
            self.c.output_sent(self.w)

    def stable(self, **kw):
        for _ in range(25):
            self.feed(**kw)
        return self

    def capture(self):
        self.stable()
        return self.c.capture(self.r, self.w)

    def holding(self):
        assert self.capture()[0]
        for _ in range(36):
            self.feed(send=True)
        assert self.c.prestream_ready(self.w)
        self.feed(mode='OFFBOARD', send=True)
        assert self.c.phase == 'HOLDING'
        return self


class CoreTests(unittest.TestCase):
    def test_default_is_observer_and_no_target(self):
        c = app.HoldCore()
        self.assertFalse(c.enabled)
        self.assertIsNone(c.target)
        self.assertEqual(c.count, 0)

    def test_boolean_intent_only(self):
        for bad in ('true', 'false', 1, None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                app.HoldCore(bad)

    def test_ground_capture_refused_even_with_output_flag(self):
        f = Fixture().stable(armed=False, landed=1)
        self.assertFalse(f.c.capture(f.r, f.w)[0])
        self.assertIsNone(f.c.target)

    def test_airborne_observer_stays_disabled(self):
        f = Fixture(False).stable()
        self.assertEqual(f.c.capture(f.r, f.w)[1], 'OUTPUT_DISABLED_OBSERVE_ONLY')

    def test_starting_process_airborne_latches_refusal(self):
        c = app.HoldCore(True)
        c.update('state', 100., dict(connected=True, armed=True, mode='POSCTL'), 100., 1.)
        self.assertEqual(c.reason, 'START_ON_GROUND_DISARMED')
        c.update('state', 101., dict(connected=True, armed=False, mode='MANUAL'), 101., 2.)
        self.assertEqual(c.phase, 'STOPPED')

    def test_capture_requires_actual_airborne_posctl(self):
        for kw in (dict(mode='ALTCTL'), dict(mode='OFFBOARD'), dict(landed=1), dict(landed=0)):
            f = Fixture().stable(**kw)
            with self.subTest(kw=kw):
                self.assertFalse(f.c.capture(f.r, f.w)[0])

    def test_capture_requires_fresh_data_from_all_streams(self):
        for key in app.TIMEOUTS:
            f = Fixture().stable()
            del f.c.times[key]
            with self.subTest(key=key):
                self.assertFalse(f.c.capture(f.r, f.w)[0])

    def test_capture_requires_graph_freshness(self):
        f = Fixture().stable()
        f.c.graph_wall = f.w - 2.
        self.assertIn('GRAPH_NOT_FRESH', f.c.capture(f.r, f.w)[1])

    def test_unstable_speed_or_tilt_or_span_blocks_capture(self):
        for field, value in (('speed', .20), ('tilt', .30)):
            f = Fixture().stable(p=pose(**{field: value}))
            with self.subTest(field=field):
                self.assertFalse(f.c.capture(f.r, f.w)[0])
        f = Fixture()
        for i in range(25):
            f.feed(p=pose(p=(3.5+i*.006, -2., -.92)))
        self.assertFalse(f.c.capture(f.r, f.w)[0])

    def test_short_burst_after_gap_is_not_one_second_stability(self):
        f = Fixture()
        f.feed(dt=1.0, tick=False)
        for _ in range(4):
            f.feed()
        self.assertFalse(f.c.capture(f.r, f.w)[0])

    def test_capture_uses_latest_airborne_sample_not_startup_zero(self):
        f = Fixture().stable(p=pose(p=(4., -3., -.8), yaw=-.3))
        self.assertTrue(f.c.capture(f.r, f.w)[0])
        self.assertEqual(f.c.target, (4., -3., -.8))
        self.assertEqual(f.c.target_yaw, -.3)

    def test_target_does_not_follow_new_pose(self):
        f = Fixture()
        self.assertTrue(f.capture()[0])
        target, yaw = f.c.target, f.c.target_yaw
        f.feed(p=pose(p=(3.55, -2., -.90), yaw=.25))
        self.assertEqual((f.c.target, f.c.target_yaw), (target, yaw))

    def test_prestream_needs_actual_sends_not_elapsed_time(self):
        f = Fixture()
        self.assertTrue(f.capture()[0])
        for _ in range(40):
            f.feed()
        self.assertFalse(f.c.prestream_ready(f.w))

    def test_prestream_needs_duration_and_continuity(self):
        f = Fixture()
        f.capture()
        for _ in range(30):
            f.feed(dt=.01, send=True)
        self.assertFalse(f.c.prestream_ready(f.w))
        f.feed(dt=.22, send=True)
        self.assertEqual(len(f.c.sent), 1)

    def test_offboard_before_prestream_ready_latches_stop(self):
        f = Fixture()
        f.capture()
        f.feed(mode='OFFBOARD')
        self.assertEqual(f.c.reason, 'OFFBOARD_BEFORE_PRESTREAM_READY')

    def test_normal_prestream_then_observed_offboard(self):
        f = Fixture().holding()
        self.assertTrue(f.c.active)
        self.assertEqual(f.c.phase, 'HOLDING')
        self.assertFalse(f.c.snapshot(f.r, f.w)['output_is_fcu_acknowledged'])

    def test_no_automatic_timer_cutoff_in_healthy_offboard(self):
        f = Fixture().holding()
        for _ in range(350):
            f.feed(mode='OFFBOARD', send=True)
        self.assertEqual(f.c.phase, 'HOLDING')

    def test_prestream_without_mode_switch_expires(self):
        f = Fixture()
        f.capture()
        for _ in range(305):
            f.feed(send=True)
        self.assertEqual(f.c.reason, 'PRESTREAM_EXPIRED_NO_OFFBOARD_OBSERVED')

    def test_mode_exit_stops_and_never_auto_recaptures(self):
        f = Fixture().holding()
        f.feed(mode='POSCTL')
        self.assertEqual(f.c.phase, 'RELEASED')
        self.assertFalse(f.c.capture(f.r, f.w)[0])
        f.feed(mode='OFFBOARD')
        self.assertFalse(f.c.active)

    def test_disarm_stops_even_if_mode_offboard(self):
        f = Fixture().holding()
        f.feed(armed=False, mode='OFFBOARD', landed=1)
        self.assertEqual(f.c.phase, 'RELEASED')

    def test_release_refused_while_offboard(self):
        f = Fixture().holding()
        self.assertFalse(f.c.release(f.r, f.w)[0])
        self.assertEqual(f.c.phase, 'HOLDING')

    def test_release_after_pilot_posctl_succeeds(self):
        f = Fixture().holding()
        f.feed(mode='POSCTL')
        self.assertTrue(f.c.release(f.r, f.w)[0])

    def test_mode_other_than_posctl_stops_prestream(self):
        f = Fixture()
        f.capture()
        f.feed(mode='AUTO.LAND')
        self.assertEqual(f.c.phase, 'RELEASED')

    def test_position_yaw_tilt_limits_are_latched(self):
        cases = (pose(p=(3.81, -2., -.92)), pose(yaw=.5), pose(tilt=.3))
        for p in cases:
            f = Fixture().holding()
            f.feed(mode='OFFBOARD', p=p)
            with self.subTest(p=p):
                self.assertEqual(f.c.phase, 'STOPPED')
                f.feed(mode='OFFBOARD')
                self.assertFalse(f.c.active)

    def test_yaw_wrap_is_not_a_large_rotation(self):
        f = Fixture().stable(p=pose(yaw=math.pi-.01))
        self.assertTrue(f.c.capture(f.r, f.w)[0])
        f.feed(p=pose(yaw=-math.pi+.01))
        self.assertTrue(f.c.active)

    def test_estimator_loss_stops(self):
        f = Fixture().holding()
        f.feed(mode='OFFBOARD', estimator=False)
        self.assertEqual(f.c.reason, 'ESTIMATOR_POSITION_NOT_VALID')

    def test_source_spoof_and_invalid_timestamp_stop(self):
        for source, stamp in (('/bag', 103.2), ('/mavros', float('nan')), ('/mavros', 200.)):
            f = Fixture().holding()
            f.c.update('pose', stamp, pose(), f.r, f.w, source)
            with self.subTest(source=source, stamp=stamp):
                self.assertEqual(f.c.phase, 'STOPPED')

    def test_duplicate_stamp_does_not_refresh_receipt_age(self):
        f = Fixture().holding()
        stamp, arrival = f.c.times['pose']
        f.c.update('pose', stamp, pose(), f.r+.1, f.w+.1)
        self.assertEqual(f.c.times['pose'], (stamp, arrival))
        for _ in range(6):
            f.feed(mode='OFFBOARD', omit=('pose',))
        self.assertIn('STALE:pose', f.c.reason)

    def test_header_backwards_stops(self):
        f = Fixture().holding()
        f.c.update('pose', f.r-.1, pose(), f.r, f.w)
        self.assertEqual(f.c.reason, 'SOURCE_TIME_REVERSED:pose')

    def test_loop_stall_and_ros_clock_jump_stop(self):
        for dr, dw in ((.3, .3), (1.0, .05), (-.05, .05)):
            f = Fixture().holding()
            f.c.tick(f.r+dr, f.w+dw)
            with self.subTest(dr=dr, dw=dw):
                self.assertEqual(f.c.reason, 'CONTROL_LOOP_OR_CLOCK_GAP')

    def test_master_query_staleness_warns_without_cutting_healthy_hold(self):
        f = Fixture().holding()
        for _ in range(50):
            f.feed(mode='OFFBOARD', graph=False, send=True)
        self.assertEqual(f.c.phase, 'HOLDING')
        self.assertFalse(f.c.snapshot(f.r, f.w)['graph_fresh'])

    def test_actual_graph_conflict_stops_hold(self):
        f = Fixture().holding()
        f.c.graph_update('COMPETING_CONTROL:/mavros/setpoint_position/local', f.w)
        self.assertFalse(f.c.active)

    def test_graph_staleness_blocks_prestream(self):
        f = Fixture()
        f.capture()
        for _ in range(35):
            f.feed(graph=False, send=True)
        self.assertEqual(f.c.reason, 'GRAPH_NOT_FRESH_BEFORE_OFFBOARD')


class Stamp:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value


class PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ, IGNORE_YAW_RATE = 64, 128, 256, 2048
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.position = NS(x=0., y=0., z=0.)
        self.yaw = 0.


class Pub:
    def __init__(self): self.messages, self.connections, self.closed = [], 1, False
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))
    def get_num_connections(self): return self.connections
    def unregister(self): self.closed = True


class Graph:
    def __init__(self):
        self.pubs = {t: ['/mavros'] for t in app.TOPICS.values()}
        self.subs, self.services = {app.OUTPUT: ['/mavros']}, {}
    def getSystemState(self):
        return list(self.pubs.items()), list(self.subs.items()), list(self.services.items())


class Ros:
    def __init__(self):
        self.r, self.w = 100., 10.
        self.params, self.pubs, self.remaps = {}, {}, {}
        self.Time = NS(now=lambda: Stamp(self.r), from_sec=Stamp)
    def get_name(self): return '/window_pass_hold_test_123'
    def resolve_name(self, n): return self.remaps.get(n, n)
    def get_param(self, n, default=None): return self.params.get(n, default)
    def Publisher(self, n, t, **kw):
        self.pubs[n] = Pub()
        return self.pubs[n]
    def Subscriber(self, *args, **kw): return NS()
    def Service(self, *args, **kw): return NS()
    def on_shutdown(self, cb): self.shutdown_cb = cb
    def logerr(self, *args): pass
    def loginfo(self, *args): pass
    def is_shutdown(self): return False


TYPES = NS(State=NS, ExtendedState=NS, EstimatorStatus=NS, Odometry=NS,
           PositionTarget=PositionTarget, String=lambda data: NS(data=data),
           Trigger=NS, TriggerResponse=lambda a, b: NS(success=a, message=b))


def messages(stamp, armed=False, mode='MANUAL', landed=1, p=(3.5, -2., -.92)):
    h = lambda frame='': NS(stamp=Stamp(stamp), frame_id=frame)
    state = NS(header=h(), connected=True, armed=armed, mode=mode)
    ext = NS(header=h(), landed_state=landed)
    odom = NS(header=h('map'), child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=p[0], y=p[1], z=p[2]),
                              orientation=NS(x=0., y=0., z=0., w=-1.))),
              twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.))))
    est = NS(header=h(), attitude_status_flag=True, velocity_horiz_status_flag=True,
             pos_horiz_rel_status_flag=True, pos_horiz_abs_status_flag=False,
             pos_vert_abs_status_flag=True, const_pos_mode_status_flag=False)
    result = dict(state=state, extended=ext, pose=odom, estimator=est)
    for k, m in result.items():
        m._connection_header = dict(callerid='/mavros', latching='1' if k == 'state' else '0')
    return result


class TransportTests(unittest.TestCase):
    def make(self, enabled=True, edit=None):
        r, g = Ros(), Graph()
        if edit: edit(r, g)
        n = app.HoldNode(r, g, TYPES, enabled, lambda: r.w)
        self.addCleanup(n.close)
        self.feed(n, r, armed=False)
        return n, r, g

    def feed(self, n, r, armed=True, mode='POSCTL', landed=None, p=(3.5, -2., -.92), dt=.05):
        r.r += dt
        r.w += dt
        for k, m in messages(r.r, armed, mode, (2 if armed else 1) if landed is None else landed, p).items():
            n.receive(k, m)
        n.core.graph_update('', r.w)
        n.once()

    def prepare(self, n, r):
        for _ in range(25): self.feed(n, r)
        result = n.service('capture')
        self.assertTrue(result.success, result.message)
        return n, r

    def holding(self):
        n, r, g = self.make()
        self.prepare(n, r)
        for _ in range(36): self.feed(n, r)
        self.feed(n, r, mode='OFFBOARD')
        self.assertEqual(n.core.phase, 'HOLDING')
        return n, r, g

    def test_observer_advertises_no_mavros_output(self):
        n, r, _ = self.make(False)
        for _ in range(30): self.feed(n, r)
        self.assertFalse(n.service('capture').success)
        self.assertNotIn(app.OUTPUT, r.pubs)
        self.assertEqual(n.core.count, 0)

    def test_enabled_cli_alone_still_advertises_no_mavros_output(self):
        n, r, _ = self.make()
        self.assertFalse(n.service('capture').success)
        self.assertNotIn(app.OUTPUT, r.pubs)

    def test_payload_mask_enu_negative_z_and_latched_current_target(self):
        n, r, _ = self.make()
        self.prepare(n, r)
        self.feed(n, r, p=(3.55, -2., -.90))
        m = r.pubs[app.OUTPUT].messages[-1]
        self.assertEqual(m.type_mask, 2552)
        self.assertEqual(m.coordinate_frame, 1)
        self.assertEqual(m.header.frame_id, 'map')
        self.assertEqual((m.position.x, m.position.y, m.position.z), (3.5, -2., -.92))
        self.assertAlmostEqual(m.yaw, 0.)
        self.assertEqual(m.header.stamp.to_sec(), r.r)

    def test_handshake_without_connection_does_not_count_prestream(self):
        n, r, _ = self.make()
        self.prepare(n, r)
        r.pubs[app.OUTPUT].connections = 0
        for _ in range(10): self.feed(n, r)
        self.assertEqual(n.core.count, 0)
        self.assertFalse(n.core.prestream_ready(r.w))

    def test_connection_timeout_before_first_publish(self):
        n, r, _ = self.make()
        self.prepare(n, r)
        r.pubs[app.OUTPUT].connections = 0
        for _ in range(65): self.feed(n, r)
        self.assertEqual(n.core.reason, 'MAVROS_SETPOINT_CONNECTION_LOST')
        self.assertEqual(n.core.count, 0)

    def test_connection_loss_and_pilot_exit_unregister_output(self):
        n, r, _ = self.holding()
        pub = r.pubs[app.OUTPUT]
        pub.connections = 0
        n.once()
        self.assertTrue(pub.closed)
        self.assertIsNone(n.pub)
        n, r, _ = self.holding()
        pub = r.pubs[app.OUTPUT]
        self.feed(n, r, mode='POSCTL')
        count = len(pub.messages)
        n.once()
        self.assertEqual(len(pub.messages), count)
        self.assertTrue(pub.closed)

    def test_bad_frame_nan_quaternion_refused(self):
        for mutate in (lambda m: setattr(m.header, 'frame_id', 'odom'),
                       lambda m: setattr(m, 'child_frame_id', 'body'),
                       lambda m: setattr(m.pose.pose.position, 'x', float('nan')),
                       lambda m: setattr(m.pose.pose.orientation, 'w', 0.)):
            n, r, _ = self.holding()
            r.r += .05
            r.w += .05
            m = messages(r.r)['pose']
            mutate(m)
            n.receive('pose', m)
            n.once()
            self.assertFalse(n.core.active)
            self.assertIsNone(n.pub)

    def test_unit_quaternion_sign_does_not_change_yaw(self):
        m = messages(100.)['pose']
        a = app.parse_pose(m)
        m.pose.pose.orientation.w = 1.
        self.assertEqual(a, app.parse_pose(m))

    def test_latched_state_from_mavros_is_accepted(self):
        n, r, _ = self.make()
        self.assertTrue(n.core.startup_disarmed_seen)
        self.assertNotIn('state', n.core.bad)

    def test_wrong_source_stops_and_no_fresh_timestamp_forgery(self):
        n, r, _ = self.holding()
        r.r += .05
        r.w += .05
        m = messages(r.r)['pose']
        m._connection_header['callerid'] = '/rosbag'
        n.receive('pose', m)
        n.once()
        self.assertFalse(n.core.active)
        self.assertIsNone(n.pub)

    def test_const_position_estimator_is_not_valid(self):
        n, r, _ = self.holding()
        r.r += .05
        r.w += .05
        m = messages(r.r)['estimator']
        m.const_pos_mode_status_flag = True
        n.receive('estimator', m)
        self.assertEqual(n.core.reason, 'ESTIMATOR_POSITION_NOT_VALID')

    def test_remaps_simulation_and_hidden_tf_control_refused(self):
        for name in app.FALSE_PARAMS + ('~enable_hold_output',):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.make(edit=lambda r, g: r.params.update({name: True}))
        with self.assertRaises(ValueError):
            self.make(edit=lambda r, g: r.remaps.update({app.OUTPUT: '/different'}))

    def test_mavros_feedback_topics_are_not_competing_control(self):
        g = Graph()
        for name in app.TELEMETRY: g.pubs[name] = ['/mavros']
        self.assertEqual(app.graph_reason(g.getSystemState(), '/ours'), '')

    def test_other_feedback_publisher_and_actual_controls_are_refused(self):
        for topic in tuple(app.DIRECT)+tuple(app.TELEMETRY)+(app.OUTPUT, '/mavros/setpoint_position/local'):
            g = Graph()
            g.pubs[topic] = ['/other']
            with self.subTest(topic=topic):
                self.assertTrue(app.graph_reason(g.getSystemState(), '/ours').startswith('COMPETING_CONTROL:'))

    def test_duplicates_refused_even_if_telemetry_missing(self):
        for services in (False, True):
            g = Graph()
            g.pubs.clear()
            if services: g.services[app.CAPTURE] = ['/other']
            else: g.pubs[app.STATUS] = ['/other']
            self.assertIn('ALREADY_RUNNING', app.graph_reason(g.getSystemState(), '/ours'))

    def test_own_output_and_recording_subscriber_are_allowed(self):
        g = Graph()
        g.pubs[app.OUTPUT] = ['/ours']
        g.subs[app.OUTPUT].append('/recorder')
        self.assertEqual(app.graph_reason(g.getSystemState(), '/ours'), '')

    def test_startup_competing_output_does_not_advertise_ours(self):
        with self.assertRaises(ValueError):
            self.make(edit=lambda r, g: g.pubs.update({app.OUTPUT: ['/other']}))

    def test_close_does_not_call_any_fcu_service(self):
        n, r, _ = self.holding()
        pub = r.pubs[app.OUTPUT]
        n.close()
        self.assertTrue(pub.closed)
        self.assertIsNone(n.pub)
        self.assertNotIn('/mavros/vision_pose/pose', r.pubs)

    def test_logger_reads_pose_history_under_lock(self):
        n, r, _ = self.make()
        original = n.core.capture_reason
        def guarded(*args):
            self.assertTrue(n.lock._is_owned())
            return original(*args)
        n.core.capture_reason = guarded
        n.monitor = lambda: None
        r.loginfo = lambda *args: n.event.set()
        n.run()

    def test_import_and_check_are_offline_python38_compatible(self):
        ast.parse(SCRIPT.read_text(encoding='utf-8'), feature_version=(3, 8))
        result = subprocess.run([sys.executable, str(SCRIPT), '--check'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout)
        self.assertFalse(payload['ROS_started'])
        self.assertFalse(payload['real_output'])
        for forbidden in ('ServiceProxy', 'subprocess.', 'os.system', 'exec(', 'eval('):
            self.assertNotIn(forbidden, SCRIPT.read_text(encoding='utf-8'))

    def test_output_cannot_be_enabled_in_check_or_by_target_arguments(self):
        for args in (['--enable-hold-output'], ['--check', '--enable-hold-output'], ['--target-x', '0']):
            result = subprocess.run([sys.executable, str(SCRIPT)]+args, capture_output=True, text=True)
            with self.subTest(args=args):
                self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main()
