"""Competition ground regression only: fake ROS, no FCU or sensor execution."""
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest
from types import SimpleNamespace as Obj
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/"scripts/competition_ground.py"
spec = importlib.util.spec_from_file_location("competition_ground", SCRIPT)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)
gm = app.load_gate(ROOT/"competition/pose_guard_v1.py")
NS = app.NS
EMPTY = dict.fromkeys(app.TRANSFORM_KEYS)
IDENTITY = dict(zip(app.TRANSFORM_KEYS, ([0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0])))
GROUND_APPROX = dict(zip(app.TRANSFORM_KEYS,
                         ([0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0.04])))


class TransformTests(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=10)

    def test_unmeasured_defaults_are_raw_not_assumed_identity_installation(self):
        transform = app.FrameTransform(EMPTY)
        self.assertFalse(transform.configured)
        self.assertEqual(transform.frame, "camera_init")
        self.assertEqual(transform.child, "body")
        self.assertEqual(transform.apply((1, 2, 3), (0, 0, 0, 1)),
                         ((1, 2, 3), (0, 0, 0, 1)))

    def test_identity(self):
        transform = app.FrameTransform(IDENTITY)
        p, q = transform.apply((1, 2, 3), (0, 0, 0, 1))
        self.assertVector(p, (1, 2, 3))
        self.assertVector(q, (0, 0, 0, 1))
        self.assertEqual(transform.frame, "competition_world")

    def test_lever_arm_subtracted_not_added(self):
        cfg = dict(IDENTITY, lio_origin_in_body_m=[0.1, -0.2, 0.3])
        p, q = app.FrameTransform(cfg).apply((1, 2, 3), (0, 0, 0, 1))
        self.assertVector(p, (0.9, 2.2, 2.7))

    def test_lever_arm_rotates_with_vehicle(self):
        cfg = dict(IDENTITY, lio_origin_in_body_m=[1, 0, 0])
        yaw90 = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        p, q = app.FrameTransform(cfg).apply((0, 0, 0), yaw90)
        self.assertVector(p, (0, -1, 0))
        self.assertVector(q, yaw90)

    def test_world_rotation_affects_position_and_orientation(self):
        yaw90 = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
        cfg = dict(IDENTITY, world_from_camera_init_xyzw=yaw90)
        p, q = app.FrameTransform(cfg).apply((1, 0, 0), (0, 0, 0, 1))
        self.assertVector(p, (0, 1, 0))
        self.assertVector(q, yaw90)

    def test_mount_rotation_uses_inverse(self):
        yaw90 = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
        cfg = dict(IDENTITY, body_from_lio_body_xyzw=yaw90)
        p, q = app.FrameTransform(cfg).apply((0, 0, 0), yaw90)
        self.assertVector(q, (0, 0, 0, 1))

    def test_general_rigid_transform_recovers_vehicle_pose(self):
        # Independent synthetic assembly: W->B, B->I, then recover W->B.
        q_wb = app.unit_quaternion([.3, .2, -.1, math.sqrt(.86)], "test")
        q_bi = app.unit_quaternion([-.2, .1, .1, math.sqrt(.94)], "test")
        p_wb, arm = (2, -1, 4), (.25, -.14, .12)
        q_wi = app.multiply(q_wb, q_bi)
        arm_w = app.rotate(q_wb, arm)
        p_wi = tuple(a+b for a, b in zip(p_wb, arm_w))
        cfg = dict(IDENTITY, body_from_lio_body_xyzw=list(q_bi), lio_origin_in_body_m=list(arm))
        p, q = app.FrameTransform(cfg).apply(p_wi, q_wi)
        self.assertVector(p, p_wb)
        self.assertVector(q, q_wb)

    def test_partial_transform_is_error_not_silent_identity(self):
        with self.assertRaisesRegex(ValueError, "Partial"):
            app.FrameTransform(dict(EMPTY, lio_origin_in_body_m=[0, 0, 0]))

    def test_invalid_values_and_extra_output_switch_refused(self):
        for key, value in ((app.TRANSFORM_KEYS[0], [0, 0, 0, 0]),
                           (app.TRANSFORM_KEYS[1], [0, 0, 0, 2]),
                           (app.TRANSFORM_KEYS[2], [0, float("nan"), 0]),
                           (app.TRANSFORM_KEYS[2], [True, 0, 0])):
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    app.FrameTransform(dict(IDENTITY, **{key: value}))
        with self.assertRaises(ValueError):
            app.FrameTransform(dict(EMPTY, real_output=True))


class Stamp:
    def __init__(self, ns):
        self.secs, self.nsecs = divmod(ns, NS)

    def to_nsec(self):
        return self.secs*NS+self.nsecs


class Clock:
    def __init__(self):
        self.ros, self.wall = 10*NS, 50*NS

    def advance(self, ns):
        self.ros += ns
        self.wall += ns


class Publisher:
    def __init__(self, name, **kw):
        self.name, self.options, self.messages = name, kw, []
        self.closed, self.fail = False, False
        self.connections = 1

    def get_num_connections(self):
        return self.connections

    def publish(self, msg):
        if self.closed or self.fail:
            raise RuntimeError("publish error")
        self.messages.append(copy.deepcopy(msg))

    def unregister(self):
        self.closed = True


class Ros:
    def __init__(self, clock, remaps=None, params=None):
        self.Time = Obj(now=lambda: Stamp(clock.ros))
        self.remaps, self.params = remaps or {}, params or {}
        self.publishers, self.subscribers, self.logs = {}, [], []
        self.shutdown = False

    def resolve_name(self, name):
        return self.remaps.get(name, name)

    def get_name(self):
        return "/window_pass_competition_ground_test"

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, topic, typ, **kw):
        pub = Publisher(topic, **kw)
        self.publishers[topic] = pub
        return pub

    def Subscriber(self, topic, typ, callback, **kw):
        sub = Obj(topic=topic, callback=callback, closed=False)
        sub.unregister = lambda: setattr(sub, "closed", True)
        self.subscribers.append(sub)
        return sub

    def is_shutdown(self):
        return self.shutdown

    def log(self, *args):
        self.logs.append(args)

    loginfo = logwarn = logerr = log


class Graph:
    def __init__(self, pubs=None):
        self.pubs = pubs or {}

    def getSystemState(self):
        return list(self.pubs.items()), [], []


class PoseStamped:
    pass


class String:
    def __init__(self, data):
        self.data = data


TYPES = dict(pose=PoseStamped, odom=object, string=String, state=object,
             estimator=object, local_odom=object, observation=object)


def odometry(stamp, seq=1, parent="camera_init", child="body", source=gm.SOURCE_NODE):
    return Obj(header=Obj(stamp=Stamp(stamp), seq=seq, frame_id=parent), child_frame_id=child,
               pose=Obj(pose=Obj(position=Obj(x=.2, y=.3, z=.4),
                                 orientation=Obj(x=0., y=0., z=0., w=1.))),
               _connection_header={"callerid": source, "latching": "0"})


def telemetry(clock, **fields):
    return Obj(header=Obj(stamp=Stamp(clock.ros)),
               _connection_header={"callerid": "/mavros", "latching": "0"}, **fields)


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.ros = Clock(), None
        self.ros = Ros(self.clock)
        self.node = app.GroundBridge(self.ros, Graph({app.INPUT: [gm.SOURCE_NODE]}), TYPES,
                                    gm, app.FrameTransform(EMPTY), lambda: self.clock.wall)
        self.addCleanup(self.node.close)
        self.pub = self.ros.publishers[app.OUTPUT]

    def feed(self, seq=1, **kw):
        msg = odometry(self.clock.ros-20_000_000, seq=seq, **kw)
        self.node.on_odometry(msg)
        return msg

    def status(self):
        return self.node.snapshot(self.clock.ros, self.clock.wall)

    def test_exact_stamp_raw_pose_and_diagnostic_topics_only(self):
        msg = self.feed()
        out = self.pub.messages[-1]
        self.assertEqual(out.header.stamp.to_nsec(), msg.header.stamp.to_nsec())
        self.assertEqual(out.header.frame_id, "camera_init")
        self.assertEqual(out.pose, msg.pose.pose)
        self.assertEqual(set(self.ros.publishers), {app.OUTPUT, app.STATUS})
        self.assertTrue(all(not p.options["latch"] for p in self.ros.publishers.values()))
        self.assertFalse(any(n.startswith("/mavros/") for n in self.ros.publishers))

    def test_configured_preview_still_never_publishes_real_mavros(self):
        self.node.transform = app.FrameTransform(dict(IDENTITY, lio_origin_in_body_m=[.1, 0, 0]))
        msg = self.feed()
        out = self.pub.messages[-1]
        self.assertEqual(out.header.stamp.to_nsec(), msg.header.stamp.to_nsec())
        self.assertAlmostEqual(out.pose.position.x, .1)
        self.assertEqual(out.header.frame_id, "competition_world")
        self.assertTrue(self.status()["diagnostic_only"])
        self.assertFalse(self.status()["flight_authorized"])

    def test_no_timer_replay_and_05s_timeout_latches(self):
        self.feed()
        for _ in range(10):
            self.clock.advance(50_000_000)
            self.node.tick()
        self.assertEqual(len(self.pub.messages), 1)
        self.assertIn("INPUT_STALE", self.node.gate.fault)
        self.clock.advance(100_000_000)
        self.feed(seq=2)
        self.assertEqual(len(self.pub.messages), 1)

    def test_callback_after_gap_stops_even_without_watchdog_tick(self):
        self.feed()
        self.clock.advance(NS)
        self.feed(seq=2)
        self.assertEqual(len(self.pub.messages), 1)
        self.assertTrue(self.node.gate.fault)

    def test_duplicates_do_not_keep_gate_alive(self):
        msg = self.feed()
        for _ in range(6):
            self.clock.advance(100_000_000)
            self.node.on_odometry(msg)
        self.assertEqual(len(self.pub.messages), 1)
        self.assertTrue(self.node.gate.fault)

    def test_stale_future_zero_stamp_and_bad_pose_not_published(self):
        for stamp in (self.clock.ros-NS, self.clock.ros+NS, 0):
            self.node.on_odometry(odometry(stamp))
        msg = odometry(self.clock.ros)
        msg.pose.pose.position.x = float("nan")
        self.node.on_odometry(msg)
        self.assertEqual(self.pub.messages, [])
        self.assertEqual(self.node.gate.rejected, 4)

    def test_backward_source_stamp_stops(self):
        self.feed(seq=3)
        self.clock.advance(100_000_000)
        self.node.on_odometry(odometry(self.clock.ros-150_000_000, seq=4))
        self.assertEqual(self.node.gate.fault, "SOURCE_STAMP_ROLLBACK")

    def test_wrong_frame_is_rejected(self):
        self.feed(parent="map")
        self.assertEqual(self.pub.messages, [])
        self.assertEqual(self.node.gate.last_reject, "FRAME_MISMATCH")

    def test_wrong_source_and_latching_fail_closed(self):
        self.feed(source="/unexpected")
        self.assertEqual(self.pub.messages, [])
        self.assertEqual(self.node.gate.fault, "UNEXPECTED_SOURCE")
        self.node.gate = gm.FreshnessGate()
        msg = odometry(self.clock.ros)
        msg._connection_header["latching"] = "1"
        self.node.on_odometry(msg)
        self.assertEqual(self.node.gate.fault, "LATCHED_INPUT_REFUSED")

    def test_clock_jump_stops(self):
        self.feed()
        self.clock.ros += NS
        self.node.tick()
        self.assertEqual(self.node.gate.fault, "CLOCK_JUMP")

    def test_pose_publication_failure_stops_without_retry(self):
        self.pub.fail = True
        self.feed()
        self.pub.fail = False
        self.clock.advance(100_000_000)
        self.feed(seq=2)
        self.assertEqual(self.pub.messages, [])
        self.assertEqual(self.node.gate.fault, "CALLBACK_ERROR")

    def test_shutdown_and_late_callbacks_cannot_publish(self):
        self.feed()
        self.node.close()
        self.clock.advance(100_000_000)
        self.feed(seq=2)
        self.node.tick()
        self.assertEqual(len(self.pub.messages), 1)
        self.assertTrue(all(p.closed for p in self.ros.publishers.values()))
        self.assertTrue(all(s.closed for s in self.ros.subscribers))

    def test_registered_odom_is_not_actual_odom(self):
        # Graph registration is intentionally not evidence in status computation.
        s = self.status()
        self.assertFalse(s["local_odom_received"])
        self.assertFalse(s["horizontal_flags_valid"])
        self.assertIsNone(s["mode"])

    def test_fresh_flags_plus_actual_odom_is_still_not_flight_authorization(self):
        self.node.on_telemetry("state", "/mavros", telemetry(
            self.clock, connected=True, armed=False, mode="MANUAL"))
        self.node.on_telemetry("estimator", "/mavros", telemetry(
            self.clock, velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=False, const_pos_mode_status_flag=False))
        odom = odometry(self.clock.ros, parent="map", child="base_link", source="/mavros")
        self.node.on_telemetry("local_odom", "/mavros", odom)
        s = self.status()
        self.assertTrue(s["disarmed_manual_observed"])
        self.assertTrue(s["horizontal_flags_valid"])
        self.assertTrue(s["local_odom_received"])
        self.assertFalse(s["flight_authorized"])
        self.clock.advance(600_000_000)
        self.assertFalse(self.status()["local_odom_received"])

    def test_state_must_be_new_fresh_and_disarmed_manual(self):
        msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertTrue(self.status()["disarmed_manual_observed"])
        self.clock.advance(100_000_000)
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertFalse(self.status()["disarmed_manual_observed"])
        self.node.on_telemetry("state", "/mavros", telemetry(
            self.clock, connected=True, armed=True, mode="MANUAL"))
        self.assertFalse(self.status()["disarmed_manual_observed"])

    def test_wrong_telemetry_source_or_latching_value_not_trusted(self):
        for c in ({"callerid": "/other", "latching": "0"},
                  {"callerid": "/other", "latching": "1"},
                  {"callerid": "/mavros", "latching": "2"}):
            msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
            msg._connection_header = c
            self.node.on_telemetry("state", "/mavros", msg)
            self.assertFalse(self.status()["disarmed_manual_observed"])

    def test_latched_state_live_heartbeats_are_displayed_not_flight_authorization(self):
        # MAVROS 1.20.0 advertises State with latch=true, including live heartbeats.
        callback = next(s.callback for s in self.ros.subscribers if s.topic == app.STATE)
        for _ in range(3):
            msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
            msg._connection_header["latching"] = "1"
            callback(msg)
            s = self.status()
            self.assertTrue(s["connected"])
            self.assertFalse(s["armed"])
            self.assertEqual(s["mode"], "MANUAL")
            self.assertTrue(s["disarmed_manual_observed"])
            self.assertTrue(s["diagnostic_only"])
            self.assertFalse(s["flight_authorized"])
            self.assertEqual(self.pub.messages, [])
            self.clock.advance(NS)

    def test_latched_state_expires_without_new_heartbeats(self):
        msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
        msg._connection_header["latching"] = "1"
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertTrue(self.status()["disarmed_manual_observed"])
        self.clock.advance(2*NS+1)
        s = self.status()
        for key in ("connected", "armed", "mode"):
            self.assertIsNone(s[key])
        self.assertFalse(s["disarmed_manual_observed"])

    def test_latched_state_expires_even_if_ros_time_is_frozen(self):
        msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
        msg._connection_header["latching"] = "1"
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertTrue(self.status()["disarmed_manual_observed"])
        self.clock.wall += 2*NS+1
        self.assertIsNone(self.status()["connected"])

    def test_stale_future_and_zero_latched_state_not_trusted(self):
        for stamp in (self.clock.ros-2*NS-1,
                      self.clock.ros+gm.MAX_FUTURE_NS+1, 0):
            with self.subTest(stamp=stamp):
                msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
                msg._connection_header["latching"] = "1"
                msg.header.stamp = Stamp(stamp)
                self.node.on_telemetry("state", "/mavros", msg)
                self.assertIsNone(self.status()["connected"])

    def test_duplicate_or_backward_latched_state_cannot_keep_status_alive(self):
        msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
        msg._connection_header["latching"] = "1"
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertTrue(self.status()["disarmed_manual_observed"])
        accepted = msg.header.stamp.to_nsec()
        self.clock.advance(NS)
        for stamp in (accepted, accepted-1):
            msg.header.stamp = Stamp(stamp)
            self.node.on_telemetry("state", "/mavros", msg)
            self.assertIsNone(self.status()["connected"])
        msg.header.stamp = Stamp(self.clock.ros)
        self.node.on_telemetry("state", "/mavros", msg)
        self.assertTrue(self.status()["disarmed_manual_observed"])

    def test_latched_state_does_not_hide_unsafe_vehicle_values(self):
        for connected, armed, mode in ((True, True, "MANUAL"),
                                       (True, False, "OFFBOARD"),
                                       (False, False, "MANUAL")):
            self.clock.advance(NS)
            msg = telemetry(self.clock, connected=connected, armed=armed, mode=mode)
            msg._connection_header["latching"] = "1"
            self.node.on_telemetry("state", "/mavros", msg)
            s = self.status()
            self.assertEqual((s["connected"], s["armed"], s["mode"]),
                             (connected, armed, mode))
            self.assertFalse(s["disarmed_manual_observed"])
            self.assertFalse(s["flight_authorized"])

    def test_latched_non_state_telemetry_still_refused(self):
        for key, source in (("estimator", "/mavros"), ("local_odom", "/mavros"),
                            ("observation", "/window_pass_observer")):
            msg = telemetry(self.clock)
            msg._connection_header = {"callerid": source, "latching": "1"}
            self.node.on_telemetry(key, source, msg)
            self.assertNotIn(key, self.node.telemetry)

    def test_state_latching_exception_is_specific_to_mavros_source(self):
        msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
        msg._connection_header = {"callerid": "/other", "latching": "1"}
        self.node.on_telemetry("state", "/other", msg)
        self.assertIsNone(self.status()["connected"])

    def test_infinite_local_pose_is_not_valid_odom(self):
        msg = odometry(self.clock.ros, source="/mavros")
        msg.pose.pose.position.x = float("inf")
        self.node.on_telemetry("local_odom", "/mavros", msg)
        self.assertFalse(self.status()["local_odom_received"])

    def test_console_is_change_only_status_does_not_spam_poses(self):
        self.node.tick()
        for _ in range(4):
            self.clock.advance(100_000_000)
            self.node.tick()
        self.assertEqual(len(self.ros.logs), 1)
        self.assertEqual(self.pub.messages, [])


class EntryTests(unittest.TestCase):
    def test_remaps_real_switch_and_output_conflicts_fail_before_advertise(self):
        cases = [({}, {app.OUTPUT: ["/old_guard"]}, {}),
                 ({}, {app.STATUS: ["/duplicate"]}, {}),
                 ({}, {app.INPUT: ["/wrong"]}, {}),
                 ({}, {}, {"/use_sim_time": True}),
                 ({}, {}, {"~real_output": True})]
        for topic in (app.INPUT, app.OUTPUT, app.STATUS, app.STATE,
                      app.ESTIMATOR, app.LOCAL_ODOM, app.OBSERVATION):
            cases.append(({topic: "/mavros/vision_pose/pose"}, {}, {}))
        for remaps, publishers, params in cases:
            with self.subTest(remaps=remaps, publishers=publishers, params=params):
                clock = Clock()
                ros = Ros(clock, remaps=remaps, params=params)
                with self.assertRaises(ValueError):
                    app.GroundBridge(ros, Graph(publishers), TYPES, gm,
                                     app.FrameTransform(EMPTY), lambda: clock.wall)
                self.assertEqual(ros.publishers, {})

    def test_default_check_runs_without_ros_and_reports_configured_preview(self):
        result = subprocess.run([sys.executable, str(SCRIPT), "--check"],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertFalse(data["ROS_started"])
        self.assertFalse(data["real_MAVROS_output"])
        self.assertTrue(data["transform_configured"])
        self.assertEqual(data["stop_after_s"], .5)
        self.assertEqual(data["installation_missing"], [])

    def test_no_service_client_tf_lookup_or_setpoint_publisher(self):
        text = SCRIPT.read_text(encoding="utf-8")
        for forbidden in ("ServiceProxy(", "lookupTransform(", "lookup_transform(",
                          "real_output = True"):
            self.assertNotIn(forbidden, text)
        tree = ast.parse(text, feature_version=(3, 8))
        self.assertIsNotNone(tree)
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Attribute) and node.func.attr == "Publisher"]
        self.assertEqual({node.args[0].id for node in calls}, {"OUTPUT", "STATUS", "REAL_OUTPUT"})

    def test_launch_only_ground_node_and_optional_observer(self):
        launch = ET.parse(ROOT/"launch/competition_ground.launch").getroot()
        self.assertEqual(launch.findall("include"), [])
        types = [n.get("type") for n in launch.findall("node")]
        self.assertEqual(types, ["competition_ground.py", "window_observer.py"])
        args = {a.get("name"): a.get("default") for a in launch.findall("arg")}
        self.assertEqual(args["start_observer"], "false")
        self.assertNotIn("enable_motion", args)

    def test_default_config_contains_ground_preview_approximation(self):
        cfg = json.loads((ROOT/"config/competition_ground.json").read_text())
        self.assertEqual(cfg, GROUND_APPROX)

    def test_existing_gate_copy_is_byte_identical_when_local_baseline_available(self):
        baseline = ROOT.parent/"_deploy/fast_lio_pose_guard_diagnostic_20261009_v1.py"
        if baseline.exists():
            self.assertEqual((ROOT/"competition/pose_guard_v1.py").read_bytes(), baseline.read_bytes())
        self.assertEqual(gm.MAX_AGE_NS, 200_000_000)
        self.assertEqual(gm.SILENCE_NS, 500_000_000)


if __name__ == "__main__":
    unittest.main(verbosity=2)
