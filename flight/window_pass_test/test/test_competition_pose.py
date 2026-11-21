"""Continuous-pose policy tests: standard library/Fake ROS, no hardware."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import unittest

from test_competition_ground import (
    gm, Clock, TYPES, EMPTY, GROUND_APPROX, Stamp, NS, odometry, telemetry)
from test_competition_real_ground import LiveGraph, LiveRos

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/"scripts/competition_pose.py"
spec = importlib.util.spec_from_file_location("competition_pose_test", SCRIPT)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)
ground = app.ground


class PreflightTests(unittest.TestCase):
    def construct(self, modify=None, cfg=GROUND_APPROX, flag=True):
        clock, graph = Clock(), LiveGraph()
        ros = LiveRos(clock, graph)
        if modify:
            modify(clock, graph, ros)
        node = app.ContinuousPoseBridge(ros, graph, TYPES, gm,
                                       ground.FrameTransform(cfg), lambda: clock.wall,
                                       continuous_pose=flag)
        self.addCleanup(node.close)
        return node, clock, graph, ros

    def test_real_publisher_not_advertised_before_safe_start(self):
        node, clock, graph, ros = self.construct()
        self.assertNotIn(ground.REAL_OUTPUT, ros.publishers)
        self.assertFalse(node.snapshot(clock.ros, clock.wall)["real_output_active"])

    def test_unconfigured_transform_and_nonboolean_intent_refused(self):
        with self.assertRaises(ValueError):
            self.construct(cfg=EMPTY)
        for flag in (False, None, 1, "true", "false"):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                self.construct(flag=flag)

    def test_ros_parameters_cannot_enable_output(self):
        for name in ("~continuous_pose", "~real_ground", "~real_output"):
            for val in (True, "false", 1):
                with self.subTest(name=name, val=val), self.assertRaises(ValueError):
                    self.construct(lambda c, g, r: r.params.update({name: val}))

    def test_source_state_and_mavros_subscriber_required_at_startup(self):
        for topic, nodes in ((ground.INPUT, []), (ground.INPUT, [gm.SOURCE_NODE, "/other"]),
                             (ground.STATE, []), (ground.STATE, ["/other"])):
            with self.subTest(topic=topic, nodes=nodes), self.assertRaises(ValueError):
                self.construct(lambda c, g, r: g.pubs.update({topic: nodes}))
        with self.assertRaises(ValueError):
            self.construct(lambda c, g, r: g.subs.update({ground.REAL_OUTPUT: []}))

    def test_busy_outputs_and_competing_estimates_refused(self):
        for topic in (ground.OUTPUT, ground.STATUS, ground.REAL_OUTPUT) + ground.OTHER_ESTIMATE_INPUTS:
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                self.construct(lambda c, g, r: g.pubs.update({topic: ["/other"]}))

    def test_independent_control_publishers_and_their_tf_params_are_not_pose_conflicts(self):
        def control(c, g, r):
            g.pubs.update({"/mavros/setpoint_raw/local": ["/controller"],
                           "/mavros/manual_control/send": ["/joystick"]})
            r.params.update({"/mavros/setpoint_position/tf/listen": True,
                             "/mavros/setpoint_attitude/tf/listen": True})
        self.construct(control)

    def test_sim_time_vision_tf_listen_and_remapping_still_refused(self):
        for param in ("/use_sim_time", "/mavros/vision_pose/tf/listen"):
            for val in (True, "false", 1):
                with self.subTest(param=param, val=val), self.assertRaises(ValueError):
                    self.construct(lambda c, g, r: r.params.update({param: val}))
        with self.assertRaises(ValueError):
            self.construct(lambda c, g, r: r.remaps.update({ground.REAL_OUTPUT: "/other"}))

    def test_unavailable_or_slow_master_refuses_initial_start(self):
        for change in (lambda c, g, r: setattr(g, "fail", True),
                       lambda c, g, r: setattr(g, "hook", lambda: c.advance(NS))):
            with self.assertRaises(ValueError):
                self.construct(change)

    def test_default_and_check_do_not_import_or_start_ros(self):
        for args in ((), ("--check",)):
            result = subprocess.run([sys.executable, str(SCRIPT)] + list(args),
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(result.stdout)
            self.assertFalse(data["ROS_started"])
            self.assertFalse(data["real_MAVROS_output"])
            self.assertFalse(data["auto_arm"])
            self.assertFalse(data["auto_mode_switch"])
            self.assertFalse(data["vehicle_control"])

    def test_cli_requires_both_flags_and_rejects_remaps_before_ros_import(self):
        for args in (("--run",), ("--continuous-pose",), ("--check", "--continuous-pose"),
                     ("--run", "--continuous-pose", "__name:=mavros"),
                     ("--run", "--continuous-pose", "_continuous_pose:=true"),
                     ("--run", "--real-ground")):
            result = subprocess.run([sys.executable, str(SCRIPT)] + list(args),
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertNotIn("ModuleNotFoundError", result.stderr)


class ContinuousTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.graph = Clock(), LiveGraph()
        self.ros = LiveRos(self.clock, self.graph)
        self.node = app.ContinuousPoseBridge(
            self.ros, self.graph, TYPES, gm, ground.FrameTransform(GROUND_APPROX),
            lambda: self.clock.wall, continuous_pose=True)
        self.addCleanup(self.node.close)
        self.seq = 0

    def state(self, **kw):
        fields = dict(connected=True, armed=False, mode="MANUAL")
        fields.update(kw)
        msg = telemetry(self.clock, **fields)
        msg._connection_header["latching"] = "1"
        self.node.on_telemetry("state", "/mavros", msg)
        return msg

    def feed(self, **kw):
        self.seq += 1
        msg = odometry(self.clock.ros-20_000_000, self.seq, **kw)
        self.node.on_odometry(msg)
        return msg

    def start(self):
        self.state()
        msg = self.feed()
        self.assertEqual(self.node.real_published, 1)
        return msg, self.ros.publishers[ground.REAL_OUTPUT]

    def step(self, check_graph=True, **kw):
        self.clock.advance(100_000_000)
        if check_graph:
            self.node.check_runtime_graph()
        return self.feed(**kw)

    def status(self):
        return self.node.snapshot(self.clock.ros, self.clock.wall)

    def assertStopped(self, part):
        self.assertIn(part, self.node.real_fault)
        self.assertIsNone(self.node.real_pub)
        self.assertFalse(self.status()["real_output_active"])

    def test_same_transform_source_stamp_queue_and_no_extra_mavros_publishers(self):
        msg, pub = self.start()
        out = pub.messages[-1]
        self.assertEqual(out.header.stamp.to_nsec(), msg.header.stamp.to_nsec())
        self.assertEqual(out.header.frame_id, "competition_world")
        self.assertAlmostEqual(out.pose.position.z, .36)
        self.assertEqual(out.pose, self.ros.publishers[ground.OUTPUT].messages[-1].pose)
        self.assertEqual(out.header.seq, msg.header.seq)
        self.assertEqual(pub.options, dict(queue_size=1, latch=False))
        self.assertEqual(set(self.ros.publishers), {ground.OUTPUT, ground.STATUS, ground.REAL_OUTPUT})

    def test_status_reports_continuous_not_ground_or_flight_authorized(self):
        self.start()
        data = self.status()
        self.assertEqual(data["real_output_state"], "FORWARDING_CONTINUOUS_POSE")
        self.assertEqual(data["state"], "INPUT_VALID")
        self.assertTrue(data["startup_disarmed_manual_confirmed"])
        self.assertTrue(data["continuous_pose_requested"])
        for name in ("flight_authorized", "diagnostic_only", "real_ground_requested", "controls_vehicle"):
            self.assertFalse(data[name], name)

    def test_arming_all_modes_and_later_disarming_continue_same_publisher(self):
        _, pub = self.start()
        for armed, mode in ((False, "POSCTL"), (True, "POSCTL"), (True, "OFFBOARD"),
                            (True, "ALTCTL"), (True, "AUTO.LAND"), (False, "MANUAL")):
            self.clock.advance(100_000_000)
            self.state(armed=armed, mode=mode)
            self.node.check_runtime_graph()
            self.feed()
            self.assertIs(self.node.real_pub, pub)
            self.assertEqual(self.node.real_fault, "")
        self.assertEqual(len(pub.messages), 7)

    def test_first_output_refused_if_already_armed_or_not_manual(self):
        for fields, part in (({"armed": True}, "STARTUP_ARMED"),
                             ({"mode": "POSCTL"}, "STARTUP_MODE")):
            self.setUp()
            self.state(**fields)
            self.feed()
            self.assertStopped(part)
            self.assertNotIn(ground.REAL_OUTPUT, self.ros.publishers)

    def test_no_state_or_disconnected_at_start_waits_without_real_output(self):
        self.feed()
        self.assertEqual(self.node.real_published, 0)
        self.clock.advance(100_000_000)
        self.state(connected=False)
        self.feed()
        self.assertEqual(self.node.real_published, 0)
        self.assertNotIn(ground.REAL_OUTPUT, self.ros.publishers)
        self.clock.advance(100_000_000)
        self.state()
        self.feed()
        self.assertEqual(self.node.real_published, 1)

    def test_untrusted_or_stale_state_cannot_authorize_start(self):
        for alter in (lambda m: m._connection_header.update(callerid="/other"),
                      lambda m: setattr(m.header, "stamp", Stamp(self.clock.ros-3*NS)),
                      lambda m: setattr(m, "armed", "false")):
            self.setUp()
            msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
            alter(msg)
            self.node.on_telemetry("state", "/mavros", msg)
            self.feed()
            self.assertEqual(self.node.real_published, 0)

    def test_delayed_initial_handshake_does_not_allow_armed_start(self):
        def pending(topic, pub):
            if topic == ground.REAL_OUTPUT:
                pub.connections = 0
        self.ros.on_advertise = pending
        self.state()
        self.feed()
        self.assertEqual(self.node.real_published, 0)
        self.clock.advance(100_000_000)
        self.state(armed=True)
        self.feed()
        self.assertStopped("STARTUP_ARMED")

    def test_connection_state_loss_after_start_does_not_cut_healthy_pose(self):
        _, pub = self.start()
        self.clock.advance(100_000_000)
        self.state(connected=False, armed=True, mode="OFFBOARD")
        self.feed()
        self.assertEqual(self.status()["telemetry_warning"], "MAVROS_NOT_CONNECTED")
        for _ in range(25):
            self.step()
        self.assertEqual(self.node.real_fault, "")
        self.assertEqual(self.status()["telemetry_warning"], "MAVROS_STATE_NOT_FRESH")
        self.assertEqual(len(pub.messages), 27)

    def test_bad_state_after_start_is_telemetry_warning_not_pose_stop(self):
        self.start()
        self.clock.advance(100_000_000)
        msg = telemetry(self.clock, connected=True, armed="false", mode="MANUAL")
        self.node.on_telemetry("state", "/mavros", msg)
        self.feed()
        self.assertEqual(self.node.real_published, 2)
        self.assertEqual(self.status()["telemetry_warning"], "MAVROS_STATE_NOT_FRESH")

    def test_control_publishers_can_appear_while_localization_continues(self):
        self.start()
        self.graph.pubs["/mavros/setpoint_raw/local"] = ["/controller"]
        self.graph.pubs["/mavros/setpoint_position/local"] = ["/controller"]
        self.step()
        self.assertEqual(self.node.real_published, 2)
        self.assertEqual(self.node.real_fault, "")

    def test_master_exception_and_recovery_after_start_do_not_cut_pose(self):
        _, pub = self.start()
        self.graph.fail = True
        for _ in range(12):
            self.step()
            self.node.tick()
        self.assertEqual(self.node.real_fault, "")
        self.assertIn("GRAPH_QUERY_FAILED", self.status()["graph_monitor_warning"])
        self.assertEqual(len(pub.messages), 13)
        self.graph.fail = False
        self.step()
        self.assertEqual(self.status()["graph_monitor_warning"], "")

    def test_stale_graph_cache_is_only_warning_after_start(self):
        self.start()
        for _ in range(12):
            self.step(check_graph=False)
            self.node.tick()
        self.assertEqual(self.node.real_published, 13)
        self.assertEqual(self.node.real_fault, "")
        self.assertFalse(self.status()["graph_monitor_fresh"])
        self.assertEqual(self.status()["graph_monitor_warning"], "GRAPH_CHECK_STALE")

    def test_graph_failure_before_first_send_still_latches_stop(self):
        self.graph.fail = True
        self.node.check_runtime_graph()
        self.state()
        self.feed()
        self.assertStopped("GRAPH_QUERY_FAILED")
        self.assertEqual(self.node.real_published, 0)

    def test_slow_graph_worker_does_not_hold_pose_callback_lock(self):
        _, pub = self.start()
        entered, release = threading.Event(), threading.Event()
        def blocked():
            entered.set()
            release.wait(3)
        self.graph.hook = blocked
        worker = threading.Thread(target=self.node.check_runtime_graph, daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            for _ in range(8):
                self.step(check_graph=False)
                self.node.tick()
            self.assertEqual(len(pub.messages), 9)
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(self.node.real_fault, "")
        self.assertEqual(self.status()["graph_monitor_warning"], "GRAPH_QUERY_TOO_SLOW")

    def test_known_graph_conflicts_still_stop_even_with_fresh_input(self):
        changes = (
            lambda g, r: g.pubs[ground.REAL_OUTPUT].append("/other"),
            lambda g, r: g.pubs.update({ground.INPUT: [gm.SOURCE_NODE, "/other"]}),
            lambda g, r: g.pubs.update({ground.INPUT: []}),
            lambda g, r: g.pubs.update({"/mavros/odometry/out": ["/other"]}),
            lambda g, r: g.pubs.update({"/mavros/vision_pose/pose_cov": ["/other"]}),
            lambda g, r: r.params.update({"/mavros/vision_pose/tf/listen": True}),
            lambda g, r: r.params.update({"/use_sim_time": True}))
        for change in changes:
            self.setUp()
            self.start()
            change(self.graph, self.ros)
            self.node.check_runtime_graph()
            self.assertTrue(self.node.real_fault)
            self.assertFalse(self.status()["real_output_active"])

    def test_known_conflict_does_not_recover_just_because_it_disappears(self):
        self.start()
        self.graph.pubs["/mavros/odometry/out"] = ["/other"]
        self.node.check_runtime_graph()
        self.graph.pubs["/mavros/odometry/out"] = []
        self.step()
        self.assertStopped("OTHER_ESTIMATE_PUBLISHER")
        self.assertEqual(self.node.real_published, 1)

    def test_source_silence_half_second_stops_and_never_replays_or_restarts(self):
        _, pub = self.start()
        for _ in range(5):
            self.clock.advance(100_000_000)
            self.node.check_runtime_graph()
            self.node.tick()
        self.assertStopped("INPUT_STALE")
        self.feed()
        self.assertEqual(len(pub.messages), 1)

    def test_clock_jump_or_rollback_stops(self):
        for delta in (-1, NS):
            self.setUp()
            self.start()
            self.clock.ros += delta
            self.node.tick()
            self.assertStopped("CLOCK")

    def test_source_stamp_and_sequence_reset_stops(self):
        for kind in ("stamp", "sequence"):
            self.setUp()
            self.start()
            self.clock.advance(100_000_000)
            msg = odometry(self.clock.ros-20_000_000, 2)
            if kind == "stamp":
                msg.header.stamp = Stamp(9*NS)
            else:
                msg.header.seq = 0
            self.node.on_odometry(msg)
            self.assertStopped("SOURCE_")

    def test_duplicate_frames_reject_without_republishing(self):
        msg, pub = self.start()
        self.node.on_odometry(msg)
        self.assertEqual(len(pub.messages), 1)
        self.assertEqual(self.node.gate.last_reject, "DUPLICATE_STAMP")

    def test_invalid_poses_old_future_and_frame_mismatch_never_sent(self):
        self.state()
        messages = [odometry(self.clock.ros, parent="wrong"), odometry(self.clock.ros-NS),
                    odometry(self.clock.ros+NS), odometry(0)]
        for field in ("position", "orientation"):
            bad = odometry(self.clock.ros)
            getattr(bad.pose.pose, field).x = float("nan")
            messages.append(bad)
        for msg in messages:
            self.node.on_odometry(msg)
        self.assertEqual(self.node.real_published, 0)

    def test_wrong_source_or_latched_lio_stops_immediately(self):
        for key, value in (("callerid", "/other"), ("latching", "1")):
            self.setUp()
            self.start()
            self.clock.advance(100_000_000)
            msg = odometry(self.clock.ros, 2)
            msg._connection_header[key] = value
            self.node.on_odometry(msg)
            self.assertTrue(self.node.real_fault)
            self.assertEqual(self.node.real_published, 1)

    def test_observer_and_state_graph_loss_after_start_is_not_a_source_reset(self):
        self.start()
        self.graph.pubs[ground.STATE] = []
        self.graph.subs[ground.REAL_OUTPUT] = []
        self.step()
        self.assertEqual(self.node.real_fault, "")
        self.assertEqual(self.node.real_published, 2)  # Local queue count, NOT an FCU ACK.

    def test_transport_reconnect_waits_for_new_pose_no_cached_replay(self):
        _, pub = self.start()
        pub.connections = 0
        self.step()
        self.step()
        self.assertEqual(len(pub.messages), 1)
        pub.connections = 1
        self.node.tick()
        self.assertEqual(len(pub.messages), 1)
        latest = self.step()
        self.assertEqual(len(pub.messages), 2)
        self.assertEqual(pub.messages[-1].header.stamp.to_nsec(), latest.header.stamp.to_nsec())

    def test_publication_delay_does_not_restamp_or_send_expired_first_pose(self):
        def slow(topic, pub):
            if topic == ground.REAL_OUTPUT:
                self.clock.advance(250_000_000)
        self.ros.on_advertise = slow
        self.state()
        self.feed()
        self.assertEqual(self.node.real_published, 0)
        self.step()
        self.assertEqual(self.node.real_published, 1)

    def test_publication_failure_latches_stop(self):
        _, pub = self.start()
        pub.fail = True
        self.step()
        self.assertStopped("REAL_PUBLISH_FAILED")
        pub.fail = False
        self.step()
        self.assertEqual(len(pub.messages), 1)

    def test_rate_uses_only_distinct_source_publications(self):
        self.start()
        for _ in range(12):
            self.step()
            self.node.tick()
        self.assertEqual(self.status()["real_output_rate_hz"], 10.0)
        before = self.node.real_published
        for _ in range(10):
            self.node.tick()
        self.assertEqual(self.node.real_published, before)
        self.clock.advance(3*NS)
        self.assertIsNone(self.status()["real_output_rate_hz"])

    def test_callbacks_and_status_do_not_make_master_rpcs(self):
        self.start()
        calls = self.graph.calls
        self.clock.advance(100_000_000)
        self.state(armed=True, mode="OFFBOARD")
        self.feed()
        self.node.tick()
        self.status()
        self.assertEqual(self.graph.calls, calls)

    def test_close_shutdown_and_late_callbacks_never_publish_or_reconnect(self):
        _, pub = self.start()
        self.node.close()
        self.step()
        self.state(armed=True, mode="OFFBOARD")
        self.node.tick()
        self.assertTrue(pub.closed)
        self.assertEqual(len(pub.messages), 1)
        self.assertFalse(self.status()["real_output_active"])

    def test_ros_shutdown_flag_prevents_publish(self):
        _, pub = self.start()
        self.ros.shutdown = True
        self.step()
        self.node.tick()
        self.assertEqual(len(pub.messages), 1)
        self.assertFalse(self.status()["real_output_active"])


class PackagingTests(unittest.TestCase):
    def test_frozen_ground_gate_and_transform_config_were_not_changed(self):
        hashes = {
            "scripts/competition_ground.py": "a71bf8f4389ca8483aabda7416827f516c80e677e7e71084312c59b28aaf6cf7",
            "competition/pose_guard_v1.py": "15dabf5e06f398f7c5756a259aed73f452d488bada6a53d0fa3dff8fe3195417",
            "config/competition_ground.json": "5401e6fa77f18a490e52fa716603c9b31493a1362d42b04f1dbc467a53cbd9d0"}
        for name, digest in hashes.items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(), digest, name)

    def test_no_control_clients_no_shell_commands_and_python38_syntax(self):
        text = SCRIPT.read_text(encoding="utf-8")
        tree = ast.parse(text, feature_version=(3, 8))
        for forbidden in ("ServiceProxy(", "set_param(", "subprocess.", "os.system(",
                          "CommandBool(", "SetMode(", "PositionTarget("):
            self.assertNotIn(forbidden, text)
        self.assertFalse(any(isinstance(n, ast.Import) and
                             any(a.name == "rospy" for a in n.names) for n in tree.body))


if __name__ == "__main__":
    unittest.main(verbosity=2)
