"""Ground real-output adapter tests. Fake ROS only; never connects to hardware."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import threading
import unittest

from test_competition_ground import (
    app, gm, Clock, Ros, TYPES, EMPTY, GROUND_APPROX, Stamp, NS,
    odometry, telemetry, SCRIPT)


class LiveGraph:
    def __init__(self):
        self.pubs = {app.INPUT: [gm.SOURCE_NODE], app.STATE: ["/mavros"]}
        self.subs = {app.REAL_OUTPUT: ["/mavros"]}
        self.calls, self.fail, self.hook = 0, False, None

    def getSystemState(self):
        self.calls += 1
        if self.hook:
            self.hook()
        if self.fail:
            raise RuntimeError("master unavailable")
        return copy.deepcopy((list(self.pubs.items()), list(self.subs.items()), []))


class LiveRos(Ros):
    def __init__(self, clock, graph, **kw):
        super().__init__(clock, **kw)
        self.graph, self.on_advertise = graph, None

    def Publisher(self, topic, typ, **kw):
        pub = super().Publisher(topic, typ, **kw)
        nodes = self.graph.pubs.setdefault(topic, [])
        if self.get_name() not in nodes:
            nodes.append(self.get_name())
        unregister = pub.unregister

        def remove():
            unregister()
            self.graph.pubs[topic] = [n for n in self.graph.pubs.get(topic, [])
                                      if n != self.get_name()]
        pub.unregister = remove
        if self.on_advertise:
            self.on_advertise(topic, pub)
        return pub


class PreflightTests(unittest.TestCase):
    def construct(self, modify=None, cfg=None, **kwargs):
        clock, graph = Clock(), LiveGraph()
        ros = LiveRos(clock, graph)
        if modify:
            modify(clock, graph, ros)
        node = app.GroundBridge(ros, graph, TYPES, gm,
                               app.FrameTransform(GROUND_APPROX if cfg is None else cfg),
                               lambda: clock.wall, real_ground=kwargs.get("flag", True))
        self.addCleanup(node.close)
        return node, clock, graph, ros

    def rejected(self, modify=None, cfg=None, **kwargs):
        with self.assertRaises(ValueError):
            self.construct(modify, cfg, **kwargs)

    def test_no_real_publisher_before_state_and_valid_source(self):
        node, clock, graph, ros = self.construct()
        self.assertNotIn(app.REAL_OUTPUT, ros.publishers)
        self.assertFalse(node.snapshot(clock.ros, clock.wall)["real_output_active"])

    def test_missing_transform_and_nonboolean_opt_in_refused(self):
        self.rejected(cfg=EMPTY)
        for flag in ("false", "true", 1, None):
            self.rejected(flag=flag)

    def test_ros_parameter_cannot_enable_real_mode(self):
        for name in ("~real_ground", "~real_output"):
            for value in (True, "false", 1):
                self.rejected(lambda c, g, r: r.params.update({name: value}), flag=False)

    def test_missing_or_multiple_source_and_state_publishers_refused(self):
        for topic, nodes in ((app.INPUT, []), (app.INPUT, [gm.SOURCE_NODE, "/other"]),
                             (app.STATE, []), (app.STATE, ["/other"]),
                             (app.STATE, ["/mavros", "/other"])):
            self.rejected(lambda c, g, r: g.pubs.update({topic: nodes}))

    def test_busy_outputs_and_other_estimates_refused(self):
        for topic in (app.OUTPUT, app.STATUS, app.REAL_OUTPUT) + app.OTHER_ESTIMATE_INPUTS:
            with self.subTest(topic=topic):
                self.rejected(lambda c, g, r: g.pubs.update({topic: ["/other"]}))

    def test_all_command_topic_families_blocked(self):
        topics = ("/mavros/setpoint_raw/local", "/mavros/setpoint_position/local",
                  "/mavros/setpoint_velocity/cmd_vel", "/mavros/setpoint_attitude/thrust",
                  "/mavros/setpoint_trajectory/local", "/mavros/setpoint_unknown/input")
        for topic in topics + tuple(app.OTHER_CONTROL_INPUTS):
            for source in ("/other", "/mavros"):
                self.rejected(lambda c, g, r: g.pubs.update({topic: [source]}))

    def test_normal_mavros_feedback_and_odometry_in_are_not_commands(self):
        def feedback(c, g, r):
            for topic in app.SETPOINT_FEEDBACK | {"/mavros/odometry/in"}:
                g.pubs[topic] = ["/mavros"]
        self.construct(feedback)

    def test_feedback_exception_does_not_allow_a_second_publisher(self):
        for topic in app.SETPOINT_FEEDBACK:
            self.rejected(lambda c, g, r: g.pubs.update({topic: ["/mavros", "/other"]}))

    def test_vision_subscriber_must_be_mavros(self):
        for subs in ([], ["/echo"]):
            self.rejected(lambda c, g, r: g.subs.update({app.REAL_OUTPUT: subs}))

    def test_tf_listening_sim_time_and_remaps_refused(self):
        for param in app.REAL_FALSE_PARAMS:
            for value in (True, "false"):
                self.rejected(lambda c, g, r: r.params.update({param: value}))
        self.rejected(lambda c, g, r: r.remaps.update({app.REAL_OUTPUT: "/different"}))

    def test_master_error_and_slow_preflight_fail_before_advertising(self):
        self.rejected(lambda c, g, r: setattr(g, "fail", True))
        self.rejected(lambda c, g, r: setattr(g, "hook", lambda: c.advance(NS)))

    def test_cli_requires_run_and_forbids_remapping_before_ros_import(self):
        for args in (("--real-ground",), ("--check", "--real-ground"),
                     ("--run", "--real-ground", "__name:=mavros"),
                     ("--run", "--real-ground", "_real_output:=true")):
            result = subprocess.run([sys.executable, str(SCRIPT)] + list(args),
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertNotIn("ModuleNotFoundError", result.stderr)


class RealBridgeTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.graph = Clock(), LiveGraph()
        self.ros = LiveRos(self.clock, self.graph)
        self.node = app.GroundBridge(self.ros, self.graph, TYPES, gm,
                                    app.FrameTransform(GROUND_APPROX), lambda: self.clock.wall,
                                    real_ground=True)
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
        return msg, self.ros.publishers[app.REAL_OUTPUT]

    def status(self):
        return self.node.snapshot(self.clock.ros, self.clock.wall)

    def assertStopped(self, reason=None):
        self.assertTrue(self.node.real_fault)
        if reason:
            self.assertIn(reason, self.node.real_fault)
        self.assertIsNone(self.node.real_pub)
        self.assertFalse(self.status()["real_output_active"])
        self.assertFalse(self.status()["flight_authorized"])

    def test_only_new_fresh_inputs_publish_identical_preview_and_real_pose(self):
        msg, pub = self.start()
        out = pub.messages[-1]
        preview = self.ros.publishers[app.OUTPUT].messages[-1]
        self.assertEqual(out.header.stamp.to_nsec(), msg.header.stamp.to_nsec())
        self.assertEqual(out.header.frame_id, "competition_world")
        self.assertEqual(out.pose, preview.pose)
        self.assertEqual((out.pose.position.x, out.pose.position.y), (.2, .3))
        self.assertAlmostEqual(out.pose.position.z, .36)
        self.assertEqual((out.pose.orientation.x, out.pose.orientation.y,
                          out.pose.orientation.z, out.pose.orientation.w), (0., 0., 0., 1.))
        self.assertFalse(pub.options["latch"])
        self.assertEqual(pub.options["queue_size"], 1)
        self.assertEqual(self.status()["real_output_state"], "FORWARDING_GROUND_ONLY")
        self.assertFalse(self.status()["diagnostic_only"])
        for _ in range(3):
            self.node.tick()
        self.assertEqual(len(pub.messages), 1)

    def test_pose_before_state_is_preview_only_never_replayed(self):
        self.feed()
        self.assertNotIn(app.REAL_OUTPUT, self.ros.publishers)
        self.state()
        self.node.tick()
        self.assertNotIn(app.REAL_OUTPUT, self.ros.publishers)
        self.clock.advance(100_000_000)
        self.feed()
        self.assertEqual(self.node.real_published, 1)

    def test_startup_can_wait_for_connection_without_output(self):
        self.state(connected=False)
        self.feed()
        self.assertNotIn(app.REAL_OUTPUT, self.ros.publishers)
        self.clock.advance(100_000_000)
        self.state()
        self.feed()
        self.assertEqual(self.node.real_published, 1)

    def test_no_tcp_connection_drops_frames_instead_of_replaying(self):
        self.ros.on_advertise = lambda topic, pub: setattr(pub, "connections", 0)
        self.state()
        self.feed()
        pub = self.ros.publishers[app.REAL_OUTPUT]
        self.assertEqual(pub.messages, [])
        pub.connections = 1
        self.node.tick()
        self.assertEqual(pub.messages, [])
        self.clock.advance(100_000_000)
        self.feed()
        self.assertEqual(len(pub.messages), 1)

    def test_arming_latches_before_next_pose_even_if_disarmed_again(self):
        _, pub = self.start()
        self.clock.advance(10_000_000)
        self.state(armed=True)
        self.assertStopped("ARMED")
        self.assertTrue(pub.closed)
        self.clock.advance(10_000_000)
        self.state()
        self.feed()
        self.assertEqual(len(pub.messages), 1)
        self.assertEqual(len(self.ros.publishers[app.OUTPUT].messages), 2)

    def test_mode_change_and_disconnection_lock_independently(self):
        for fields, reason in (({"mode": "OFFBOARD"}, "MODE_NOT_MANUAL"),
                               ({"mode": "POSCTL"}, "MODE_NOT_MANUAL"),
                               ({"connected": False}, "MAVROS_NOT_CONNECTED")):
            with self.subTest(fields=fields):
                self.setUp()
                self.start()
                self.clock.advance(1)
                self.state(**fields)
                self.assertStopped(reason)

    def test_armed_or_nonmanual_at_start_cannot_create_a_real_publisher(self):
        self.state(armed=True, mode="OFFBOARD")
        self.feed()
        self.assertStopped("ARMED")
        self.assertNotIn(app.REAL_OUTPUT, self.ros.publishers)

    def test_state_expiry_with_continuing_lio_stops(self):
        _, pub = self.start()
        for _ in range(21):
            self.clock.advance(100_000_000)
            self.node.check_runtime_graph()
            self.feed()
            self.node.tick()
        self.assertStopped("MAVROS_STATE_NOT_FRESH")
        self.assertTrue(pub.closed)

    def test_bad_state_source_duplicate_future_or_malformed_state_stops(self):
        edits = (lambda m: m._connection_header.update(callerid="/other"),
                 lambda m: setattr(m.header, "stamp", Stamp(10*NS)),
                 lambda m: setattr(m.header, "stamp", Stamp(0)),
                 lambda m: setattr(m, "armed", "false"),
                 lambda m: setattr(m.header, "stamp", Stamp(self.clock.ros+NS)))
        for edit in edits:
            self.setUp()
            self.start()
            self.clock.advance(100_000_000)
            msg = telemetry(self.clock, connected=True, armed=False, mode="MANUAL")
            edit(msg)
            self.node.on_telemetry("state", "/mavros", msg)
            self.assertStopped("MAVROS_STATE_NOT_FRESH")

    def test_source_silence_locks_at_half_second_no_timer_pose(self):
        _, pub = self.start()
        for _ in range(5):
            self.clock.advance(100_000_000)
            self.node.check_runtime_graph()
            self.node.tick()
        self.assertStopped("INPUT_STALE")
        self.feed()
        self.assertEqual(len(pub.messages), 1)

    def test_source_gap_is_caught_in_callback_even_without_tick(self):
        _, pub = self.start()
        self.clock.advance(500_000_000)
        self.node.check_runtime_graph()
        self.feed()
        self.assertStopped("INPUT_STALE")
        self.assertEqual(len(pub.messages), 1)

    def test_bad_frames_and_invalid_poses_are_never_real_outputs(self):
        self.state()
        messages = [odometry(self.clock.ros, parent="wrong"),
                    odometry(self.clock.ros-NS), odometry(self.clock.ros+NS), odometry(0)]
        invalid = odometry(self.clock.ros)
        invalid.pose.pose.position.x = float("nan")
        messages.append(invalid)
        for msg in messages:
            self.node.on_odometry(msg)
        self.assertEqual(self.node.real_published, 0)
        self.assertNotIn(app.REAL_OUTPUT, self.ros.publishers)

    def test_wrong_or_latched_source_immediately_removes_real_publisher(self):
        for kind in ("source", "latch"):
            self.setUp()
            self.start()
            self.clock.advance(100_000_000)
            msg = odometry(self.clock.ros, seq=2)
            msg._connection_header["callerid" if kind == "source" else "latching"] = (
                "/other" if kind == "source" else "1")
            self.node.on_odometry(msg)
            self.assertStopped("LIO_GUARD")

    def test_clock_and_source_rollbacks_fail_closed(self):
        for kind in ("ros_jump", "ros_back", "wall_back", "source_back", "seq_reset"):
            self.setUp()
            self.start()
            if kind == "ros_jump":
                self.clock.ros += NS
            elif kind == "ros_back":
                self.clock.ros -= NS
            elif kind == "wall_back":
                self.clock.wall -= NS
            else:
                self.clock.advance(100_000_000)
                msg = odometry(10*NS-30_000_000 if kind == "source_back" else self.clock.ros,
                               seq=0 if kind == "seq_reset" else 2)
                self.node.on_odometry(msg)
            self.node.tick()
            self.assertStopped()

    def test_duplicate_inputs_cannot_keep_real_output_alive(self):
        msg, pub = self.start()
        for _ in range(6):
            self.clock.advance(100_000_000)
            self.node.check_runtime_graph()
            self.node.on_odometry(msg)
        self.assertStopped("INPUT_STALE")
        self.assertEqual(len(pub.messages), 1)

    def test_runtime_graph_conflicts_stop_without_waiting_for_pose(self):
        changes = (
            lambda g, r: g.pubs.update({app.REAL_OUTPUT: [r.get_name(), "/other"]}),
            lambda g, r: g.pubs.update({app.INPUT: [gm.SOURCE_NODE, "/other"]}),
            lambda g, r: g.pubs.update({app.INPUT: []}),
            lambda g, r: g.pubs.update({app.STATE: ["/other"]}),
            lambda g, r: g.pubs.update({"/mavros/setpoint_raw/local": ["/control"]}),
            lambda g, r: g.pubs.update({"/mavros/vision_pose/pose_cov": ["/other"]}),
            lambda g, r: g.pubs.update({"/mavros/odometry/out": ["/other"]}),
            lambda g, r: g.subs.update({app.REAL_OUTPUT: []}),
            lambda g, r: r.params.update({"/mavros/vision_pose/tf/listen": True}),
            lambda g, r: r.params.update({"/use_sim_time": True}))
        for change in changes:
            self.setUp()
            _, pub = self.start()
            change(self.graph, self.ros)
            self.node.check_runtime_graph()
            self.assertStopped()
            self.assertTrue(pub.closed)

    def test_graph_error_stale_cache_and_slow_reply_fail_closed(self):
        for kind in ("error", "stale", "slow"):
            self.setUp()
            self.start()
            if kind == "error":
                self.graph.fail = True
                self.node.check_runtime_graph()
            elif kind == "slow":
                self.graph.hook = lambda: self.clock.advance(500_000_000)
                self.node.check_runtime_graph()
            else:
                self.clock.advance(500_000_000)
                # Refresh just the LIO gate to isolate graph expiry from silence.
                self.node.gate.last_accept_wall_ns = self.clock.wall
                self.node.tick()
            self.assertStopped("GRAPH")

    def test_status_and_callbacks_do_not_make_blocking_graph_calls(self):
        self.start()
        calls = self.graph.calls
        self.node.tick()
        self.status()
        self.clock.advance(100_000_000)
        self.state()
        self.feed()
        self.assertEqual(self.graph.calls, calls)

    def test_a_blocked_graph_worker_cannot_block_arming_detection(self):
        self.start()
        entered, release = threading.Event(), threading.Event()
        def blocked():
            entered.set()
            release.wait(2)
        self.graph.hook = blocked
        worker = threading.Thread(target=self.node.check_runtime_graph, daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.clock.advance(1)
            self.state(armed=True)
            self.assertStopped("ARMED")
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_publisher_registration_delay_cannot_send_stale_pose(self):
        def slow(topic, pub):
            if topic == app.REAL_OUTPUT:
                self.clock.advance(250_000_000)
        self.ros.on_advertise = slow
        self.state()
        self.feed()
        pub = self.ros.publishers[app.REAL_OUTPUT]
        self.assertEqual(pub.messages, [])
        self.clock.advance(10_000_000)
        self.feed()
        self.assertEqual(len(pub.messages), 1)

    def test_real_publish_failure_latches_but_diagnostic_can_continue(self):
        _, pub = self.start()
        pub.fail = True
        self.clock.advance(100_000_000)
        self.feed()
        self.assertStopped("REAL_PUBLISH_FAILED")
        pub.fail = False
        self.clock.advance(100_000_000)
        self.feed()
        self.assertEqual(len(pub.messages), 1)
        self.assertEqual(len(self.ros.publishers[app.OUTPUT].messages), 3)

    def test_diagnostic_publish_failure_latches_both_outputs(self):
        _, pub = self.start()
        self.ros.publishers[app.OUTPUT].fail = True
        self.clock.advance(100_000_000)
        self.feed()
        self.assertStopped("CALLBACK_ERROR")
        self.assertTrue(pub.closed)

    def test_close_and_late_callbacks_never_publish_or_reconnect(self):
        _, pub = self.start()
        self.node.close()
        self.clock.advance(100_000_000)
        self.state()
        self.feed()
        self.node.tick()
        self.node.check_runtime_graph()
        self.assertTrue(pub.closed)
        self.assertEqual(len(pub.messages), 1)
        self.assertFalse(self.status()["real_output_active"])
        self.assertNotIn(self.ros.get_name(), self.graph.pubs[app.REAL_OUTPUT])

    def test_shutdown_flag_prevents_late_real_pose(self):
        _, pub = self.start()
        self.ros.shutdown = True
        self.clock.advance(100_000_000)
        self.feed()
        self.assertEqual(len(pub.messages), 1)

    def test_flags_and_actual_odom_never_grant_flight_authorization(self):
        self.start()
        self.node.on_telemetry("estimator", "/mavros", telemetry(
            self.clock, velocity_horiz_status_flag=True, pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=False, const_pos_mode_status_flag=False))
        self.node.on_telemetry("local_odom", "/mavros", odometry(
            self.clock.ros, parent="map", child="base_link", source="/mavros"))
        result = self.status()
        self.assertTrue(result["horizontal_flags_valid"])
        self.assertTrue(result["local_odom_received"])
        self.assertFalse(result["flight_authorized"])
        self.node.tick()
        status_json = json.loads(self.ros.publishers[app.STATUS].messages[-1].data)
        self.assertFalse(status_json["diagnostic_only"])
        self.assertFalse(status_json["flight_authorized"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
