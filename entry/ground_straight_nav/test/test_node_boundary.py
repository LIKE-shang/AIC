#!/usr/bin/env python3
"""Boundary test: the messages actually published carry angular.z == 0.

Real ``rospy`` is replaced by stubs before importing the node, so this runs on
a machine without ROS as well as on the Nano.  What is under test here is not
the control law (``test_straight_nav_core`` covers that) but the ROS edge: the
published ``Twist``, the frame filter, and the service gates.
"""
import math
import os
import sys
import types
import unittest
import time
from unittest.mock import patch


def _install_stubs():
    """Put minimal fake rospy/rosgraph/message modules into sys.modules."""
    if 'rospy' in sys.modules and not getattr(sys.modules['rospy'], '_is_stub', False):
        # A real rospy is loaded; leave it alone, the caller should have
        # imported the stubs first.  Tests below degenerate to quat_to_yaw.
        return False

    rospy = types.ModuleType('rospy')
    rospy._is_stub = True
    rospy.published = []          # (topic, msg) captured from publishers
    rospy.services = {}
    rospy.subscribers = []
    rospy.timers = []
    rospy.shutdown_hooks = []

    class Publisher(object):
        def __init__(self, topic, msg_class, queue_size=None):
            self.topic = topic
            self.msg_class = msg_class

        def publish(self, msg):
            rospy.published.append((self.topic, msg))

    class Subscriber(object):
        def __init__(self, topic, msg_class, callback, queue_size=None):
            self.topic = topic
            self.callback = callback
            rospy.subscribers.append(self)

    class Timer(object):
        def __init__(self, duration, callback):
            self.duration = duration
            self.callback = callback
            rospy.timers.append(self)

    class Duration(object):
        def __init__(self, seconds):
            self.seconds = seconds

    def Service(topic, srv_class, callback):
        rospy.services[topic] = callback

    def init_node(name, anonymous=False):
        rospy.node_name = name

    def on_shutdown(hook):
        rospy.shutdown_hooks.append(hook)

    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    rospy.Timer = Timer
    rospy.Duration = Duration
    rospy.Service = Service
    rospy.init_node = init_node
    rospy.on_shutdown = on_shutdown
    rospy.get_param = lambda name, default=None: default
    rospy.get_name = lambda: '/ground_straight_nav'
    rospy.resolve_name = lambda name: name
    rospy.myargv = lambda argv=None: [a for a in (sys.argv if argv is None else argv) if ':=' not in a]
    rospy.is_shutdown = lambda: False
    rospy.spin = lambda: None
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.logerr = lambda *a, **k: None
    rospy.ROSInterruptException = type('ROSInterruptException', (Exception,), {})
    class Stamp:
        def __init__(self, seconds=None):
            self.seconds = time.time() if seconds is None else seconds
        def to_sec(self):
            return self.seconds
    rospy.Time = types.SimpleNamespace(now=Stamp)

    class _Rate(object):
        def __init__(self, hz):
            self.hz = hz

        def sleep(self):
            pass

    rospy.Rate = _Rate

    rosgraph = types.ModuleType('rosgraph')

    class Master(object):
        def __init__(self, name):
            self.name = name

        def getSystemState(self):
            return ([('/cmd_vel', ['/ground_straight_nav'])], [], [])

    rosgraph.Master = Master

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class Vector3(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class Twist(object):
        def __init__(self):
            self.linear = Vector3()
            self.angular = Vector3()

    geometry_msgs_msg.Twist = Twist
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')

    class Position(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class Orientation(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class Pose(object):
        def __init__(self):
            self.position = Position()
            self.orientation = Orientation()

    class PoseWithCovariance(object):
        def __init__(self):
            self.pose = Pose()

    class Header(object):
        def __init__(self):
            self.frame_id = 'map'
            self.stamp = Stamp()

    class Odometry(object):
        def __init__(self):
            self.header = Header()
            self.pose = PoseWithCovariance()

    nav_msgs_msg.Odometry = Odometry
    nav_msgs.msg = nav_msgs_msg

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')

    class String(object):
        def __init__(self, data=''):
            self.data = data

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    std_srvs = types.ModuleType('std_srvs')
    std_srvs_srv = types.ModuleType('std_srvs.srv')

    class TriggerResponse(object):
        def __init__(self, success=False, message=''):
            self.success = success
            self.message = message

    std_srvs_srv.Trigger = object
    std_srvs_srv.TriggerResponse = TriggerResponse
    std_srvs.srv = std_srvs_srv

    for name, module in (('rospy', rospy), ('rosgraph', rosgraph),
                         ('geometry_msgs', geometry_msgs),
                         ('geometry_msgs.msg', geometry_msgs_msg),
                         ('nav_msgs', nav_msgs), ('nav_msgs.msg', nav_msgs_msg),
                         ('std_msgs', std_msgs), ('std_msgs.msg', std_msgs_msg),
                         ('std_srvs', std_srvs), ('std_srvs.srv', std_srvs_srv)):
        sys.modules[name] = module
    return True


_HAVE_STUBS = _install_stubs()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))

import straight_nav_node as node_module  # noqa: E402
from ground_straight_nav.core import DEFAULTS, check_config  # noqa: E402


def make_node(**overrides):
    cfg = check_config(overrides)
    waypoints = [{'map': [1.0, 0.0]}, {'map': [1.0, 0.5]}]
    node_module.rospy.published[:] = []
    return node_module.StraightNavNode(cfg, waypoints, '/localization',
                                       cfg['control_rate_hz'], dry_run=False)


_stamp_counter = 0

def odom(x, y, yaw, frame='map'):
    global _stamp_counter
    _stamp_counter += 1
    msg = sys.modules['nav_msgs.msg'].Odometry()
    msg.header.frame_id = frame
    msg.header.stamp.seconds += _stamp_counter * 0.00001
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


@unittest.skipUnless(_HAVE_STUBS, 'real rospy present; stubs not installed')
class TestPublishedTwist(unittest.TestCase):
    def test_terminal_status_survives_expired_identity_watchdog(self):
        for state, reason in [('DONE', 'ALL_WAYPOINTS_REACHED'), ('ABORT', 'POSE_STALE')]:
            node = make_node(heading_hold_enabled=True)
            node.run.state, node.run.reason = state, reason
            node.started = True
            node._heading_graph_wall = 1.
            with patch.object(node_module.time, 'monotonic', return_value=10.):
                node._tick(None)
                node._tick(None)
            self.assertEqual((node.run.state, node.run.reason), (state, reason))
            self.assertFalse(node.started)
            last = [m for t,m in node_module.rospy.published if t == node_module.CMD_TOPIC][-1]
            self.assertEqual((last.linear.x,last.linear.y,last.angular.z), (0.,0.,0.))

    def test_active_identity_watchdog_still_aborts(self):
        node = make_node(heading_hold_enabled=True)
        node.run.state = 'RUNNING'
        node.started = True
        node._heading_graph_wall = 1.
        with patch.object(node_module.time, 'monotonic', return_value=10.):
            node._tick(None)
        self.assertEqual((node.run.state,node.run.reason), ('ABORT','HEADING_IDENTITY_STALE'))
        self.assertFalse(node.started)

    def _cmd_messages(self):
        return [m for topic, m in node_module.rospy.published
                if topic == node_module.CMD_TOPIC]

    def test_idle_publishes_zero_and_zero_angular(self):
        node = make_node()
        node._tick(None)
        cmds = self._cmd_messages()
        self.assertTrue(cmds)
        for cmd in cmds:
            self.assertEqual(cmd.angular.z, 0.0)
            self.assertEqual(cmd.linear.x, 0.0)
            self.assertEqual(cmd.linear.y, 0.0)

    def test_running_publishes_angular_zero_exactly(self):
        node = make_node()
        node._pose_cb(odom(0.0, 0.0, 0.0))
        ok = node._start_srv(None)
        self.assertTrue(ok.success, ok.message)
        for _ in range(20):
            node._tick(None)
        cmds = self._cmd_messages()
        self.assertGreater(len(cmds), 10)
        for cmd in cmds:
            # Literal zero, not "small": the invariant is structural.
            self.assertEqual(cmd.angular.z, 0.0)
            self.assertEqual(cmd.angular.x, 0.0)
            self.assertEqual(cmd.angular.y, 0.0)

    def test_command_grows_towards_goal_and_stays_capped(self):
        node = make_node(max_speed_mps=0.1, accel_limit_mps2=1.0)
        node._pose_cb(odom(0.0, 0.0, 0.0))
        node._start_srv(None)
        speeds = []
        for _ in range(60):
            node._tick(None)
            cmd = self._cmd_messages()[-1]
            speeds.append(math.hypot(cmd.linear.x, cmd.linear.y))
        self.assertGreater(speeds[-1], speeds[0])
        for speed in speeds:
            self.assertLessEqual(speed, 0.1 + 1e-9)

    def test_enabled_heading_is_published_and_stop_clears_it(self):
        node = make_node(heading_hold_enabled=True)
        node._pose_cb(odom(0,0,0))
        now=time.monotonic();node.run.heading_update(0.,1.,now)
        with patch.object(node,'_heading_identity',return_value=('lio','uri','run')):
            self.assertTrue(node._start_srv(None).success)
            node.run.heading_update(.06,1.1,now+.1)
            with patch.object(node_module.time,'monotonic',return_value=now+.1):
                node._tick(None)
            cmd=self._cmd_messages()[-1]
            self.assertLess(cmd.angular.z,0.)
            self.assertEqual(cmd.angular.x,0.)
            self.assertEqual(cmd.angular.y,0.)
            node._stop_srv(None);node._tick(None)
            self.assertEqual(self._cmd_messages()[-1].angular.z,0.)

    def test_wrong_lio_source_latches_fault(self):
        node=make_node(heading_hold_enabled=True)
        m=odom(0,0,0,frame='camera_init');m._connection_header={'callerid':'/other_lio'}
        node._heading_cb(m)
        self.assertEqual(node.run.heading.fault,'HEADING_SOURCE_CHANGED')

    def test_identity_change_stops_active_heading_control(self):
        node=make_node(heading_hold_enabled=True)
        node._pose_cb(odom(0,0,0));node.run.heading_update(0.,1.,time.monotonic())
        with patch.object(node,'_heading_identity',return_value=('lio','uri','run')):
            self.assertTrue(node._start_srv(None).success)
        with patch.object(node,'_heading_identity',return_value=('lio','new_uri','run')):
            node._heading_identity_tick(None)
        self.assertEqual(node.run.state,'ABORT')
        self.assertEqual(self._cmd_messages()[-1].angular.z,0.)


@unittest.skipUnless(_HAVE_STUBS, 'real rospy present; stubs not installed')
class TestFrameFilter(unittest.TestCase):
    def test_camera_init_pose_is_rejected(self):
        node = make_node()
        node._pose_cb(odom(5.0, 5.0, 1.0, frame='camera_init'))
        self.assertIsNone(node.run.pose)
        self.assertTrue(node._warned_frame)

    def test_map_pose_is_accepted(self):
        node = make_node()
        node._pose_cb(odom(1.0, 2.0, 0.3))
        self.assertIsNotNone(node.run.pose)
        self.assertAlmostEqual(node.run.pose[1], 2.0, places=9)

    def test_allow_non_map_flag_accepts_other_frames(self):
        cfg = check_config(DEFAULTS)
        node_module.rospy.published[:] = []
        node = node_module.StraightNavNode(cfg, [{'map': [0.0, 0.0]}],
                                           '/Odometry', 20.0,
                                           allow_non_map=True)
        node._pose_cb(odom(1.0, 2.0, 0.3, frame='camera_init'))
        self.assertIsNotNone(node.run.pose)


@unittest.skipUnless(_HAVE_STUBS, 'real rospy present; stubs not installed')
class TestServices(unittest.TestCase):
    def test_start_without_pose_is_refused(self):
        node = make_node()
        response = node._start_srv(None)
        self.assertFalse(response.success)
        # No pose was ever received, so the machine never left WAITING_POSE.
        self.assertIn('WAITING_POSE', response.message)

    def test_stop_aborts_and_publishes_zero(self):
        node = make_node()
        node._pose_cb(odom(0.0, 0.0, 0.0))
        node._start_srv(None)
        node._tick(None)
        response = node._stop_srv(None)
        self.assertTrue(response.success)
        self.assertEqual(node.run.state, 'ABORT')
        last = [m for t, m in node_module.rospy.published
                if t == node_module.CMD_TOPIC][-1]
        self.assertEqual((last.linear.x, last.linear.y, last.angular.z),
                         (0.0, 0.0, 0.0))

    def test_dry_run_creates_no_publisher(self):
        node_module.rospy.published[:] = []  # drop the previous node's records
        node = node_module.StraightNavNode(check_config(DEFAULTS),
                                           [{'map': [0.0, 0.0]}],
                                           '/localization', 20.0, dry_run=True)
        self.assertIsNone(node.cmd_pub)
        node._pose_cb(odom(0.0, 0.0, 0.0))
        node._start_srv(None)
        for _ in range(5):
            node._tick(None)  # must not raise and must not publish
        self.assertEqual([m for t, m in node_module.rospy.published
                          if t == node_module.CMD_TOPIC], [])


class TestQuatToYaw(unittest.TestCase):
    def test_round_trip(self):
        for yaw in (0.0, 0.3, -1.2, 3.0, -3.0):
            msg = odom(0.0, 0.0, yaw)
            q = msg.pose.pose.orientation
            back = node_module.quat_to_yaw(q)
            self.assertAlmostEqual(math.atan2(math.sin(yaw - back),
                                              math.cos(yaw - back)), 0.0,
                                   places=9)


class TestAuditRegressions(unittest.TestCase):
    def test_empty_frame_rejected(self):
        node = make_node()
        node._pose_cb(odom(0, 0, 0, frame=''))
        self.assertIsNone(node.run.pose)

    def test_old_timestamp_rejected(self):
        node = make_node()
        msg = odom(0, 0, 0)
        msg.header.stamp.seconds -= 10
        node._pose_cb(msg)
        self.assertIsNone(node.run.pose)

    def test_zero_quaternion_rejected(self):
        node = make_node()
        msg = odom(0, 0, 0)
        msg.pose.pose.orientation.w = 0
        node._pose_cb(msg)
        self.assertIsNone(node.run.pose)

    def test_repeated_pose_does_not_refresh_watchdog(self):
        node = make_node()
        msg = odom(0, 0, 0)
        with patch.object(node_module.time, 'monotonic', return_value=10):
            node._pose_cb(msg)
            self.assertTrue(node._start_srv(None).success)
        with patch.object(node_module.time, 'monotonic', return_value=11):
            node._pose_cb(msg)
            node._tick(None)
        self.assertEqual(node.run.state, 'ABORT')

    def test_restart_relatches_at_service_boundary(self):
        node = make_node()
        node.waypoints = [{'relative': [1, 0]}]
        node._pose_cb(odom(0, 0, 0))
        self.assertTrue(node._start_srv(None).success)
        node._stop_srv(None)
        node._pose_cb(odom(2, 3, math.pi/2))
        self.assertTrue(node._start_srv(None).success)
        self.assertAlmostEqual(node.run.waypoints[0][0], 2)
        self.assertAlmostEqual(node.run.waypoints[0][1], 4)

    def test_first_start_uses_latest_pose(self):
        node = make_node()
        node._pose_cb(odom(0, 0, 0))
        node._pose_cb(odom(2, 3, 0.4))
        self.assertTrue(node._start_srv(None).success)
        self.assertEqual(node.run.start_xy, (2, 3))

    def test_start_in_dwell_does_not_reset_waypoints(self):
        node = make_node()
        node._pose_cb(odom(0, 0, 0))
        node._start_srv(None)
        node.run.state = 'DWELL'
        node.run.index = 1
        self.assertFalse(node._start_srv(None).success)
        self.assertEqual(node.run.index, 1)

    def test_start_with_other_cmd_publisher_refused(self):
        node = make_node()
        node._pose_cb(odom(0, 0, 0))
        with patch.object(node_module.rosgraph.Master, 'getSystemState',
                          return_value=([('/cmd_vel', ['/move_base', '/ground_straight_nav'])], [], [])):
            self.assertFalse(node._start_srv(None).success)

    def test_start_with_old_received_pose_refused(self):
        node = make_node()
        node._pose_cb(odom(0, 0, 0))
        node.run.pose_wall -= 10
        self.assertFalse(node._start_srv(None).success)

    def test_stop_waits_for_inflight_tick_and_is_last_command(self):
        import threading
        node = make_node()
        node._pose_cb(odom(0, 0, 0))
        self.assertTrue(node._start_srv(None).success)
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        original_publish = node.cmd_pub.publish
        def publish(msg):
            if msg.linear.x != 0:
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('test synchronization timeout')
            original_publish(msg)
        node.cmd_pub.publish = publish
        tick = threading.Thread(target=node._tick, args=(None,))
        def stop():
            node._stop_srv(None)
            stopped.set()
        tick.start()
        self.assertTrue(entered.wait(2))
        stopper = threading.Thread(target=stop)
        stopper.start()
        self.assertFalse(stopped.wait(0.05))
        release.set()
        tick.join(2)
        stopper.join(2)
        self.assertTrue(stopped.is_set())
        last = [m for t,m in node_module.rospy.published if t == node_module.CMD_TOPIC][-1]
        self.assertEqual((last.linear.x,last.linear.y,last.angular.z),(0,0,0))

    def test_roslaunch_remapping_arguments(self):
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write('waypoints: [{relative: [1, 0]}]\n')
            name = f.name
        try:
            self.assertEqual(node_module.main(['--config', name, '__name:=ground_straight_nav', '__log:=/tmp/ros.log']), 0)
        finally:
            os.unlink(name)


if __name__ == '__main__':
    unittest.main()
