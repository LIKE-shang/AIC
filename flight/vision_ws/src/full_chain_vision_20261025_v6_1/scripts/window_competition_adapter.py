#!/usr/bin/env python3
"""Ground calibration adapter for the legacy window pixel interface.

Default outputs are isolated below /window_test. The optional legacy topic
is for disarmed interface inspection, NOT a flight controller or safety layer.
"""
import json
import math
import threading
import time

import rosgraph
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from robot_vision_test.competition_adapter import (
    Confirmation, capture_age, parse_window_status,
)


def bool_param(name, default):
    value = rospy.get_param('~' + name, default)
    if type(value) is not bool:
        raise ValueError(name + ' must be a ROS boolean, not a string/number')
    return value


def positive_int_param(name, default):
    value = rospy.get_param('~' + name, default)
    if type(value) is not int or value < 1:
        raise ValueError(name + ' must be a positive integer')
    return value


def positive_float_param(name, default, allow_zero=False):
    value = rospy.get_param('~' + name, default)
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + ' must be finite')
    if value < 0 or (not allow_zero and value == 0):
        raise ValueError(name + ' has an invalid lower limit')
    return float(value)


def test_topic(name, default):
    value = rospy.resolve_name(rospy.get_param('~' + name, default))
    if not value.startswith('/window_test/'):
        raise ValueError(name + ' must stay under /window_test/')
    return value


class WindowCompetitionAdapter:
    def __init__(self):
        self.status_topic = test_topic('status_topic', '/window_test/status')
        self.allow_legacy_output = bool_param('allow_legacy_output', False)
        self.publish_legacy = bool_param('publish_legacy_object_position', False)
        self.output_topic = rospy.resolve_name(rospy.get_param(
            '~output_topic', '/window_test/mission_target'))
        if self.publish_legacy:
            if not self.allow_legacy_output or self.output_topic != '/object_position':
                raise ValueError('legacy inspection requires both explicit flags and '
                                 'output_topic:=/object_position')
        elif not self.output_topic.startswith('/window_test/'):
            raise ValueError('normal output must stay under /window_test/')

        self.diagnostics_topic = test_topic(
            'diagnostics_topic', '/window_test/mission_target_status')
        if len({self.status_topic, self.output_topic, self.diagnostics_topic}) != 3:
            raise ValueError('status, output and diagnostic topics must be distinct')
        self.expected_width = positive_int_param('expected_width', 640)
        self.expected_height = positive_int_param('expected_height', 480)
        if self.publish_legacy and (self.expected_width, self.expected_height) != (640, 480):
            raise ValueError('legacy tracker hardcodes 320/240; it requires 640x480')
        self.max_capture_age = positive_float_param('max_capture_age', 0.8)
        self.future_tolerance = positive_float_param('future_tolerance', 0.2, True)
        if not bool_param('require_capture_stamp', True):
            raise ValueError('capture timestamp validation cannot be disabled')
        self.on_frames = positive_int_param('on_frames', 5)
        self.off_frames = positive_int_param('off_frames', 3)
        self.status_timeout = positive_float_param('status_timeout', 1.0)
        self.min_width_px = positive_float_param('min_width_px', 25.0)
        self.min_height_px = positive_float_param('min_height_px', 25.0)
        self.alignment_tolerance_px = positive_float_param('alignment_tolerance_px', 20.0)
        methods = rospy.get_param('~allowed_methods',
                                  ['checker_texture', 'checker_contour', 'nested_frame'])
        supported = {'checker_texture', 'checker_contour', 'nested_frame'}
        if (not isinstance(methods, (list, tuple)) or not methods
                or any(type(m) is not str or m not in supported for m in methods)):
            raise ValueError('allowed_methods must be a nonempty list of supported methods')
        self.allowed_methods = tuple(methods)
        self.legacy_frame_id = rospy.get_param('~legacy_frame_id', 'square')
        self.no_target_frame_id = rospy.get_param('~no_target_frame_id', 'no object')
        if self.legacy_frame_id != 'square' or self.no_target_frame_id != 'no object':
            raise ValueError('legacy window protocol requires square / no object class IDs')
        self.publish_diagnostics = bool_param('publish_diagnostics', True)
        if self.publish_legacy:
            self.check_legacy_graph()

        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.closing = False
        self.gate = Confirmation(self.on_frames, self.off_frames)
        self.last_status_wall = None
        self.last_capture_stamp = None
        self.last_target = None
        self.last_reason = 'startup'
        self.last_published_on = False
        self.target_pub = rospy.Publisher(self.output_topic, PointStamped, queue_size=1)
        self.diag_pub = (rospy.Publisher(self.diagnostics_topic, String, queue_size=1, latch=True)
                         if self.publish_diagnostics else None)
        # Also check the actual Publisher names: command-line remaps must not
        # bypass namespace/legacy-output validation.
        if self.target_pub.resolved_name != self.output_topic:
            raise ValueError('unexpected target publisher remap')
        if self.diag_pub and self.diag_pub.resolved_name != self.diagnostics_topic:
            raise ValueError('unexpected diagnostic publisher remap')
        self.watchdog_interval = min(0.1, self.status_timeout / 3.0, self.max_capture_age / 3.0)
        rospy.on_shutdown(self.shutdown)
        self.publish_off('startup')
        self.sub = rospy.Subscriber(self.status_topic, String, self.status_cb,
                                    queue_size=1, buff_size=65536)
        # Wall-clock waiting works even when /clock stops or ROS time jumps.
        self.watchdog = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog.start()
        rospy.logwarn('Window GROUND adapter: topic=%s legacy=%s. No motion interface; '
                      'NOT approved for flight.', self.output_topic, self.publish_legacy)

    def check_legacy_graph(self):
        try:
            publishers, subscribers, services = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as exc:
            raise ValueError('cannot check legacy topic/mission conflicts: %s' % exc)
        for topic, nodes in publishers:
            other = [n for n in nodes if n != rospy.get_name()]
            if topic == '/object_position' and other:
                raise ValueError('/object_position already has publishers: %s' % other)
        protected = {'uav_mission', 'agv_mission', 'complete_mission'}
        nodes = {n for group in (publishers, subscribers, services) for _, owners in group for n in owners}
        active = sorted(n for n in nodes if n.rsplit('/', 1)[-1] in protected)
        if active:
            raise ValueError('legacy inspection refused while mission nodes are present: %s' % active)

    def publish_diag(self, reason, target=None):
        if self.diag_pub is None:
            return
        payload = dict(state='target_valid' if self.last_published_on else 'no_target',
                       output_valid=self.last_published_on, reason=reason,
                       confirmed=self.gate.on, absence_confirmed=self.gate.misses >= self.off_frames,
                       hits=self.gate.hits, misses=self.gate.misses,
                       legacy_output_enabled=self.publish_legacy,
                       output_topic=self.output_topic, flight_ready=False)
        if target is not None:
            error_x = target['center_x'] - target['image_width'] / 2.0
            error_y = target['center_y'] - target['image_height'] / 2.0
            payload.update(center_px=[target['center_x'], target['center_y']],
                           error_x_px=error_x, error_y_px=error_y,
                           horizontal_centered=abs(error_x) < self.alignment_tolerance_px,
                           alignment_tolerance_px=self.alignment_tolerance_px,
                           capture_stamp=target['capture_stamp'], method=target['method'],
                           outline_kind=target['outline_kind'],
                           camera_frame_id=target['camera_frame_id'])
        self.diag_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_off(self, reason, candidate=None):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.no_target_frame_id
        self.target_pub.publish(msg)
        self.last_published_on = False
        self.last_reason = str(reason)
        self.publish_diag(self.last_reason, candidate)

    def publish_target(self, target):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.from_sec(target['capture_stamp'])
        msg.header.seq = target['frame_seq']
        msg.header.frame_id = self.legacy_frame_id
        msg.point.x = target['center_x']
        msg.point.y = target['center_y']
        msg.point.z = 0.0  # Unmeasured, NOT a zero-metre range.
        self.target_pub.publish(msg)
        self.last_published_on = True
        self.last_reason = 'target_confirmed'
        self.publish_diag(self.last_reason, target)

    def expired_reason(self, now_wall, now_ros):
        if self.last_status_wall is None:
            return 'waiting_for_status'
        if now_wall - self.last_status_wall >= self.status_timeout:
            return 'status_timeout'
        if self.last_target is not None:
            fresh, reason = capture_age(self.last_target, now_ros,
                                        self.max_capture_age, self.future_tolerance)
            if not fresh:
                return reason
        return None

    def status_cb(self, msg):
        target, reason = parse_window_status(
            msg.data, expected_width=self.expected_width, expected_height=self.expected_height,
            min_width_px=self.min_width_px, min_height_px=self.min_height_px,
            allowed_methods=self.allowed_methods)
        with self.lock:
            if self.closing:
                return
            now_wall, now_ros = time.monotonic(), rospy.Time.now().to_sec()
            # Reconfirm after a gap even if the watchdog has not run yet.
            if self.expired_reason(now_wall, now_ros):
                self.gate.invalidate()
                self.last_target = None
            self.last_status_wall = now_wall
            if target is not None:
                fresh, reason = capture_age(target, now_ros, self.max_capture_age, self.future_tolerance)
                if not fresh:
                    target = None
                elif (self.last_capture_stamp is not None
                      and target['capture_stamp'] <= self.last_capture_stamp):
                    target, reason = None, 'duplicate_or_reordered_capture'
                else:
                    self.last_capture_stamp = target['capture_stamp']
            if target is not None:
                self.last_target = target
                if self.gate.found():
                    self.publish_target(target)
                else:
                    self.publish_off('confirming_target', target)
            else:
                self.last_target = None
                self.gate.missing()
                self.publish_off(reason)

    def watchdog_cb(self):
        with self.lock:
            if self.closing:
                return
            reason = self.expired_reason(time.monotonic(), rospy.Time.now().to_sec())
            if reason:
                self.last_target = None
                self.gate.invalidate()
                self.publish_off(reason)
            elif not self.last_published_on:
                self.publish_off(self.last_reason, self.last_target)

    def watchdog_loop(self):
        while not self.stop_event.wait(self.watchdog_interval):
            try:
                self.watchdog_cb()
            except Exception as exc:
                with self.lock:
                    self.gate.invalidate()
                    self.last_target = None
                rospy.logerr('Window adapter watchdog error: %s', exc)
                self.shutdown()
                rospy.signal_shutdown('adapter watchdog failed')
                return

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            if self.closing:
                return
            self.closing = True
            self.gate.invalidate()
            self.last_target = None
            try:
                self.publish_off('adapter_shutdown')
            except Exception as exc:
                rospy.logerr('Failed to publish no-target on shutdown: %s', exc)
        if hasattr(self, 'watchdog') and threading.current_thread() is not self.watchdog:
            self.watchdog.join(timeout=1.0)


if __name__ == '__main__':
    rospy.init_node('window_competition_adapter')
    node = None
    try:
        node = WindowCompetitionAdapter()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('Window competition adapter refused/failed: %s', exc)
        raise
    finally:
        if node is not None:
            node.shutdown()
