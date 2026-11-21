#!/usr/bin/env python3
"""Independent three-ring handheld test; no motion/flight or PWM clients here."""
import json
import threading
import time
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot_vision.msg import LaserDecision
from robot_vision_test.air_target import (air_parameters, detect_air_target, detect_spot,
                                        assess_aim, assess_spot, draw_air_overlay, jsonable)
from robot_vision_test.safety import ContinuousHitGate, FrameFreshness, ContentFreshness, safe_test_topic


class CircleLaserTest:
    def __init__(self):
        self.bridge = CvBridge()
        self.params = air_parameters(rospy.get_param('~', {}))
        enabled = rospy.get_param('~enable_laser', False)
        if type(enabled) is not bool: raise ValueError('enable_laser must be boolean')
        self.enable_laser = enabled
        if self.enable_laser and rospy.get_param('/use_sim_time', False):
            raise ValueError('real laser forbidden with simulated ROS time/replay')
        if self.enable_laser:
            p = self.params
            if not (p['dimensions_confirmed'] and p['calibration']['confirmed'] and
                    p['operating_envelope_confirmed'] and p['live_camera_verified']):
                raise ValueError('real laser requires verified dimensions, calibration, live camera and operating envelope')
            if p['feedback_mode'] == 'camera' and not p['spot']['validated']:
                raise ValueError('camera feedback must be validated against actual spot/background')
            if p['feedback_mode'] == 'prediction_only' and not p['prediction_only_acknowledged']:
                raise ValueError('prediction-only requires explicit acknowledgement and manual physical record')
        self.image_timeout = float(rospy.get_param('~image_timeout', .35))
        if rospy.get_param('~off_frames', 1) != 1:
            raise ValueError('handheld test requires immediate OFF: off_frames=1')
        self.gate = ContinuousHitGate(rospy.get_param('~on_frames', 5), self.image_timeout,
                                      float(rospy.get_param('~hold_seconds', 3.0)), self.params['feedback_mode'],
                                      float(rospy.get_param('~spot_acquire_timeout', .30)),
                                      float(rospy.get_param('~max_on_seconds', 5.0)))
        self.freshness = FrameFreshness(float(rospy.get_param('~max_image_age', .35)),
                                       float(rospy.get_param('~future_tolerance', .05)), True)
        self.content_freshness = ContentFreshness(float(rospy.get_param('~frozen_content_timeout', .25)))
        self.lock = threading.RLock()
        self.frame_lock = threading.Lock()
        self.stop = threading.Event()
        self.closed = False
        self.previous_target = None
        self.generation = 0
        self.last_header = None
        self.last_shape = (480, 640, 3)  # warning canvas only, NEVER an aim coordinate
        self.laser_status = dict(state='CONTROLLER_UNAVAILABLE', dry_run=not self.enable_laser)
        self.laser_status_wall = 0.0
        self.ack_ros = None
        self.details = dict(target_valid=False, aim_valid=False, calibration_valid=False)
        def topic(param, default):
            name = safe_test_topic(rospy.resolve_name(rospy.get_param('~'+param, default)))
            if not name.startswith('/circle_laser_test/'):
                raise ValueError('circle topics must stay in /circle_laser_test/')
            return name
        self.decision_pub = rospy.Publisher(topic('decision_topic', '/circle_laser_test/decision'), LaserDecision, queue_size=1, latch=False)
        self.image_pub = rospy.Publisher(topic('output_topic', '/circle_laser_test/image'), Image, queue_size=1)
        self.edges_pub = rospy.Publisher(topic('edges_topic', '/circle_laser_test/edges'), Image, queue_size=1)
        self.status_pub = rospy.Publisher(topic('detection_status_topic', '/circle_laser_test/detection_status'), String, queue_size=1)
        self.publish_edges = bool(rospy.get_param('~publish_edges', True))
        self.dry_status_pub = None
        if self.enable_laser:
            self.status_sub = rospy.Subscriber(topic('laser_status_topic', '/circle_laser_test/laser_status'), String, self.status_callback, queue_size=1)
        else:
            self.dry_status_pub = rospy.Publisher(topic('laser_status_topic', '/circle_laser_test/laser_status'), String, queue_size=1, latch=True)
        rospy.on_shutdown(self.close)
        self._decision(False, 'startup')
        self.watchdog = threading.Thread(target=self.watchdog_loop, name='circle_wall_watchdog', daemon=True)
        self.watchdog.start()
        self.input_topic = rospy.resolve_name(rospy.get_param('~image_topic', '/usb_cam/image_raw'))
        self.sub = rospy.Subscriber(self.input_topic, Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.loginfo('Three-ring handheld test: input=%s enable_laser=%s; 3s is continuous, ACK != physical light', self.input_topic, self.enable_laser)

    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict): raise ValueError('expected status object')
            with self.lock:
                if status.get('state') == 'ON_ACKNOWLEDGED' and self.laser_status.get('state') != 'ON_ACKNOWLEDGED':
                    self.ack_ros = rospy.Time.now().to_sec()
                self.laser_status, self.laser_status_wall = status, time.monotonic()
                if status.get('fault_latched') or status.get('state') == 'UNKNOWN':
                    self.invalidate('controller_fault_or_unknown')
        except (ValueError, TypeError):
            self.invalidate('malformed_controller_status')
            rospy.logwarn_throttle(2.0, 'Malformed laser controller status -> OFF')

    def _decision(self, valid, reason):
        msg = LaserDecision()
        if self.last_header is not None: msg.header = self.last_header
        msg.enable_laser = self.enable_laser
        msg.observation_valid = valid
        msg.request_on = self.gate.on if valid else False
        msg.hit_count, msg.miss_count = self.gate.hits, self.gate.misses
        msg.target_valid = bool(valid and self.details.get('target_valid', False))
        msg.aim_valid = bool(valid and self.details.get('aim_valid', False))
        msg.calibration_valid = bool(valid and self.details.get('calibration_valid', False))
        msg.observed_hit = bool(valid and self.details.get('observed_hit', False))
        msg.continuous_seconds = self.gate.elapsed
        msg.feedback_mode = self.params['feedback_mode']
        msg.reason = reason
        self.decision_pub.publish(msg)
        if self.dry_status_pub is not None:
            self.laser_status = dict(state='DRY_RUN_ON' if msg.request_on else 'DRY_RUN_OFF',
                                     dry_run=True, requested_on=msg.request_on,
                                     hardware_light_verified=False, reason=reason)
            self.laser_status_wall = time.monotonic()
            self.dry_status_pub.publish(String(data=json.dumps(self.laser_status)))
        status = dict(self.details, valid=valid, request_on=msg.request_on, hits=msg.hit_count,
                      misses=msg.miss_count, reason=reason, continuous_seconds=self.gate.elapsed,
                      result=self.gate.result, completed_seconds=self.gate.completed_seconds,
                      completed_capture_interval=self.gate.completed_capture_interval, session_latched=self.gate.latched,
                      dry_run=not self.enable_laser, physical_hit_verified=False,
                      feedback_mode=self.params['feedback_mode'], controller=self.laser_status)
        self.status_pub.publish(String(data=json.dumps(jsonable(status), allow_nan=False)))

    def invalidate(self, reason):
        with self.lock:
            self.gate.invalidate(reason)
            self.previous_target = None
            self.generation += 1
            # Never leave old true geometry/evidence fields in an invalid status.
            self.details.update(target_valid=False, aim_valid=False, calibration_valid=False, observed_hit=False)
            self._decision(False, reason)

    def _overlay(self, frame, detection=None, aim=None, spot=None):
        state = self.laser_status.get('state', 'UNKNOWN') if time.monotonic()-self.laser_status_wall < .6 else 'STALE_CONTROLLER'
        lines = [self.gate.reason, 'REQUEST %s | %s | dry=%s' % ('ON' if self.gate.on else 'OFF', state, not self.enable_laser),
                 'hold %.2f/3s | result %s | physical acceptance UNVERIFIED' % (self.gate.elapsed, self.gate.result)]
        if detection:
            score = detection['targets'][0]['confidence'] if detection['targets'] else 0.0
            lines.append('target %s | rings=%d geometry_score=%.2f (not probability)' % (detection['reason'], len(detection['rings']), score))
        if spot:
            lines.append('spot: %s | validated=%s' % (spot['reason'], self.params['spot']['validated']))
        if aim and aim.get('error_px') is not None:
            lines.append('predicted error px=(%.1f,%.1f) norm=%.3f effective=%.1fmm' %
                         (*aim['error_px'], aim['normalized_error'], aim['effective_radius_m']*1000))
        draw_air_overlay(frame, detection or dict(targets=[]), aim, spot or {}, lines)

    def callback(self, msg):
        if not self.frame_lock.acquire(False): return
        result_before = self.gate.result
        try:
            started = time.monotonic()
            with self.lock:
                if self.closed: return
                generation = self.generation
                self.last_header = msg.header
                stamp = msg.header.stamp.to_sec()
                self.freshness.check(stamp, rospy.Time.now().to_sec())
            if msg.encoding.upper() in ('16UC1', '32FC1', 'MONO16'):
                raise ValueError('requires USB RGB/mono8, not raw depth')
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
            self.content_freshness.check(frame.tobytes(), time.monotonic())
            detection, edges = detect_air_target(frame, self.params)
            spot = detect_spot(frame, self.params)
            target = detection['targets'][0] if detection['valid'] else None
            aim = assess_aim(target, frame.shape, msg.header.frame_id, self.params) if target is not None else {}
            with self.lock:
                if self.closed or generation != self.generation: return
                age = rospy.Time.now().to_sec()-stamp
                if age > self.freshness.max_age or age < -self.freshness.future_tolerance:
                    raise ValueError('processing finished with stale/future capture')
                now = time.monotonic()
                changed = False
                if target is not None and self.previous_target is not None:
                    old = self.previous_target
                    changed = (np.linalg.norm(target['center']-old['center']) > self.params['track_center_ratio']*old['radius'] or
                               abs(target['radius']/old['radius']-1) > self.params['track_scale_ratio'])
                observed = bool(target is not None and assess_spot(spot, aim, target, self.params))
                eligible = bool(target is not None and aim.get('aim_valid') and not changed)
                reason = 'target_changed' if changed else aim.get('reason', detection['reason'])
                acknowledged = not self.enable_laser  # dry-run simulation only, never a hardware claim
                if self.enable_laser:
                    status = self.laser_status
                    healthy = (now-self.laser_status_wall < .6 and status.get('startup_off_acked') is True and
                               status.get('fault_latched') is False and status.get('fcu_connected') is True and status.get('fcu_armed') is False)
                    if not healthy or (not self.gate.on and status.get('state') != 'OFF_ACKNOWLEDGED'):
                        eligible = False; reason = 'controller_not_ready'
                    acknowledged = bool(status.get('state') == 'ON_ACKNOWLEDGED' and self.ack_ros is not None and stamp > self.ack_ros)
                # Camera mode demands dark/off negative evidence BEFORE permitting any ON.
                if self.params['feedback_mode'] == 'camera' and not self.gate.on and (spot['candidates'] or spot.get('uncertain')):
                    eligible = False; reason = 'spot_like_background_while_off'
                # An identified off-target/multiple spot is never granted acquisition grace.
                if self.params['feedback_mode'] == 'camera' and self.gate.on and (spot['candidates'] or spot.get('uncertain')) and not observed:
                    eligible = False; reason = 'actual_spot_outside_or_ambiguous'
                self.previous_target = target
                self.details = dict(target_valid=detection['valid'], calibration_valid=aim.get('calibration_valid', False),
                                    aim_valid=aim.get('aim_valid', False), observed_hit=bool(acknowledged and observed and self.gate.on),
                                    detection=detection, aim=aim, spot=spot,
                                    input_topic=self.input_topic, image_width=frame.shape[1], image_height=frame.shape[0],
                                    camera_frame_id=msg.header.frame_id, capture_stamp=stamp, frame_seq=msg.header.seq,
                                    capture_age_s=age, processing_ms=(now-started)*1000)
                self.gate.observe(eligible, now, stamp, reason, acknowledged, observed)
                self.details['observed_hit'] = bool(eligible and acknowledged and observed and self.gate.on)
                self.last_shape = frame.shape
                self._overlay(frame, detection, aim, spot)
                out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'); out.header = msg.header
                self.image_pub.publish(out)
                if self.publish_edges:
                    debug = self.bridge.cv2_to_imgmsg(edges, encoding='mono8'); debug.header = msg.header
                    self.edges_pub.publish(debug)
                final_age = rospy.Time.now().to_sec()-stamp
                if final_age > self.freshness.max_age or final_age < -self.freshness.future_tolerance:
                    raise ValueError('capture expired during output preparation')
                if time.monotonic()-now >= self.image_timeout:
                    raise ValueError('output preparation exceeded image lease')
                # Decision LAST: conversion/render/publish failures force OFF.
                self._decision(detection['valid'], self.gate.reason)
        except Exception as exc:
            with self.lock:
                if result_before == 'none' and self.gate.result != 'none':
                    self.gate.result = 'none'
                    self.gate.completed_seconds = 0.0
                    self.gate.completed_capture_interval = None
            self.invalidate('invalid_image: '+str(exc))
            rospy.logerr_throttle(2.0, 'Three-ring frame invalid -> OFF: %s' % exc)
        finally:
            self.frame_lock.release()

    def watchdog_loop(self):
        while not self.stop.wait(.025):
            try:
                with self.lock:
                    if self.closed: return
                    previous_reason = self.gate.reason
                    if self.gate.watchdog(time.monotonic()):
                        if previous_reason != self.gate.reason:
                            self.generation += 1
                        self.previous_target = None
                        self.details.update(target_valid=False, aim_valid=False, calibration_valid=False, observed_hit=False)
                        self._decision(False, self.gate.reason)
                        frame = np.zeros(self.last_shape, np.uint8)
                        self._overlay(frame)
                        warning = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        if self.last_header is not None: warning.header = self.last_header
                        self.image_pub.publish(warning)  # old capture header, not a fake new camera frame
            except Exception as exc:
                rospy.logerr('Watchdog failure -> shutdown: %s', exc)
                self.invalidate('watchdog_exception'); rospy.signal_shutdown('watchdog_exception'); return

    def close(self):
        with self.lock:
            if self.closed: return
            self.closed = True; self.stop.set()
            self.gate.invalidate('detector_shutdown')
            self.details.update(target_valid=False, aim_valid=False, calibration_valid=False, observed_hit=False)
            for _ in range(3): self._decision(False, 'detector_shutdown')


if __name__ == '__main__':
    rospy.init_node('circle_laser_test')
    node = None
    try:
        node = CircleLaserTest(); rospy.spin()
    finally:
        if node is not None: node.close()
